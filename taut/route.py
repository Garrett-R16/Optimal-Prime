"""Taut-string routing over a placed board.

For each connection the router asks what shape a rubber band would take, stretched between
two pads and pulled taut around everything in the way, and answers it exactly. There is no
grid, no preferred direction, and no 45-degree rule.

**Nets are not routed one at a time.** They were, once, and that has a failure no amount of
re-ordering or ripping up can fix. Consider a gap wide enough for two tracks. The first net
through it is asked, alone, where a rubber band would sit -- and a rubber band sits in the
middle. The second arrives and finds the gap full. Re-routing the first puts it back exactly
where it was, because the taut path is deterministic; swapping their order only swaps who is
stranded.

A board's copper is one arrangement, not a sequence of decisions. So every connection is
first routed as if it were alone -- which gives it the shape it *wants* -- and the whole set
is then relaxed together (:mod:`taut.relax`): all bands re-solved against the previous
positions of all the others, all moved at once, with the clearance between them annealed from
soft to solid so they can pass through each other while sorting out who goes which way.

Whatever is still overlapping once the clearance is fully hard is settled the old way, in
order, against solid copper. That settling also chooses which layer each connection ends on,
since picking a layer before the contention is known strands nets on a crowded one.

Correctness never rests on the relaxation converging: the finished arrangement is checked on
real geometry, and anything too close is routed again.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .board import Board, Net, Pad
from .obstacles import Obstacle, arc_obstacle, pad_obstacle, track_obstacle
from .relax import relax
from .tangent import (NoPathFound, PathArc, PathLine, TautPath, solve,
                      violated_obstacles)
from .units import CLEARANCE_MARGIN, GUARDBAND_NM

__all__ = ["Track", "ArcTrack", "RouteResult", "route_board"]


@dataclass(frozen=True, slots=True)
class Track:
    net: int
    layer: str
    x1: int
    y1: int
    x2: int
    y2: int
    width_nm: int

    @property
    def length_nm(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)


@dataclass(frozen=True, slots=True)
class Via:
    """A through via: the same hole on every copper layer, so a net can change sides."""

    net: int
    x: int
    y: int
    diameter_nm: int
    drill_nm: int


@dataclass(frozen=True, slots=True)
class ArcTrack:
    net: int
    layer: str
    x1: int
    y1: int
    xm: int
    ym: int
    x2: int
    y2: int
    width_nm: int
    length_nm: float = 0.0


#: What one through via adds to a signal's length: the barrel is the board's thickness of
#: conductor in series with the tracks. Counted so that "shorter" can never be bought by
#: quietly moving distance into the third dimension.
VIA_LENGTH_NM = 1_600_000


@dataclass(frozen=True, slots=True)
class Pour:
    """A net served by a copper zone instead of tracks: the plane a real board would use."""

    net: int
    name: str
    layer: str
    #: the zone's boundary, usually the board outline
    polygon: tuple


@dataclass
class RouteResult:
    tracks: list[Track | ArcTrack] = field(default_factory=list)
    vias: list[Via] = field(default_factory=list)
    pours: list["Pour"] = field(default_factory=list)
    routed: list[tuple[int, str]] = field(default_factory=list)
    failed: list[tuple[int, str, str]] = field(default_factory=list)
    stats: dict = field(default_factory=dict)

    @property
    def total_length_nm(self) -> float:
        return sum(t.length_nm for t in self.tracks) + VIA_LENGTH_NM * len(self.vias)

    @property
    def arc_count(self) -> int:
        return sum(1 for t in self.tracks if isinstance(t, ArcTrack))


#: An arc flatter than this is emitted as a straight segment instead.
#:
#: KiCad stores an arc as three points and rebuilds the circle from them. When the arc is
#: nearly flat those points are nearly collinear and the reconstruction is violently
#: ill-conditioned: a nanometre of rounding can change the radius by a large factor and flip
#: the sweep, turning a 12 um sliver into copper that loops most of the way round a circle.
MIN_SAGITTA_NM = 5_000

#: Copper shorter than this is dropped outright.
MIN_PIECE_NM = 1_000

#: Widest gap, in tracks, still treated as a shared resource.
MAX_GAP_TRACKS = 8


# --------------------------------------------------------------------------- helpers

def _board_boundary(board: Board):
    """The outline as exact primitives for the solver to keep its distance from."""
    from .geometry import Arc as GeoArc

    shapes = []
    for edge in board.edges:
        if edge.kind == "arc":
            arc = GeoArc.from_three_points(edge.x1, edge.y1, edge.xm, edge.ym,
                                           edge.x2, edge.y2)
            shapes.append(arc if not arc.degenerate
                          else (float(edge.x1), float(edge.y1),
                                float(edge.x2), float(edge.y2)))
        else:
            shapes.append((float(edge.x1), float(edge.y1),
                           float(edge.x2), float(edge.y2)))
    return shapes


def _convex_hull(points):
    """Andrew's monotone chain. Obstacles must be convex for the tangent construction."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return tuple(pts)

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2:
                (ax, ay), (bx, by) = out[-2], out[-1]
                if (bx - ax) * (p[1] - ay) - (by - ay) * (p[0] - ax) <= 0:
                    out.pop()
                else:
                    break
            out.append(p)
        return out

    return tuple(half(pts)[:-1] + half(list(reversed(pts)))[:-1])


def _copper_shape_obstacles(board: Board, layer: str, halo: float) -> list[Obstacle]:
    """Graphics drawn on a copper layer. They belong to no net, so nothing may share them."""
    out: list[Obstacle] = []
    for shape in board.copper_shapes:
        if shape.layer != layer:
            continue
        radius = halo + shape.width_nm / 2.0
        vertices = tuple((float(x), float(y)) for x, y in shape.vertices)
        if len(vertices) == 1:
            out.append(Obstacle(vertices=vertices, r=radius, net=0, label=shape.label))
        elif len(vertices) == 2:
            out.append(track_obstacle(*vertices[0], *vertices[1], radius, 0, shape.label))
        else:
            hull = _convex_hull(vertices)
            if len(hull) >= 3:
                out.append(Obstacle(vertices=hull, r=radius, net=0, label=shape.label))
            else:
                for i in range(len(vertices) - 1):
                    out.append(track_obstacle(*vertices[i], *vertices[i + 1],
                                              radius, 0, shape.label))
    return out


def _solve_lazily(start, goal, obstacles: list[Obstacle], boundary=None,
                  boundary_gap: float = 0.0, max_rounds: int = 8):
    """Solve against a nearby subset, then add back whatever the path actually hits.

    A taut path is shaped only by obstacles close to it, but the tangent graph is quadratic
    in the wrap-circle count, so handing it every obstacle on the board is ruinous. Starting
    local and repairing gives the same answer, because the loop exits only when nothing at
    all is violated.
    """
    sx, sy = start
    gx, gy = goal
    dx, dy = gx - sx, gy - sy
    span_sq = dx * dx + dy * dy
    corridor = max(math.sqrt(span_sq) * 0.12, 3_000_000.0)

    def near(obstacle: Obstacle) -> bool:
        ox, oy = obstacle.centre
        limit = corridor + obstacle.reach
        if span_sq <= 0.0:
            return math.hypot(ox - sx, oy - sy) <= limit
        t = max(0.0, min(1.0, ((ox - sx) * dx + (oy - sy) * dy) / span_sq))
        return math.hypot(ox - (sx + t * dx), oy - (sy + t * dy)) <= limit

    active = [o for o in obstacles if near(o)]
    active_ids = {id(o) for o in active}

    for _ in range(max_rounds):
        path = solve(start, goal, active, boundary=boundary, boundary_gap=boundary_gap)
        fresh = [obstacles[i] for i in violated_obstacles(path, obstacles)
                 if id(obstacles[i]) not in active_ids]
        if not fresh:
            return path
        active.extend(fresh)
        active_ids.update(id(o) for o in fresh)
        if len(active) > 120:
            # The tangent graph is quadratic in the wrap-circle count; past this density a
            # single call was measured in the tens of minutes. Declaring no-path is honest
            # here: the caller treats it as this layer having no room and falls back.
            raise NoPathFound("taut context too dense to price exactly")

    if len(obstacles) > 120:
        raise NoPathFound("taut context too dense to price exactly")
    return solve(start, goal, obstacles, boundary=boundary, boundary_gap=boundary_gap)


def _mst_edges(pads: list[Pad]) -> list[tuple[int, int]]:
    """Minimum spanning tree over a net's pads, so each net is connected once."""
    if len(pads) < 2:
        return []
    unvisited = set(range(1, len(pads)))
    best = {i: (math.dist((pads[0].x, pads[0].y), (pads[i].x, pads[i].y)), 0)
            for i in unvisited}
    edges: list[tuple[int, int]] = []
    while unvisited:
        nearest = min(unvisited, key=lambda i: best[i][0])
        edges.append((best[nearest][1], nearest))
        unvisited.discard(nearest)
        for i in unvisited:
            d = math.dist((pads[nearest].x, pads[nearest].y), (pads[i].x, pads[i].y))
            if d < best[i][0]:
                best[i] = (d, nearest)
    return edges


def _path_to_tracks(path: TautPath, net: int, layer: str, width_nm: int) -> list:
    out: list = []
    for element in path.elements:
        if isinstance(element, PathLine):
            if element.length < MIN_PIECE_NM:
                continue
            out.append(Track(net=net, layer=layer,
                             x1=int(round(element.x1)), y1=int(round(element.y1)),
                             x2=int(round(element.x2)), y2=int(round(element.y2)),
                             width_nm=width_nm))
            continue

        if element.length < MIN_PIECE_NM:
            continue

        sx, sy = element.start
        mx, my = element.mid
        ex, ey = element.end
        sagitta = element.r * (1.0 - math.cos(abs(element.sweep) / 2.0))
        if sagitta < MIN_SAGITTA_NM:
            if math.hypot(ex - sx, ey - sy) < MIN_PIECE_NM:
                continue
            out.append(Track(net=net, layer=layer,
                             x1=int(round(sx)), y1=int(round(sy)),
                             x2=int(round(ex)), y2=int(round(ey)),
                             width_nm=width_nm))
            continue

        out.append(ArcTrack(net=net, layer=layer,
                            x1=int(round(sx)), y1=int(round(sy)),
                            xm=int(round(mx)), ym=int(round(my)),
                            x2=int(round(ex)), y2=int(round(ey)),
                            width_nm=width_nm, length_nm=element.length))
    return out


def _path_obstacles(path: TautPath, radius: float, net: int) -> list[Obstacle]:
    """A finished path as keep-outs: one capsule per straight run, a few per arc."""
    out: list[Obstacle] = []
    for element in path.elements:
        if isinstance(element, PathLine):
            out.append(track_obstacle(element.x1, element.y1, element.x2, element.y2,
                                      radius, net))
        else:
            out.extend(arc_obstacle(element.as_geo(), radius, net))
    return out


@dataclass
class _Connection:
    index: int
    net: Net
    pad_a: Pad
    pad_b: Pad
    span: float
    width: float
    clearance: float
    half: float
    layer: str | None = None
    path: TautPath | None = None
    bundled: bool = False
    #: Set before relaxation; relax() reads these rather than reaching into the board.
    net_code: int = 0
    halo: float = 0.0
    layer_options: tuple[str, ...] = ()


# --------------------------------------------------------------------------- the router

def route_board(board: Board, layers: list[str] | None = None,
                order: str = "longest-first", bundle: bool = True,
                relax_steps: int = 5, relax_seconds: float = 240.0,
                verbose: bool = False) -> RouteResult:
    """Route every net on ``board`` with taut strings.

    ``layers`` restricts which copper layers may be used; a connection is placed on whichever
    permitted layer yields the shortest taut path. It never inserts a via, so a connection
    between surface pads on opposite faces has no solution here.

    ``bundle`` enables the joint relaxation described above. Setting it False settles the
    connections one at a time against solid copper, which is what this did originally and is
    kept so the two can be compared.
    """
    usable = tuple(layers) if layers else board.copper_layers
    unknown = [layer for layer in usable if layer not in board.copper_layers]
    if unknown:
        raise ValueError(f"board has no layer(s) {unknown}")

    result = RouteResult()
    result.stats = {"board": board.name, "layers": list(usable), "order": order,
                    "bundle": bundle}

    boundary = _board_boundary(board)
    static_cache: dict[tuple[str, int, int], list[Obstacle]] = {}

    def statics(layer: str, clearance: float, half: float) -> list[Obstacle]:
        """Pads and copper graphics on a layer -- everything that never moves."""
        key = (layer, int(clearance), int(half))
        cached = static_cache.get(key)
        if cached is None:
            cached = [pad_obstacle(pad, clearance, half) for pad in board.pads
                      if pad.on_layer(layer)]
            cached.extend(_copper_shape_obstacles(board, layer, clearance + half))
            static_cache[key] = cached
        return cached

    connections: list[_Connection] = []
    for net in board.routable:
        pads = list(net.pads)
        netclass = board.netclass_for(net.name)
        width = netclass.track_width_nm
        clearance = netclass.clearance_nm * (1.0 + CLEARANCE_MARGIN)
        for a, b in _mst_edges(pads):
            connections.append(_Connection(
                index=len(connections), net=net, pad_a=pads[a], pad_b=pads[b],
                span=math.dist((pads[a].x, pads[a].y), (pads[b].x, pads[b].y)),
                width=width, clearance=clearance,
                half=width / 2.0 + GUARDBAND_NM))

    connections.sort(key=lambda c: c.span, reverse=(order == "longest-first"))

    def attempt(conn: _Connection, layer: str, blockers: list[Obstacle]) -> TautPath:
        edge_halo = board.edge_clearance_nm + conn.width / 2.0 + GUARDBAND_NM
        return _solve_lazily((float(conn.pad_a.x), float(conn.pad_a.y)),
                             (float(conn.pad_b.x), float(conn.pad_b.y)), blockers,
                             boundary=boundary, boundary_gap=edge_halo)

    # ---- stage 1: what does each connection *want*? ----------------------------------
    #
    # Every connection is routed against the static obstacles alone -- pads, copper
    # graphics, the board edge -- and *not* against the other nets. That is the point: a
    # connection that would be blocked in a sequential pass still gets to say which gap it
    # wanted, and so still gets a slot in it. Assigning slots only among the nets that
    # happened to succeed is what left the stranded net stranded, because nothing ever knew
    # to make room for it.
    for conn in connections:
        options = [layer for layer in usable
                   if conn.pad_a.on_layer(layer) and conn.pad_b.on_layer(layer)]
        if not options:
            result.failed.append((conn.net.code, conn.net.name,
                                  f"pads {conn.pad_a.footprint}.{conn.pad_a.number} and "
                                  f"{conn.pad_b.footprint}.{conn.pad_b.number} share no "
                                  "usable layer (a via would be needed)"))
            continue

        best = None
        reasons: list[str] = []
        for layer in options:
            blockers = [o for o in statics(layer, conn.clearance, conn.half)
                        if o.net != conn.net.code]
            try:
                path = attempt(conn, layer, blockers)
            except NoPathFound as exc:
                reasons.append(f"{layer}: {exc}")
                continue
            if best is None or path.length < best[0]:
                best = (path.length, layer, path)

        if best is None:
            result.failed.append((conn.net.code, conn.net.name,
                                  "; ".join(reasons) or "no path"))
            continue
        _length, conn.layer, conn.path = best

    desired_length = sum(c.path.length for c in connections if c.path is not None)

    # ---- stage 2: relax the whole set together ---------------------------------------
    stats = {"relax_steps": 0, "relax_moved": 0, "relax_stuck": 0,
             "relax_converged": False, "fallbacks": 0, "dropped": 0, "ripped": 0,
             "length_by_step": [], "overlaps_by_step": []}

    if bundle:
        movable = [c for c in connections if c.path is not None]
        for conn in movable:
            conn.net_code = conn.net.code
            conn.halo = conn.clearance + conn.width + GUARDBAND_NM
            conn.layer_options = tuple(
                layer for layer in usable
                if conn.pad_a.on_layer(layer) and conn.pad_b.on_layer(layer))

        def solve_one(conn, layer, blockers):
            statics_here = [o for o in statics(layer, conn.clearance, conn.half)
                            if o.net != conn.net.code]
            return attempt(conn, layer, statics_here + blockers)

        # Relaxation is the expensive part and it improves monotonically, so it is given a
        # wall-clock budget rather than a fixed amount of work. Running out means fewer
        # annealing steps, not a worse-than-sequential answer: the settling pass afterwards
        # makes whatever it produced legal.
        import time as _time
        deadline = _time.monotonic() + relax_seconds
        report = relax(movable, solve_one, _path_obstacles, steps=relax_steps,
                       verbose=verbose, budget=lambda: _time.monotonic() > deadline)
        stats.update({
            "relax_steps": report.steps,
            "relax_moved": report.moved,
            "relax_stuck": report.stuck,
            "relax_converged": report.converged,
            "length_by_step": report.length_by_step,
            "overlaps_by_step": report.overlaps_by_step,
        })

    _repair_overlaps(connections, usable, statics, attempt, stats, verbose)

    sequential_length = desired_length

    for conn in connections:
        if conn.path is None or conn.layer is None:
            if not any(f[0] == conn.net.code and f[1] == conn.net.name
                       for f in result.failed):
                result.failed.append((conn.net.code, conn.net.name,
                                      "no legal path once the bundle was seated"))
            continue
        result.tracks.extend(_path_to_tracks(conn.path, conn.net.code, conn.layer,
                                             int(conn.width)))
        result.routed.append((conn.net.code, conn.net.name))

    result.stats.update({
        "connections": len(connections),
        "routed": len(result.routed),
        "failed": len(result.failed),
        "tracks": len(result.tracks),
        "arcs": result.arc_count,
        "length_mm": round(result.total_length_nm / 1e6, 2),
        "sequential_length_mm": round(sequential_length / 1e6, 2),
        **{f"bundle_{k}": v for k, v in stats.items()},
    })
    return result


def _repair_overlaps(connections, usable, statics, attempt, stats, verbose,
                     rip_budget: int = 40) -> None:
    """Settle the arrangement into something legal, and verify that it is.

    Relaxation moves every band at once and gets the arrangement close. This finishes it:
    connections are placed longest-first -- a long one has the least freedom -- and each is
    checked against the copper already down.

    The part that matters is what happens when one will not fit. Placing in order and never
    revisiting means the last connection in gets whatever is left, and if that is nothing it
    is simply dropped -- even though a path plainly exists and some other net is merely
    sitting on it. So a connection that cannot be placed **rips up the copper that is blocking
    it**, takes the space, and hands the displaced connections back to the queue to find
    somewhere else. Rip-ups are budgeted, so a pair that would trade the same space forever
    cannot.

    The check is on real geometry, not on the model that produced it, so the result is legal
    whatever the relaxation did or did not manage.
    """
    settled: dict[str, list[_Connection]] = {layer: [] for layer in usable}
    ripped = 0

    def blockers_on(layer: str, conn: _Connection, skip=()) -> list[Obstacle]:
        halo = conn.clearance + conn.width + GUARDBAND_NM
        out: list[Obstacle] = []
        for other in settled[layer]:
            if (other.net.code == conn.net.code or other.path is None
                    or other in skip):
                continue
            out.extend(_path_obstacles(other.path, halo, other.net.code))
        return out

    def options_for(conn: _Connection) -> list[str]:
        return [layer for layer in usable
                if conn.pad_a.on_layer(layer) and conn.pad_b.on_layer(layer)]

    def place(conn: _Connection, skip=()) -> tuple[float, str, TautPath] | None:
        best = None
        for layer in options_for(conn):
            blockers = [o for o in statics(layer, conn.clearance, conn.half)
                        if o.net != conn.net.code]
            blockers.extend(blockers_on(layer, conn, skip))
            try:
                path = attempt(conn, layer, blockers)
            except NoPathFound:
                continue
            if best is None or path.length < best[0]:
                best = (path.length, layer, path)
        return best

    def culprits(conn: _Connection) -> list[_Connection]:
        """Which settled connections are sitting where this one needs to be."""
        guilty: list[_Connection] = []
        for layer in options_for(conn):
            for other in settled[layer]:
                if other.net.code == conn.net.code or other.path is None:
                    continue
                halo = conn.clearance + conn.width + GUARDBAND_NM
                obstacles = _path_obstacles(other.path, halo, other.net.code)
                if conn.path is not None and violated_obstacles(conn.path, obstacles):
                    guilty.append(other)
        return guilty

    queue = sorted((c for c in connections if c.path is not None), key=lambda c: -c.span)
    index = 0
    while index < len(queue):
        conn = queue[index]
        index += 1

        # Already where it wants to be and clear of everything? Leave it.
        if conn.layer in options_for(conn) and conn.path is not None \
                and not violated_obstacles(conn.path, blockers_on(conn.layer, conn)):
            settled[conn.layer].append(conn)
            continue

        stats["fallbacks"] += 1
        found = place(conn)
        if found is not None:
            _length, conn.layer, conn.path = found
            settled[conn.layer].append(conn)
            continue

        # Nothing fits. Take the space from whoever is on it, and let them look elsewhere.
        blocking = culprits(conn)
        if blocking and ripped < rip_budget:
            retry = place(conn, skip=blocking)
            if retry is not None:
                for victim in blocking:
                    for layer in usable:
                        if victim in settled[layer]:
                            settled[layer].remove(victim)
                    queue.append(victim)
                    ripped += 1
                    stats["ripped"] += 1
                _length, conn.layer, conn.path = retry
                settled[conn.layer].append(conn)
                if verbose:
                    print(f"    {conn.net.name} took space from "
                          f"{[v.net.name for v in blocking]}")
                continue

        conn.path = None
        conn.layer = None
        stats["dropped"] += 1
