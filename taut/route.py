"""Taut-string routing over a placed board.

For each connection the router asks one question -- *what shape would a rubber band take,
stretched between these two pads and pulled taut around everything in the way* -- and answers
it exactly. There is no grid, no preferred direction, and no 45-degree rule; those are
conventions for human legibility, and nothing here needs them.

Obstacles are discs (see :mod:`taut.obstacles`), so the answer is provably straight tangent
lines joined by arcs, which is exactly what a ``.kicad_pcb`` can hold.

Nets are routed sequentially and each finished track becomes an obstacle for the next, so net
ordering still matters. That is a known limitation, not an oversight: negotiated congestion
and topology search are what fix it, and neither is in this MVP.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .board import Board, Net, Pad
from .obstacles import Obstacle, arc_obstacle, pad_obstacle, track_obstacle
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


@dataclass
class RouteResult:
    tracks: list[Track | ArcTrack] = field(default_factory=list)
    routed: list[tuple[int, str]] = field(default_factory=list)
    failed: list[tuple[int, str, str]] = field(default_factory=list)
    stats: dict = field(default_factory=dict)

    @property
    def total_length_nm(self) -> float:
        return sum(t.length_nm for t in self.tracks)

    @property
    def arc_count(self) -> int:
        return sum(1 for t in self.tracks if isinstance(t, ArcTrack))


def _copper_shape_obstacles(board: Board, layer: str, halo: float) -> list[Obstacle]:
    """Graphics drawn on a copper layer, as keep-outs.

    They belong to no net, so nothing may share space with them. A stroked line or polygon
    is inflated by half its stroke width on top of the clearance halo.
    """
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
            # A polyline that is not a closed convex outline is covered edge by edge, which
            # is correct for any shape and does not assume convexity.
            hull = _convex_hull(vertices)
            if len(hull) >= 3:
                out.append(Obstacle(vertices=hull, r=radius, net=0, label=shape.label))
            else:
                for i in range(len(vertices) - 1):
                    out.append(track_obstacle(*vertices[i], *vertices[i + 1],
                                              radius, 0, shape.label))
    return out


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

    lower = half(pts)
    upper = half(reversed(pts))
    return tuple(lower[:-1] + upper[:-1])


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


def _solve_lazily(start, goal, obstacles: list[Obstacle], boundary=None,
                  boundary_gap: float = 0.0, max_rounds: int = 8):
    """Solve against a nearby subset, then add back whatever the path actually hits.

    A taut path is shaped only by obstacles close to it, but the tangent graph is quadratic in
    the wrap-circle count, so handing it every obstacle on the board is ruinous. Starting
    local and repairing converges in a handful of rounds and gives the same answer, because
    the loop exits only when nothing at all is violated.
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


#: An arc flatter than this is emitted as a straight segment instead.
#:
#: KiCad stores an arc as three points and rebuilds the circle from them. When the arc is
#: nearly flat those three points are nearly collinear, and the reconstruction is violently
#: ill-conditioned: a nanometre of rounding can change the radius by a large factor and flip
#: the sweep direction, turning a 12 um sliver into copper that loops most of the way round a
#: circle and shorts whatever it crosses. Our own model never sees it, because our model has
#: the arc we intended.
#:
#: Five microns of sagitta is two orders of magnitude below any clearance rule on a board, so
#: replacing such an arc with its chord is geometrically invisible and numerically safe.
MIN_SAGITTA_NM = 5_000

#: Copper shorter than this is dropped outright.
MIN_PIECE_NM = 1_000


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


def _track_obstacles(tracks: list, radius: float, net: int) -> list[Obstacle]:
    """Finished copper as keep-outs.

    A straight track is a single capsule -- two wrap circles instead of the forty-odd discs a
    chain of them needed, and a tighter fit into the bargain.
    """
    from .geometry import Arc as GeoArc

    out: list[Obstacle] = []
    for track in tracks:
        if isinstance(track, Track):
            out.append(track_obstacle(track.x1, track.y1, track.x2, track.y2, radius, net))
        else:
            arc = GeoArc.from_three_points(track.x1, track.y1, track.xm, track.ym,
                                           track.x2, track.y2)
            if arc.degenerate:
                out.append(track_obstacle(track.x1, track.y1, track.x2, track.y2,
                                          radius, net))
            else:
                out.extend(arc_obstacle(arc, radius, net))
    return out


def route_board(board: Board, layers: list[str] | None = None,
                order: str = "longest-first", verbose: bool = False) -> RouteResult:
    """Route every net on ``board`` with taut strings.

    ``layers`` restricts which copper layers may be used; a connection is placed on whichever
    permitted layer yields the shortest taut path. With one layer this is single-layer
    routing; with two it is per-connection layer assignment, which is enough to route boards
    a single layer cannot -- but it never inserts a via, so a connection between surface pads
    on opposite faces has no solution here.
    """
    usable = tuple(layers) if layers else board.copper_layers
    unknown = [layer for layer in usable if layer not in board.copper_layers]
    if unknown:
        raise ValueError(f"board has no layer(s) {unknown}")

    result = RouteResult()
    result.stats = {"board": board.name, "layers": list(usable), "order": order}

    # Static obstacles are the same for every connection sharing a netclass, so they are
    # built once and cached. Rebuilding them per connection was most of the first version's
    # runtime.
    pad_cache: dict[tuple[str, int, int], list[Obstacle]] = {}
    graphic_cache: dict[tuple[str, int, int], list[Obstacle]] = {}
    boundary = _board_boundary(board)

    def pads_for(layer: str, clearance: int, half: float) -> list[Obstacle]:
        key = (layer, int(clearance), int(half))
        cached = pad_cache.get(key)
        if cached is None:
            cached = [pad_obstacle(pad, clearance, half) for pad in board.pads
                      if pad.on_layer(layer)]
            pad_cache[key] = cached
        return cached

    def graphics_for(layer: str, clearance: float, half: float) -> list[Obstacle]:
        key = (layer, int(clearance), int(half))
        cached = graphic_cache.get(key)
        if cached is None:
            cached = _copper_shape_obstacles(board, layer, clearance + half)
            graphic_cache[key] = cached
        return cached

    routed_copper: dict[str, list[Obstacle]] = {layer: [] for layer in usable}

    connections: list[tuple[Net, Pad, Pad, float]] = []
    for net in board.routable:
        pads = list(net.pads)
        for a, b in _mst_edges(pads):
            span = math.dist((pads[a].x, pads[a].y), (pads[b].x, pads[b].y))
            connections.append((net, pads[a], pads[b], span))

    connections.sort(key=lambda c: c[3], reverse=(order == "longest-first"))

    for index, (net, pad_a, pad_b, _span) in enumerate(connections, 1):
        netclass = board.netclass_for(net.name)
        width = netclass.track_width_nm
        clearance = netclass.clearance_nm * (1.0 + CLEARANCE_MARGIN)
        half = width / 2.0 + GUARDBAND_NM
        # Track-to-track: two centrelines this far apart leave a full clearance between their
        # *edges*, so the halo carries a whole track width, not half of one. Using half was
        # worth six clearance violations on the first run.
        halo = clearance + width + GUARDBAND_NM
        edge_halo = board.edge_clearance_nm + width / 2.0 + GUARDBAND_NM

        candidates = [layer for layer in usable
                      if pad_a.on_layer(layer) and pad_b.on_layer(layer)]
        if not candidates:
            result.failed.append((net.code, net.name,
                                  f"pads {pad_a.footprint}.{pad_a.number} and "
                                  f"{pad_b.footprint}.{pad_b.number} share no usable layer "
                                  "(a via would be needed)"))
            continue

        best: tuple[float, str, TautPath] | None = None
        reasons: list[str] = []
        for layer in candidates:
            blockers = [d for d in pads_for(layer, clearance, half) if d.net != net.code]
            blockers.extend(d for d in routed_copper[layer] if d.net != net.code)
            blockers.extend(graphics_for(layer, clearance, half))
            try:
                path = _solve_lazily((float(pad_a.x), float(pad_a.y)),
                                     (float(pad_b.x), float(pad_b.y)), blockers,
                                     boundary=boundary, boundary_gap=edge_halo)
            except NoPathFound as exc:
                reasons.append(f"{layer}: {exc}")
                continue
            if best is None or path.length < best[0]:
                best = (path.length, layer, path)

        if best is None:
            result.failed.append((net.code, net.name, "; ".join(reasons) or "no path"))
            if verbose:
                print(f"  [{index}/{len(connections)}] {net.name}: FAILED")
            continue

        _length, layer, path = best
        tracks = _path_to_tracks(path, net.code, layer, width)
        result.tracks.extend(tracks)
        result.routed.append((net.code, net.name))
        routed_copper[layer].extend(_track_obstacles(tracks, halo, net.code))
        if verbose:
            print(f"  [{index}/{len(connections)}] {net.name} on {layer}: "
                  f"{_length / 1e6:.1f}mm, {len(tracks)} pieces")

    result.stats.update({
        "connections": len(connections),
        "routed": len(result.routed),
        "failed": len(result.failed),
        "tracks": len(result.tracks),
        "arcs": result.arc_count,
        "length_mm": round(result.total_length_nm / 1e6, 2),
    })
    return result
