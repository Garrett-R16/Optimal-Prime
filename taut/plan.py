"""Topology first, geometry second.

The order of operations is the whole point of this module, and it is the opposite of what
came before.

1. **Cut the free space into triangles** over the obstacle corners (:mod:`taut.mesh`). Every
   doorway between two triangles is a portal with a countable capacity.
2. **Choose a portal sequence for every connection at once**, negotiating over those
   capacities (:mod:`taut.topo`). This decides which side of every pad each net passes -- its
   homotopy class -- explicitly, where a search can reason about it.
3. **Seat the nets across each shared portal** and hand each one its own slice of the doorway.
4. **Embed**, with the class fixed: funnel through the narrowed gates, then tangent lines and
   arcs around the corners it wraps (:mod:`taut.funnel`).

5. **Check it, and repair what does not hold.** The embedding is a construction, not a
   proof: a doorway may be oriented from an approximate direction, or an arc may round a
   corner further than that corner's own normal cone allows. So every finished path is
   measured against the real obstacles, and anything too close is re-solved by the exact
   tangent solver against solid copper. That solver chooses its own homotopy class, which is
   the thing this module exists to avoid -- but a connection that falls back is one connection
   choosing badly, where an unchecked violation is a board that cannot be made.

Earlier versions did (4) alone and hoped (2) would emerge from it. It does not: two nets that
both want the same side of the same pad are in topological conflict, and sliding geometry
around never resolves that. One of them has to go the other way, which is a decision only
step 2 can make.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .board import Board, Net, Pad
from .funnel import Curve, Gate, Line, funnel, orient, taut_through
from .rubberband import Arc as RbArc, Crossing, Segment as RbSegment
from .rubberband import Wire as RbWire, rubberband, to_geometry
from .mesh import Mesh, build_mesh
from .obstacles import Obstacle, pad_obstacle
from .route import (ArcTrack, RouteResult, Track, _board_boundary, _convex_hull,
                    _copper_shape_obstacles, _path_obstacles, _solve_lazily,
                    MIN_PIECE_NM, MIN_SAGITTA_NM)
from .tangent import NoPathFound, PathArc, PathLine, TautPath
from .tangent import violated_obstacles
from .topo import Route, route_topology
from .units import CLEARANCE_MARGIN, GUARDBAND_NM

__all__ = ["plan_board"]

#: A funnel path no longer than this multiple of its straight-line span is kept without
#: consulting the exact solver. Above it, both are computed and the shorter wins -- the check
#: is cheap next to a wasted millimetre of copper and the space it denies everything after it.
_KEEP_WITHOUT_ASKING = 1.25

#: How many times to re-read the crossing order off the geometry and embed again.
_SEATING_PASSES = 4


# --------------------------------------------------------------------------- outline

def board_polygon(board: Board) -> list[tuple[float, float]]:
    """The board outline as an ordered ring, chained from its Edge.Cuts pieces."""
    segments = [((float(e.x1), float(e.y1)), (float(e.x2), float(e.y2)))
                for e in board.edges]
    if not segments:
        x0, y0, x1, y1 = board.bbox_nm
        return [(float(x0), float(y0)), (float(x1), float(y0)),
                (float(x1), float(y1)), (float(x0), float(y1))]

    remaining = list(segments)
    ring = list(remaining.pop(0))
    while remaining:
        tail = ring[-1]
        for index, (a, b) in enumerate(remaining):
            if math.dist(a, tail) < 1_000:
                ring.append(b)
                remaining.pop(index)
                break
            if math.dist(b, tail) < 1_000:
                ring.append(a)
                remaining.pop(index)
                break
        else:
            break
    if math.dist(ring[0], ring[-1]) < 1_000:
        ring.pop()
    # A ring that failed to close is worse than useless for containment; fall back.
    if len(ring) < 3:
        x0, y0, x1, y1 = board.bbox_nm
        return [(float(x0), float(y0)), (float(x1), float(y0)),
                (float(x1), float(y1)), (float(x0), float(y1))]
    return ring


def _inside(polygon, x: float, y: float) -> bool:
    inside = False
    count = len(polygon)
    for i in range(count):
        ax, ay = polygon[i]
        bx, by = polygon[(i + 1) % count]
        if (ay > y) != (by > y):
            t = (y - ay) / (by - ay) if by != ay else 0.0
            if x < ax + t * (bx - ax):
                inside = not inside
    return inside


# --------------------------------------------------------------------------- planning

@dataclass
class _Link:
    key: int
    net: Net
    pad_a: Pad
    pad_b: Pad
    span: float
    width: float
    clearance: float
    halo: float
    layer: str | None = None
    elements: list = field(default_factory=list)
    reason: str = ""


def _mst_edges(pads):
    if len(pads) < 2:
        return []
    unvisited = set(range(1, len(pads)))
    best = {i: (math.dist((pads[0].x, pads[0].y), (pads[i].x, pads[i].y)), 0)
            for i in unvisited}
    out = []
    while unvisited:
        nearest = min(unvisited, key=lambda i: best[i][0])
        out.append((best[nearest][1], nearest))
        unvisited.discard(nearest)
        for i in unvisited:
            d = math.dist((pads[nearest].x, pads[nearest].y), (pads[i].x, pads[i].y))
            if d < best[i][0]:
                best[i] = (d, nearest)
    return out


#: Facets per quarter turn when a round shape is given a boundary.
#:
#: A triangulation can only put vertices where it is given points, so a disc described by its
#: centre alone has no vertices anywhere on its copper: the triangles around it span from one
#: pad centre to the next, are mostly solid, and the gaps between them measure nothing real.
#: On a dense board that fragmented the free space into seventy pieces, stranding two thirds
#: of the connections on islands they could not leave. Facets put vertices on the boundary
#: instead, where the actual gaps are.
_FACETS_PER_QUARTER = 2


def _faceted(obstacle: Obstacle) -> Obstacle:
    """Re-describe a round shape by its boundary, so a triangulation can see its edges.

    The polygon *circumscribes* the true shape -- vertices at ``r / cos(pi/n)`` rather than
    ``r`` -- so the facets lie outside the copper everywhere. Over-covering costs a sliver of
    routing room; under-covering would let a track clip the pad it was avoiding.
    """
    if obstacle.r <= 0.0 or len(obstacle.vertices) > 2:
        return obstacle

    steps = max(4, _FACETS_PER_QUARTER * 4)
    reach = obstacle.r / math.cos(math.pi / steps)

    if len(obstacle.vertices) == 1:
        cx, cy = obstacle.vertices[0]
        ring = [(cx + reach * math.cos(2 * math.pi * i / steps),
                 cy + reach * math.sin(2 * math.pi * i / steps))
                for i in range(steps)]
    else:
        # A capsule: half a ring about each end, oriented across its axis.
        (ax, ay), (bx, by) = obstacle.vertices
        axis = math.atan2(by - ay, bx - ax)
        half = steps // 2
        ring = [(bx + reach * math.cos(axis - math.pi / 2 + math.pi * i / half),
                 by + reach * math.sin(axis - math.pi / 2 + math.pi * i / half))
                for i in range(half + 1)]
        ring += [(ax + reach * math.cos(axis + math.pi / 2 + math.pi * i / half),
                  ay + reach * math.sin(axis + math.pi / 2 + math.pi * i / half))
                 for i in range(half + 1)]

    return Obstacle(vertices=tuple(ring), r=0.0, net=obstacle.net, label=obstacle.label)


def _cores(board: Board, layer: str) -> list[Obstacle]:
    """Obstacle *cores* -- actual copper, with no routing clearance baked in.

    Clearance belongs to the embedding, where it narrows each doorway. Inflating here would
    delete every triangle that touches a pad. Round shapes are faceted first, so the copper
    they occupy is described by a boundary the triangulation can place vertices on.
    """
    out = [_faceted(pad_obstacle(pad, 0.0, 0.0)) for pad in board.pads
           if pad.on_layer(layer)]
    out.extend(_faceted(shape) for shape in _copper_shape_obstacles(board, layer, 0.0))
    return out


def _gates_for(mesh: Mesh, route: Route, slot_of, clearance: float, width: float,
               halo: float) -> list[Gate] | None:
    """Turn a portal sequence into the doorways this route may actually use.

    Every offset is measured from the **copper**, not from the mesh vertex. A round pad is a
    single vertex at its centre carrying a radius, so a gate placed ``halo`` from the vertex
    sits inside the pad by exactly that radius -- which is what put two thirds of the gate
    endpoints on this board in solid copper, and left every wrap arc short by the pad's own
    radius.
    """
    gates: list[Gate] = []

    for index, key in enumerate(route.portals):
        portal = mesh.portals[key]
        pa = (float(mesh.points[key[0]][0]), float(mesh.points[key[0]][1]))
        pb = (float(mesh.points[key[1]][0]), float(mesh.points[key[1]][1]))
        ra = float(mesh.radius[key[0]])
        rb = float(mesh.radius[key[1]])

        slot, total = slot_of(key, route)
        span = portal.length
        usable = span - ra - rb - 2.0 * halo - (total - 1) * (width + clearance)
        if usable < 0.0:
            return None
        share = usable / total
        stride = (width + clearance) + share

        from_a = ra + halo + slot * stride
        from_b = rb + halo + (total - 1 - slot) * stride
        if from_a + from_b > span:
            return None

        ux, uy = (pb[0] - pa[0]) / span, (pb[1] - pa[1]) / span
        end_a = (pa[0] + ux * from_a, pa[1] + uy * from_a)
        end_b = (pa[0] + ux * (span - from_b), pa[1] + uy * (span - from_b))

        # Both ends of the crossing come from triangle centroids, including the first and the
        # last. Substituting the pad centres there -- which sit inside their own pads -- put
        # the direction far enough out to flip left for right on 4 of 87 gates.
        heading = mesh.centroid(route.triangles[index])
        ahead = mesh.centroid(route.triangles[index + 1])
        left, right = orient(heading, ahead, end_a, end_b)

        a_is_left = left is end_a
        gates.append(Gate(
            left=left, right=right,
            left_vertex=pa if a_is_left else pb,
            right_vertex=pb if a_is_left else pa,
            left_radius=max(from_a if a_is_left else from_b, 1.0),
            right_radius=max(from_b if a_is_left else from_a, 1.0),
        ))

    return gates


def _crossing_order(mesh: Mesh, routes) -> dict[tuple[int, int], list[int]]:
    """Put the routes sharing each doorway into an order across it.

    Only the order is kept. Where a wire actually sits is never recorded, because it is not a
    property of the wire: the distance it stands off a corner is the accumulated width of
    whatever lies between it and *that* corner, and that differs at every corner it passes.
    Seating wires at fixed positions instead -- an equal share of each doorway, decided once --
    is what made a bundle spread correctly in one gap and waste half of the next.

    The order itself comes from where each route's local chord crosses the doorway, which is a
    good enough guess to start from and, being an order rather than a position, survives the
    embedding changing every coordinate underneath it.
    """
    seats: dict[tuple[int, int], list[tuple[float, int]]] = {}

    for route in routes:
        if not route.found:
            continue
        points = [route.start]
        for key in route.portals:
            pa, pb = mesh.points[key[0]], mesh.points[key[1]]
            points.append(((pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0))
        points.append(route.goal)

        for index, key in enumerate(route.portals):
            pa, pb = mesh.points[key[0]], mesh.points[key[1]]
            dx, dy = float(pb[0] - pa[0]), float(pb[1] - pa[1])
            before, after = points[index], points[index + 2]
            ex, ey = after[0] - before[0], after[1] - before[1]
            denominator = dx * ey - dy * ex
            if abs(denominator) < 1e-9:
                mid = ((before[0] + after[0]) / 2.0, (before[1] + after[1]) / 2.0)
                span = dx * dx + dy * dy
                where = (((mid[0] - pa[0]) * dx + (mid[1] - pa[1]) * dy) / span
                         if span > 1e-9 else 0.5)
            else:
                where = ((before[0] - pa[0]) * ey - (before[1] - pa[1]) * ex) / denominator
            seats.setdefault(key, []).append((min(1.0, max(0.0, where)), route.key))

    return {key: [route_key for _, route_key in sorted(value)]
            for key, value in seats.items()}


def _crossings_for(mesh: Mesh, route, order, wires: dict[int, RbWire]) -> list[Crossing]:
    """The doorways on one route, each carrying the full crossing order and this wire's rank."""
    out: list[Crossing] = []
    for key in route.portals:
        holders = order.get(key) or [route.key]
        if route.key not in holders:
            holders = list(holders) + [route.key]
        pa, pb = mesh.points[key[0]], mesh.points[key[1]]
        out.append(Crossing(
            ax=float(pa[0]), ay=float(pa[1]), bx=float(pb[0]), by=float(pb[1]),
            order=tuple(wires[k] for k in holders), mine=holders.index(route.key),
            ra=float(mesh.radius[key[0]]), rb=float(mesh.radius[key[1]]),
        ))
    return out


def _flatten(start, goal, elements, per_arc: int = 12) -> list[tuple[float, float]]:
    """The embedded path as a polyline, for asking where it crosses a doorway."""
    points = [start]
    for element in elements:
        if isinstance(element, Line):
            points.append((element.x2, element.y2))
        else:
            sweep = element.sweep
            for step in range(1, per_arc + 1):
                angle = element.start_angle + sweep * step / per_arc
                points.append((element.cx + element.r * math.cos(angle),
                               element.cy + element.r * math.sin(angle)))
    points.append(goal)
    return points


def _order_from_geometry(mesh: Mesh, routes, elements) -> dict[tuple[int, int], list[int]]:
    """Re-read the crossing order off the geometry that was just embedded.

    The order is the one thing the embedding depends on and the one thing it cannot be told
    reliably in advance: guessing it from a straight chord gets a wire ranked outermost at one
    doorway and third at the next, and an offset stack computed from that puts the wire on the
    far side of a gap it was supposed to hug. Reading it back off real geometry and embedding
    again settles it, usually within two passes.
    """
    seats: dict[tuple[int, int], list[tuple[float, int]]] = {}

    for route in routes:
        if not route.found or route.key not in elements:
            continue
        polyline = _flatten(route.start, route.goal, elements[route.key])
        cursor = 0
        for key in route.portals:
            pa, pb = mesh.points[key[0]], mesh.points[key[1]]
            ax, ay = float(pa[0]), float(pa[1])
            dx, dy = float(pb[0]) - ax, float(pb[1]) - ay
            span = dx * dx + dy * dy
            where = None
            for step in range(cursor, len(polyline) - 1):
                (px, py), (qx, qy) = polyline[step], polyline[step + 1]
                ex, ey = qx - px, qy - py
                denominator = dx * ey - dy * ex
                if abs(denominator) < 1e-12:
                    continue
                t = ((px - ax) * ey - (py - ay) * ex) / denominator
                u = ((px - ax) * dy - (py - ay) * dx) / denominator
                if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
                    where, cursor = t, step
                    break
            if where is None:
                # The path never reaches this doorway -- fall back to the nearest approach, so
                # a wire that has drifted still gets a rank rather than dropping out of the
                # bundle entirely.
                best = min(range(len(polyline)),
                           key=lambda step: (polyline[step][0] - ax - dx / 2) ** 2
                           + (polyline[step][1] - ay - dy / 2) ** 2)
                px, py = polyline[best]
                where = (((px - ax) * dx + (py - ay) * dy) / span) if span > 1e-12 else 0.5
            seats.setdefault(key, []).append((min(1.0, max(0.0, where)), route.key))

    return {key: [route_key for _, route_key in sorted(value)]
            for key, value in seats.items()}


def _rubberband_elements(start, goal, crossings) -> list:
    """Pull one wire taut and hand back pieces the checker and emitter already understand."""
    out = []
    for piece in to_geometry(start, goal, rubberband(start, goal, crossings)):
        if isinstance(piece, RbSegment):
            out.append(Line(piece.x1, piece.y1, piece.x2, piece.y2))
        else:
            out.append(Curve(piece.cx, piece.cy, piece.r,
                             piece.start_angle, piece.end_angle, piece.ccw))
    return out


def _clears_boundary(path: TautPath, boundary, gap: float) -> bool:
    """Whether a path keeps ``gap`` from every piece of the board outline.

    ``violated_obstacles`` only knows about obstacles, and the outline is not one of them --
    it is carried separately because it is exact geometry rather than an inflated shape. A
    path checked without it can run off the edge of the board and be pronounced legal.
    """
    from .geometry import Arc as GeoArc
    from . import geometry as geo

    for element in path.elements:
        if isinstance(element, PathLine):
            for shape in boundary:
                if isinstance(shape, GeoArc):
                    if geo.segment_arc(element.x1, element.y1,
                                       element.x2, element.y2, shape) < gap:
                        return False
                else:
                    ax, ay, bx, by = shape
                    if geo.segment_segment(element.x1, element.y1, element.x2, element.y2,
                                           ax, ay, bx, by) < gap:
                        return False
        else:
            own = element.as_geo()
            for shape in boundary:
                if isinstance(shape, GeoArc):
                    if geo.arc_arc(own, shape) < gap:
                        return False
                else:
                    ax, ay, bx, by = shape
                    if geo.segment_arc(ax, ay, bx, by, own) < gap:
                        return False
    return True


def _as_path(elements) -> TautPath:
    """Funnel output in the form the geometric checker and emitter already understand."""
    out = []
    for element in elements:
        if isinstance(element, Line):
            out.append(PathLine(element.x1, element.y1, element.x2, element.y2))
        else:
            out.append(PathArc(element.cx, element.cy, element.r,
                               element.start_angle, element.end_angle, element.ccw))
    return TautPath(out)


def _elements_to_tracks(elements, net: int, layer: str, width_nm: int) -> list:
    out = []
    for element in elements:
        if isinstance(element, (Line, PathLine)):
            if element.length < MIN_PIECE_NM:
                continue
            out.append(Track(net=net, layer=layer,
                             x1=int(round(element.x1)), y1=int(round(element.y1)),
                             x2=int(round(element.x2)), y2=int(round(element.y2)),
                             width_nm=width_nm))
        else:
            if element.length < MIN_PIECE_NM:
                continue
            at = element.at if hasattr(element, "at") else element.point_at
            sx, sy = at(0.0)
            mx, my = at(0.5)
            ex, ey = at(1.0)
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


def plan_board(board: Board, layers: list[str] | None = None,
               rounds: int = 12, verbose: bool = False) -> RouteResult:
    """Route a board topology-first."""
    usable = tuple(layers) if layers else board.copper_layers
    result = RouteResult()
    result.stats = {"board": board.name, "layers": list(usable), "router": "topological"}

    polygon = board_polygon(board)

    links: list[_Link] = []
    for net in board.routable:
        pads = list(net.pads)
        netclass = board.netclass_for(net.name)
        width = float(netclass.track_width_nm)
        clearance = netclass.clearance_nm * (1.0 + CLEARANCE_MARGIN)
        for a, b in _mst_edges(pads):
            links.append(_Link(
                key=len(links), net=net, pad_a=pads[a], pad_b=pads[b],
                span=math.dist((pads[a].x, pads[a].y), (pads[b].x, pads[b].y)),
                width=width, clearance=clearance,
                halo=clearance + width / 2.0 + GUARDBAND_NM))

    # One mesh per layer, built from the widest rule any net on it will need.
    clearance = max((link.clearance for link in links), default=200_000.0)
    width = max((link.width for link in links), default=250_000.0)

    meshes: dict[str, Mesh] = {}
    for layer in usable:
        cores = _cores(board, layer)
        mesh = build_mesh(cores, polygon, clearance, width)
        for index in range(len(mesh.triangles)):
            if not mesh.free[index]:
                continue
            cx, cy = mesh.centroid(index)
            if not _inside(polygon, cx, cy):
                mesh.free[index] = False
        meshes[layer] = mesh
        if verbose:
            print(f"  {layer}: {int(mesh.free.sum())} free triangles, "
                  f"{len(mesh.portals)} portals")

    # Assign each link to a layer by asking each layer what it would cost, rather than by
    # counting how many links are already on it. Load-balancing blind puts short links on a
    # crowded face and long ones on an empty one purely by arrival order, and nothing
    # afterwards revisits the choice.
    def portal_chain_length(mesh: Mesh, route) -> float:
        if not route.found:
            return math.inf
        points = [route.start]
        for key in route.portals:
            pa, pb = mesh.points[key[0]], mesh.points[key[1]]
            points.append(((pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0))
        points.append(route.goal)
        return sum(math.dist(a, b) for a, b in zip(points, points[1:]))

    by_layer: dict[str, list[_Link]] = {layer: [] for layer in usable}
    for link in links:
        options = [layer for layer in usable
                   if link.pad_a.on_layer(layer) and link.pad_b.on_layer(layer)]
        if not options:
            result.failed.append((link.net.code, link.net.name,
                                  "pads share no usable layer (a via would be needed)"))
            continue
        if len(options) == 1:
            link.layer = options[0]
            by_layer[link.layer].append(link)
            continue

        # Probe each candidate on an empty board -- what the route costs before anyone else
        # is on it. Contention is settled afterwards, by the negotiation.
        best_layer, best_cost = options[0], math.inf
        request = [(link.key, link.net.code,
                    (float(link.pad_a.x), float(link.pad_a.y)),
                    (float(link.pad_b.x), float(link.pad_b.y)))]
        for layer in options:
            probe, _ = route_topology(meshes[layer], request, rounds=1)
            cost = portal_chain_length(meshes[layer], probe[0])
            if cost < best_cost:
                best_layer, best_cost = layer, cost
        link.layer = best_layer
        by_layer[best_layer].append(link)

    topo_stats = {"rounds": 0, "overfull": 0, "unroutable": 0, "converged": True}

    for layer, group in by_layer.items():
        if not group:
            continue
        mesh = meshes[layer]
        requests = [(link.key, link.net.code,
                     (float(link.pad_a.x), float(link.pad_a.y)),
                     (float(link.pad_b.x), float(link.pad_b.y))) for link in group]
        routes, report = route_topology(mesh, requests, rounds=rounds, verbose=verbose)
        topo_stats["rounds"] = max(topo_stats["rounds"], report.rounds)
        topo_stats["overfull"] += len(report.overfull)
        topo_stats["unroutable"] += report.unroutable
        topo_stats["converged"] = topo_stats["converged"] and report.converged

        order = _crossing_order(mesh, routes)
        wires = {link.key: RbWire(key=link.key, net=link.net.code,
                                  half_width=link.width / 2.0,
                                  clearance=link.clearance + GUARDBAND_NM)
                 for link in group}

        # Embed, re-read the order off the result, embed again. The first order is a guess
        # from straight chords; two or three passes is enough for it to stop changing.
        index = {link.key: link for link in group}
        elements: dict[int, list] = {}
        for _pass in range(_SEATING_PASSES):
            elements = {route.key: _rubberband_elements(
                            route.start, route.goal,
                            _crossings_for(mesh, route, order, wires))
                        for route in routes if route.found}
            settled_order = _order_from_geometry(mesh, routes, elements)
            if settled_order == order:
                break
            order = settled_order

        for route in routes:
            link = index[route.key]
            if not route.found:
                link.reason = "no route through the free space"
                continue
            link.elements = elements.get(route.key, [])

    # ---- check the geometry, and repair what does not hold --------------------------
    checked = {"taut": 0, "fell_back": 0, "dropped": 0}
    boundary = _board_boundary(board)

    static_cache: dict[tuple[str, int, int], list[Obstacle]] = {}

    def statics(layer: str, clr: float, half: float) -> list[Obstacle]:
        key = (layer, int(clr), int(half))
        cached = static_cache.get(key)
        if cached is None:
            cached = [pad_obstacle(pad, clr, half) for pad in board.pads
                      if pad.on_layer(layer)]
            cached.extend(_copper_shape_obstacles(board, layer, clr + half))
            static_cache[key] = cached
        return cached

    settled: dict[str, list[tuple[int, TautPath, float]]] = {layer: [] for layer in usable}

    for link in sorted(links, key=lambda l: -l.span):
        if link.layer is None:
            continue
        half = link.width / 2.0 + GUARDBAND_NM
        blockers = [o for o in statics(link.layer, link.clearance, half)
                    if o.net != link.net.code]
        for net_code, path, halo in settled[link.layer]:
            if net_code != link.net.code:
                blockers.extend(_path_obstacles(path, link.halo, net_code))

        edge_gap = board.edge_clearance_nm + link.width / 2.0 + GUARDBAND_NM
        path = _as_path(link.elements) if link.elements else None
        legal = (path is not None and not violated_obstacles(path, blockers)
                 and _clears_boundary(path, boundary, edge_gap))

        # Legal is not the same as good. The funnel can return a path that clears everything
        # while wandering at nearly twice its own span, and keeping it because it is legal
        # both wastes copper and takes space the connections after it need -- a *better*
        # channel made the board worse that way, by producing a merely-adequate path where
        # the previous one had been rejected outright. So a wandering path is measured
        # against what the exact solver would do, and the shorter one wins.
        if legal and path.length <= link.span * _KEEP_WITHOUT_ASKING:
            checked["taut"] += 1
            settled[link.layer].append((link.net.code, path, link.halo))
            continue

        # Either the construction did not hold, or it held but wandered. Ask the exact
        # solver, which chooses its own class -- worse topology, but geometry that is
        # certainly legal and, when the funnel wandered, usually much shorter.
        # The fallback may also change layer. Pinning it to whichever face topology chose
        # strands a connection on a crowded one when the other is empty.
        alternative = None
        alt_layer = link.layer
        for candidate in [layer for layer in usable
                          if link.pad_a.on_layer(layer) and link.pad_b.on_layer(layer)]:
            here = [o for o in statics(candidate, link.clearance, half)
                    if o.net != link.net.code]
            for net_code, other, _halo in settled[candidate]:
                if net_code != link.net.code:
                    here.extend(_path_obstacles(other, link.halo, net_code))
            try:
                found = _solve_lazily(
                    (float(link.pad_a.x), float(link.pad_a.y)),
                    (float(link.pad_b.x), float(link.pad_b.y)), here,
                    boundary=boundary, boundary_gap=edge_gap)
            except NoPathFound:
                continue
            if alternative is None or found.length < alternative.length:
                alternative, alt_layer = found, candidate

        if alternative is None and not legal:
            link.elements = []
            link.reason = "funnel geometry rejected and no fallback path on any layer"
            checked["dropped"] += 1
            continue

        if legal and (alternative is None or path.length <= alternative.length):
            checked["taut"] += 1
            settled[link.layer].append((link.net.code, path, link.halo))
            continue

        checked["fell_back"] += 1
        link.layer = alt_layer
        link.elements = list(alternative.elements)
        settled[link.layer].append((link.net.code, alternative, link.halo))

    for link in links:
        if not link.elements:
            if link.reason and not any(f[0] == link.net.code and f[2] == link.reason
                                       for f in result.failed):
                result.failed.append((link.net.code, link.net.name, link.reason))
            continue
        result.tracks.extend(_elements_to_tracks(link.elements, link.net.code,
                                                 link.layer, int(link.width)))
        result.routed.append((link.net.code, link.net.name))

    result.stats.update({
        "connections": len(links),
        "routed": len(result.routed),
        "failed": len(result.failed),
        "tracks": len(result.tracks),
        "arcs": result.arc_count,
        "length_mm": round(result.total_length_nm / 1e6, 2),
        **{f"topo_{k}": v for k, v in topo_stats.items()},
        **{f"embed_{k}": v for k, v in checked.items()},
    })
    return result
