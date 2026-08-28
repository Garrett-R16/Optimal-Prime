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
from .rubberband import Wire as RbWire, _segments_cross, rubberband, to_geometry
from .rubberband import spacing_between
from .mesh import Mesh, build_mesh
from .obstacles import Obstacle, pad_obstacle
from .route import (ArcTrack, RouteResult, Track, Via, _board_boundary, _convex_hull,
                    _copper_shape_obstacles, _path_obstacles, _solve_lazily,
                    MIN_PIECE_NM, MIN_SAGITTA_NM)
from .tangent import (NoPathFound, PathArc, PathLine, TautPath,
                      segment_to_obstacle)
from .tangent import violated_obstacles
from .layered import Leg, route_stack
from .weave import Weave
from .topo import Route
from .vias import via_sites
from .units import CLEARANCE_MARGIN, GUARDBAND_NM

__all__ = ["plan_board"]

#: A funnel path no longer than this multiple of its straight-line span is kept without
#: consulting the exact solver. Above it, both are computed and the shorter wins -- the check
#: is cheap next to a wasted millimetre of copper and the space it denies everything after it.
_KEEP_WITHOUT_ASKING = 1.0

#: How many times to re-read the crossing order off the geometry and embed again.
_SEATING_PASSES = 8

#: How many settled tracks rip-up will move to make room for one that has nowhere to go.
_RIPUP_LIMIT = 4

#: How far the displacement may cascade -- a track moved out of the way may move one itself.
_RIPUP_DEPTH = 3


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
    #: the connection this piece belongs to. A via splits one connection into several, and
    #: none of them is worth anything on its own -- a via with copper on one side only is a
    #: hole in the board, so either every piece goes down or none of them does.
    parent: int = -1
    #: the leg of the stack route this piece came from, once topology has chosen one
    route: object | None = None
    elements: list = field(default_factory=list)
    reason: str = ""


@dataclass(frozen=True, slots=True)
class _ViaPoint:
    """One end of a leg that is a via rather than a pad.

    A through via reaches every copper layer, so a leg ending at one may sit on any of them --
    which is what lets the repair below move a leg to the other side without breaking the
    connection.
    """

    x: float
    y: float
    net: int
    #: the connection this via belongs to, so it can be left off the board if that
    #: connection does not go down whole
    owner: int
    diameter: int
    drill: int

    def on_layer(self, _layer: str) -> bool:
        return True


def _pad_like(pad):
    """Both kinds of leg end answer to the same three things, so nothing else has to care."""
    return pad


def _as_route(link: "_Link") -> Route:
    """One leg, in the shape the embedding has always taken."""
    return Route(key=link.key, net=link.net.code,
                 start=(float(link.pad_a.x), float(link.pad_a.y)),
                 goal=(float(link.pad_b.x), float(link.pad_b.y)),
                 triangles=list(link.route.triangles), portals=list(link.route.portals))


@dataclass(frozen=True, slots=True)
class _Placed:
    """A connection already on the board, and the room it takes up."""

    key: int
    net: int
    path: TautPath
    halo: float
    #: this connection's own half width and clearance, so the wire meeting it can work out
    #: the gap the two of them need between their centre lines
    half: float
    clearance: float


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
    """Put the routes sharing each doorway into an order across it -- from the topology.

    The order used to be read off the embedded geometry and re-read after every pass. That
    is circular: when two wires cross, the geometry is exactly the thing that is wrong, and
    an order read from it re-embeds the crossing it should be fixing. Rank is a topological
    fact, so it is derived from the topology: each route's *chain* -- terminal, doorway
    midpoints, terminal -- crosses every one of its doorways at the midpoint, and the order
    of wires across a doorway is the order of where their chains come from and go to,
    projected on the doorway. The geometry then follows the rank, never the reverse.
    """
    chains: dict[int, list[tuple[float, float]]] = {}
    for route in routes:
        if not route.found:
            continue
        points = [route.start]
        for key in route.portals:
            pa, pb = mesh.points[key[0]], mesh.points[key[1]]
            points.append(((float(pa[0]) + float(pb[0])) / 2.0,
                           (float(pa[1]) + float(pb[1])) / 2.0))
        points.append(route.goal)
        chains[route.key] = points

    seats: dict[tuple[int, int], list[tuple[float, int]]] = {}
    for route in routes:
        if not route.found:
            continue
        chain = chains[route.key]
        for index, key in enumerate(route.portals):
            pa, pb = mesh.points[key[0]], mesh.points[key[1]]
            ax, ay = float(pa[0]), float(pa[1])
            dx, dy = float(pb[0]) - ax, float(pb[1]) - ay
            span = dx * dx + dy * dy
            if span < 1e-12:
                seats.setdefault(key, []).append((0.5, route.key))
                continue
            before = chain[index]
            after = chain[index + 2]
            here = (((before[0] + after[0]) / 2.0 - ax) * dx
                    + ((before[1] + after[1]) / 2.0 - ay) * dy) / span
            seats.setdefault(key, []).append((here, route.key))

    return {key: [route_key for _, route_key in sorted(value)]
            for key, value in seats.items()}


def _side_by_side(routes) -> list[tuple[int, int, list[tuple[int, int]]]]:
    """For every pair of wires, the stretches of doorway they cross side by side.

    A stretch is a set of doorways consecutive in *both* routes. There is nothing between two
    consecutive doorways for a wire to go around, so within a stretch the two wires cannot
    swap sides without crossing -- and two wires on one layer never cross. Their order over a
    stretch is therefore a single fact, not one fact per doorway.

    Where they part and meet again the order is free to differ, because in between they may
    have passed opposite sides of the same pad; that is why this is per stretch rather than
    global.
    """
    live = {route.key: route for route in routes if route.found}
    seats = {key: {portal: index for index, portal in enumerate(route.portals)}
             for key, route in live.items()}

    out: list[tuple[int, int, list[tuple[int, int]]]] = []
    keys = sorted(live)
    for position, first in enumerate(keys):
        for second in keys[position + 1:]:
            here, there = seats[first], seats[second]
            run: list[tuple[int, int]] = []
            for portal in live[first].portals:
                if portal not in there:
                    continue
                if run and not (abs(here[portal] - here[run[-1]]) == 1
                                and abs(there[portal] - there[run[-1]]) == 1):
                    out.append((first, second, run))
                    run = []
                run.append(portal)
            if run:
                out.append((first, second, run))
    return out


def _consistent(routes, raw: dict[tuple[int, int], list[int]]) -> dict[tuple[int, int], list[int]]:
    """Force each pair of wires into one order for the whole stretch they run together.

    Read doorway by doorway off real geometry, the order comes back inconsistent: a wire ranks
    outermost at one doorway and third at the next few microns further on. An offset stack
    computed from that puts it on the far side of a gap it was meant to hug, and two wires
    whose order disagrees between consecutive doorways have to cross each other in the triangle
    between them.

    Each stretch takes the majority verdict of its doorways, and each doorway is then rebuilt
    by counting how many of the wires present are known to come before each one. Counting
    rather than sorting on the comparison directly, because separate stretches can disagree and
    leave no total order to sort by; a count still produces a sensible one.
    """
    verdicts: dict[tuple[int, int, tuple[int, int]], bool] = {}
    for first, second, run in _side_by_side(routes):
        ahead = 0
        for portal in run:
            order = raw.get(portal)
            if order and first in order and second in order:
                ahead += 1 if order.index(first) < order.index(second) else -1
        for portal in run:
            verdicts[(first, second, portal)] = ahead >= 0

    out: dict[tuple[int, int], list[int]] = {}
    for portal, order in raw.items():
        rank = {wire: order.index(wire) for wire in order}
        behind = {}
        for wire in order:
            count = 0
            for other in order:
                if other == wire:
                    continue
                pair = (wire, other) if wire < other else (other, wire)
                verdict = verdicts.get((pair[0], pair[1], portal))
                leader = (pair[0] if verdict else pair[1]) if verdict is not None else (
                    wire if rank[wire] < rank[other] else other)
                if leader != wire:
                    count += 1
            behind[wire] = count
        out[portal] = sorted(order, key=lambda wire: (behind[wire], rank[wire]))
    return out


def _push_apart(places: list[float], gaps: list[float], low: float, high: float) -> list[float]:
    """Slide wires along a doorway until each clears the next, moving them as little as possible.

    A forward pass takes each wire out far enough to clear the one before it, a backward pass
    brings the far end back inside the doorway, and a wire that was already clear is not moved
    at all. That is what makes this different from handing every wire a slot up front: a
    doorway with room to spare leaves its wires exactly where being taut put them, and only a
    crowded one pushes.
    """
    out = list(places)
    out[0] = max(out[0], low)
    for index in range(1, len(out)):
        out[index] = max(out[index], out[index - 1] + gaps[index - 1])

    out[-1] = min(out[-1], high)
    for index in range(len(out) - 2, -1, -1):
        out[index] = min(out[index], out[index + 1] - gaps[index])
    return out


def _bundle(mesh: Mesh, routes, elements, wires: dict[int, RbWire]) -> dict:
    """Where every wire sits in every doorway, once the bundle has settled against itself.

    Ranking wires across a doorway keeps them in order but says nothing about the gap between
    them, and the offset each one keeps is only ever expressed where it wraps a corner. Two
    wires running side by side through open channel wrap nothing, so nothing holds them apart
    -- which is how taut paths that were each in the right corridor still ended up touching.

    Reading where they actually cross, pushing them apart along the doorway, and pulling every
    wire taut again against those positions is the bands doing to each other what the obstacles
    already do to them.
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
            span = math.hypot(dx, dy)
            where, cursor = _meets(polyline, cursor, ax, ay, dx, dy)
            seats.setdefault(key, []).append((max(0.0, min(span, where * span)), route.key))

    room: dict[tuple[int, int, int], tuple[float, float]] = {}
    for key, found in seats.items():
        found.sort()
        span = mesh.portals[key].length
        holders = [wires[route_key] for _, route_key in found]
        gaps = [spacing_between(a, b) for a, b in zip(holders, holders[1:])]

        first, last = holders[0], holders[-1]
        low = float(mesh.radius[key[0]]) + first.half_width + first.clearance
        high = span - float(mesh.radius[key[1]]) - last.half_width - last.clearance

        settled = _push_apart([place for place, _ in found], gaps, low, high)
        for index, (_, route_key) in enumerate(found):
            behind = settled[index - 1] + gaps[index - 1] if index else low
            ahead = (settled[index + 1] - gaps[index] if index + 1 < len(settled) else high)
            room[(key[0], key[1], route_key)] = (max(low, behind), span - min(high, ahead))
    return room


def _meets(polyline, cursor: int, ax: float, ay: float,
           dx: float, dy: float) -> tuple[float, int]:
    """Where a path crosses a doorway, as a fraction along it, and where to resume looking."""
    for step in range(cursor, len(polyline) - 1):
        (px, py), (qx, qy) = polyline[step], polyline[step + 1]
        ex, ey = qx - px, qy - py
        denominator = dx * ey - dy * ex
        if abs(denominator) < 1e-12:
            continue
        along = ((px - ax) * ey - (py - ay) * ex) / denominator
        across = ((px - ax) * dy - (py - ay) * dx) / denominator
        if 0.0 <= along <= 1.0 and 0.0 <= across <= 1.0:
            return along, step

    span = dx * dx + dy * dy
    nearest = min(range(len(polyline)),
                  key=lambda step: (polyline[step][0] - ax - dx / 2) ** 2
                  + (polyline[step][1] - ay - dy / 2) ** 2)
    px, py = polyline[nearest]
    where = (((px - ax) * dx + (py - ay) * dy) / span) if span > 1e-12 else 0.5
    return min(1.0, max(0.0, where)), cursor


def _crossings_for(mesh: Mesh, route, order, wires: dict[int, RbWire],
                   room: dict | None = None) -> list[Crossing]:
    """The doorways on one route, each carrying the full crossing order and this wire's rank."""
    out: list[Crossing] = []
    for key in route.portals:
        holders = order.get(key) or [route.key]
        if route.key not in holders:
            holders = list(holders) + [route.key]
        pa, pb = mesh.points[key[0]], mesh.points[key[1]]
        given = (room or {}).get((key[0], key[1], route.key), (None, None))
        out.append(Crossing(
            ax=float(pa[0]), ay=float(pa[1]), bx=float(pb[0]), by=float(pb[1]),
            order=tuple(wires[k] for k in holders), mine=holders.index(route.key),
            ra=float(mesh.radius[key[0]]), rb=float(mesh.radius[key[1]]),
            room_a=given[0], room_b=given[1],
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


def _rubberband_elements(start, goal, crossings, extras=()) -> list:
    """Pull one wire taut and hand back pieces the checker and emitter already understand."""
    out = []
    for piece in to_geometry(start, goal, rubberband(start, goal, crossings, extras)):
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
        if isinstance(element, (PathLine, PathArc)):
            out.append(element)
        elif isinstance(element, Line):
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


#: How many route-embed-check-learn rounds before accepting what stands. Each round only
#: adds bans, so the loop is monotone; this is a backstop, not a working limit.
_SKETCH_ROUNDS = 8

#: How many times to re-pull the wires with newly named wrap points before looking at
#: crossings. Clips converge like the exact solver's lazy obstacle loop: fast.
_CLIP_ROUNDS = 4

_TAIL_FAMILIES = True

#: How many times the weave may promote a blocked wire to the front and start over.
_WEAVE_RESTARTS = 20

#: Rounds of take-out-and-reinsert refinement over the finished weave.
_WEAVE_DESCENT = 4

#: How many times placement may veto a settlement and send the whole pipeline round again.
_VETO_ROUNDS = 3


def _embed_group(mesh: Mesh, group: list["_Link"], extras: dict,
                 resolved: dict | None = None, woven_order: dict | None = None) -> None:
    """Pull one layer's wires taut together, seating them against each other by rank.

    A wire in ``resolved`` was moved by the cost arbiter and its geometry is settled: it is
    seated -- everyone else keeps clear of where it actually is -- but never re-pulled.

    A ``woven_order`` is authoritative: it came out of the weave planar by construction,
    and the chain-adjacency guess plus its consistency pass would only disturb it.
    """
    if not group:
        return
    resolved = resolved or {}
    routes = [_as_route(link) for link in group]
    if woven_order is not None:
        mine = {link.key for link in group}
        order = {key: [wire for wire in wires_here if wire in mine]
                 for key, wires_here in woven_order.items()}
        order = {key: wires_here for key, wires_here in order.items() if wires_here}
    else:
        order = _consistent(routes, _crossing_order(mesh, routes))
    wires = {link.key: RbWire(key=link.key, net=link.net.code,
                              half_width=link.width / 2.0,
                              clearance=link.clearance + GUARDBAND_NM)
             for link in group}

    # Rank comes from the topology and stays put; the passes are only for the *room* --
    # where each wire actually sits inside its doorways once the whole bundle has pressed
    # against itself, which is geometric and needs a fixpoint.
    index = {link.key: link for link in group}
    elements: dict[int, list] = {}
    room: dict = {}
    for _pass in range(_SEATING_PASSES):
        elements = {}
        for route in routes:
            if not route.found:
                continue
            if route.key in resolved:
                elements[route.key] = list(resolved[route.key])
                continue
            elements[route.key] = _rubberband_elements(
                route.start, route.goal,
                _crossings_for(mesh, route, order, wires, room),
                tuple(extras.get(route.key, ())))
        room = _bundle(mesh, routes, elements, wires)

    for route in routes:
        link = index[route.key]
        if not route.found:
            link.reason = "no route through the free space"
            continue
        link.elements = elements.get(route.key, [])


def _check_pieces(board: Board, meshes: dict, usable, pieces: list["_Link"],
                  placed_vias) -> tuple[list, list]:
    """Referee the pure sketch: every wire against every wire and all bare copper.

    Returns crossings as ``(key_a, key_b, x, y)`` and clips as ``(key, (x, y, required))``
    -- each clip already shaped as the extra wrap point that repairs it, centred on the
    copper boundary nearest the offence.
    """
    from .sketch import SketchWire, check_sketch

    crossings: list = []
    clips: list = []
    for layer in usable:
        group = [link for link in pieces if link.layer == layer and link.elements]
        if not group:
            continue
        wires = [SketchWire(key=link.key, net=link.net.code,
                            path=_as_path(link.elements),
                            half_width=link.width / 2.0, clearance=link.clearance)
                 for link in group]
        statics = [pad_obstacle(pad, 0.0, 0.0) for pad in board.pads
                   if pad.on_layer(layer)]
        statics.extend(_copper_shape_obstacles(board, layer, 0.0))
        statics.extend(Obstacle(vertices=((via.x, via.y),), r=via.diameter / 2.0,
                                net=via.net, label="via") for via in placed_vias)

        crossed, grazed, clipped = check_sketch(wires, statics, guard=GUARDBAND_NM)
        crossings.extend((c.first, c.second, c.points) for c in crossed)

        # A graze is a lens that has not quite happened: compatible classes, geometry not
        # finished. Same cure -- the roomier wire is told to treat the closest-approach
        # point of the other as copper, and the next pull spaces them.
        for graze in grazed:
            by_key = {link.key: link for link in group}
            one, two = by_key.get(graze.first), by_key.get(graze.second)
            if one is None or two is None:
                continue
            mover = one if one.span >= two.span else two
            other = two if mover is one else one
            required = (mover.width / 2.0 + other.width / 2.0
                        + max(mover.clearance, other.clearance) + GUARDBAND_NM)
            clips.append((mover.key, (graze.x, graze.y, required)))
        for clip in clipped:
            obstacle = statics[clip.obstacle]
            bx, by = _boundary_point(obstacle, clip.x, clip.y)
            wire = next(link for link in group if link.key == clip.key)
            required = wire.width / 2.0 + wire.clearance + GUARDBAND_NM
            clips.append((clip.key, (bx, by, required)))
    return crossings, clips


def _boundary_point(obstacle: Obstacle, x: float, y: float) -> tuple[float, float]:
    """The point on the copper's edge nearest an offending point of a path."""
    if len(obstacle.vertices) == 1:
        cx, cy = obstacle.vertices[0]
        span = math.hypot(x - cx, y - cy)
        if span < 1e-9:
            return cx + obstacle.r, cy
        return cx + (x - cx) / span * obstacle.r, cy + (y - cy) / span * obstacle.r

    ring = list(obstacle.vertices)
    closing = ring[1:] + ring[:1] if len(ring) > 2 else ring[1:]
    best = (math.inf, ring[0])
    for (ax, ay), (bx, by) in zip(ring, closing):
        dx, dy = bx - ax, by - ay
        span = dx * dx + dy * dy
        t = 0.0 if span < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / span))
        px, py = ax + t * dx, ay + t * dy
        d = math.hypot(x - px, y - py)
        if d < best[0]:
            best = (d, (px, py))
    px, py = best[1]
    if obstacle.r > 0.0:
        span = math.hypot(x - px, y - py)
        if span > 1e-9:
            px += (x - px) / span * obstacle.r
            py += (y - py) / span * obstacle.r
    return px, py


def _ban_for_crossing(meshes: dict, usable, pieces: list["_Link"],
                      first: int, second: int, x: float, y: float) -> set:
    """Every turn that lets this crossing exist, banned at once.

    Two mid-route wires through one triangle always share a doorway and rank keeps them
    apart, so a crossing always has a terminal stub on at least one side. Banning just the
    one turn the passing wire took moves it one triangle around the fan and the same
    crossing comes back next round a doorway over -- measured, eight rounds of it. What the
    passing wire must actually be denied is the *separation*: every way through the fan that
    parts the other wire's terminal from its exit. That is one stub-segment intersection
    test per pair of doorways per fan triangle, computed here in one go.
    """
    by_key = {link.key: link for link in pieces}
    a, b = by_key.get(first), by_key.get(second)
    if a is None or b is None:
        return set()

    def context(link):
        leg = link.route
        mesh = meshes[link.layer]
        polyline = _flatten((float(link.pad_a.x), float(link.pad_a.y)),
                            (float(link.pad_b.x), float(link.pad_b.y)), link.elements)
        s_cross, s_total = 0.0, 0.0
        best = math.inf
        lengths = []
        for (px, py), (qx, qy) in zip(polyline, polyline[1:]):
            step = math.hypot(qx - px, qy - py)
            lengths.append((s_total, (px, py), (qx, qy), step))
            s_total += step
        for s0, (px, py), (qx, qy), step in lengths:
            if step < 1e-9:
                continue
            t = max(0.0, min(1.0, ((x - px) * (qx - px) + (y - py) * (qy - py))
                             / (step * step)))
            d = math.hypot(x - px - t * (qx - px), y - py - t * (qy - py))
            if d < best:
                best = d
                s_cross = s0 + t * step
        marks = []
        for slot, key in enumerate(leg.portals):
            pa, pb = mesh.points[key[0]], mesh.points[key[1]]
            ax, ay = float(pa[0]), float(pa[1])
            dx, dy = float(pb[0]) - ax, float(pb[1]) - ay
            hit = None
            for s0, (px, py), (qx, qy), step in lengths:
                ex, ey = qx - px, qy - py
                denominator = dx * ey - dy * ex
                if abs(denominator) < 1e-12:
                    continue
                t = ((px - ax) * ey - (py - ay) * ex) / denominator
                u = ((px - ax) * dy - (py - ay) * dx) / denominator
                if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
                    hit = s0 + u * step
                    break
            if hit is not None:
                marks.append((hit, slot))
        marks.sort()
        before = [slot for s_here, slot in marks if s_here <= s_cross]
        after = [slot for s_here, slot in marks if s_here > s_cross]
        if before and after:
            return False, None
        # A stub, and which end: the crossing sits before the first doorway or after the last.
        return True, ("tail" if before else "head")

    a_stub, a_side = context(a)
    b_stub, b_side = context(b)

    def layer_index(link):
        return usable.index(link.layer)

    def fan_family(passer, owner, side) -> set:
        """Deny ``passer`` every fan turn that separates ``owner``'s terminal from its exit.

        ``side`` says which terminal: a wire has a stub at both ends, and guarding the head
        while the crossing is at the tail bans turns nobody is taking.
        """
        leg = owner.route
        mesh = meshes[owner.layer]
        if not leg.portals:
            return set()
        if side == "tail":
            centre = (float(owner.pad_b.x), float(owner.pad_b.y))
            exit_key = leg.portals[-1]
        else:
            centre = (float(owner.pad_a.x), float(owner.pad_a.y))
            exit_key = leg.portals[0]
        exit_mid = _portal_mid(mesh, exit_key)

        out: set = set()
        layer = layer_index(owner)
        ring = mesh.terminals(*centre)
        for tri in ring:
            doors = [portal.key() for _other, portal in mesh.adjacent(int(tri))]
            for i_slot in range(len(doors)):
                for j_slot in range(i_slot + 1, len(doors)):
                    p, q = doors[i_slot], doors[j_slot]
                    mp = _portal_mid(mesh, p)
                    mq = _portal_mid(mesh, q)
                    if _segments_cross(mp[0], mp[1], mq[0], mq[1],
                                       centre[0], centre[1], exit_mid[0], exit_mid[1]):
                        out.add((passer.net.code, layer, int(tri),
                                 frozenset((_portal_key(p), _portal_key(q)))))
        return out

    def stub_bans(link, side) -> set:
        leg = link.route
        layer = layer_index(link)
        if not leg.portals:
            # A direct line with no doorways at all: nothing to ban but the shortcut itself.
            return {(link.net.code, layer, -1, frozenset())}
        if side == "tail":
            return {(link.net.code, layer, int(leg.triangles[len(leg.portals) - 1]),
                     frozenset((_portal_key(leg.portals[-1]),)))}
        return {(link.net.code, layer, int(leg.triangles[0]),
                 frozenset((_portal_key(leg.portals[0]),)))}

    if not a_stub and b_stub:
        return fan_family(a, b, b_side if _TAIL_FAMILIES else "head")
    if not b_stub and a_stub:
        return fan_family(b, a, a_side if _TAIL_FAMILIES else "head")
    if a_stub and b_stub:
        mover = a if len(a.route.portals) >= len(b.route.portals) else b
        other = b if mover is a else a
        mover_side = a_side if mover is a else b_side
        other_side = b_side if mover is a else a_side
        return stub_bans(mover, mover_side) | fan_family(mover, other, other_side)
    # Both mid-route should be impossible; if the geometry disagrees, move the longer one
    # off the shorter one's whole fan at both ends.
    mover, other = (a, b) if a.span >= b.span else (b, a)
    return fan_family(mover, other, "head") | fan_family(mover, other, "tail")


def _settle_crossing(board: Board, meshes: dict, usable, pieces: list["_Link"],
                     resolved: dict, first: int, second: int, boundary,
                     frozen: frozenset = frozenset()):
    """Price both ways out of a crossing and commit the cheaper wire's move.

    The cost of moving a wire is what the exact solver charges for its shortest legal path
    with everything else -- copper and every other wire's current geometry -- standing.
    Whoever pays less, pays; ties go to the longer wire, which has the slack.
    """
    if ("off",) in frozen:
        # Arbitration stood down: placement could not house everyone under any settlement,
        # and a complete board outranks a shorter one.
        return None

    by_key = {link.key: link for link in pieces}
    a, b = by_key.get(first), by_key.get(second)
    if a is None or b is None:
        return None

    def blockers_for(link: "_Link") -> list:
        half = link.width / 2.0 + GUARDBAND_NM
        out = [pad_obstacle(pad, link.clearance, half) for pad in board.pads
               if pad.on_layer(link.layer) and pad.net != link.net.code]
        out.extend(_copper_shape_obstacles(board, link.layer, link.clearance + half))
        for other in pieces:
            if other.key == link.key or other.layer != link.layer:
                continue
            if other.net.code == link.net.code or not other.elements:
                continue
            gap = (max(link.clearance, other.clearance) + link.width / 2.0
                   + other.width / 2.0 + GUARDBAND_NM)
            out.extend(_path_obstacles(_as_path(other.elements), gap, other.net.code))
        return out

    offers = []
    for link in (a, b):
        edge_gap = board.edge_clearance_nm + link.width / 2.0 + GUARDBAND_NM
        try:
            found = _solve_lazily((float(link.pad_a.x), float(link.pad_a.y)),
                                  (float(link.pad_b.x), float(link.pad_b.y)),
                                  blockers_for(link),
                                  boundary=boundary, boundary_gap=edge_gap)
        except NoPathFound:
            continue
        current = _as_path(link.elements).length if link.elements else link.span
        pieces_out = []
        for element in found.elements:
            if isinstance(element, PathLine):
                pieces_out.append(Line(element.x1, element.y1, element.x2, element.y2))
            else:
                pieces_out.append(Curve(element.cx, element.cy, element.r,
                                        element.start_angle, element.end_angle,
                                        element.ccw))
        offers.append((found.length - current, -link.span, link.key, pieces_out))

    if not offers:
        return None
    offers.sort()
    _cost, _tie, mover_key, elements = offers[0]
    return mover_key, elements


def _lens_wraps(pieces: list["_Link"], first: int, second: int, points) -> list:
    """The extra wrap points that unpick a lens.

    The two wires cross an even number of times; between each pair of crossings one of them
    holds territory the other's taut path wants. The wire with the longer span has the more
    slack, so it gives way -- told to keep a full spacing off every crossing point and the
    midpoint between each pair, so the whole overlapped stretch is pushed aside at once
    rather than one timid nudge per round.
    """
    by_key = {link.key: link for link in pieces}
    a, b = by_key[first], by_key[second]
    mover, other = (a, b) if a.span >= b.span else (b, a)
    required = (mover.width / 2.0 + other.width / 2.0
                + max(mover.clearance, other.clearance) + GUARDBAND_NM)

    spots = list(points)
    for one, two in zip(points, points[1:]):
        spots.append(((one[0] + two[0]) / 2.0, (one[1] + two[1]) / 2.0))
    return [(mover.key, (x, y, required)) for x, y in spots]


def _portal_mid(mesh: Mesh, key) -> tuple[float, float]:
    pa, pb = mesh.points[key[0]], mesh.points[key[1]]
    return (float(pa[0]) + float(pb[0])) / 2.0, (float(pa[1]) + float(pb[1])) / 2.0


def _route_crossings(weave, piece):
    """The (portal, fraction) records a piece currently holds in the weave."""
    out = []
    for pkey in piece.route.portals:
        fraction = next((f for f, wire in weave.on_portal.get(pkey, ())
                         if wire == piece.key), 0.5)
        out.append((pkey, fraction))
    return out


def _portal_key(key) -> tuple[int, int]:
    return int(key[0]), int(key[1])


def plan_board(board: Board, layers: list[str] | None = None,
               rounds: int = 12, verbose: bool = False,
               _frozen_movers: frozenset = frozenset(),
               _veto_depth: int = 0) -> RouteResult:
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

    # Where a via could go, and then one search over the whole stack rather than one per
    # layer with the layer decided beforehand. See taut.layered for why that matters.
    via_radius = max((board.netclass_for(link.net.name).via_diameter_nm
                      for link in links), default=800_000) / 2.0
    sites = via_sites([meshes[layer] for layer in usable], via_radius, clearance,
                      inside=lambda x, y: _inside(polygon, x, y))
    if verbose:
        print(f"  {len(sites)} places a via could go")

    reachable: dict[tuple[float, float], list[list[int]]] = {}
    on_layer: dict[tuple[float, float], set[int]] = {}

    def terminals(point):
        found = reachable.get(point)
        if found is None:
            allowed = on_layer.get(point, set(range(len(usable))))
            found = reachable[point] = [
                meshes[layer].terminals(*point) if index in allowed else []
                for index, layer in enumerate(usable)]
        return found

    def would_cross(one: _Link, other: _Link) -> bool:
        return (one.net.code != other.net.code
                and _segments_cross(float(one.pad_a.x), float(one.pad_a.y),
                                    float(one.pad_b.x), float(one.pad_b.y),
                                    float(other.pad_a.x), float(other.pad_a.y),
                                    float(other.pad_b.x), float(other.pad_b.y)))

    # Which side each connection would rather be on, from which straight pad-to-pad lines
    # cross which. With the corridor right a taut wire wraps nothing unless something is in
    # its way, so that line predicts almost exactly which pairs will fight -- and it costs a
    # cross product to ask. The search is told this as a preference, not a rule: two wires
    # that must cross whatever is done about the layers can still buy a via instead.
    taken: dict[int, list[_Link]] = {index: [] for index in range(len(usable))}
    prefer: dict[int, int | None] = {}
    for link in sorted(links, key=lambda l: -l.span):
        options = [index for index, name in enumerate(usable)
                   if link.pad_a.on_layer(name) and link.pad_b.on_layer(name)]
        if not options:
            prefer[link.key] = None
            continue
        # Ties on crossings are broken by how much copper the straight line runs into on
        # each side. Falling back to the lower-numbered layer instead puts everything that
        # ties on the front, which is not a choice at all.
        def obstructed(index: int, _link=link) -> int:
            mesh = meshes[usable[index]]
            return sum(1 for shape in mesh.obstacles
                       if shape.net != _link.net.code
                       and segment_to_obstacle(shape,
                                               float(_link.pad_a.x), float(_link.pad_a.y),
                                               float(_link.pad_b.x), float(_link.pad_b.y))
                       <= shape.r)

        best = min(options, key=lambda index: (sum(1 for other in taken[index]
                                                   if would_cross(link, other)),
                                               obstructed(index), index))
        prefer[link.key] = best
        taken[best].append(link)

    requests = []
    for link in links:
        here = (float(link.pad_a.x), float(link.pad_a.y))
        there = (float(link.pad_b.x), float(link.pad_b.y))
        on_layer[here] = {i for i, name in enumerate(usable) if link.pad_a.on_layer(name)}
        on_layer[there] = {i for i, name in enumerate(usable) if link.pad_b.on_layer(name)}
        requests.append((link.key, link.net.code, here, there, prefer.get(link.key)))

    # ---- route, embed, check, learn, repeat ----------------------------------------
    #
    # The sketch is not hoped legal; it is *made* legal, by the two feedback loops the two
    # kinds of defect deserve. A wire clipping copper is an embedding problem: the copper had
    # no vertex in the wire's doorways, so it is named as an extra wrap point and the wire is
    # pulled taut again (lazy obstacle addition -- the exact solver's own trick). Two wires
    # crossing is a *class* problem, and it is always a terminal stub, because two mid-route
    # wires through one triangle must share a doorway and rank keeps ordered wires apart. A
    # crossing therefore becomes a ban -- the moving wire may no longer make the turn that
    # cut the other's terminal off -- and the whole stack is re-searched. Bans only
    # accumulate, so this converges; the reference systems (SURF's region fans, Leiserson &
    # Maley's terminal annotations) encode the same fact in the search space itself.
    bans: set = set()
    by_route = {link.key: link for link in links}
    pieces: list[_Link] = []
    placed_vias: list[_ViaPoint] = []
    unrouted: list[_Link] = []
    stack_report = None
    sketch_stats = {"clip_wraps": 0, "cross_settled": 0, "cross_bans": 0,
                    "woven": 0, "sketch_rounds": 0}

    ban_mode = ("off",) in _frozen_movers
    bans: set = set()
    chosen, stack_report = route_stack(
        [meshes[layer] for layer in usable], sites, requests,
        terminals=terminals, rounds=rounds, verbose=verbose)
    resolved: dict[int, list] = {}
    boundary_of = _board_boundary(board)

    def _rebuild_pieces() -> None:
        pieces.clear()
        placed_vias.clear()
        unrouted.clear()
        for route in chosen:
            parent = by_route[route.key]
            if not route.found:
                unrouted.append(parent)
                continue
            stops = [_pad_like(parent.pad_a)]
            for index in route.vias:
                site = sites[index]
                point = _ViaPoint(x=site.x, y=site.y, net=parent.net.code,
                                  owner=parent.key,
                                  diameter=board.netclass_for(parent.net.name).via_diameter_nm,
                                  drill=board.netclass_for(parent.net.name).via_drill_nm)
                placed_vias.append(point)
                stops.append(point)
            stops.append(_pad_like(parent.pad_b))
            for leg, (here, there) in zip(route.legs, zip(stops, stops[1:])):
                piece = _Link(key=len(pieces), net=parent.net, pad_a=here, pad_b=there,
                              span=math.dist((here.x, here.y), (there.x, there.y)),
                              width=parent.width, clearance=parent.clearance,
                              halo=parent.halo)
                piece.parent = parent.key
                piece.layer = usable[leg.layer]
                piece.route = leg
                pieces.append(piece)

    # ---- the weave: sides decided by construction --------------------------------------
    #
    # Wires are inserted one at a time, shortest first, into a search space where crossing
    # a committed wire is not illegal but *unrepresentable* (taut/weave.py, taut/cells.py --
    # the region-fan structure of the reference systems). What comes out is a crossing
    # order on every doorway that is planar by construction, so nothing downstream has a
    # side left to decide: settle and ban stay in the loop purely as safety nets, and
    # placement order stops mattering. If any wire cannot be woven -- the committed board
    # genuinely separates its ends on its assigned layer -- the weave stands down and the
    # tiers below carry the board exactly as before.
    woven_order: dict | None = None
    if not ban_mode:
        _rebuild_pieces()

        # A wire that cannot be woven late may weave fine early -- its blockers then thread
        # around *it*. Promotion restarts are the sequential router's oldest trick, and here
        # they cost almost nothing: the weave is pure graph work, no solver in the loop. A
        # wire that fails even when it goes first is genuinely separated, and the weave
        # stands down to the tiers below.
        promoted: list = []
        complete = False
        for _restart in range(_WEAVE_RESTARTS):
            weaves = {layer: Weave(meshes[layer]) for layer in usable}
            rest = sorted((piece for piece in pieces if piece not in promoted),
                          key=lambda item: item.span)
            failed = None
            for piece in promoted + rest:
                weave = weaves[piece.layer]
                head = (float(piece.pad_a.x), float(piece.pad_a.y))
                tail = (float(piece.pad_b.x), float(piece.pad_b.y))
                got = weave.insert(piece.key, head, tail,
                                   weave.mesh.terminals(*head),
                                   weave.mesh.terminals(*tail),
                                   need=piece.width / 2.0 + piece.clearance
                                   + GUARDBAND_NM)
                if not got.found:
                    failed = piece
                    break
                piece.route = Leg(layer=usable.index(piece.layer), start=head,
                                  goal=tail, triangles=list(got.triangles),
                                  portals=[key for key, _ in got.crossings])
            if failed is None:
                complete = True
                if verbose:
                    print(f"  weave: complete, {len(promoted)} promotion(s)")
                break
            if failed in promoted:
                if verbose:
                    print(f"  weave: net {failed.net.name} has no planar corridor even "
                          f"woven first; standing down")
                break
            if verbose:
                print(f"  weave: net {failed.net.name} blocked; promoting and restarting")
            promoted.insert(0, failed)
        else:
            if verbose:
                print(f"  weave: restarts exhausted after {len(promoted)} promotions; "
                      f"standing down")

        if complete:
            # ---- descent: re-offer every wire against the finished weave ---------------
            #
            # Insertion order baked arbitrary detours in: an early wire threaded around
            # others that have since gone elsewhere. Taking one wire out and re-inserting
            # it is sound *by construction* here -- the reinsertion cannot cross anyone --
            # so this is a descent that only ever shortens, and it converges because each
            # accepted move strictly reduces total chain length.
            latest = {piece.key: piece for piece in pieces}
            for _descent in range(_WEAVE_DESCENT):
                improved = False
                for piece in sorted(pieces, key=lambda item: -item.span):
                    weave = weaves[piece.layer]
                    head = (float(piece.pad_a.x), float(piece.pad_a.y))
                    tail = (float(piece.pad_b.x), float(piece.pad_b.y))
                    old_len = weave.chain_length(
                        head, tail, [(k, f) for k, f in _route_crossings(weave, piece)])
                    keep = weave.snapshot(piece.key)
                    weave.remove(piece.key)
                    got = weave.insert(piece.key, head, tail,
                                       weave.mesh.terminals(*head),
                                       weave.mesh.terminals(*tail),
                                       need=piece.width / 2.0 + piece.clearance
                                       + GUARDBAND_NM)
                    if (got.found and weave.chain_length(head, tail, got.crossings)
                            < old_len - 10_000.0):
                        piece.route = Leg(layer=usable.index(piece.layer), start=head,
                                          goal=tail, triangles=list(got.triangles),
                                          portals=[k for k, _ in got.crossings])
                        improved = True
                        sketch_stats["rewoven"] = sketch_stats.get("rewoven", 0) + 1
                    else:
                        if got.found:
                            weave.remove(piece.key)
                        weave.restore(piece.key, keep)
                if not improved:
                    break

            # Per layer: portal keys are vertex pairs in each layer's own mesh, and the
            # same pair of numbers names different doorways on different layers.
            woven_order = {layer: weave.order() for layer, weave in weaves.items()}
            sketch_stats["woven"] = len(pieces)
        else:
            _rebuild_pieces()

    for _sketch_round in range(_SKETCH_ROUNDS):
        sketch_stats["sketch_rounds"] += 1

        # A via splits a connection into pieces that are each one net, one layer, two
        # fixed ends; in ban mode a topology re-search rebuilds them between rounds.
        if not pieces:
            _rebuild_pieces()

        by_layer = {layer: [link for link in pieces if link.layer == layer]
                    for layer in usable}
        extras: dict[int, list[tuple[float, float, float]]] = {}

        crossings = []
        for _clip_round in range(_CLIP_ROUNDS):
            for layer, group in by_layer.items():
                _embed_group(meshes[layer], group, extras, resolved,
                             woven_order.get(layer) if woven_order else None)
            crossings, clips = _check_pieces(board, meshes, usable, pieces, placed_vias)

            # An even number of crossings between a pair is a *lens*: the classes are
            # compatible -- a non-crossing embedding exists -- but each wire was pulled
            # taut blind to the other and they bulge through each other in open space. The
            # cure is geometric, exactly like a clip: the roomier wire is told to treat a
            # point of the other as copper and is pulled taut again.
            lenses = [entry for entry in crossings if len(entry[2]) % 2 == 0]
            for first, second, points in lenses:
                clips.extend(_lens_wraps(pieces, first, second, points))


            if not clips:
                break
            for key, extra in clips:
                extras.setdefault(key, []).append(extra)
                sketch_stats["clip_wraps"] += 1

        if woven_order is not None:
            # In the woven world parity has done its work: whatever the clip loop could
            # not finish -- odd or even -- goes to the cost arbiter, whose answer is legal
            # against everything standing. The wrap dance above was measured oscillating
            # (2 crossings, then 14, then 2) on exactly one stubborn pair.
            pass
        else:
            crossings = [entry for entry in crossings if len(entry[2]) % 2 == 1]
        if not crossings:
            break

        if ban_mode:
            # Completeness mode. Settling by cost gives the shorter board, but its
            # reshaping of the sketch can leave a wire with no home in a way no blame can
            # trace -- measured surviving per-settlement vetoes, inverted movers, deep
            # rip-up and a via audition. The ban machinery reshapes the *topology* instead
            # -- the passing net loses every fan turn that separates the other wire's
            # terminal from its exit, and only the implicated connections are re-searched
            # -- which costs about ten millimetres more and is the configuration measured
            # to house everyone.
            grown = False
            movers = set()
            parent_of = {piece.key: piece.parent for piece in pieces}
            for first, second, points in crossings:
                x, y = points[0]
                family = _ban_for_crossing(meshes, usable, pieces, first, second, x, y)
                fresh = family - bans
                if fresh:
                    bans |= fresh
                    grown = True
                    sketch_stats["cross_bans"] += len(fresh)
                for key in (first, second):
                    if parent_of.get(key) is not None:
                        movers.add(parent_of[key])

            if verbose:
                print(f"  sketch round {_sketch_round + 1}: {len(crossings)} class "
                      f"crossings, {len(bans)} bans, rerouting {len(movers)}")
            if not grown:
                break
            chosen, stack_report = route_stack(
                [meshes[layer] for layer in usable], sites, requests,
                terminals=terminals, rounds=rounds, bans=frozenset(bans),
                warm=chosen, only=movers, verbose=verbose)
            _rebuild_pieces()
            continue

        # Primary mode: settle by cost.
        moved_this_round: set[int] = set()
        progressed = False
        for first, second, points in sorted(
                crossings, key=lambda entry: -len(entry[2])):
            if first in moved_this_round or second in moved_this_round:
                continue
            outcome = _settle_crossing(board, meshes, usable, pieces, resolved,
                                       first, second, boundary_of)
            if outcome is None:
                continue
            mover_key, elements = outcome
            resolved[mover_key] = elements
            moved_this_round.add(first)
            moved_this_round.add(second)
            progressed = True
            sketch_stats["cross_settled"] += 1

        if verbose:
            print(f"  sketch round {_sketch_round + 1}: {len(crossings)} class crossings, "
                  f"{len(resolved)} settled by cost")
        if not progressed:
            break

    for parent in unrouted:
        result.failed.append((parent.net.code, parent.net.name,
                              "no route through the free space on any layer"))

    topo_stats = {"rounds": stack_report.rounds, "overfull": len(stack_report.overfull),
                  "unroutable": stack_report.unroutable, "vias": stack_report.vias,
                  "converged": stack_report.converged}
    topo_stats.update(sketch_stats)
    links = pieces

    # ---- check the geometry, and repair what does not hold --------------------------
    checked = {"taut": 0, "fell_back": 0, "dropped": 0, "rescued": 0}
    boundary = _board_boundary(board)

    static_cache: dict[tuple[str, int, int], list[Obstacle]] = {}

    def statics(layer: str, clr: float, half: float) -> list[Obstacle]:
        key = (layer, int(clr), int(half))
        cached = static_cache.get(key)
        if cached is None:
            cached = [pad_obstacle(pad, clr, half) for pad in board.pads
                      if pad.on_layer(layer)]
            cached.extend(_copper_shape_obstacles(board, layer, clr + half))
            # A via is on every layer, so it is in everyone's way on every layer -- including
            # the layer whose leg does not touch it.
            cached.extend(Obstacle(vertices=((via.x, via.y),),
                                   r=via.diameter / 2.0 + clr + half,
                                   net=via.net, label="via")
                          for via in placed_vias)
            static_cache[key] = cached
        return cached

    settled: dict[str, list[_Placed]] = {layer: [] for layer in usable}
    by_key = {link.key: link for link in links}
    #: how each connection currently on the board got there; counted up at the end, so that
    #: taking a track up and putting it back somewhere else is not tallied twice
    how: dict[int, str] = {}

    def candidates(link: _Link) -> list[str]:
        # A leg that meets a via keeps the side topology put it on. A via is only worth
        # drilling because the two legs are on different layers; let the repair move one of
        # them for being a little shorter and they end up on the same side, and the via is a
        # hole with copper on one face -- which KiCad calls dangling, and rightly.
        if link.layer and (isinstance(link.pad_a, _ViaPoint)
                           or isinstance(link.pad_b, _ViaPoint)):
            return [link.layer]
        return [layer for layer in usable
                if link.pad_a.on_layer(layer) and link.pad_b.on_layer(layer)]

    def between(link: _Link, placed: _Placed) -> float:
        """Centre line to centre line, for a wire and a track already down.

        Both half widths, not just the wire's own. Charging only its own let two tracks sit
        their two half widths closer than the rules allow -- 0.12 mm on a 0.25 mm track, which
        is most of a clearance -- and the check passed geometry that DRC did not.
        """
        return (max(link.clearance, placed.clearance) + link.width / 2.0 + placed.half
                + GUARDBAND_NM)

    def in_the_way(layer: str, link: _Link, ignore: frozenset[int]) -> list[Obstacle]:
        half = link.width / 2.0 + GUARDBAND_NM
        out = [o for o in statics(layer, link.clearance, half) if o.net != link.net.code]
        for placed in settled[layer]:
            if placed.key in ignore or placed.net == link.net.code:
                continue
            out.extend(_path_obstacles(placed.path, between(link, placed), placed.net))
        return out

    def attempt(link: _Link, ignore: frozenset[int]) -> tuple[TautPath, str, str] | None:
        """The best legal geometry for one connection, given what is already down."""
        edge_gap = board.edge_clearance_nm + link.width / 2.0 + GUARDBAND_NM
        ends = ((float(link.pad_a.x), float(link.pad_a.y)),
                (float(link.pad_b.x), float(link.pad_b.y)))
        best: tuple[TautPath, str, str] | None = None

        if link.elements and link.layer:
            taut = _as_path(link.elements)
            if (not violated_obstacles(taut, in_the_way(link.layer, link, ignore))
                    and _clears_boundary(taut, boundary, edge_gap)):
                # In the woven world the taut geometry is *jointly* legal -- the whole
                # sketch was refereed as one object -- and a solver path taken here for
                # being a hair shorter re-creates exactly the conflicts the weave removed:
                # measured, the last wire to place paid for all 27 substitutions. Joint
                # legality outranks individual length, so a legal taut path is kept.
                if woven_order is not None:
                    return taut, link.layer, "taut"
                # Otherwise, legal is not the same as good: a taut path that wanders well
                # past its own span is worth measuring against the exact solver.
                if taut.length <= link.span * _KEEP_WITHOUT_ASKING:
                    return taut, link.layer, "taut"
                best = (taut, link.layer, "taut")

        for layer in candidates(link):
            try:
                found = _solve_lazily(*ends, in_the_way(layer, link, ignore),
                                      boundary=boundary, boundary_gap=edge_gap)
            except NoPathFound:
                continue
            if best is None or found.length < best[0].length:
                best = (found, layer, "fell_back")

        return best

    def lay(link: _Link, outcome: tuple[TautPath, str, str]) -> None:
        path, layer, kind = outcome
        link.layer = layer
        link.elements = list(path.elements)
        link.reason = ""
        settled[layer].append(_Placed(key=link.key, net=link.net.code, path=path,
                                      halo=link.halo, half=link.width / 2.0,
                                      clearance=link.clearance))
        how[link.key] = kind

    def take_up(link: _Link) -> None:
        for layer in usable:
            settled[layer] = [p for p in settled[layer] if p.key != link.key]
        link.elements = []
        how.pop(link.key, None)

    def restore(state) -> None:
        rows, records, places = state
        for name in usable:
            settled[name] = list(rows[name])
        how.clear()
        how.update(records)
        for key, (layer, elements) in places.items():
            by_key[key].layer = layer
            by_key[key].elements = list(elements)

    def snapshot():
        return ({name: list(rows) for name, rows in settled.items()},
                dict(how),
                {link.key: (link.layer, list(link.elements)) for link in links})

    def place(link: _Link, protected: frozenset[int], depth: int) -> bool:
        """Put one connection down, taking up what is in the way if it has to.

        Topology allocated a corridor to every connection before any geometry existed, so a
        connection with nowhere to go is not evidence that the board is full -- it is
        evidence that a track laid earlier took more room than its own corridor and sat down
        on someone else's. Ask where this one would go on an empty board, take up only the
        tracks actually sitting on that answer, and put them back somewhere else. If any of
        them cannot be put back, undo the whole exchange rather than trade one unrouted
        connection for another.
        """
        outcome = attempt(link, frozenset())
        if outcome is not None:
            lay(link, outcome)
            return True
        if depth >= _RIPUP_DEPTH:
            link.reason = "no legal geometry, and rip-up has gone as deep as it will go"
            return False

        everything = frozenset(placed.key for rows in settled.values() for placed in rows)
        alone = attempt(link, everything)
        if alone is None:
            link.reason = "no geometry between these pads even on an empty board"
            return False

        path, layer, _kind = alone
        culprits = [placed for placed in settled[layer]
                    if placed.net != link.net.code and placed.key not in protected
                    and violated_obstacles(path, _path_obstacles(placed.path,
                                                                 between(link, placed),
                                                                 placed.net))]
        blocked = [placed for placed in settled[layer]
                   if placed.key in protected and placed.net != link.net.code
                   and violated_obstacles(path, _path_obstacles(placed.path,
                                                                between(link, placed),
                                                                placed.net))]
        if blocked or not culprits or len(culprits) > _RIPUP_LIMIT:
            link.reason = ("blocked by a track this exchange is not allowed to move" if blocked
                           else f"blocked by {len(culprits)} settled tracks, more than rip-up "
                                f"will move" if culprits else "blocked by static copper")
            return False

        outcome = attempt(link, frozenset(placed.key for placed in culprits))
        if outcome is None:
            link.reason = "no legal geometry even with the tracks in the way taken up"
            return False

        undo = snapshot()
        displaced = [by_key[placed.key] for placed in culprits if placed.key in by_key]
        for other in displaced:
            take_up(other)
        lay(link, outcome)

        keep = protected | {link.key}
        for other in displaced:
            if not place(other, keep, depth + 1):
                restore(undo)
                link.elements = []
                link.reason = "rip-up would have stranded a track that was already placed"
                return False
        return True

    #: the embedding's own geometry, kept aside so a second attempt at placing the board
    #: starts from the taut answers rather than from whatever the first attempt left behind
    taut_of = {link.key: list(link.elements) for link in links}
    taut_layer = {link.key: link.layer for link in links}

    def run_placement(sequence, rescue: bool = True) -> tuple[list["_Link"], list["_Link"]]:
        for layer in usable:
            settled[layer] = []
        how.clear()
        for link in links:
            link.elements = list(taut_of[link.key])
            link.layer = taut_layer[link.key]
            link.reason = ""

        missed: list[_Link] = []
        for link in sequence:
            if link.layer is None:
                continue
            outcome = attempt(link, frozenset())
            if outcome is None:
                missed.append(link)
                continue
            lay(link, outcome)

        saved: list[_Link] = []
        if rescue:
            for link in list(missed):
                if place(link, frozenset(), 0):
                    missed.remove(link)
                    saved.append(link)
        return missed, saved

    # Shortest first: a short connection bent around a long one costs almost nothing, a
    # long one bent around dozens of short ones costs a lot -- measured, 77 mm of it. But
    # short-first can wall a long connection in completely, so if anything is left
    # stranded the whole placement is redone longest-first, which completes; the shorter
    # board is only the better board when it is a whole one.
    # The first pass carries no rescue: rip-up is the expensive tool, and spending minutes
    # of it inside an ordering that is about to be abandoned buys nothing. Whoever
    # short-first strands is not evidence against the order, only against their own place in
    # it -- so the second try seats exactly them first, longest leading, and everyone else
    # still shortest-first. Only if that also strands someone does the placement fall back
    # to longest-first throughout, which completes.
    ordered = sorted((link for link in links if link.layer is not None),
                     key=lambda l: l.span)
    stranded, rescued_links = run_placement(ordered, rescue=False)
    if stranded:
        stranded, rescued_links = run_placement(list(reversed(ordered)))

    # The cost arbiter answers to placement. Its settlements are priced pairwise, and a
    # pairwise price is blind to one cost: reshaping the sketch so that a third wire has no
    # home at all. Blame for that is genuinely untraceable -- it was measured surviving
    # per-settlement vetoes, inverted movers, deep rip-up, and a via audition -- so the
    # response is not cleverer blame but a guarantee: if anyone is stranded, the whole
    # pipeline runs once more with arbitration off. A complete board outranks a shorter
    # one, always; the arbitration is kept exactly where it keeps everyone housed.
    if stranded and _veto_depth == 0 and not _frozen_movers:
        if verbose:
            for link in stranded:
                print(f"  stranded: net {link.net.name} span {link.span / 1e6:.2f} mm "
                      f"layer {link.layer}: {link.reason}")
                if link.elements:
                    taut_path = _as_path(link.elements)
                    naming = {}
                    for placed_row in settled[link.layer]:
                        for ob in _path_obstacles(placed_row.path, 1.0, placed_row.net):
                            naming[id(ob)] = placed_row.net
                    hits = []
                    everything = in_the_way(link.layer, link, frozenset())
                    for slot in violated_obstacles(taut_path, everything):
                        ob = everything[slot]
                        hits.append(naming.get(id(ob), getattr(ob, "label", "?")))
                    print(f"    taut blocked by: {hits[:10]}")
            print(f"  {len(stranded)} stranded after settling; "
                  f"re-running in ban mode")
        return plan_board(board, layers=layers, rounds=rounds, verbose=verbose,
                          _frozen_movers=frozenset({("off",)}), _veto_depth=1)

    # ---- shorten what the whole board can spare ----------------------------------------
    #
    # With everyone placed and legal, a swap that is shorter *and* legal against the entire
    # standing board preserves global legality outright -- placement is over, nothing after
    # depends on it. In the tiered world this descent measured zero (every wire was already
    # at its best response); the woven world holds wires to their corridors, so slack
    # exists, and this is where the corridor discipline's cost is paid back.
    if woven_order is not None and not stranded:
        for _swap_round in range(3):
            improved = False
            standing = [link for link in links if link.elements and link.layer]
            standing.sort(key=lambda l: -(_as_path(l.elements).length - l.span))
            for link in standing:
                before = _as_path(link.elements).length
                if before - link.span < 100_000:
                    continue
                keep = (list(link.elements), link.layer, how.get(link.key, "taut"))
                take_up(link)
                edge_gap = board.edge_clearance_nm + link.width / 2.0 + GUARDBAND_NM
                ends = ((float(link.pad_a.x), float(link.pad_a.y)),
                        (float(link.pad_b.x), float(link.pad_b.y)))
                best = None
                for layer in candidates(link):
                    try:
                        found = _solve_lazily(*ends, in_the_way(layer, link, frozenset()),
                                              boundary=boundary, boundary_gap=edge_gap)
                    except NoPathFound:
                        continue
                    if best is None or found.length < best[0].length:
                        best = (found, layer)
                if best is not None and best[0].length < before - 10_000.0:
                    lay(link, (best[0], best[1], "swapped"))
                    improved = True
                    checked["swapped"] = checked.get("swapped", 0) + 1
                else:
                    link.elements, link.layer = keep[0], keep[1]
                    how[link.key] = keep[2]
                    settled[link.layer].append(_Placed(
                        key=link.key, net=link.net.code, path=_as_path(link.elements),
                        halo=link.halo, half=link.width / 2.0,
                        clearance=link.clearance))
            if not improved:
                break

    # A connection that could not be placed keeps the geometry the embedding gave it, and the
    # emitter below writes whatever a link is holding. Nothing had cleared it, so a connection
    # reported as dropped was still being drawn -- unchecked, on top of whatever was there.
    for link in stranded:
        link.elements = []

    # Everything above checks a connection against what was already down when it was placed.
    # That is not the same as checking the board, and the difference is where a rip-up
    # exchange can leave two tracks overlapping with nobody having compared them. Ask once
    # more, at the end, about exactly what is going to be drawn.
    drawn = [link for link in links if link.elements and link.layer]
    overlaps = 0
    for position, link in enumerate(drawn):
        mine = _as_path(link.elements)
        for other in drawn[position + 1:]:
            if other.net.code == link.net.code or other.layer != link.layer:
                continue
            gap = (max(link.clearance, other.clearance) + link.width / 2.0
                   + other.width / 2.0 + GUARDBAND_NM)
            if violated_obstacles(mine, _path_obstacles(_as_path(other.elements),
                                                        gap, other.net.code)):
                overlaps += 1
    checked["overlaps"] = overlaps

    checked["taut"] = sum(1 for kind in how.values() if kind == "taut")
    checked["fell_back"] = sum(1 for kind in how.values() if kind == "fell_back")
    checked["rescued"] = len(rescued_links)
    checked["dropped"] = sum(1 for link in stranded if not link.elements)

    # A connection is either on the board or it is not. Half of one -- copper up to a via
    # and nothing on the other side -- is worse than none: the via is then a hole joined to
    # one track, which is a defect in its own right rather than a missing connection.
    whole: dict[int, bool] = {}
    for link in links:
        whole[link.parent] = whole.get(link.parent, True) and bool(link.elements)

    for owner, complete in sorted(whole.items()):
        parent = by_route[owner]
        if complete:
            result.routed.append((parent.net.code, parent.net.name))
            continue
        excuse = next((link.reason for link in links
                       if link.parent == owner and link.reason), "no legal geometry")
        result.failed.append((parent.net.code, parent.net.name, excuse))

    for link in links:
        if not link.elements or not whole.get(link.parent):
            continue
        result.tracks.extend(_elements_to_tracks(link.elements, link.net.code,
                                                 link.layer, int(link.width)))

    result.vias.extend(Via(net=via.net, x=int(round(via.x)), y=int(round(via.y)),
                           diameter_nm=int(via.diameter), drill_nm=int(via.drill))
                       for via in placed_vias if whole.get(via.owner))

    result.stats.update({
        "connections": len(whole),
        "routed": len(result.routed),
        "failed": len(result.failed),
        "tracks": len(result.tracks),
        "arcs": result.arc_count,
        "length_mm": round(result.total_length_nm / 1e6, 2),
        "vias": len(result.vias),
        **{f"topo_{k}": v for k, v in topo_stats.items()},
        **{f"embed_{k}": v for k, v in checked.items()},
    })
    return result
