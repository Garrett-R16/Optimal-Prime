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
from .mesh import Mesh, build_mesh
from .obstacles import Obstacle, pad_obstacle
from .route import (ArcTrack, RouteResult, Track, _convex_hull,
                    _copper_shape_obstacles, MIN_PIECE_NM, MIN_SAGITTA_NM)
from .topo import Route, route_topology
from .units import CLEARANCE_MARGIN, GUARDBAND_NM

__all__ = ["plan_board"]


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


def _cores(board: Board, layer: str) -> list[Obstacle]:
    """Obstacle *cores* -- actual copper, with no clearance baked in.

    Clearance belongs to the embedding, where it narrows each doorway. Inflating here would
    delete every triangle that touches a pad and leave no free space at all.
    """
    out = [pad_obstacle(pad, 0.0, 0.0) for pad in board.pads if pad.on_layer(layer)]
    out.extend(_copper_shape_obstacles(board, layer, 0.0))
    return out


def _gates_for(mesh: Mesh, route: Route, slot_of, clearance: float, width: float,
               halo: float) -> list[Gate] | None:
    """Turn a portal sequence into the doorways this route may actually use."""
    gates: list[Gate] = []
    cursor = route.start

    for index, key in enumerate(route.portals):
        portal = mesh.portals[key]
        pa = (float(mesh.points[key[0]][0]), float(mesh.points[key[0]][1]))
        pb = (float(mesh.points[key[1]][0]), float(mesh.points[key[1]][1]))

        slot, total = slot_of(key, route)
        span = portal.length
        usable = span - 2.0 * halo - (total - 1) * (width + clearance)
        if usable < 0.0:
            return None
        share = usable / total
        stride = (width + clearance) + share

        low = halo + slot * stride
        high = low + share

        ux, uy = (pb[0] - pa[0]) / span, (pb[1] - pa[1]) / span
        end_a = (pa[0] + ux * low, pa[1] + uy * low)
        end_b = (pa[0] + ux * (span - (halo + (total - 1 - slot) * stride)),
                 pa[1] + uy * (span - (halo + (total - 1 - slot) * stride)))
        if math.dist(end_a, end_b) < 0.0:
            return None

        heading = route.start if index == 0 else cursor
        ahead = route.goal if index == len(route.portals) - 1 else \
            mesh.centroid(route.triangles[index + 1])
        left, right = orient(heading, ahead, end_a, end_b)

        left_vertex = pa if left is end_a else pb
        right_vertex = pb if left is end_a else pa
        left_radius = low if left is end_a else halo + (total - 1 - slot) * stride
        right_radius = halo + (total - 1 - slot) * stride if left is end_a else low

        gates.append(Gate(left=left, right=right,
                          left_vertex=left_vertex, right_vertex=right_vertex,
                          left_radius=max(left_radius, 1.0),
                          right_radius=max(right_radius, 1.0)))
        cursor = mesh.centroid(route.triangles[index + 1])

    return gates


def _elements_to_tracks(elements, net: int, layer: str, width_nm: int) -> list:
    out = []
    for element in elements:
        if isinstance(element, Line):
            if element.length < MIN_PIECE_NM:
                continue
            out.append(Track(net=net, layer=layer,
                             x1=int(round(element.x1)), y1=int(round(element.y1)),
                             x2=int(round(element.x2)), y2=int(round(element.y2)),
                             width_nm=width_nm))
        else:
            if element.length < MIN_PIECE_NM:
                continue
            sx, sy = element.at(0.0)
            mx, my = element.at(0.5)
            ex, ey = element.at(1.0)
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

    # Assign each link to a layer, then choose topology per layer.
    by_layer: dict[str, list[_Link]] = {layer: [] for layer in usable}
    for link in links:
        options = [layer for layer in usable
                   if link.pad_a.on_layer(layer) and link.pad_b.on_layer(layer)]
        if not options:
            result.failed.append((link.net.code, link.net.name,
                                  "pads share no usable layer (a via would be needed)"))
            continue
        # Spread across layers by current load, so one face does not absorb everything.
        link.layer = min(options, key=lambda l: len(by_layer[l]))
        by_layer[link.layer].append(link)

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

        # Seat the nets across every shared portal.
        users: dict[tuple[int, int], list[int]] = {}
        for route in routes:
            for key in route.portals:
                if route.net not in users.setdefault(key, []):
                    users[key].append(route.net)

        def slot_of(key, route, _users=users):
            holders = _users.get(key, [route.net])
            try:
                return holders.index(route.net), len(holders)
            except ValueError:
                return 0, max(1, len(holders))

        index = {link.key: link for link in group}
        for route in routes:
            link = index[route.key]
            if not route.found:
                link.reason = "no route through the free space"
                continue
            gates = _gates_for(mesh, route, slot_of, clearance, width, link.halo)
            if gates is None:
                link.reason = "a doorway on the route is too narrow for its slot"
                continue
            wraps = funnel(route.start, route.goal, gates)
            link.elements = taut_through(route.start, route.goal, wraps)

    for link in links:
        if not link.elements:
            if link.reason:
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
    })
    return result
