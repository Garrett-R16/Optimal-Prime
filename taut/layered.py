"""Choosing routes across the whole stack at once, vias included.

:mod:`taut.topo` chooses a route through the doorways of one layer. That is the right idea
applied to half the problem: which layer a connection goes on was decided beforehand and never
revisited, so a connection whose only way through is to change layers had no way to say so, and
the layer assignment could do no better than hope the graph of what-crosses-what was bipartite.
Three nets that pairwise cross settle that hope on any real board.

Here a route is a path through a graph whose nodes are *(layer, triangle)* and whose edges are
of two kinds: a doorway, joining two triangles of one layer, and a via, joining the same place
on every layer. A via is not a repair applied afterwards, it is an edge with a price, and the
search spends it when it is worth spending. The price is deliberately blunt -- one number, in
the same units as length -- because that is what a via is: a fixed amount of nuisance that buys
you the other side of everything in the way.

Both kinds of edge are negotiated the same way, by the congestion pricing already used for
doorways: a doorway holds as many tracks as fit across it, a site holds one via, and wanting
what is taken costs more every round until somebody goes elsewhere. Sites are found once from
the geometry (:mod:`taut.vias`); through vias only, so taking one takes the spot out of every
layer at once and the price says so.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

from .mesh import Mesh, Portal
from .vias import Site, sites_by_triangle

__all__ = ["Leg", "Route", "StackReport", "route_stack"]

#: What a via costs the search, in the same units as length. Big enough that a connection does
#: not take one to save a fraction of a millimetre, small enough that it will cross the board
#: to reach one rather than give up.
VIA_COST_NM = 6_000_000.0

#: Charged per doorway, so that of two routes of equal length the search prefers the one
#: through fewer, wider gaps.
PORTAL_PENALTY_NM = 25_000.0

#: How much dearer a layer is that a connection would rather not be on. Small: enough that it
#: keeps to the side it was asked to, not so much that it would sooner go round the board than
#: change over.
OFF_PREFERENCE = 1.35


@dataclass
class Leg:
    """The part of a route that stays on one layer."""

    layer: int
    start: tuple[float, float]
    goal: tuple[float, float]
    triangles: list[int] = field(default_factory=list)
    portals: list[tuple[int, int]] = field(default_factory=list)


@dataclass
class Route:
    """One connection's chosen class: which side of everything, and where it changes layer."""

    key: int
    net: int
    start: tuple[float, float]
    goal: tuple[float, float]
    legs: list[Leg] = field(default_factory=list)
    #: indices into the site list, one per layer change, in order along the route
    vias: list[int] = field(default_factory=list)

    @property
    def found(self) -> bool:
        return bool(self.legs)

    def uses(self):
        """Every resource this route holds, as keys the negotiation can count."""
        for leg in self.legs:
            for portal in leg.portals:
                yield ("portal", leg.layer, portal)
        for site in self.vias:
            yield ("via", site)


@dataclass
class StackReport:
    rounds: int = 0
    routed: int = 0
    unroutable: int = 0
    vias: int = 0
    tangled: int = 0
    overfull: list = field(default_factory=list)
    converged: bool = False


def _midpoint(mesh: Mesh, key: tuple[int, int]) -> tuple[float, float]:
    a, b = mesh.points[key[0]], mesh.points[key[1]]
    return float((a[0] + b[0]) / 2), float((a[1] + b[1]) / 2)


def _direct_walk(mesh: Mesh, layer: int, start, goal, home: int):
    """The straight line between two terminals, told as the doorways it actually crosses.

    A shortcut that reports no doorways is invisible: it holds no rank in any crossing
    order, consumes no capacity, and cannot be seated against the wires it passes among --
    which is how straight legs ended up crossing other wires with nothing anywhere
    recording the conflict. The line stays a line; what changes is that every doorway it
    passes through is on the record, so everything downstream treats it like any other
    wire.
    """
    ax, ay = start
    bx, by = goal
    dx, dy = bx - ax, by - ay

    crossed: list[tuple[float, tuple[int, int]]] = []
    for key, portal in mesh.portals.items():
        pa, pb = mesh.points[key[0]], mesh.points[key[1]]
        px, py = float(pa[0]), float(pa[1])
        ex, ey = float(pb[0]) - px, float(pb[1]) - py
        denominator = dx * ey - dy * ex
        if abs(denominator) < 1e-12:
            continue
        t = ((px - ax) * ey - (py - ay) * ex) / denominator
        u = ((px - ax) * dy - (py - ay) * dx) / denominator
        if 0.0 < t < 1.0 and 0.0 < u < 1.0:
            crossed.append((t, key))
    crossed.sort()

    if not crossed:
        return (layer, home), [(layer, home, ("d",))]

    # The triangle after each crossing, read from the line itself: the point midway to the
    # next crossing is inside it.
    stations = [t for t, _ in crossed] + [1.0]
    walk = []
    previous = None
    for index, (t, key) in enumerate(crossed):
        middle = (t + stations[index + 1]) / 2.0
        tri = mesh._locate(ax + dx * middle, ay + dy * middle)
        if tri < 0 or not mesh.free[tri]:
            tri = previous if previous is not None else home
        walk.append((layer, int(tri), ("p", key)))
        previous = int(tri)

    first_middle = stations[0] / 2.0
    origin = mesh._locate(ax + dx * first_middle, ay + dy * first_middle)
    if origin < 0 or not mesh.free[origin]:
        origin = home
    return (layer, int(origin)), walk


def _crosses_avoid(avoid, layer: int, ax: float, ay: float, bx: float, by: float) -> bool:
    """Would a step from a to b on this layer cross any committed wire it must respect?"""
    lo_x, hi_x = (ax, bx) if ax <= bx else (bx, ax)
    lo_y, hi_y = (ay, by) if ay <= by else (by, ay)
    for wall_layer, polyline, box in avoid:
        if wall_layer != layer:
            continue
        if hi_x < box[0] or box[2] < lo_x or hi_y < box[1] or box[3] < lo_y:
            continue
        for (px, py), (qx, qy) in zip(polyline, polyline[1:]):
            if _crosses(ax, ay, bx, by, px, py, qx, qy):
                return True
    return False


def _search(meshes: list[Mesh], sites: list[Site], by_triangle, start, goal,
            start_tris: list[list[int]], goal_tris: list[list[int]], net: int,
            price, via_cost: float, prefer: int | None = None,
            bias: float = 1.0, bans: frozenset = frozenset(),
            avoid: tuple = (), node_limit: int = 600_000,
            veto: frozenset = frozenset(), site_veto: frozenset = frozenset()):
    """A* over (layer, triangle), stepping doorway to doorway and layer to layer.

    ``veto`` lists layers this connection may not touch at all -- the weave's feedback
    when a layer that looked cheap turns out to have no planar corridor on it.
    """
    goals = {(layer, tri) for layer, tris in enumerate(goal_tris) for tri in tris}
    if not goals or not any(start_tris):
        return None

    # If the two ends are in the same place on some layer and nothing is between them, the
    # answer is the straight line and there is nothing to choose. Sending it through a doorway
    # anyway to keep the bookkeeping uniform costs a few millimetres on every short hop, which
    # on ecc83-pp came to 22 mm over twenty connections.
    order = ([prefer] if prefer is not None else []) + [
        index for index in range(len(meshes)) if index != prefer]
    for layer in order:
        if layer in veto:
            continue
        if (net, layer, -1, frozenset()) in bans:
            # The straight line is clear of copper but crosses another wire; this net has
            # to route properly on this layer, doorway by doorway.
            continue
        together = set(start_tris[layer]) & set(goal_tris[layer])
        if (together and meshes[layer].clear_between(start, goal)
                and not _crosses_avoid(avoid, layer, start[0], start[1],
                                       goal[0], goal[1])):
            # Measured both ways: telling the straight leg its crossed doorways bends it
            # into the rank stacks at every portal it touches -- +3.3 mm on ecc83-pp and a
            # short on sonde xilinx -- so the shortcut stays a shortcut, and crossings that
            # involve one are settled by the check phase instead.
            home = next(iter(together))
            return (layer, home), [(layer, home, ("d",))]

    gx, gy = goal
    cache: dict[tuple[int, tuple[int, int]], tuple[float, float]] = {}

    def where(layer: int, marker) -> tuple[float, float]:
        if marker[0] == "v":
            site = sites[marker[1]]
            return site.x, site.y
        key = (layer, marker[1])
        point = cache.get(key)
        if point is None:
            point = cache[key] = _midpoint(meshes[layer], marker[1])
        return point

    heap: list[tuple[float, float, tuple]] = []
    best: dict[tuple, float] = {}
    came: dict[tuple, tuple | None] = {}

    def relax(cost: float, state, previous, px: float, py: float,
              charge: float, extra: float) -> None:
        mx, my = where(state[0], state[2])
        step = math.hypot(mx - px, my - py) * charge + extra
        if state[0] in veto:
            return
        if state[2][0] == "v" and state[2][1] in site_veto:
            return
        rest = math.hypot(mx - gx, my - gy)
        if (state[0], state[1]) in goals:
            step += rest
            rest = 0.0
        total = cost + step
        if total < best.get(state, math.inf) - 1e-9:
            best[state] = total
            came[state] = previous
            heapq.heappush(heap, (total + rest, total, state))

    origins = {(layer, tri) for layer, tris in enumerate(start_tris) for tri in tris
               if layer not in veto}
    for layer, tri in sorted(origins):
        _expand(meshes, sites, by_triangle, layer, tri, origins, price, net, via_cost,
                lambda state, charge, extra, _l=layer, _t=tri: relax(
                    0.0, state, ("S", _l, _t), start[0], start[1],
                    charge * (bias if prefer is not None and state[0] != prefer else 1.0),
                    extra)
                if not (state[2][0] == "p"
                        and ((net, _l, _t, frozenset((state[2][1],))) in bans
                             or (avoid and _crosses_avoid(
                                 avoid, state[0], start[0], start[1],
                                 *where(state[0], state[2])))))
                else None)

    expanded = 0
    final = None
    while heap:
        _, cost, state = heapq.heappop(heap)
        if cost > best.get(state, math.inf):
            continue
        layer, tri, marker = state
        if (layer, tri) in goals:
            final = state
            break
        expanded += 1
        if expanded > node_limit:
            return None
        px, py = where(layer, marker)
        entered = marker[1] if marker[0] == "p" else None
        _expand(meshes, sites, by_triangle, layer, tri, set(), price, net, via_cost,
                lambda nxt, charge, extra, _c=cost, _s=state, _p=(px, py):
                relax(_c, nxt, _s, _p[0], _p[1],
                      charge * (bias if prefer is not None and nxt[0] != prefer else 1.0),
                      extra)
                if nxt[2] != marker and not (
                    nxt[2][0] == "p"
                    and ((net, layer, tri, frozenset((nxt[2][1],))) in bans
                         or (entered is not None
                             and (net, layer, tri,
                                  frozenset((entered, nxt[2][1]))) in bans)
                         or (avoid and _crosses_avoid(
                             avoid, nxt[0], px, py, *where(nxt[0], nxt[2])))))
                else None)

    if final is None:
        return None

    walk = []
    cursor = final
    origin = None
    while cursor is not None:
        if cursor[0] == "S":
            origin = (cursor[1], cursor[2])
            break
        walk.append(cursor)
        cursor = came[cursor]
    walk.reverse()
    return origin, walk


def _expand(meshes, sites, by_triangle, layer: int, tri: int, skip, price, net: int,
            via_cost: float, offer) -> None:
    """Every move out of one triangle: through a doorway, or down a via."""
    mesh = meshes[layer]
    for other, portal in mesh.adjacent(tri):
        if (layer, other) in skip:
            continue
        offer((layer, other, ("p", portal.key())),
              price(("portal", layer, portal.key()), net, portal.capacity),
              PORTAL_PENALTY_NM)

    for index in by_triangle[layer].get(tri, ()):
        site = sites[index]
        for other_layer in range(len(meshes)):
            if other_layer == layer:
                continue
            landing = site.triangle_on(other_layer)
            if not meshes[other_layer].free[landing]:
                continue
            # The marker carries the layer being left as well as the site. Without it the
            # only layer recorded is the one arrived on, and the leg *up to* the via gets
            # closed with the layer on the far side of it -- so both legs come out on the
            # same face and the via joins a track to itself.
            offer((other_layer, landing, ("v", index, layer)), 1.0,
                  via_cost * price(("via", index), net, 1))


def _crosses(ax, ay, bx, by, cx, cy, dx, dy) -> bool:
    def wind(px, py, qx, qy, rx, ry):
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)
    d1 = wind(cx, cy, dx, dy, ax, ay)
    d2 = wind(cx, cy, dx, dy, bx, by)
    d3 = wind(ax, ay, bx, by, cx, cy)
    d4 = wind(ax, ay, bx, by, dx, dy)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _chain(mesh: Mesh, leg: Leg) -> list[tuple[float, float]]:
    return [leg.start] + [_midpoint(mesh, key) for key in leg.portals] + [leg.goal]


def _tangles(meshes: list[Mesh], routes: list[Route]):
    """Pairs of routes that would cross, and roughly where.

    Two wires on one layer crossing is a short, and no amount of pulling them taut afterwards
    separates them -- which side of each other they pass is settled here or not at all. It is
    also exactly what a via is for, so a crossing is priced like any other contested resource
    and the search decides for itself whether changing layer is worth less than going round.
    """
    chains: dict[int, list[tuple[Route, list[tuple[float, float]]]]] = {}
    for route in routes:
        if not route.found:
            continue
        for leg in route.legs:
            chains.setdefault(leg.layer, []).append((route, _chain(meshes[leg.layer], leg)))

    out = []
    for items in chains.values():
        boxes = [(min(x for x, _ in pts), min(y for _, y in pts),
                  max(x for x, _ in pts), max(y for _, y in pts)) for _, pts in items]
        for first in range(len(items)):
            route_a, path_a = items[first]
            for second in range(first + 1, len(items)):
                route_b, path_b = items[second]
                if route_a.net == route_b.net:
                    continue
                one, two = boxes[first], boxes[second]
                if (one[2] < two[0] or two[2] < one[0]
                        or one[3] < two[1] or two[3] < one[1]):
                    continue
                where = _where_crossed(path_a, path_b)
                if where is not None:
                    out.append((route_a, route_b, where))
    return out


def _where_crossed(first, second):
    for pa, pb in zip(first, first[1:]):
        for qa, qb in zip(second, second[1:]):
            if _crosses(pa[0], pa[1], pb[0], pb[1], qa[0], qa[1], qb[0], qb[1]):
                return (pa[0] + pb[0] + qa[0] + qb[0]) / 4.0, (pa[1] + pb[1] + qa[1] + qb[1]) / 4.0
    return None


def _blame(meshes: list[Mesh], route: Route, where, history: dict) -> None:
    """Make the doorway nearest a crossing dearer for the route that used it."""
    best, resource = math.inf, None
    for leg in route.legs:
        for key in leg.portals:
            mx, my = _midpoint(meshes[leg.layer], key)
            reach = math.hypot(mx - where[0], my - where[1])
            if reach < best:
                best, resource = reach, ("portal", leg.layer, key)
    if resource is not None:
        history[resource] = history.get(resource, 0.0) + 1.0


def route_stack(meshes: list[Mesh], sites: list[Site], requests, *,
                terminals, via_cost: float = VIA_COST_NM, rounds: int = 12,
                present: float = 0.6, bias: float = OFF_PREFERENCE,
                bans: frozenset = frozenset(), warm: list | None = None,
                only: set | None = None, avoid_for: dict | None = None,
                veto_for: dict | None = None, site_veto_for: dict | None = None,
                verbose: bool = False):
    """Choose a route for every connection at once, over the whole stack.

    ``requests`` are ``(key, net, start, goal, preferred_layer?)``; ``terminals(point)``
    gives, per layer, the free triangles a connection may leave that point by -- empty for a
    layer the pad is not on.

    ``bans`` are turns a specific net may not make: ``(net, layer, triangle,
    frozenset({portal_in, portal_out}))``. A wire that enters a triangle and leaves it by two
    edges that are not another wire's exit separates that wire's terminal from its own first
    doorway -- they *must* cross, and no shared doorway ever records it. The reference systems
    make such moves unrepresentable (SURF's region fans; the valid-point pruning in Zhan
    2017); a ban is the same fact expressed to this search. Bans only accumulate, so reroute
    rounds converge: the banned net goes around the terminal or buys a via.
    """
    #: Which layer each connection would rather be on, worked out before any search from
    #: which straight pad-to-pad lines cross which. It is a preference and not a rule: the
    #: search pays a little more to leave it, which it will do to reach a via.
    wanted = {entry[0]: (entry[4] if len(entry) > 4 else None) for entry in requests}
    routes = [Route(key=entry[0], net=entry[1], start=entry[2], goal=entry[3])
              for entry in requests]

    # A repair pass re-routes only the wires implicated in a defect, against everything else
    # exactly where it stands. Re-negotiating the whole board for one bad turn churns every
    # route and accumulates stale constraints; the reference systems repair sequentially for
    # this reason, and so does this.
    if warm is not None:
        by_key = {route.key: route for route in warm}
        for route in routes:
            old = by_key.get(route.key)
            if old is not None and old.found:
                route.legs = [Leg(layer=leg.layer, start=leg.start, goal=leg.goal,
                                  triangles=list(leg.triangles),
                                  portals=list(leg.portals)) for leg in old.legs]
                route.vias = list(old.vias)
    report = StackReport()
    by_triangle = sites_by_triangle(sites, len(meshes))

    usage: dict[tuple, set[int]] = {}
    history: dict[tuple, float] = {}

    def price(resource, net: int, capacity: int) -> float:
        holders = usage.get(resource)
        others = 0 if not holders else len(holders) - (1 if net in holders else 0)
        over = max(0, others + 1 - max(1, capacity))
        return (1.0 + history.get(resource, 0.0)) * (1.0 + present * over)

    ends: dict[tuple[float, float], list[list[int]]] = {}

    def reach(point):
        found = ends.get(point)
        if found is None:
            found = ends[point] = terminals(point)
        return found

    for _round in range(rounds):
        report.rounds += 1

        for route in routes:
            for resource in route.uses():
                holders = usage.get(resource)
                if holders:
                    holders.discard(route.net)

        for route in routes:
            if only is not None and route.key not in only and route.found:
                for resource in route.uses():
                    usage.setdefault(resource, set()).add(route.net)
                continue
            found = _search(meshes, sites, by_triangle, route.start, route.goal,
                            reach(route.start), reach(route.goal), route.net,
                            price, via_cost, wanted.get(route.key), bias, bans,
                            tuple((avoid_for or {}).get(route.key, ())),
                            veto=frozenset((veto_for or {}).get(route.key, ())),
                            site_veto=frozenset(
                                (site_veto_for or {}).get(route.key, ())))
            origin, walk = found if found else (None, None)
            _rebuild(route, origin, walk, sites)
            for resource in route.uses():
                usage.setdefault(resource, set()).add(route.net)

        tangled = _tangles(meshes, routes)

        overfull = []
        for resource, holders in usage.items():
            if resource[0] == "via":
                if len(holders) > 1:
                    overfull.append(resource)
            else:
                _, layer, key = resource
                if len(holders) > max(1, meshes[layer].portals[key].capacity):
                    overfull.append(resource)

        if verbose:
            print(f"  round {report.rounds}: "
                  f"{sum(1 for r in routes if r.found)}/{len(routes)} routed, "
                  f"{sum(len(r.vias) for r in routes)} vias, "
                  f"{len(overfull)} resources over capacity, "
                  f"{len(tangled)} pairs crossing")

        if not overfull:
            report.converged = True
            break
        for resource in overfull:
            history[resource] = history.get(resource, 0.0) + 1.0
        # Crossings are counted but not priced. Charging the doorway nearest one does move
        # the route, and then it comes back: measured on ecc83-pp the count went 5, 5, 4, 4,
        # 6 and never settled, because a route with few doorways has nowhere to be pushed to.
        # Which side of each other two wires pass is decided by the layer preference above,
        # where it converges because it is decided once.

    report.routed = sum(1 for route in routes if route.found)
    report.unroutable = len(routes) - report.routed
    report.vias = sum(len(route.vias) for route in routes)
    report.overfull = overfull
    report.tangled = len(tangled)
    return routes, report


def _rebuild(route: Route, origin, walk, sites: list[Site]) -> None:
    """Turn the search's states back into legs, split wherever it changed layer.

    Each leg's ``triangles`` lead its ``portals`` by one: ``triangles[i]`` is the triangle
    ``portals[i]`` is crossed *out of*, and the last triangle is where the leg ends. The
    stub ban needs that alignment -- "may not leave triangle T by portal P" is only
    expressible if T is on record.
    """
    route.legs = []
    route.vias = []
    if not walk:
        return

    opening = walk[0][2][2] if walk[0][2][0] == "v" else walk[0][0]
    leg = Leg(layer=opening, start=route.start, goal=route.goal)
    if origin is not None:
        leg.triangles.append(origin[1])
    for layer, tri, marker in walk:
        if marker[0] == "d":
            leg.layer = layer
            if tri not in leg.triangles:
                leg.triangles.append(tri)
            continue
        if marker[0] == "v":
            site = sites[marker[1]]
            leg.goal = (site.x, site.y)
            leg.layer = marker[2]
            route.legs.append(leg)
            route.vias.append(marker[1])
            leg = Leg(layer=layer, start=(site.x, site.y), goal=route.goal)
            leg.triangles.append(tri)
            continue
        leg.portals.append(marker[1])
        leg.triangles.append(tri)

    route.legs.append(leg)
    for one in route.legs:
        if not one.triangles:
            one.triangles.append(walk[0][1])
