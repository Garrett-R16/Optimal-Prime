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
    overfull: list = field(default_factory=list)
    converged: bool = False


def _midpoint(mesh: Mesh, key: tuple[int, int]) -> tuple[float, float]:
    a, b = mesh.points[key[0]], mesh.points[key[1]]
    return float((a[0] + b[0]) / 2), float((a[1] + b[1]) / 2)


def _search(meshes: list[Mesh], sites: list[Site], by_triangle, start, goal,
            start_tris: list[list[int]], goal_tris: list[list[int]], net: int,
            price, via_cost: float, node_limit: int = 600_000):
    """A* over (layer, triangle), stepping doorway to doorway and layer to layer."""
    goals = {(layer, tri) for layer, tris in enumerate(goal_tris) for tri in tris}
    if not goals or not any(start_tris):
        return None

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
        rest = math.hypot(mx - gx, my - gy)
        if (state[0], state[1]) in goals:
            step += rest
            rest = 0.0
        total = cost + step
        if total < best.get(state, math.inf) - 1e-9:
            best[state] = total
            came[state] = previous
            heapq.heappush(heap, (total + rest, total, state))

    origins = {(layer, tri) for layer, tris in enumerate(start_tris) for tri in tris}
    for layer, tri in sorted(origins):
        _expand(meshes, sites, by_triangle, layer, tri, origins, price, net, via_cost,
                lambda state, charge, extra: relax(0.0, state, None, start[0], start[1],
                                                   charge, extra))

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
        _expand(meshes, sites, by_triangle, layer, tri, set(), price, net, via_cost,
                lambda nxt, charge, extra, _c=cost, _s=state, _p=(px, py):
                relax(_c, nxt, _s, _p[0], _p[1], charge, extra) if nxt[2] != marker else None)

    if final is None:
        return None

    walk = []
    cursor = final
    while cursor is not None:
        walk.append(cursor)
        cursor = came[cursor]
    walk.reverse()
    return walk


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
            offer((other_layer, landing, ("v", index)), 1.0,
                  via_cost * price(("via", index), net, 1))


def route_stack(meshes: list[Mesh], sites: list[Site], requests, *,
                terminals, via_cost: float = VIA_COST_NM, rounds: int = 12,
                present: float = 0.6, verbose: bool = False):
    """Choose a route for every connection at once, over the whole stack.

    ``requests`` are ``(key, net, start, goal)``; ``terminals(point)`` gives, per layer, the
    free triangles a connection may leave that point by -- empty for a layer the pad is not on.
    """
    routes = [Route(key=key, net=net, start=start, goal=goal)
              for key, net, start, goal in requests]
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
            walk = _search(meshes, sites, by_triangle, route.start, route.goal,
                           reach(route.start), reach(route.goal), route.net,
                           price, via_cost)
            _rebuild(route, walk, sites)
            for resource in route.uses():
                usage.setdefault(resource, set()).add(route.net)

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
                  f"{len(overfull)} resources over capacity")

        if not overfull:
            report.converged = True
            break
        for resource in overfull:
            history[resource] = history.get(resource, 0.0) + 1.0

    report.routed = sum(1 for route in routes if route.found)
    report.unroutable = len(routes) - report.routed
    report.vias = sum(len(route.vias) for route in routes)
    report.overfull = overfull
    return routes, report


def _rebuild(route: Route, walk, sites: list[Site]) -> None:
    """Turn the search's states back into legs, split wherever it changed layer."""
    route.legs = []
    route.vias = []
    if not walk:
        return

    leg = Leg(layer=walk[0][0], start=route.start, goal=route.goal)
    for layer, tri, marker in walk:
        if marker[0] == "v":
            site = sites[marker[1]]
            leg.goal = (site.x, site.y)
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
