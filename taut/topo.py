"""Choosing routes as sequences of portals, before any geometry exists.

This is the part every earlier version skipped. A shortest-path solver handed the obstacles
picks a homotopy class -- which side of each pad to pass -- silently, as a side effect, and
two nets can pick incompatible ones without anything noticing. Here the class is the thing
being chosen, explicitly, for every net at once, against a resource that can be counted.

The resource is portal capacity (:mod:`taut.mesh`). Routes are found by A* over the doorways
themselves -- from one portal to the next, rather than from one triangle to the next, since a
route is a chain of doorways and its length is theirs -- and a portal wanted by more nets than
fit gets progressively more expensive to use, so nets peel off to other doorways rather than
piling up and being discovered in collision afterwards. That is PathFinder's negotiated
congestion, applied where it belongs: to the topology, not to the geometry.

Two costs, as in the original formulation:

* **present** -- how over-subscribed a portal is right now, scaled by a factor that grows each
  round, so sharing starts cheap and becomes unaffordable;
* **history** -- accumulated for portals that stayed contested, which is the memory that stops
  two nets swapping the same doorway forever.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

from .mesh import Mesh, Portal

__all__ = ["Route", "TopoReport", "route_topology"]


@dataclass
class Route:
    """One connection's chosen homotopy class."""

    key: int
    net: int
    start: tuple[float, float]
    goal: tuple[float, float]
    triangles: list[int] = field(default_factory=list)
    portals: list[tuple[int, int]] = field(default_factory=list)

    @property
    def found(self) -> bool:
        return bool(self.triangles)


@dataclass
class TopoReport:
    rounds: int = 0
    routed: int = 0
    unroutable: int = 0
    overfull: list[tuple[int, int]] = field(default_factory=list)
    overfull_by_round: list[int] = field(default_factory=list)
    converged: bool = False


def _portal_midpoint(mesh: Mesh, key: tuple[int, int]) -> tuple[float, float]:
    a, b = mesh.points[key[0]], mesh.points[key[1]]
    return (float((a[0] + b[0]) / 2), float((a[1] + b[1]) / 2))


def _search(mesh: Mesh, start_tris: list[int], goal_tris: list[int], net: int,
            start: tuple[float, float], goal: tuple[float, float],
            cost_of, node_limit: int = 400_000) -> tuple[list[int], list[tuple[int, int]]]:
    """A* over doorways. Returns (triangle sequence, portal sequence).

    A route is a chain of doorways, and what it costs once embedded is the length of that
    chain -- not of the triangles that happen to lie between them. So a step is measured
    from one portal midpoint to the next, and the search state is *(triangle, the portal it
    was entered by)* rather than the triangle alone: where a route enters a triangle decides
    how far it must travel to leave, and a triangle-only state cannot say that.

    Charging centroid -> midpoint -> centroid instead, as this did, adds to every step a
    detour no embedded track will make, and adds a different amount depending on where the
    centroids happen to fall. Routes then follow the triangles whose centroids line up rather
    than the channel that is actually shortest -- worst where triangles are long and thin,
    which is exactly where the channels are.

    A midpoint is a fixed property of a portal, so this remains an ordinary weighted graph
    and A* over it remains exact. Anchoring the step somewhere path-dependent instead -- the
    point on the doorway nearest where the route came from, or nearest the goal -- tracks the
    taut length more closely, but makes an edge weight depend on the prefix that reached it,
    and measured no better; nor did a flat charge per doorway crossed, which changed nothing
    until it was large enough to buy detours. Neither is worth giving up exactness for.
    """
    if not start_tris or not goal_tris:
        return [], []
    goals = set(goal_tris)
    if goals & set(start_tris):
        return [next(iter(goals & set(start_tris)))], []

    gx, gy = goal
    midpoints: dict[tuple[int, int], tuple[float, float]] = {}

    def midpoint(key: tuple[int, int]) -> tuple[float, float]:
        point = midpoints.get(key)
        if point is None:
            point = midpoints[key] = _portal_midpoint(mesh, key)
        return point

    # Straight from the doorway to the goal. Whatever follows is a polyline through that
    # midpoint, and congestion only ever multiplies a step upwards (``cost_of`` >= 1), so
    # this never overestimates what is left: A* stays admissible, and in fact consistent.
    def heuristic(px: float, py: float) -> float:
        return math.hypot(px - gx, py - gy)

    State = tuple[int, tuple[int, int]]
    heap: list[tuple[float, float, State]] = []
    best: dict[State, float] = {}
    came: dict[State, State | None] = {}

    def relax(cost: float, state: State, previous: State | None, portal: Portal,
              px: float, py: float) -> None:
        """Charge crossing into ``state``'s triangle, and queue it if that is an improvement."""
        mx, my = midpoint(state[1])
        step = math.hypot(mx - px, my - py) * cost_of(portal, net)
        rest = heuristic(mx, my)
        if state[0] in goals:
            # The last leg runs from the final doorway to the pad, so charge it here rather
            # than letting the goal triangle be crossed for free.
            step += rest
            rest = 0.0
        total = cost + step
        if total < best.get(state, math.inf) - 1e-9:
            best[state] = total
            came[state] = previous
            heapq.heappush(heap, (total + rest, total, state))

    origins = set(start_tris)
    for tri in start_tris:
        for other, portal in mesh.adjacent(tri):
            if other in origins:
                continue
            relax(0.0, (other, portal.key()), None, portal, start[0], start[1])

    expanded = 0
    final: State | None = None
    while heap:
        _, cost, state = heapq.heappop(heap)
        if cost > best.get(state, math.inf):
            continue
        tri, entry = state
        if tri in goals:
            final = state
            break
        expanded += 1
        if expanded > node_limit:
            return [], []
        px, py = midpoint(entry)
        for other, portal in mesh.adjacent(tri):
            key = portal.key()
            if key == entry:
                continue
            relax(cost, (other, key), state, portal, px, py)

    if final is None:
        return [], []

    triangles: list[int] = []
    portals: list[tuple[int, int]] = []
    cursor: State | None = final
    while cursor is not None:
        tri, key = cursor
        triangles.append(tri)
        portals.append(key)
        cursor = came[cursor]
    triangles.append(start_tris[0])
    triangles.reverse()
    portals.reverse()
    return triangles, portals


def route_topology(mesh: Mesh, requests, rounds: int = 12,
                   present_growth: float = 1.8, history_gain: float = 0.7,
                   verbose: bool = False) -> tuple[list[Route], TopoReport]:
    """Choose a portal sequence for every request, negotiating over portal capacity.

    ``requests`` are ``(key, net, start_xy, goal_xy)``. Nothing is embedded here; the result
    is purely which doorways each connection goes through, and in what order.
    """
    routes = [Route(key=key, net=net, start=start, goal=goal)
              for key, net, start, goal in requests]
    report = TopoReport()

    usage: dict[tuple[int, int], set[int]] = {}
    history: dict[tuple[int, int], float] = {}
    present = 0.6

    triangle_of: dict[tuple[float, float], list[int]] = {}

    def locate(point: tuple[float, float]) -> list[int]:
        cached = triangle_of.get(point)
        if cached is None:
            cached = mesh.terminals(point[0], point[1])
            triangle_of[point] = cached
        return cached

    def cost_of(portal: Portal, net: int) -> float:
        key = portal.key()
        holders = usage.get(key)
        others = 0 if not holders else len(holders) - (1 if net in holders else 0)
        over = max(0, others + 1 - portal.capacity)
        return (1.0 + history.get(key, 0.0)) * (1.0 + present * over)

    for round_index in range(rounds):
        report.rounds += 1

        for route in routes:
            for key in route.portals:
                holders = usage.get(key)
                if holders:
                    holders.discard(route.net)
                    if not holders:
                        del usage[key]
            route.triangles, route.portals = _search(
                mesh, locate(route.start), locate(route.goal), route.net,
                route.start, route.goal, cost_of)
            for key in route.portals:
                usage.setdefault(key, set()).add(route.net)

        overfull = [key for key, holders in usage.items()
                    if len(holders) > max(1, mesh.portals[key].capacity)]
        report.overfull_by_round.append(len(overfull))
        if verbose:
            print(f"    topology round {round_index + 1}: "
                  f"{len(overfull)} portals over capacity")
        if not overfull:
            report.converged = True
            break

        for key in overfull:
            history[key] = history.get(key, 0.0) + history_gain
        present *= present_growth

    report.overfull = [key for key, holders in usage.items()
                       if len(holders) > max(1, mesh.portals[key].capacity)]
    report.routed = sum(1 for r in routes if r.found)
    report.unroutable = sum(1 for r in routes if not r.found)
    return routes, report
