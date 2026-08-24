"""Choosing routes as sequences of portals, before any geometry exists.

This is the part every earlier version skipped. A shortest-path solver handed the obstacles
picks a homotopy class -- which side of each pad to pass -- silently, as a side effect, and
two nets can pick incompatible ones without anything noticing. Here the class is the thing
being chosen, explicitly, for every net at once, against a resource that can be counted.

The resource is portal capacity (:mod:`taut.mesh`). Routes are found by A* over the triangle
dual graph, and a portal wanted by more nets than fit gets progressively more expensive to
use, so nets peel off to other doorways rather than piling up and being discovered in
collision afterwards. That is PathFinder's negotiated congestion, applied where it belongs:
to the topology, not to the geometry.

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


def _search(mesh: Mesh, start_tri: int, goal_tri: int, net: int,
            cost_of, node_limit: int = 400_000) -> tuple[list[int], list[tuple[int, int]]]:
    """A* over triangles. Returns (triangle sequence, portal sequence)."""
    if start_tri < 0 or goal_tri < 0:
        return [], []
    if start_tri == goal_tri:
        return [start_tri], []

    goal_c = mesh.centroid(goal_tri)

    def heuristic(tri: int) -> float:
        cx, cy = mesh.centroid(tri)
        return math.hypot(cx - goal_c[0], cy - goal_c[1])

    heap = [(heuristic(start_tri), 0.0, start_tri)]
    best = {start_tri: 0.0}
    came: dict[int, tuple[int, tuple[int, int]]] = {}
    expanded = 0

    while heap:
        _, cost, tri = heapq.heappop(heap)
        if cost > best.get(tri, math.inf):
            continue
        if tri == goal_tri:
            break
        expanded += 1
        if expanded > node_limit:
            return [], []

        cx, cy = mesh.centroid(tri)
        for other, portal in mesh.adjacent(tri):
            mx, my = _portal_midpoint(mesh, portal.key())
            ox, oy = mesh.centroid(other)
            step = math.hypot(mx - cx, my - cy) + math.hypot(ox - mx, oy - my)
            nxt = cost + step * cost_of(portal, net)
            if nxt < best.get(other, math.inf) - 1e-9:
                best[other] = nxt
                came[other] = (tri, portal.key())
                heapq.heappush(heap, (nxt + heuristic(other), nxt, other))

    if goal_tri not in best:
        return [], []

    triangles = [goal_tri]
    portals: list[tuple[int, int]] = []
    cursor = goal_tri
    while cursor != start_tri:
        previous, key = came[cursor]
        portals.append(key)
        triangles.append(previous)
        cursor = previous
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

    triangle_of: dict[tuple[float, float], int] = {}

    def locate(point: tuple[float, float]) -> int:
        cached = triangle_of.get(point)
        if cached is None:
            cached = mesh.triangle_at(point[0], point[1])
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
                mesh, locate(route.start), locate(route.goal), route.net, cost_of)
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
