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


import os
_MODE = os.environ.get("MODE", "mid")
_PORTAL_PENALTY = float(os.environ.get("TP", 0.0))


def _portal_midpoint(mesh: Mesh, key: tuple[int, int]) -> tuple[float, float]:
    a, b = mesh.points[key[0]], mesh.points[key[1]]
    return (float((a[0] + b[0]) / 2), float((a[1] + b[1]) / 2))


def _closest_on(mesh, key, px, py):
    a, b = mesh.points[key[0]], mesh.points[key[1]]
    ax, ay = float(a[0]), float(a[1])
    dx, dy = float(b[0]) - ax, float(b[1]) - ay
    dd = dx * dx + dy * dy
    if dd <= 0.0:
        return ax, ay
    t = ((px - ax) * dx + (py - ay) * dy) / dd
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return ax + dx * t, ay + dy * t


def _aim(mesh, key, px, py, gx, gy):
    a, b = mesh.points[key[0]], mesh.points[key[1]]
    ax, ay = float(a[0]), float(a[1])
    dx, dy = float(b[0]) - ax, float(b[1]) - ay
    lo, hi = 0.0, 1.0
    for _ in range(28):
        m1 = lo + (hi - lo) / 3.0
        m2 = hi - (hi - lo) / 3.0
        x1, y1 = ax + dx * m1, ay + dy * m1
        x2, y2 = ax + dx * m2, ay + dy * m2
        f1 = math.hypot(x1 - px, y1 - py) + math.hypot(x1 - gx, y1 - gy)
        f2 = math.hypot(x2 - px, y2 - py) + math.hypot(x2 - gx, y2 - gy)
        if f1 < f2:
            hi = m2
        else:
            lo = m1
    t = (lo + hi) / 2.0
    return ax + dx * t, ay + dy * t


def _search(mesh: Mesh, start_tri: int, goal_tri: int, net: int,
            start: tuple[float, float], goal: tuple[float, float],
            cost_of, node_limit: int = 400_000) -> tuple[list[int], list[tuple[int, int]]]:
    if start_tri < 0 or goal_tri < 0:
        return [], []
    if start_tri == goal_tri:
        return [start_tri], []

    gx, gy = goal
    cache: dict[tuple[int, int], tuple[float, float]] = {}

    def anchor(key, px, py):
        if _MODE == "mid":
            point = cache.get(key)
            if point is None:
                point = cache[key] = _portal_midpoint(mesh, key)
            return point
        if _MODE == "near":
            return _closest_on(mesh, key, px, py)
        if _MODE == "goal":
            point = cache.get(key)
            if point is None:
                point = cache[key] = _closest_on(mesh, key, gx, gy)
            return point
        return _aim(mesh, key, px, py, gx, gy)

    def heuristic(px, py):
        return math.hypot(px - gx, py - gy)

    heap = []
    best: dict[tuple, float] = {}
    came: dict[tuple, tuple | None] = {}
    where: dict[tuple, tuple[float, float]] = {}

    sx, sy = start
    for other, portal in mesh.adjacent(start_tri):
        key = portal.key()
        ax, ay = anchor(key, sx, sy)
        step = math.hypot(ax - sx, ay - sy) + _PORTAL_PENALTY
        cost = step * cost_of(portal, net)
        rest = 0.0
        if other == goal_tri:
            cost += heuristic(ax, ay)
        else:
            rest = heuristic(ax, ay)
        state = (other, key)
        if cost < best.get(state, math.inf):
            best[state] = cost
            came[state] = None
            where[state] = (ax, ay)
            heapq.heappush(heap, (cost + rest, cost, state))

    expanded = 0
    final = None
    while heap:
        _, cost, state = heapq.heappop(heap)
        if cost > best.get(state, math.inf):
            continue
        tri, entry = state
        if tri == goal_tri:
            final = state
            break
        expanded += 1
        if expanded > node_limit:
            return [], []
        px, py = where[state]
        for other, portal in mesh.adjacent(tri):
            key = portal.key()
            if key == entry:
                continue
            ax, ay = anchor(key, px, py)
            step = math.hypot(ax - px, ay - py) + _PORTAL_PENALTY
            nxt = cost + step * cost_of(portal, net)
            rest = 0.0
            if other == goal_tri:
                nxt += heuristic(ax, ay)
            else:
                rest = heuristic(ax, ay)
            successor = (other, key)
            if nxt < best.get(successor, math.inf) - 1e-9:
                best[successor] = nxt
                came[successor] = state
                where[successor] = (ax, ay)
                heapq.heappush(heap, (nxt + rest, nxt, successor))

    if final is None:
        return [], []
    triangles = []
    portals = []
    cursor = final
    while cursor is not None:
        tri, key = cursor
        triangles.append(tri)
        portals.append(key)
        cursor = came[cursor]
    triangles.append(start_tri)
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
