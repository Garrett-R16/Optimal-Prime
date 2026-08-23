"""S4 -- PathFinder negotiated congestion.

The idea that makes this different from S1 is one line: **other nets' copper is a cost, not a
wall.** S1 routes first-come-first-served and whoever arrives first owns the space, so the
outcome is decided by net order -- which E2 measured at a median 50% swing in violation
count, and 140% on the worst board. PathFinder lets nets share a resource, then charges an
escalating price until they stop wanting to, so the order matters progressively less each
iteration.

Two costs, from the original formulation:

* **present congestion** ``p_n`` -- how over-subscribed a node is *right now*, scaled by a
  factor that grows every iteration. Early on, sharing is nearly free and nets find their
  natural paths; later it becomes unaffordable and they separate.
* **historical congestion** ``h_n`` -- accumulated over iterations for nodes that stayed
  contested. This is the memory that stops two nets swapping the same conflict back and
  forth forever.

Node cost is ``(1 + h_n) * (1 + pres_factor * overuse_n)``.

**DRC feedback.** Every few iterations the partial board is scored by KiCad itself and real
violation locations are added to the history term. That is affordable here and nowhere else:
PathFinder runs tens of outer iterations, not the ~300,000 inner queries a single A* sweep
issues. It is what lets the router learn from the actual rule engine rather than only from
our own approximation of one rule.
"""

from __future__ import annotations

import heapq
import math
import tempfile
from pathlib import Path
from random import Random

from arena import oracle
from arena.emit import write_solution
from arena.problem import Pad, Problem
from arena.solution import Segment, Solution, Via
from arena.strategy import Budget, register
from arena.units import mm_to_nm

from .s1_grid_astar import BLOCKED, Grid, _MOVES, _path_to_items

Node = tuple[int, int, int]


class CongestionGrid(Grid):
    """A static-obstacle grid plus per-node congestion state.

    Inherits pad claiming, obstacle rasterisation and pitch selection from S1's grid, then
    departs in the one way that matters: routed copper is never written into ``cells``.
    Pads and obstacles stay hard blocks; tracks become priced resources.
    """

    def __init__(self, problem: Problem, via_cost_cells: float = 10.0) -> None:
        super().__init__(problem, via_cost_cells=via_cost_cells)
        self.usage: dict[Node, set[int]] = {}
        self.history: dict[Node, float] = {}
        self.pres_factor = 0.5

        # The negotiated resource is exclusive use of a track's *clearance envelope*, not of
        # a single cell centre. Pricing bare cells lets two nets sit in adjacent cells with
        # zero recorded contention while breaking clearance -- which is precisely how the
        # first version of S4 managed to lose to S1.
        reach = int(math.ceil((self.track_nm / 2 + self.clearance_nm) / self.pitch))
        self._envelope = tuple(
            (dx, dy)
            for dx in range(-reach, reach + 1)
            for dy in range(-reach, reach + 1)
            if math.hypot(dx, dy) <= reach + 1e-9
        )

    # ------------------------------------------------------------------ occupancy

    def _envelope_nodes(self, path: list[Node]):
        for plane, ix, iy in path:
            for dx, dy in self._envelope:
                nx_, ny_ = ix + dx, iy + dy
                if self.inside(nx_, ny_):
                    yield (plane, nx_, ny_)

    def claim(self, path: list[Node], net: int) -> None:
        for node in self._envelope_nodes(path):
            self.usage.setdefault(node, set()).add(net)

    def release(self, path: list[Node], net: int) -> None:
        for node in self._envelope_nodes(path):
            holders = self.usage.get(node)
            if holders:
                holders.discard(net)
                if not holders:
                    del self.usage[node]

    def overuse(self, node: Node, net: int) -> int:
        """How many *other* nets already want this node."""
        holders = self.usage.get(node)
        if not holders:
            return 0
        return len(holders) - (1 if net in holders else 0)

    def contested(self) -> list[Node]:
        return [node for node, holders in self.usage.items() if len(holders) > 1]

    def node_cost(self, node: Node, net: int) -> float:
        return ((1.0 + self.history.get(node, 0.0))
                * (1.0 + self.pres_factor * self.overuse(node, net)))

    def bump_history(self, nodes, amount: float) -> None:
        for node in nodes:
            self.history[node] = self.history.get(node, 0.0) + amount

    def node_at(self, x: int, y: int) -> tuple[int, int] | None:
        cell = self.to_cell(x, y)
        return cell if self.inside(*cell) else None

    # ------------------------------------------------------------------ search

    def negotiate(self, sources: set[Node], targets: set[Node], net: int,
                  node_limit: int = 1_500_000) -> list[Node] | None:
        """A* where other nets' copper is priced rather than forbidden."""
        if not sources or not targets:
            return None
        overlap = sources & targets
        if overlap:
            return [next(iter(overlap))]

        goals = [(ix, iy) for _, ix, iy in targets]

        def heuristic(ix: int, iy: int) -> float:
            best = math.inf
            for gx, gy in goals:
                dx, dy = abs(ix - gx), abs(iy - gy)
                octile = (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
                if octile < best:
                    best = octile
            return best

        heap: list[tuple[float, float, Node]] = []
        best_g: dict[Node, float] = {}
        parent: dict[Node, Node | None] = {}

        for node in sources:
            best_g[node] = 0.0
            parent[node] = None
            heapq.heappush(heap, (heuristic(node[1], node[2]), 0.0, node))

        expanded = 0
        while heap:
            _, cost, node = heapq.heappop(heap)
            if cost > best_g.get(node, math.inf):
                continue
            if node in targets:
                path = [node]
                while parent[path[-1]] is not None:
                    path.append(parent[path[-1]])
                return list(reversed(path))

            expanded += 1
            if expanded > node_limit:
                return None

            plane, ix, iy = node
            for dx, dy, step in _MOVES:
                nx_, ny_ = ix + dx, iy + dy
                if not self.inside(nx_, ny_) or not self.passable(plane, nx_, ny_, net):
                    continue
                if dx and dy:
                    if not (self.passable(plane, ix + dx, iy, net)
                            and self.passable(plane, ix, iy + dy, net)):
                        continue
                neighbour = (plane, nx_, ny_)
                tentative = cost + step * self.node_cost(neighbour, net)
                if tentative < best_g.get(neighbour, math.inf):
                    best_g[neighbour] = tentative
                    parent[neighbour] = node
                    heapq.heappush(heap, (tentative + heuristic(nx_, ny_), tentative,
                                          neighbour))

            if self.nl > 1 and self.via_possible(ix, iy, net):
                for other in range(self.nl):
                    if other == plane:
                        continue
                    neighbour = (other, ix, iy)
                    tentative = cost + self.via_cost * self.node_cost(neighbour, net)
                    if tentative < best_g.get(neighbour, math.inf):
                        best_g[neighbour] = tentative
                        parent[neighbour] = node
                        heapq.heappush(heap, (tentative + heuristic(ix, iy), tentative,
                                              neighbour))
        return None


@register("S4")
class PathFinder:
    """Rip-up and reroute every net each iteration, with an escalating price on sharing."""

    def __init__(self, iterations: int = 24, pres_growth: float = 1.7,
                 history_gain: float = 0.6, via_cost_cells: float = 10.0,
                 drc_feedback: bool = True, drc_every: int = 6,
                 drc_history_gain: float = 3.0) -> None:
        self.iterations = iterations
        self.pres_growth = pres_growth
        self.history_gain = history_gain
        self.via_cost_cells = via_cost_cells
        self.drc_feedback = drc_feedback
        self.drc_every = drc_every
        self.drc_history_gain = drc_history_gain

    # ------------------------------------------------------------------ helpers

    @staticmethod
    def _pad_nodes(grid: CongestionGrid, pads) -> list[set[Node]]:
        out: list[set[Node]] = []
        for pad in pads:
            ix, iy = grid.to_cell(pad.x, pad.y)
            if not grid.inside(ix, iy):
                out.append(set())
                continue
            out.append({(plane, ix, iy) for plane in grid.planes_for(pad)
                        if grid.cells[plane][iy * grid.nx + ix] != BLOCKED})
        return out

    def _build_solution(self, problem: Problem, grid: CongestionGrid,
                        routes: dict[int, list[list[Node]]]) -> Solution:
        solution = Solution(meta={"pitch_nm": grid.pitch,
                                  "grid": [grid.nl, grid.ny, grid.nx],
                                  "pres_factor": grid.pres_factor})
        for net in problem.routable_nets:
            paths = routes.get(net.code)
            if not paths:
                solution.abandoned_nets.add(net.code)
                continue
            netclass = problem.rules.for_net(net.name)
            width = max(netclass.track_width_nm, problem.rules.min_track_width_nm)
            via_d = max(netclass.via_diameter_nm, problem.rules.min_via_diameter_nm)
            for path in paths:
                items, _ = _path_to_items(grid, path, net.code, width, via_d,
                                          netclass.via_drill_nm)
                for item in items:
                    solution.add(item)
            solution.routed_nets.add(net.code)
        return solution

    def _drc_hotspots(self, problem: Problem, grid: CongestionGrid,
                      solution: Solution, seed: int) -> list[Node]:
        """Grid nodes at the locations KiCad actually complained about.

        This is the outer loop, so it can afford the engine: tens of calls per board, not
        one per node expansion.
        """
        with tempfile.TemporaryDirectory() as work:
            board = Path(work) / "iter.kicad_pcb"
            try:
                write_solution(problem, solution, board, "S4", seed)
                result = oracle.run_drc(board)
            except (oracle.OracleError, OSError):
                return []

        hotspots: list[Node] = []
        for violation in result.violations:
            if violation.type not in ("clearance", "shorting_items", "tracks_crossing",
                                      "copper_edge_clearance"):
                continue
            for x_mm, y_mm in violation.positions:
                cell = grid.node_at(mm_to_nm(x_mm), mm_to_nm(y_mm))
                if cell is None:
                    continue
                ix, iy = cell
                for plane in range(grid.nl):
                    for dx in (-1, 0, 1):
                        for dy in (-1, 0, 1):
                            if grid.inside(ix + dx, iy + dy):
                                hotspots.append((plane, ix + dx, iy + dy))
        return hotspots

    # ------------------------------------------------------------------ main

    def route(self, problem: Problem, rng: Random, budget: Budget) -> Solution:
        grid = CongestionGrid(problem, via_cost_cells=self.via_cost_cells)
        nets = [n for n in problem.routable_nets]
        rng.shuffle(nets)

        pad_nodes = {net.code: self._pad_nodes(grid, net.pads) for net in nets}
        routes: dict[int, list[list[Node]]] = {}
        history: list[dict] = []

        for iteration in range(self.iterations):
            if budget.exhausted():
                break

            for net in nets:
                for path in routes.pop(net.code, []):
                    grid.release(path, net.code)

                reachable = [nodes for nodes in pad_nodes[net.code] if nodes]
                if len(reachable) < 2:
                    continue

                connected: set[Node] = set(reachable[0])
                found: list[list[Node]] = []
                for targets in reachable[1:]:
                    if budget.exhausted():
                        break
                    path = grid.negotiate(connected, targets, net.code)
                    if path is None:
                        continue
                    grid.claim(path, net.code)
                    connected.update(path)
                    found.append(path)
                if found:
                    routes[net.code] = found

            contested = grid.contested()
            history.append({"iteration": iteration, "contested": len(contested),
                            "pres_factor": round(grid.pres_factor, 3)})

            if not contested:
                break

            grid.bump_history(contested, self.history_gain)
            grid.pres_factor *= self.pres_growth

            if (self.drc_feedback and self.drc_every
                    and (iteration + 1) % self.drc_every == 0
                    and not budget.exhausted()):
                partial = self._build_solution(problem, grid, routes)
                hotspots = self._drc_hotspots(problem, grid, partial, iteration)
                if hotspots:
                    grid.bump_history(hotspots, self.drc_history_gain)
                    history[-1]["drc_hotspots"] = len(hotspots)

        solution = self._build_solution(problem, grid, routes)
        solution.meta["iterations"] = history
        solution.meta["converged"] = bool(history) and history[-1]["contested"] == 0
        return solution
