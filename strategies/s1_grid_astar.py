"""S1 -- random net order, greedy grid A*, no rip-up.

The point of S1 is not that it routes well. It is that running it across many seeds on the
same board measures how much of the routing problem is *net ordering and nothing else*: the
router is deterministic given an order, so the entire spread across seeds is attributable to
the order alone. That spread is experiment E2, and it is P1's exit criterion.

Everything here is deliberately the simplest thing that works. Uniform grid, octile moves,
first-come-first-served, no negotiation, no rip-up. S4 (PathFinder) in P2 is what this is
meant to lose to; if it does not, the harness is broken rather than the algorithm.

Three geometric details are not optional, and each one showed up as hundreds of DRC
violations before being fixed:

* **The grid pitch must account for diagonal moves.** Two parallel tracks in diagonally
  adjacent cells are only ``pitch/sqrt(2)`` apart, not ``pitch``. So the pitch is scaled by
  sqrt(2), which costs routing density and buys correctness.
* **A via is much larger than a cell.** A 1.6 mm via on a 0.75 mm grid overlaps its
  neighbours, so its whole footprint has to be claimed on every layer, not just its own cell.
* **Stub segments must start at the pad that actually owns the path's first cell.** Joining a
  path to whichever pad happened to be routed previously draws copper straight across the
  board.
"""

from __future__ import annotations

import heapq
import math
from random import Random

from arena.problem import Pad, Problem
from arena.solution import Segment, Solution, Via
from arena.strategy import Budget, register
from arena.units import GUARDBAND_NM

FREE = 0
BLOCKED = -1

#: In-layer moves: 4 orthogonal at cost 1, 4 diagonal at cost sqrt(2).
_MOVES = (
    (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
)


class Grid:
    """Uniform occupancy grid over the board, one plane per copper layer.

    Cell values: ``0`` free, ``-1`` blocked for everyone, ``n > 0`` usable only by net ``n``.
    A cell contested by two nets becomes blocked, which is conservative -- it can cost
    completion but never correctness.
    """

    def __init__(self, problem: Problem, via_cost_cells: float = 10.0) -> None:
        rules = problem.rules
        default = rules.default
        # Size the grid for the *widest* track and clearance any netclass demands, not the
        # Default class: a uniform grid has to hold for every net that will use it.
        self.track_nm = rules.max_track_width_nm
        self.clearance_nm = rules.max_clearance_nm

        # Diagonally adjacent parallel tracks are pitch/sqrt(2) apart, so the required
        # centre-to-centre spacing has to be scaled up by sqrt(2) to hold on the diagonal.
        required = self.track_nm + self.clearance_nm + GUARDBAND_NM
        self.pitch = max(int(required * math.sqrt(2)) + 1, 50_000)

        margin = rules.min_copper_edge_clearance_nm + self.track_nm // 2 + self.clearance_nm
        x0, y0, x1, y1 = problem.bbox_nm
        self.x0, self.y0 = x0 + margin, y0 + margin
        span_x, span_y = (x1 - margin) - self.x0, (y1 - margin) - self.y0
        self.nx = max(1, int(span_x // self.pitch) + 1)
        self.ny = max(1, int(span_y // self.pitch) + 1)
        self.layers = problem.copper_layers
        self.nl = len(self.layers)
        self.via_cost = via_cost_cells

        self.cells = [[FREE] * (self.nx * self.ny) for _ in range(self.nl)]
        self.pad_at: dict[tuple[int, int, int], Pad] = {}
        self._block_obstacles(problem)
        self._claim_pads(problem)

    # ------------------------------------------------------------------ construction

    def to_cell(self, x: int, y: int) -> tuple[int, int]:
        return (int((x - self.x0) // self.pitch), int((y - self.y0) // self.pitch))

    def to_nm(self, ix: int, iy: int) -> tuple[int, int]:
        return (self.x0 + ix * self.pitch + self.pitch // 2,
                self.y0 + iy * self.pitch + self.pitch // 2)

    def inside(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def _stamp(self, plane: int, cx: int, cy: int, radius_nm: int, value: int,
               contest: bool) -> None:
        cells = self.cells[plane]
        radius_cells = int(radius_nm // self.pitch) + 1
        ix0, iy0 = self.to_cell(cx, cy)
        for dy in range(-radius_cells, radius_cells + 1):
            iy = iy0 + dy
            if not 0 <= iy < self.ny:
                continue
            row = iy * self.nx
            for dx in range(-radius_cells, radius_cells + 1):
                ix = ix0 + dx
                if not 0 <= ix < self.nx:
                    continue
                px, py = self.to_nm(ix, iy)
                if math.hypot(px - cx, py - cy) > radius_nm:
                    continue
                index = row + ix
                current = cells[index]
                if current == BLOCKED:
                    continue
                if current == FREE or current == value:
                    cells[index] = value
                elif contest:
                    cells[index] = BLOCKED

    def _block_obstacles(self, problem: Problem) -> None:
        halo = self.clearance_nm + self.track_nm // 2
        for obstacle in problem.obstacles:
            for index, layer in enumerate(self.layers):
                if obstacle.layers and not any(
                        pattern in (layer, "*.Cu") for pattern in obstacle.layers):
                    continue
                self._stamp(index, obstacle.x, obstacle.y,
                            obstacle.radius_nm + halo, BLOCKED, contest=False)

    def _claim_pads(self, problem: Problem) -> None:
        """Claim pad footprints and their clearance halos, and record which pad owns which cell."""
        halo = self.clearance_nm + self.track_nm // 2

        for pad in problem.pads:
            extent = max(pad.radius_nm, self.pitch // 2)
            for plane in self.planes_for(pad):
                self._stamp(plane, pad.x, pad.y, extent + halo,
                            pad.net if pad.net else BLOCKED, contest=True)

        # A pad's own cell is always reachable by its own net, even if the halo pass
        # contested it -- otherwise dense placements become trivially unroutable.
        for pad in problem.pads:
            if not pad.net:
                continue
            ix, iy = self.to_cell(pad.x, pad.y)
            if not self.inside(ix, iy):
                continue
            for plane in self.planes_for(pad):
                node = (plane, ix, iy)
                if node in self.pad_at and self.pad_at[node].net != pad.net:
                    self.cells[plane][iy * self.nx + ix] = BLOCKED
                    continue
                self.cells[plane][iy * self.nx + ix] = pad.net
                self.pad_at[node] = pad

    def planes_for(self, pad: Pad) -> list[int]:
        if pad.drill_nm > 0:
            return list(range(self.nl))
        return [i for i, layer in enumerate(self.layers) if pad.on_layer(layer)]

    # ------------------------------------------------------------------ search

    def passable(self, plane: int, ix: int, iy: int, net: int) -> bool:
        value = self.cells[plane][iy * self.nx + ix]
        return value == FREE or value == net

    def via_possible(self, ix: int, iy: int, net: int) -> bool:
        """A through-hole via occupies every layer, so every layer must be available."""
        return all(self.passable(plane, ix, iy, net) for plane in range(self.nl))

    def search(self, sources: set[tuple[int, int, int]], targets: set[tuple[int, int, int]],
               net: int, node_limit: int = 2_000_000) -> list[tuple[int, int, int]] | None:
        """Multi-source, multi-target A* over (layer, x, y). Returns a cell path or None."""
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

        heap: list[tuple[float, float, tuple[int, int, int]]] = []
        best_g: dict[tuple[int, int, int], float] = {}
        parent: dict[tuple[int, int, int], tuple[int, int, int] | None] = {}

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
                if dx and dy:  # no corner-cutting between two blocked orthogonal neighbours
                    if not (self.passable(plane, ix + dx, iy, net)
                            and self.passable(plane, ix, iy + dy, net)):
                        continue
                neighbour = (plane, nx_, ny_)
                tentative = cost + step
                if tentative < best_g.get(neighbour, math.inf):
                    best_g[neighbour] = tentative
                    parent[neighbour] = node
                    heapq.heappush(heap, (tentative + heuristic(nx_, ny_), tentative, neighbour))

            if self.nl > 1 and self.via_possible(ix, iy, net):
                for other in range(self.nl):
                    if other == plane:
                        continue
                    neighbour = (other, ix, iy)
                    tentative = cost + self.via_cost
                    if tentative < best_g.get(neighbour, math.inf):
                        best_g[neighbour] = tentative
                        parent[neighbour] = node
                        heapq.heappush(heap,
                                       (tentative + heuristic(ix, iy), tentative, neighbour))
        return None

    def occupy(self, path: list[tuple[int, int, int]], net: int, width_nm: int) -> None:
        """Claim a routed path, widening the claim to the track's actual clearance envelope."""
        radius = width_nm // 2 + self.clearance_nm + GUARDBAND_NM
        for plane, ix, iy in path:
            cx, cy = self.to_nm(ix, iy)
            self._stamp(plane, cx, cy, radius, net, contest=False)

    def occupy_via(self, ix: int, iy: int, net: int, diameter_nm: int) -> None:
        """A via is far larger than a cell, so claim its footprint on every layer."""
        cx, cy = self.to_nm(ix, iy)
        radius = diameter_nm // 2 + self.clearance_nm + GUARDBAND_NM
        for plane in range(self.nl):
            self._stamp(plane, cx, cy, radius, net, contest=False)


def _path_to_items(grid: Grid, path: list[tuple[int, int, int]], net: int, width_nm: int,
                   via_d: int, via_drill: int) -> tuple[list, list[tuple[int, int]]]:
    """Turn a cell path into copper, merging collinear runs and inserting vias.

    Stub segments to a pad centre are added only where the path genuinely begins or ends on
    that pad's own cell -- never to some other pad that happens to be in the same net.
    """
    items: list = []
    via_sites: list[tuple[int, int]] = []
    if not path:
        return items, via_sites

    runs: list[tuple[int, list[tuple[int, int]]]] = []
    current_plane = path[0][0]
    current: list[tuple[int, int]] = []
    for plane, ix, iy in path:
        if plane != current_plane:
            runs.append((current_plane, current))
            current_plane, current = plane, []
        current.append(grid.to_nm(ix, iy))
    runs.append((current_plane, current))

    start_pad = grid.pad_at.get(path[0])
    end_pad = grid.pad_at.get(path[-1])

    for run_index, (plane, points) in enumerate(runs):
        layer = grid.layers[plane]
        polyline = list(points)
        if run_index == 0 and start_pad is not None:
            polyline.insert(0, (start_pad.x, start_pad.y))
        if run_index == len(runs) - 1 and end_pad is not None:
            polyline.append((end_pad.x, end_pad.y))

        simplified = [polyline[0]]
        for point in polyline[1:]:
            simplified.append(point)
            if len(simplified) >= 3:
                (ax, ay), (bx, by), (cx, cy) = simplified[-3:]
                if (bx - ax) * (cy - ay) == (by - ay) * (cx - ax):
                    del simplified[-2]

        for (ax, ay), (bx, by) in zip(simplified, simplified[1:]):
            if (ax, ay) != (bx, by):
                items.append(Segment(net=net, layer=layer, x1=ax, y1=ay, x2=bx, y2=by,
                                     width_nm=width_nm))

        if run_index + 1 < len(runs):
            vx, vy = points[-1]
            via_sites.append((vx, vy))
            items.append(Via(net=net, x=vx, y=vy, diameter_nm=via_d, drill_nm=via_drill,
                             layer_from=grid.layers[0], layer_to=grid.layers[-1]))
    return items, via_sites


@register("S1")
class RandomOrderGridAStar:
    """Route nets in a random order; within a net, grow a tree pad by pad."""

    def __init__(self, via_cost_cells: float = 10.0) -> None:
        self.via_cost_cells = via_cost_cells

    def route(self, problem: Problem, rng: Random, budget: Budget) -> Solution:
        grid = Grid(problem, via_cost_cells=self.via_cost_cells)
        solution = Solution(meta={
            "pitch_nm": grid.pitch,
            "grid": [grid.nl, grid.ny, grid.nx],
            "via_cost_cells": self.via_cost_cells,
        })

        nets = list(problem.routable_nets)
        rng.shuffle(nets)
        solution.meta["net_order"] = [net.code for net in nets]

        for net in nets:
            if budget.exhausted():
                solution.abandoned_nets.add(net.code)
                continue

            netclass = problem.rules.for_net(net.name)
            width = max(netclass.track_width_nm, problem.rules.min_track_width_nm)
            via_d = max(netclass.via_diameter_nm, problem.rules.min_via_diameter_nm)
            via_drill = netclass.via_drill_nm

            pad_nodes: list[set[tuple[int, int, int]]] = []
            for pad in net.pads:
                ix, iy = grid.to_cell(pad.x, pad.y)
                if not grid.inside(ix, iy):
                    pad_nodes.append(set())
                    continue
                pad_nodes.append({
                    (plane, ix, iy) for plane in grid.planes_for(pad)
                    if grid.cells[plane][iy * grid.nx + ix] != BLOCKED
                })

            reachable = [i for i, nodes in enumerate(pad_nodes) if nodes]
            if len(reachable) < 2:
                solution.abandoned_nets.add(net.code)
                continue

            connected: set[tuple[int, int, int]] = set(pad_nodes[reachable[0]])
            failed = False
            for index in reachable[1:]:
                path = grid.search(connected, pad_nodes[index], net.code)
                if path is None:
                    failed = True
                    break
                grid.occupy(path, net.code, width)
                items, via_sites = _path_to_items(grid, path, net.code, width, via_d, via_drill)
                for vx, vy in via_sites:
                    grid.occupy_via(*grid.to_cell(vx, vy), net.code, via_d)
                for item in items:
                    solution.add(item)
                connected.update(path)

            (solution.abandoned_nets if failed else solution.routed_nets).add(net.code)

        return solution
