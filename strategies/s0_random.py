"""S0 -- uniform-random geometry. The floor.

Random waypoints, connected in order, obstacles and design rules ignored entirely. It will
score terribly, and that is the whole point: without it, a Clean Pass of 0.4 is an
uninterpretable number. Is 0.4 an achievement or an embarrassment? It depends on whether
random scores 0.02 or 0.38, and almost nobody publishing routing results reports that line.

S0 is also a useful canary for the harness itself. It emits geometry that is definitely
illegal, so if the oracle ever passes an S0 board, something upstream is broken.
"""

from __future__ import annotations

from random import Random

from arena.problem import Problem
from arena.solution import Segment, Solution
from arena.strategy import Budget, register


@register("S0")
class UniformRandom:
    """Connect each net's pads with straight copper, wandering through random waypoints."""

    def __init__(self, waypoints: int = 1) -> None:
        self.waypoints = waypoints

    def route(self, problem: Problem, rng: Random, budget: Budget) -> Solution:
        solution = Solution(meta={"waypoints": self.waypoints})
        x0, y0, x1, y1 = problem.bbox_nm
        layers = problem.copper_layers

        for net in problem.routable_nets:
            width = problem.rules.for_net(net.name).track_width_nm
            pads = list(net.pads)
            rng.shuffle(pads)

            for start, end in zip(pads, pads[1:]):
                layer = rng.choice(layers)
                path = [(start.x, start.y)]
                for _ in range(self.waypoints):
                    path.append((rng.randint(x0, x1), rng.randint(y0, y1)))
                path.append((end.x, end.y))

                for (ax, ay), (bx, by) in zip(path, path[1:]):
                    if (ax, ay) == (bx, by):
                        continue
                    solution.add(Segment(net=net.code, layer=layer,
                                         x1=ax, y1=ay, x2=bx, y2=by, width_nm=width))
            solution.routed_nets.add(net.code)

        return solution
