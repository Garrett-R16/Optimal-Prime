"""Metrics, computed from the oracle's output.

Two design decisions here matter more than the arithmetic.

**Baseline subtraction.** Real boards arrive with pre-existing violations -- a courtyard
overlap, a silkscreen clash, a zone sliver -- that placement freezes in and we are not
allowed to fix. Charging those to the router would make Clean Pass unreachable on perfectly
good instances and would quietly rank strategies by which boards they were given. So every
board is scored *bare* once, and only violations that were not already there count.

**Filtering happens here, not in the project file.** The oracle is always run with
``--severity-all``; what counts as an error is :data:`arena.rules.ROUTING_RULES`, a decision
recorded in our code and versioned with it, rather than whatever severity map a board
happened to ship with.
"""

from __future__ import annotations

import math
from collections import Counter
from dataclasses import asdict, dataclass, field

from .oracle import DrcResult, Violation
from .problem import Problem
from .rules import ROUTING_RULES
from .solution import Solution
from .units import nm_to_mm

__all__ = ["Score", "SCORE_VERSION", "score_solution", "new_violations",
           "steiner_lower_bound_nm", "routability"]

#: Bump when a metric definition changes. Old runs are never silently rescored.
SCORE_VERSION = 1

#: Euclidean Steiner ratio. A Steiner minimal tree is never shorter than sqrt(3)/2 of the
#: minimum spanning tree, so (sqrt(3)/2) * MST is a valid lower bound on any routing of the
#: net. Cheap, obstacle-oblivious, and therefore a bound on a bound -- which is fine, because
#: WL_ratio only has to be *consistent* across strategies, not tight. P4 replaces this with
#: exact GeoSteiner trees.
STEINER_RATIO = math.sqrt(3) / 2


@dataclass(frozen=True, slots=True)
class Score:
    """One run's metrics. ``cp`` is the headline; everything else is diagnosis."""

    cp: int                      # Clean Pass: 1 or 0. Binary and non-negotiable.
    rout: float                  # fraction of connections completed
    drv: int                     # new error-severity violations attributable to us
    wl_mm: float
    wl_ratio: float              # wirelength over the Steiner lower bound
    vias: int
    time_s: float
    unconnected: int
    drv_by_type: dict[str, int] = field(default_factory=dict)
    preexisting_drv: int = 0
    score_version: int = SCORE_VERSION

    def j_lite(self, via_weight: float) -> float:
        """Scalarised objective. Reported as a sweep over ``via_weight``, never as one number.

        The length/via tradeoff is the weight nobody agrees on, so collapsing it to a single
        scalar hides the disagreement instead of reporting it (MVP-PLAN section 2.4).
        """
        return self.wl_ratio + via_weight * self.vias

    def to_dict(self) -> dict:
        return asdict(self)


def new_violations(current: DrcResult, baseline: DrcResult | None) -> list[Violation]:
    """Violations present now that were not present on the bare board.

    Matched as a multiset on ``(type, rounded positions)`` so that a board with three
    pre-existing courtyard overlaps still flags a fourth.
    """
    routing_only = [v for v in current.violations if v.type in ROUTING_RULES]
    if baseline is None:
        return routing_only

    remaining = Counter(v.key() for v in baseline.violations if v.type in ROUTING_RULES)
    fresh: list[Violation] = []
    for violation in routing_only:
        key = violation.key()
        if remaining.get(key, 0) > 0:
            remaining[key] -= 1
        else:
            fresh.append(violation)
    return fresh


def routability(current: DrcResult, baseline: DrcResult | None) -> float:
    """Fraction of the board's connections that got made.

    Defined against the bare board's own unconnected count, which is the number of
    connections that needed making. That makes it comparable across boards of wildly
    different net counts without needing to model the ratsnest ourselves.
    """
    if baseline is None or baseline.unconnected_count == 0:
        return 1.0 if current.unconnected_count == 0 else 0.0
    done = baseline.unconnected_count - current.unconnected_count
    return max(0.0, min(1.0, done / baseline.unconnected_count))


def steiner_lower_bound_nm(problem: Problem) -> float:
    """Sum over nets of ``(sqrt(3)/2) * MST``, a valid lower bound on total routed length.

    Obstacle-oblivious and layer-oblivious, so it is a bound on a bound. Its job is to make
    wirelength comparable across boards, not to be tight.
    """
    total = 0.0
    for net in problem.routable_nets:
        points = [(p.x, p.y) for p in net.pads]
        if len(points) < 2:
            continue
        # Prim's algorithm; net sizes here are small enough that O(n^2) is irrelevant.
        unvisited = set(range(1, len(points)))
        best = {i: math.dist(points[0], points[i]) for i in unvisited}
        mst = 0.0
        while unvisited:
            nearest = min(unvisited, key=lambda i: best[i])
            mst += best[nearest]
            unvisited.discard(nearest)
            for i in unvisited:
                d = math.dist(points[nearest], points[i])
                if d < best[i]:
                    best[i] = d
        total += mst
    return total * STEINER_RATIO


def score_solution(problem: Problem, solution: Solution, current: DrcResult,
                   baseline: DrcResult | None, time_s: float) -> Score:
    """Turn one oracle result into the run's metrics."""
    fresh = new_violations(current, baseline)
    unconnected = current.unconnected_count

    wl_nm = solution.wirelength_nm
    bound_nm = steiner_lower_bound_nm(problem)
    wl_ratio = (wl_nm / bound_nm) if bound_nm > 0 else float("inf") if wl_nm else 1.0

    counts: dict[str, int] = {}
    for violation in fresh:
        counts[violation.type] = counts.get(violation.type, 0) + 1

    preexisting = 0
    if baseline is not None:
        preexisting = sum(1 for v in baseline.violations if v.type in ROUTING_RULES)

    return Score(
        cp=int(not fresh and unconnected == 0),
        rout=routability(current, baseline),
        drv=len(fresh),
        wl_mm=nm_to_mm(wl_nm),
        wl_ratio=wl_ratio,
        vias=len(solution.vias),
        time_s=time_s,
        unconnected=unconnected,
        drv_by_type=counts,
        preexisting_drv=preexisting,
    )
