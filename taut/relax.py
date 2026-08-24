"""Relaxing every band at once, instead of routing them one at a time.

A board's copper is a set of rubber bands, each pinned between two pads, each with a width,
none able to pass through another. Their resting shape is a *joint* property: move one and
the rest re-settle. Routing them one at a time asks each in isolation, and the first through
any gap sits in the middle of it because nothing else is there yet.

The one thing that matters is that they move **together**: every band is re-solved against
the *previous* positions of all the others, and only then are the new positions adopted.
Updating one at a time is what makes the first band's answer permanent and everyone else's
answer a detour around it. Simultaneous updates let two bands contending for the same space
both give way, which is the behaviour that was missing.

An earlier version also annealed the band-to-band clearance from soft to solid, on the
argument that the taut solver treats other copper as walls and walls do not yield. The
argument is sound but the effect is not: on `ecc83-pp` the finished route is identical at
0.30, 0.60 and 1.00 hardness, so the softening is gone rather than kept on a plausible story.
Bands turn out to have room to go *around* each other far more often than they need to pass
*through*.

Which **layer** a band sits on is part of the same settlement. Fixing it beforehand -- by
giving each connection whichever layer was shortest for it alone -- puts every band on the
same face, and the relaxation is then good enough to make that work while being materially
longer than spreading out. So each step re-tries every permitted layer and keeps the shortest
result, which lets a crowded face shed bands to an empty one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .obstacles import Obstacle
from .tangent import NoPathFound, TautPath, violated_obstacles

__all__ = ["RelaxReport", "relax"]


@dataclass
class RelaxReport:
    steps: int = 0
    moved: int = 0
    stuck: int = 0
    converged: bool = False
    length_by_step: list[float] = field(default_factory=list)
    overlaps_by_step: list[int] = field(default_factory=list)


def relax(items, solve_one, path_obstacles, steps: int = 5,
          verbose: bool = False, budget=None) -> RelaxReport:
    """Settle a set of bands into a joint arrangement.

    ``items`` are objects carrying ``path``, ``layer``, ``net_code``, ``halo`` and
    ``layer_options``. ``solve_one(item, layer, blockers)`` returns a fresh taut path or
    raises :class:`~taut.tangent.NoPathFound`. ``path_obstacles(path, radius, net)`` turns a
    finished path into keep-outs.

    Bands whose new path cannot be found at a given hardness keep the one they had; the
    schedule ends at full clearance, and whatever is still overlapping then is the caller's
    problem to settle.

    Only bands that are actually in conflict are re-solved. A band sitting in open copper with
    nothing near it has already found its taut shape and re-deriving it every step costs a
    full tangent solve to arrive at the same answer -- which on a 66-connection board is most
    of the runtime and none of the benefit.
    """
    report = RelaxReport()

    for step in range(steps):
        if budget is not None and budget():
            break
        report.steps += 1
        proposals: dict[int, TautPath] = {}
        contended = _conflicted(items, path_obstacles)

        for item in items:
            if item.path is None or item.layer is None or id(item) not in contended:
                continue

            best = None
            for layer in (item.layer_options or [item.layer]):
                # Every band sees where the others *were*, not where they are becoming.
                # That is what lets two of them yield to each other in one iteration.
                blockers = [
                    obstacle
                    for other in items
                    if other is not item
                    and other.path is not None
                    and other.layer == layer
                    and other.net_code != item.net_code
                    for obstacle in path_obstacles(other.path, item.halo,
                                                   other.net_code)
                ]
                try:
                    candidate = solve_one(item, layer, blockers)
                except NoPathFound:
                    continue
                if best is None or candidate.length < best[0]:
                    best = (candidate.length, layer, candidate)

            if best is None:
                report.stuck += 1
            else:
                proposals[id(item)] = (best[1], best[2])

        for item in items:
            fresh = proposals.get(id(item))
            if fresh is not None:
                layer, path = fresh
                if (item.path is None or abs(path.length - item.path.length) > 1.0
                        or layer != item.layer):
                    report.moved += 1
                item.layer = layer
                item.path = path

        total = sum(i.path.length for i in items if i.path is not None)
        report.length_by_step.append(round(total / 1e6, 2))
        report.overlaps_by_step.append(_count_overlaps(items, path_obstacles))
        if report.overlaps_by_step[-1] == 0:
            break

        if verbose:
            print(f"    relax step {step + 1}/{steps}  length {total / 1e6:8.1f}mm  "
                  f"overlaps {report.overlaps_by_step[-1]}")

    report.converged = bool(report.overlaps_by_step) and report.overlaps_by_step[-1] == 0
    return report


def _conflicted(items, path_obstacles) -> set[int]:
    """Bands too close to another right now, and the specific bands they are close to.

    Both sides of a conflict are included, because the whole point is that they give way to
    each other rather than one of them yielding. Bands elsewhere on the board are left alone:
    pulling in everything on the layer costs a tangent solve apiece to reproduce answers that
    were already correct.
    """
    guilty: set[int] = set()
    for item in items:
        if item.path is None or item.layer is None:
            continue
        for other in items:
            if (other is item or other.path is None or other.layer != item.layer
                    or other.net_code == item.net_code):
                continue
            blockers = path_obstacles(other.path, item.halo, other.net_code)
            if violated_obstacles(item.path, blockers):
                guilty.add(id(item))
                guilty.add(id(other))
    return guilty


def _count_overlaps(items, path_obstacles) -> int:
    """How many bands are still too close to another at full clearance."""
    guilty = 0
    for item in items:
        if item.path is None or item.layer is None:
            continue
        blockers = [
            obstacle
            for other in items
            if other is not item
            and other.path is not None
            and other.layer == item.layer
            and other.net_code != item.net_code
            for obstacle in path_obstacles(other.path, item.halo, other.net_code)
        ]
        if blockers and violated_obstacles(item.path, blockers):
            guilty += 1
    return guilty
