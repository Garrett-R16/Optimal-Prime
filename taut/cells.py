"""Cells: what a triangle looks like once wires are running through it.

Every mechanism this router has needed and lacked comes down to one missing fact: inside a
triangle, a committed wire is a *chord*, and a chord splits the triangle in two. A later wire
entering on one side cannot leave on the other without crossing -- so the search must know
the sides. Six attempts approximated this from outside (fan-turn bans, walls, pinned seats);
this module states it exactly, the way SURF's region graph does.

The construction: a triangle's boundary is a cycle. Everything on it -- corners, doorway
spans, the points where committed wires cross -- lives at a parameter ``t`` in ``[0, 3)``,
one unit per edge. Each committed wire through the triangle is a chord between two boundary
parameters. Committed wires are mutually planar (each was routed against the ones before it),
so the chords do not cross each other, and the cells they cut are read off directly: two
boundary intervals lie in the same cell exactly when they are on the same side of every
chord.

A terminal's stub is a chord too, from the pad's own corner to its exit -- and the two
intervals flanking that corner then land in different cells. Those are the fan *sectors* of
the reference systems, not modelled specially but falling out of the same arithmetic as
everything else.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

__all__ = ["Chord", "Interval", "TriangleCells"]

_EPS = 1e-7


@dataclass(frozen=True, slots=True)
class Chord:
    """One committed wire's passage through the triangle, as boundary parameters."""

    t1: float
    t2: float
    #: whose chord this is, so its own wire is never walled in by itself
    key: int = -1


@dataclass(frozen=True, slots=True)
class Interval:
    """A run of boundary between consecutive marked points, and the cell it belongs to."""

    t_from: float
    t_to: float
    cell: int

    @property
    def mid(self) -> float:
        return (self.t_from + self.t_to) / 2.0


@dataclass
class TriangleCells:
    """The triangle's boundary, cut by chords into intervals, grouped into cells.

    ``intervals`` walk the boundary once, in parameter order. Two intervals share a ``cell``
    id exactly when a wire may run from one to the other without crossing any chord.
    """

    chords: list[Chord] = field(default_factory=list)
    intervals: list[Interval] = field(default_factory=list)

    @classmethod
    def build(cls, chords: list[Chord]) -> "TriangleCells":
        cuts = sorted({0.0, 1.0, 2.0} | {c.t1 % 3.0 for c in chords}
                      | {c.t2 % 3.0 for c in chords})
        out = cls(chords=list(chords))

        signatures: dict[tuple, int] = {}
        spans = list(zip(cuts, cuts[1:] + [cuts[0] + 3.0]))
        for t_from, t_to in spans:
            mid = (t_from + t_to) / 2.0 % 3.0
            signature = tuple(_inside(chord, mid) for chord in chords)
            cell = signatures.setdefault(signature, len(signatures))
            out.intervals.append(Interval(t_from, t_to % 3.0, cell))
        return out

    def cell_at(self, t: float) -> int:
        """The cell of the boundary point at parameter ``t`` (nudged off exact cuts)."""
        t = t % 3.0
        for interval in self.intervals:
            lo, hi = interval.t_from, interval.t_to
            if hi < lo:
                hi += 3.0
            here = t if t >= lo else t + 3.0
            if lo - _EPS <= here <= hi + _EPS:
                return interval.cell
        return self.intervals[0].cell if self.intervals else 0

    def cells_at_corner(self, corner: int) -> list[int]:
        """The sectors flanking a corner: distinct when a stub chord starts there."""
        t = float(corner % 3)
        before = self.cell_at((t - _EPS * 4) % 3.0)
        after = self.cell_at((t + _EPS * 4) % 3.0)
        return [after] if after == before else [after, before]

    def reachable(self, t_a: float, t_b: float) -> bool:
        """May a wire run from boundary point a to boundary point b without crossing?"""
        return self.cell_at(t_a) == self.cell_at(t_b)


def _inside(chord: Chord, t: float) -> bool:
    """Which side of the chord a boundary parameter lies on.

    The chord's endpoints split the boundary cycle into two arcs; ``t`` is on one of them.
    Points exactly at an endpoint belong to neither side cleanly and are nudged by the
    caller before asking.
    """
    lo, hi = sorted((chord.t1 % 3.0, chord.t2 % 3.0))
    return lo < t < hi


def edge_parameter(edge: int, fraction: float) -> float:
    """Boundary parameter of a point ``fraction`` of the way along edge ``edge``."""
    return (edge % 3) + min(max(fraction, 0.0), 1.0)


def point_parameter(corners, x: float, y: float) -> float:
    """Boundary parameter of the boundary point nearest (x, y).

    ``corners`` are the triangle's three vertices in order; edge ``i`` runs from corner
    ``i`` to corner ``i + 1``.
    """
    best = (math.inf, 0.0)
    for edge in range(3):
        ax, ay = corners[edge]
        bx, by = corners[(edge + 1) % 3]
        dx, dy = bx - ax, by - ay
        span = dx * dx + dy * dy
        f = 0.0 if span < 1e-18 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / span))
        d = math.hypot(x - ax - f * dx, y - ay - f * dy)
        if d < best[0]:
            best = (d, edge + f)
    return best[1] % 3.0
