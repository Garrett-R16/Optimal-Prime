"""What a strategy returns.

Deliberately independent of how the strategy found it: a grid router, a triangulation router
and a random baseline all produce the same three primitives, so their outputs are comparable
and interchangeable.

Those three primitives are also *exactly* what KiCad copper supports -- straight segments,
circular arcs, and vias. There is no spline or Bezier for copper. That constraint turns out
to be a gift rather than a limit: the taut path around obstacles inflated by the clearance
radius is provably made of straight tangent lines plus circular arcs on those inflation
circles and nothing else, so the optimal geometry and the expressible geometry coincide.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

__all__ = ["Segment", "Arc", "Via", "Solution", "Item"]


@dataclass(frozen=True, slots=True)
class Segment:
    net: int
    layer: str
    x1: int
    y1: int
    x2: int
    y2: int
    width_nm: int

    @property
    def length_nm(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

    @property
    def endpoints(self) -> tuple[tuple[int, int], tuple[int, int]]:
        return (self.x1, self.y1), (self.x2, self.y2)


@dataclass(frozen=True, slots=True)
class Arc:
    """A circular arc through three points, matching KiCad's own representation."""

    net: int
    layer: str
    x1: int
    y1: int
    xm: int
    ym: int
    x2: int
    y2: int
    width_nm: int

    @property
    def endpoints(self) -> tuple[tuple[int, int], tuple[int, int]]:
        return (self.x1, self.y1), (self.x2, self.y2)

    @property
    def length_nm(self) -> float:
        """True arc length, falling back to the chord for a degenerate (collinear) arc."""
        ax, ay = self.x1 - self.xm, self.y1 - self.ym
        bx, by = self.x2 - self.xm, self.y2 - self.ym
        cross = ax * by - ay * bx
        if abs(cross) < 1e-9:
            return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

        # Circumcentre of the three points.
        d = 2 * (self.x1 * (self.ym - self.y2)
                 + self.xm * (self.y2 - self.y1)
                 + self.x2 * (self.y1 - self.ym))
        if d == 0:
            return math.hypot(self.x2 - self.x1, self.y2 - self.y1)
        s1 = self.x1 ** 2 + self.y1 ** 2
        sm = self.xm ** 2 + self.ym ** 2
        s2 = self.x2 ** 2 + self.y2 ** 2
        cx = (s1 * (self.ym - self.y2) + sm * (self.y2 - self.y1) + s2 * (self.y1 - self.ym)) / d
        cy = (s1 * (self.x2 - self.xm) + sm * (self.x1 - self.x2) + s2 * (self.xm - self.x1)) / d
        radius = math.hypot(self.x1 - cx, self.y1 - cy)

        a0 = math.atan2(self.y1 - cy, self.x1 - cx)
        am = math.atan2(self.ym - cy, self.xm - cx)
        a2 = math.atan2(self.y2 - cy, self.x2 - cx)

        def norm(theta: float) -> float:
            return theta % (2 * math.pi)

        forward = norm(a2 - a0)
        mid_forward = norm(am - a0)
        sweep = forward if mid_forward <= forward else 2 * math.pi - forward
        return radius * sweep


@dataclass(frozen=True, slots=True)
class Via:
    """A through-hole via. MVP-01 freezes via type, so there is exactly one geometry."""

    net: int
    x: int
    y: int
    diameter_nm: int
    drill_nm: int
    layer_from: str
    layer_to: str


Item = Segment | Arc | Via


@dataclass
class Solution:
    """A strategy's answer, plus whatever it wants recorded about how it got there."""

    items: list[Item] = field(default_factory=list)
    #: Nets the strategy believes it fully routed. Claiming a net here does not make it so --
    #: the oracle decides -- but the discrepancy is itself worth measuring.
    routed_nets: set[int] = field(default_factory=set)
    #: Nets the strategy gave up on. Reporting failure is strictly better than emitting a
    #: violating board, and the scorer treats it that way.
    abandoned_nets: set[int] = field(default_factory=set)
    meta: dict = field(default_factory=dict)

    def add(self, item: Item) -> None:
        self.items.append(item)

    @property
    def segments(self) -> list[Segment]:
        return [i for i in self.items if isinstance(i, Segment)]

    @property
    def arcs(self) -> list[Arc]:
        return [i for i in self.items if isinstance(i, Arc)]

    @property
    def vias(self) -> list[Via]:
        return [i for i in self.items if isinstance(i, Via)]

    @property
    def wirelength_nm(self) -> float:
        """Total copper length. Arcs contribute true arc length, not chord length."""
        return sum(i.length_nm for i in self.items if not isinstance(i, Via))

    def stats(self) -> dict:
        return {
            "segments": len(self.segments),
            "arcs": len(self.arcs),
            "vias": len(self.vias),
            "wirelength_nm": self.wirelength_nm,
            "routed_nets": len(self.routed_nets),
            "abandoned_nets": len(self.abandoned_nets),
        }
