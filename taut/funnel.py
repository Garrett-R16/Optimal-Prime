"""Embedding a route once its topology is fixed.

A route arrives here as a *sequence of portals* -- which side of every obstacle it passes, and
which slot it occupies in every shared doorway. That is its homotopy class, already decided.
Nothing here searches or chooses; it computes the shape the rubber band takes given that
class, which is a solved problem rather than a hard one.

Two steps.

**Funnel.** The classic linear-time shortest path through a sequence of portals, narrowed to
this route's slot in each. It yields the corners the path turns at, and each corner is an
obstacle vertex the band is wrapping.

**Tangents.** A band cannot actually turn a corner -- it wraps at its clearance distance, so
each corner becomes an arc of that radius about the vertex, joined to its neighbours by the
tangent lines between those arcs. Straight lines and circular arcs, which is exactly what a
``.kicad_pcb`` holds.

The difference from the tangent *graph* is the whole point of the rebuild. There, the search
chose a homotopy class implicitly, as a side effect of finding a shortest path, and two nets
could silently choose incompatible ones. Here the class is an input.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ["Gate", "Wrap", "funnel", "taut_through", "orient"]

_EPS = 1e-9


@dataclass(frozen=True, slots=True)
class Gate:
    """One doorway, narrowed to a single route's slot.

    ``left`` and ``right`` are the ends of the usable interval, oriented so that ``left`` is
    to the left of the direction of travel **in a y-down coordinate system** -- which is the
    one KiCad uses, so "left" here is left as drawn on screen. Getting this backwards does not
    fail loudly: the funnel simply wraps the far side of every doorway and returns a path that
    is legal but needlessly long. :func:`orient` decides it, so callers never have to. ``left_vertex`` and ``right_vertex`` say which
    obstacle corner each end was measured from, and ``left_radius``/``right_radius`` how far
    off it the band must stay -- which becomes the radius of the arc if it wraps there.
    """

    left: tuple[float, float]
    right: tuple[float, float]
    left_vertex: tuple[float, float]
    right_vertex: tuple[float, float]
    left_radius: float
    right_radius: float


@dataclass(frozen=True, slots=True)
class Wrap:
    """A corner the band turns at: a circle to ride, and which way round."""

    cx: float
    cy: float
    r: float
    ccw: bool


def orient(from_pt, to_pt, p, q):
    """Order two gate ends into (left, right) for travel from ``from_pt`` to ``to_pt``.

    Left is the end with a negative cross product against the direction of travel, which in a
    y-down system is the left-hand side.
    """
    dx, dy = to_pt[0] - from_pt[0], to_pt[1] - from_pt[1]
    cross_p = dx * (p[1] - from_pt[1]) - dy * (p[0] - from_pt[0])
    return (p, q) if cross_p < 0 else (q, p)


def _area2(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])


def funnel(start: tuple[float, float], goal: tuple[float, float],
           gates: list[Gate]) -> list[Wrap]:
    """Shortest path through a sequence of gates, as the corners it wraps.

    The simple stupid funnel algorithm: carry a left and a right bound from an apex, tighten
    them as each gate is read, and whenever they cross, the tighter one was a real corner --
    emit it and restart the funnel there.
    """
    left_pts = [start] + [g.left for g in gates] + [goal]
    right_pts = [start] + [g.right for g in gates] + [goal]
    # side, vertex and radius for each index; the endpoints wrap nothing
    # Wrap direction. A band turning at its *left* bound curves clockwise about that corner
    # and one turning at its right bound curves counter-clockwise -- in KiCad's y-down frame,
    # which is where these coordinates live. Getting it backwards is not subtle: every arc
    # takes the long way round its circle, sweeping ~2*pi instead of a fraction of one.
    meta_left = [None] + [(g.left_vertex, g.left_radius, False) for g in gates] + [None]
    meta_right = [None] + [(g.right_vertex, g.right_radius, True) for g in gates] + [None]

    wraps: list[Wrap] = []
    apex = start
    apex_index = 0
    left_index = right_index = 0
    portal_left, portal_right = left_pts[0], right_pts[0]

    index = 1
    while index < len(left_pts):
        left, right = left_pts[index], right_pts[index]

        # tighten the right bound
        if _area2(apex, portal_right, right) <= _EPS:
            if apex == portal_right or _area2(apex, portal_left, right) > _EPS:
                portal_right, right_index = right, index
            else:
                meta = meta_left[left_index]
                if meta is not None:
                    (vx, vy), radius, ccw = meta
                    wraps.append(Wrap(vx, vy, radius, ccw))
                apex, apex_index = portal_left, left_index
                portal_left = portal_right = apex
                left_index = right_index = apex_index
                index = apex_index + 1
                continue

        # tighten the left bound
        if _area2(apex, portal_left, left) >= -_EPS:
            if apex == portal_left or _area2(apex, portal_right, left) < -_EPS:
                portal_left, left_index = left, index
            else:
                meta = meta_right[right_index]
                if meta is not None:
                    (vx, vy), radius, ccw = meta
                    wraps.append(Wrap(vx, vy, radius, ccw))
                apex, apex_index = portal_right, right_index
                portal_left = portal_right = apex
                left_index = right_index = apex_index
                index = apex_index + 1
                continue

        index += 1

    return wraps


# --------------------------------------------------------------------------- tangents

@dataclass(frozen=True, slots=True)
class Line:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def length(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)


@dataclass(frozen=True, slots=True)
class Curve:
    cx: float
    cy: float
    r: float
    start_angle: float
    end_angle: float
    ccw: bool

    @property
    def sweep(self) -> float:
        delta = (self.end_angle - self.start_angle) % (2 * math.pi)
        return delta if self.ccw else -((2 * math.pi - delta) % (2 * math.pi))

    @property
    def length(self) -> float:
        return self.r * abs(self.sweep)

    def at(self, t: float) -> tuple[float, float]:
        theta = self.start_angle + self.sweep * t
        return self.cx + self.r * math.cos(theta), self.cy + self.r * math.sin(theta)


def _tangent_point(px: float, py: float, wrap: Wrap, leaving: bool) -> float:
    """Angle on ``wrap``'s circle where the tangent from an outside point touches."""
    dx, dy = px - wrap.cx, py - wrap.cy
    span = math.hypot(dx, dy)
    if span <= wrap.r + _EPS:
        return math.atan2(dy, dx)
    base = math.atan2(dy, dx)
    offset = math.acos(max(-1.0, min(1.0, wrap.r / span)))
    # Which of the two tangents depends on the direction the band goes round.
    sign = 1.0 if (wrap.ccw != leaving) else -1.0
    return base + sign * offset


def _outer_tangent(a: Wrap, b: Wrap) -> tuple[float, float] | None:
    """Angles on ``a`` and ``b`` of the tangent that respects both wrap directions."""
    dx, dy = b.cx - a.cx, b.cy - a.cy
    gap = math.hypot(dx, dy)
    if gap < _EPS:
        return None
    base = math.atan2(dy, dx)

    if a.ccw == b.ccw:
        delta = a.r - b.r
        if abs(delta) > gap:
            return None
        alpha = math.acos(max(-1.0, min(1.0, delta / gap)))
        theta = base + (alpha if a.ccw else -alpha)
        return theta, theta

    total = a.r + b.r
    if total > gap:
        return None
    alpha = math.acos(max(-1.0, min(1.0, total / gap)))
    theta = base + (alpha if a.ccw else -alpha)
    return theta, theta + math.pi


def taut_through(start: tuple[float, float], goal: tuple[float, float],
                 wraps: list[Wrap]) -> list[Line | Curve]:
    """The taut band through a fixed sequence of wraps: tangent lines joined by arcs."""
    if not wraps:
        return [Line(start[0], start[1], goal[0], goal[1])]

    entry: list[float] = [0.0] * len(wraps)
    exit_: list[float] = [0.0] * len(wraps)

    entry[0] = _tangent_point(start[0], start[1], wraps[0], leaving=False)
    for i in range(len(wraps) - 1):
        pair = _outer_tangent(wraps[i], wraps[i + 1])
        if pair is None:
            # Circles too close to admit a tangent; fall back to the line of centres so the
            # path stays continuous rather than vanishing.
            direction = math.atan2(wraps[i + 1].cy - wraps[i].cy,
                                   wraps[i + 1].cx - wraps[i].cx)
            exit_[i], entry[i + 1] = direction, direction + math.pi
        else:
            exit_[i], entry[i + 1] = pair
    exit_[-1] = _tangent_point(goal[0], goal[1], wraps[-1], leaving=True)

    out: list[Line | Curve] = []
    cursor = start
    for index, wrap in enumerate(wraps):
        # The side a band wraps is fixed by the topology; the *direction* it sweeps is then
        # whichever of the two is consistent with arriving and leaving tangentially, which is
        # always the short way round. Asserting the direction from the side instead sends
        # every arc the long way -- 6.2 radians where 0.3 was wanted.
        span = (exit_[index] - entry[index]) % (2 * math.pi)
        wrap = Wrap(wrap.cx, wrap.cy, wrap.r, span <= math.pi)
        ax = wrap.cx + wrap.r * math.cos(entry[index])
        ay = wrap.cy + wrap.r * math.sin(entry[index])
        if math.hypot(ax - cursor[0], ay - cursor[1]) > _EPS:
            out.append(Line(cursor[0], cursor[1], ax, ay))
        arc = Curve(wrap.cx, wrap.cy, wrap.r, entry[index], exit_[index], wrap.ccw)
        if abs(arc.sweep) > 1e-9:
            out.append(arc)
        cursor = (wrap.cx + wrap.r * math.cos(exit_[index]),
                  wrap.cy + wrap.r * math.sin(exit_[index]))

    if math.hypot(goal[0] - cursor[0], goal[1] - cursor[1]) > _EPS:
        out.append(Line(cursor[0], cursor[1], goal[0], goal[1]))
    return out
