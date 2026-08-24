"""Exact-enough distance predicates for copper geometry.

This is the inner-loop query, not a DRC engine. It answers one question -- *how far is the
nearest copper of another net* -- millions of times during a search. KiCad's DRC answers a
different question ("is this finished board legal?") over ~30 rule types, and remains the
sole authority for scoring. Calling it per query is not slow, it is impossible: on a 34-net
board a single S1 run issues ~300,000 clearance queries, and at 1.47 s per ``kicad-cli``
invocation that is 124 hours for one board and one seed.

**Conservatism is the contract.** Everything here may report a distance *smaller* than the
truth, never larger. A router that believes copper is closer than it is loses completions; a
router that believes it is further apart emits violations it never saw. ``tests/`` pins the
arrow in that one direction against a brute-force reference and against KiCad itself.

Distances are computed in float64 on integer-nanometre inputs. Board coordinates run to
~1e8 nm and float64 carries ~15-16 significant digits, so absolute error is ~1e-7 nm --
eleven orders of magnitude below the smallest design rule anyone writes. "Exact" below means
that, not algebraic exactness, which for arc-arc distance would require unbounded-degree
algebraic numbers.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ["Arc", "point_point", "point_segment", "segment_segment", "point_arc",
           "segment_arc", "arc_arc", "segments_intersect", "TAU"]

TAU = 2.0 * math.pi
_EPS = 1e-9


def point_point(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(bx - ax, by - ay)


# --------------------------------------------------------------------------- segments

def point_segment(px: float, py: float,
                  ax: float, ay: float, bx: float, by: float) -> float:
    """Distance from a point to a segment."""
    dx, dy = bx - ax, by - ay
    span = dx * dx + dy * dy
    if span <= _EPS:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / span
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def segments_intersect(ax: float, ay: float, bx: float, by: float,
                       cx: float, cy: float, dx: float, dy: float) -> bool:
    """Whether two closed segments share at least one point."""
    def orient(px, py, qx, qy, rx, ry) -> float:
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)

    d1 = orient(cx, cy, dx, dy, ax, ay)
    d2 = orient(cx, cy, dx, dy, bx, by)
    d3 = orient(ax, ay, bx, by, cx, cy)
    d4 = orient(ax, ay, bx, by, dx, dy)

    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return True

    def on_segment(px, py, qx, qy, rx, ry) -> bool:
        return (min(px, qx) - _EPS <= rx <= max(px, qx) + _EPS
                and min(py, qy) - _EPS <= ry <= max(py, qy) + _EPS)

    if abs(d1) <= _EPS and on_segment(cx, cy, dx, dy, ax, ay):
        return True
    if abs(d2) <= _EPS and on_segment(cx, cy, dx, dy, bx, by):
        return True
    if abs(d3) <= _EPS and on_segment(ax, ay, bx, by, cx, cy):
        return True
    if abs(d4) <= _EPS and on_segment(ax, ay, bx, by, dx, dy):
        return True
    return False


def segment_segment(ax: float, ay: float, bx: float, by: float,
                    cx: float, cy: float, dx: float, dy: float) -> float:
    """Minimum distance between two segments; 0 if they touch or cross."""
    if segments_intersect(ax, ay, bx, by, cx, cy, dx, dy):
        return 0.0
    return min(
        point_segment(ax, ay, cx, cy, dx, dy),
        point_segment(bx, by, cx, cy, dx, dy),
        point_segment(cx, cy, ax, ay, bx, by),
        point_segment(dx, dy, ax, ay, bx, by),
    )


# --------------------------------------------------------------------------- arcs

@dataclass(frozen=True, slots=True)
class Arc:
    """A circular arc, stored the way distance queries want it.

    KiCad stores arcs as three points (start, mid, end). ``from_three_points`` converts, and
    falls back to a degenerate zero-radius arc when the points are collinear -- callers must
    treat ``degenerate`` arcs as the segment start->end instead.
    """

    cx: float
    cy: float
    r: float
    start_angle: float   # radians, in [0, TAU)
    sweep: float         # radians, signed: positive is counter-clockwise in math convention
    x1: float = 0.0
    y1: float = 0.0
    x2: float = 0.0
    y2: float = 0.0
    degenerate: bool = False

    @classmethod
    def from_three_points(cls, x1: float, y1: float, xm: float, ym: float,
                          x2: float, y2: float) -> "Arc":
        d = 2.0 * (x1 * (ym - y2) + xm * (y2 - y1) + x2 * (y1 - ym))
        if abs(d) < _EPS:
            return cls(cx=x1, cy=y1, r=0.0, start_angle=0.0, sweep=0.0,
                       x1=x1, y1=y1, x2=x2, y2=y2, degenerate=True)

        s1 = x1 * x1 + y1 * y1
        sm = xm * xm + ym * ym
        s2 = x2 * x2 + y2 * y2
        cx = (s1 * (ym - y2) + sm * (y2 - y1) + s2 * (y1 - ym)) / d
        cy = (s1 * (x2 - xm) + sm * (x1 - x2) + s2 * (xm - x1)) / d
        r = math.hypot(x1 - cx, y1 - cy)

        a0 = math.atan2(y1 - cy, x1 - cx) % TAU
        am = math.atan2(ym - cy, xm - cx) % TAU
        a2 = math.atan2(y2 - cy, x2 - cx) % TAU

        ccw_total = (a2 - a0) % TAU
        ccw_mid = (am - a0) % TAU
        sweep = ccw_total if ccw_mid <= ccw_total else -((TAU - ccw_total) % TAU)

        return cls(cx=cx, cy=cy, r=r, start_angle=a0, sweep=sweep,
                   x1=x1, y1=y1, x2=x2, y2=y2)

    @property
    def length(self) -> float:
        if self.degenerate:
            return math.hypot(self.x2 - self.x1, self.y2 - self.y1)
        return self.r * abs(self.sweep)

    def contains_angle(self, theta: float) -> bool:
        """Whether a direction from the centre falls inside the arc's angular span."""
        if self.degenerate:
            return False
        delta = (theta - self.start_angle) % TAU
        if self.sweep >= 0.0:
            return delta <= self.sweep + _EPS
        return (TAU - delta) % TAU <= -self.sweep + _EPS

    def point_at_angle(self, theta: float) -> tuple[float, float]:
        return self.cx + self.r * math.cos(theta), self.cy + self.r * math.sin(theta)


def point_arc(px: float, py: float, arc: Arc) -> float:
    """Distance from a point to an arc."""
    if arc.degenerate:
        return point_segment(px, py, arc.x1, arc.y1, arc.x2, arc.y2)

    dx, dy = px - arc.cx, py - arc.cy
    dist = math.hypot(dx, dy)
    if dist > _EPS and arc.contains_angle(math.atan2(dy, dx)):
        return abs(dist - arc.r)
    return min(math.hypot(px - arc.x1, py - arc.y1),
               math.hypot(px - arc.x2, py - arc.y2))


def segment_arc(ax: float, ay: float, bx: float, by: float, arc: Arc) -> float:
    """Minimum distance between a segment and an arc.

    Candidate enumeration rather than iterative refinement, so the result is deterministic
    and has no convergence tolerance to tune. The candidates are: each arc endpoint against
    the segment, each segment endpoint against the arc, the crossing case, and the
    perpendicular case where the closest circle point projects onto the segment's interior.
    """
    if arc.degenerate:
        return segment_segment(ax, ay, bx, by, arc.x1, arc.y1, arc.x2, arc.y2)

    best = min(
        point_segment(arc.x1, arc.y1, ax, ay, bx, by),
        point_segment(arc.x2, arc.y2, ax, ay, bx, by),
        point_arc(ax, ay, arc),
        point_arc(bx, by, arc),
    )
    if best <= 0.0:
        return 0.0

    dx, dy = bx - ax, by - ay
    span = dx * dx + dy * dy
    if span <= _EPS:
        return best

    # Where the segment crosses the circle, and whether those crossings lie on the arc.
    fx, fy = ax - arc.cx, ay - arc.cy
    b_coef = 2.0 * (fx * dx + fy * dy)
    c_coef = fx * fx + fy * fy - arc.r * arc.r
    disc = b_coef * b_coef - 4.0 * span * c_coef
    if disc >= 0.0:
        root = math.sqrt(disc)
        for t in ((-b_coef - root) / (2.0 * span), (-b_coef + root) / (2.0 * span)):
            if 0.0 <= t <= 1.0:
                px, py = ax + t * dx, ay + t * dy
                if arc.contains_angle(math.atan2(py - arc.cy, px - arc.cx)):
                    return 0.0

    # Perpendicular case: the point on the segment nearest the centre. Distance from any
    # segment point P to the full circle is |dist(P, centre) - r|, minimised at that foot.
    t = ((arc.cx - ax) * dx + (arc.cy - ay) * dy) / span
    if 0.0 <= t <= 1.0:
        px, py = ax + t * dx, ay + t * dy
        vx, vy = px - arc.cx, py - arc.cy
        dist = math.hypot(vx, vy)
        if dist > _EPS and arc.contains_angle(math.atan2(vy, vx)):
            best = min(best, abs(dist - arc.r))

    return best


def arc_arc(first: Arc, second: Arc) -> float:
    """Minimum distance between two arcs."""
    if first.degenerate:
        return segment_arc(first.x1, first.y1, first.x2, first.y2, second)
    if second.degenerate:
        return segment_arc(second.x1, second.y1, second.x2, second.y2, first)

    best = min(
        point_arc(first.x1, first.y1, second),
        point_arc(first.x2, first.y2, second),
        point_arc(second.x1, second.y1, first),
        point_arc(second.x2, second.y2, first),
    )
    if best <= 0.0:
        return 0.0

    gap = math.hypot(second.cx - first.cx, second.cy - first.cy)

    # Concentric: the circles never meet, and the extreme points lie in every direction.
    if gap <= _EPS:
        radial = abs(first.r - second.r)
        for theta in (first.start_angle, first.start_angle + first.sweep):
            if second.contains_angle(theta % TAU):
                best = min(best, radial)
        return best

    # Circle-circle intersections; a shared point counts only if both arcs span it.
    if abs(first.r - second.r) - _EPS <= gap <= first.r + second.r + _EPS:
        a = (first.r * first.r - second.r * second.r + gap * gap) / (2.0 * gap)
        h_sq = first.r * first.r - a * a
        if h_sq >= 0.0:
            h = math.sqrt(h_sq)
            ux, uy = (second.cx - first.cx) / gap, (second.cy - first.cy) / gap
            mx, my = first.cx + a * ux, first.cy + a * uy
            for sign in (1.0, -1.0):
                px, py = mx - sign * h * uy, my + sign * h * ux
                if (first.contains_angle(math.atan2(py - first.cy, px - first.cx))
                        and second.contains_angle(math.atan2(py - second.cy,
                                                             px - second.cx))):
                    return 0.0

    # Along the line of centres, where two disjoint circles are closest or furthest.
    ux, uy = (second.cx - first.cx) / gap, (second.cy - first.cy) / gap
    for sign_a in (1.0, -1.0):
        theta_a = math.atan2(sign_a * uy, sign_a * ux) % TAU
        if not first.contains_angle(theta_a):
            continue
        pax, pay = first.point_at_angle(theta_a)
        for sign_b in (1.0, -1.0):
            theta_b = math.atan2(sign_b * uy, sign_b * ux) % TAU
            if not second.contains_angle(theta_b):
                continue
            pbx, pby = second.point_at_angle(theta_b)
            best = min(best, math.hypot(pbx - pax, pby - pay))

    return best
