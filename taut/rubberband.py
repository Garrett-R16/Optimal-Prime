"""The rubber-band equivalent: pulling a wire taut through a fixed topology.

This is the embedding step done the way Leiserson & Maley's result and Dayan's thesis do it,
rather than the way that seemed obvious. The difference is one idea, and it is the whole
reason the obvious way produced paths two and a half times too long.

**A wire's position in a doorway is never stored. Only its rank is.**

When several wires cross the same portal they are kept in order across it, and nothing else.
The offset a wire must keep from an obstacle is then *recomputed at every obstacle it wraps*,
as the cumulative clearance of every wire lying between it and that obstacle:

    r = spacing(me, next) + spacing(next, next-next) + ... + spacing(last, the obstacle)

So the same wire, in the same bundle, stands a different distance off at each corner it
touches -- because the set of wires between it and each corner is different. Assigning fixed
slots up front cannot express that, which is why a bundle seated that way spreads correctly in
one gap and wastes space in the next.

The taut path itself is not found by sweeping forward through the doorways. It is found by
recursion on the largest violation:

    1. Draw the chord from one end of the segment to the other.
    2. Of every obstacle the chord passes too close to, or on the wrong side of, take the
       worst.
    3. Wrap it -- that fixes one arc.
    4. Recurse on the two halves either side of it.

Pulling a string taut, in other words: it snags first on whatever it is most wrong about, and
each snag splits the problem. A wire crossing k doorways costs O(k log k) typically and O(k^2)
at worst, and wires are independent of each other, which is what makes this cheap.

Two measures of "wrong", and they are deliberately not on the same scale. Where the doorway
straddles the chord, the wire merely passes too close: ``d = required - actual``, always less
than the requirement. Where it does not, the wire is on the *wrong side* of the obstacle
entirely and must be pulled across it: ``d = actual + required``, always more. So being on the
wrong side always outranks being too close, and gross errors are fixed before fine ones.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

__all__ = ["Crossing", "Arc", "Segment", "rubberband", "spacing_between"]

_EPS = 1e-9


# --------------------------------------------------------------------------- inputs

@dataclass(frozen=True, slots=True)
class Wire:
    """What the spacing sum needs to know about a wire sharing a doorway."""

    key: int
    net: int
    half_width: float
    clearance: float


def spacing_between(a: Wire, b: Wire) -> float:
    """Centre-to-centre distance two wires must keep from each other."""
    if a.net == b.net:
        return 0.0
    return a.half_width + b.half_width + max(a.clearance, b.clearance)


@dataclass(frozen=True, slots=True)
class Crossing:
    """One doorway on a wire's route, and where this wire ranks in it.

    ``order`` runs from the ``a`` end to the ``b`` end. ``mine`` indexes into it. The wire's
    coordinate on the portal is *derived* from that rank, never stored independently -- it is
    used for winding tests only, never for a clearance.
    """

    ax: float
    ay: float
    bx: float
    by: float
    order: tuple[Wire, ...]
    mine: int
    #: How much of the doorway the wires either side of this one have left it, once they have
    #: been pushed apart far enough to clear each other. Set from real geometry and re-set
    #: every pass; ``None`` before there is any geometry to read, when the rank stack below
    #: stands in for it.
    room_a: float | None = None
    room_b: float | None = None
    #: copper radius carried by each end, for a shape the mesh describes by its centre
    ra: float = 0.0
    rb: float = 0.0
    constraint: bool = False

    @property
    def me(self) -> Wire:
        return self.order[self.mine]

    def offset_from(self, toward_a: bool) -> float:
        """Cumulative clearance between this wire and the doorway end it is measured to.

        Walks *towards* that end, summing the gap to each wire on the way and finally the
        wire's own clearance from the obstacle. Walking the other way yields the complementary
        sum and a radius that is wrong by the width of the rest of the bundle.
        """
        given = self.room_a if toward_a else self.room_b
        if given is not None:
            return given

        skin = self.ra if toward_a else self.rb
        if self.constraint:
            # A doorway that is itself a piece of copper -- a pad edge, the board rim. The
            # wire stands off it by its own clearance and no more; there is no stack of other
            # wires between it and a boundary it is running along.
            return skin + min(self.me.half_width + self.me.clearance, self._span() / 2.0)

        total = 0.0
        step = -1 if toward_a else 1
        index = self.mine
        while True:
            nxt = index + step
            if nxt < 0 or nxt >= len(self.order):
                break
            total += spacing_between(self.order[index], self.order[nxt])
            index = nxt
        total += self.order[index].half_width + self.order[index].clearance
        room = self._span() - self.ra - self.rb
        return skin + (min(total, room - 1.0) if room > 1.0 else total)

    def _span(self) -> float:
        return math.hypot(self.bx - self.ax, self.by - self.ay)

    def point(self) -> tuple[float, float]:
        """A nominal coordinate for this wire on the doorway, for winding tests only."""
        span = self._span()
        if span < _EPS:
            return self.ax, self.ay
        offset = min(self.offset_from(toward_a=True), span)
        ux, uy = (self.bx - self.ax) / span, (self.by - self.ay) / span
        return self.ax + ux * offset, self.ay + uy * offset

    def ends(self):
        yield (self.ax, self.ay), True
        yield (self.bx, self.by), False


# --------------------------------------------------------------------------- output

@dataclass
class Arc:
    """A corner the wire wraps: a circle about an obstacle, and which way round."""

    cx: float
    cy: float
    r: float
    ccw: bool
    #: tangent entry and exit, filled in once the neighbours are known
    x0: float = 0.0
    y0: float = 0.0
    x1: float = 0.0
    y1: float = 0.0
    first: int = 0          # index of the first crossing in this contact run
    last: int = 0

    @property
    def start_angle(self) -> float:
        return math.atan2(self.y0 - self.cy, self.x0 - self.cx)

    @property
    def end_angle(self) -> float:
        return math.atan2(self.y1 - self.cy, self.x1 - self.cx)

    @property
    def sweep(self) -> float:
        delta = (self.end_angle - self.start_angle) % (2 * math.pi)
        return delta if self.ccw else -((2 * math.pi - delta) % (2 * math.pi))

    @property
    def length(self) -> float:
        return self.r * abs(self.sweep)


@dataclass(frozen=True, slots=True)
class Segment:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def length(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)


# --------------------------------------------------------------------------- geometry

def _wind(ax, ay, bx, by, cx, cy) -> float:
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def _point_to_arc(px: float, py: float, arc: Arc, leaving: bool):
    """Tangent from a point to a circle, in vectors.

    The C original works in slope-intercept form and apologises for the degenerate cases that
    produces. Vectors have none of them: the tangent is the centre-to-point direction rotated
    by ``acos(r/d)``, and the sign of that rotation is the wrap direction.
    """
    dx, dy = px - arc.cx, py - arc.cy
    span = math.hypot(dx, dy)
    if span <= arc.r + _EPS:
        return None
    base = math.atan2(dy, dx)
    offset = math.acos(max(-1.0, min(1.0, arc.r / span)))
    sign = -1.0 if (arc.ccw == leaving) else 1.0
    theta = base + sign * offset
    return arc.cx + arc.r * math.cos(theta), arc.cy + arc.r * math.sin(theta)


def _arc_to_arc(a: Arc, b: Arc):
    """The bitangent between two circles that respects both wrap directions."""
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
        theta = base + (alpha if not a.ccw else -alpha)
        return ((a.cx + a.r * math.cos(theta), a.cy + a.r * math.sin(theta)),
                (b.cx + b.r * math.cos(theta), b.cy + b.r * math.sin(theta)))

    total = a.r + b.r
    if total > gap:
        return None
    alpha = math.acos(max(-1.0, min(1.0, total / gap)))
    theta = base + (alpha if not a.ccw else -alpha)
    opposite = theta + math.pi
    return ((a.cx + a.r * math.cos(theta), a.cy + a.r * math.sin(theta)),
            (b.cx + b.r * math.cos(opposite), b.cy + b.r * math.sin(opposite)))


def _neighbours(crossings, index, start, goal, reach: int = 3):
    """The path either side of a doorway, at increasing distance from it.

    The last test below asks whether the path really does pass around an obstacle, by asking
    whether what comes before and what comes after lie on opposite sides of it. Immediate
    neighbours are often both on the same side simply because they are close to each other, so
    the question is put again further out before it is answered no.
    """
    out = []
    for step in range(1, reach + 1):
        before = crossings[index - step].point() if index - step >= 0 else start
        after = crossings[index + step].point() if index + step < len(crossings) else goal
        out.append((before, after))
    return out


def _wrong_side(vx, vy, other, x0, y0, x1, y1, required, pairs):
    """How wrong the chord is about an obstacle it passes clean to one side of.

    Two things have to be true before this counts. The obstacle must lie *between* the chord
    and the rest of its doorway -- otherwise the wire reaches the doorway without going near
    it, and wrapping it would invent a detour. And the path either side of the doorway must
    genuinely pass around it, which is checked by asking whether the neighbours straddle the
    ray from the chord out through the obstacle.

    Without both, every doorway the chord happens to miss becomes a candidate, and the
    recursion decorates a perfectly good chord with arcs around corners it was never near.
    """
    foot = _foot_on_segment(vx, vy, x0, y0, x1, y1)
    if foot is None:
        # The obstacle is off the end of the chord rather than beside it. Measure from the
        # nearer end, and take the reference direction across that line instead.
        foot = ((x0, y0) if (vx - x0) ** 2 + (vy - y0) ** 2 < (vx - x1) ** 2 + (vy - y1) ** 2
                else (x1, y1))
        mx, my = -(foot[1] - vy), foot[0] - vx
    else:
        mx, my = x1 - x0, y1 - y0

    inside = mx * (foot[1] - vy) - my * (foot[0] - vx)
    outside = mx * (other[1] - vy) - my * (other[0] - vx)
    if outside == 0.0 or (inside > 0) == (outside > 0):
        return None

    gap = math.hypot(vx - foot[0], vy - foot[1])
    if gap < _EPS:
        return None
    ux, uy = (vx - foot[0]) / gap, (vy - foot[1]) / gap
    tip = (foot[0] + ux * (required + gap), foot[1] + uy * (required + gap))
    for before, after in pairs:
        ahead = _wind(foot[0], foot[1], tip[0], tip[1], before[0], before[1])
        behind = _wind(foot[0], foot[1], tip[0], tip[1], after[0], after[1])
        if ahead == 0.0 or behind == 0.0 or (ahead > 0) != (behind > 0):
            break
    else:
        return None

    return gap + required


def _segments_cross(ax, ay, bx, by, cx, cy, dx, dy) -> bool:
    d1 = _wind(cx, cy, dx, dy, ax, ay)
    d2 = _wind(cx, cy, dx, dy, bx, by)
    d3 = _wind(ax, ay, bx, by, cx, cy)
    d4 = _wind(ax, ay, bx, by, dx, dy)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _distance_to(px, py, x0, y0, x1, y1) -> float:
    foot = _foot_on_segment(px, py, x0, y0, x1, y1)
    if foot is not None:
        return math.hypot(px - foot[0], py - foot[1])
    return min(math.hypot(px - x0, py - y0), math.hypot(px - x1, py - y1))


def _foot_on_segment(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    span = dx * dx + dy * dy
    if span < _EPS:
        return None
    t = ((px - x0) * dx + (py - y0) * dy) / span
    if t < 0.0 or t > 1.0:
        return None
    return x0 + t * dx, y0 + t * dy


# --------------------------------------------------------------------------- the recursion

@dataclass
class _Candidate:
    violation: float
    index: int
    cx: float
    cy: float
    r: float
    ccw: bool


def _candidates(crossings: list[Crossing], lo: int, hi: int,
                x0: float, y0: float, x1: float, y1: float,
                start: tuple[float, float], goal: tuple[float, float]) -> list[_Candidate]:
    """Every obstacle in this stretch that the chord is wrong about, and by how much."""
    out: list[_Candidate] = []

    for index in range(lo, hi):
        crossing = crossings[index]
        here = crossing.point()
        # The neighbours are the ones on the *path*, not the ends of whatever sub-chord this
        # level of the recursion happens to be looking at. Substituting the segment's own
        # endpoints at each boundary reverses the turn at the first and last doorway of every
        # sub-range, which is most of them once the recursion is a few levels deep.
        before = crossings[index - 1].point() if index > 0 else start
        after = crossings[index + 1].point() if index + 1 < len(crossings) else goal

        wa = _wind(x0, y0, x1, y1, crossing.ax, crossing.ay)
        wb = _wind(x0, y0, x1, y1, crossing.bx, crossing.by)
        if wa == 0.0 and wb == 0.0:
            continue
        straddles = wa != 0.0 and wb != 0.0 and (wa > 0) != (wb > 0)

        for (vx, vy), toward_a in crossing.ends():
            required = crossing.offset_from(toward_a)
            if required <= 0.0:
                continue

            other = (crossing.bx, crossing.by) if toward_a else (crossing.ax, crossing.ay)

            if straddles:
                # The chord passes through the doorway, so the only question is whether it
                # leaves this end enough room. The measure is always less than the
                # requirement, which is what keeps it below every wrong-side case.
                foot = _foot_on_segment(vx, vy, x0, y0, x1, y1)
                if foot is None:
                    continue
                actual = math.hypot(vx - foot[0], vy - foot[1])
                if actual >= required:
                    continue
                violation = required - actual
            else:
                measure = _wrong_side(vx, vy, other, x0, y0, x1, y1, required,
                                      _neighbours(crossings, index, start, goal))
                if measure is None:
                    continue
                violation = measure

            # Which way the wire goes round comes from the turn the *path* makes at this
            # doorway, not from which side of the chord the obstacle fell. The chord is what
            # is being corrected; it cannot also be the authority on the answer.
            turn = _wind(before[0], before[1], here[0], here[1], vx, vy)
            out.append(_Candidate(violation, index, vx, vy, required, turn > 0))

    return out


def _contact_run(crossings: list[Crossing], index: int, lo: int, hi: int,
                 cx: float, cy: float) -> tuple[int, int]:
    """How many consecutive doorways share this obstacle -- they make one arc, not several."""
    first = last = index
    while first - 1 >= lo and _touches(crossings[first - 1], cx, cy):
        first -= 1
    while last + 1 < hi and _touches(crossings[last + 1], cx, cy):
        last += 1
    return first, last


def _touches(crossing: Crossing, cx: float, cy: float) -> bool:
    return (math.hypot(crossing.ax - cx, crossing.ay - cy) < 1.0
            or math.hypot(crossing.bx - cx, crossing.by - cy) < 1.0)


def rubberband(start: tuple[float, float], goal: tuple[float, float],
               crossings: list[Crossing], max_arcs: int = 400) -> list[Arc]:
    """Pull one wire taut through its doorways. Returns the corners it wraps, in order."""

    def solve(lo: int, hi: int, left: Arc | None, right: Arc | None) -> list[Arc]:
        if hi <= lo or len(result) > max_arcs:
            return []

        p0 = (left.x1, left.y1) if left else start
        p1 = (right.x0, right.y0) if right else goal

        found = _candidates(crossings, lo, hi, p0[0], p0[1], p1[0], p1[1], start, goal)
        if not found:
            return []

        # Try the worst first. If wrapping it cannot be joined to its neighbours -- circles
        # too close to admit a bitangent -- take the next worst rather than abandoning the
        # wire. The C original leaves this retry commented out and deletes bad arcs
        # afterwards instead, which is why it can silently emit a straight line through
        # everything.
        for candidate in sorted(found, key=lambda c: -c.violation):
            first, last = _contact_run(crossings, candidate.index, lo, hi,
                                       candidate.cx, candidate.cy)
            arc = Arc(candidate.cx, candidate.cy, candidate.r, candidate.ccw,
                      first=first, last=last)

            entry = (_arc_to_arc(left, arc) if left else
                     (None if (t := _point_to_arc(*p0, arc, False)) is None else (p0, t)))
            if entry is None:
                continue
            exit_ = (_arc_to_arc(arc, right) if right else
                     (None if (t := _point_to_arc(*p1, arc, True)) is None else (t, p1)))
            if exit_ is None:
                continue

            arc.x0, arc.y0 = entry[1]
            arc.x1, arc.y1 = exit_[0]
            if left is not None:
                left.x1, left.y1 = entry[0]
            if right is not None:
                right.x0, right.y0 = exit_[1]

            result.append(arc)
            before = solve(lo, first, left, arc)
            after = solve(last + 1, hi, arc, right)
            return before + [arc] + after

        return []

    result: list[Arc] = []
    arcs = solve(0, len(crossings), None, None)
    return _drop_loops(start, goal, arcs)


def _drop_loops(start, goal, arcs: list[Arc]) -> list[Arc]:
    """Remove arcs that doubled back once their neighbours were placed.

    An arc inserted early can become slack when an outer one is added later, leaving a visible
    hook. Detect it as the chord in and the chord out crossing each other, drop the arc, and
    re-tangent its neighbours.
    """
    changed = True
    while changed and arcs:
        changed = False
        for index, arc in enumerate(arcs):
            before = (arcs[index - 1].x1, arcs[index - 1].y1) if index else start
            after = (arcs[index + 1].x0, arcs[index + 1].y0) if index + 1 < len(arcs) else goal
            if _crosses(before, (arc.x0, arc.y0), (arc.x1, arc.y1), after):
                arcs.pop(index)
                _retangent(start, goal, arcs)
                changed = True
                break
    return arcs


def _crosses(p, q, r, s) -> bool:
    d1 = _wind(*r, *s, *p)
    d2 = _wind(*r, *s, *q)
    d3 = _wind(*p, *q, *r)
    d4 = _wind(*p, *q, *s)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _retangent(start, goal, arcs: list[Arc]) -> None:
    for index, arc in enumerate(arcs):
        previous = arcs[index - 1] if index else None
        following = arcs[index + 1] if index + 1 < len(arcs) else None
        if previous is None:
            point = _point_to_arc(*start, arc, False)
            if point:
                arc.x0, arc.y0 = point
        else:
            pair = _arc_to_arc(previous, arc)
            if pair:
                previous.x1, previous.y1 = pair[0]
                arc.x0, arc.y0 = pair[1]
        if following is None:
            point = _point_to_arc(*goal, arc, True)
            if point:
                arc.x1, arc.y1 = point


def to_geometry(start, goal, arcs: list[Arc]) -> list[Segment | Arc]:
    """Interleave the arcs with the straight runs between them."""
    if not arcs:
        return [Segment(start[0], start[1], goal[0], goal[1])]
    out: list[Segment | Arc] = []
    cursor = start
    for arc in arcs:
        if math.hypot(arc.x0 - cursor[0], arc.y0 - cursor[1]) > _EPS:
            out.append(Segment(cursor[0], cursor[1], arc.x0, arc.y0))
        out.append(arc)
        cursor = (arc.x1, arc.y1)
    if math.hypot(goal[0] - cursor[0], goal[1] - cursor[1]) > _EPS:
        out.append(Segment(cursor[0], cursor[1], goal[0], goal[1]))
    return out
