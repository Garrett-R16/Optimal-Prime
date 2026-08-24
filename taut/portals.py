"""Portals: the gaps between obstacles, and how many tracks fit through them.

This is what makes a bundle a bundle. Routing nets one at a time asks each of them, alone,
*where would a rubber band sit* -- and the answer is always the middle of whatever gap it
finds, because nothing else is there. A second net arriving at the same gap then finds it
full, and no amount of re-routing helps: the taut path is deterministic, so ripping the first
net up and laying it again puts it back exactly where it was.

The fix is to stop asking one net at a time. A gap of width ``D`` holds

    n = floor((D - clearance) / (track_width + clearance))

tracks. If two nets both need it and two fit, they are *assigned slots* -- and each one is
then routed against obstacles inflated by its own slot's share of the gap. The first net ends
up hugging one wall not because anything pushed it, but because the problem it was given
already had the second net in it.

Slots are the whole mechanism. Once they are assigned, nets are routed **ignoring each
other's copper entirely**: the separation is guaranteed by the geometry of the assignment
rather than negotiated after the fact.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .obstacles import Obstacle
from .tangent import PathArc, PathLine, TautPath, segment_to_obstacle

__all__ = ["Portal", "core_distance", "build_portals", "crossings", "SlotPlan",
           "assign_slots"]


def core_distance(a: Obstacle, b: Obstacle) -> float:
    """Shortest distance between two obstacles' cores, ignoring their inflation."""
    bare_b = Obstacle(vertices=b.vertices, r=0.0)
    best = math.inf
    for edge in a.edges():
        best = min(best, segment_to_obstacle(bare_b, *edge))
    if not a.edges():
        vx, vy = a.vertices[0]
        best = min(best, bare_b.distance_to_point(vx, vy))
    for edge in b.edges():
        bare_a = Obstacle(vertices=a.vertices, r=0.0)
        best = min(best, segment_to_obstacle(bare_a, *edge))
    if not b.edges():
        vx, vy = b.vertices[0]
        bare_a = Obstacle(vertices=a.vertices, r=0.0)
        best = min(best, bare_a.distance_to_point(vx, vy))
    return best


def _closest_points(a: Obstacle, b: Obstacle) -> tuple[tuple[float, float],
                                                       tuple[float, float]]:
    """A representative pair of nearest points, used as the portal's cut line."""
    best = (math.inf, a.vertices[0], b.vertices[0])
    for ax, ay in _sample_boundary(a):
        for bx, by in _sample_boundary(b):
            d = math.hypot(bx - ax, by - ay)
            if d < best[0]:
                best = (d, (ax, ay), (bx, by))
    return best[1], best[2]


def _sample_boundary(o: Obstacle, per_edge: int = 5):
    if len(o.vertices) == 1:
        return [o.vertices[0]]
    points = []
    for ax, ay, bx, by in o.edges():
        for i in range(per_edge + 1):
            t = i / per_edge
            points.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return points


@dataclass(frozen=True, slots=True)
class Portal:
    """A gap between two obstacles, with the number of tracks that fit through it."""

    a: int                       # index into the obstacle list
    b: int
    ax: float
    ay: float
    bx: float
    by: float
    width_nm: float
    capacity: int

    @property
    def cut(self) -> tuple[float, float, float, float]:
        return self.ax, self.ay, self.bx, self.by


def build_portals(obstacles: list[Obstacle], clearance: float, track_width: float,
                  max_tracks: int = 8) -> list[Portal]:
    """Gaps narrow enough to be worth reasoning about, with their capacities.

    Only pairs close enough to constrain anything are considered -- a gap that already holds
    ``max_tracks`` is not a constraint, it is open space, and enumerating every pair of
    obstacles on the board would cost more than it could ever save.
    """
    pitch = track_width + clearance
    interesting = clearance + max_tracks * pitch

    portals: list[Portal] = []
    count = len(obstacles)
    for i in range(count):
        a = obstacles[i]
        acx, acy = a.centre
        areach = a.reach
        for j in range(i + 1, count):
            b = obstacles[j]
            bcx, bcy = b.centre
            if math.hypot(bcx - acx, bcy - acy) > areach + b.reach + interesting:
                continue
            gap = core_distance(a, b)
            if gap <= 0.0 or gap > interesting:
                continue
            capacity = int((gap - clearance) // pitch)
            if capacity < 1:
                capacity = 0
            (ax, ay), (bx, by) = _closest_points(a, b)
            portals.append(Portal(i, j, ax, ay, bx, by, gap, capacity))
    return portals


# --------------------------------------------------------------------------- crossings

def _segments_of(path: TautPath, samples: int = 12):
    """The path as a polyline, which is all a crossing test needs."""
    points: list[tuple[float, float]] = []
    for element in path.elements:
        if isinstance(element, PathLine):
            points.append((element.x1, element.y1))
            points.append((element.x2, element.y2))
        else:
            for i in range(samples + 1):
                points.append(element.point_at(i / samples))
    out = []
    for p, q in zip(points, points[1:]):
        if p != q:
            out.append((p[0], p[1], q[0], q[1]))
    return out


def _cross_parameter(ax, ay, bx, by, px, py, qx, qy):
    """Where segment PQ crosses portal cut AB, as a fraction along AB; None if it misses."""
    rx, ry = bx - ax, by - ay
    sx, sy = qx - px, qy - py
    denom = rx * sy - ry * sx
    if abs(denom) < 1e-12:
        return None
    t = ((px - ax) * sy - (py - ay) * sx) / denom
    u = ((px - ax) * ry - (py - ay) * rx) / denom
    if -1e-9 <= t <= 1 + 1e-9 and -1e-9 <= u <= 1 + 1e-9:
        return max(0.0, min(1.0, t))
    return None


def crossings(path: TautPath, portals: list[Portal]) -> dict[int, float]:
    """Which portals a path passes through, and where along each cut.

    The parameter runs 0 at the portal's ``a`` side to 1 at its ``b`` side, which is what
    puts the nets in order across the gap.
    """
    found: dict[int, float] = {}
    pieces = _segments_of(path)
    for index, portal in enumerate(portals):
        for px, py, qx, qy in pieces:
            t = _cross_parameter(*portal.cut, px, py, qx, qy)
            if t is not None:
                found[index] = t
                break
    return found


# --------------------------------------------------------------------------- slots

@dataclass
class SlotPlan:
    """Who sits where in every shared gap, and what that costs each net in clearance."""

    #: (connection index, obstacle index) -> extra inflation required, in nm
    extra: dict[tuple[int, int], float] = field(default_factory=dict)
    #: portals that more nets wanted than could fit
    oversubscribed: list[int] = field(default_factory=list)
    shared: int = 0

    def inflation(self, connection: int, obstacle: int) -> float:
        return self.extra.get((connection, obstacle), 0.0)


def assign_slots(portals: list[Portal], per_connection: dict[int, dict[int, float]],
                 clearance: float, track_width: float) -> SlotPlan:
    """Order the nets across each shared gap and turn that order into clearances.

    A net sitting at slot ``i`` (counting from the portal's ``a`` side) has ``i`` tracks
    between it and that obstacle, so it must stand ``i * (width + clearance)`` further off
    than it otherwise would -- and symmetrically from the other side. Feeding those numbers
    back as per-obstacle inflation is what makes the bundle spread out.
    """
    plan = SlotPlan()
    pitch = track_width + clearance

    users: dict[int, list[tuple[float, int]]] = {}
    for connection, found in per_connection.items():
        for portal_index, t in found.items():
            users.setdefault(portal_index, []).append((t, connection))

    for portal_index, entries in users.items():
        if len(entries) < 2:
            continue
        portal = portals[portal_index]
        plan.shared += 1
        entries.sort()
        if len(entries) > max(portal.capacity, 1):
            plan.oversubscribed.append(portal_index)

        total = len(entries)

        # A slot is a *band*, not a line -- the gap is usually a little wider than the
        # tracks strictly need. Handing every net the whole of that slack lets them all
        # drift to the inside of their band and end up closer together than a track pitch,
        # which is how the first version of this seated two nets 1.5 apart in a gap that
        # required 2.0. So the slack is divided evenly and folded into the spacing:
        #
        #     share  = (D - 2c - w - (n-1)(w+c)) / n
        #     stride = (w + c) + share
        #
        # With that stride, any position inside one band is at least a full pitch from any
        # position inside the next, whatever each net does with its own freedom.
        slack = portal.width_nm - 2 * clearance - track_width - (total - 1) * pitch
        share = max(0.0, slack / total)
        stride = pitch + share

        for slot, (_t, connection) in enumerate(entries):
            from_a = slot * stride
            from_b = (total - 1 - slot) * stride
            if from_a > 0:
                key = (connection, portal.a)
                plan.extra[key] = max(plan.extra.get(key, 0.0), from_a)
            if from_b > 0:
                key = (connection, portal.b)
                plan.extra[key] = max(plan.extra.get(key, 0.0), from_b)

    return plan
