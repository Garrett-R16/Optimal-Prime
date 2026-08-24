"""The taut-string solver: shortest collision-free path among discs.

A rubber band stretched between two points and pulled taut around circular obstacles takes a
shape with exactly two ingredients -- straight lines tangent to the circles, and arcs riding
on their surfaces. That is a theorem, not an approximation, and it is why this produces
KiCad-native geometry with no rounding step: ``segment`` and ``arc`` are precisely the two
primitives the optimum is made of.

The construction is the classical tangent (visibility) graph:

* **nodes** -- the two endpoints, plus every distinct tangent point where a bitangent touches
  a disc;
* **line edges** -- bitangents between discs (up to four per pair: two outer, two inner) and
  tangents from the endpoints, kept only if the segment clears every other disc;
* **arc edges** -- travel along a disc's rim between two of its own tangent points, kept only
  if the arc clears every other disc.

Dijkstra over that graph returns the shortest path, and the shortest collision-free path *is*
the taut string. No grid, no discretisation, no preferred directions.

Two things make it fast enough to be usable. Tangent points are **deduplicated by angle**, so
a disc touched by twenty bitangents contributes twenty nodes rather than twenty per pair --
without that, arc edges grow with the square of a number that was already quadratic.
Collision tests are **vectorised**: every disc is screened at once with numpy, and only the
handful that survive the screen get the exact scalar test.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np

from .obstacles import Disc

__all__ = ["TautPath", "PathLine", "PathArc", "solve", "NoPathFound", "violated_discs"]

_EPS = 1e-6
TAU = 2.0 * math.pi
_ANGLE_QUANT = 1e-9


class NoPathFound(RuntimeError):
    """No collision-free taut path exists between the endpoints."""


@dataclass(frozen=True, slots=True)
class PathLine:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def length(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)


@dataclass(frozen=True, slots=True)
class PathArc:
    cx: float
    cy: float
    r: float
    start_angle: float
    end_angle: float
    ccw: bool

    @property
    def sweep(self) -> float:
        delta = (self.end_angle - self.start_angle) % TAU
        return delta if self.ccw else -((TAU - delta) % TAU)

    @property
    def length(self) -> float:
        return self.r * abs(self.sweep)

    def point_at(self, t: float) -> tuple[float, float]:
        theta = self.start_angle + self.sweep * t
        return self.cx + self.r * math.cos(theta), self.cy + self.r * math.sin(theta)

    @property
    def mid(self) -> tuple[float, float]:
        return self.point_at(0.5)

    @property
    def start(self) -> tuple[float, float]:
        return self.point_at(0.0)

    @property
    def end(self) -> tuple[float, float]:
        return self.point_at(1.0)


@dataclass
class TautPath:
    elements: list[PathLine | PathArc]

    @property
    def length(self) -> float:
        return sum(e.length for e in self.elements)


# --------------------------------------------------------------------------- field

class _Field:
    """Disc geometry as numpy arrays, plus an exact board boundary.

    The boundary is kept as real segments and arcs rather than chopped into a chain of
    keep-out discs. A board outline discretised at the clearance scale needs hundreds of
    discs, and every one of them joins the quadratic bitangent enumeration; as primitives it
    costs a handful of distance calls.
    """

    def __init__(self, discs, boundary=None, boundary_gap: float = 0.0) -> None:
        self.discs = discs
        self.boundary = boundary or []
        self.boundary_gap = boundary_gap
        self.n = len(discs)
        if self.n:
            self.x = np.fromiter((d.x for d in discs), float, self.n)
            self.y = np.fromiter((d.y for d in discs), float, self.n)
            self.r = np.fromiter((d.r for d in discs), float, self.n)
        else:
            self.x = self.y = self.r = np.empty(0)

    def _boundary_clears_segment(self, x1, y1, x2, y2) -> bool:
        from . import geometry as geo
        for shape in self.boundary:
            if isinstance(shape, geo.Arc):
                if geo.segment_arc(x1, y1, x2, y2, shape) < self.boundary_gap - _EPS:
                    return False
            else:
                ax, ay, bx, by = shape
                if geo.segment_segment(x1, y1, x2, y2,
                                       ax, ay, bx, by) < self.boundary_gap - _EPS:
                    return False
        return True

    def _boundary_clears_arc(self, arc) -> bool:
        from . import geometry as geo
        sx, sy = arc.start
        ex, ey = arc.end
        own = geo.Arc(cx=arc.cx, cy=arc.cy, r=arc.r,
                      start_angle=(arc.start_angle if arc.ccw else arc.end_angle),
                      sweep=abs(arc.sweep), x1=sx, y1=sy, x2=ex, y2=ey)
        for shape in self.boundary:
            if isinstance(shape, geo.Arc):
                if geo.arc_arc(own, shape) < self.boundary_gap - _EPS:
                    return False
            else:
                ax, ay, bx, by = shape
                if geo.segment_arc(ax, ay, bx, by, own) < self.boundary_gap - _EPS:
                    return False
        return True

    def segment_clears(self, x1, y1, x2, y2, skip: set[int]) -> bool:
        if self.boundary and not self._boundary_clears_segment(x1, y1, x2, y2):
            return False
        if not self.n:
            return True
        dx, dy = x2 - x1, y2 - y1
        span = dx * dx + dy * dy
        if span <= _EPS:
            near = np.hypot(self.x - x1, self.y - y1)
        else:
            t = np.clip(((self.x - x1) * dx + (self.y - y1) * dy) / span, 0.0, 1.0)
            near = np.hypot(self.x - (x1 + t * dx), self.y - (y1 + t * dy))
        hit = near < self.r - _EPS
        if skip:
            hit[list(skip)] = False
        return not bool(hit.any())

    def arc_clears(self, arc: PathArc, skip: set[int]) -> bool:
        if self.boundary and not self._boundary_clears_arc(arc):
            return False
        if not self.n:
            return True
        gap = np.hypot(self.x - arc.cx, self.y - arc.cy)
        # Screen: only discs whose circle can meet the arc's circle matter at all.
        near = np.nonzero(gap < arc.r + self.r - _EPS)[0]
        for index in near:
            if index in skip:
                continue
            distance = gap[index]
            radius = self.r[index]
            if distance + arc.r <= radius + _EPS:
                return False                        # the arc's circle is swallowed whole
            if distance <= _EPS:
                continue
            cosine = (distance * distance + arc.r * arc.r - radius * radius) / (
                2.0 * distance * arc.r)
            if cosine >= 1.0 - _EPS:
                continue
            half = math.acos(max(-1.0, min(1.0, cosine)))
            centre = math.atan2(self.y[index] - arc.cy, self.x[index] - arc.cx)
            if _span_meets(arc, centre, half):
                return False
        return True


def _span_meets(arc: PathArc, centre: float, half: float) -> bool:
    """Does the arc's swept range enter the angular window of half-width ``half``?"""
    steps = max(3, int(abs(arc.sweep) / 0.12) + 3)
    for i in range(steps + 1):
        theta = arc.start_angle + arc.sweep * i / steps
        if abs(((theta - centre + math.pi) % TAU) - math.pi) < half - 1e-9:
            return True
    return False


# --------------------------------------------------------------------------- tangents

def _bitangents(a: Disc, b: Disc):
    """Every bitangent between two discs as ((ax, ay), (bx, by), angle_a, angle_b)."""
    dx, dy = b.x - a.x, b.y - a.y
    gap = math.hypot(dx, dy)
    if gap < _EPS:
        return
    base = math.atan2(dy, dx)

    delta = a.r - b.r
    if abs(delta) <= gap:
        alpha = math.acos(max(-1.0, min(1.0, delta / gap)))
        for sign in (1.0, -1.0):
            theta = base + sign * alpha
            yield ((a.x + a.r * math.cos(theta), a.y + a.r * math.sin(theta)),
                   (b.x + b.r * math.cos(theta), b.y + b.r * math.sin(theta)),
                   theta, theta)

    total = a.r + b.r
    if total <= gap:
        alpha = math.acos(max(-1.0, min(1.0, total / gap)))
        for sign in (1.0, -1.0):
            theta = base + sign * alpha
            opposite = theta + math.pi
            yield ((a.x + a.r * math.cos(theta), a.y + a.r * math.sin(theta)),
                   (b.x + b.r * math.cos(opposite), b.y + b.r * math.sin(opposite)),
                   theta, opposite)


# --------------------------------------------------------------------------- solver

def solve(start: tuple[float, float], goal: tuple[float, float],
          discs: list[Disc], boundary=None, boundary_gap: float = 0.0) -> TautPath:
    """Shortest collision-free path from ``start`` to ``goal`` among ``discs``."""
    sx, sy = float(start[0]), float(start[1])
    gx, gy = float(goal[0]), float(goal[1])

    # An endpoint inside a keep-out has no legal path at all. Silently dropping the
    # offending disc -- which an earlier version did -- lets the router lay copper straight
    # over a neighbouring pad and call it a success; the violation only turns up later in
    # DRC. Failing loudly here is worth more than a route that is quietly wrong.
    for disc in discs:
        if disc.contains(sx, sy) or disc.contains(gx, gy):
            raise NoPathFound(
                f"endpoint lies inside the keep-out of {disc.label or 'an obstacle'}"
            )
    field = _Field(discs, boundary=boundary, boundary_gap=boundary_gap)
    live = discs

    # Endpoints are zero-radius discs, so one piece of tangent code covers every case.
    entities: list[Disc] = [Disc(sx, sy, 0.0, label="start"),
                            Disc(gx, gy, 0.0, label="goal")] + list(live)
    OFFSET = 2

    points: list[tuple[int, float, float, float]] = [(0, 0.0, sx, sy), (1, 0.0, gx, gy)]
    lookup: dict[tuple[int, int], int] = {}
    per_disc: dict[int, list[int]] = {}
    edges: dict[int, list[tuple[int, float]]] = {0: [], 1: []}

    def node_for(entity: int, angle: float, x: float, y: float) -> int:
        if entity < OFFSET:
            return entity
        key = (entity, int(round((angle % TAU) / _ANGLE_QUANT)))
        existing = lookup.get(key)
        if existing is not None:
            return existing
        node = len(points)
        points.append((entity, angle % TAU, x, y))
        lookup[key] = node
        per_disc.setdefault(entity, []).append(node)
        edges[node] = []
        return node

    # ---- line edges -----------------------------------------------------------------
    for i in range(len(entities)):
        for j in range(i + 1, len(entities)):
            skip = {k - OFFSET for k in (i, j) if k >= OFFSET}
            for (ax, ay), (bx, by), angle_a, angle_b in _bitangents(entities[i],
                                                                    entities[j]):
                if not field.segment_clears(ax, ay, bx, by, skip):
                    continue
                node_a = node_for(i, angle_a, ax, ay)
                node_b = node_for(j, angle_b, bx, by)
                if node_a == node_b:
                    continue
                cost = math.hypot(bx - ax, by - ay)
                edges[node_a].append((node_b, cost))
                edges[node_b].append((node_a, cost))

    # ---- arc edges ------------------------------------------------------------------
    for entity, nodes in per_disc.items():
        disc = entities[entity]
        skip = {entity - OFFSET}
        for p in range(len(nodes)):
            for q in range(p + 1, len(nodes)):
                na, nb = nodes[p], nodes[q]
                angle_a, angle_b = points[na][1], points[nb][1]
                for ccw in (True, False):
                    arc = PathArc(disc.x, disc.y, disc.r, angle_a, angle_b, ccw)
                    if abs(arc.sweep) < 1e-9 or not field.arc_clears(arc, skip):
                        continue
                    edges[na].append((nb, arc.length))
                    edges[nb].append((na, arc.length))

    # ---- Dijkstra -------------------------------------------------------------------
    dist = {0: 0.0}
    prev: dict[int, int | None] = {0: None}
    heap: list[tuple[float, int]] = [(0.0, 0)]
    while heap:
        cost, node = heapq.heappop(heap)
        if cost > dist.get(node, math.inf):
            continue
        if node == 1:
            break
        for neighbour, weight in edges.get(node, ()):
            nxt = cost + weight
            if nxt < dist.get(neighbour, math.inf) - 1e-9:
                dist[neighbour] = nxt
                prev[neighbour] = node
                heapq.heappush(heap, (nxt, neighbour))

    if 1 not in dist:
        raise NoPathFound("no collision-free taut path between the endpoints")

    chain: list[int] = []
    cursor: int | None = 1
    while cursor is not None:
        chain.append(cursor)
        cursor = prev[cursor]
    chain.reverse()

    # ---- rebuild geometry -----------------------------------------------------------
    elements: list[PathLine | PathArc] = []
    for a, b in zip(chain, chain[1:]):
        entity_a, angle_a, ax, ay = points[a]
        entity_b, angle_b, bx, by = points[b]
        if entity_a == entity_b and entity_a >= OFFSET:
            disc = entities[entity_a]
            best = None
            for ccw in (True, False):
                arc = PathArc(disc.x, disc.y, disc.r, angle_a, angle_b, ccw)
                if abs(arc.sweep) < 1e-9:
                    continue
                if not field.arc_clears(arc, {entity_a - OFFSET}):
                    continue
                if best is None or arc.length < best.length:
                    best = arc
            if best is not None:
                elements.append(best)
                continue
        if math.hypot(bx - ax, by - ay) > 1e-9:
            elements.append(PathLine(ax, ay, bx, by))

    return TautPath(elements)


# --------------------------------------------------------------------------- checking

def violated_discs(path: TautPath, discs: list[Disc], tol: float = 1e-3) -> list[int]:
    """Indices of discs the path penetrates.

    Used for lazy obstacle addition: a taut path is shaped only by obstacles near it, so the
    solver runs against a local subset and this reports whatever was missed. The loop exits
    only when nothing is violated, so the answer is identical to solving against everything.
    """
    if not discs:
        return []
    field = _Field(discs)
    guilty = np.zeros(field.n, dtype=bool)

    for element in path.elements:
        if isinstance(element, PathLine):
            dx = element.x2 - element.x1
            dy = element.y2 - element.y1
            span = dx * dx + dy * dy
            if span <= _EPS:
                near = np.hypot(field.x - element.x1, field.y - element.y1)
            else:
                t = np.clip(((field.x - element.x1) * dx
                             + (field.y - element.y1) * dy) / span, 0.0, 1.0)
                near = np.hypot(field.x - (element.x1 + t * dx),
                                field.y - (element.y1 + t * dy))
            guilty |= near < field.r - tol
        else:
            steps = max(4, int(abs(element.sweep) / 0.1) + 4)
            for i in range(steps + 1):
                px, py = element.point_at(i / steps)
                guilty |= np.hypot(field.x - px, field.y - py) < field.r - tol

    return [int(i) for i in np.nonzero(guilty)[0]]
