"""The taut-string solver: shortest collision-free path among convex obstacles.

A rubber band stretched between two points and pulled taut around convex obstacles takes a
shape with exactly two ingredients -- straight lines tangent to the obstacles, and arcs riding
on their rounded corners. That is a theorem, not an approximation, and it is why this produces
KiCad-native geometry with no rounding step: ``segment`` and ``arc`` are precisely the two
primitives the optimum is made of.

Each obstacle is a convex polygon inflated by ``r`` (see :mod:`taut.obstacles`). The band can
only ever bend on a **wrap circle** of radius ``r`` centred on one of the polygon's vertices;
between bends it runs straight. So the construction is the classical tangent graph, with the
wrap circles as its discs:

* **nodes** -- the two endpoints, plus every distinct tangent point on a wrap circle;
* **line edges** -- bitangents between wrap circles, kept only if the segment clears every
  obstacle *body* and every other wrap circle;
* **arc edges** -- travel along a wrap circle between two of its own tangent points, kept
  under the same test.

Dijkstra over that graph returns the shortest path, and the shortest collision-free path *is*
the taut string. No grid, no discretisation, no preferred directions.

Three things keep it fast. Tangent points are **deduplicated by angle**, so a circle touched
by twenty bitangents contributes twenty nodes rather than twenty per pair. Circle collision is
**vectorised** with numpy. Body collision is **screened** by each obstacle's bounding circle,
so the exact polygon test runs only for the handful that could possibly be in the way.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np

from . import geometry as geo
from .obstacles import Obstacle

__all__ = ["TautPath", "PathLine", "PathArc", "solve", "NoPathFound", "violated_obstacles"]

_EPS = 1e-6
_BODY_TOL = 1e-3          # nanometres; slack for a tangent point sitting exactly on a boundary
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

    def as_geo(self) -> geo.Arc:
        sx, sy = self.start
        ex, ey = self.end
        return geo.Arc(cx=self.cx, cy=self.cy, r=self.r,
                       start_angle=(self.start_angle if self.ccw else self.end_angle),
                       sweep=abs(self.sweep), x1=sx, y1=sy, x2=ex, y2=ey)


@dataclass
class TautPath:
    elements: list[PathLine | PathArc]

    @property
    def length(self) -> float:
        return sum(e.length for e in self.elements)


# --------------------------------------------------------------------------- field

@dataclass(frozen=True, slots=True)
class _Circle:
    x: float
    y: float
    r: float
    owner: int          # index into the obstacle list, or negative for an endpoint


class _Field:
    """Wrap circles, obstacle bodies, and the board boundary."""

    def __init__(self, obstacles: list[Obstacle], boundary=None,
                 boundary_gap: float = 0.0) -> None:
        self.obstacles = obstacles
        self.boundary = boundary or []
        self.boundary_gap = boundary_gap

        self.circles: list[_Circle] = []
        for index, obstacle in enumerate(obstacles):
            for vx, vy in obstacle.vertices:
                self.circles.append(_Circle(vx, vy, obstacle.r, index))

        self.n = len(self.circles)
        if self.n:
            self.cx = np.fromiter((c.x for c in self.circles), float, self.n)
            self.cy = np.fromiter((c.y for c in self.circles), float, self.n)
            self.cr = np.fromiter((c.r for c in self.circles), float, self.n)
        else:
            self.cx = self.cy = self.cr = np.empty(0)

        self.m = len(obstacles)
        if self.m:
            centres = [o.centre for o in obstacles]
            self.bx = np.fromiter((c[0] for c in centres), float, self.m)
            self.by = np.fromiter((c[1] for c in centres), float, self.m)
            self.breach = np.fromiter((o.reach for o in obstacles), float, self.m)
        else:
            self.bx = self.by = self.breach = np.empty(0)

    # -- boundary ---------------------------------------------------------------------

    def _boundary_safe_box(self):
        """The axis-aligned interior no boundary shape can reach, or None.

        Most boards are rectangles, and on a rectangle every wholly-interior segment
        clears the outline by inspection -- two comparisons instead of a distance test per
        outline shape. Profiled on a 282-connection board, the outline test was the single
        hottest frame in the whole pipeline; this removes it for the common case.
        """
        cached = getattr(self, "_safe_box", "unset")
        if cached != "unset":
            return cached
        xs, ys = [], []
        for shape in self.boundary:
            if isinstance(shape, geo.Arc):
                self._safe_box = None
                return None
            ax, ay, bx, by = shape
            xs.extend((ax, bx))
            ys.extend((ay, by))
        if not xs:
            self._safe_box = None
            return None
        margin = self.boundary_gap + 1.0
        self._safe_box = (min(xs) + margin, min(ys) + margin,
                          max(xs) - margin, max(ys) - margin)
        return self._safe_box

    def _boundary_clears_segment(self, x1, y1, x2, y2) -> bool:
        box = self._boundary_safe_box()
        if box is not None:
            lo_x, lo_y, hi_x, hi_y = box
            if (lo_x <= x1 <= hi_x and lo_y <= y1 <= hi_y
                    and lo_x <= x2 <= hi_x and lo_y <= y2 <= hi_y):
                return True
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

    def _boundary_clears_arc(self, arc: PathArc) -> bool:
        own = arc.as_geo()
        for shape in self.boundary:
            if isinstance(shape, geo.Arc):
                if geo.arc_arc(own, shape) < self.boundary_gap - _EPS:
                    return False
            else:
                ax, ay, bx, by = shape
                if geo.segment_arc(ax, ay, bx, by, own) < self.boundary_gap - _EPS:
                    return False
        return True

    # -- bodies -----------------------------------------------------------------------

    def _body_candidates(self, x1, y1, x2, y2) -> np.ndarray:
        """Obstacles whose bounding circle the query could reach; the rest cannot matter."""
        if not self.m:
            return np.empty(0, dtype=int)
        dx, dy = x2 - x1, y2 - y1
        span = dx * dx + dy * dy
        if span <= _EPS:
            near = np.hypot(self.bx - x1, self.by - y1)
        else:
            t = np.clip(((self.bx - x1) * dx + (self.by - y1) * dy) / span, 0.0, 1.0)
            near = np.hypot(self.bx - (x1 + t * dx), self.by - (y1 + t * dy))
        return np.nonzero(near < self.breach)[0]

    # -- public tests -----------------------------------------------------------------

    def segment_clears(self, x1, y1, x2, y2, skip_circles: set[int],
                       owners: set[int]) -> bool:
        if self.boundary and not self._boundary_clears_segment(x1, y1, x2, y2):
            return False

        if self.n:
            dx, dy = x2 - x1, y2 - y1
            span = dx * dx + dy * dy
            if span <= _EPS:
                near = np.hypot(self.cx - x1, self.cy - y1)
            else:
                t = np.clip(((self.cx - x1) * dx + (self.cy - y1) * dy) / span, 0.0, 1.0)
                near = np.hypot(self.cx - (x1 + t * dx), self.cy - (y1 + t * dy))
            hit = near < self.cr - _EPS
            if skip_circles:
                hit[list(skip_circles)] = False
            if bool(hit.any()):
                return False

        for index in self._body_candidates(x1, y1, x2, y2):
            obstacle = self.obstacles[index]
            tolerance = _BODY_TOL if index in owners else _EPS
            if segment_to_obstacle(obstacle, x1, y1, x2, y2) < obstacle.r - tolerance:
                return False
        return True

    def arc_clears(self, arc: PathArc, skip_circles: set[int], owners: set[int]) -> bool:
        if self.boundary and not self._boundary_clears_arc(arc):
            return False

        if self.n:
            gap = np.hypot(self.cx - arc.cx, self.cy - arc.cy)
            for index in np.nonzero(gap < arc.r + self.cr - _EPS)[0]:
                if index in skip_circles:
                    continue
                distance = gap[index]
                radius = self.cr[index]
                if distance + arc.r <= radius + _EPS:
                    return False
                if distance <= _EPS:
                    continue
                cosine = (distance * distance + arc.r * arc.r - radius * radius) / (
                    2.0 * distance * arc.r)
                if cosine >= 1.0 - _EPS:
                    continue
                half = math.acos(max(-1.0, min(1.0, cosine)))
                centre = math.atan2(self.cy[index] - arc.cy, self.cx[index] - arc.cx)
                if _span_meets(arc, centre, half):
                    return False

        sx, sy = arc.start
        ex, ey = arc.end
        for index in self._body_candidates(sx, sy, ex, ey):
            obstacle = self.obstacles[index]
            tolerance = _BODY_TOL if index in owners else _EPS
            if arc_to_obstacle(obstacle, arc) < obstacle.r - tolerance:
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


# --------------------------------------------------------------------------- distances

def segment_to_obstacle(obstacle: Obstacle, x1, y1, x2, y2) -> float:
    """Distance from a segment to an obstacle's *core* polygon, before inflation."""
    if len(obstacle.vertices) == 1:
        vx, vy = obstacle.vertices[0]
        return geo.point_segment(vx, vy, x1, y1, x2, y2)
    if obstacle.distance_to_point(x1, y1) <= 0.0 or obstacle.distance_to_point(x2, y2) <= 0.0:
        return 0.0
    return min(geo.segment_segment(x1, y1, x2, y2, *edge) for edge in obstacle.edges())


def arc_to_obstacle(obstacle: Obstacle, arc: PathArc) -> float:
    own = arc.as_geo()
    if len(obstacle.vertices) == 1:
        vx, vy = obstacle.vertices[0]
        return geo.point_arc(vx, vy, own)
    sx, sy = arc.start
    ex, ey = arc.end
    if obstacle.distance_to_point(sx, sy) <= 0.0 or obstacle.distance_to_point(ex, ey) <= 0.0:
        return 0.0
    return min(geo.segment_arc(*edge, own) for edge in obstacle.edges())


# --------------------------------------------------------------------------- tangents

def _bitangents(ax, ay, ar, bx, by, br):
    """Every bitangent between two circles, as ((px, py), (qx, qy), angle_a, angle_b)."""
    dx, dy = bx - ax, by - ay
    gap = math.hypot(dx, dy)
    if gap < _EPS:
        return
    base = math.atan2(dy, dx)

    delta = ar - br
    if abs(delta) <= gap:
        alpha = math.acos(max(-1.0, min(1.0, delta / gap)))
        for sign in (1.0, -1.0):
            theta = base + sign * alpha
            yield ((ax + ar * math.cos(theta), ay + ar * math.sin(theta)),
                   (bx + br * math.cos(theta), by + br * math.sin(theta)),
                   theta, theta)

    total = ar + br
    if total <= gap:
        alpha = math.acos(max(-1.0, min(1.0, total / gap)))
        for sign in (1.0, -1.0):
            theta = base + sign * alpha
            opposite = theta + math.pi
            yield ((ax + ar * math.cos(theta), ay + ar * math.sin(theta)),
                   (bx + br * math.cos(opposite), by + br * math.sin(opposite)),
                   theta, opposite)


# --------------------------------------------------------------------------- solver

def solve(start: tuple[float, float], goal: tuple[float, float],
          obstacles: list[Obstacle], boundary=None,
          boundary_gap: float = 0.0) -> TautPath:
    """Shortest collision-free path from ``start`` to ``goal`` among ``obstacles``."""
    sx, sy = float(start[0]), float(start[1])
    gx, gy = float(goal[0]), float(goal[1])

    # An endpoint inside a keep-out has no legal path. Quietly ignoring the obstacle -- which
    # an earlier version did -- lets the router lay copper straight over a neighbouring pad
    # and report success, with the violation only surfacing later in DRC.
    for obstacle in obstacles:
        if obstacle.contains(sx, sy) or obstacle.contains(gx, gy):
            raise NoPathFound(
                f"endpoint lies inside the keep-out of {obstacle.label or 'an obstacle'}")

    field = _Field(obstacles, boundary=boundary, boundary_gap=boundary_gap)

    # Endpoints join the circle list as zero-radius circles owned by nothing, so one piece of
    # tangent code covers every case.
    circles = [_Circle(sx, sy, 0.0, -1), _Circle(gx, gy, 0.0, -2)] + field.circles
    OFFSET = 2

    points: list[tuple[int, float, float, float]] = [(0, 0.0, sx, sy), (1, 0.0, gx, gy)]
    lookup: dict[tuple[int, int], int] = {}
    per_circle: dict[int, list[int]] = {}
    edges: dict[int, list[tuple[int, float]]] = {0: [], 1: []}

    def node_for(circle_index: int, angle: float, x: float, y: float) -> int:
        if circle_index < OFFSET:
            return circle_index
        key = (circle_index, int(round((angle % TAU) / _ANGLE_QUANT)))
        existing = lookup.get(key)
        if existing is not None:
            return existing
        node = len(points)
        points.append((circle_index, angle % TAU, x, y))
        lookup[key] = node
        per_circle.setdefault(circle_index, []).append(node)
        edges[node] = []
        return node

    def owners_of(*indices: int) -> set[int]:
        return {circles[i].owner for i in indices if i >= OFFSET}

    # ---- line edges -----------------------------------------------------------------
    for i in range(len(circles)):
        a = circles[i]
        for j in range(i + 1, len(circles)):
            b = circles[j]
            skip = {k - OFFSET for k in (i, j) if k >= OFFSET}
            owners = owners_of(i, j)
            for (px, py), (qx, qy), angle_a, angle_b in _bitangents(a.x, a.y, a.r,
                                                                    b.x, b.y, b.r):
                if not field.segment_clears(px, py, qx, qy, skip, owners):
                    continue
                node_a = node_for(i, angle_a, px, py)
                node_b = node_for(j, angle_b, qx, qy)
                if node_a == node_b:
                    continue
                cost = math.hypot(qx - px, qy - py)
                edges[node_a].append((node_b, cost))
                edges[node_b].append((node_a, cost))

    # ---- arc edges ------------------------------------------------------------------
    for circle_index, nodes in per_circle.items():
        circle = circles[circle_index]
        skip = {circle_index - OFFSET}
        owners = owners_of(circle_index)
        for p in range(len(nodes)):
            for q in range(p + 1, len(nodes)):
                na, nb = nodes[p], nodes[q]
                angle_a, angle_b = points[na][1], points[nb][1]
                for ccw in (True, False):
                    arc = PathArc(circle.x, circle.y, circle.r, angle_a, angle_b, ccw)
                    if abs(arc.sweep) < 1e-9 or not field.arc_clears(arc, skip, owners):
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
        circle_a, angle_a, ax, ay = points[a]
        circle_b, angle_b, bx, by = points[b]
        if circle_a == circle_b and circle_a >= OFFSET:
            circle = circles[circle_a]
            skip = {circle_a - OFFSET}
            owners = owners_of(circle_a)
            best = None
            for ccw in (True, False):
                arc = PathArc(circle.x, circle.y, circle.r, angle_a, angle_b, ccw)
                if abs(arc.sweep) < 1e-9 or not field.arc_clears(arc, skip, owners):
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

def violated_obstacles(path: TautPath, obstacles: list[Obstacle],
                       tol: float = 1e-3) -> list[int]:
    """Indices of obstacles the path penetrates.

    Used for lazy obstacle addition: a taut path is shaped only by obstacles near it, so the
    solver runs against a local subset and this reports whatever was missed. The loop exits
    only when nothing is violated, so the answer matches solving against everything.
    """
    guilty: list[int] = []
    for index, obstacle in enumerate(obstacles):
        for element in path.elements:
            if isinstance(element, PathLine):
                distance = segment_to_obstacle(obstacle, element.x1, element.y1,
                                               element.x2, element.y2)
            else:
                distance = arc_to_obstacle(obstacle, element)
            if distance < obstacle.r - tol:
                guilty.append(index)
                break
    return guilty
