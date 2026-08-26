"""Is this set of taut wires a legal board? The one question everything else serves.

The pipeline's promise is: choose every wire's class first, then embed each class as its taut
geometry, and the result is legal *by construction*. This module is the referee for that
promise. It takes the whole sketch -- every wire's embedded geometry, plus the copper -- and
reports every way the promise failed: two wires crossing, two wires too close, a wire inside
copper it does not belong to.

It exists as its own module because it is both the test oracle and the runtime driver. The
first-principles tests hand it tiny synthetic scenes with known answers; the router runs it
after embedding and feeds each defect back to the stage that owns it -- a crossing to the
topology, a graze to the embedding. When it returns nothing, the fallback solver has nothing
to do, which is the point: the fallback chooses its own homotopy class, silently, and every
wire that takes it is a wire the negotiation no longer controls.

Defects are reported with *who* and *where*, not just counts, because the repair is always
directed: some one wire must move, around some one thing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .obstacles import Obstacle
from .tangent import PathArc, PathLine, TautPath, arc_to_obstacle, segment_to_obstacle

__all__ = ["SketchWire", "Crossing", "Graze", "Clip", "check_sketch"]


@dataclass(frozen=True, slots=True)
class SketchWire:
    """One embedded wire, as the checker needs to see it."""

    key: int
    net: int
    path: TautPath
    half_width: float
    clearance: float


@dataclass(frozen=True, slots=True)
class Crossing:
    """Two wires passing through each other.

    ``points`` holds every intersection of the pair, because the *parity* is the diagnosis:
    an odd count means the two homotopy classes genuinely conflict and topology must move
    someone; an even count is a lens -- compatible classes whose individually-taut
    geometries bulge through each other -- and the cure is geometric, one wire treating the
    other as copper. ``x, y`` remain the first point for convenience."""

    first: int
    second: int
    x: float
    y: float
    points: tuple = ()


@dataclass(frozen=True, slots=True)
class Graze:
    """Two wires too close. The classes are compatible; the geometry is not finished."""

    first: int
    second: int
    gap: float
    needed: float


@dataclass(frozen=True, slots=True)
class Clip:
    """A wire inside foreign copper's clearance."""

    key: int
    obstacle: int
    label: str
    x: float
    y: float
    depth: float


# --------------------------------------------------------------------------- flattening

def _points(path: TautPath, per_arc: int = 24) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for element in path.elements:
        if isinstance(element, PathLine):
            if not out:
                out.append((element.x1, element.y1))
            out.append((element.x2, element.y2))
        else:
            sweep = element.sweep
            steps = max(2, int(math.ceil(abs(sweep) / (2 * math.pi / per_arc))))
            for step in range(0 if not out else 1, steps + 1):
                angle = element.start_angle + sweep * step / steps
                out.append((element.cx + element.r * math.cos(angle),
                            element.cy + element.r * math.sin(angle)))
    return out


def _bounds(points):
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    return min(xs), min(ys), max(xs), max(ys)


def _cross(ax, ay, bx, by, cx, cy, dx, dy):
    def wind(px, py, qx, qy, rx, ry):
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)
    d1 = wind(cx, cy, dx, dy, ax, ay)
    d2 = wind(cx, cy, dx, dy, bx, by)
    d3 = wind(ax, ay, bx, by, cx, cy)
    d4 = wind(ax, ay, bx, by, dx, dy)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        denom = (bx - ax) * (dy - cy) - (by - ay) * (dx - cx)
        if abs(denom) < 1e-12:
            return None
        t = ((cx - ax) * (dy - cy) - (cy - ay) * (dx - cx)) / denom
        return ax + t * (bx - ax), ay + t * (by - ay)
    return None


def _segment_gap(ax, ay, bx, by, cx, cy, dx, dy) -> float:
    def point_seg(px, py, x0, y0, x1, y1):
        ex, ey = x1 - x0, y1 - y0
        span = ex * ex + ey * ey
        t = 0.0 if span < 1e-18 else max(0.0, min(1.0, ((px - x0) * ex + (py - y0) * ey) / span))
        return math.hypot(px - x0 - t * ex, py - y0 - t * ey)
    return min(point_seg(ax, ay, cx, cy, dx, dy), point_seg(bx, by, cx, cy, dx, dy),
               point_seg(cx, cy, ax, ay, bx, by), point_seg(dx, dy, ax, ay, bx, by))


# --------------------------------------------------------------------------- the check

def check_sketch(wires: list[SketchWire], obstacles: list[Obstacle],
                 guard: float = 0.0):
    """Every defect in the sketch: crossings, grazes, and clips, in that order of severity.

    ``obstacles`` carry bare copper (their own true extent); the wire's clearance and half
    width are added here, plus ``guard``. Wires of the same net are never compared.
    """
    crossings: list[Crossing] = []
    grazes: list[Graze] = []
    clips: list[Clip] = []

    flat = {wire.key: _points(wire.path) for wire in wires}
    boxes = {wire.key: _bounds(flat[wire.key]) for wire in wires if flat[wire.key]}

    ordered = sorted(wires, key=lambda wire: wire.key)
    for index, one in enumerate(ordered):
        for two in ordered[index + 1:]:
            if one.net == two.net:
                continue
            if one.key not in boxes or two.key not in boxes:
                continue
            needed = (one.half_width + two.half_width
                      + max(one.clearance, two.clearance) + guard)
            a, b = boxes[one.key], boxes[two.key]
            if (a[2] + needed < b[0] or b[2] + needed < a[0]
                    or a[3] + needed < b[1] or b[3] + needed < a[1]):
                continue

            met: list = []
            best = math.inf
            pa = flat[one.key]
            pb = flat[two.key]
            for sa, sb in zip(pa, pa[1:]):
                for sc, sd in zip(pb, pb[1:]):
                    hit = _cross(sa[0], sa[1], sb[0], sb[1], sc[0], sc[1], sd[0], sd[1])
                    if hit is not None:
                        met.append(hit)
                        continue
                    if not met:
                        gap = _segment_gap(sa[0], sa[1], sb[0], sb[1],
                                           sc[0], sc[1], sd[0], sd[1])
                        if gap < best:
                            best = gap

            if met:
                crossings.append(Crossing(one.key, two.key, met[0][0], met[0][1],
                                          points=tuple(met)))
            elif best < needed:
                grazes.append(Graze(one.key, two.key, best, needed))

    for wire in wires:
        halo = wire.half_width + wire.clearance + guard
        for index, obstacle in enumerate(obstacles):
            if obstacle.net == wire.net:
                continue
            worst = math.inf
            for element in wire.path.elements:
                if isinstance(element, PathLine):
                    distance = segment_to_obstacle(obstacle, element.x1, element.y1,
                                                   element.x2, element.y2)
                else:
                    distance = arc_to_obstacle(obstacle, element)
                if distance < worst:
                    worst = distance
            if worst < obstacle.r + halo - 1e-3:
                where = _nearest_point(flat[wire.key], obstacle)
                clips.append(Clip(wire.key, index, obstacle.label, where[0], where[1],
                                  obstacle.r + halo - worst))

    return crossings, grazes, clips


def _nearest_point(points, obstacle: Obstacle) -> tuple[float, float]:
    best = (math.inf, points[0])
    for point in points:
        d = obstacle.distance_to_point(*point)
        if d < best[0]:
            best = (d, point)
    return best[1]
