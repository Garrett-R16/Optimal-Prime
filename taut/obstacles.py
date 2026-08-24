"""Obstacles: convex shapes, inflated by a clearance radius.

An obstacle is a convex polygon grown outward by ``r`` -- the Minkowski sum of the polygon
with a disc. Its boundary is therefore the polygon's edges pushed out by ``r``, joined by
quarter-turn arcs of radius ``r`` centred on the polygon's *vertices*.

That is the whole reason this shape family was chosen. A rubber band pulled taut around such a
region touches it only along those vertex arcs and leaves along the offset edges, so the path
is still straight lines and circular arcs -- exactly what a ``.kicad_pcb`` can hold -- while
the obstacle itself now fits the copper instead of swallowing it.

One representation covers everything on a board:

* **1 vertex** -- a disc. A round pad, or a via.
* **2 vertices** -- a capsule. A track segment, and the reason a finished track now costs two
  wrap circles instead of the forty-odd discs a chain needed.
* **4 vertices** -- a rounded rectangle. A rectangular or oval pad, at its true size rather
  than the circle that encloses it.

The enclosing circle was the previous model, and it over-covered a long pad by up to
sqrt(2) in every direction: room the router could not use, and connections it refused that
would in fact have fitted.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ["Obstacle", "disc", "capsule", "pad_obstacle", "track_obstacle", "arc_obstacle"]


@dataclass(frozen=True, slots=True)
class Obstacle:
    """A convex polygon inflated by ``r``. ``r`` already includes clearance and half a track."""

    vertices: tuple[tuple[float, float], ...]
    r: float
    net: int = 0
    label: str = ""

    # -- geometry ---------------------------------------------------------------------

    @property
    def centre(self) -> tuple[float, float]:
        xs = [v[0] for v in self.vertices]
        ys = [v[1] for v in self.vertices]
        return (sum(xs) / len(xs), sum(ys) / len(ys))

    @property
    def reach(self) -> float:
        """Radius of a circle about ``centre`` containing the whole inflated shape."""
        cx, cy = self.centre
        return max(math.hypot(x - cx, y - cy) for x, y in self.vertices) + self.r

    def distance_to_point(self, px: float, py: float) -> float:
        """Distance from a point to the *core* polygon, before inflation. 0 if inside."""
        count = len(self.vertices)
        if count == 1:
            vx, vy = self.vertices[0]
            return math.hypot(px - vx, py - vy)
        if count == 2:
            return _point_segment(px, py, *self.vertices[0], *self.vertices[1])
        if self._contains_core(px, py):
            return 0.0
        return min(
            _point_segment(px, py, *self.vertices[i], *self.vertices[(i + 1) % count])
            for i in range(count)
        )

    def _contains_core(self, px: float, py: float) -> bool:
        """Point-in-convex-polygon by consistent winding."""
        count = len(self.vertices)
        sign = 0
        for i in range(count):
            ax, ay = self.vertices[i]
            bx, by = self.vertices[(i + 1) % count]
            cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
            if cross > 1e-9:
                if sign < 0:
                    return False
                sign = 1
            elif cross < -1e-9:
                if sign > 0:
                    return False
                sign = -1
        return True

    def contains(self, px: float, py: float, tol: float = 1e-6) -> bool:
        """Whether a point lies inside the inflated shape."""
        return self.distance_to_point(px, py) < self.r - tol

    def edges(self):
        """Core polygon edges as (ax, ay, bx, by). Empty for a single vertex."""
        count = len(self.vertices)
        if count < 2:
            return
        limit = count if count > 2 else 1
        for i in range(limit):
            ax, ay = self.vertices[i]
            bx, by = self.vertices[(i + 1) % count]
            yield ax, ay, bx, by


def _point_segment(px, py, ax, ay, bx, by) -> float:
    dx, dy = bx - ax, by - ay
    span = dx * dx + dy * dy
    if span <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / span))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


# --------------------------------------------------------------------------- builders

def disc(x: float, y: float, r: float, net: int = 0, label: str = "") -> Obstacle:
    return Obstacle(vertices=((float(x), float(y)),), r=float(r), net=net, label=label)


def capsule(x1: float, y1: float, x2: float, y2: float, r: float,
            net: int = 0, label: str = "") -> Obstacle:
    if math.hypot(x2 - x1, y2 - y1) < 1e-9:
        return disc(x1, y1, r, net, label)
    return Obstacle(vertices=((float(x1), float(y1)), (float(x2), float(y2))),
                    r=float(r), net=net, label=label)


def pad_obstacle(pad, clearance_nm: float, half_track_nm: float) -> Obstacle:
    """A pad as its own rectangle, inflated -- not as the circle that swallows it.

    Circular and small pads collapse to a disc, which is the same shape and one wrap circle
    instead of four.
    """
    halo = clearance_nm + half_track_nm
    hx, hy = pad.size_x / 2.0, pad.size_y / 2.0

    if pad.shape == "circle" or max(hx, hy) < 1e-9:
        return disc(pad.x, pad.y, max(hx, hy) + halo, pad.net, f"{pad.footprint}.{pad.number}")

    # An oval is a capsule: a segment along its long axis, inflated by the short half-axis.
    if pad.shape == "oval":
        angle = math.radians(pad.angle)
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        if hx >= hy:
            reach, radius = hx - hy, hy
            ux, uy = cos_a, -sin_a
        else:
            reach, radius = hy - hx, hx
            ux, uy = sin_a, cos_a
        return capsule(pad.x - reach * ux, pad.y - reach * uy,
                       pad.x + reach * ux, pad.y + reach * uy,
                       radius + halo, pad.net, f"{pad.footprint}.{pad.number}")

    angle = math.radians(pad.angle)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    corners = []
    for lx, ly in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
        corners.append((pad.x + lx * cos_a + ly * sin_a,
                        pad.y - lx * sin_a + ly * cos_a))
    return Obstacle(vertices=tuple(corners), r=halo, net=pad.net,
                    label=f"{pad.footprint}.{pad.number}")


def track_obstacle(x1: float, y1: float, x2: float, y2: float, radius: float,
                   net: int, label: str = "track") -> Obstacle:
    """A finished straight track: one capsule, two wrap circles."""
    return capsule(x1, y1, x2, y2, radius, net, label)


def arc_obstacle(arc, radius: float, net: int, label: str = "track") -> list[Obstacle]:
    """A finished arc, as a short chain of capsules along it.

    A capsule chord cuts inside the arc it approximates, so the chords are placed on an
    *outset* circle -- radius scaled by 1/cos(half-step) -- which puts the chord's midpoint
    back on the true arc and keeps the covered region outside it. Erring the other way would
    let a later track graze copper that is really there.
    """
    steps = max(1, int(math.ceil(abs(arc.sweep) / 0.35)))
    half = abs(arc.sweep) / (2 * steps)
    outset = arc.r / math.cos(half) if half < 1.4 else arc.r

    points = []
    for i in range(steps + 1):
        theta = arc.start_angle + arc.sweep * i / steps
        points.append((arc.cx + outset * math.cos(theta),
                       arc.cy + outset * math.sin(theta)))
    return [capsule(*points[i], *points[i + 1], radius, net, label)
            for i in range(steps)]
