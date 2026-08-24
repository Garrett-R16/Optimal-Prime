"""The free space, cut into triangles, so that *which side* becomes a thing you can name.

Everything before this computed geometry and hoped topology would fall out of it. Each net
was handed the obstacles and asked for its shortest path, which silently chose a homotopy
class -- which side of every pad to pass -- and embedded it in the same indivisible step. For
one net that is fine. For a board it is not: two nets wanting the same side of the same pad
are in *topological* conflict, and no amount of sliding paths around resolves it. One of them
has to go the other way, and nothing in a geometric solver can express that.

So the free space is triangulated over the obstacle corners. A triangle is free if its
interior is outside every obstacle's copper; the routing *clearance* is not baked in here,
because that belongs to the embedding.

A shape's copper is not always its vertices. A round pad is one vertex at its centre carrying
a radius, and an oval is two -- so a mesh built from vertices alone models those pads as
points, triangulates straight through their copper, and measures portal spans centre to
centre. On a board where most pads are round, that is most of the mesh: 31 of 88 triangles
"free" while containing solid copper, 74 of 120 portals with their span buried in a pad. Every
vertex therefore carries the radius of the shape that owns it, and freeness, portal span and
capacity are all measured from the copper boundary rather than from the vertex. Two free triangles that share an edge are connected by a **portal**, and a
net's route becomes a sequence of portals -- which *is* its homotopy class, written down
where a search can reason about it.

Each portal carries a capacity: with a span of ``L`` between two obstacle corners, tracks of
width ``w`` needing clearance ``c`` from the corners and from each other,

    n = floor((L - 2c - w) / (w + c)) + 1     when L >= 2c + w, otherwise 0

That number is what makes contention a countable, allocatable resource instead of something
discovered by collision after the fact.

The triangulation is unconstrained Delaunay, which means an edge between two obstacle corners
can pass straight *through* a third obstacle. Such an edge is not a doorway, and a route using
it goes through copper -- so any portal whose span crosses an obstacle it does not belong to
is given zero capacity and never appears as a way through.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import Delaunay

from .obstacles import Obstacle
from .tangent import segment_to_obstacle

__all__ = ["Portal", "Mesh", "build_mesh"]


@dataclass(frozen=True, slots=True)
class Portal:
    """The shared edge between two free triangles: a doorway with a track count."""

    left: int              # triangle index
    right: int             # triangle index
    a: int                 # vertex index
    b: int                 # vertex index
    length: float
    capacity: int

    def key(self) -> tuple[int, int]:
        return (self.a, self.b) if self.a < self.b else (self.b, self.a)


@dataclass
class Mesh:
    points: np.ndarray                     # (N, 2) vertex coordinates
    owner: list[int]                       # obstacle index per vertex, -1 for the outline
    radius: list[float]                    # copper radius of the shape owning each vertex
    triangles: np.ndarray                  # (M, 3) vertex indices
    free: np.ndarray                       # (M,) bool
    neighbours: np.ndarray                 # (M, 3) triangle indices, -1 for none
    portals: dict[tuple[int, int], Portal] = field(default_factory=dict)
    obstacles: list[Obstacle] = field(default_factory=list)

    # ------------------------------------------------------------------ lookup

    def triangle_at(self, x: float, y: float) -> int:
        """The free triangle containing a point, or the nearest one if it sits on an edge."""
        best = -1
        best_d = math.inf
        for index in range(len(self.triangles)):
            if not self.free[index]:
                continue
            if self._contains(index, x, y):
                return index
            cx, cy = self.centroid(index)
            d = math.hypot(cx - x, cy - y)
            if d < best_d:
                best_d, best = d, index
        return best

    def _contains(self, tri: int, x: float, y: float) -> bool:
        a, b, c = (self.points[i] for i in self.triangles[tri])
        sign = 0
        for p, q in ((a, b), (b, c), (c, a)):
            cross = (q[0] - p[0]) * (y - p[1]) - (q[1] - p[1]) * (x - p[0])
            if cross > 1e-9:
                if sign < 0:
                    return False
                sign = 1
            elif cross < -1e-9:
                if sign > 0:
                    return False
                sign = -1
        return True

    def centroid(self, tri: int) -> tuple[float, float]:
        pts = self.points[self.triangles[tri]]
        return float(pts[:, 0].mean()), float(pts[:, 1].mean())

    def portal_between(self, tri_a: int, tri_b: int) -> Portal | None:
        shared = sorted(set(self.triangles[tri_a]) & set(self.triangles[tri_b]))
        if len(shared) != 2:
            return None
        return self.portals.get((shared[0], shared[1]))

    def adjacent(self, tri: int):
        """Free neighbours of a triangle, with the portal joining them."""
        for other in self.neighbours[tri]:
            if other < 0 or not self.free[other]:
                continue
            portal = self.portal_between(tri, int(other))
            if portal is not None and portal.capacity > 0:
                yield int(other), portal


def _capacity(span: float, clearance: float, width: float) -> int:
    """How many tracks fit across a doorway, given the *copper-to-copper* span."""
    usable = span - 2.0 * clearance - width
    if usable < 0.0:
        return 0
    return int(usable // (width + clearance)) + 1


def build_mesh(obstacles: list[Obstacle], outline: list[tuple[float, float]],
               clearance: float, width: float) -> Mesh:
    """Triangulate the free space between ``obstacles``, inside ``outline``."""
    points: list[tuple[float, float]] = []
    owner: list[int] = []
    radius: list[float] = []

    for index, obstacle in enumerate(obstacles):
        for vx, vy in obstacle.vertices:
            points.append((float(vx), float(vy)))
            owner.append(index)
            radius.append(float(obstacle.r))
    for vx, vy in outline:
        points.append((float(vx), float(vy)))
        owner.append(-1)
        radius.append(0.0)

    if len(points) < 3:
        raise ValueError("need at least three points to triangulate")

    array = np.asarray(points, dtype=float)
    tri = Delaunay(array, qhull_options="Qbb Qc Qz")

    free = np.ones(len(tri.simplices), dtype=bool)
    for index, simplex in enumerate(tri.simplices):
        cx, cy = array[simplex][:, 0].mean(), array[simplex][:, 1].mean()
        for obstacle in obstacles:
            # Against the copper, which for a disc or capsule extends `r` beyond the vertices
            # the triangulation was built from. Testing the bare vertices instead calls a
            # triangle sitting squarely inside a round pad free.
            if obstacle.distance_to_point(cx, cy) <= obstacle.r:
                free[index] = False
                break

    mesh = Mesh(points=array, owner=owner, radius=radius, triangles=tri.simplices,
                free=free, neighbours=tri.neighbors, obstacles=list(obstacles))

    for index, simplex in enumerate(tri.simplices):
        if not free[index]:
            continue
        for slot, other in enumerate(tri.neighbors[index]):
            if other < 0 or not free[other] or other < index:
                continue
            shared = sorted(set(simplex) & set(tri.simplices[other]))
            if len(shared) != 2:
                continue
            key = (shared[0], shared[1])
            if key in mesh.portals:
                continue
            pa, pb = array[shared[0]], array[shared[1]]
            length = float(math.hypot(pb[0] - pa[0], pb[1] - pa[1]))

            # The usable doorway runs from copper to copper, not vertex to vertex. For two
            # round pads facing each other, the vertex span is centre to centre and counts
            # both radii of solid copper as though a track could use them.
            span = length - radius[shared[0]] - radius[shared[1]]
            capacity = _capacity(span, clearance, width) if span > 0.0 else 0

            # A span that cuts through an obstacle is not a doorway. The obstacles owning the
            # two endpoints are exempt only for their own copper, which the span subtraction
            # above has already accounted for.
            owners = {owner[shared[0]], owner[shared[1]]}
            for oi, obstacle in enumerate(obstacles):
                if oi in owners:
                    continue
                if segment_to_obstacle(obstacle, float(pa[0]), float(pa[1]),
                                       float(pb[0]), float(pb[1])) <= obstacle.r:
                    capacity = 0
                    break

            mesh.portals[key] = Portal(left=index, right=int(other),
                                       a=shared[0], b=shared[1], length=length,
                                       capacity=capacity)

    return mesh
