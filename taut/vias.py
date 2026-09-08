"""Where a connection may change layer.

Assigning each connection a layer and leaving it there can only ever two-colour the graph of
which nets would cross which. That is not a routing algorithm, it is a bet that the graph is
bipartite, and on a real board it is not: three nets that pairwise cross cannot be laid on two
faces however cleverly the faces are chosen, and one of them will be left unrouted with nothing
wrong with it except that it had nowhere to go. A via is what makes the problem soluble rather
than merely lucky, and it is the reason a topological router can claim to be looking for the
best answer rather than the best answer of a particular shape.

A **site** is a place a via could physically go: free of copper on every layer it passes
through, by its own radius plus the clearance its net needs, and inside the board. Through vias
only, so a site is either available to all layers or to none -- which also means a via placed
here takes the spot out of the free space of every layer at once, and that cost is real. Sites
are found once, from the geometry, and then treated as a resource the topology negotiates over
exactly as it negotiates over doorways: a site holds one net, and wanting one that is taken
costs more each round until somebody goes elsewhere.

Candidates are the centroids of free triangles. That is not a lattice chosen for tidiness --
a triangle of the free space is a place with room in it, and its centroid is the point in that
triangle furthest from the three things bounding it, which is exactly where a via wants to be.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .mesh import Mesh

__all__ = ["Site", "via_sites"]

#: Two sites closer than this are the same opportunity, and keeping both only makes the
#: search do the same work twice.
_MERGE_NM = 250_000.0


@dataclass(frozen=True, slots=True)
class Site:
    """A place a via could go, and the triangle it sits in on each layer."""

    index: int
    x: float
    y: float
    #: triangle containing the site, per layer, in the order the layers were given
    triangles: tuple[int, ...]

    def triangle_on(self, layer: int) -> int:
        return self.triangles[layer]


def _room_for(mesh: Mesh, x: float, y: float, reach: float) -> bool:
    """Whether a via of this reach clears every piece of copper on one layer."""
    for obstacle in mesh.obstacles:
        if obstacle.distance_to_point(x, y) - obstacle.r < reach:
            return False
    return True


def via_sites(meshes: list[Mesh], radius: float, clearance: float,
              inside=None) -> list[Site]:
    """Every place a through via could be dropped, on the free space of all layers at once.

    ``inside`` is an optional test for the board outline; a via outside it is no use however
    much copper it clears.
    """
    if len(meshes) < 2:
        return []

    reach = radius + clearance
    seen: dict[tuple[int, int], None] = {}
    found: list[Site] = []

    for mesh in meshes:
        for index in range(len(mesh.triangles)):
            if not mesh.free[index]:
                continue
            x, y = mesh.centroid(index)

            cell = (int(x // _MERGE_NM), int(y // _MERGE_NM))
            if cell in seen:
                continue

            if inside is not None and not inside(x, y):
                continue

            where: list[int] = []
            for other in meshes:
                triangle = other.triangle_at(x, y)
                if triangle < 0 or not other.free[triangle]:
                    break
                if not other._contains(triangle, x, y):
                    break
                if not _room_for(other, x, y, reach):
                    break
                where.append(triangle)
            else:
                seen[cell] = None
                found.append(Site(index=len(found), x=x, y=y, triangles=tuple(where)))

    return found


def sites_by_triangle(sites: list[Site], layers: int) -> list[dict[int, list[int]]]:
    """Which sites sit in which triangle, per layer, so the search can find them quickly."""
    out: list[dict[int, list[int]]] = [{} for _ in range(layers)]
    for site in sites:
        for layer, triangle in enumerate(site.triangles):
            out[layer].setdefault(triangle, []).append(site.index)
    return out


def spacing_between_sites(a: Site, b: Site) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)
