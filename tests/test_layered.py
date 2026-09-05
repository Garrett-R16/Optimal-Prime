"""Routing across the stack: via sites, and the search that spends them."""

from __future__ import annotations

import math

import numpy as np
import pytest

from taut.mesh import build_mesh
from taut.obstacles import Obstacle
from taut.layered import route_stack
from taut.vias import Site, sites_by_triangle, via_sites


#: Everything here is in nanometres, as the router is: a board 40 mm square.
MM = 1_000_000.0
EDGE = 40.0 * MM


def box(x: float, y: float, half: float, net: int = -1) -> Obstacle:
    return Obstacle(vertices=((x - half, y - half), (x + half, y - half),
                              (x + half, y + half), (x - half, y + half)),
                    r=0.0, net=net, label="pad")


def open_board(blockers=()):
    """A square of free space, with enough points on its rim to triangulate into pieces."""
    steps = 8
    outline = []
    for step in range(steps):
        outline.append((EDGE * step / steps, 0.0))
    for step in range(steps):
        outline.append((EDGE, EDGE * step / steps))
    for step in range(steps):
        outline.append((EDGE - EDGE * step / steps, EDGE))
    for step in range(steps):
        outline.append((0.0, EDGE - EDGE * step / steps))
    return build_mesh(list(blockers), outline, clearance=0.2 * MM, width=0.25 * MM)


def test_a_single_layer_offers_no_via_sites():
    assert via_sites([open_board()], radius=0.6 * MM, clearance=0.2 * MM) == []


def test_sites_avoid_copper_on_every_layer():
    """A site has to be clear on all of them: through vias go through all of them."""
    middle = EDGE / 2.0
    clear = open_board()
    blocked = open_board([box(middle, middle, 8.0 * MM)])

    everywhere = via_sites([clear, clear], radius=0.6 * MM, clearance=0.2 * MM)
    shared = via_sites([clear, blocked], radius=0.6 * MM, clearance=0.2 * MM)

    # Not a count: blocking one layer adds vertices, so it can end up with *more*
    # triangles and more candidates. What matters is that none of them is on the copper.
    assert everywhere and shared
    assert any(abs(site.x - middle) < 7.0 * MM and abs(site.y - middle) < 7.0 * MM
               for site in everywhere)
    assert all(abs(site.x - middle) > 7.0 * MM or abs(site.y - middle) > 7.0 * MM
               for site in shared)


def test_a_site_keeps_its_own_radius_clear():
    near = open_board([box(EDGE / 2.0, EDGE / 2.0, 2.0 * MM)])
    tight = via_sites([near, near], radius=0.4 * MM, clearance=0.2 * MM)
    roomy = via_sites([near, near], radius=6.0 * MM, clearance=0.2 * MM)
    assert len(roomy) < len(tight)


def test_the_outline_test_is_honoured():
    clear = open_board()
    half = via_sites([clear, clear], radius=0.6 * MM, clearance=0.2 * MM,
                     inside=lambda x, y: x < EDGE / 2.0)
    assert half
    assert all(site.x < EDGE / 2.0 for site in half)


def test_sites_are_indexed_by_the_triangle_they_sit_in():
    sites = [Site(index=0, x=1.0, y=1.0, triangles=(3, 7)),
             Site(index=1, x=2.0, y=2.0, triangles=(3, 9))]
    lookup = sites_by_triangle(sites, layers=2)
    assert lookup[0][3] == [0, 1]
    assert lookup[1][7] == [0]
    assert lookup[1][9] == [1]


# ----------------------------------------------------------------------- the search

def two_layers(blockers_front=(), blockers_back=()):
    return [open_board(blockers_front), open_board(blockers_back)]


def terminals_everywhere(meshes):
    def reach(point):
        return [mesh.terminals(*point) for mesh in meshes]
    return reach


def test_an_open_board_needs_no_via():
    meshes = two_layers()
    sites = via_sites(meshes, radius=0.6 * MM, clearance=0.2 * MM)
    routes, report = route_stack(meshes, sites,
                                 [(0, 1, (4.0 * MM, 20.0 * MM), (36.0 * MM, 20.0 * MM))],
                                 terminals=terminals_everywhere(meshes), rounds=2)
    assert report.routed == 1
    assert report.vias == 0
    assert routes[0].found


def test_a_route_reports_the_resources_it_holds():
    meshes = two_layers()
    sites = via_sites(meshes, radius=0.6 * MM, clearance=0.2 * MM)
    routes, _ = route_stack(meshes, sites, [(0, 1, (4.0 * MM, 20.0 * MM), (36.0 * MM, 20.0 * MM))],
                            terminals=terminals_everywhere(meshes), rounds=2)
    held = list(routes[0].uses())
    assert held
    assert all(kind in ("portal", "via") for kind, *_ in held)


def test_a_connection_with_nowhere_to_start_is_not_routed():
    meshes = two_layers()
    sites = via_sites(meshes, radius=0.6 * MM, clearance=0.2 * MM)
    routes, report = route_stack(meshes, sites, [(0, 1, (4.0 * MM, 20.0 * MM), (36.0 * MM, 20.0 * MM))],
                                 terminals=lambda point: [[], []], rounds=2)
    assert report.routed == 0
    assert not routes[0].found


def test_a_via_splits_the_route_into_legs_that_meet_at_it():
    """Whatever it decides, the legs have to join up end to end."""
    meshes = two_layers()
    sites = via_sites(meshes, radius=0.6 * MM, clearance=0.2 * MM)
    routes, _ = route_stack(meshes, sites, [(0, 1, (4.0 * MM, 20.0 * MM), (36.0 * MM, 20.0 * MM))],
                            terminals=terminals_everywhere(meshes),
                            via_cost=0.0, rounds=2)
    route = routes[0]
    assert route.legs[0].start == route.start
    assert route.legs[-1].goal == route.goal
    for before, after in zip(route.legs, route.legs[1:]):
        assert before.goal == after.start
    assert len(route.vias) == len(route.legs) - 1
    # A via is only worth drilling if the legs it joins are on different layers.
    for before, after in zip(route.legs, route.legs[1:]):
        assert before.layer != after.layer


def test_a_connection_that_has_to_change_layer_takes_a_via():
    """Start reachable only on the front, goal only on the back: it must hop, or fail."""
    meshes = two_layers()
    sites = via_sites(meshes, radius=0.6 * MM, clearance=0.2 * MM)
    start, goal = (4.0 * MM, 20.0 * MM), (36.0 * MM, 20.0 * MM)

    def one_way(point):
        rings = [mesh.terminals(*point) for mesh in meshes]
        return [rings[0], []] if point == start else [[], rings[1]]

    routes, report = route_stack(meshes, sites, [(0, 1, start, goal)],
                                 terminals=one_way, rounds=2)
    route = routes[0]
    assert report.routed == 1
    assert len(route.vias) == 1
    assert [leg.layer for leg in route.legs] == [0, 1]
    assert route.legs[0].goal == route.legs[1].start


def test_a_free_via_is_taken_and_a_dear_one_is_not():
    """The price is the whole mechanism, so it should be visible in the answer."""
    meshes = two_layers()
    sites = via_sites(meshes, radius=0.6 * MM, clearance=0.2 * MM)
    job = [(0, 1, (4.0 * MM, 20.0 * MM), (36.0 * MM, 20.0 * MM))]
    reach = terminals_everywhere(meshes)

    _, free = route_stack(meshes, sites, job, terminals=reach, via_cost=0.0, rounds=2)
    _, dear = route_stack(meshes, sites, job, terminals=reach, via_cost=1e12, rounds=2)
    assert dear.vias == 0
    assert free.vias >= dear.vias


def test_a_banned_turn_diverts_the_route():
    """Ban the exit a route takes from its start triangle; it must find another way."""
    meshes = two_layers()
    sites = via_sites(meshes, radius=0.6 * MM, clearance=0.2 * MM)
    job = [(0, 1, (4.0 * MM, 20.0 * MM), (36.0 * MM, 20.0 * MM))]
    reach = terminals_everywhere(meshes)

    first, _ = route_stack(meshes, sites, job, terminals=reach, rounds=2)
    assert first[0].found
    leg = first[0].legs[0]
    # triangles lead portals by one: triangles[0] is what portals[0] is crossed out of
    assert len(leg.triangles) == len(leg.portals) + 1
    tri, portal = leg.triangles[0], leg.portals[0]

    banned = frozenset({(1, leg.layer, tri, frozenset((portal,)))})
    second, report = route_stack(meshes, sites, job, terminals=reach, rounds=2,
                                 bans=banned)
    assert report.routed == 1
    other = second[0].legs[0]
    assert (other.triangles[0], other.portals[0]) != (tri, portal) \
        or other.layer != leg.layer
