"""The weave: sequential insertion where crossing a committed wire is unrepresentable."""

from __future__ import annotations

import math

import pytest

from taut.mesh import build_mesh
from taut.obstacles import Obstacle
from taut.weave import Weave

MM = 1_000_000.0
EDGE = 40.0 * MM


def pad(x: float, y: float, r: float = 0.8 * MM, net: int = 1) -> Obstacle:
    return Obstacle(vertices=((x, y),), r=r, net=net, label="pad")


def board(obstacles):
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
    return build_mesh(list(obstacles), outline, clearance=0.2 * MM, width=0.25 * MM)


def polyline_of(weave, start, goal, result):
    points = [start]
    for key, fraction in result.crossings:
        points.append(weave._lane_point(key, fraction))
    points.append(goal)
    return points


def crosses(a, b):
    def seg(p, q, r, s):
        def wind(ax, ay, bx, by, cx, cy):
            return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        d1 = wind(*r, *s, *p)
        d2 = wind(*r, *s, *q)
        d3 = wind(*p, *q, *r)
        d4 = wind(*p, *q, *s)
        return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))
    return sum(1 for i in range(len(a) - 1) for j in range(len(b) - 1)
               if seg(a[i], a[i + 1], b[j], b[j + 1]))


def rings(mesh, point):
    return mesh.terminals(point[0], point[1])


def test_a_lone_wire_walks_straight_across():
    p1, p2 = pad(4 * MM, 20 * MM), pad(36 * MM, 20 * MM)
    mesh = board([p1, p2])
    weave = Weave(mesh)
    start, goal = (4 * MM, 20 * MM), (36 * MM, 20 * MM)
    got = weave.insert(1, start, goal, rings(mesh, start), rings(mesh, goal))
    assert got.found
    line = polyline_of(weave, start, goal, got)
    length = sum(math.dist(a, b) for a, b in zip(line, line[1:]))
    assert length <= 32 * MM * 1.05


def test_the_second_wire_cannot_cross_the_first():
    """The fan-cut pair, at weave level: the crossing option must not exist.

    Wire A runs along the middle; wire B connects a pad above to a pad below A's run.
    B's straight line crosses A, so B must route around one of A's terminals -- and the
    result must show zero intersections, not because a checker caught one but because the
    search graph never contained one.
    """
    a1, a2 = pad(4 * MM, 20 * MM, net=1), pad(36 * MM, 20 * MM, net=1)
    b1, b2 = pad(20 * MM, 12 * MM, net=2), pad(20 * MM, 28 * MM, net=2)
    mesh = board([a1, a2, b1, b2])
    weave = Weave(mesh)

    sa, ga = (4 * MM, 20 * MM), (36 * MM, 20 * MM)
    first = weave.insert(1, sa, ga, rings(mesh, sa), rings(mesh, ga))
    assert first.found

    sb, gb = (20 * MM, 12 * MM), (20 * MM, 28 * MM)
    second = weave.insert(2, sb, gb, rings(mesh, sb), rings(mesh, gb))
    assert second.found

    line_a = polyline_of(weave, sa, ga, first)
    line_b = polyline_of(weave, sb, gb, second)
    assert crosses(line_a, line_b) == 0
    # and B paid for the detour: it is longer than its 16 mm straight line
    length_b = sum(math.dist(p, q) for p, q in zip(line_b, line_b[1:]))
    assert length_b > 16 * MM * 1.5


def test_two_parallel_wires_share_their_doorways_in_order():
    """Co-travelling wires squeezed through one gap: both fit, uncrossed, in order."""
    mesh = board([pad(4 * MM, 18 * MM, net=1), pad(36 * MM, 18 * MM, net=1),
                  pad(4 * MM, 22 * MM, net=2), pad(36 * MM, 22 * MM, net=2),
                  pad(20 * MM, 12 * MM, r=6 * MM, net=9),
                  pad(20 * MM, 28 * MM, r=6 * MM, net=9)])
    weave = Weave(mesh)
    s1, g1 = (4 * MM, 18 * MM), (36 * MM, 18 * MM)
    s2, g2 = (4 * MM, 22 * MM), (36 * MM, 22 * MM)
    one = weave.insert(1, s1, g1, rings(mesh, s1), rings(mesh, g1))
    two = weave.insert(2, s2, g2, rings(mesh, s2), rings(mesh, g2))
    assert one.found and two.found
    assert crosses(polyline_of(weave, s1, g1, one),
                   polyline_of(weave, s2, g2, two)) == 0
    shared = [key for key, order in weave.order().items() if len(order) == 2]
    assert shared, "the parallel pair should share at least one doorway"


def test_a_truly_separated_wire_reports_failure():
    """Three mutually-crossing connections on one layer: the third must fail its weave,
    honestly, rather than cross."""
    mesh = board([pad(4 * MM, 4 * MM, net=1), pad(36 * MM, 36 * MM, net=1),
                  pad(4 * MM, 36 * MM, net=2), pad(36 * MM, 4 * MM, net=2),
                  pad(20 * MM, 2 * MM, net=3), pad(20 * MM, 38 * MM, net=3)])
    weave = Weave(mesh)
    jobs = [((4 * MM, 4 * MM), (36 * MM, 36 * MM)),
            ((4 * MM, 36 * MM), (36 * MM, 4 * MM)),
            ((20 * MM, 2 * MM), (20 * MM, 38 * MM))]
    results = []
    lines = []
    for index, (s, g) in enumerate(jobs, start=1):
        got = weave.insert(index, s, g, rings(mesh, s), rings(mesh, g))
        results.append(got)
        if got.found:
            lines.append(polyline_of(weave, s, g, got))
    # whatever routed, routed planar
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            assert crosses(lines[i], lines[j]) == 0
