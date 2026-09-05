"""A net's own copper is never a wall to it.

Two connections of one net whose straight lines cross: with every chord a wall the
second must detour around the first's terminal; with same-net transparency it goes
straight through, crossing its sibling, which copper of one net may do. On the
630-pad board the majority of weave separations were nets walled in by their own
earlier wires. Positions are read after both wires are in -- every commit re-seats
its doorways by rank.
"""
from taut.weave import Weave
from test_weave import MM, board, crosses, pad, polyline_of, rings


def test_same_net_wires_may_cross_each_other():
    mesh = board([pad(4 * MM, 4 * MM, net=1), pad(36 * MM, 36 * MM, net=1),
                  pad(4 * MM, 36 * MM, net=1), pad(36 * MM, 4 * MM, net=1)])
    weave = Weave(mesh)
    a, b = (4 * MM, 4 * MM), (36 * MM, 36 * MM)
    c, d = (4 * MM, 36 * MM), (36 * MM, 4 * MM)
    first = weave.insert(1, a, b, rings(mesh, a), rings(mesh, b), net=1)
    second = weave.insert(2, c, d, rings(mesh, c), rings(mesh, d), net=1)
    assert first.found and second.found
    line_1 = polyline_of(weave, a, b, first)
    line_2 = polyline_of(weave, c, d, second)
    assert crosses(line_1, line_2) >= 1


def test_other_nets_still_cannot_cross():
    mesh = board([pad(4 * MM, 4 * MM, net=1), pad(36 * MM, 36 * MM, net=1),
                  pad(4 * MM, 36 * MM, net=2), pad(36 * MM, 4 * MM, net=2)])
    weave = Weave(mesh)
    a, b = (4 * MM, 4 * MM), (36 * MM, 36 * MM)
    c, d = (4 * MM, 36 * MM), (36 * MM, 4 * MM)
    first = weave.insert(1, a, b, rings(mesh, a), rings(mesh, b), net=1)
    second = weave.insert(2, c, d, rings(mesh, c), rings(mesh, d), net=2)
    assert second.found
    assert crosses(polyline_of(weave, a, b, first), polyline_of(weave, c, d, second)) == 0
