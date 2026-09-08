"""A net's own copper is never a wall to it.

The fan-cut pair: wire A runs along the middle, wire B connects a pad above to a pad
below A's run. Of different nets, B must go around one of A's terminals (the crossing
option does not exist in its graph). Of the SAME net, B goes straight through: copper
of one net may touch and may cross. On the 630-pad board the majority of weave
separations were nets walled in by their own earlier wires. Positions are read after
both wires are in -- every commit re-seats its doorways by rank.
"""
from taut.weave import Weave
from test_weave import MM, board, crosses, pad, polyline_of, rings


def _fan_cut(net_b):
    a1, a2 = pad(4 * MM, 20 * MM, net=1), pad(36 * MM, 20 * MM, net=1)
    b1, b2 = pad(20 * MM, 12 * MM, net=net_b), pad(20 * MM, 28 * MM, net=net_b)
    mesh = board([a1, a2, b1, b2])
    weave = Weave(mesh)
    sa, ga = (4 * MM, 20 * MM), (36 * MM, 20 * MM)
    sb, gb = (20 * MM, 12 * MM), (20 * MM, 28 * MM)
    first = weave.insert(1, sa, ga, rings(mesh, sa), rings(mesh, ga), net=1)
    second = weave.insert(2, sb, gb, rings(mesh, sb), rings(mesh, gb), net=net_b)
    assert first.found and second.found
    return crosses(polyline_of(weave, sa, ga, first), polyline_of(weave, sb, gb, second))


def test_same_net_wires_may_cross_each_other():
    assert _fan_cut(net_b=1) >= 1


def test_other_nets_still_cannot_cross():
    assert _fan_cut(net_b=2) == 0
