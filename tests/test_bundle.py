"""The scenario sequential routing cannot solve, and the bundle's answer to it.

A gap wide enough for two tracks. Route one net through it alone and a rubber band does what
rubber bands do -- it sits in the middle. The second net then finds the gap full, and no
amount of re-ordering or ripping up helps, because the taut path is deterministic: put the
first net back and it lands in exactly the same place.

The fix is not to route them in a better order. It is to count the gap, seat both nets in it,
and route each one against a problem that already contains the other.
"""

from __future__ import annotations

import math

import pytest

from taut.obstacles import Obstacle, disc
from taut.portals import assign_slots, build_portals, core_distance, crossings
from taut.tangent import NoPathFound, solve

CLEARANCE = 1.0
WIDTH = 1.0
#: A gap this wide holds floor((D - c) / (w + c)) = floor((5.5 - 1) / 2) = 2 tracks.
GAP = 5.5


def walls(gap: float = GAP) -> list[Obstacle]:
    """Two blocks with a corridor of ``gap`` between their cores, at x = 0."""
    half = gap / 2.0
    lower = Obstacle(vertices=((-6.0, -20.0), (6.0, -20.0), (6.0, -half), (-6.0, -half)),
                     r=0.0, net=0, label="lower wall")
    upper = Obstacle(vertices=((-6.0, half), (6.0, half), (6.0, 20.0), (-6.0, 20.0)),
                     r=0.0, net=0, label="upper wall")
    return [lower, upper]


def inflate(obstacles: list[Obstacle], r: float, extra: dict[int, float] | None = None):
    extra = extra or {}
    return [Obstacle(vertices=o.vertices, r=r + extra.get(i, 0.0), net=o.net, label=o.label)
            for i, o in enumerate(obstacles)]


# --------------------------------------------------------------------------- capacity

def test_a_gap_is_counted_not_guessed():
    lower, upper = walls()
    assert core_distance(lower, upper) == pytest.approx(GAP)

    portals = build_portals(inflate([lower, upper], CLEARANCE + WIDTH / 2),
                            CLEARANCE, WIDTH)
    assert portals, "the corridor must be recognised as a gap"
    assert portals[0].capacity == 2, "5.5 wide with 1.0 clearance and 1.0 tracks holds two"


def test_a_narrow_gap_holds_one_and_a_wide_gap_holds_more():
    def capacity(gap):
        lower, upper = walls(gap)
        portals = build_portals(inflate([lower, upper], CLEARANCE + WIDTH / 2),
                                CLEARANCE, WIDTH)
        return portals[0].capacity if portals else 0

    assert capacity(3.5) == 1
    assert capacity(5.5) == 2
    assert capacity(7.5) == 3


# --------------------------------------------------------------------------- the scenario

def test_the_first_net_alone_sits_in_the_middle_of_the_gap():
    """This is the behaviour that strands the second net, stated as a fact about geometry."""
    obstacles = inflate(walls(), CLEARANCE + WIDTH / 2)
    path = solve((-15.0, 0.0), (15.0, 0.0), obstacles)
    ys = []
    for element in path.elements:
        if hasattr(element, "y1"):
            ys.extend([element.y1, element.y2])
    assert max(abs(y) for y in ys) < 1e-6, "a lone rubber band runs straight down the middle"


def test_sequentially_the_second_net_cannot_fit():
    """With the first net treated as solid copper, the remaining space is too narrow."""
    obstacles = inflate(walls(), CLEARANCE + WIDTH / 2)
    first = solve((-15.0, 0.0), (15.0, 0.0), obstacles)

    from taut.route import _path_obstacles
    occupied = _path_obstacles(first, CLEARANCE + WIDTH, net=1)

    with pytest.raises(NoPathFound):
        solve((-15.0, -0.2), (15.0, 0.2), obstacles + occupied)


def test_seated_in_slots_both_nets_fit():
    """The bundle's answer: count the gap, seat both, and route each with its own share.

    Neither net is pushed. Each is simply given a problem that already contains the other,
    and the taut path does the rest.
    """
    cores = walls()
    base_r = CLEARANCE + WIDTH / 2
    obstacles = inflate(cores, base_r)

    portals = build_portals(obstacles, CLEARANCE, WIDTH)
    assert portals[0].capacity == 2

    # Both nets want the corridor.
    wanted = {
        0: crossings(solve((-15.0, -0.2), (15.0, -0.2), obstacles), portals),
        1: crossings(solve((-15.0, 0.2), (15.0, 0.2), obstacles), portals),
    }
    assert all(wanted[k] for k in wanted), "both nets must be seen to want the gap"

    plan = assign_slots(portals, wanted, CLEARANCE, WIDTH)
    assert plan.shared == 1
    assert not plan.oversubscribed, "two nets in a gap that holds two is not oversubscribed"

    paths = []
    for connection in (0, 1):
        extra = {i: plan.inflation(connection, i) for i in range(len(obstacles))}
        seated = inflate(cores, base_r, extra)
        paths.append(solve((-15.0, 0.0), (15.0, 0.0), seated))

    assert len(paths) == 2

    # Measure inside the corridor, not over the whole path -- both nets start and end on
    # the centre line, so averaging the lot would just report zero.
    def y_at_middle(path):
        best = None
        for element in path.elements:
            if hasattr(element, "y1"):
                for t in (i / 20 for i in range(21)):
                    x = element.x1 + (element.x2 - element.x1) * t
                    y = element.y1 + (element.y2 - element.y1) * t
                    if best is None or abs(x) < abs(best[0]):
                        best = (x, y)
            else:
                for t in (i / 20 for i in range(21)):
                    x, y = element.point_at(t)
                    if best is None or abs(x) < abs(best[0]):
                        best = (x, y)
        return best[1]

    a, b = y_at_middle(paths[0]), y_at_middle(paths[1])
    assert a * b < 0, f"the two nets must take opposite sides of the gap, got {a} and {b}"
    assert abs(a - b) >= WIDTH + CLEARANCE - 1e-6, \
        f"they must be at least a track pitch apart, got {abs(a - b)}"


def test_an_oversubscribed_gap_is_reported_rather_than_overfilled():
    """Three nets into a two-track gap does not silently become three tracks."""
    cores = walls()
    obstacles = inflate(cores, CLEARANCE + WIDTH / 2)
    portals = build_portals(obstacles, CLEARANCE, WIDTH)

    wanted = {
        k: crossings(solve((-15.0, off), (15.0, off), obstacles), portals)
        for k, off in enumerate((-0.3, 0.0, 0.3))
    }
    plan = assign_slots(portals, wanted, CLEARANCE, WIDTH)
    assert plan.oversubscribed, "three into a gap of two must be flagged"
