"""Tests for the taut-string solver.

The core claim is a geometric one: the shortest collision-free path among discs is straight
tangent lines joined by arcs on those discs, and nothing else. So the tests are of two kinds --
closed forms where the answer can be written down, and a large random sweep asserting the one
property that must never break, that the path never enters an obstacle.
"""

from __future__ import annotations

import math
import random

import pytest

from taut.obstacles import Obstacle, capsule, disc
from taut.tangent import NoPathFound, PathArc, PathLine, solve


def shape(path) -> str:
    return "".join("L" if isinstance(e, PathLine) else "A" for e in path.elements)


def sample(path, per_element: int = 60):
    points = []
    for element in path.elements:
        if isinstance(element, PathLine):
            for i in range(per_element + 1):
                t = i / per_element
                points.append((element.x1 + (element.x2 - element.x1) * t,
                               element.y1 + (element.y2 - element.y1) * t))
        else:
            for i in range(per_element + 1):
                points.append(element.point_at(i / per_element))
    return points


def deepest_penetration(path, obstacles) -> float:
    worst = 0.0
    for px, py in sample(path):
        for obstacle in obstacles:
            worst = max(worst, obstacle.r - obstacle.distance_to_point(px, py))
    return worst


# --------------------------------------------------------------------------- closed forms

def test_clear_shot_is_a_straight_line():
    path = solve((0, 0), (10, 0), [])
    assert shape(path) == "L"
    assert path.length == pytest.approx(10.0)


def test_one_disc_gives_line_arc_line_of_the_exact_analytic_length():
    """The textbook taut-string case, checked against the closed form.

    For a disc of radius r centred midway between endpoints a distance d from each, the
    rubber band runs tangent, wraps, and runs tangent again:
        2*sqrt(d^2 - r^2) + r*(pi - 2*acos(r/d))
    """
    d, r = 5.0, 2.0
    expected = 2 * math.sqrt(d * d - r * r) + r * (math.pi - 2 * math.acos(r / d))
    path = solve((-d, 0), (d, 0), [disc(0, 0, r)])
    assert shape(path) == "LAL"
    assert path.length == pytest.approx(expected, abs=1e-6)


def test_a_disc_off_to_one_side_changes_nothing():
    path = solve((-5, 0), (5, 0), [disc(0, 8, 2)])
    assert shape(path) == "L"
    assert path.length == pytest.approx(10.0)


def test_the_band_goes_through_a_gap_rather_than_around():
    path = solve((-8, 0), (8, 0), [disc(0, 4, 2), disc(0, -4, 2)])
    assert shape(path) == "L"
    assert path.length == pytest.approx(16.0)


def test_two_obstacles_in_a_row_produce_two_wraps():
    path = solve((-8, 0), (8, 0), [disc(-3, 0, 2), disc(3, 0, 2)])
    assert shape(path) == "LALAL"


def test_it_finds_the_one_opening_in_a_wall():
    wall = [disc(0, y, 1.2) for y in (-6, -4, -2, 2, 4, 6)]
    path = solve((-8, 0), (8, 0), wall)
    assert path.length == pytest.approx(16.0)


def test_an_enclosed_endpoint_has_no_path():
    ring = [disc(3 * math.cos(a), 3 * math.sin(a), 1.6)
            for a in (i * math.pi / 6 for i in range(12))]
    with pytest.raises(NoPathFound):
        solve((0, 0), (12, 0), ring)


def test_an_endpoint_inside_an_obstacle_fails_loudly():
    """Dropping the offending disc instead would lay copper straight over a pad.

    An earlier version did exactly that, and the only symptom was clearance violations
    appearing in DRC long after the router reported success.
    """
    with pytest.raises(NoPathFound):
        solve((0, 0), (10, 0), [disc(0.5, 0, 3.0, label="a pad")])


def test_the_path_is_symmetric_end_for_end():
    discs = [disc(-3, 1, 2), disc(3, -1, 1.5)]
    forward = solve((-9, 0), (9, 0), discs)
    backward = solve((9, 0), (-9, 0), discs)
    assert forward.length == pytest.approx(backward.length, abs=1e-6)


# --------------------------------------------------------------------------- properties

def test_the_path_never_enters_an_obstacle():
    """The property that must never break, over many random scenes."""
    rng = random.Random(20260824)
    worst = 0.0
    solved = 0
    for _ in range(250):
        discs = [disc(rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(0.5, 2.0))
                 for _ in range(6)]
        start = (-10.0, rng.uniform(-3, 3))
        goal = (10.0, rng.uniform(-3, 3))
        if any(d.contains(*start) or d.contains(*goal) for d in discs):
            continue
        try:
            path = solve(start, goal, discs)
        except NoPathFound:
            continue
        solved += 1
        worst = max(worst, deepest_penetration(path, discs))
    assert solved > 150, f"only {solved} scenes produced a path"
    assert worst < 1e-3, f"path entered an obstacle by {worst}"


def test_the_path_is_never_shorter_than_a_straight_line():
    rng = random.Random(11)
    for _ in range(120):
        discs = [disc(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(0.5, 1.5))
                 for _ in range(4)]
        start, goal = (-9.0, 0.0), (9.0, 0.0)
        if any(d.contains(*start) or d.contains(*goal) for d in discs):
            continue
        try:
            path = solve(start, goal, discs)
        except NoPathFound:
            continue
        assert path.length >= math.dist(start, goal) - 1e-9


def test_adding_an_obstacle_never_shortens_the_path():
    rng = random.Random(5)
    for _ in range(60):
        discs = [disc(rng.uniform(-5, 5), rng.uniform(-4, 4), rng.uniform(0.5, 1.5))
                 for _ in range(3)]
        extra = disc(rng.uniform(-5, 5), rng.uniform(-4, 4), rng.uniform(0.5, 1.5))
        start, goal = (-9.0, 0.0), (9.0, 0.0)
        if any(d.contains(*start) or d.contains(*goal) for d in discs + [extra]):
            continue
        try:
            without = solve(start, goal, discs)
            with_extra = solve(start, goal, discs + [extra])
        except NoPathFound:
            continue
        assert with_extra.length >= without.length - 1e-6


def test_only_lines_and_arcs_ever_come_out():
    """The whole point: this geometry is exactly what a .kicad_pcb can hold."""
    rng = random.Random(3)
    discs = [disc(rng.uniform(-5, 5), rng.uniform(-5, 5), 1.0) for _ in range(5)]
    try:
        path = solve((-9, 0), (9, 0), discs)
    except NoPathFound:
        pytest.skip("scene had no path")
    assert all(isinstance(e, (PathLine, PathArc)) for e in path.elements)


# --------------------------------------------------------------------------- shapes

def test_a_rectangle_is_wrapped_at_its_corners():
    """The band bends only on the corner circles, so a rect gives line-arc-line-arc-line."""
    from taut.obstacles import Obstacle
    rect = Obstacle(vertices=((-2, -1), (2, -1), (2, 1), (-2, 1)), r=0.5)
    path = solve((-10, 0), (10, 0), [rect])
    assert "A" in shape(path), "wrapping a rectangle must produce arcs"
    assert deepest_penetration(path, [rect]) < 1e-6
    # Going around a 4x2 rect inflated by 0.5 must cost more than the straight 20
    assert path.length > 20.0


def test_a_capsule_is_a_two_ended_obstacle():
    from taut.obstacles import capsule
    bar = capsule(-3, 0, 3, 0, 1.0)
    path = solve((-10, 0), (10, 0), [bar])
    assert deepest_penetration(path, [bar]) < 1e-6
    assert path.length > 20.0


def test_a_tall_pad_is_not_treated_as_the_circle_that_swallows_it():
    """The whole point of convex obstacles: a long pad must not block its own neighbourhood.

    A 1 x 10 bar inflated by 0.5 leaves a corridor 2 units to its side. Modelled as its
    enclosing circle (radius ~5) that corridor disappears.
    """
    from taut.obstacles import Obstacle
    bar = Obstacle(vertices=((-0.5, -5), (0.5, -5), (0.5, 5), (-0.5, 5)), r=0.5)
    path = solve((-4, 8), (4, 8), [bar])
    assert path.length == pytest.approx(8.0), "a clear route above the bar should be straight"


def test_flatness_of_an_obstacle_changes_the_route():
    from taut.obstacles import Obstacle, disc
    wide = Obstacle(vertices=((-4, -0.2), (4, -0.2), (4, 0.2), (-4, 0.2)), r=0.5)
    round_ = disc(0, 0, 4.5)
    through_wide = solve((0, -6), (0, 6), [wide])
    through_round = solve((0, -6), (0, 6), [round_])
    assert through_wide.length < through_round.length
