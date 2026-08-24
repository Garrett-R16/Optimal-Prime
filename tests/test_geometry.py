"""Level 0 -- geometric predicates against a brute-force reference and closed forms.

Two kinds of check, because neither alone is enough. Random cases validated against dense
sampling catch structural errors but cannot catch a systematic bias (sampling has its own
error in the same direction). Closed forms catch bias but only where someone thought to look.

The property that matters most is one-directional: **a computed distance may be smaller than
the truth, never larger.** A router that thinks copper is closer than it is loses
completions; one that thinks it is further apart emits violations it never saw.
"""

from __future__ import annotations

import math
import random

import pytest

from arena import geometry as g

SCALE = 100.0
SAMPLES = 150


# --------------------------------------------------------------------------- helpers

def sample_segment(ax, ay, bx, by, n=SAMPLES):
    return [(ax + (bx - ax) * i / n, ay + (by - ay) * i / n) for i in range(n + 1)]


def sample_arc(arc, n=SAMPLES):
    if arc.degenerate:
        return sample_segment(arc.x1, arc.y1, arc.x2, arc.y2, n)
    return [arc.point_at_angle(arc.start_angle + arc.sweep * i / n) for i in range(n + 1)]


def brute_force(first, second):
    return min(math.hypot(bx - ax, by - ay) for ax, ay in first for bx, by in second)


def random_point(rng, scale=SCALE):
    return rng.uniform(-scale, scale), rng.uniform(-scale, scale)


def random_arc(rng, scale=SCALE):
    """A random arc, sometimes degenerate, often reflex."""
    if rng.random() < 0.08:
        ax, ay = random_point(rng, scale)
        bx, by = random_point(rng, scale)
        return g.Arc.from_three_points(ax, ay, (ax + bx) / 2, (ay + by) / 2, bx, by)
    cx, cy = random_point(rng, scale / 2)
    radius = rng.uniform(0.5, scale / 2)
    start = rng.uniform(0, g.TAU)
    sweep = rng.uniform(-1.95 * math.pi, 1.95 * math.pi)
    if abs(sweep) < 0.02:
        sweep = 0.5

    def at(t):
        theta = start + sweep * t
        return cx + radius * math.cos(theta), cy + radius * math.sin(theta)

    (sx, sy), (mx, my), (ex, ey) = at(0.0), at(0.5), at(1.0)
    return g.Arc.from_three_points(sx, sy, mx, my, ex, ey)


def sampling_slack(*primitives) -> float:
    """How far the sampled reference may exceed the truth, given the sample spacing."""
    return sum(p / SAMPLES for p in primitives) + 1e-6


# --------------------------------------------------------------------------- closed forms

def test_parallel_segments():
    assert g.segment_segment(0, 0, 10, 0, 0, 3, 10, 3) == pytest.approx(3.0)


def test_crossing_segments_are_zero():
    assert g.segment_segment(-5, 0, 5, 0, 0, -5, 0, 5) == 0.0


def test_touching_segments_are_zero():
    assert g.segment_segment(0, 0, 5, 0, 5, 0, 5, 5) == 0.0


def test_collinear_disjoint_segments():
    assert g.segment_segment(0, 0, 1, 0, 4, 0, 6, 0) == pytest.approx(3.0)


def test_semicircle_from_three_points():
    arc = g.Arc.from_three_points(1, 0, 0, 1, -1, 0)
    assert arc.cx == pytest.approx(0.0, abs=1e-9)
    assert arc.cy == pytest.approx(0.0, abs=1e-9)
    assert arc.r == pytest.approx(1.0)
    assert abs(arc.sweep) == pytest.approx(math.pi)
    assert arc.length == pytest.approx(math.pi)


def test_reflex_arc_keeps_its_long_way_round():
    """An arc whose midpoint is on the far side must sweep more than pi, not less."""
    arc = g.Arc.from_three_points(1, 0, -1, 0, 0, -1)
    assert abs(arc.sweep) > math.pi
    assert arc.length == pytest.approx(1.5 * math.pi)


def test_collinear_three_points_degenerate_to_a_segment():
    arc = g.Arc.from_three_points(0, 0, 5, 0, 10, 0)
    assert arc.degenerate
    assert arc.length == pytest.approx(10.0)
    assert g.point_arc(5, 4, arc) == pytest.approx(4.0)


def test_point_outside_and_inside_a_circle():
    full = g.Arc.from_three_points(1, 0, -1, 0, 0, -1)  # 3/4 turn, radius 1
    assert g.point_arc(3, 0, full) == pytest.approx(2.0)   # outside, on the arc's span
    assert g.point_arc(0, 0, full) == pytest.approx(1.0)   # the centre


def test_point_off_the_arcs_span_falls_back_to_an_endpoint():
    quarter = g.Arc.from_three_points(1, 0, math.sqrt(0.5), math.sqrt(0.5), 0, 1)
    # Directly below the centre: outside the quarter's span, so the nearest point is (1, 0).
    assert g.point_arc(0, -5, quarter) == pytest.approx(math.hypot(1, 5))


def test_concentric_arcs():
    inner = g.Arc.from_three_points(1, 0, 0, 1, -1, 0)
    outer = g.Arc.from_three_points(3, 0, 0, 3, -3, 0)
    assert g.arc_arc(inner, outer) == pytest.approx(2.0)


def test_externally_tangent_arcs_touch():
    """Unit circles centred 2 apart meet at exactly one point, and both arcs span it."""
    left = g.Arc.from_three_points(1, 0, -1, 0, 0, -1)      # centre (0,0), r=1
    right = g.Arc.from_three_points(1, 0, 2, 1, 3, 0)       # centre (2,0), r=1
    assert g.arc_arc(left, right) == pytest.approx(0.0, abs=1e-6)


def test_separated_circles_measure_the_gap_between_them():
    left = g.Arc.from_three_points(1, 0, -1, 0, 0, -1)      # centre (0,0), r=1
    right = g.Arc.from_three_points(3, 0, 5, 0, 4, 1)       # centre (4,0), r=1
    assert g.arc_arc(left, right) == pytest.approx(2.0)


def test_segment_tangent_to_an_arc():
    arc = g.Arc.from_three_points(1, 0, 0, 1, -1, 0)   # upper unit semicircle
    assert g.segment_arc(-5, 1, 5, 1, arc) == pytest.approx(0.0, abs=1e-6)
    assert g.segment_arc(-5, 3, 5, 3, arc) == pytest.approx(2.0)


def test_segment_crossing_an_arc_is_zero():
    arc = g.Arc.from_three_points(1, 0, 0, 1, -1, 0)
    assert g.segment_arc(0, -5, 0, 5, arc) == 0.0


def test_segment_missing_the_arcs_span():
    """A chord below a purely upper semicircle must measure to an endpoint, not the circle."""
    arc = g.Arc.from_three_points(1, 0, 0, 1, -1, 0)
    assert g.segment_arc(-0.5, -2, 0.5, -2, arc) == pytest.approx(math.hypot(0.5, 2))


def test_zero_length_segment_behaves_as_a_point():
    assert g.point_segment(3, 4, 0, 0, 0, 0) == pytest.approx(5.0)
    assert g.segment_segment(0, 0, 0, 0, 3, 4, 3, 4) == pytest.approx(5.0)


# --------------------------------------------------------------------------- random

@pytest.mark.parametrize("trials", [2000])
def test_point_segment_matches_brute_force(trials):
    rng = random.Random(1)
    for _ in range(trials):
        px, py = random_point(rng)
        ax, ay = random_point(rng)
        bx, by = random_point(rng)
        got = g.point_segment(px, py, ax, ay, bx, by)
        ref = min(math.hypot(px - x, py - y)
                  for x, y in sample_segment(ax, ay, bx, by))
        assert got <= ref + 1e-9, "computed distance exceeded the sampled reference"
        assert got >= ref - sampling_slack(math.hypot(bx - ax, by - ay))


@pytest.mark.parametrize("trials", [800])
def test_segment_segment_matches_brute_force(trials):
    rng = random.Random(2)
    for _ in range(trials):
        a = random_point(rng)
        b = random_point(rng)
        c = random_point(rng)
        d = random_point(rng)
        got = g.segment_segment(*a, *b, *c, *d)
        ref = brute_force(sample_segment(*a, *b), sample_segment(*c, *d))
        assert got <= ref + 1e-9
        assert got >= ref - sampling_slack(math.dist(a, b), math.dist(c, d))


@pytest.mark.parametrize("trials", [2000])
def test_point_arc_matches_brute_force(trials):
    rng = random.Random(3)
    for _ in range(trials):
        px, py = random_point(rng)
        arc = random_arc(rng)
        got = g.point_arc(px, py, arc)
        ref = min(math.hypot(px - x, py - y) for x, y in sample_arc(arc))
        assert got <= ref + 1e-9
        assert got >= ref - sampling_slack(arc.length)


@pytest.mark.parametrize("trials", [700])
def test_segment_arc_matches_brute_force(trials):
    rng = random.Random(4)
    for _ in range(trials):
        a = random_point(rng)
        b = random_point(rng)
        arc = random_arc(rng)
        got = g.segment_arc(*a, *b, arc)
        ref = brute_force(sample_segment(*a, *b), sample_arc(arc))
        assert got <= ref + 1e-9
        assert got >= ref - sampling_slack(math.dist(a, b), arc.length)


@pytest.mark.parametrize("trials", [600])
def test_arc_arc_matches_brute_force(trials):
    rng = random.Random(5)
    for _ in range(trials):
        first = random_arc(rng)
        second = random_arc(rng)
        got = g.arc_arc(first, second)
        ref = brute_force(sample_arc(first), sample_arc(second))
        assert got <= ref + 1e-9
        assert got >= ref - sampling_slack(first.length, second.length)


def test_distances_are_symmetric():
    rng = random.Random(6)
    for _ in range(300):
        first, second = random_arc(rng), random_arc(rng)
        assert g.arc_arc(first, second) == pytest.approx(g.arc_arc(second, first), abs=1e-9)
