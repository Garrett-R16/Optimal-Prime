"""The rubber-band embedding: exact geometry, and offsets that come from rank."""

from __future__ import annotations

import itertools
import math

import pytest

from taut.rubberband import (Arc, Crossing, Wire, _arc_to_arc, _point_to_arc,
                             rubberband, spacing_between, to_geometry)


def wire(key: int = 0, net: int = 1, half: float = 1.5, clear: float = 1.5) -> Wire:
    return Wire(key=key, net=net, half_width=half, clearance=clear)


# --------------------------------------------------------------------- tangent construction

@pytest.mark.parametrize("r1,r2", [(3.0, 2.0), (2.0, 3.0), (3.0, 3.0)])
@pytest.mark.parametrize("ccw1,ccw2", list(itertools.product([True, False], repeat=2)))
def test_bitangent_is_exact(r1, r2, ccw1, ccw2):
    """Same wrap direction gives the outer tangent, opposite gives the crossed one."""
    gap = 10.0
    a, b = Arc(0.0, 0.0, r1, ccw1), Arc(gap, 0.0, r2, ccw2)
    (px, py), (qx, qy) = _arc_to_arc(a, b)

    expected = (math.sqrt(gap ** 2 - (r1 - r2) ** 2) if ccw1 == ccw2
                else math.sqrt(gap ** 2 - (r1 + r2) ** 2))
    assert math.hypot(qx - px, qy - py) == pytest.approx(expected, abs=1e-9)

    # A tangent meets each circle at a right angle to its radius.
    assert (px - a.cx) * (qx - px) + (py - a.cy) * (qy - py) == pytest.approx(0.0, abs=1e-9)
    assert (qx - b.cx) * (qx - px) + (qy - b.cy) * (qy - py) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("ccw", [True, False])
@pytest.mark.parametrize("leaving", [True, False])
def test_point_to_arc_is_exact(ccw, leaving):
    arc = Arc(0.0, 0.0, 3.0, ccw)
    px, py = 10.0, 0.0
    tx, ty = _point_to_arc(px, py, arc, leaving)
    assert math.hypot(tx - px, ty - py) == pytest.approx(math.sqrt(100.0 - 9.0), abs=1e-9)
    assert (tx - arc.cx) * (px - tx) + (ty - arc.cy) * (py - ty) == pytest.approx(0.0, abs=1e-9)


def test_point_inside_the_circle_has_no_tangent():
    assert _point_to_arc(1.0, 0.0, Arc(0.0, 0.0, 3.0, True), False) is None


# ------------------------------------------------------------------------- the taut path

def test_single_obstacle_matches_the_analytic_length():
    """Two points either side of one disc: line, arc, line, and the length is known exactly."""
    reach, radius, nudge = 10.0, 3.0, 1e-6
    crossing = Crossing(ax=0.0, ay=0.0, bx=0.0, by=100.0,
                        order=(wire(half=radius / 2, clear=radius / 2),), mine=0)
    start, goal = (-reach, -nudge), (reach, -nudge)

    length = sum(piece.length for piece
                 in to_geometry(start, goal, rubberband(start, goal, [crossing])))

    span = math.hypot(reach, nudge)
    exact = (2.0 * math.sqrt(span ** 2 - radius ** 2)
             + radius * (math.pi - 2.0 * math.acos(radius / span)))
    assert length == pytest.approx(exact, abs=1e-6)


def test_a_clear_chord_is_left_straight():
    """Nothing to wrap means nothing is wrapped -- the answer is the straight line."""
    start, goal = (0.0, 0.0), (100.0, 0.0)
    crossing = Crossing(ax=50.0, ay=-40.0, bx=50.0, by=40.0, order=(wire(),), mine=0)
    pieces = to_geometry(start, goal, rubberband(start, goal, [crossing]))
    assert len(pieces) == 1
    assert pieces[0].length == pytest.approx(100.0)


def test_no_doorways_at_all_is_the_straight_line():
    pieces = to_geometry((0.0, 0.0), (10.0, 5.0), rubberband((0.0, 0.0), (10.0, 5.0), []))
    assert len(pieces) == 1
    assert pieces[0].length == pytest.approx(math.hypot(10.0, 5.0))


# ------------------------------------------------------------------ offsets come from rank

def test_spacing_ignores_wires_of_the_same_net():
    same = wire(key=1, net=7), wire(key=2, net=7)
    assert spacing_between(*same) == 0.0


def test_offset_is_the_stack_between_the_wire_and_that_corner():
    """The outermost wire stands off by everything between it and the far corner."""
    wires = tuple(wire(key=index, net=index + 1, half=0.5, clear=0.5) for index in range(3))
    crossing = Crossing(ax=0.0, ay=0.0, bx=0.0, by=100.0, order=wires, mine=0)

    # Nearest the `a` end: its own half width and clearance, and nothing else.
    assert crossing.offset_from(toward_a=True) == pytest.approx(1.0)
    # Toward `b`: two gaps of (0.5 + 0.5 + 0.5) plus its own 1.0 at the corner.
    assert crossing.offset_from(toward_a=False) == pytest.approx(1.5 + 1.5 + 1.0)


def test_the_same_wire_stands_off_differently_at_each_corner():
    """Rank, not position: reverse who is in the middle and the offsets swap."""
    wires = tuple(wire(key=index, net=index + 1, half=0.5, clear=0.5) for index in range(3))
    outer = Crossing(ax=0.0, ay=0.0, bx=0.0, by=100.0, order=wires, mine=0)
    inner = Crossing(ax=0.0, ay=0.0, bx=0.0, by=100.0, order=wires, mine=2)
    assert outer.offset_from(True) == pytest.approx(inner.offset_from(False))
    assert outer.offset_from(False) == pytest.approx(inner.offset_from(True))


def test_copper_radius_at_a_corner_is_added_to_the_offset():
    """A shape the mesh describes by its centre still keeps the wire off its copper."""
    bare = Crossing(ax=0.0, ay=0.0, bx=0.0, by=100.0, order=(wire(half=0.5, clear=0.5),), mine=0)
    skinned = Crossing(ax=0.0, ay=0.0, bx=0.0, by=100.0, order=(wire(half=0.5, clear=0.5),),
                       mine=0, ra=2.0)
    assert skinned.offset_from(True) - bare.offset_from(True) == pytest.approx(2.0)


def test_a_doorway_never_asks_for_more_room_than_it_has():
    """Six wires through a gap that fits three still produce offsets inside the gap."""
    wires = tuple(wire(key=index, net=index + 1, half=0.5, clear=0.5) for index in range(6))
    crossing = Crossing(ax=0.0, ay=0.0, bx=0.0, by=4.0, order=wires, mine=0)
    assert crossing.offset_from(False) <= 4.0
    assert crossing.point()[1] <= 4.0
