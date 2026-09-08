"""The cell decomposition: chords cut a triangle, sides become unrepresentable."""

from __future__ import annotations

import pytest

from taut.cells import Chord, TriangleCells, edge_parameter, point_parameter


def test_an_empty_triangle_is_one_cell():
    cells = TriangleCells.build([])
    assert cells.reachable(0.5, 1.5)
    assert cells.reachable(0.1, 2.9)
    assert len({interval.cell for interval in cells.intervals}) == 1


def test_one_chord_makes_two_cells():
    """A wire from edge 0 to edge 1 separates edge 2 from the corner between 0 and 1."""
    cells = TriangleCells.build([Chord(0.5, 1.5)])
    # between the chord's feet: the corner-1 side
    assert cells.reachable(0.7, 1.2)
    # outside them: the edge-2 side
    assert cells.reachable(0.2, 2.5)
    # and never across
    assert not cells.reachable(0.7, 2.5)
    assert not cells.reachable(1.2, 0.2)


def test_nested_chords_make_three_cells():
    """Two wires through the same pair of edges: inner, middle, outer."""
    cells = TriangleCells.build([Chord(0.4, 1.6), Chord(0.6, 1.4)])
    assert len({interval.cell for interval in cells.intervals}) == 3
    assert cells.reachable(0.5, 1.5)          # between the chords
    assert not cells.reachable(0.5, 1.0)      # middle vs inner
    assert not cells.reachable(0.5, 0.2)      # middle vs outer
    assert not cells.reachable(1.0, 2.5)      # inner vs outer


def test_a_stub_chord_splits_its_own_corner_into_sectors():
    """The fan sectors of the reference systems, from plain arithmetic.

    A stub from corner 0 to the opposite edge: the boundary just before the corner and
    just after it are now different cells -- a wire hugging the pad must pick a side.
    """
    cells = TriangleCells.build([Chord(0.0, 1.5)])
    sectors = cells.cells_at_corner(0)
    assert len(sectors) == 2
    assert not cells.reachable(0.05, 2.95)
    # both sectors still reach the opposite edge -- on their own sides of the stub
    assert cells.reachable(0.05, 1.4)
    assert cells.reachable(2.95, 1.6)


def test_a_corner_without_a_stub_is_one_sector():
    cells = TriangleCells.build([Chord(0.5, 1.5)])
    assert len(cells.cells_at_corner(2)) == 1


def test_two_wires_through_one_doorway_keep_their_order():
    """Both chords cross edge 0 and edge 2; entering between them stays between them."""
    cells = TriangleCells.build([Chord(0.3, 2.7), Chord(0.6, 2.4)])
    assert cells.reachable(0.45, 2.55)        # the lane between the two wires
    assert not cells.reachable(0.45, 0.1)     # not the outside lane
    assert not cells.reachable(0.45, 1.5)     # not the far side


def test_edge_and_point_parameters():
    corners = [(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]
    assert edge_parameter(0, 0.5) == pytest.approx(0.5)
    assert edge_parameter(2, 0.25) == pytest.approx(2.25)
    # midpoint of edge 0
    assert point_parameter(corners, 5.0, 0.0) == pytest.approx(0.5)
    # a point near corner 2 lands at parameter ~2.0 (start of edge 2 back to corner 0)
    t = point_parameter(corners, 0.2, 9.5)
    assert 1.8 <= t <= 2.2


def test_reachability_is_symmetric_and_reflexive():
    cells = TriangleCells.build([Chord(0.5, 1.5), Chord(1.7, 2.6)])
    spots = [0.2, 0.8, 1.2, 1.6, 2.0, 2.8]
    for a in spots:
        assert cells.reachable(a, a)
        for b in spots:
            assert cells.reachable(a, b) == cells.reachable(b, a)
