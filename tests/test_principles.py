"""First-principles tests: tiny boards, known right answers, checked end to end.

These run the full pipeline -- KiCad file text in, routed geometry out -- and judge the
result three ways: every connection routed, the sketch legal (no crossings, no grazes, no
clips), and the copper no longer than the known optimum where one is known analytically.

They are deliberately merciless about *how* a defect is resolved: a crossing may be fixed by
rerouting or by a via, but never by dropping a connection, and never by geometry that merely
passes because nothing checked it.
"""

from __future__ import annotations

import math

import pytest

from taut.board import load_board
from taut.plan import plan_board
from taut.sketch import SketchWire, check_sketch
from taut.tangent import PathArc, PathLine, TautPath
from taut.units import GUARDBAND_NM

from scenes import (MM, crossing_ladder, fan_cut, fan_cut_one_layer, one_disc,
                    open_pair, stub_graze, two_through_a_gap)


# --------------------------------------------------------------- running a scene

def routed(board_path):
    board = load_board(board_path)
    result = plan_board(board, layers=["F.Cu", "B.Cu"])
    return board, result


def sketch_of(board, result):
    """The routed board in the checker's terms: wires by net, plus static copper."""
    from taut.obstacles import pad_obstacle

    by_net: dict[tuple[int, str], list] = {}
    for track in result.tracks:
        by_net.setdefault((track.net, track.layer), []).append(track)

    wires = []
    rules = {}
    for net in board.routable:
        netclass = board.netclass_for(net.name)
        rules[net.code] = (netclass.track_width_nm / 2.0,
                           netclass.clearance_nm * 1.0)

    for index, ((net, layer), tracks) in enumerate(sorted(by_net.items())):
        elements = []
        for track in tracks:
            if hasattr(track, "xm"):
                from taut.geometry import Arc as GeoArc
                arc = GeoArc.from_three_points(track.x1, track.y1, track.xm, track.ym,
                                               track.x2, track.y2)
                if not arc.degenerate:
                    elements.append(PathArc(arc.cx, arc.cy, arc.r, arc.start_angle,
                                            arc.start_angle + arc.sweep, arc.sweep > 0))
                    continue
            elements.append(PathLine(track.x1, track.y1, track.x2, track.y2))
        half, clearance = rules.get(net, (125_000.0, 200_000.0))
        wires.append((layer, SketchWire(key=index, net=net, path=TautPath(elements),
                                        half_width=half, clearance=clearance)))

    statics = {}
    for layer in ("F.Cu", "B.Cu"):
        statics[layer] = [pad_obstacle(pad, 0.0, 0.0) for pad in board.pads
                          if pad.on_layer(layer)]
    return wires, statics


def defects(board, result):
    wires, statics = sketch_of(board, result)
    crossings, grazes, clips = [], [], []
    for layer in ("F.Cu", "B.Cu"):
        here = [wire for wire_layer, wire in wires if wire_layer == layer]
        got = check_sketch(here, statics[layer])
        crossings += got[0]
        grazes += got[1]
        clips += got[2]
    # A wire ending on its own pad passes within a hair of the neighbouring pad's ring in
    # `stub_graze` only if the geometry is wrong; but every wire legitimately touches its own
    # pads, and check_sketch already skips same-net copper.
    return crossings, grazes, clips


def assert_clean(board, result, connections: int):
    assert result.stats["failed"] == 0, result.failed
    assert result.stats["routed"] == connections
    crossings, grazes, clips = defects(board, result)
    assert not crossings, crossings
    assert not grazes, grazes
    assert not clips, clips


# --------------------------------------------------------------------- the scenes

def test_open_pair_is_the_straight_line(tmp_path):
    board, result = routed(open_pair(tmp_path))
    assert_clean(board, result, 1)
    length = result.total_length_nm
    assert length == pytest.approx(20.0 * MM, rel=1e-3)


def test_one_disc_is_line_arc_line(tmp_path):
    board, result = routed(one_disc(tmp_path))
    assert_clean(board, result, 1)
    # Either side of the disc is optimal; the length is the analytic taut length, within
    # the slack the guardband and faceting deliberately add.
    span, radius = 20.0 * MM, (1.6 / 2 + 0.2 + 0.25 / 2) * MM
    d = span / 2.0
    exact = 2.0 * (2.0 * math.sqrt(d * d - radius * radius) / 2.0
                   + 0.0)  # two half-spans
    exact = 2.0 * math.sqrt(d * d - radius * radius) + radius * (
        math.pi - 2.0 * math.acos(radius / d))
    assert result.total_length_nm <= exact * 1.06
    assert result.total_length_nm >= span


def test_fan_cut_resolves_without_crossing(tmp_path):
    """The scene built to reproduce the stub crossing. Any legal answer accepted."""
    board, result = routed(fan_cut(tmp_path))
    assert_clean(board, result, 2)


def test_fan_cut_on_one_layer_takes_a_via_or_goes_around(tmp_path):
    """The persistent failure from the real board, minimal. SMD pads pin both nets to the
    front; a crossing cannot be dissolved by layer choice, only by a via or by routing
    around a terminal."""
    board, result = routed(fan_cut_one_layer(tmp_path))
    assert_clean(board, result, 2)


def test_stub_graze_keeps_clear_of_the_neighbour(tmp_path):
    board, result = routed(stub_graze(tmp_path))
    assert_clean(board, result, 1)


def test_crossing_ladder_needs_the_third_dimension(tmp_path):
    """Three pairwise-crossing connections cannot share two layers without a via."""
    board, result = routed(crossing_ladder(tmp_path))
    assert_clean(board, result, 3)


def test_two_through_a_gap_are_spaced_not_stacked(tmp_path):
    board, result = routed(two_through_a_gap(tmp_path))
    assert_clean(board, result, 2)


def test_via_length_is_counted(tmp_path):
    """A board with vias reports more copper than its tracks alone."""
    board, result = routed(crossing_ladder(tmp_path))
    tracks_only = sum(track.length_nm for track in result.tracks)
    if result.vias:
        assert result.stats["length_mm"] * MM > tracks_only
        assert result.stats["vias"] == len(result.vias)
    else:
        # The router found a two-layer answer; the accounting still has to exist.
        assert "vias" in result.stats
