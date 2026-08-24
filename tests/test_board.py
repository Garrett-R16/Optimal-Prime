"""End-to-end: read a real KiCad board, route it, and let KiCad judge the result.

There is no internal DRC here and there is not going to be one. The router emits a board and
``kicad-cli`` says whether it is legal; that is the only verdict that matters.
"""

from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from taut.board import load_board
from taut.emit import write_board
from taut.route import ArcTrack, Track, route_board

DEMOS = Path(os.environ.get("KICAD_DEMOS",
                            r"C:\Program Files\KiCad\9.0\share\kicad\demos"))

ROUTING_RULES = {
    "clearance", "shorting_items", "tracks_crossing", "track_dangling", "via_dangling",
    "copper_edge_clearance", "hole_clearance", "hole_to_hole", "track_width",
    "items_not_allowed", "copper_sliver", "isolated_copper", "solder_mask_bridge",
}

pytestmark = pytest.mark.skipif(not DEMOS.exists(), reason="KiCad demos not installed")


def find_cli() -> Path | None:
    override = os.environ.get("KICAD_CLI")
    if override and Path(override).exists():
        return Path(override)
    found = shutil.which("kicad-cli")
    if found:
        return Path(found)
    hint = Path(r"C:\Program Files\KiCad\9.0\bin\kicad-cli.exe")
    return hint if hint.exists() else None


def demo(name: str) -> Path:
    matches = glob.glob(str(DEMOS / "**" / f"{name}.kicad_pcb"), recursive=True)
    if not matches:
        pytest.skip(f"demo board {name} not installed")
    return Path(matches[0])


@pytest.fixture(scope="module")
def ecc83():
    return load_board(demo("ecc83-pp"))


# --------------------------------------------------------------------------- reading

def test_board_reads(ecc83):
    assert ecc83.copper_layers == ("F.Cu", "B.Cu")
    assert len(ecc83.pads) > 20
    assert len(ecc83.routable) >= 8
    assert ecc83.edges, "the board outline must be extracted, or copper runs off the board"


def test_every_pad_lands_inside_the_board():
    """A wrong footprint-rotation sign displaces pads silently; this catches it."""
    board = load_board(demo("ecc83-pp"))
    x0, y0, x1, y1 = board.bbox_nm
    for pad in board.pads:
        assert x0 - 1_000_000 <= pad.x <= x1 + 1_000_000
        assert y0 - 1_000_000 <= pad.y <= y1 + 1_000_000


def test_netclass_patterns_are_honoured():
    """KiCad 9 assigns netclasses by glob; missing it gives every net the Default rules."""
    board = load_board(demo("pic_programmer"))
    assert board.patterns, "pic_programmer defines netclass patterns"
    power = board.netclass_for("GND")
    default = board.netclass_for("Net-(D1-Pad1)")
    assert power.clearance_nm > default.clearance_nm


# --------------------------------------------------------------------------- routing

def test_two_layer_route_is_drc_clean(ecc83, tmp_path):
    """The headline result: a whole board routed as taut strings, judged by KiCad."""
    cli = find_cli()
    if cli is None:
        pytest.skip("kicad-cli not found")

    result = route_board(ecc83, layers=["F.Cu", "B.Cu"])
    assert not result.failed, f"unrouted connections: {result.failed[:3]}"
    assert result.arc_count > 0, "a taut route around obstacles must contain arcs"

    board_out = tmp_path / "routed.kicad_pcb"
    write_board(ecc83, result, board_out)

    report = board_out.with_suffix(".drc.json")
    subprocess.run([str(cli), "pcb", "drc", "--format", "json", "--severity-all",
                    "--units", "mm", "-o", str(report), str(board_out)],
                   capture_output=True, text=True, timeout=900)
    drc = json.loads(report.read_text(encoding="utf-8"))

    violations = [v for v in drc["violations"] if v["type"] in ROUTING_RULES]
    assert not violations, f"DRC errors: {[v['type'] for v in violations[:5]]}"
    assert not drc["unconnected_items"], "every net must be fully connected"


def test_single_layer_route_is_drc_clean_on_what_it_routes(ecc83, tmp_path):
    """One layer cannot finish this board, but nothing it does place may be illegal."""
    cli = find_cli()
    if cli is None:
        pytest.skip("kicad-cli not found")

    result = route_board(ecc83, layers=["F.Cu"])
    assert result.routed, "a single layer should still route most of the board"

    board_out = tmp_path / "single.kicad_pcb"
    write_board(ecc83, result, board_out)
    report = board_out.with_suffix(".drc.json")
    subprocess.run([str(cli), "pcb", "drc", "--format", "json", "--severity-all",
                    "--units", "mm", "-o", str(report), str(board_out)],
                   capture_output=True, text=True, timeout=900)
    drc = json.loads(report.read_text(encoding="utf-8"))

    violations = [v for v in drc["violations"] if v["type"] in ROUTING_RULES]
    assert not violations, f"DRC errors: {[v['type'] for v in violations[:5]]}"


def test_output_is_only_segments_and_arcs(ecc83):
    result = route_board(ecc83, layers=["F.Cu", "B.Cu"])
    assert all(isinstance(t, (Track, ArcTrack)) for t in result.tracks)


def test_routing_is_deterministic(ecc83):
    first = route_board(ecc83, layers=["F.Cu", "B.Cu"])
    second = route_board(ecc83, layers=["F.Cu", "B.Cu"])
    assert first.stats == second.stats
    assert len(first.tracks) == len(second.tracks)


def test_arcs_survive_the_round_trip(ecc83, tmp_path):
    """Arcs must reach the file as native (arc ...), not flattened to polylines."""
    result = route_board(ecc83, layers=["F.Cu"])
    board_out = tmp_path / "arcs.kicad_pcb"
    write_board(ecc83, result, board_out)
    text = board_out.read_text(encoding="utf-8")
    assert text.count("(arc") >= result.arc_count
    assert "(mid " in text
