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


# --------------------------------------------------------------------------- conventions

def test_pads_never_overlap_each_other():
    """Pinning the pad-orientation convention by a property the boards guarantee.

    KiCad reports no overlapping pads on these demos, so a correct pad model cannot make two
    overlap. Treating a pad's declared angle as an offset to be *added* to its footprint's
    invents 26 overlaps on sonde xilinx and, downstream, 23 DRC errors from routing into
    copper that was modelled 90 degrees out.
    """
    import math

    from taut.obstacles import Obstacle
    from taut.tangent import segment_to_obstacle

    def core(pad):
        hx, hy = pad.size_x / 2.0, pad.size_y / 2.0
        a = math.radians(pad.angle)
        ca, sa = math.cos(a), math.sin(a)
        return Obstacle(vertices=tuple(
            (pad.x + lx * ca + ly * sa, pad.y - lx * sa + ly * ca)
            for lx, ly in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))), r=0.0)

    for name in ("sonde xilinx", "interf_u", "pic_programmer"):
        try:
            board = load_board(demo(name))
        except Exception:
            continue
        pads = [p for p in board.pads
                if p.shape in ("rect", "roundrect", "trapezoid")
                and abs(p.size_x - p.size_y) > 50_000]
        overlaps = 0
        for i in range(len(pads)):
            for j in range(i + 1, len(pads)):
                a, b = pads[i], pads[j]
                if a.net == b.net:
                    continue
                if not any(a.on_layer(L) and b.on_layer(L) for L in board.copper_layers):
                    continue
                if math.dist((a.x, a.y), (b.x, b.y)) > a.radius_nm + b.radius_nm:
                    continue
                ca, cb = core(a), core(b)
                touching = any(segment_to_obstacle(cb, *e) <= 0.0 for e in ca.edges()) or \
                    any(segment_to_obstacle(ca, *e) <= 0.0 for e in cb.edges())
                overlaps += touching
        assert overlaps == 0, f"{name}: pad model invents {overlaps} pad-pad overlaps"


def test_copper_graphics_are_obstacles():
    """Text drawn on a copper layer belongs to no net, so copper must clear it."""
    board = load_board(demo("sonde xilinx"))
    assert board.copper_shapes, "sonde xilinx carries copper text on F.Cu"
    assert all(s.layer.endswith(".Cu") for s in board.copper_shapes)


def test_near_flat_arcs_are_emitted_as_segments():
    """A nearly-collinear three-point arc reconstructs unstably; emit its chord instead."""
    import math

    from taut.route import MIN_SAGITTA_NM, _path_to_tracks
    from taut.tangent import PathArc, TautPath

    flat = PathArc(cx=0.0, cy=0.0, r=1_000_000.0, start_angle=0.0,
                   end_angle=0.002, ccw=True)
    assert flat.r * (1 - math.cos(abs(flat.sweep) / 2)) < MIN_SAGITTA_NM
    pieces = _path_to_tracks(TautPath([flat]), net=1, layer="F.Cu", width_nm=250_000)
    assert pieces and all(isinstance(p, Track) for p in pieces)

    curved = PathArc(cx=0.0, cy=0.0, r=1_000_000.0, start_angle=0.0,
                     end_angle=1.2, ccw=True)
    pieces = _path_to_tracks(TautPath([curved]), net=1, layer="F.Cu", width_nm=250_000)
    assert any(isinstance(p, ArcTrack) for p in pieces)


def test_a_denser_board_routes_clean():
    """sonde xilinx: 26 nets, rotated connector pads, copper text. DRC must be clean."""
    cli = find_cli()
    if cli is None:
        pytest.skip("kicad-cli not found")
    board = load_board(demo("sonde xilinx"))
    result = route_board(board, layers=["F.Cu", "B.Cu"])
    assert len(result.routed) > 55

    out = Path(__file__).parent / "_dense.kicad_pcb"
    try:
        write_board(board, result, out)
        report = out.with_suffix(".drc.json")
        subprocess.run([str(cli), "pcb", "drc", "--format", "json", "--severity-all",
                        "--units", "mm", "-o", str(report), str(out)],
                       capture_output=True, text=True, timeout=900)
        drc = json.loads(report.read_text(encoding="utf-8"))
        violations = [v for v in drc["violations"] if v["type"] in ROUTING_RULES]
        assert not violations, f"DRC errors: {[v['type'] for v in violations[:5]]}"
    finally:
        for suffix in (".kicad_pcb", ".kicad_pro", ".drc.json"):
            out.with_suffix(suffix).unlink(missing_ok=True)
