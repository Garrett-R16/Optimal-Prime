"""P0 exit criteria, as executable tests.

These are the tests that decide whether any later number can be trusted. If the oracle does
not catch a deliberately broken board, every strategy result downstream is meaningless.

They shell out to KiCad and are correspondingly slow (a few seconds per board), so they are
grouped under the ``oracle`` marker.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from arena import oracle, score, sexpr
from arena.emit import clear_routing, deterministic_uuid, emit
from arena.problem import load_problem
from arena.solution import Segment, Solution, Via

PIC = Path(r"C:\Program Files\KiCad\9.0\share\kicad\demos\pic_programmer\pic_programmer.kicad_pcb")

pytestmark = [
    pytest.mark.oracle,
    pytest.mark.skipif(not PIC.exists(), reason="KiCad demo boards not installed"),
]


@pytest.fixture(scope="module")
def prob():
    return load_problem(PIC)


def _place(tree, tmp_path: Path, name: str, project: Path) -> Path:
    board = tmp_path / f"{name}.kicad_pcb"
    board.write_text(sexpr.dumps(tree), encoding="utf-8", newline="\n")
    shutil.copy(project, board.with_suffix(".kicad_pro"))
    return board


@pytest.fixture(scope="module")
def bare_result(prob, tmp_path_factory):
    """The board with all routing stripped -- the baseline every score is measured against."""
    tmp = tmp_path_factory.mktemp("bare")
    tree = sexpr.parse(sexpr.dumps(prob.tree))
    removed = clear_routing(tree)
    assert removed > 100, "expected a routed demo board to strip a lot of copper"
    return oracle.run_drc(_place(tree, tmp, "bare", prob.project_path))


# --------------------------------------------------------------- criterion 1: round trip

def test_reemitted_board_scores_identically(prob, tmp_path):
    """P0 exit criterion 1: emitting the parsed tree changes nothing KiCad can see."""
    original = tmp_path / "orig.kicad_pcb"
    shutil.copy(PIC, original)
    shutil.copy(prob.project_path, original.with_suffix(".kicad_pro"))

    reemitted = _place(prob.tree, tmp_path, "reemitted", prob.project_path)

    before = oracle.run_drc(original)
    after = oracle.run_drc(reemitted)

    assert before.by_type() == after.by_type()
    assert before.unconnected_count == after.unconnected_count == 0


# --------------------------------------------------------------- criterion 2: determinism

def test_emission_is_byte_identical_for_the_same_seed(prob):
    """P0 exit criterion 2: reruns are diffable, not merely statistically similar."""
    solution = Solution()
    solution.add(Segment(net=1, layer="F.Cu", x1=0, y1=0, x2=1_000_000, y2=0, width_nm=250_000))

    assert emit(prob, solution, "S1", 7) == emit(prob, solution, "S1", 7)
    assert emit(prob, solution, "S1", 7) != emit(prob, solution, "S1", 8)
    assert emit(prob, solution, "S1", 7) != emit(prob, solution, "S2", 7)


def test_deterministic_uuid_is_stable_and_well_formed():
    first = deterministic_uuid("S7", 3, 12)
    assert first == deterministic_uuid("S7", 3, 12)
    assert first != deterministic_uuid("S7", 3, 13)
    assert [len(part) for part in first.split("-")] == [8, 4, 4, 4, 12]


# --------------------------------------------------------- criterion 3: it catches failure

def test_oracle_catches_a_deleted_track(prob, tmp_path, bare_result):
    """Removing one segment must make the board report an unconnected item."""
    tree = sexpr.parse(sexpr.dumps(prob.tree))
    segments = sexpr.find_all(tree, "segment")
    assert segments, "demo board should be routed"
    tree.remove(segments[len(segments) // 2])

    result = oracle.run_drc(_place(tree, tmp_path, "cut", prob.project_path))
    assert result.unconnected_count > 0


def test_oracle_catches_overlapping_copper(prob, tmp_path, bare_result):
    """Two tracks on different nets laid on top of each other must be flagged."""
    x0, y0, x1, y1 = prob.bbox_nm
    cx, cy = (x0 + x1) // 2, (y0 + y1) // 2

    solution = Solution()
    solution.add(Segment(net=1, layer="F.Cu", x1=cx, y1=cy,
                         x2=cx + 20_000_000, y2=cy, width_nm=500_000))
    solution.add(Segment(net=2, layer="F.Cu", x1=cx, y1=cy + 20_000,
                         x2=cx + 20_000_000, y2=cy + 20_000, width_nm=500_000))

    board = tmp_path / "short.kicad_pcb"
    board.write_text(emit(prob, solution, "probe", 0, replace_nets=set()),
                     encoding="utf-8", newline="\n")
    shutil.copy(prob.project_path, board.with_suffix(".kicad_pro"))

    result = oracle.run_drc(board)
    fresh = score.new_violations(result, bare_result)
    types = {v.type for v in fresh}
    assert types & {"clearance", "shorting_items", "tracks_crossing"}, \
        f"overlapping copper was not flagged; got {types}"


# --------------------------------------------------------------------------- scoring

def test_bare_board_has_no_routing_rule_violations(bare_result):
    """The demo board's own violations are all library noise, which we filter out.

    If this ever fails it means a board arrives with pre-existing violations we would
    otherwise be charged for -- which is exactly what baseline subtraction exists to absorb.
    """
    from arena.rules import ROUTING_RULES
    charged = [v for v in bare_result.violations if v.type in ROUTING_RULES]
    assert not charged, f"unexpected pre-existing routing violations: {charged[:3]}"
    assert bare_result.unconnected_count > 0, "a stripped board must report unconnected items"


def test_baseline_subtraction_forgives_preexisting_violations(bare_result):
    """A violation already present on the bare board must not be charged to the router."""
    assert score.new_violations(bare_result, bare_result) == []


def test_scoring_a_broken_board_fails_it(prob, bare_result, tmp_path):
    solution = Solution()
    x0, y0, x1, y1 = prob.bbox_nm
    solution.add(Segment(net=1, layer="F.Cu", x1=(x0 + x1) // 2, y1=(y0 + y1) // 2,
                         x2=(x0 + x1) // 2 + 20_000_000, y2=(y0 + y1) // 2, width_nm=500_000))

    board = tmp_path / "broken.kicad_pcb"
    board.write_text(emit(prob, solution, "probe", 0, replace_nets=set()),
                     encoding="utf-8", newline="\n")
    shutil.copy(prob.project_path, board.with_suffix(".kicad_pro"))

    result = oracle.run_drc(board)
    scored = score.score_solution(prob, solution, result, bare_result, time_s=0.1)

    assert scored.cp == 0
    assert scored.unconnected > 0
    assert scored.wl_mm == pytest.approx(20.0, abs=0.01)
    assert scored.vias == 0
    assert scored.wl_ratio > 0


def test_kicad_version_is_recorded(bare_result):
    """A score without an engine version attached is not a score."""
    assert bare_result.kicad_version and bare_result.kicad_version != "unknown"


def test_wirelength_counts_via_free_copper_only(prob):
    solution = Solution()
    solution.add(Segment(net=1, layer="F.Cu", x1=0, y1=0, x2=3_000_000, y2=4_000_000,
                         width_nm=250_000))
    solution.add(Via(net=1, x=0, y=0, diameter_nm=800_000, drill_nm=400_000,
                     layer_from="F.Cu", layer_to="B.Cu"))
    assert solution.wirelength_nm == pytest.approx(5_000_000)
    assert len(solution.vias) == 1
