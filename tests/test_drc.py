"""Level 1 -- the differential test between our clearance checker and KiCad's DRC.

This is the highest-value test in the plan, and it is deliberately one-directional.

    if WE say clean, KiCad must say clean

The reverse is allowed and expected: we are stricter, which costs completions rather than
correctness. A checker that is ever *more permissive* than the engine lets a router emit
boards carrying violations it never saw, and the whole run is wasted.

Nothing here checks whether KiCad is right. KiCad is right by definition; these tests check
that our fast approximation of one rule stays on the safe side of it.
"""

from __future__ import annotations

import json
import math
import random
import shutil
from pathlib import Path

import pytest

from arena import drc, oracle
from arena.emit import write_solution
from arena.problem import load_problem
from arena.solution import Segment, Solution, Via

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / "boards" / "manifest.json"

#: KiCad violation types our checker is answerable for. It says nothing about connectivity,
#: annular rings, mask slivers, or the two dozen other rules the engine enforces.
CLEARANCE_FAMILY = {"clearance", "shorting_items", "tracks_crossing", "copper_edge_clearance"}

pytestmark = pytest.mark.skipif(
    not MANIFEST.exists(),
    reason="no board manifest; run scripts/fetch_boards.py --source kicad-demos",
)


@pytest.fixture(scope="module")
def problem():
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    entry = next(b for b in manifest["boards"] if b["name"] == "ecc83-pp")
    return load_problem(ROOT / entry["path"], ROOT / entry["project"])


def random_layout(problem, rng, count, marginal):
    """Random tracks; ``marginal`` adds parallel neighbours near the clearance limit."""
    x0, y0, x1, y1 = problem.bbox_nm
    nets = [n.code for n in problem.routable_nets][:6]
    clearance = problem.rules.max_clearance_nm
    width = problem.rules.max_track_width_nm

    solution = Solution()
    for _ in range(count):
        net = rng.choice(nets)
        layer = rng.choice(problem.copper_layers)
        ax = rng.randint(x0 + 2_000_000, x1 - 2_000_000)
        ay = rng.randint(y0 + 2_000_000, y1 - 2_000_000)
        length = rng.randint(2_000_000, 15_000_000)
        theta = rng.uniform(0, 2 * math.pi)
        bx = int(ax + length * math.cos(theta))
        by = int(ay + length * math.sin(theta))
        solution.add(Segment(net=net, layer=layer, x1=ax, y1=ay, x2=bx, y2=by,
                             width_nm=width))
        if marginal and rng.random() < 0.7:
            offset = width + clearance + rng.randint(-clearance // 2, clearance)
            nx = -math.sin(theta) * offset
            ny = math.cos(theta) * offset
            other = rng.choice([c for c in nets if c != net])
            solution.add(Segment(net=other, layer=layer,
                                 x1=int(ax + nx), y1=int(ay + ny),
                                 x2=int(bx + nx), y2=int(by + ny), width_nm=width))
    return solution


def empty_corridor(problem, length_nm=8_000_000, margin_nm=2_000_000):
    """Find a horizontal run of empty board: no pad within ``margin_nm``, inside the outline.

    Real boards are mostly covered by pads and their halos, so "put a track at x0+5mm" lands
    on copper and every clean-layout assertion fails for the wrong reason.
    """
    x0, y0, x1, y1 = problem.bbox_nm
    step = 250_000
    for y in range(y0 + margin_nm, y1 - margin_nm, step):
        for x in range(x0 + margin_nm, x1 - margin_nm - length_nm, step * 4):
            if all(not (x - margin_nm <= pad.x <= x + length_nm + margin_nm
                        and abs(pad.y - y) <= margin_nm + pad.radius_nm)
                   for pad in problem.pads):
                return x, y, x + length_nm
    raise AssertionError("no empty corridor on this board")


# --------------------------------------------------------------------------- unit

def test_far_apart_tracks_are_clear(problem):
    ax, ay, bx = empty_corridor(problem)
    solution = Solution()
    solution.add(Segment(net=1, layer="F.Cu", x1=ax, y1=ay, x2=bx, y2=ay, width_nm=250_000))
    solution.add(Segment(net=2, layer="F.Cu", x1=ax, y1=ay + 4_000_000,
                         x2=bx, y2=ay + 4_000_000, width_nm=250_000))
    between = [c for c in drc.check_solution(problem, solution)
               if {c.net_a, c.net_b} == {1, 2}]
    assert not between


def test_overlapping_tracks_on_different_nets_conflict(problem):
    ax, ay, bx = empty_corridor(problem)
    solution = Solution()
    for net, dy in ((1, 0), (2, 20_000)):
        solution.add(Segment(net=net, layer="F.Cu", x1=ax, y1=ay + dy, x2=bx, y2=ay + dy,
                             width_nm=500_000))
    conflicts = [c for c in drc.check_solution(problem, solution)
                 if {c.net_a, c.net_b} == {1, 2}]
    assert conflicts
    assert conflicts[0].actual_nm < conflicts[0].required_nm


def test_same_net_copper_may_touch(problem):
    ax, ay, bx = empty_corridor(problem)
    solution = Solution()
    for dy in (0, 20_000):
        solution.add(Segment(net=1, layer="F.Cu", x1=ax, y1=ay + dy, x2=bx, y2=ay + dy,
                             width_nm=500_000))
    assert not [c for c in drc.check_solution(problem, solution)
                if c.net_a == 1 and c.net_b == 1]


def test_different_layers_do_not_interact(problem):
    ax, ay, bx = empty_corridor(problem)
    solution = Solution()
    for net, layer in ((1, "F.Cu"), (2, "B.Cu")):
        solution.add(Segment(net=net, layer=layer, x1=ax, y1=ay, x2=bx, y2=ay,
                             width_nm=500_000))
    between = [c for c in drc.check_solution(problem, solution)
               if {c.net_a, c.net_b} == {1, 2}]
    assert not between, "copper on different layers must not conflict"


def test_a_via_interacts_on_every_layer(problem):
    """A through-hole via occupies the whole stack, so a track on any layer must clear it."""
    ax, ay, _ = empty_corridor(problem)
    solution = Solution()
    cx, cy = ax + 2_000_000, ay
    solution.add(Via(net=1, x=cx, y=cy, diameter_nm=1_600_000, drill_nm=600_000,
                     layer_from="F.Cu", layer_to="B.Cu"))
    solution.add(Segment(net=2, layer="B.Cu", x1=cx - 5_000_000, y1=cy + 700_000,
                         x2=cx + 5_000_000, y2=cy + 700_000, width_nm=250_000))
    conflicts = [c for c in drc.check_solution(problem, solution)
                 if {c.net_a, c.net_b} == {1, 2}]
    assert conflicts, "a via must be seen from every copper layer"


def test_copper_outside_the_board_is_flagged(problem):
    """Board-edge clearance must use the real outline, not the bounding box."""
    x0, y0, x1, y1 = problem.bbox_nm
    solution = Solution()
    solution.add(Segment(net=1, layer="F.Cu", x1=x0 - 5_000_000, y1=(y0 + y1) // 2,
                         x2=x0 + 5_000_000, y2=(y0 + y1) // 2, width_nm=250_000))
    assert any(c.label_b == "board edge" for c in drc.check_solution(problem, solution))


# --------------------------------------------------------------------------- differential

@pytest.mark.oracle
def test_our_checker_is_never_more_permissive_than_kicad(problem, tmp_path):
    """The contract: if we say clean, KiCad must say clean.

    Layouts alternate between free-form and *marginal* -- parallel neighbours placed within
    half a clearance of the limit, which is exactly where a sloppy predicate goes wrong.
    """
    rng = random.Random(4242)
    unsafe: list[str] = []
    stricter = both_clean = both_dirty = 0

    for index in range(24):
        solution = random_layout(problem, rng, rng.randint(2, 10), marginal=index % 2 == 0)
        board = tmp_path / f"trial{index}.kicad_pcb"
        write_solution(problem, solution, board, "diff", 0, replace_nets=set())

        engine = [v for v in oracle.run_drc(board).violations if v.type in CLEARANCE_FAMILY]
        ours = drc.check_solution(problem, solution)

        if not ours and engine:
            unsafe.append(f"trial {index}: we said clean, KiCad found "
                          f"{[v.type for v in engine[:3]]}")
        elif ours and not engine:
            stricter += 1
        elif not ours and not engine:
            both_clean += 1
        else:
            both_dirty += 1

        for suffix in (".kicad_pcb", ".kicad_pro", ".drc.json"):
            board.with_suffix(suffix).unlink(missing_ok=True)

    assert not unsafe, "\n".join(unsafe)
    assert both_dirty + stricter > 0, "the generator produced nothing worth checking"


@pytest.mark.oracle
def test_a_deliberately_clean_layout_agrees_with_kicad(problem, tmp_path):
    """Well-separated copper must be clean on both sides -- conservatism has limits too.

    Without this, a checker that simply reported everything as a conflict would pass the
    one-directional contract while being useless.
    """
    # The corridor is verified only in a band around `ay`, so every track must stay inside
    # it. Spacing them further apart walks the outer ones back onto real copper.
    ax, ay, bx = empty_corridor(problem, length_nm=6_000_000, margin_nm=4_000_000)
    solution = Solution()
    for index, offset in enumerate((-1_500_000, 0, 1_500_000)):
        solution.add(Segment(net=1 + index, layer="F.Cu",
                             x1=ax, y1=ay + offset, x2=bx, y2=ay + offset,
                             width_nm=250_000))

    board = tmp_path / "clean.kicad_pcb"
    write_solution(problem, solution, board, "clean", 0, replace_nets=set())
    engine = [v for v in oracle.run_drc(board).violations if v.type in CLEARANCE_FAMILY]
    ours = [c for c in drc.check_solution(problem, solution) if c.net_b > 0]

    assert not engine, f"KiCad flagged a deliberately clean layout: {engine[:2]}"
    assert not ours, f"our checker flagged a clean layout: {ours[:2]}"
