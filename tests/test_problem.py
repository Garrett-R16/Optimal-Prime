"""Level 2/3 -- board ingestion, pad placement, and the rule set.

The load-bearing test here is :func:`test_pad_placement_convention_is_pinned`. A wrong
rotation sign does not crash; it silently displaces every pad on a rotated footprint, and the
only symptom is boards that mysteriously will not route. So the convention is pinned against
real boards rather than asserted from documentation.
"""

from __future__ import annotations

import glob
import math
import os
from pathlib import Path

import pytest

from arena import problem, sexpr
from arena.rules import CANONICAL_SEVERITIES, ROUTING_RULES, load_design_rules
from arena.units import mm_to_nm

DEMOS = Path(r"C:\Program Files\KiCad\9.0\share\kicad\demos")
PIC = DEMOS / "pic_programmer" / "pic_programmer.kicad_pcb"

pytestmark = pytest.mark.skipif(not DEMOS.exists(), reason="KiCad demos not installed")


def demo_boards() -> list[Path]:
    return [Path(p) for p in sorted(glob.glob(str(DEMOS / "**" / "*.kicad_pcb"), recursive=True))]


def track_endpoints(tree) -> dict[int, list[tuple[int, int]]]:
    """Every existing copper endpoint on the board, grouped by net code."""
    out: dict[int, list[tuple[int, int]]] = {}

    def add(net_node, coords):
        code = net_node[1].as_int() if net_node is not None and len(net_node) > 1 else 0
        out.setdefault(code, []).append(coords)

    for item in sexpr.find_all(tree, "segment") + sexpr.find_all(tree, "arc"):
        for key in ("start", "end"):
            vals = problem._numbers(sexpr.find(item, key))
            if len(vals) >= 2:
                add(sexpr.find(item, "net"), (mm_to_nm(vals[0]), mm_to_nm(vals[1])))
    for via in sexpr.find_all(tree, "via"):
        vals = problem._numbers(sexpr.find(via, "at"))
        if len(vals) >= 2:
            add(sexpr.find(via, "net"), (mm_to_nm(vals[0]), mm_to_nm(vals[1])))
    return out


def pad_coverage(prob: problem.Problem) -> tuple[int, int]:
    """(pads reached by an existing copper endpoint, pads that ought to be reachable)."""
    endpoints = track_endpoints(prob.tree)
    candidates = [p for p in prob.pads if p.net in endpoints and p.net != 0]
    hit = sum(1 for p in candidates if any(p.contains(x, y) for x, y in endpoints[p.net]))
    return hit, len(candidates)


# --------------------------------------------------------------------------- placement

def test_pad_placement_convention_is_pinned():
    """On a fully track-routed board every connected pad must be reached by copper.

    This is the end-to-end proof that the footprint rotation, the pad offset and the unit
    conversion all agree with KiCad's.
    """
    prob = problem.load_problem(PIC)
    hit, total = pad_coverage(prob)
    assert total > 50, "expected a board with a meaningful number of connected pads"
    assert hit / total >= 0.99, f"pad coverage only {hit}/{total}"


def test_rotation_sign_beats_the_alternative_across_the_corpus():
    """The opposite rotation sign must score strictly worse on boards with rotated parts."""
    def rotate_wrong(x, y, degrees):
        if not degrees:
            return x, y
        a = math.radians(degrees)
        return x * math.cos(a) - y * math.sin(a), y * math.cos(a) + x * math.sin(a)

    right = wrong = 0
    boards = 0
    for path in demo_boards():
        try:
            prob = problem.load_problem(path)
        except problem.BoardLoadError:
            continue
        rotated = any(
            problem._at(sexpr.find(fp, "at"))[2] % 180 != 0
            for fp in sexpr.find_all(prob.tree, "footprint")
        )
        if not rotated:
            continue
        hit, total = pad_coverage(prob)
        if total == 0:
            continue
        boards += 1
        right += hit

        original = problem._rotate
        problem._rotate = rotate_wrong
        try:
            wrong += pad_coverage(problem.load_problem(path))[0]
        finally:
            problem._rotate = original

    assert boards >= 5, f"expected several boards with rotated footprints, got {boards}"
    assert right > wrong * 1.2, f"rotation sign not decisive: right={right} wrong={wrong}"


def test_oval_drill_parses():
    """(drill oval 0.6 1.2) must not throw -- it appears on real boards."""
    node = sexpr.parse("(pad \"1\" thru_hole oval (at 0 0) (size 2 3) (drill oval 0.6 1.2))")
    assert problem._numbers(sexpr.find(node, "drill")) == [0.6, 1.2]


def test_pad_contains_respects_rotation():
    pad = problem.Pad(net=1, number="1", x=0, y=0, size_x=2_000_000, size_y=400_000,
                      shape="rect", angle=90.0, layers=("F.Cu",), drill_nm=0, footprint="R1")
    assert pad.contains(0, 900_000)      # along the rotated long axis
    assert not pad.contains(900_000, 0)  # along the rotated short axis


# --------------------------------------------------------------------------- corpus health

def test_every_demo_board_loads_or_fails_loudly():
    loaded, failures = 0, []
    for path in demo_boards():
        try:
            prob = problem.load_problem(path)
        except problem.BoardLoadError as exc:
            failures.append((path.name, str(exc)))
            continue
        assert prob.layer_count >= 1
        assert prob.bbox_nm[0] < prob.bbox_nm[2]
        loaded += 1
    assert loaded >= 15, f"only {loaded} boards loaded; failures={failures}"
    # The only tolerated failures are the corrupt demo and boards with no project file.
    for name, message in failures:
        assert "multiple root expressions" in message or "no .kicad_pro" in message, \
            f"unexpected load failure on {name}: {message}"


# --------------------------------------------------------------------------- rules

def test_canonical_severity_map_covers_what_kicad_ships():
    """Every rule key in a real project file must be classified, not silently defaulted."""
    rules = load_design_rules(PIC.with_suffix(".kicad_pro"))
    assert not rules.unclassified_severities, \
        f"unclassified DRC rules: {rules.unclassified_severities}"


def test_severity_map_values_are_legal_and_routing_set_is_sane():
    assert set(CANONICAL_SEVERITIES.values()) <= {"error", "ignore"}
    assert "clearance" in ROUTING_RULES
    assert "unconnected_items" in ROUTING_RULES
    assert "lib_footprint_issues" not in ROUTING_RULES
    assert "courtyards_overlap" not in ROUTING_RULES  # placement is frozen, not ours


def test_design_rules_are_integer_nanometres():
    rules = load_design_rules(PIC.with_suffix(".kicad_pro"))
    default = rules.default
    assert isinstance(default.clearance_nm, int)
    assert default.clearance_nm > 0
    assert default.track_width_nm > 0
    assert default.via_diameter_nm > default.via_drill_nm
    assert not rules.allow_blind_buried_vias, "MVP-01 freezes via type to through-hole"
