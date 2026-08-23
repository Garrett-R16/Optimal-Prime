"""Level 2 -- round-trip and invariant properties of the s-expression layer."""

from __future__ import annotations

import glob
import io
import os

import pytest

from arena import sexpr

DEMO_GLOB = r"C:\Program Files\KiCad\9.0\share\kicad\demos/**/*.kicad_pcb"

# KiCad ships one demo board with 349 missing open-parens (a ")filter_ratio 0.9)" pattern
# throughout). KiCad's own lexer tolerates it and silently mis-nests the result; we reject it
# instead. Kept here so the corpus test stays honest about what it skips.
KNOWN_MALFORMED = {"RoyalBlue54L-Feather.kicad_pcb"}


def demo_boards() -> list[str]:
    return sorted(glob.glob(DEMO_GLOB, recursive=True))


# --------------------------------------------------------------------------- basics

def test_parse_simple():
    node = sexpr.parse('(segment (start 1.0 2.0) (net 3) (layer "F.Cu"))')
    assert sexpr.head(node) == "segment"
    assert sexpr.floats(sexpr.find(node, "start")) == [1.0, 2.0]
    assert sexpr.find(node, "layer")[1].text == "F.Cu"


def test_atom_preserves_raw_number_text():
    node = sexpr.parse("(at 1.500000 -0.0)")
    assert [a.raw for a in sexpr.atoms(node)] == ["1.500000", "-0.0"]
    assert sexpr.dumps(node).strip() == "(at 1.500000 -0.0)"


def test_quoted_string_escapes_round_trip():
    original = r'(property "Desc" "a \"quoted\" and \\ backslash")'
    node = sexpr.parse(original)
    assert node[2].text == 'a "quoted" and \\ backslash'
    assert sexpr.parse(sexpr.dumps(node)) == node


def test_paren_inside_string_is_not_structure():
    node = sexpr.parse('(property "Val" "Capacitor (SMD)")')
    assert len(node) == 3
    assert node[2].text == "Capacitor (SMD)"


@pytest.mark.parametrize("bad", [
    "(unbalanced",
    "(a)) ",
    "orphan-atom",
    "",
    '(unterminated "string',
    "(a) (b)",
])
def test_malformed_input_raises(bad):
    with pytest.raises(sexpr.SExprError):
        sexpr.parse(bad)


def test_num_formatting_is_stable():
    assert sexpr.num(1.0).raw == "1"
    assert sexpr.num(1.5).raw == "1.5"
    assert sexpr.num(-0.0).raw == "0"
    assert sexpr.num(0.1 + 0.2).raw == "0.3"
    # Emitting the same value twice must produce identical text -- this is what makes
    # deterministic reruns byte-identical.
    assert sexpr.num(1.234567891).raw == sexpr.num(1.234567891).raw


def test_remove_children():
    node = sexpr.parse("(board (segment 1) (via 2) (segment 3) (zone 4))")
    removed = sexpr.remove_children(node, ["segment", "via"])
    assert removed == 3
    assert [sexpr.head(c) for c in node if isinstance(c, list)] == ["zone"]


# --------------------------------------------------------------------------- corpus

@pytest.mark.skipif(not demo_boards(), reason="KiCad demo boards not installed")
def test_round_trip_is_a_fixpoint_on_every_demo_board():
    """P0 exit criterion 1: parse -> emit -> parse is the identity."""
    checked = 0
    for path in demo_boards():
        if os.path.basename(path) in KNOWN_MALFORMED:
            continue
        src = io.open(path, encoding="utf-8").read()
        first = sexpr.parse(src)
        second = sexpr.parse(sexpr.dumps(first))
        third = sexpr.parse(sexpr.dumps(second))
        assert first == second, f"not a fixpoint: {path}"
        assert second == third, f"not idempotent: {path}"
        checked += 1
    assert checked >= 10, f"expected a meaningful corpus, checked only {checked}"
