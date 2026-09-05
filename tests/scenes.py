"""Tiny boards with known right answers, written as real ``.kicad_pcb`` text.

Every first-principles test routes one of these through the same door a user's board comes
through -- ``load_board`` on KiCad file text -- because the promise under test is not "the
algorithm is clever" but "the board that comes out of KiCad's checker is clean". A scene is a
few pads on a small two-layer board, built by string assembly here so a failing test can print
the whole board it ran on.

Coordinates are millimetres, as KiCad writes them. Pads are through-hole circles unless a test
needs otherwise, so every pad exists on both layers and any connection may use either side.
"""

from __future__ import annotations

import textwrap
from pathlib import Path

MM = 1_000_000  # nanometres

_HEADER = """\
(kicad_pcb (version 20240108) (generator "scenes")
  (general (thickness 1.6) (legacy_teardrops no))
  (paper "A4")
  (layers
    (0 "F.Cu" signal)
    (2 "B.Cu" signal)
    (9 "F.Mask" user "F.Mask")
    (11 "B.Mask" user "B.Mask")
    (25 "Edge.Cuts" user "Edge.Cuts")
  )
  (setup
    (pad_to_mask_clearance 0)
    (allow_soldermask_bridges_in_footprints no)
  )
"""

_PROJECT = """\
{
  "board": {
    "design_settings": {
      "rules": {
        "min_clearance": 0.2,
        "min_copper_edge_clearance": 0.3,
        "min_track_width": 0.2
      }
    }
  },
  "net_settings": {
    "classes": [
      {
        "name": "Default",
        "clearance": 0.2,
        "track_width": 0.25,
        "via_diameter": 0.8,
        "via_drill": 0.4
      }
    ]
  }
}
"""


def _outline(width: float, height: float) -> str:
    corners = [(0.0, 0.0), (width, 0.0), (width, height), (0.0, height)]
    out = []
    for (x1, y1), (x2, y2) in zip(corners, corners[1:] + corners[:1]):
        out.append(f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) '
                   f'(stroke (width 0.1) (type default)) (layer "Edge.Cuts"))')
    return "\n".join(out)


def _net_table(names: list[str]) -> str:
    lines = ['  (net 0 "")']
    for index, name in enumerate(names, start=1):
        lines.append(f'  (net {index} "{name}")')
    return "\n".join(lines)


def _pad(reference: str, x: float, y: float, net: int, net_name: str,
         diameter: float = 1.6, shape: str = "circle",
         size: tuple[float, float] | None = None, smd: bool = False) -> str:
    sx, sy = size if size else (diameter, diameter)
    if smd:
        copper = (f'(pad "1" smd {shape} (at 0 0) (size {sx} {sy}) '
                  f'(layers "F.Cu" "F.Mask") (net {net} "{net_name}"))')
    else:
        copper = (f'(pad "1" thru_hole {shape} (at 0 0) (size {sx} {sy}) (drill 0.6) '
                  f'(layers "*.Cu" "*.Mask") (net {net} "{net_name}"))')
    return textwrap.dedent(f"""\
      (footprint "scene:{reference}" (layer "F.Cu")
        (at {x} {y})
        (property "Reference" "{reference}" (at 0 0 0) (layer "F.SilkS") hide
          (effects (font (size 1 1) (thickness 0.15))))
        (property "Value" "" (at 0 0 0) (layer "F.Fab") hide
          (effects (font (size 1 1) (thickness 0.15))))
        {copper}
      )""")


def scene(width: float, height: float, nets: list[str],
          pads: list[tuple], extra: str = "") -> str:
    """A board. ``pads`` rows are (reference, x, y, net_index_from_1[, smd])."""
    body = [_HEADER, _net_table(nets), _outline(width, height)]
    for row in pads:
        reference, x, y, net = row[:4]
        smd = bool(row[4]) if len(row) > 4 else False
        body.append(textwrap.indent(
            _pad(reference, x, y, net, nets[net - 1] if net else "", smd=smd), "  "))
    if extra:
        body.append(extra)
    body.append(")")
    return "\n".join(body)


def write_scene(tmp_path: Path, name: str, text: str) -> Path:
    board = tmp_path / f"{name}.kicad_pcb"
    board.write_text(text, encoding="utf-8")
    board.with_suffix(".kicad_pro").write_text(_PROJECT, encoding="utf-8")
    return board


# ------------------------------------------------------------------ the canonical scenes

def open_pair(tmp_path: Path) -> Path:
    """Two pads, nothing between them. The answer is the straight line, exactly."""
    return write_scene(tmp_path, "open-pair", scene(
        30.0, 20.0, ["A"],
        [("P1", 5.0, 10.0, 1), ("P2", 25.0, 10.0, 1)]))


def one_disc(tmp_path: Path) -> Path:
    """Two pads with one round obstacle dead between them: line, arc, line."""
    return write_scene(tmp_path, "one-disc", scene(
        30.0, 20.0, ["A", "BLOCK"],
        [("P1", 5.0, 10.0, 1), ("P2", 25.0, 10.0, 1), ("BLK", 15.0, 10.0, 2)]))


def fan_cut(tmp_path: Path) -> Path:
    """The stub-crossing scene, minimal.

    Net A runs left to right along the middle. Net B's two pads sit above and below the
    middle of A's run, so B's natural straight path cuts A -- and near B's terminals the
    conflict lives in the fan of triangles around B's pads, which no shared doorway records.
    A legal answer must either route around an endpoint or spend a via; crossing is not one.
    """
    return write_scene(tmp_path, "fan-cut", scene(
        30.0, 20.0, ["A", "B"],
        [("A1", 4.0, 10.0, 1), ("A2", 26.0, 10.0, 1),
         ("B1", 15.0, 6.0, 2), ("B2", 15.0, 14.0, 2)]))


def stub_graze(tmp_path: Path) -> Path:
    """Adjacent pads on one component; the wire from one must not clip its neighbour.

    Three pads in a tight row. The connection leaves the middle pad and runs parallel to the
    row, so its stub -- pad centre to first doorway -- passes right beside the flanking pads.
    """
    return write_scene(tmp_path, "stub-graze", scene(
        30.0, 20.0, ["MID", "L", "R"],
        [("PL", 13.0, 10.0, 2), ("PM", 15.0, 10.0, 1), ("PR", 17.0, 10.0, 3),
         ("FAR", 15.0, 3.0, 1)]))


def crossing_ladder(tmp_path: Path) -> Path:
    """Three connections that pairwise cross. Two layers cannot hold them without a via."""
    pads = []
    nets = ["X", "Y", "Z"]
    # Ends on a circle so every pair of chords crosses near the middle.
    spots = [("X1", 5.0, 5.0, 1), ("X2", 25.0, 15.0, 1),
             ("Y1", 5.0, 15.0, 2), ("Y2", 25.0, 5.0, 2),
             ("Z1", 15.0, 3.0, 3), ("Z2", 15.0, 17.0, 3)]
    pads.extend(spots)
    return write_scene(tmp_path, "crossing-ladder", scene(30.0, 20.0, nets, pads))


def fan_cut_one_layer(tmp_path: Path) -> Path:
    """The persistent sonde crossing, minimal: every pad on the front only.

    Net A runs along the middle; net B crosses it, and neither may change layer at its
    pads because they exist on F.Cu alone. The only legal answers are a via or routing
    around one of the other net's terminals -- crossing is not one, and the layer
    preference that dissolves the through-hole version of this scene has nothing to offer.
    """
    return write_scene(tmp_path, "fan-cut-one-layer", scene(
        30.0, 20.0, ["A", "B"],
        [("A1", 4.0, 10.0, 1, True), ("A2", 26.0, 10.0, 1, True),
         ("B1", 15.0, 6.0, 2, True), ("B2", 15.0, 14.0, 2, True)]))


def highway(tmp_path: Path) -> Path:
    """A long straight leg with company: neighbours that share its doorways but never cross.

    Net H runs the length of the board, dead straight. Nets P and Q run parallel above and
    below, close enough that all three thread the same doorways between the flanking pads.
    The right answer keeps H exactly straight and P and Q taut beside it.
    """
    return write_scene(tmp_path, "highway", scene(
        40.0, 20.0, ["H", "P", "Q", "W1", "W2", "W3", "W4"],
        [("H1", 4.0, 10.0, 1), ("H2", 36.0, 10.0, 1),
         ("P1", 4.0, 8.0, 2), ("P2", 36.0, 8.0, 2),
         ("Q1", 4.0, 12.0, 3), ("Q2", 36.0, 12.0, 3),
         ("B1", 14.0, 4.5, 4), ("B2", 14.0, 15.5, 5),
         ("B3", 26.0, 4.5, 6), ("B4", 26.0, 15.5, 7)]))


def crossing_highway(tmp_path: Path) -> Path:
    """A straight leg crossed by a wire, all pads on the front only.

    One of the two must yield, and the cheaper yield is H bending around X's pad -- the
    scene that proved "a straight wire stays straight" is doctrine, not optimality.
    """
    return write_scene(tmp_path, "crossing-highway", scene(
        40.0, 20.0, ["H", "X"],
        [("H1", 4.0, 10.0, 1, True), ("H2", 36.0, 10.0, 1, True),
         ("X1", 20.0, 5.0, 2, True), ("X2", 20.0, 15.0, 2, True)]))


def two_through_a_gap(tmp_path: Path) -> Path:
    """Two nets share one doorway between two blocks; both must fit, correctly spaced."""
    return write_scene(tmp_path, "two-through-a-gap", scene(
        30.0, 20.0, ["A", "B", "W1", "W2"],
        [("A1", 4.0, 9.0, 1), ("A2", 26.0, 9.0, 1),
         ("B1", 4.0, 11.0, 2), ("B2", 26.0, 11.0, 2),
         ("W1", 15.0, 5.5, 3), ("W2", 15.0, 14.5, 4)]))
