"""Doorway capacity must be a count, not a function of where earlier wires sat.

A gate that holds exactly three wires, three straight lines crossing it mid-gap. The
weave commits each wire where its line crosses the doorway; two wires near the middle
leave two slivers that hold nothing, and the third is rejected although the total free
width holds it. Measured on the 630-pad board this is the mechanism behind 97 of 264
connections being outlawed. The fix is the rubber-band discipline: only ORDER at a
doorway, capacity by count, positions re-seated by rank after every commit.
"""
import math
from pathlib import Path

from scenes import scene, write_scene
from taut.board import load_board
from taut.mesh import build_mesh
from taut.plan import _cores, _inside, board_polygon
from taut.units import CLEARANCE_MARGIN, GUARDBAND_NM
from taut.weave import Weave


def _woven(tmp_path: Path, gap_mm: float, order: tuple[str, ...]) -> int:
    pads = []
    for i, name in enumerate(("A", "B", "C")):
        y = 10.0 + 0.5 * (i - 1)
        pads.append((f"{name}1", 3.0, y, i + 1, True))
        pads.append((f"{name}2", 27.0, y, i + 1, True))
    half = 0.85
    walls = []
    y, k = 10.0 - gap_mm / 2 - half, 0
    while y > -half:
        walls.append((f"XL{k}", 15.0, y, 4, True))
        y -= 2 * half
        k += 1
    y = 10.0 + gap_mm / 2 + half
    while y < 20.0 + half:
        walls.append((f"XU{k}", 15.0, y, 4, True))
        y += 2 * half
        k += 1
    path = write_scene(tmp_path, f"frag-gate-{gap_mm:.2f}-{''.join(order)}",
                       scene(30.0, 20.0, ["A", "B", "C", "X"], pads + walls))
    board = load_board(str(path))
    netclass = board.netclass_for("A")
    clearance = netclass.clearance_nm * (1 + CLEARANCE_MARGIN)
    width = float(netclass.track_width_nm)
    need = width / 2 + clearance + GUARDBAND_NM
    polygon = board_polygon(board)
    mesh = build_mesh(_cores(board, "F.Cu"), polygon, clearance, width)
    for i in range(len(mesh.triangles)):
        if mesh.free[i] and not _inside(polygon, *mesh.centroid(i)):
            mesh.free[i] = False
    weave = Weave(mesh)
    nets = {net.name: net for net in board.nets.values()}
    got = 0
    for k, name in enumerate(order):
        a, b = list(nets[name].pads)
        head, tail = (float(a.x), float(a.y)), (float(b.x), float(b.y))
        got += weave.insert(k, head, tail, mesh.terminals(*head),
                            mesh.terminals(*tail), need=need,
                            clearance=clearance).found
    return got


def test_capacity_does_not_depend_on_insertion_order(tmp_path):
    gap = 2.0  # three wires fit with room to spare
    assert _woven(tmp_path, gap, ("B", "A", "C")) == 3
    assert _woven(tmp_path, gap, ("A", "C", "B")) == 3


def test_three_wires_really_do_not_fit_at_two_pitches(tmp_path):
    assert _woven(tmp_path, 1.2, ("A", "C", "B")) < 3
