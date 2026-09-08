"""A walled wire dives: the weave places its own escape vias.

A one-wire gate in a full-height wall on the front layer: A takes it, and B -- all
front-side pads -- has no way through on the front. escape_route finds pad -> via
beside the pad -> back layer -> via -> pad, and every leg of that answer then weaves
for real.
"""
from pathlib import Path

from scenes import scene, write_scene
from taut.board import load_board
from taut.escape import escape_route
from taut.mesh import build_mesh
from taut.plan import _cores, _inside, _pocket_sites, _Link, board_polygon
from taut.units import CLEARANCE_MARGIN, GUARDBAND_NM
from taut.vias import via_sites
from taut.weave import Weave


def test_walled_wire_escapes_through_the_back(tmp_path: Path):
    # Two full-height walls of X pads on the front leave a gate wide enough for exactly
    # one wire. A takes it; B, whose pads are all front-side, cannot follow on the front.
    half, gap = 0.85, 1.2
    pads = [("A1", 3.0, 10.0, 1, True), ("A2", 27.0, 10.0, 1, True),
            ("B1", 3.0, 12.0, 2, True), ("B2", 27.0, 12.0, 2, True)]
    y, k = 10.0 - gap / 2 - half, 0
    while y > -half:
        pads.append((f"XL{k}", 15.0, y, 3, True))
        y -= 2 * half
        k += 1
    y = 10.0 + gap / 2 + half
    while y < 20.0 + half:
        pads.append((f"XU{k}", 15.0, y, 3, True))
        y += 2 * half
        k += 1
    path = write_scene(tmp_path, "one-wire-gate", scene(30.0, 20.0, ["A", "B", "X"], pads))
    board = load_board(str(path))
    usable = ("F.Cu", "B.Cu")
    nets = {n.name: n for n in board.nets.values()}
    nc = board.netclass_for("A")
    clearance = nc.clearance_nm * (1 + CLEARANCE_MARGIN)
    width = float(nc.track_width_nm)
    need = width / 2 + clearance + GUARDBAND_NM
    polygon = board_polygon(board)
    meshes = {}
    for layer in usable:
        mesh = build_mesh(_cores(board, layer), polygon, clearance, width)
        for i in range(len(mesh.triangles)):
            if mesh.free[i] and not _inside(polygon, *mesh.centroid(i)):
                mesh.free[i] = False
        meshes[layer] = mesh
    via_r = nc.via_diameter_nm / 2.0
    sites = via_sites([meshes[l] for l in usable], via_r, clearance,
                      inside=lambda x, y: _inside(polygon, x, y))
    weaves = {layer: Weave(meshes[layer]) for layer in usable}

    a1, a2 = list(nets["A"].pads)
    sa, ga = (float(a1.x), float(a1.y)), (float(a2.x), float(a2.y))
    wall = weaves["F.Cu"].insert(1, sa, ga, meshes["F.Cu"].terminals(*sa),
                                 meshes["F.Cu"].terminals(*ga), need=need,
                                 clearance=clearance, net=1)
    assert wall.found

    b1, b2 = list(nets["B"].pads)
    sb, gb = (float(b1.x), float(b1.y)), (float(b2.x), float(b2.y))
    direct = weaves["F.Cu"].insert(2, sb, gb, meshes["F.Cu"].terminals(*sb),
                                   meshes["F.Cu"].terminals(*gb), need=need,
                                   clearance=clearance, net=2)
    assert not direct.found, "B must be shut out of the gate for this test to mean anything"

    parent = _Link(key=7, net=nets["B"], pad_a=b1, pad_b=b2, span=1.0, halo=0.0,
                   width=width, clearance=clearance)
    gifts = _pocket_sites(board, meshes, usable, polygon, parent, len(sites),
                          via_r + clearance)
    sites.extend(gifts)
    route = escape_route(weaves, usable, sites, parent, need, clearance, 2.0, set(),
                         gifts)
    assert route is not None and route.found
    assert len(route.vias) in (1, 2)
    layers = [leg.layer for leg in route.legs]
    assert 1 in layers, "the escape must use the back layer"

    # and the answer really weaves, leg by leg, with real keys
    for index, leg in enumerate(route.legs):
        weave = weaves[usable[leg.layer]]
        got = weave.insert(10 + index, leg.start, leg.goal,
                           weave.mesh.terminals(*leg.start),
                           weave.mesh.terminals(*leg.goal), need=need,
                           clearance=clearance, net=2)
        assert got.found
