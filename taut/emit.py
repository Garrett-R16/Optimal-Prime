"""Write routed copper back into a ``.kicad_pcb``.

Splices into the token tree parsed from the original board rather than regenerating one, so
every field KiCad knows about and we do not is carried through untouched.

Arcs are emitted as native ``(arc (start ...) (mid ...) (end ...))`` -- KiCad's own
three-point form. Nothing is flattened to a polyline on the way out: the taut path is made of
segments and arcs, and so is the file.
"""

from __future__ import annotations

import copy
import hashlib
from pathlib import Path

from . import sexpr
from .board import Board
from .route import ArcTrack, RouteResult, Track
from .sexpr import Node, num, quoted, sym
from .units import nm_to_mm

__all__ = ["emit", "write_board", "strip_copper"]

ROUTING_NODES = ("segment", "arc", "via")


def _uuid(seed: str, index: int) -> str:
    digest = hashlib.sha256(f"{seed}|{index}".encode("utf-8")).hexdigest()
    return f"{digest[0:8]}-{digest[8:12]}-{digest[12:16]}-{digest[16:20]}-{digest[20:32]}"


def strip_copper(tree: Node, drop_zones: bool = True) -> tuple[int, int]:
    """Remove existing tracks, vias and (optionally) filled copper pours.

    Pours have to go for a re-route to mean anything: KiCad recomputes a zone's fill around
    whatever tracks exist, ``kicad-cli`` has no zone-fill command, and a retained pour keeps
    the fill computed for the *original* routing -- so every new track reads as a clearance
    violation against copper that would not be there in reality. Keepout zones stay.
    """
    kept: list = []
    tracks = zones = 0
    for child in tree:
        if isinstance(child, list):
            head = sexpr.head(child)
            if head in ROUTING_NODES:
                tracks += 1
                continue
            if head == "zone" and drop_zones and sexpr.find(child, "keepout") is None:
                zones += 1
                continue
        kept.append(child)
    tree[:] = kept
    return tracks, zones


def _segment_node(track: Track, uuid: str) -> Node:
    return [
        sym("segment"),
        [sym("start"), num(nm_to_mm(track.x1)), num(nm_to_mm(track.y1))],
        [sym("end"), num(nm_to_mm(track.x2)), num(nm_to_mm(track.y2))],
        [sym("width"), num(nm_to_mm(track.width_nm))],
        [sym("layer"), quoted(track.layer)],
        [sym("net"), num(track.net)],
        [sym("uuid"), quoted(uuid)],
    ]


def _via_node(via, layers: tuple[str, ...], uuid: str) -> Node:
    """A through via, named by the outermost copper layers it joins."""
    return [
        sym("via"),
        [sym("at"), num(nm_to_mm(via.x)), num(nm_to_mm(via.y))],
        [sym("size"), num(nm_to_mm(via.diameter_nm))],
        [sym("drill"), num(nm_to_mm(via.drill_nm))],
        [sym("layers"), quoted(layers[0]), quoted(layers[-1])],
        [sym("net"), num(via.net)],
        [sym("uuid"), quoted(uuid)],
    ]


def _arc_node(track: ArcTrack, uuid: str) -> Node:
    return [
        sym("arc"),
        [sym("start"), num(nm_to_mm(track.x1)), num(nm_to_mm(track.y1))],
        [sym("mid"), num(nm_to_mm(track.xm)), num(nm_to_mm(track.ym))],
        [sym("end"), num(nm_to_mm(track.x2)), num(nm_to_mm(track.y2))],
        [sym("width"), num(nm_to_mm(track.width_nm))],
        [sym("layer"), quoted(track.layer)],
        [sym("net"), num(track.net)],
        [sym("uuid"), quoted(uuid)],
    ]


def emit(board: Board, result: RouteResult, seed: str = "taut") -> str:
    """Render the board text carrying ``result``'s copper."""
    tree = copy.deepcopy(board.tree)
    strip_copper(tree)
    for index, track in enumerate(result.tracks):
        uuid = _uuid(seed, index)
        tree.append(_arc_node(track, uuid) if isinstance(track, ArcTrack)
                    else _segment_node(track, uuid))

    copper = tuple(board.copper_layers)
    for index, via in enumerate(result.vias):
        tree.append(_via_node(via, copper, _uuid(seed, len(result.tracks) + index)))
    return sexpr.dumps(tree)


def write_board(board: Board, result: RouteResult, out_path: str | Path,
                seed: str = "taut") -> Path:
    """Write the routed board, carrying the project file alongside it.

    The project must travel with the board: design rules and DRC severities live there, so
    checking a board without it would silently use KiCad's defaults instead of the board's.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(emit(board, result, seed), encoding="utf-8", newline="\n")
    out_path.with_suffix(".kicad_pro").write_bytes(board.project.read_bytes())
    return out_path
