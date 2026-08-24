"""KiCad plugin entry point: route the open board with taut strings.

KiCad launches this as a subprocess with ``KICAD_API_SOCKET`` and ``KICAD_API_TOKEN`` in the
environment; :class:`kipy.KiCad` picks both up on its own.

The board is read from disk with :mod:`taut.board` rather than over the API, because that is
the path the headless runs exercise and it keeps one implementation instead of two. Only the
*write* side goes through the API, inside a single commit so the whole route is one undo step.
"""

from __future__ import annotations

import sys
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from taut.board import load_board                            # noqa: E402
from taut.route import ArcTrack as TautArc                   # noqa: E402
from taut.route import Track as TautTrack                    # noqa: E402
from taut.route import route_board                           # noqa: E402


def board_path(document) -> Path:
    """Locate the .kicad_pcb on disk from the API's document specifier."""
    filename = Path(document.board_filename)
    if filename.is_absolute() and filename.exists():
        return filename
    project = getattr(document, "project", None)
    project_path = getattr(project, "path", "") if project is not None else ""
    if project_path:
        candidate = Path(project_path) / filename.name
        if candidate.exists():
            return candidate
    raise SystemExit(
        f"cannot locate the board on disk (board_filename={document.board_filename!r}). "
        "Save the board once and try again."
    )


def layer_id(board_layer_enum, name: str) -> int:
    """"F.Cu" -> BoardLayer.BL_F_Cu, "In1.Cu" -> BoardLayer.BL_In1_Cu."""
    attribute = "BL_" + name.replace(".", "_")
    value = getattr(board_layer_enum, attribute, None)
    if value is None:
        raise SystemExit(f"KiCad has no layer {name!r}")
    return int(value)


def main() -> int:
    try:
        from kipy import KiCad
        from kipy.board_types import ArcTrack, BoardLayer, Track
        from kipy.geometry import Vector2
    except ImportError:
        print("kicad-python is not installed in the plugin's environment "
              "(pip install kicad-python)")
        return 1

    kicad = KiCad()
    live = kicad.get_board()
    source = board_path(live.document)

    board = load_board(source)
    layers = [layer for layer in board.copper_layers]
    print(f"routing {board.name}: {len(board.pads)} pads, {len(board.routable)} nets, "
          f"layers {layers}")

    result = route_board(board, layers=layers)

    nets_by_code = {}
    for net in live.get_nets():
        code = getattr(net, "code", None)
        nets_by_code[code if code is not None else net.name] = net
    nets_by_name = {net.name: net for net in live.get_nets()}

    items = []
    for track in result.tracks:
        net_name = board.nets[track.net].name if track.net in board.nets else None
        live_net = nets_by_name.get(net_name)
        if isinstance(track, TautTrack):
            item = Track()
            item.start = Vector2.from_xy(track.x1, track.y1)
            item.end = Vector2.from_xy(track.x2, track.y2)
        elif isinstance(track, TautArc):
            item = ArcTrack()
            item.start = Vector2.from_xy(track.x1, track.y1)
            item.mid = Vector2.from_xy(track.xm, track.ym)
            item.end = Vector2.from_xy(track.x2, track.y2)
        else:
            continue
        item.width = int(track.width_nm)
        item.layer = layer_id(BoardLayer, track.layer)
        if live_net is not None:
            item.net = live_net
        items.append(item)

    commit = live.begin_commit()
    try:
        live.create_items(items)
        live.push_commit(commit, "Taut-string route")
    except Exception:
        live.drop_commit(commit)
        raise

    stats = result.stats
    print(f"placed {len(items)} pieces ({stats['arcs']} arcs), "
          f"{stats['length_mm']} mm, {stats['routed']}/{stats['connections']} connections")
    if result.failed:
        print(f"{len(result.failed)} connections could not be routed:")
        for code, name, reason in result.failed[:10]:
            print(f"  {name}: {reason[:100]}")
    print("Run DRC to check the result.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SystemExit:
        raise
    except Exception:
        traceback.print_exc()
        raise SystemExit(1)
