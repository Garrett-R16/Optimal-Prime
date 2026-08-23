"""``Solution`` -> ``.kicad_pcb``.

Works by splicing into the token tree parsed from the original board, not by regenerating a
board from a model. Everything we do not understand is carried through verbatim.

UUIDs are derived deterministically from ``(strategy, seed, item index)`` rather than being
random, so re-running the same strategy on the same board with the same seed produces a
byte-identical file. That turns reproducibility into a ``diff`` rather than a statistical
argument, and makes an accidental nondeterminism bug immediately visible.
"""

from __future__ import annotations

import copy
import hashlib
from pathlib import Path

from . import sexpr
from .problem import Problem
from .sexpr import Node, num, quoted, sym
from .solution import Arc, Item, Segment, Solution, Via
from .units import nm_to_mm

__all__ = ["emit", "write_solution", "clear_routing", "deterministic_uuid"]

#: Board-level nodes that constitute routing. These are what we replace.
ROUTING_NODES = ("segment", "arc", "via")


def deterministic_uuid(strategy: str, seed: int, index: int) -> str:
    """A stable UUID-shaped identifier for the ``index``-th emitted item."""
    digest = hashlib.sha256(f"{strategy}|{seed}|{index}".encode("utf-8")).hexdigest()
    return f"{digest[0:8]}-{digest[8:12]}-{digest[12:16]}-{digest[16:20]}-{digest[20:32]}"


def clear_routing(tree: Node, nets: set[int] | None = None) -> int:
    """Strip existing routing from a board tree.

    With ``nets=None`` every segment, arc and via goes. Otherwise only those belonging to the
    named nets are removed, which is what an incremental strategy wants -- pre-existing
    copper on other nets stays put and is treated as an obstacle.
    """
    def is_doomed(child) -> bool:
        if not isinstance(child, list) or sexpr.head(child) not in ROUTING_NODES:
            return False
        if nets is None:
            return True
        net_node = sexpr.find(child, "net")
        code = net_node[1].as_int() if net_node is not None and len(net_node) > 1 else 0
        return code in nets

    keep = [c for c in tree if not is_doomed(c)]
    removed = len(tree) - len(keep)
    tree[:] = keep
    return removed


def _segment_node(item: Segment, uuid: str) -> Node:
    return [
        sym("segment"),
        [sym("start"), num(nm_to_mm(item.x1)), num(nm_to_mm(item.y1))],
        [sym("end"), num(nm_to_mm(item.x2)), num(nm_to_mm(item.y2))],
        [sym("width"), num(nm_to_mm(item.width_nm))],
        [sym("layer"), quoted(item.layer)],
        [sym("net"), num(item.net)],
        [sym("uuid"), quoted(uuid)],
    ]


def _arc_node(item: Arc, uuid: str) -> Node:
    return [
        sym("arc"),
        [sym("start"), num(nm_to_mm(item.x1)), num(nm_to_mm(item.y1))],
        [sym("mid"), num(nm_to_mm(item.xm)), num(nm_to_mm(item.ym))],
        [sym("end"), num(nm_to_mm(item.x2)), num(nm_to_mm(item.y2))],
        [sym("width"), num(nm_to_mm(item.width_nm))],
        [sym("layer"), quoted(item.layer)],
        [sym("net"), num(item.net)],
        [sym("uuid"), quoted(uuid)],
    ]


def _via_node(item: Via, uuid: str) -> Node:
    return [
        sym("via"),
        [sym("at"), num(nm_to_mm(item.x)), num(nm_to_mm(item.y))],
        [sym("size"), num(nm_to_mm(item.diameter_nm))],
        [sym("drill"), num(nm_to_mm(item.drill_nm))],
        [sym("layers"), quoted(item.layer_from), quoted(item.layer_to)],
        [sym("net"), num(item.net)],
        [sym("uuid"), quoted(uuid)],
    ]


def _node_for(item: Item, uuid: str) -> Node:
    if isinstance(item, Segment):
        return _segment_node(item, uuid)
    if isinstance(item, Arc):
        return _arc_node(item, uuid)
    if isinstance(item, Via):
        return _via_node(item, uuid)
    raise TypeError(f"cannot emit {type(item).__name__}")


def emit(problem: Problem, solution: Solution, strategy: str = "unknown",
         seed: int = 0, replace_nets: set[int] | None = None) -> str:
    """Render the board text carrying ``solution``'s routing.

    ``replace_nets`` defaults to the nets the solution actually touches, so copper on
    untouched nets survives. Pass an explicit set (or ``set()`` meaning *all*) to override.
    """
    tree = copy.deepcopy(problem.tree)

    if replace_nets is None:
        touched = {item.net for item in solution.items}
        replace_nets = touched | solution.routed_nets | solution.abandoned_nets
    clear_routing(tree, replace_nets or None)

    for index, item in enumerate(solution.items):
        tree.append(_node_for(item, deterministic_uuid(strategy, seed, index)))

    return sexpr.dumps(tree)


def write_solution(problem: Problem, solution: Solution, out_path: str | Path,
                   strategy: str = "unknown", seed: int = 0,
                   replace_nets: set[int] | None = None) -> Path:
    """Write the routed board, and copy the project file next to it.

    The project file must travel with the board: the design rules and DRC severities live
    there, so scoring a board without its ``.kicad_pro`` would silently use KiCad's defaults
    instead of the board's own rules.
    """
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        emit(problem, solution, strategy=strategy, seed=seed, replace_nets=replace_nets),
        encoding="utf-8",
        newline="\n",
    )

    if problem.project_path is not None:
        destination = out_path.with_suffix(".kicad_pro")
        destination.write_bytes(problem.project_path.read_bytes())

    return out_path
