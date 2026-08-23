"""Board file -> ``Problem`` IR.

Deliberately small: everything the frozen MVP-01 scope allows us to ignore is dropped here.
What survives is what a router needs to decide where copper may go, plus enough provenance to
put the result back into the same board file.

The one genuinely fiddly part is pad placement. A pad's ``(at ...)`` in the file is relative
to its footprint origin in the footprint's *unrotated* frame; the absolute position is the
footprint position plus that offset rotated by the footprint's orientation. KiCad rotates in
a Y-down coordinate system::

    x' = x*cos(a) + y*sin(a)
    y' = y*cos(a) - x*sin(a)

Getting the sign wrong puts every pad on a rotated part in the wrong place, and the symptom
is not a crash -- it is quietly unroutable boards. ``tests/test_problem.py`` pins the
convention against real boards by checking that existing track endpoints land inside pads.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field
from pathlib import Path

from . import sexpr
from .rules import DesignRules, load_design_rules, rules_hash
from .sexpr import Atom, Node
from .units import mm_to_nm

__all__ = ["Pad", "Net", "Obstacle", "Problem", "load_problem", "BoardLoadError"]


class BoardLoadError(ValueError):
    """Raised when a board cannot be turned into a Problem."""


# --------------------------------------------------------------------------- geometry bits

@dataclass(frozen=True, slots=True)
class Pad:
    """One pad, positioned absolutely on the board, in integer nanometres."""

    net: int
    number: str
    x: int
    y: int
    size_x: int
    size_y: int
    shape: str            # circle | rect | oval | roundrect | trapezoid | custom
    angle: float          # degrees, absolute (footprint orientation + pad rotation)
    layers: tuple[str, ...]
    drill_nm: int         # 0 for SMD
    footprint: str

    @property
    def is_smd(self) -> bool:
        return self.drill_nm == 0

    @property
    def radius_nm(self) -> int:
        """Radius of the enclosing circle -- a cheap conservative extent."""
        return int(math.hypot(self.size_x, self.size_y) / 2)

    def on_layer(self, layer: str) -> bool:
        for pattern in self.layers:
            if pattern == layer:
                return True
            if pattern == "*.Cu" and layer.endswith(".Cu"):
                return True
        return False

    def contains(self, x: int, y: int) -> bool:
        """Whether a point lies inside the pad, for the shapes MVP-01 needs.

        ``roundrect`` is treated as its bounding rectangle and ``custom`` as its enclosing
        circle -- both are *over*-inclusive, which is the safe direction for a connectivity
        test and the unsafe direction for a clearance test. This is not a clearance test.
        """
        dx, dy = x - self.x, y - self.y
        if self.angle:
            a = math.radians(self.angle)
            cos_a, sin_a = math.cos(a), math.sin(a)
            dx, dy = dx * cos_a - dy * sin_a, dy * cos_a + dx * sin_a
        hx, hy = self.size_x / 2, self.size_y / 2
        if self.shape == "circle":
            return math.hypot(dx, dy) <= max(hx, hy)
        if self.shape == "oval":
            if hx == 0 or hy == 0:
                return False
            return (dx / hx) ** 2 + (dy / hy) ** 2 <= 1.0
        if self.shape == "custom":
            return math.hypot(dx, dy) <= self.radius_nm
        return abs(dx) <= hx and abs(dy) <= hy


@dataclass(frozen=True, slots=True)
class Net:
    code: int
    name: str
    pads: tuple[Pad, ...]

    @property
    def is_routable(self) -> bool:
        """A net needs at least two pads to be worth routing."""
        return self.code != 0 and len(self.pads) >= 2


@dataclass(frozen=True, slots=True)
class Obstacle:
    """Anything copper must avoid that is not a pad: keepouts, holes, locked copper.

    A hole is a disc (``polygon`` is empty). A keepout is a polygon -- approximating one by
    its bounding circle is catastrophic for the long thin keepouts real boards use: on
    multichannel_mixer four slot keepouts became four 53.7 mm discs on a 110 mm board and
    blocked 95% of the routing area.
    """

    kind: str
    x: int
    y: int
    radius_nm: int
    layers: tuple[str, ...]
    polygon: tuple[tuple[int, int], ...] = ()
    blocks_tracks: bool = True
    blocks_vias: bool = True


@dataclass
class Problem:
    """Everything a strategy is given, plus what emit.py needs to put the answer back."""

    name: str
    tree: Node                       # the full token tree, preserved for lossless emission
    copper_layers: tuple[str, ...]   # ordered front -> back
    nets: dict[int, Net]
    pads: tuple[Pad, ...]
    obstacles: tuple[Obstacle, ...]
    rules: DesignRules
    bbox_nm: tuple[int, int, int, int]   # xmin, ymin, xmax, ymax of the board outline
    board_sha256: str
    rules_sha256: str
    source_path: Path
    project_path: Path | None
    net_names: dict[int, str] = field(default_factory=dict)

    @property
    def layer_count(self) -> int:
        return len(self.copper_layers)

    @property
    def routable_nets(self) -> list[Net]:
        return [n for n in self.nets.values() if n.is_routable]

    def summary(self) -> dict:
        return {
            "name": self.name,
            "layers": self.layer_count,
            "nets": len(self.routable_nets),
            "pads": len(self.pads),
            "sha256": self.board_sha256,
            "rules_sha256": self.rules_sha256,
        }


# --------------------------------------------------------------------------- extraction

def _rotate(x: float, y: float, degrees: float) -> tuple[float, float]:
    """KiCad's RotatePoint, in its Y-down coordinate system."""
    if not degrees:
        return x, y
    a = math.radians(degrees)
    cos_a, sin_a = math.cos(a), math.sin(a)
    return x * cos_a + y * sin_a, y * cos_a - x * sin_a


def _numbers(node: Node | None) -> list[float]:
    """Numeric atoms of a node, skipping symbolic ones.

    KiCad mixes symbols into numeric payloads -- ``(drill oval 0.6 1.2)``,
    ``(at 1 2 90)`` -- so a strict float conversion over every atom throws on real boards.
    """
    if node is None:
        return []
    out: list[float] = []
    for atom in sexpr.atoms(node):
        try:
            out.append(float(atom.text))
        except ValueError:
            continue
    return out


def _at(node: Node | None) -> tuple[float, float, float]:
    vals = _numbers(node)
    x = vals[0] if len(vals) > 0 else 0.0
    y = vals[1] if len(vals) > 1 else 0.0
    angle = vals[2] if len(vals) > 2 else 0.0
    return x, y, angle


def _layer_names(node: Node | None) -> tuple[str, ...]:
    if node is None:
        return ()
    return tuple(a.text for a in sexpr.atoms(node))


def _copper_layers(tree: Node) -> tuple[str, ...]:
    block = sexpr.find(tree, "layers")
    if block is None:
        raise BoardLoadError("board has no (layers ...) block")
    names: list[str] = []
    for entry in block:
        if not isinstance(entry, list) or len(entry) < 2:
            continue
        name = entry[1].text if isinstance(entry[1], Atom) else ""
        if name.endswith(".Cu"):
            names.append(name)
    if not names:
        raise BoardLoadError("board declares no copper layers")
    return tuple(names)


def _pads_of_footprint(fp: Node) -> list[Pad]:
    fx, fy, fangle = _at(sexpr.find(fp, "at"))
    ref = ""
    for prop in sexpr.find_all(fp, "property"):
        if len(prop) >= 3 and isinstance(prop[1], Atom) and prop[1].text == "Reference":
            ref = prop[2].text if isinstance(prop[2], Atom) else ""

    out: list[Pad] = []
    for pad in sexpr.find_all(fp, "pad"):
        pad_atoms = sexpr.atoms(pad)
        if len(pad_atoms) < 3:
            continue
        number, ptype, shape = pad_atoms[0].text, pad_atoms[1].text, pad_atoms[2].text
        if ptype == "np_thru_hole":
            continue  # mechanical hole: an obstacle, handled separately

        px, py, pangle = _at(sexpr.find(pad, "at"))
        rx, ry = _rotate(px, py, fangle)
        size = _numbers(sexpr.find(pad, "size")) or [0.0, 0.0]

        # (drill 0.6), (drill oval 0.6 1.2) and (drill 0.6 (offset 0 0.2)) all occur.
        # Take the smallest numeric payload: for an oval drill that is the minor axis, which
        # is the conservative choice for anything that has to fit around it.
        drill_vals = _numbers(sexpr.find(pad, "drill"))
        drill = min(drill_vals) if drill_vals else 0.0

        net_node = sexpr.find(pad, "net")
        net_code = net_node[1].as_int() if net_node is not None and len(net_node) > 1 else 0

        out.append(Pad(
            net=net_code,
            number=number,
            x=mm_to_nm(fx + rx),
            y=mm_to_nm(fy + ry),
            size_x=mm_to_nm(size[0]),
            size_y=mm_to_nm(size[1] if len(size) > 1 else size[0]),
            shape=shape,
            angle=fangle + pangle,
            layers=_layer_names(sexpr.find(pad, "layers")),
            drill_nm=mm_to_nm(drill),
            footprint=ref,
        ))
    return out


def _obstacles(tree: Node) -> list[Obstacle]:
    out: list[Obstacle] = []
    for fp in sexpr.find_all(tree, "footprint"):
        fx, fy, fangle = _at(sexpr.find(fp, "at"))
        for pad in sexpr.find_all(fp, "pad"):
            pad_atoms = sexpr.atoms(pad)
            if len(pad_atoms) < 2 or pad_atoms[1].text != "np_thru_hole":
                continue
            px, py, _ = _at(sexpr.find(pad, "at"))
            rx, ry = _rotate(px, py, fangle)
            size = _numbers(sexpr.find(pad, "size")) or [0.0]
            out.append(Obstacle(
                kind="npth",
                x=mm_to_nm(fx + rx),
                y=mm_to_nm(fy + ry),
                radius_nm=mm_to_nm(size[0]) // 2,
                layers=("*.Cu",),
            ))
    for zone in sexpr.find_all(tree, "zone"):
        keepout = sexpr.find(zone, "keepout")
        if keepout is None:
            continue

        # A keepout says which *kinds* of thing are forbidden. One that only excludes copper
        # pours must not block tracks, or whole boards become unroutable for no reason.
        def forbids(what: str) -> bool:
            rule = sexpr.find(keepout, what)
            return rule is not None and len(rule) > 1 and rule[1].text == "not_allowed"

        blocks_tracks = forbids("tracks")
        blocks_vias = forbids("vias")
        if not (blocks_tracks or blocks_vias):
            continue

        poly = sexpr.find(zone, "polygon")
        pts = sexpr.find(poly, "pts") if poly is not None else None
        if pts is None:
            continue
        points: list[tuple[int, int]] = []
        for pt in sexpr.find_all(pts, "xy"):
            vals = _numbers(pt)
            if len(vals) >= 2:
                points.append((mm_to_nm(vals[0]), mm_to_nm(vals[1])))
        if len(points) < 3:
            continue

        xs = [px for px, _ in points]
        ys = [py for _, py in points]
        out.append(Obstacle(
            kind="keepout",
            x=(min(xs) + max(xs)) // 2,
            y=(min(ys) + max(ys)) // 2,
            radius_nm=max(max(xs) - min(xs), max(ys) - min(ys)) // 2,
            layers=_layer_names(sexpr.find(zone, "layers")) or ("*.Cu",),
            polygon=tuple(points),
            blocks_tracks=blocks_tracks,
            blocks_vias=blocks_vias,
        ))
    return out


def _outline_bbox(tree: Node) -> tuple[int, int, int, int]:
    xs: list[float] = []
    ys: list[float] = []

    def add(vals: list[float]) -> None:
        for i in range(0, len(vals) - 1, 2):
            xs.append(vals[i])
            ys.append(vals[i + 1])

    for node in sexpr.walk(tree):
        layer = sexpr.find(node, "layer")
        if layer is None or len(layer) < 2 or not isinstance(layer[1], Atom):
            continue
        if layer[1].text != "Edge.Cuts":
            continue
        for key in ("start", "end", "mid", "center", "at"):
            add(_numbers(sexpr.find(node, key))[:2])
        pts = sexpr.find(node, "pts")
        if pts is not None:
            for pt in sexpr.find_all(pts, "xy"):
                add(_numbers(pt)[:2])

    if not xs:
        # No Edge.Cuts: fall back to the extent of the footprints themselves.
        for fp in sexpr.find_all(tree, "footprint"):
            fx, fy, _ = _at(sexpr.find(fp, "at"))
            xs.append(fx)
            ys.append(fy)
    if not xs:
        raise BoardLoadError("board has neither a board outline nor any footprints")
    return mm_to_nm(min(xs)), mm_to_nm(min(ys)), mm_to_nm(max(xs)), mm_to_nm(max(ys))


def load_problem(board_path: str | Path, project_path: str | Path | None = None) -> Problem:
    """Parse a ``.kicad_pcb`` (and its sibling ``.kicad_pro``) into a :class:`Problem`."""
    board_path = Path(board_path)
    raw = board_path.read_bytes()
    text = raw.decode("utf-8")

    try:
        tree = sexpr.parse(text)
    except sexpr.SExprError as exc:
        raise BoardLoadError(f"{board_path.name}: {exc}") from exc
    if sexpr.head(tree) != "kicad_pcb":
        raise BoardLoadError(f"{board_path.name}: not a kicad_pcb file")

    if project_path is None:
        candidate = board_path.with_suffix(".kicad_pro")
        project_path = candidate if candidate.exists() else None
    project_path = Path(project_path) if project_path else None

    if project_path is None:
        raise BoardLoadError(
            f"{board_path.name}: no .kicad_pro alongside the board. The design rules live "
            "there, not in the board file, so scoring without it would be meaningless."
        )

    rules = load_design_rules(project_path)

    net_names: dict[int, str] = {}
    for net in sexpr.find_all(tree, "net"):
        if len(net) >= 3 and isinstance(net[1], Atom) and isinstance(net[2], Atom):
            net_names[net[1].as_int()] = net[2].text

    pads: list[Pad] = []
    for fp in sexpr.find_all(tree, "footprint"):
        pads.extend(_pads_of_footprint(fp))

    by_net: dict[int, list[Pad]] = {}
    for pad in pads:
        by_net.setdefault(pad.net, []).append(pad)

    nets = {
        code: Net(code=code, name=net_names.get(code, f"net{code}"), pads=tuple(members))
        for code, members in by_net.items()
    }

    return Problem(
        name=board_path.stem,
        tree=tree,
        copper_layers=_copper_layers(tree),
        nets=nets,
        pads=tuple(pads),
        obstacles=tuple(_obstacles(tree)),
        rules=rules,
        bbox_nm=_outline_bbox(tree),
        board_sha256=hashlib.sha256(raw).hexdigest(),
        rules_sha256=rules_hash(project_path),
        source_path=board_path,
        project_path=project_path,
        net_names=net_names,
    )
