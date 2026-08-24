"""Minimal KiCad board reader -- only what taut-string routing needs.

Pads, copper layers, netclass geometry, and the board outline. Nothing else.

Two conventions here are easy to get wrong, and neither failure crashes -- both just quietly
misplace copper.

**Position.** A pad's ``(at ...)`` offset is relative to its footprint origin in the
footprint's *unrotated* frame, and KiCad rotates in a Y-down coordinate system::

    x' = x*cos(a) + y*sin(a)
    y' = y*cos(a) - x*sin(a)

Pinned empirically against KiCad's demo boards: this sign reaches 100% of connected pads on
pic_programmer, where the opposite reaches 12 points fewer on every board tested.

**Orientation.** The angle in a pad's ``(at x y angle)`` is the pad's *absolute* orientation,
not an offset to be added to the footprint's. Adding them is wrong by exactly the footprint
angle, which for a rotated part turns a tall pad into a wide one.

That was also pinned empirically, by a property the boards themselves guarantee: KiCad reports
no overlapping pads on these demos, so the correct model cannot make two pads overlap. Adding
the angles invents 26 pad-pad overlaps on `sonde xilinx`; taking the pad's own angle invents
none.
"""

from __future__ import annotations

import fnmatch
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from . import sexpr
from .sexpr import Atom, Node
from .units import mm_to_nm

__all__ = ["Pad", "Net", "NetClass", "Edge", "CopperShape", "Board",
           "load_board", "BoardError"]


class BoardError(ValueError):
    """The board cannot be read, or lacks something routing needs."""


@dataclass(frozen=True, slots=True)
class Pad:
    net: int
    number: str
    x: int
    y: int
    size_x: int
    size_y: int
    shape: str
    angle: float                 # the orientation we apply to the pad's own rectangle
    layers: tuple[str, ...]
    drill_nm: int
    footprint: str
    #: The two contributions to `angle`, kept apart so the convention can be re-checked.
    footprint_angle: float = 0.0
    pad_angle: float = 0.0

    @property
    def is_through_hole(self) -> bool:
        return self.drill_nm > 0

    @property
    def radius_nm(self) -> float:
        return math.hypot(self.size_x, self.size_y) / 2.0

    def on_layer(self, layer: str) -> bool:
        return any(p == layer or p == "*.Cu" for p in self.layers)


@dataclass(frozen=True, slots=True)
class Edge:
    """One piece of the board outline. Copper must keep its distance from these."""

    kind: str          # "segment" | "arc"
    x1: int
    y1: int
    x2: int
    y2: int
    xm: int = 0
    ym: int = 0


@dataclass(frozen=True, slots=True)
class CopperShape:
    """A graphic drawn directly on a copper layer -- text, a line, a rectangle, a polygon.

    Boards really do carry these: silkscreen-style labels placed on copper, keep-out
    outlines, logos. They belong to no net, so a track must clear them like anything else.
    Missing them entirely is worth a clearance violation apiece.
    """

    layer: str
    vertices: tuple[tuple[int, int], ...]
    width_nm: int = 0
    label: str = "copper graphic"


@dataclass(frozen=True, slots=True)
class NetClass:
    name: str
    clearance_nm: int
    track_width_nm: int
    via_diameter_nm: int
    via_drill_nm: int


@dataclass(frozen=True, slots=True)
class Net:
    code: int
    name: str
    pads: tuple[Pad, ...]

    @property
    def needs_routing(self) -> bool:
        return self.code != 0 and len(self.pads) >= 2


@dataclass
class Board:
    name: str
    tree: Node
    copper_layers: tuple[str, ...]
    pads: tuple[Pad, ...]
    nets: dict[int, Net]
    classes: dict[str, NetClass]
    patterns: tuple[tuple[str, str], ...]
    bbox_nm: tuple[int, int, int, int]
    edges: tuple[Edge, ...]
    copper_shapes: tuple[CopperShape, ...]
    edge_clearance_nm: int
    source: Path
    project: Path

    def netclass_for(self, net_name: str) -> NetClass:
        for pattern, cls in self.patterns:
            if fnmatch.fnmatchcase(net_name, pattern) and cls in self.classes:
                return self.classes[cls]
        return self.classes["Default"]

    @property
    def routable(self) -> list[Net]:
        return [n for n in self.nets.values() if n.needs_routing]


# --------------------------------------------------------------------------- helpers

def _numbers(node: Node | None) -> list[float]:
    """Numeric atoms, skipping symbols -- KiCad writes ``(drill oval 0.6 1.2)``."""
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
    return (vals[0] if len(vals) > 0 else 0.0,
            vals[1] if len(vals) > 1 else 0.0,
            vals[2] if len(vals) > 2 else 0.0)


def _rotate(x: float, y: float, degrees: float) -> tuple[float, float]:
    if not degrees:
        return x, y
    a = math.radians(degrees)
    cos_a, sin_a = math.cos(a), math.sin(a)
    return x * cos_a + y * sin_a, y * cos_a - x * sin_a


def _copper_layers(tree: Node) -> tuple[str, ...]:
    block = sexpr.find(tree, "layers")
    if block is None:
        raise BoardError("board has no (layers ...) block")
    names = [entry[1].text for entry in block
             if isinstance(entry, list) and len(entry) > 1 and isinstance(entry[1], Atom)
             and entry[1].text.endswith(".Cu")]
    if not names:
        raise BoardError("board declares no copper layers")
    return tuple(names)


def _pads(tree: Node) -> list[Pad]:
    out: list[Pad] = []
    for fp in sexpr.find_all(tree, "footprint"):
        fx, fy, fangle = _at(sexpr.find(fp, "at"))
        reference = ""
        for prop in sexpr.find_all(fp, "property"):
            if len(prop) >= 3 and isinstance(prop[1], Atom) and prop[1].text == "Reference":
                reference = prop[2].text if isinstance(prop[2], Atom) else ""

        for pad in sexpr.find_all(fp, "pad"):
            fields = sexpr.atoms(pad)
            if len(fields) < 3 or fields[1].text == "np_thru_hole":
                continue
            pad_at = sexpr.find(pad, "at")
            px, py, pangle = _at(pad_at)
            pad_declares_angle = len(_numbers(pad_at)) > 2
            rx, ry = _rotate(px, py, fangle)
            size = _numbers(sexpr.find(pad, "size")) or [0.0, 0.0]
            drill = _numbers(sexpr.find(pad, "drill"))
            net_node = sexpr.find(pad, "net")
            out.append(Pad(
                net=net_node[1].as_int() if net_node is not None and len(net_node) > 1 else 0,
                number=fields[0].text,
                x=mm_to_nm(fx + rx), y=mm_to_nm(fy + ry),
                size_x=mm_to_nm(size[0]),
                size_y=mm_to_nm(size[1] if len(size) > 1 else size[0]),
                # The pad's own angle is absolute when it declares one; otherwise it
                # inherits the footprint's. See the note at the top of this module.
                shape=fields[2].text, angle=pangle if pad_declares_angle else fangle,
                footprint_angle=fangle, pad_angle=pangle,
                layers=tuple(a.text for a in sexpr.atoms(sexpr.find(pad, "layers") or [])),
                drill_nm=mm_to_nm(min(drill)) if drill else 0,
                footprint=reference,
            ))
    return out


def _edges(tree: Node) -> list[Edge]:
    """Every Edge.Cuts graphic, reduced to segments and arcs."""
    out: list[Edge] = []
    for node in sexpr.walk(tree):
        layer = sexpr.find(node, "layer")
        if layer is None or len(layer) < 2 or not isinstance(layer[1], Atom):
            continue
        if layer[1].text != "Edge.Cuts":
            continue
        kind = sexpr.head(node)
        start = _numbers(sexpr.find(node, "start"))
        end = _numbers(sexpr.find(node, "end"))
        if kind == "gr_line" and len(start) >= 2 and len(end) >= 2:
            out.append(Edge("segment", mm_to_nm(start[0]), mm_to_nm(start[1]),
                            mm_to_nm(end[0]), mm_to_nm(end[1])))
        elif kind == "gr_arc":
            mid = _numbers(sexpr.find(node, "mid"))
            if len(start) >= 2 and len(mid) >= 2 and len(end) >= 2:
                out.append(Edge("arc", mm_to_nm(start[0]), mm_to_nm(start[1]),
                                mm_to_nm(end[0]), mm_to_nm(end[1]),
                                mm_to_nm(mid[0]), mm_to_nm(mid[1])))
        elif kind == "gr_rect" and len(start) >= 2 and len(end) >= 2:
            x0, y0 = mm_to_nm(start[0]), mm_to_nm(start[1])
            x1, y1 = mm_to_nm(end[0]), mm_to_nm(end[1])
            corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
            for i in range(4):
                out.append(Edge("segment", *corners[i], *corners[(i + 1) % 4]))
        elif kind == "gr_circle":
            centre = _numbers(sexpr.find(node, "center"))
            if len(centre) >= 2 and len(end) >= 2:
                cx, cy = mm_to_nm(centre[0]), mm_to_nm(centre[1])
                radius = int(math.dist((cx, cy), (mm_to_nm(end[0]), mm_to_nm(end[1]))))
                out.append(Edge("arc", cx - radius, cy, cx + radius, cy, cx, cy - radius))
                out.append(Edge("arc", cx + radius, cy, cx - radius, cy, cx, cy + radius))
        elif kind == "gr_poly":
            pts = sexpr.find(node, "pts")
            if pts is None:
                continue
            points = [tuple(mm_to_nm(v) for v in _numbers(pt)[:2])
                      for pt in sexpr.find_all(pts, "xy") if len(_numbers(pt)) >= 2]
            for i in range(len(points)):
                out.append(Edge("segment", *points[i], *points[(i + 1) % len(points)]))
    return out


#: Advance width of a stroke-font glyph, as a fraction of the nominal text size.
#:
#: KiCad's stroke font draws each glyph in roughly a unit cell and advances by about one
#: size per character. Estimating low leaves the box too small and lets copper run through
#: the ends of a label, so this errs high: over-covering costs routing room, never
#: correctness.
_GLYPH_ADVANCE = 1.05


def _copper_shapes(tree: Node) -> list[CopperShape]:
    """Graphics drawn on copper layers, as convex-ish boxes and polylines."""
    out: list[CopperShape] = []

    for node in sexpr.walk(tree):
        layer_node = sexpr.find(node, "layer")
        if layer_node is None or len(layer_node) < 2 or not isinstance(layer_node[1], Atom):
            continue
        layer = layer_node[1].text
        if not layer.endswith(".Cu"):
            continue

        kind = sexpr.head(node)
        stroke = sexpr.find(node, "stroke")
        width = 0
        if stroke is not None:
            w = _numbers(sexpr.find(stroke, "width"))
            width = mm_to_nm(w[0]) if w else 0

        if kind in ("gr_line", "gr_arc", "gr_rect"):
            start = _numbers(sexpr.find(node, "start"))
            end = _numbers(sexpr.find(node, "end"))
            if len(start) < 2 or len(end) < 2:
                continue
            if kind == "gr_rect":
                x0, y0 = mm_to_nm(start[0]), mm_to_nm(start[1])
                x1, y1 = mm_to_nm(end[0]), mm_to_nm(end[1])
                verts = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
            else:
                verts = ((mm_to_nm(start[0]), mm_to_nm(start[1])),
                         (mm_to_nm(end[0]), mm_to_nm(end[1])))
            out.append(CopperShape(layer, verts, width, kind))

        elif kind in ("gr_poly", "gr_curve"):
            pts = sexpr.find(node, "pts")
            if pts is None:
                continue
            verts = tuple((mm_to_nm(v[0]), mm_to_nm(v[1]))
                          for v in (_numbers(pt) for pt in sexpr.find_all(pts, "xy"))
                          if len(v) >= 2)
            if verts:
                out.append(CopperShape(layer, verts, width, kind))

        elif kind == "gr_circle":
            centre = _numbers(sexpr.find(node, "center"))
            rim = _numbers(sexpr.find(node, "end"))
            if len(centre) >= 2 and len(rim) >= 2:
                cx, cy = mm_to_nm(centre[0]), mm_to_nm(centre[1])
                radius = int(math.dist((cx, cy), (mm_to_nm(rim[0]), mm_to_nm(rim[1]))))
                out.append(CopperShape(layer, ((cx, cy),), radius * 2, "gr_circle"))

        elif kind in ("gr_text", "text"):
            atoms = sexpr.atoms(node)
            if not atoms:
                continue
            content = atoms[0].text
            at = _numbers(sexpr.find(node, "at"))
            if len(at) < 2:
                continue
            effects = sexpr.find(node, "effects")
            font = sexpr.find(effects, "font") if effects is not None else None
            size = _numbers(sexpr.find(font, "size")) if font is not None else []
            thickness = _numbers(sexpr.find(font, "thickness")) if font is not None else []
            height = mm_to_nm(size[1]) if len(size) > 1 else 1_000_000
            glyph_w = mm_to_nm(size[0]) if size else 1_000_000
            stroke_w = mm_to_nm(thickness[0]) if thickness else height // 8

            # The stroke has width of its own, so the drawn extent exceeds the nominal box
            # by half a stroke on every side.
            half_w = int(len(content) * glyph_w * _GLYPH_ADVANCE / 2) + stroke_w // 2
            half_h = height // 2 + stroke_w // 2
            angle = at[2] if len(at) > 2 else 0.0
            cx, cy = mm_to_nm(at[0]), mm_to_nm(at[1])
            a = math.radians(angle)
            ca, sa = math.cos(a), math.sin(a)
            verts = tuple(
                (int(cx + lx * ca + ly * sa), int(cy - lx * sa + ly * ca))
                for lx, ly in ((-half_w, -half_h), (half_w, -half_h),
                               (half_w, half_h), (-half_w, half_h))
            )
            out.append(CopperShape(layer, verts, stroke_w, f"text {content!r}"))

    return out


def _bbox(tree: Node) -> tuple[int, int, int, int]:
    xs: list[float] = []
    ys: list[float] = []
    for node in sexpr.walk(tree):
        layer = sexpr.find(node, "layer")
        if layer is None or len(layer) < 2 or not isinstance(layer[1], Atom):
            continue
        if layer[1].text != "Edge.Cuts":
            continue
        for key in ("start", "end", "mid", "center", "at"):
            vals = _numbers(sexpr.find(node, key))[:2]
            if len(vals) == 2:
                xs.append(vals[0])
                ys.append(vals[1])
        pts = sexpr.find(node, "pts")
        if pts is not None:
            for pt in sexpr.find_all(pts, "xy"):
                vals = _numbers(pt)[:2]
                if len(vals) == 2:
                    xs.append(vals[0])
                    ys.append(vals[1])
    if not xs:
        raise BoardError("board has no Edge.Cuts outline")
    return mm_to_nm(min(xs)), mm_to_nm(min(ys)), mm_to_nm(max(xs)), mm_to_nm(max(ys))


def load_board(board_path: str | Path, project_path: str | Path | None = None) -> Board:
    """Read a ``.kicad_pcb`` and its sibling project file."""
    board_path = Path(board_path)
    tree = sexpr.parse(board_path.read_text(encoding="utf-8"))
    if sexpr.head(tree) != "kicad_pcb":
        raise BoardError(f"{board_path.name} is not a kicad_pcb file")

    project_path = Path(project_path) if project_path else board_path.with_suffix(".kicad_pro")
    if not project_path.exists():
        raise BoardError(f"{board_path.name}: no .kicad_pro -- the design rules live there")

    data = json.loads(project_path.read_text(encoding="utf-8"))
    net_settings = data.get("net_settings", {})
    design = data.get("board", {}).get("design_settings", {})
    raw_rules = design.get("rules", {})

    classes: dict[str, NetClass] = {}
    for entry in net_settings.get("classes", []):
        classes[entry.get("name", "Default")] = NetClass(
            name=entry.get("name", "Default"),
            clearance_nm=mm_to_nm(entry.get("clearance", 0.2)) or 200_000,
            track_width_nm=mm_to_nm(entry.get("track_width", 0.25)) or 250_000,
            via_diameter_nm=mm_to_nm(entry.get("via_diameter", 0.8)) or 800_000,
            via_drill_nm=mm_to_nm(entry.get("via_drill", 0.4)) or 400_000,
        )
    classes.setdefault("Default", NetClass("Default", 200_000, 250_000, 800_000, 400_000))

    # KiCad 9 assigns netclasses by glob pattern. Missing this silently gives every net the
    # Default rules, which surfaces much later as clearance violations on exactly the nets
    # that needed wider spacing.
    patterns = tuple(
        (str(e.get("pattern", "")), str(e.get("netclass", "Default")))
        for e in (net_settings.get("netclass_patterns") or []) if e.get("pattern")
    )

    net_names = {n[1].as_int(): n[2].text for n in sexpr.find_all(tree, "net")
                 if len(n) >= 3 and isinstance(n[1], Atom) and isinstance(n[2], Atom)}

    pads = _pads(tree)
    grouped: dict[int, list[Pad]] = {}
    for pad in pads:
        grouped.setdefault(pad.net, []).append(pad)

    return Board(
        name=board_path.stem,
        tree=tree,
        copper_layers=_copper_layers(tree),
        pads=tuple(pads),
        nets={code: Net(code, net_names.get(code, f"net{code}"), tuple(members))
              for code, members in grouped.items()},
        classes=classes,
        patterns=patterns,
        bbox_nm=_bbox(tree),
        edges=tuple(_edges(tree)),
        copper_shapes=tuple(_copper_shapes(tree)),
        edge_clearance_nm=mm_to_nm(raw_rules.get("min_copper_edge_clearance", 0.01)) or 10_000,
        source=board_path,
        project=project_path,
    )
