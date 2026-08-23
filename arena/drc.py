"""The internal clearance checker -- the inner-loop query, not a DRC engine.

KiCad's DRC decides every score and is the only authority. This exists because a router
cannot call it: one S1 run on a 34-net board issues ~300,000 clearance queries, and at
1.47 s per ``kicad-cli`` invocation that is 124 hours for a single board and seed.

So this answers exactly one question -- *is any copper of another net closer than the rule
allows* -- and does so conservatively. Every approximation here errs toward reporting copper
as **closer** than it really is:

* pads are modelled as their bounding rectangle, so ovals and roundrects read as slightly
  larger than they are;
* the clearance required between two nets is the **larger** of the two netclass values;
* a guardband is added on top.

Being stricter than KiCad costs completions. Being looser would emit boards carrying
violations we never saw, and the run would be wasted. ``tests/test_drc.py`` pins that arrow
in one direction against KiCad itself.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import geometry as geo
from .problem import Obstacle, Pad, Problem
from .solution import Arc, Segment, Solution, Via
from .units import GUARDBAND_NM

__all__ = ["Piece", "Conflict", "ClearanceIndex", "check_solution"]

#: Sentinel net for geometry that belongs to no net and may never be touched.
NO_NET = -1

#: Sentinel layer set meaning "present on every copper layer" (vias, through-hole pads).
ALL_LAYERS = frozenset({"*"})


@dataclass(frozen=True, slots=True)
class Piece:
    """One piece of copper or one obstacle, as a centreline plus a halo radius."""

    kind: str                      # "seg" | "arc" | "disc" | "rect"
    net: int
    layers: frozenset[str]
    half_nm: float                 # inflation of the centreline (track half-width, via radius)
    coords: tuple[float, ...] = ()
    arc: geo.Arc | None = None
    label: str = ""

    def shares_layer(self, other: "Piece") -> bool:
        return (self.layers is ALL_LAYERS or other.layers is ALL_LAYERS
                or bool(self.layers & other.layers))

    def bbox(self) -> tuple[float, float, float, float]:
        if self.kind == "seg":
            x1, y1, x2, y2 = self.coords
            lo_x, hi_x = min(x1, x2), max(x1, x2)
            lo_y, hi_y = min(y1, y2), max(y1, y2)
        elif self.kind == "arc":
            arc = self.arc
            if arc is None or arc.degenerate:
                x1, y1, x2, y2 = self.coords
                lo_x, hi_x = min(x1, x2), max(x1, x2)
                lo_y, hi_y = min(y1, y2), max(y1, y2)
            else:
                # Conservative: the whole circle. Cheap, and only widens the candidate set.
                lo_x, hi_x = arc.cx - arc.r, arc.cx + arc.r
                lo_y, hi_y = arc.cy - arc.r, arc.cy + arc.r
        elif self.kind == "disc":
            cx, cy = self.coords
            lo_x, hi_x = cx, cx
            lo_y, hi_y = cy, cy
        else:  # rect: four corners
            xs = self.coords[0::2]
            ys = self.coords[1::2]
            lo_x, hi_x = min(xs), max(xs)
            lo_y, hi_y = min(ys), max(ys)
        return lo_x - self.half_nm, lo_y - self.half_nm, hi_x + self.half_nm, hi_y + self.half_nm


@dataclass(frozen=True, slots=True)
class Conflict:
    """Two pieces closer than the rule between them allows."""

    net_a: int
    net_b: int
    label_a: str
    label_b: str
    layer: str
    actual_nm: float
    required_nm: float
    x: float
    y: float

    @property
    def shortfall_nm(self) -> float:
        return self.required_nm - self.actual_nm


# --------------------------------------------------------------------------- distances

def _rect_edges(coords: tuple[float, ...]):
    for i in range(4):
        ax, ay = coords[2 * i], coords[2 * i + 1]
        bx, by = coords[2 * ((i + 1) % 4)], coords[2 * ((i + 1) % 4) + 1]
        yield ax, ay, bx, by


def _point_in_rect(px: float, py: float, coords: tuple[float, ...]) -> bool:
    inside = False
    for ax, ay, bx, by in _rect_edges(coords):
        if (ay > py) != (by > py):
            t = (py - ay) / (by - ay) if by != ay else 0.0
            if px < ax + t * (bx - ax):
                inside = not inside
    return inside


def _sample_points(piece: Piece) -> tuple[tuple[float, float], ...]:
    """A few representative points, used for the cheap inside-a-rect test."""
    if piece.kind == "seg":
        x1, y1, x2, y2 = piece.coords
        return ((x1, y1), ((x1 + x2) / 2, (y1 + y2) / 2), (x2, y2))
    if piece.kind == "arc":
        arc = piece.arc
        if arc is None or arc.degenerate:
            x1, y1, x2, y2 = piece.coords
            return ((x1, y1), (x2, y2))
        mid = arc.point_at_angle(arc.start_angle + arc.sweep / 2.0)
        return ((arc.x1, arc.y1), mid, (arc.x2, arc.y2))
    if piece.kind == "disc":
        return (tuple(piece.coords),)  # type: ignore[return-value]
    return tuple(zip(piece.coords[0::2], piece.coords[1::2]))


def centreline_distance(a: Piece, b: Piece) -> float:
    """Distance between two pieces' centrelines, ignoring their halos."""
    if a.kind == "rect" or b.kind == "rect":
        rect, other = (a, b) if a.kind == "rect" else (b, a)
        if other.kind == "rect":
            best = min(geo.segment_segment(*e1, *e2)
                       for e1 in _rect_edges(rect.coords)
                       for e2 in _rect_edges(other.coords))
            if best > 0.0 and _point_in_rect(other.coords[0], other.coords[1], rect.coords):
                return 0.0
            return best
        for px, py in _sample_points(other):
            if _point_in_rect(px, py, rect.coords):
                return 0.0
        return min(_distance_to_edge(other, edge) for edge in _rect_edges(rect.coords))

    if a.kind == "disc":
        return _point_distance(a.coords[0], a.coords[1], b)
    if b.kind == "disc":
        return _point_distance(b.coords[0], b.coords[1], a)

    if a.kind == "seg" and b.kind == "seg":
        return geo.segment_segment(*a.coords, *b.coords)
    if a.kind == "seg" and b.kind == "arc":
        return geo.segment_arc(*a.coords, b.arc)
    if a.kind == "arc" and b.kind == "seg":
        return geo.segment_arc(*b.coords, a.arc)
    return geo.arc_arc(a.arc, b.arc)


def _distance_to_edge(piece: Piece, edge) -> float:
    ax, ay, bx, by = edge
    if piece.kind == "seg":
        return geo.segment_segment(*piece.coords, ax, ay, bx, by)
    if piece.kind == "arc":
        return geo.segment_arc(ax, ay, bx, by, piece.arc)
    return geo.point_segment(piece.coords[0], piece.coords[1], ax, ay, bx, by)


def _point_distance(px: float, py: float, piece: Piece) -> float:
    if piece.kind == "seg":
        return geo.point_segment(px, py, *piece.coords)
    if piece.kind == "arc":
        return geo.point_arc(px, py, piece.arc)
    if piece.kind == "disc":
        return math.hypot(px - piece.coords[0], py - piece.coords[1])
    if _point_in_rect(px, py, piece.coords):
        return 0.0
    return min(geo.point_segment(px, py, *edge) for edge in _rect_edges(piece.coords))


# --------------------------------------------------------------------------- pieces

def pad_piece(pad: Pad, all_layers: tuple[str, ...]) -> Piece:
    """A pad as its bounding rectangle -- over-inclusive, therefore conservative."""
    hx, hy = pad.size_x / 2.0, pad.size_y / 2.0
    angle = math.radians(pad.angle)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    corners: list[float] = []
    for lx, ly in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
        corners.append(pad.x + lx * cos_a + ly * sin_a)
        corners.append(pad.y - lx * sin_a + ly * cos_a)

    layers = ALL_LAYERS if pad.drill_nm > 0 else frozenset(
        layer for layer in all_layers if pad.on_layer(layer))
    return Piece(kind="rect", net=pad.net if pad.net else NO_NET, layers=layers,
                 half_nm=0.0, coords=tuple(corners),
                 label=f"pad {pad.footprint}.{pad.number}")


def obstacle_pieces(obstacle: Obstacle) -> list[Piece]:
    if not obstacle.blocks_tracks:
        return []
    layers = ALL_LAYERS if "*.Cu" in obstacle.layers else frozenset(obstacle.layers)
    if obstacle.polygon:
        return [
            Piece(kind="seg", net=NO_NET, layers=layers, half_nm=0.0,
                  coords=(float(ax), float(ay), float(bx), float(by)),
                  label=f"{obstacle.kind} boundary")
            for (ax, ay), (bx, by) in zip(obstacle.polygon,
                                          obstacle.polygon[1:] + obstacle.polygon[:1])
        ]
    return [Piece(kind="disc", net=NO_NET, layers=layers, half_nm=float(obstacle.radius_nm),
                  coords=(float(obstacle.x), float(obstacle.y)), label=obstacle.kind)]


def item_piece(item, all_layers: tuple[str, ...]) -> Piece:
    if isinstance(item, Segment):
        return Piece(kind="seg", net=item.net, layers=frozenset({item.layer}),
                     half_nm=item.width_nm / 2.0,
                     coords=(float(item.x1), float(item.y1), float(item.x2), float(item.y2)),
                     label=f"track net {item.net}")
    if isinstance(item, Arc):
        arc = geo.Arc.from_three_points(item.x1, item.y1, item.xm, item.ym, item.x2, item.y2)
        return Piece(kind="arc", net=item.net, layers=frozenset({item.layer}),
                     half_nm=item.width_nm / 2.0,
                     coords=(float(item.x1), float(item.y1), float(item.x2), float(item.y2)),
                     arc=arc, label=f"arc net {item.net}")
    if isinstance(item, Via):
        return Piece(kind="disc", net=item.net, layers=ALL_LAYERS,
                     half_nm=item.diameter_nm / 2.0,
                     coords=(float(item.x), float(item.y)), label=f"via net {item.net}")
    raise TypeError(f"cannot index {type(item).__name__}")


# --------------------------------------------------------------------------- the index

class ClearanceIndex:
    """A bucketed spatial index over board copper, answering clearance queries."""

    def __init__(self, problem: Problem, guardband_nm: int = GUARDBAND_NM) -> None:
        self.problem = problem
        self.guardband = guardband_nm
        self.layers = problem.copper_layers
        self.edge_clearance = problem.rules.min_copper_edge_clearance_nm

        self._clearance: dict[int, int] = {}
        for net in problem.nets.values():
            self._clearance[net.code] = problem.rules.for_net(net.name).clearance_nm
        self._default_clearance = problem.rules.max_clearance_nm

        self.bucket = max(problem.rules.max_track_width_nm * 4,
                          problem.rules.max_clearance_nm * 8, 1_000_000)
        self._cells: dict[tuple[int, int], list[Piece]] = {}

        for pad in problem.pads:
            self.insert(pad_piece(pad, self.layers))
        for obstacle in problem.obstacles:
            for piece in obstacle_pieces(obstacle):
                self.insert(piece)

        self.edges: list[Piece] = []
        for edge in problem.edges:
            if edge.kind == "segment":
                self.edges.append(Piece(kind="seg", net=NO_NET, layers=ALL_LAYERS,
                                        half_nm=0.0, coords=(float(edge.x1), float(edge.y1),
                                                             float(edge.x2), float(edge.y2)),
                                        label="board edge"))
            else:
                arc = geo.Arc.from_three_points(edge.x1, edge.y1, edge.xm, edge.ym,
                                                edge.x2, edge.y2)
                self.edges.append(Piece(kind="arc", net=NO_NET, layers=ALL_LAYERS,
                                        half_nm=0.0, coords=(float(edge.x1), float(edge.y1),
                                                             float(edge.x2), float(edge.y2)),
                                        arc=arc, label="board edge"))

    # ------------------------------------------------------------------ bookkeeping

    def clearance_for(self, net: int) -> int:
        return self._clearance.get(net, self._default_clearance)

    def required_between(self, a: Piece, b: Piece) -> float:
        """The larger of the two nets' clearances, plus the guardband."""
        if a.net == b.net and a.net > 0:
            return 0.0
        wanted = max(self.clearance_for(a.net) if a.net > 0 else self._default_clearance,
                     self.clearance_for(b.net) if b.net > 0 else self._default_clearance)
        return wanted + self.guardband

    def _keys(self, piece: Piece):
        lo_x, lo_y, hi_x, hi_y = piece.bbox()
        margin = self._default_clearance + self.guardband
        for gx in range(int((lo_x - margin) // self.bucket),
                        int((hi_x + margin) // self.bucket) + 1):
            for gy in range(int((lo_y - margin) // self.bucket),
                            int((hi_y + margin) // self.bucket) + 1):
                yield gx, gy

    def insert(self, piece: Piece) -> None:
        for key in self._keys(piece):
            self._cells.setdefault(key, []).append(piece)

    def candidates(self, piece: Piece) -> list[Piece]:
        seen: dict[int, Piece] = {}
        for key in self._keys(piece):
            for other in self._cells.get(key, ()):
                seen[id(other)] = other
        return list(seen.values())

    # ------------------------------------------------------------------ queries

    def conflicts_for(self, piece: Piece, limit: int | None = None) -> list[Conflict]:
        """Everything already in the index that ``piece`` would come too close to."""
        found: list[Conflict] = []

        for other in self.candidates(piece):
            if other is piece or not piece.shares_layer(other):
                continue
            required = self.required_between(piece, other)
            if required <= 0.0:
                continue
            gap = centreline_distance(piece, other) - piece.half_nm - other.half_nm
            if gap < required:
                px, py = _sample_points(piece)[0]
                layer = next(iter(piece.layers)) if piece.layers is not ALL_LAYERS else "*"
                found.append(Conflict(net_a=piece.net, net_b=other.net,
                                      label_a=piece.label, label_b=other.label,
                                      layer=layer, actual_nm=max(0.0, gap),
                                      required_nm=required, x=px, y=py))
                if limit is not None and len(found) >= limit:
                    return found

        for edge in self.edges:
            gap = centreline_distance(piece, edge) - piece.half_nm
            required = self.edge_clearance + self.guardband
            if gap < required:
                px, py = _sample_points(piece)[0]
                found.append(Conflict(net_a=piece.net, net_b=NO_NET,
                                      label_a=piece.label, label_b="board edge",
                                      layer="*", actual_nm=max(0.0, gap),
                                      required_nm=required, x=px, y=py))
                if limit is not None and len(found) >= limit:
                    return found

        return found

    def is_clear(self, piece: Piece) -> bool:
        """Fast inner-loop query: may this piece be placed?"""
        return not self.conflicts_for(piece, limit=1)


def check_solution(problem: Problem, solution: Solution) -> list[Conflict]:
    """Every clearance conflict in a finished solution.

    Deliberately not a substitute for :mod:`arena.oracle` -- it checks clearance and board
    edge only, and says nothing about connectivity, annular rings, mask slivers or the two
    dozen other rules KiCad enforces. Its job is to agree with KiCad on the subset it does
    cover, and never to be more permissive.
    """
    index = ClearanceIndex(problem)
    pieces = [item_piece(item, problem.copper_layers) for item in solution.items]

    conflicts: list[Conflict] = []
    for piece in pieces:
        conflicts.extend(index.conflicts_for(piece))
        index.insert(piece)
    return conflicts
