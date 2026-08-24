"""Obstacles, as discs.

The taut-string path among circular obstacles is *provably* a sequence of straight lines
tangent to those circles, joined by arcs riding on them -- nothing else. That is exactly and
only what KiCad copper can express (``segment`` and ``arc``), so the optimal geometry and the
expressible geometry are the same set and nothing is lost on the way out.

Everything a track must avoid is therefore reduced to a disc, or a chain of discs:

* a **pad** becomes one disc covering it, inflated by the clearance and half the track width;
* **already-routed copper** becomes a chain of discs laid along its centreline.

Discs over-cover rectangular and elongated shapes, which costs routing space and never costs
correctness -- the same one-directional conservatism the rest of the project runs on.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = ["Disc", "pad_disc", "discs_along_segment", "discs_along_arc"]


@dataclass(frozen=True, slots=True)
class Disc:
    """A circular keep-out. ``r`` already includes clearance and half the track width."""

    x: float
    y: float
    r: float
    net: int = 0
    label: str = ""

    def contains(self, px: float, py: float, tol: float = 1e-6) -> bool:
        return math.hypot(px - self.x, py - self.y) < self.r - tol

    def distance_to(self, other: "Disc") -> float:
        return math.hypot(other.x - self.x, other.y - self.y) - self.r - other.r


def pad_disc(pad, clearance_nm: float, half_track_nm: float) -> Disc:
    """The keep-out disc for a pad: its enclosing circle plus clearance plus half a track."""
    radius = math.hypot(pad.size_x, pad.size_y) / 2.0
    return Disc(x=float(pad.x), y=float(pad.y),
                r=radius + clearance_nm + half_track_nm,
                net=pad.net, label=f"{pad.footprint}.{pad.number}")


#: A chain of equal discs spaced ``s`` apart does not cover a strip of width ``r`` -- midway
#: between two centres the union pinches in to ``sqrt(r^2 - (s/2)^2)``. To guarantee that
#: nothing comes within ``d`` of the centreline, the discs are inflated and the spacing tied
#: to that inflation: waist = sqrt((1.15d)^2 - (0.45d)^2) = 1.06d, comfortably over ``d``.
#: Skipping this arithmetic leaves gaps a track can slip through, which is exactly how three
#: copper_edge_clearance violations survived a board outline that was already modelled.
_INFLATE = 1.15
_SPACING = 0.90


def discs_along_segment(x1: float, y1: float, x2: float, y2: float, radius: float,
                        net: int, spacing: float | None = None,
                        label: str = "track") -> list[Disc]:
    """Cover a segment with discs so nothing may come within ``radius`` of it."""
    disc_r = radius * _INFLATE
    spacing = spacing or max(radius * _SPACING, 1.0)
    length = math.hypot(x2 - x1, y2 - y1)
    steps = max(1, int(math.ceil(length / spacing)))
    return [
        Disc(x=x1 + (x2 - x1) * i / steps, y=y1 + (y2 - y1) * i / steps,
             r=disc_r, net=net, label=label)
        for i in range(steps + 1)
    ]


def discs_along_arc(arc, radius: float, net: int, spacing: float | None = None,
                    label: str = "track") -> list[Disc]:
    """Cover a routed arc with overlapping discs."""
    disc_r = radius * _INFLATE
    spacing = spacing or max(radius * _SPACING, 1.0)
    length = arc.r * abs(arc.sweep)
    steps = max(1, int(math.ceil(length / spacing)))
    out = []
    for i in range(steps + 1):
        theta = arc.start_angle + arc.sweep * i / steps
        out.append(Disc(x=arc.cx + arc.r * math.cos(theta),
                        y=arc.cy + arc.r * math.sin(theta),
                        r=disc_r, net=net, label=label))
    return out
