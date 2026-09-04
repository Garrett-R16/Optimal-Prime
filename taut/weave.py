"""The weave: wires inserted one at a time into a board that cannot be crossed.

The negotiation decides layers, vias and capacity jointly; what it cannot decide is which
side of each other the wires pass on, and every repair mechanism that inferred sides after
the fact spent its effort chasing geometry that was already wrong. Here sides are decided
the way the reference systems decide them: *by construction*. Wires are inserted
sequentially; each committed wire becomes chords in the triangles it passes through
(:mod:`taut.cells`); a later wire searches over *(triangle, cell)* states, and a step that
would cross a committed wire simply does not exist in its graph.

What comes out is the one artefact the embedding has always needed and never had: a crossing
order on every doorway that is planar by construction. With that in hand, placement order
stops mattering -- there is nothing left for arrival order to resolve.

The insertion order is a length policy, not a correctness one: shortest first, so the short
wires take their straight lines and the long wires -- which pay proportionally least --
thread around them. The pipeline measured 76 mm between orders on a 66-wire board; this is
where that prize is collected.

A wire the committed board genuinely separates from its goal (three mutually-crossing
connections on one layer, say) fails its weave, and the caller falls back to the tiered
pipeline for the whole board. Dynamic via insertion inside the weave is the successor to
that fallback, not part of it.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

from .cells import Chord, TriangleCells, point_parameter
from .mesh import Mesh

__all__ = ["WeaveResult", "Weave"]

_EPS = 1e-7


@dataclass
class WeaveResult:
    """One wire's committed passage: its corridor, and where it crosses each doorway."""

    key: int
    found: bool = False
    triangles: list[int] = field(default_factory=list)
    #: (portal key, fraction along the portal in canonical a->b direction)
    crossings: list[tuple[tuple[int, int], float]] = field(default_factory=list)


class Weave:
    """The committed state of one layer, and the insertion machinery."""

    def __init__(self, mesh: Mesh):
        self.mesh = mesh
        #: per triangle: committed chords
        self.chords: dict[int, list[Chord]] = {}
        #: per portal key: committed crossings as (fraction, wire key), kept sorted
        self.on_portal: dict[tuple[int, int], list[tuple[float, int]]] = {}
        self._cells: dict[int, TriangleCells] = {}
        self._corners: dict[int, list[tuple[float, float]]] = {}
        self._edge_of: dict[int, dict[tuple[int, int], tuple[int, bool]]] = {}
        #: per wire: the half-pitch it needs at a doorway (half width + clearance)
        self.need_of: dict[int, float] = {}
        #: per wire: its clearance, so neighbours share one gap instead of paying two
        self.clr_of: dict[int, float] = {}
        #: every result ever returned, kept live: re-seating updates their fractions
        self._results: dict[int, WeaveResult] = {}
        #: per (triangle, wire): the portals its chord's two ends sit on (None = terminal
        #: anchor), so the chord can be re-seated whenever a doorway's ranks are
        self._chord_ends: dict = {}
        #: per portal key: the free triangles on its two sides
        self._portal_tris: dict = {}

    # ------------------------------------------------------------------ geometry helpers

    def corners(self, tri: int) -> list[tuple[float, float]]:
        got = self._corners.get(tri)
        if got is None:
            got = [(float(p[0]), float(p[1]))
                   for p in self.mesh.points[self.mesh.triangles[tri]]]
            self._corners[tri] = got
        return got

    def cells(self, tri: int) -> TriangleCells:
        got = self._cells.get(tri)
        if got is None:
            got = TriangleCells.build(self.chords.get(tri, []))
            self._cells[tri] = got
        return got

    def _edges(self, tri: int) -> dict[tuple[int, int], tuple[int, bool]]:
        """Portal key -> (edge index, whether the key's a-vertex is the edge's start)."""
        got = self._edge_of.get(tri)
        if got is None:
            got = {}
            vertices = [int(v) for v in self.mesh.triangles[tri]]
            for edge in range(3):
                v1, v2 = vertices[edge], vertices[(edge + 1) % 3]
                key = (v1, v2) if v1 < v2 else (v2, v1)
                got[key] = (edge, key[0] == v1)
            self._edge_of[tri] = got
        return got

    def portal_param(self, tri: int, key: tuple[int, int], fraction: float) -> float:
        """Boundary parameter in ``tri`` of the point ``fraction`` along portal ``key``."""
        edge, forward = self._edges(tri)[key]
        f = fraction if forward else 1.0 - fraction
        return (edge + min(max(f, _EPS), 1.0 - _EPS)) % 3.0

    def open_lanes(self, tri: int, key: tuple[int, int]) -> list[tuple[float, float]]:
        """The uncrossed stretches of a doorway, as (from, to) fractions along it."""
        taken = sorted(fraction for fraction, _ in self.on_portal.get(key, ()))
        stops = [0.0] + taken + [1.0]
        return [(a, b) for a, b in zip(stops, stops[1:]) if b - a > _EPS * 4]

    # ------------------------------------------------------------------ terminals

    def terminal_cells(self, tri: int, x: float, y: float) -> list[int]:
        """The sectors of ``tri`` a wire anchored at (x, y) may leave by.

        For a pad terminal the anchor sits on copper that owns one or more of the
        triangle's corners, and the sectors are the cells flanking those corners. For a
        via terminal the anchor floats inside free space and separates nothing: the sector
        is simply the cell of the nearest boundary point.
        """
        vertices = [int(v) for v in self.mesh.triangles[tri]]
        cells = self.cells(tri)
        owner = self._owner_at(x, y)
        out: list[int] = []
        if owner is not None:
            for corner, vertex in enumerate(vertices):
                if self.mesh.owner[vertex] == owner:
                    for cell in cells.cells_at_corner(corner):
                        if cell not in out:
                            out.append(cell)
        if not out:
            t = point_parameter(self.corners(tri), x, y)
            out.append(cells.cell_at(t))
        return out

    def _owner_at(self, x: float, y: float):
        for index, obstacle in enumerate(self.mesh.obstacles):
            if obstacle.distance_to_point(x, y) <= obstacle.r + 1.0:
                return index
        return None

    def _anchor_param(self, tri: int, x: float, y: float, cell: int) -> float:
        """Where on ``tri``'s boundary the stub chord for this terminal anchors.

        The anchor must lie *in the given sector*: for a pad owning a corner that is the
        corner itself (nudged into the sector's side); for a floating via it is the nearest
        boundary point.
        """
        vertices = [int(v) for v in self.mesh.triangles[tri]]
        cells = self.cells(tri)
        owner = self._owner_at(x, y)
        if owner is not None:
            for corner, vertex in enumerate(vertices):
                if self.mesh.owner[vertex] != owner:
                    continue
                for sign in (1.0, -1.0):
                    t = (corner + sign * _EPS * 8) % 3.0
                    if cells.cell_at(t) == cell:
                        return t
        return point_parameter(self.corners(tri), x, y)

    # ------------------------------------------------------------------ the insertion

    def insert(self, key: int, start: tuple[float, float], goal: tuple[float, float],
               start_tris: list[int], goal_tris: list[int], need: float = 0.0,
               node_limit: int = 200_000,
               admit: float = 2.0, clearance: float = 0.0) -> WeaveResult:
        """Route one wire through the committed board and commit it.

        Dijkstra over *(triangle, cell)*; a transition exists only where an uncrossed lane
        of a shared doorway lies on the current cell's boundary. The result is committed
        immediately: its doorway crossings take their lane midpoints, and its passage
        through each triangle becomes a chord.
        """
        result = WeaveResult(key=key)

        starts: dict[tuple[int, int], float] = {}
        for tri in start_tris:
            for cell in self.terminal_cells(tri, start[0], start[1]):
                starts[(tri, cell)] = 0.0
        goals: set[tuple[int, int]] = set()
        for tri in goal_tris:
            for cell in self.terminal_cells(tri, goal[0], goal[1]):
                goals.add((tri, cell))
        if not starts or not goals:
            return result

        heap: list = []
        best: dict[tuple[int, int], float] = {}
        came: dict = {}
        where: dict[tuple[int, int], tuple[float, float]] = {}

        for state in starts:
            best[state] = 0.0
            came[state] = None
            where[state] = start
            heapq.heappush(heap, (math.dist(start, goal), 0.0, state))

        final = None
        expanded = 0
        while heap:
            _, cost, state = heapq.heappop(heap)
            if cost > best.get(state, math.inf) + _EPS:
                continue
            if state in goals:
                final = state
                break
            expanded += 1
            if expanded > node_limit:
                return result

            tri, cell = state
            cells = self.cells(tri)
            px, py = where[state]
            for other, portal in self.mesh.adjacent(tri):
                pkey = portal.key()
                span = portal.length
                sides = self._portal_tris.setdefault(pkey, set())
                sides.add(tri)
                sides.add(other)
                # Capacity is a COUNT, not a place. A doorway holds a wire if the width
                # its committed wires have not claimed still holds one more -- whichever
                # slot it takes. Committing wires where their straight lines crossed was
                # measured to make a three-wire doorway hold two or three depending on
                # insertion order: two near the middle left two useless slivers. Only the
                # order at a doorway is decided here; positions are re-seated by rank
                # after every commit, so free width is never fragmented.
                # One spare clearance beyond the exact fit: the embed seats by rank with
                # its own guard and margin, and a doorway filled to the last micron was
                # measured to cost sonde 32 mm in wraps downstream.
                if need > 0.0 and self._room(pkey, span) - clearance < admit * need:
                    continue
                for lane_from, lane_to in self.open_lanes(tri, pkey):
                    # Sit where the wire actually wants to pass: the straight line's own
                    # crossing of this doorway, clamped into the lane. Committing at lane
                    # midpoints made every wire an exaggerated obstacle to the ones after
                    # it, and the berth they gave it was pure length.
                    margin = min(0.45 * (lane_to - lane_from),
                                 max((need / span) if span > 1.0 else 0.0, 0.02))
                    want = self._line_fraction(pkey, start, goal)
                    lane_mid = (lane_from + lane_to) / 2.0 if want is None else                         min(max(want, lane_from + margin), lane_to - margin)
                    if cells.cell_at(self.portal_param(tri, pkey, lane_mid)) != cell:
                        continue
                    over = self.cells(other).cell_at(
                        self.portal_param(other, pkey, lane_mid))
                    nxt = (other, over)
                    mx, my = self._lane_point(pkey, lane_mid)
                    total = cost + math.hypot(mx - px, my - py)
                    if total < best.get(nxt, math.inf) - _EPS:
                        best[nxt] = total
                        came[nxt] = (state, pkey, lane_mid)
                        where[nxt] = (mx, my)
                        heapq.heappush(
                            heap, (total + math.hypot(mx - goal[0], my - goal[1]),
                                   total, nxt))

        if final is None:
            return result

        walk = []
        cursor = final
        while came[cursor] is not None:
            state, pkey, lane_mid = came[cursor]
            walk.append((cursor[0], pkey, lane_mid))
            cursor = state
        walk.reverse()

        result.found = True
        result.triangles = [cursor[0]] + [tri for tri, _, _ in walk]
        result.crossings = [(pkey, lane_mid) for _, pkey, lane_mid in walk]
        self.need_of[key] = need
        self.clr_of[key] = clearance
        self._results[key] = result
        self._commit(key, start, goal, cursor, final, result)
        for pkey, _ in result.crossings:
            self._reseat(pkey)
        result.crossings = [(pkey, self._fraction_of(pkey, key))
                            for pkey, _ in result.crossings]
        return result

    def _line_fraction(self, key: tuple[int, int], start, goal):
        """Where the straight start-goal line crosses this doorway, if it does."""
        pa, pb = self.mesh.points[key[0]], self.mesh.points[key[1]]
        ax, ay = float(pa[0]), float(pa[1])
        dx, dy = float(pb[0]) - ax, float(pb[1]) - ay
        ex, ey = goal[0] - start[0], goal[1] - start[1]
        denominator = dx * ey - dy * ex
        if abs(denominator) < 1e-12:
            return None
        u = ((start[0] - ax) * ey - (start[1] - ay) * ex) / denominator
        t = ((start[0] - ax) * dy - (start[1] - ay) * dx) / denominator
        if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
            return u
        return None

    def _lane_point(self, key: tuple[int, int], fraction: float) -> tuple[float, float]:
        pa, pb = self.mesh.points[key[0]], self.mesh.points[key[1]]
        return (float(pa[0]) + (float(pb[0]) - float(pa[0])) * fraction,
                float(pa[1]) + (float(pb[1]) - float(pa[1])) * fraction)

    def _commit(self, key: int, start, goal, first_state, final_state,
                result: WeaveResult) -> None:
        """Write the wire into the board: portal records, and a chord per triangle."""
        for pkey, fraction in result.crossings:
            records = self.on_portal.setdefault(pkey, [])
            records.append((fraction, key))
            records.sort()

        stations: list[tuple[int, float | None, float | None]] = []
        # per triangle along the corridor: (tri, entry param, exit param); None = terminal
        for index, tri in enumerate(result.triangles):
            entry = None
            if index > 0:
                pkey, fraction = result.crossings[index - 1]
                entry = self.portal_param(tri, pkey, fraction)
            exit_ = None
            if index < len(result.crossings):
                pkey, fraction = result.crossings[index]
                exit_ = self.portal_param(tri, pkey, fraction)
            stations.append((tri, entry, exit_))

        for index, (tri, entry, exit_) in enumerate(stations):
            pkey_in = result.crossings[index - 1][0] if index > 0 else None
            pkey_out = (result.crossings[index][0]
                        if index < len(result.crossings) else None)
            if entry is None:
                cell = first_state[1] if index == 0 else 0
                entry = self._anchor_param(tri, start[0], start[1], cell)
            if exit_ is None:
                cell = final_state[1]
                exit_ = self._anchor_param(tri, goal[0], goal[1], cell)
            self.chords.setdefault(tri, []).append(Chord(entry, exit_, key))
            self._chord_ends[(tri, key)] = (pkey_in, pkey_out)
            self._cells.pop(tri, None)

    # ------------------------------------------------------------------ rank seating

    def _room(self, pkey: tuple[int, int], span: float) -> float:
        """Width of this doorway not yet claimed by committed wires.

        A wire claims its width plus ONE clearance: neighbours share the gap between
        them. Charging every wire two clearances made three wires need 2.06 mm of a
        doorway where 1.63 is the rule.
        """
        free = span - float(self.mesh.radius[pkey[0]]) - float(self.mesh.radius[pkey[1]])
        for _, wire in self.on_portal.get(pkey, ()):
            free -= 2.0 * self.need_of.get(wire, 0.0) - self.clr_of.get(wire, 0.0)
        return free

    def _fraction_of(self, pkey: tuple[int, int], key: int) -> float:
        for fraction, wire in self.on_portal.get(pkey, ()):
            if wire == key:
                return fraction
        raise KeyError((pkey, key))

    def _reseat(self, pkey: tuple[int, int]) -> None:
        """Seat every wire on a doorway by rank: pitches stacked, slack shared evenly.

        Crossing-freedom is a property of the ORDER of chord endpoints around a
        triangle's boundary, so an order-preserving re-seating cannot introduce a
        crossing -- and it leaves the doorway's free width in one piece, which is what
        makes capacity a count.
        """
        records = self.on_portal.get(pkey)
        if not records:
            return
        pa, pb = self.mesh.points[pkey[0]], self.mesh.points[pkey[1]]
        span = math.hypot(float(pb[0]) - float(pa[0]), float(pb[1]) - float(pa[1]))
        if span <= 0.0:
            return
        ra = float(self.mesh.radius[pkey[0]])
        rb = float(self.mesh.radius[pkey[1]])
        clrs = [self.clr_of.get(wire, 0.0) for _, wire in records]
        widths = [2.0 * (self.need_of.get(wire, 0.0) - clr)
                  for (_, wire), clr in zip(records, clrs)]
        free = span - ra - rb
        used = sum(widths) + sum(clrs) + (clrs[0] if clrs else 0.0)
        gap = max(free - used, 0.0) / (len(records) + 1)
        cursor = ra + (clrs[0] if clrs else 0.0) + gap
        seated = []
        for (_, wire), width, clr in zip(records, widths, clrs):
            centre = cursor + width / 2.0
            seated.append((min(max(centre / span, _EPS * 16), 1.0 - _EPS * 16), wire))
            cursor += width + clr + gap
        self.on_portal[pkey] = seated
        for fraction, wire in seated:
            live = self._results.get(wire)
            if live is not None:
                live.crossings = [(k, fraction if k == pkey else f)
                                  for k, f in live.crossings]
        # chords whose ends sit on this doorway move with it, on both sides
        for tri in self._portal_tris.get(pkey, ()):
            rows = self.chords.get(tri)
            if not rows:
                continue
            fresh = []
            for chord in rows:
                ends = self._chord_ends.get((tri, chord.key))
                if ends is None:
                    fresh.append(chord)
                    continue
                pkey_in, pkey_out = ends
                t1, t2 = chord.t1, chord.t2
                if pkey_in == pkey:
                    t1 = self.portal_param(tri, pkey, self._fraction_of(pkey, chord.key))
                if pkey_out == pkey:
                    t2 = self.portal_param(tri, pkey, self._fraction_of(pkey, chord.key))
                fresh.append(Chord(t1, t2, chord.key))
            self.chords[tri] = fresh
            self._cells.pop(tri, None)

    # ------------------------------------------------------------------ revision

    def snapshot(self, key: int):
        """Everything the board holds for one wire, so it can be put back verbatim."""
        chords = [(tri, chord) for tri, chords in self.chords.items()
                  for chord in chords if chord.key == key]
        records = [(pkey, fraction) for pkey, records in self.on_portal.items()
                   for fraction, wire in records if wire == key]
        ends = [((tri, wire), portals) for (tri, wire), portals in self._chord_ends.items()
                if wire == key]
        return chords, records, ends

    def remove(self, key: int) -> None:
        """Take one wire out of the board; everyone else's cells reopen behind it."""
        for tri in list(self.chords):
            kept = [chord for chord in self.chords[tri] if chord.key != key]
            if len(kept) != len(self.chords[tri]):
                self.chords[tri] = kept
                self._cells.pop(tri, None)
        touched = []
        for pkey in list(self.on_portal):
            before = len(self.on_portal[pkey])
            self.on_portal[pkey] = [(fraction, wire)
                                    for fraction, wire in self.on_portal[pkey]
                                    if wire != key]
            if len(self.on_portal[pkey]) != before:
                touched.append(pkey)
        for tri in list(self.chords):
            self._chord_ends.pop((tri, key), None)
        for pkey in touched:
            self._reseat(pkey)

    def restore(self, key: int, snapshot) -> None:
        chords, records, ends = snapshot
        for tri, chord in chords:
            self.chords.setdefault(tri, []).append(chord)
            self._cells.pop(tri, None)
        for (tri, wire), portals in ends:
            self._chord_ends[(tri, wire)] = portals
        for pkey, fraction in records:
            rows = self.on_portal.setdefault(pkey, [])
            rows.append((fraction, key))
            rows.sort()
        for pkey, _ in records:
            self._reseat(pkey)

    def chain_length(self, start, goal, crossings) -> float:
        points = [start] + [self._lane_point(pkey, fraction)
                            for pkey, fraction in crossings] + [goal]
        return sum(math.dist(a, b) for a, b in zip(points, points[1:]))

    # ------------------------------------------------------------------ the order

    def order(self) -> dict[tuple[int, int], list[int]]:
        """Every doorway's crossing order, planar by construction."""
        return {key: [wire for _, wire in records]
                for key, records in self.on_portal.items()}
