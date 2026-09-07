"""The weave places its own escape vias.

The stack deals vias blind: it cannot see the walls the weave has committed, so when a
piece is walled on its layer the stack keeps offering routes through the same walls,
and every feedback vocabulary tried -- site bans, turn bans, layer flips, gifted sites,
cheaper vias -- measured no better than standing down. The wire that knows where the
walls are is the wire that must choose where to dive.

So a walled connection is re-searched HERE, on the live weave of both layers: leave the
pad on its own layer to a via beside it, cross on the other layer, and come back up
beside the far pad. Candidate via sites are the tangent pocket sites of the two pads
(the escape a hand router reaches for) plus any stock site within reach; every trial is
a real insertion into the live weave, undone afterwards, so a route this returns is one
the weave has already proven it can hold. The stack is handed the result as a route of
its own kind -- legs and site indices -- and the next weave round places it like any
other.
"""

from __future__ import annotations

import math

from .layered import Leg, Route
from .vias import Site

__all__ = ["escape_route"]

#: how far from a pad a stock site may sit and still count as that pad's escape
_NEAR_NM = 4_000_000.0
#: candidate pairs tried before giving up on one connection
_TRIALS = 40


def _near_sites(sites: list[Site], pad, exclude: set[int]) -> list[Site]:
    out = [site for site in sites
           if site.index not in exclude
           and math.hypot(site.x - pad.x, site.y - pad.y) <= _NEAR_NM]
    out.sort(key=lambda site: math.hypot(site.x - pad.x, site.y - pad.y))
    return out


def _try_leg(weave, key: int, start, goal, start_tris, goal_tris, need, clearance,
             admit, net):
    got = weave.insert(key, start, goal, start_tris, goal_tris, need=need,
                       admit=admit, clearance=clearance, net=net)
    return got if got.found else None


def _leg_of(layer_index: int, start, goal, got) -> Leg:
    return Leg(layer=layer_index, start=start, goal=goal,
               triangles=[int(t) for t in got.triangles],
               portals=[key for key, _ in got.crossings])


def escape_route(weaves: dict, usable, sites: list[Site], parent, need: float,
                 clearance: float, admit: float, taken: set[int],
                 gifts: list[Site] | None = None) -> Route | None:
    """A two-via (or one-via) route for ``parent`` that the live weave can hold, or None.

    ``taken`` are site indices already claimed by other routes. ``gifts`` are tangent
    sites conjured for this connection's pads (already appended to ``sites``).
    """
    a, b = parent.pad_a, parent.pad_b
    start = (float(a.x), float(a.y))
    goal = (float(b.x), float(b.y))
    net = parent.net.code
    layers_a = [i for i, name in enumerate(usable) if a.on_layer(name)]
    layers_b = [i for i, name in enumerate(usable) if b.on_layer(name)]
    if not layers_a or not layers_b or len(usable) < 2:
        return None
    exclude = set(taken)
    pool_a = _near_sites(sites, a, exclude)
    pool_b = _near_sites(sites, b, exclude)
    if gifts:
        mine = {g.index for g in gifts}
        pool_a = [s for s in pool_a if s.index in mine] + [s for s in pool_a
                                                            if s.index not in mine]
        pool_b = [s for s in pool_b if s.index in mine] + [s for s in pool_b
                                                            if s.index not in mine]

    provisional = -1_000_000  # keys for trial insertions, never real pieces
    trials = 0

    def undo(*legs):
        for weave_, key_ in legs:
            weave_.remove(key_)

    # Two vias: pad A on its layer to s, s to t on the other layer, t to pad B.
    for s in pool_a[:8]:
        for t in pool_b[:8]:
            if s.index == t.index:
                continue
            for la in layers_a:
                lb_options = [i for i in range(len(usable)) if i != la]
                for lb in lb_options:
                    for lc in layers_b:
                        if lc == lb:
                            continue  # then a single via would do; handled below
                        trials += 1
                        if trials > _TRIALS:
                            return None
                        wa, wb, wc = weaves[usable[la]], weaves[usable[lb]], weaves[usable[lc]]
                        ka, kb, kc = provisional, provisional - 1, provisional - 2
                        sa = (s.x, s.y)
                        tb = (t.x, t.y)
                        got_a = _try_leg(wa, ka, start, sa, wa.mesh.terminals(*start),
                                         [s.triangle_on(la)], need, clearance, admit, net)
                        if got_a is None:
                            continue
                        got_b = _try_leg(wb, kb, sa, tb, [s.triangle_on(lb)],
                                         [t.triangle_on(lb)], need, clearance, admit, net)
                        if got_b is None:
                            undo((wa, ka))
                            continue
                        got_c = _try_leg(wc, kc, tb, goal, [t.triangle_on(lc)],
                                         wc.mesh.terminals(*goal), need, clearance, admit, net)
                        if got_c is None:
                            undo((wa, ka), (wb, kb))
                            continue
                        route = Route(key=parent.key, net=net, start=start, goal=goal,
                                      legs=[_leg_of(la, start, sa, got_a),
                                            _leg_of(lb, sa, tb, got_b),
                                            _leg_of(lc, tb, goal, got_c)],
                                      vias=[s.index, t.index])
                        undo((wa, ka), (wb, kb), (wc, kc))
                        return route

    # One via: pad A to s on A's layer, s to pad B on a layer B is on.
    for s in pool_a[:12] + pool_b[:12]:
        for la in layers_a:
            for lb in layers_b:
                if la == lb:
                    continue
                trials += 1
                if trials > _TRIALS * 2:
                    return None
                wa, wb = weaves[usable[la]], weaves[usable[lb]]
                ka, kb = provisional, provisional - 1
                sa = (s.x, s.y)
                got_a = _try_leg(wa, ka, start, sa, wa.mesh.terminals(*start),
                                 [s.triangle_on(la)], need, clearance, admit, net)
                if got_a is None:
                    continue
                got_b = _try_leg(wb, kb, sa, goal, [s.triangle_on(lb)],
                                 wb.mesh.terminals(*goal), need, clearance, admit, net)
                if got_b is None:
                    undo((wa, ka))
                    continue
                route = Route(key=parent.key, net=net, start=start, goal=goal,
                              legs=[_leg_of(la, start, sa, got_a),
                                    _leg_of(lb, sa, goal, got_b)],
                              vias=[s.index])
                undo((wa, ka), (wb, kb))
                return route
    return None
