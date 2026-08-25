# Topology-first rebuild — status

**Not merged. `main` still runs the working router.** One connection short on `sonde xilinx`,
and connectivity decides. Everything else is better, most of it by a lot.

| `ecc83-pp`, two layers | connections | copper | DRC | time |
|---|---|---|---|---|
| `main` (relax + rip-up) | 20 / 20 | 239.5 mm | 0 | 16 s |
| this branch | 20 / 20 | 239.6 mm | 0 | **1.6 s** |

| `sonde xilinx`, two layers | connections | copper | DRC | time |
|---|---|---|---|---|
| `main` | **66 / 66** | 718.4 mm | 0 | 324 s |
| this branch | 65 / 66 | **692.5 mm** | 0 | **46 s** |

Both are KiCad's own demo boards. DRC is `kicad-cli pcb drc --format json --severity-all`.
Neither router places vias, so 66/66 on two layers is known to be reachable.

## The embedding is the rubber-band equivalent

The old embedding funnelled each wire through doorways it had been given a fixed slot in. That
produced 2.6× the taut length of the channel it was handed, and on `ecc83-pp` exactly **one** of
twenty connections kept its funnelled path — the other nineteen lost to a solver that ignores
topology, which meant the topological half of the router was doing no work at all.

`taut/rubberband.py` replaces it with the construction from Leiserson & Maley, by way of Dayan's
thesis and gEDA's `toporouter`. One idea carries it:

> **A wire's position in a doorway is never stored. Only its rank is.**

How far a wire stands off a corner is recomputed *at every corner*, as the accumulated clearance
of everything between it and **that** corner. The same wire, in the same bundle, therefore
stands a different distance off at each corner it touches. Fixed slots cannot express that,
which is why a bundle seated that way spread correctly in one gap and wasted half of the next.

The path is found by recursion on the largest violation rather than a forward sweep: draw the
chord, find the obstacle it is most wrong about, wrap it, recurse on both halves. Being on the
*wrong side* of an obstacle is measured on a deliberately different scale from merely passing
too close, so gross errors are corrected first. The tangent geometry is in vectors rather than
the original's slope-intercept form, which removes its degenerate cases; it reproduces the
analytic single-obstacle length to 1e-6 and every bitangent to 1e-15.

Kept taut paths on `ecc83-pp`: **1/20 → 19/20**, at 1.008× the straight-line floor.

## Three defects the correct embedding then exposed

**A terminal could not leave its own pad in the right direction.** A pad centre lies inside its
own pad, so no free triangle contains it, and the lookup resolved it to the free triangle with
the nearest *centroid* — routinely one on the far side. Routes set off the wrong way and came
back around their own pad before starting. On `ecc83-pp` the portal-midpoint chain went from
611.8 mm to 260.1 mm against a 237.5 mm floor — 2.58× to **1.10×** — and the worst single detour
from 16.3 mm to 6.5 mm on a board 25 mm across. It also accounts for all **52** connections
topology could not route on `sonde xilinx`. Topology now converges in **one round** on both
boards, with zero over-capacity doorways and nothing unroutable.

**Layer assignment never asked what a wire would cross.** Every crossing pair on the board
shared not one doorway, so no ordering could separate them — and both wires were dead straight,
four doorways each with nothing to wrap. Two straight lines between different pad pairs simply
cross; the only fix is to put one somewhere else. Layer choice now asks about crossings first
and falls back to a topological cost probe only to break a tie. With the corridor right a taut
wire wraps nothing unless something is in its way, so the straight line between two pads
predicts almost exactly which pairs will fight — and it costs a cross product to ask, against
an A* per layer per connection for the probe it replaces. `sonde xilinx`: 127 s → 34 s.

**The clearance check charged one track's width where two meet.** A halo of `clearance + own
half width` lets two tracks sit both their half widths closer than the rules allow — 0.12 mm on
a 0.25 mm track, most of a clearance. The embedding always had this right; the check did not, so
it passed geometry DRC rejected.

## A correction

An earlier version of this document said every remaining clearance violation was against static
copper, and named constraint edges — copper edges between two corners — as the next piece of
work. That was wrong, and it came from a diagnostic that read `violated_obstacles` as returning
obstacles when it returns indices, so everything looked like copper. Measured properly it is 66
violations against other tracks and 4 against copper, and the 4 are copper *graphics*, not pads.
The constraint-edge machinery was built, measured identical with and without, and removed.

## What is left

**One connection on `sonde xilinx`, and it now fails in geometry, not topology.** The stack
search routes all 66 — with vias when they are worth it — and converges in one round. The drop
happens later, in the check phase: the failing pair needs one of two tracks to take a
deliberately *non-shortest* path, and every mechanism in the check (rip-up three deep,
alternative routes, re-ordering) only ever offers shortest paths. The fix is the one already
named: pull each wire taut against its neighbours' actual geometry rather than checking
afterwards, so "shortest given the others" replaces "shortest, then checked".

## Vias

Layer assignment alone can only two-colour the graph of which nets would cross which — a bet
that the graph is bipartite, which three pairwise-crossing nets settle on any real board. So
the topology now searches the whole stack at once: nodes are *(layer, triangle)*, and a via is
an edge with a price, negotiated exactly like doorway capacity — a site holds one via, and
wanting a taken one costs more each round. Sites are found from the geometry: centroids of free
triangles, clear of copper on every layer by radius plus clearance, through vias only.

Which side of each other two wires pass is decided before the search as a layer *preference*
(from which straight pad-to-pad lines cross which — the thing that fixed the crossings); the
search pays 35% over cost to leave its side, which it will do to reach a via but not to save a
corner. Crossing pressure inside the negotiation was tried and does not converge (5, 5, 4, 4,
6 on ecc83-pp): a route with few doorways has nowhere to be pushed to, so the choice has to be
made once, not bargained over.

Neither demo board buys a via at the standing price — both route to the same quality as the
per-layer search, so the capability costs nothing until a board needs it. It is proven by tests
that force a layer change (start reachable only on the front, goal only on the back) and by
`ecc83-pp` routing identically when vias are free.

## Why this is still the right direction

`main` reaches 100% connectivity, but by relaxation plus rip-up: nothing bounds quality, and it
finds contention by collision. This branch decides contention before any geometry exists,
converges in one round, and produces 3.6% less copper on `sonde xilinx` in a tenth of the time —
with an embedding that is the shortest curve through the corridor it is given, rather than one
that merely passes DRC.
