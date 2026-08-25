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
| this branch | 65 / 66 | **692.3 mm** | 0 | **32 s** |

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

**One connection on `sonde xilinx`**, and it is understood rather than mysterious. `/TD0-PROG-D4`
fits as soon as two tracks are lifted; one of those two then cannot come back, because its only
blocker is the connection just placed. Rip-up was extended to cascade three deep, to try each of
the placed connection's own alternative routes, and to re-run the whole board with the failed
connection going down first. None of it helps: those two connections conflict irreducibly given
that each takes a *shortest* path.

That is the structural gap. `main` gets this connection with five minutes of relaxation, which
can push a settled track aside by a fraction of a millimetre. This branch has no such move —
every connection is either its taut path or the solver's shortest, with nothing in between. The
fix is to make the embedding itself aware of the tracks already down, so a wire can be pulled
taut *against* its neighbours rather than checked against them afterwards. That is also what
would let the 18 remaining fallbacks go away, and with them most of the 32 s.

## Why this is still the right direction

`main` reaches 100% connectivity, but by relaxation plus rip-up: nothing bounds quality, and it
finds contention by collision. This branch decides contention before any geometry exists,
converges in one round, and produces 3.6% less copper on `sonde xilinx` in a tenth of the time —
with an embedding that is the shortest curve through the corridor it is given, rather than one
that merely passes DRC.
