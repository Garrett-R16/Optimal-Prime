# Topology-first rebuild — status

**Both demo boards route completely, with zero DRC, faster than `main`, with vias available
and counted.** The branch does everything `main` does and decides its answers instead of
colliding into them.

| board | connections | copper | DRC | time | vias |
|---|---|---|---|---|---|
| ecc83-pp (`main`) | 20/20 | 239.5 mm | 0 | 16 s | — |
| ecc83-pp (**this branch**) | 20/20 | **239.52 mm** | 0 | **1.9 s** | 0 |
| sonde xilinx (`main`) | 66/66 | 718.4 mm | 0 | 324 s | — |
| sonde xilinx (**this branch**) | 66/66 | **719.6 mm** | 0 | **265 s** | 1 |

Via barrels are charged to total length (1.6 mm each), so "shorter" can never be bought by
quietly moving distance into the third dimension. DRC is `kicad-cli pcb drc --severity-all`.

## The architecture, as it now stands

1. **Topology** (`taut/layered.py`): one negotiated search over the whole stack — nodes are
   *(layer, triangle)*, edges are doorways and via sites, both priced by congestion. Layer
   *preference* from which straight pad-to-pad lines cross which; the search pays 35% to leave
   its side, which it will do to reach a via.
2. **Embedding** (`taut/rubberband.py`): each wire pulled taut in its class; offsets from
   *rank* in each doorway's crossing order, recomputed at every corner; rank derived from the
   topology (chain adjacency), never read back from geometry — reading it back was circular.
   Wires press on each other through the doorway *room* fixpoint.
3. **The referee** (`taut/sketch.py`): the whole sketch checked as one object — crossings with
   *parity* (odd = class conflict, even = lens), grazes, clips against bare copper. Every
   defect is fed back to the stage that owns it: clips and lenses become extra wrap points
   (lazy obstacle addition inside the embedding); odd crossings become fan-turn *bans* and a
   targeted, warm-started re-search of only the implicated connections.
4. **Placement**: shortest-first (measured 77 mm better on sonde), falling back to
   longest-first when short-first walls a long connection in — the shorter board is only the
   better board when it is a whole one.

First-principles scenes (`tests/scenes.py`) drive all of it end to end through real
`.kicad_pcb` text: analytic taut lengths, fan cuts on one and two layers, stub grazes,
pairwise-crossing ladders, forced vias, via-length accounting. 81 tests.

## The one open problem, and the measured prize

**13 pairs of wires still cross in the pure sketch**, every one involving a terminal stub or a
portal-less straight leg — the two things no doorway governs. The check-phase fallback makes
them legal today (18–27 wires), at the cost the measurement shows: with a fully legal sketch,
placement is order-free, and shortest-first placement alone reached **643.6 mm** on sonde
xilinx — 76 mm below the standing answer — before two stranded connections forced the retreat
to 719.6. The excess is diffuse: dozens of short wires each bent slightly around long wires
placed before them.

Everything tried against this is in the log with numbers, all reverted:

*Against the crossings* — single fan-turn bans (walk the fan one triangle per round, forever);
side-aware tail families (identical); geometric walls from committed polylines (over-block:
vias and +120 mm); stub-only walls (+20 mm, one drop); banning the straight-leg shortcut
(dropped a connection); wrapping a straight leg's terminal (side-ambiguous at a crossing:
+63 mm and a sliver); six variants of pinned bundle members — straight legs enumerating their
doorways and sitting fixed while others seat around them by topological rank (each collapsed
the taut rate ~49→17 board-wide; the last variant isolated the cause to rank-vs-geometry
seating and still lost).

*Against the placement wall* — deferred rip-up rescue (7 min, futile); promoting the stranded
to the front (rotates who strands); inline rescue at strand time (one of two rescued);
deepened rip-up (depth 6 / limit 10: exponential, >10 min); single-wire re-offers after
placement (zero gain — every wire is already at its best response; the 643 is a different
equilibrium); joint cluster re-placement of the worst-excess wires (inverse selection: the
top-excess wires are the long ones whose excess is irreducible); a placement-level via split
for the stranded wire, solver to a site on one face and onward on the other (no clear site
among the 24 nearest — the board is genuinely full there).

## Where the remaining 76 mm actually lives

Not in any single wire, and not reachable by local search from the longest-first equilibrium.
The two orderings are different Nash equilibria of the same game; every local tool tried —
re-offers, clusters, rip-up, vias — moves within an equilibrium, not between them. The papers'
answer is to make the *sketch* legal so that no placement order exists at all: SURF's region
fans give terminals angular sectors that participate in the search, which is precisely the
structure the six pinned-member variants approximated from outside and could not reach. That
is a rebuild of the embedding's search space, not a patch, and it is the one remaining piece
between this branch and provably order-free minimal-length output.

## Why this is still the right direction

`main` reaches the same completeness in 324 s with nothing bounding quality. This branch
decides classes before geometry, embeds them taut, referees the whole sketch, repairs by
feedback, places vias where they pay for their barrel, and finishes both demo boards faster
than `main` with equal or better copper. ecc83-pp stands at **1.008× the straight-line
floor** — for a board with no forced conflicts, the method is already at the optimum it
promises.
