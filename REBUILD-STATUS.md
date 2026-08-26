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

**13 pairs of wires still cross in the pure sketch**, and every one involves a terminal stub
or a portal-less straight leg — the two things no doorway governs. Those pairs are made legal
today by the check-phase fallback (18 wires), which costs the exact thing the measurement
shows: with the sketch fully legal, placement is order-free, and shortest-first alone reached
**643.6 mm** on sonde xilinx — 76 mm below the current answer — before two stranded
connections forced the retreat to 719.6.

Five attacks on those 13 are in the log with numbers, all reverted: single fan-turn bans
(walk the fan one triangle per round, forever), geometric walls from committed polylines
(over-block: bought vias and 120 mm), telling straight legs their crossed doorways (rank
stacks bend them: +3.3 mm and a short), wrapping a straight leg's terminal (wrap side
ambiguous at a crossing: +63 mm and a sliver), pinned bundle members (pinned positions
violate neighbour gaps and poison the room bounds: taut rate collapsed 49→17).

What the papers say the missing piece is (SURF's region fans; Leiserson–Maley's
left/right/terminal annotations; Zhan's edge-splitting): terminals need an explicit angular
*sector* representation that participates in the search, so a stub is a first-class occupant
of its fan and a separating wire is unrepresentable, not discovered post hoc. That is the
next real piece of work, and it is worth 76 mm and the fallback's retirement.
