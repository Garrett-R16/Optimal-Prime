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

## Crossings are settled by cost, with bans as the completeness fallback

The residual sketch crossings are resolved the way the board itself would resolve them: by
price. Each wire of a crossing pair is asked what its cheapest legal path would be with the
other standing — the exact solver against the full current sketch — and whoever moves cheaper,
moves; the mover's geometry is fixed and everyone else seats around it. On sonde xilinx this
dissolves 14 of 15 crossings in two rounds and reaches **708.9 mm**.

The scene that forced this design (`crossing_highway` in `tests/scenes.py`): a straight wire
crossed by a front-only wire. Doctrine says the straight wire stays straight and the other
dives under with two vias (+3.2 mm of barrel); the optimum bends the straight wire around the
other's pad (+2.3 mm). Every mechanism built on "straight wires stay straight" — six pinned-
member variants included — was optimizing the wrong invariant. The only invariants are legal
and minimal.

Cost settling has one blind spot no blame can trace: reshaping the sketch can leave a third
wire homeless, and that survived per-settlement vetoes, inverted movers, rip-up to depth six,
and a via audition — all measured. Meanwhile the ban machinery's +10.7 mm topology reshuffle,
which looked like pure cost, is exactly what houses that wire. So the two run tiered: settle
first; if anyone is stranded, the pipeline runs once more in ban mode, which completes.
**A complete board outranks a shorter one, always.**

| | connections | copper | DRC | path taken |
|---|---|---|---|---|
| ecc83-pp | 20/20 | 239.52 mm (1.008× floor) | 0 | settle |
| sonde xilinx | 66/66 | 719.59 mm | 0 | settle strands one → ban mode |

## The measured frontier, still open

Two shorter answers exist and are real, both blocked by the same wire (`/TD0-PROG-D4`, 6.75 mm
in the densest region): the settle pass at **708.9 mm** (strands it), and shortest-first
placement at **643.6 mm** (strands it plus one more). Every local tool loses to it; the ban
reshuffle is the one configuration measured to house it, at +10.7 mm. Getting the shorter
boards *and* that wire means global search the current architecture approximates but does not
have: SURF's region-fan search space, where terminals are angular sectors and a stub conflict
is unrepresentable rather than repaired. That remains the one structural piece left, and it is
a rebuild of the embedding's search space, not a patch.

## Why this is the right direction

`main` reaches the same completeness in 324 s with nothing bounding quality. This branch
decides classes before geometry, embeds them taut, referees the whole sketch, prices conflict
resolutions, places vias where they pay for their barrel (charged 1.6 mm each to total
length), and guarantees completeness by construction. On an uncongested board it sits 0.8%
off the absolute floor — the optimum the method promises. 83 tests, all through real
`.kicad_pcb` files and `kicad-cli` DRC.
