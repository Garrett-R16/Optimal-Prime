# Topology-first rebuild — status

**The region-fan architecture is built and is now the primary pipeline.** Both demo boards
route completely, with zero DRC, in a single order-free pass — no fallback tiers fired — and
sonde xilinx lands at the shortest complete answer this project has produced.

| board | connections | copper | DRC | time | path |
|---|---|---|---|---|---|
| ecc83-pp (`main`) | 20/20 | 239.5 mm | 0 | 16 s | — |
| ecc83-pp (**this branch**) | 20/20 | 239.54 mm | 0 | 2.8 s | woven, single pass |
| sonde xilinx (`main`) | 66/66 | 718.4 mm | 0 | 324 s | — |
| sonde xilinx (**this branch**) | 66/66 | **713.58 mm** | 0 | 375 s | woven, single pass |

Vias are charged 1.6 mm of barrel each to total length. DRC is `kicad-cli pcb drc
--severity-all`. 95 tests, all end to end through real `.kicad_pcb` files.

## The architecture, final form

1. **Topology** (`taut/layered.py`): one negotiated search over the whole stack — layers,
   vias, capacity — nodes *(layer, triangle)*, via sites priced like doorways.
2. **The weave** (`taut/cells.py`, `taut/weave.py`): wires inserted shortest-first over
   *(triangle, cell)* states. Inside a triangle a committed wire is a chord; chords cut
   cells; a step that would cross a committed wire does not exist in the graph. A terminal's
   stub is a chord from the pad's own corner, so the fan *sectors* of the reference systems
   (SURF's region graph, Leiserson–Maley's terminal annotations) fall out of plain
   arithmetic. Lanes gate on physical width; a blocked wire is promoted to the front and the
   weave restarts; the output is a crossing order on every doorway that is planar by
   construction.
3. **Embedding** (`taut/rubberband.py`): each wire pulled taut in its corridor; offsets from
   rank, recomputed at every corner; the woven order is authoritative.
4. **The referee** (`taut/sketch.py`): the whole sketch checked as one object; residual
   defects (a handful of clips, one stubborn lens) repaired by escalating wrap points and
   the cost arbiter.
5. **Placement**: a formality in the woven world — legal taut geometry is kept without
   competing against the solver, because joint legality outranks individual length; letting
   wires swap to solver paths mid-placement was measured to re-create exactly the conflicts
   the weave removed.
6. **The swap** (post-placement): with everyone placed and legal, substitutions that are
   shorter and legal against the entire standing board only ever shorten. This descent
   measured zero in the pre-weave world (every wire at its best response) and reclaimed
   13.9 mm in the woven world — the corridor discipline stores slack, and the swap spends
   exactly what the whole board can spare.

The settle and ban tiers remain beneath the weave as safety nets; on both demo boards they
no longer fire.

## What the build settled

- Six earlier approximations of the fan structure (turn bans, walls, pinned seats) failed
  from outside; the cell construction gets it exactly, and the crucial test asserts the
  *absence* of intersections in the search space, not any checker's verdict.
- Corridor wiggles do not move embedded length (19 shorter corridors, zero copper change) —
  length lives in the crossing order and the seating.
- Per-layer crossing orders must never be merged: portal keys are vertex pairs in each
  layer's own mesh, and one pair names different doorways on different layers.

## What is left

- The weave still relies on the topology's layer/via plan; dynamic via insertion inside the
  weave (a via edge between cells on different layers) would let a genuinely separated wire
  buy its own way under instead of standing the weave down.
- ecc83-pp keeps a one-wire fallback (its lens); sonde keeps ten. Retiring those through
  the arbiter's residue would make the sketch fully self-sufficient.
- The 643.6 mm shortest-first measurement remains the known frontier below 713.58; the swap
  closes part of the gap, and deeper joint moves over the woven board are now sound tools.
