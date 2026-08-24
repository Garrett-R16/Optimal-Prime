# Topology-first rebuild — status

**Not merged. `main` still runs the working router.** This branch builds the right
architecture and it runs end to end, but its embedding is wrong: it produces 173 DRC
violations where the router on `main` produces none. Merging it would trade a board that
fabricates for one that does not.

## What is built and working

| piece | state |
|---|---|
| `taut/mesh.py` — free space triangulated, portals with capacity | **working** |
| `taut/topo.py` — portal sequences chosen jointly, negotiating capacity | **working** |
| `taut/funnel.py` — funnel + tangent embedding | works in isolation |
| `taut/plan.py` — the three wired together | runs, output not legal |

On `ecc83-pp`, two layers:

```
88 free triangles, 120 portals per layer
topology converges in 1 round, 0 portals over capacity
20/20 connections routed
0.1 s          (against 16 s for the relaxation router on main)
```

The topological half does exactly what it was built to do. Homotopy classes are chosen
explicitly for every net at once, against a countable resource, and the negotiation settles
immediately. That was the thing four earlier mechanisms were unable to do, and it is a
hundred times faster than the relaxation it replaces.

Both embedding primitives are verified on their own. `taut_through` reproduces the analytic
single-obstacle taut length — `2·√(d²−r²) + r·(π−2·acos(r/d))` — to 1e-6, for either wrap
direction. `funnel` returns the near side of an offset doorway and no wraps at all for one
the path passes straight through.

## What is broken

Composed together on a real board: **173 DRC violations, 732 mm of copper against 239.5 mm
from the router on `main`.**

Two bugs were found and fixed on the way, which is why the number came down from 180:

- Unconstrained Delaunay lets a portal span pass straight *through* a third obstacle. Such a
  span is not a doorway; those now carry zero capacity.
- Arc sweep direction was asserted from which side of the doorway the band turned at, and was
  simply wrong — every arc took the long way round its circle, 6.2 radians where 0.3 was
  wanted. Sweep now follows the tangents.

What remains, in the order I would attack it:

1. **Gate orientation is approximate.** `orient()` decides left from right using the direction
   between two triangle centroids. That is a proxy for the direction the band actually crosses
   the doorway, and where it is wrong the funnel wraps the far side and the path leaves its
   channel entirely — which is enough on its own to explain copper crossing copper.
2. **The wrap radius may not belong to the vertex being wrapped.** A gate end is a point
   *along* the portal span, offset from an obstacle corner. The embedding then rounds that
   corner at that offset, which is right when the band really does turn there and wrong when
   the gate end came from a distant vertex or the board outline.
3. **Nothing checks the output.** The router on `main` verifies its finished arrangement
   against real geometry and repairs what is too close. This one emits whatever the funnel
   produced. Even with (1) and (2) fixed, that check has to exist.
4. **Channel quality, separately from correctness.** A* costs a portal by the distance between
   triangle centroids, which is a poor proxy for path length, so the channels wander. That is
   most of the 3× length and none of the violations.

(1) and (2) are geometry with sign conventions, which is exactly the kind of work that goes
wrong when hurried. (3) is straightforward. (4) is a better cost function, or re-costing by
funnel length.

## Why this is still the right direction

The architecture is not in doubt; the arithmetic is. `main` reaches 100% connectivity and
zero DRC by relaxation plus rip-up, but nothing in it bounds quality, it takes five minutes
on a 66-connection board, and it discovers contention by collision. This branch decides
contention before any geometry exists, converges in one round, and finishes in a tenth of a
second.

The right next step is to finish the embedding, not to abandon it.
