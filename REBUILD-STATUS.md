# Topology-first rebuild — status

**Not merged. `main` still runs the working router.** This branch is now faster, shorter and
cleaner than `main` on both boards, but it is one connection short on `sonde xilinx`, and
connectivity is the metric that decides.

| `ecc83-pp`, two layers | connections | copper | DRC | time |
|---|---|---|---|---|
| `main` (relax + rip-up) | 20 / 20 | 239.5 mm | 0 | 16 s |
| this branch | 20 / 20 | **239.6 mm** | 0 | **2.0 s** |

| `sonde xilinx`, two layers | connections | copper | DRC | time |
|---|---|---|---|---|
| `main` | **66 / 66** | 718.4 mm | 0 | 324 s |
| this branch | 65 / 66 | **697.3 mm** | 0 | **129 s** |

## The embedding is now the rubber-band equivalent

The old embedding funnelled each wire through doorways it had been assigned a fixed slot in.
That produced 2.6× the taut length of the channel it was given, and of 20 connections on
`ecc83-pp` exactly **one** kept its funnelled path — the other 19 lost to a solver that ignores
topology entirely, which meant the topological half of the router was doing no work.

`taut/rubberband.py` replaces it with the construction from Leiserson & Maley, by way of
Dayan's thesis and gEDA's `toporouter`. One idea carries it:

> **A wire's position in a doorway is never stored. Only its rank is.**

How far a wire must stand off a corner is recomputed *at every corner*, as the accumulated
clearance of whatever lies between it and **that** corner:

```
r = spacing(me, next) + spacing(next, next-next) + ... + spacing(last, the corner)
```

The same wire, in the same bundle, therefore stands a different distance off at each corner it
touches, because the set of wires between it and each corner is different. Fixed slots cannot
express that, which is exactly why a bundle seated that way spread correctly in one gap and
wasted half of the next.

The path is found by recursion on the largest violation rather than by sweeping forward: draw
the chord, find the obstacle it is most wrong about, wrap it, recurse on both halves. Being on
the *wrong side* of an obstacle is measured on a deliberately different scale from merely
passing too close, so gross errors are always corrected first.

The tangent geometry is written in vectors rather than the original's slope-intercept form,
which removes its degenerate cases. It reproduces the analytic single-obstacle length to 1e-6
and every bitangent to 1e-15.

Result on `ecc83-pp`: kept taut paths went **1/20 → 15/20**, and the embedded length is now
**239.5 mm against a straight-line floor of 237.5 mm — 1.008×**.

## What the correct embedding then exposed

With the embedding no longer the bottleneck, the measurement moved, and it overturned what
this document previously claimed. The topology was **not** within 6% of optimal. It was 2.58×.

**A terminal could not leave its own pad in the right direction.** A pad centre lies inside its
own pad, so no free triangle contains it, and `triangle_at` was resolving it to the free
triangle with the nearest *centroid* — routinely one on the far side of the pad. Routes set off
in the wrong direction and came back around their own pad before they could start.

Measured as portal-midpoint chain against the straight-line floor on `ecc83-pp`:

| | midpoint chain | vs floor | worst single detour |
|---|---|---|---|
| before | 611.8 mm | 2.58× | 16.3 mm |
| after | 260.1 mm | **1.10×** | 6.5 mm |

on a board 25 mm across. It also accounts for **all 52** connections topology could not route
on `sonde xilinx`, and for that board's 3 clearance violations. Topology now converges in **one
round** on both boards with zero over-capacity doorways and nothing unroutable.

This was worth the detour: the defect was invisible while the embedding was wrong, because
every route was being thrown away and re-solved anyway.

## Rip-up

A connection that ends up with no legal geometry is not evidence that the board is full —
topology gave every connection a corridor. It is evidence that a track laid earlier took more
room than its own corridor. The check phase now asks a stranded connection where it would go on
an empty board, takes up only the tracks sitting on that answer, places it, and puts them back
elsewhere; a displaced track may displace one itself, three deep. If any of them cannot be put
back, the whole exchange is undone rather than trading one unrouted connection for another.

That took `sonde xilinx` from 63/66 to 65/66.

## What is left

**One connection on `sonde xilinx`.** `main` places it, using five minutes of relaxation and
rip-up. Until the branch matches 66/66 it should not be merged, however much better the rest of
the numbers are.

**25 of 66 taut paths on `sonde xilinx` clip copper** and fall back to the exact solver. Every
one of the 69 violations is against **static copper — not one is against another track**, which
says the rank-and-spacing machinery is doing its job and locates the remaining defect exactly:

> The embedding keeps a wire clear of the *corners* of a doorway, and nothing keeps it clear of
> the copper *edge* running between two corners.

A wire crossing near the middle of a rectangular pad's edge satisfies its clearance from both
that pad's corners and still cuts the flat side between them. Round pads are safe by accident,
because their facets circumscribe the true circle; rectangular ones are not.

This is the case `toporouter` calls a *constraint edge* — a triangulation edge that is itself
copper — where the offset becomes a flat `min(spacing, edge length / 2)` about the whole edge
rather than a stack about a corner. `Crossing.constraint` exists for it and is never set,
because the mesh does not yet mark which doorways lie along copper. Setting it, and charging
the walls of each triangle the path crosses as candidates alongside the doorways, is the next
piece of work — and it is the same lever that would let the fallback, the last unrouted
connection, and most of the remaining 129 s go away together.

## Why this is still the right direction

`main` reaches 100% connectivity, but by relaxation plus rip-up: nothing bounds quality, and it
finds contention by collision. This branch decides contention before any geometry exists,
converges in one round, and produces 3% less copper on `sonde xilinx` in 40% of the time — with
an embedding that is provably the shortest curve through the corridor it is given, rather than
one that merely passes DRC.
