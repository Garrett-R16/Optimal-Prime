# Topology-first rebuild — status

**Not merged. `main` still runs the working router.** This branch now produces legal boards,
but it does not yet beat the one on `main`, and the reason has been narrowed to one component.

## Where it stands

`ecc83-pp`, two layers:

| | connections | copper | DRC | time |
|---|---|---|---|---|
| `main` (relax + rip-up) | 20 / 20 | 239.5 mm | 0 | 16 s |
| this branch | 20 / 20 | 351.4 mm | 0 | 3.5 s |

## The topology is nearly optimal. The embedding is not.

This is the finding that matters, and it came from measuring the two halves separately.

Take the portal sequences this branch chooses and compute the true taut path through them —
the shortest curve that stays inside those doorways. That is **251 mm**, against a
sum-of-straight-lines floor of **237.5 mm**. The chosen homotopy classes are within **6%** of
the best any router could do with them.

Now embed those same sequences with `taut/funnel.py`: **656 mm**. The embedding turns a 251 mm
answer into a 656 mm one, and consequently loses nearly every comparison against the exact
tangent solver — of 20 connections, **1** keeps its funnel path and 19 fall back.

So the part that was hard is working, and the part that was supposed to be a solved problem is
where the defect is.

## What was found and fixed

**A round pad was being modelled as a point.** `pad_obstacle(pad, 0, 0)` on a circular pad
returns a single vertex at its centre with the copper radius carried in `r`, and `build_mesh`
tested freeness with `distance_to_point(...) <= 0`, which ignores `r`. With 28 of 33 cores
being such discs, the mesh triangulated straight through copper: 31 of 88 "free" triangles
contained solid copper, 74 of 120 portals had their span buried in a pad, 65 of 174 gate
endpoints landed inside copper (worst 1.97 mm deep), and 30 of 55 wrap arcs were short by
exactly the pad's own radius. Freeness, portal span, capacity, gate offsets and wrap radii now
all measure from the copper boundary.

**Gate orientation used pad centres at the first and last portal.** A pad centre is inside its
own pad, far enough off the true crossing direction to flip left for right on 4 of 87 gates.
Both ends now come from triangle centroids.

**Funnelled geometry was never checked against the board outline.** Only the fallback solver
was given the boundary, so a route along the rim could leave the board and be pronounced legal.

**A* costed triangles rather than doorways.** The search state is now *(triangle, portal it was
entered by)* and a step is measured between consecutive portal midpoints, which is what an
embedded track actually traverses. Measured on channel length alone: **−12%**.

**A legal funnel path was kept even when it wandered.** Any path that cleared obstacles was
accepted, at up to 1.8× its own span — so a *better* channel could make the board worse by
producing a merely-adequate path where the previous one had been rejected outright. A path
longer than 1.25× its span now competes with the exact solver and the shorter wins.

## What is left

One thing, precisely located: **`taut/funnel.py` produces ~2.6× the taut length of the channel
it is given.** Both halves are correct in isolation — `taut_through` reproduces the analytic
single-obstacle length to 1e-6, and `funnel` returns the near side of an offset doorway — so
the defect is in how the wrap sequence and the tangent construction meet. The likeliest
suspect is `_outer_tangent` returning `None` for overlapping or same-obstacle wrap circles and
falling back to the line of centres, which runs straight through the obstacle.

There is also a hole in the verification: on one intermediate build, two *already settled*
paths overlapped each other, which the pairwise check should have caught.

## Why this is still the right direction

`main` reaches 100% connectivity and zero DRC, but by relaxation plus rip-up: nothing bounds
quality, it takes five minutes on a 66-connection board, and it finds contention by collision.
This branch decides contention before any geometry exists, converges in one to three rounds
with zero over-capacity portals, and picks channels within 6% of optimal. Finish the embedding
and it wins on both counts.
