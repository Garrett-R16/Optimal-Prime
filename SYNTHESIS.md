# Optimal Prime — Research Synthesis on Automatic Routing

**Scope.** A survey of the algorithmic literature behind automatic routing (PCB, IC, FPGA,
photonic, and the adjacent robotics / computational-geometry / optimization fields that
turn out to matter more than the EDA literature does), assembled to answer one question:

> *If we stop requiring routes to look like something a human would draw, what is the
> actual optimum, and what algorithm finds it?*

116 papers are archived under [`papers/`](papers/), annotated in
[`papers/INDEX.md`](papers/INDEX.md). This document is the synthesis.

---

## 0. Executive summary

Seven findings, in order of how much they should change what we build.

1. **Routing splits into three sub-problems, and only one of them is hard.** Choosing the
   *topology* (which side of each obstacle each net passes on) is the combinatorial core.
   Given a topology, computing the *shortest geometric realization* is polynomial-time and
   **exactly solvable** — the answer is a taut rubber band, which is automatically
   any-angle (straight segments + circular arcs). Leiserson–Maley and Schrijver proved this
   in the 1980s and essentially nobody builds on it. **Search in topology space; let
   geometry relax.**

2. **The wirelength prize for going any-angle is bounded and computable: ~21.5%.** For
   uniformly-oriented two-point connections, the average λ-geometry detour factor is
   `2(1 − cos α)/(α sin α)` with `α = π/λ`. That gives **1.273×** for Manhattan (λ=2),
   **1.103×** for hexagonal/Y (λ=3), **1.055×** for octilinear/X (λ=4), and **1.0** for true
   Euclidean. Manhattan → arbitrary-angle recovers 21.5% of length; Manhattan → octilinear
   already recovers 17.2% of it. This independently reproduces the ~20% claimed by the X
   Architecture and the ~17–18% claimed by hexagonal/Y interconnect work.
   **Corollary: 80% of the any-angle prize is available from just 8 directions.** The
   remaining 4% is not where the value is. (Derivation in Appendix A.)

3. **The unbounded prize is elsewhere.** A badly-chosen topology detours a net by 3–10×,
   not 27%. Via count, layer count, and coupling-driven timing dominate real designs.
   So "optimal" must be defined on a *physical objective functional*, not Σ wirelength —
   and the optimizer must attack net ordering and topology, not corner angles.
   Any-angle geometry is the *enabler* (it makes the topology space continuous and removes
   grid quantization), not the payoff.

4. **Three hardness results bound what "optimal" can mean, and we should state them up
   front:** Euclidean Steiner minimal tree is not known to be in NP (optimal Steiner points
   are algebraic numbers of unbounded degree); the weighted-region shortest path problem is
   *provably unsolvable* in the algebraic computation model over the rationals — only
   (1+ε)-approximable; and shortest *bounded-curvature* path among polygonal obstacles is
   NP-hard (Reif–Wang). Meanwhile edge-disjoint paths is NP-complete even in planar graphs
   — **but becomes polynomial once the homotopy class is fixed** (Schrijver). That
   asymmetry is the entire strategic opening.

5. **The multi-agent path finding (MAPF) community already solved our net-ordering
   problem, optimally.** Conflict-Based Search branches lazily on *conflicts* rather than
   committing to a net order, and there are optimal any-angle and continuous-time variants,
   plus variants for spatially-extended agents (i.e. agents with width — i.e. wires). One
   2025 paper (UGPCB-CBS) already applies CBS on a Delaunay mesh to PCB routing. This is
   the single most transferable idea in the entire review, and it is a strict upgrade over
   rip-up-and-reroute.

6. **Curvilinear routing is already in production — in photonics.** Silicon-photonic
   waveguide routers (LiDAR, LiDAR 2.0, Apollo; 2024–2026) must honour minimum bend radius,
   curvature-dependent loss, and crossing penalties. They are the most mature genuinely
   curvilinear detailed routers that exist. On the manufacturing side, PCB fabs use laser
   direct imaging (curves are free), Gerber X2 has native arcs, alkaline etch has largely
   retired the acid-trap objection, and imec argues curvilinear IC design can *reduce metal
   layer count* at 14A and beyond. **The blocker is EDA data models and DRC engines, not
   fabrication.**

7. **Learning helps as guidance, not as the router.** The strongest results are learned
   *heuristics inside* an exact skeleton — GeniusRoute's generative routing guidance,
   congestion/DRC GNN predictors, learned net-ordering policies, and LLM-driven *algorithm*
   evolution (GR-Evolve, 2026). The end-to-end RL claims have a documented reproducibility
   controversy (AlphaChip and its rebuttals are all in `papers/`). Learn the heuristic; keep
   the guarantee.

---

## 1. Why routers look the way they do

The premise — "current routers only work in a way a human can understand" — is
directionally right, but the binding constraints are not aesthetic. Ranked by how hard they
actually bind:

**1.1 Hanan's theorem is the real reason.** Hanan (1966) proved that for the *rectilinear*
Steiner minimal tree, an optimal solution exists whose Steiner points all lie on the
`O(n²)` grid formed by horizontal and vertical lines through the terminals. The continuum
collapses to a finite set **with no loss of optimality**. Every rectilinear routing
algorithm since is a beneficiary: a grid is not an approximation, it is an exact witness.
No such collapse exists in Euclidean geometry — see GeoSteiner, where exact Euclidean SMT
requires enumerating full topologies and solving algebraic systems. Generalized Hanan grids
do exist for λ-geometries (a *multi-level* Hanan grid suffices for any finite λ), which is
precisely why octilinear routing is tractable and free-form routing is not.
→ `papers/03-steiner-trees/1966-Hanan-on-the-rectilinear-steiner-problem.pdf`

**1.2 The entire geometric stack is axis-aligned.** Corner stitching, tile planes, R-trees,
scanline Boolean mask operations, and DRC-as-edge-comparison all assume rectangles with
integer coordinates. Exact predicates are cheap and robust. Arcs and arbitrary angles force
either floating-point predicates (robustness failures at scale) or exact algebraic
arithmetic (slow). The 1995 DEC WRL `Contour` tile-based gridless router is the classic
demonstration of how much machinery gridless routing costs even when staying rectilinear.
→ `papers/05-detailed-routing/1995-Contour-tile-based-gridless-router-DEC-WRL.pdf`

**1.3 Process rules forbid it at the leading edge.** Self-aligned multi-patterning and EUV
at the tightest metal pitches mandate unidirectional, fixed-pitch metal. There is no
"angle" to choose on M0/M1. Non-Manhattan freedom exists only on the relaxed upper layers,
in MOL/local-interconnect shapes, and on PCBs/packages.

**1.4 DRC vocabulary is Manhattan.** End-of-line spacing, parallel-run-length spacing,
corner-to-corner spacing, min-area, cut spacing — the ISPD-2018/2019 rule set — are all
phrased over rectilinear edges. A curvilinear router needs a curvilinear DRC engine, and
that engine does not exist. This is the largest single piece of missing infrastructure.

**1.5 Manufacturing (PCB) is a largely-solved objection.** Acid traps are acute-angle
etchant pooling; modern alkaline etch and LDI have largely retired the concern, though DFM
checks still flag sub-90° copper. Gerber RS-274X/X2 supports `G02`/`G03` circular
interpolation natively, so a segment-and-arc route is *already* representable in the
standard handoff format. **A tangent-arc rubber-band route is directly manufacturable
today.**

**1.6 Human legibility** is real (debug, rework, review) but is fifth on this list, and is
addressable with rendering and annotation rather than with geometry.

---

## 2. What "optimal" should mean

Wirelength is a proxy that stopped being accurate around 130 nm. The objective a
next-generation router should minimize is a **functional over the whole layout**:

```
J[layout] = Σ_nets  w_t · delay(net)                 # RC / transmission-line
          + Σ_pairs w_x · coupling(net_i, net_j)     # crosstalk-induced delay + noise
          + Σ_nets  w_z · Z_discontinuity(net)       # impedance / return-path integrity
          + w_v · vias   + w_L · layers   + w_A · area
          + w_y · critical_area(layout)              # particle-defect yield
          + w_e · EM_risk(layout)                    # electromigration / thermal
          subject to: connectivity, DRC, and process constraints
```

Three consequences fall straight out:

- **It is not separable per net.** Coupling and congestion are pairwise/field terms. A
  greedy per-net optimum is not a global optimum, which is why net ordering dominates
  outcomes and why CBS-style conflict reasoning (§6.1) beats rip-up-and-reroute.
- **It is naturally a field.** Coupling, congestion, thermal, and critical area are all
  densities over the plane. That argues for a continuous cost field and an
  Eikonal/weighted-region formulation (§5.2, §6.3) rather than an edge-weighted graph.
- **Vias and layers are step functions with huge coefficients.** Any router that optimizes
  length while spending vias is optimizing the wrong thing. This is precisely the X
  Architecture's real argument: its ~30% via reduction mattered more than its ~20% length
  reduction.

---

## 3. The three-layer decomposition (the load-bearing idea)

| Layer | Problem | Complexity | Status in literature |
|---|---|---|---|
| **A. Topology** | Which homotopy class does each net take? Which side of each obstacle/pin? | NP-complete (disjoint paths, even planar) | The real difficulty. Attacked by rip-up/reroute, negotiation, ILP, RL. |
| **B. Embedding** | Given the topology, what is the shortest legal geometry? | **Polynomial, exact** | Solved 1985–1991. Almost never used. |
| **C. Refinement** | Local shape, clearance, curvature, DRC, physics | Continuous optimization | Mature in adjacent fields (adjoint EM, FM2, clothoids). |

### 3.1 Layer B is a solved problem, and it is any-angle for free

Leiserson & Maley (1985) and Maley's thesis / MIT Press book *Single-Layer Wire Routing and
Compaction* (1989) give polynomial-time algorithms that (i) **decide routability of a
sketch** and (ii) **produce the routing minimizing both individual and total wire length**,
using an explicit data structure called the **rubber-band equivalent**. Schrijver's
*Homotopic Routing Methods* and *Paths in Graphs and Curves on Surfaces* give the
graph-theoretic form: **disjoint paths homotopic to given paths can be found in polynomial
time**, and this extends to disjoint homotopic *trees*. Dai, Dayan and Staepelaere's SURF
system (DAC 1991) turned this into a working multi-layer topological router producing
rubber-band sketches; Dayan's 1997 UCSC thesis is the basis for gEDA `toporouter`; Eremex
TopoR and Cadence's Q* engine are the commercial descendants.

The geometric object this produces is exactly what we want: **a taut path in a homotopy
class is a sequence of straight segments and circular arcs tangent to clearance circles
around obstacles.** It is the shortest curve in its class, it is any-angle by construction,
and it maps 1:1 onto Gerber `G01`/`G02`/`G03`.

> **Design consequence.** Do not search over geometry. Search over topology, and let the
> rubber band find the geometry. This converts "any-angle routing" from an open-ended
> continuous optimization into a discrete search **whose every leaf is exactly optimal**.

→ `papers/01-foundations-theory/1990-Schrijver-homotopic-routing-methods.pdf`,
  `papers/01-foundations-theory/1991-Schrijver-paths-in-graphs-and-curves-on-surfaces.pdf`,
  `papers/06-topological-rubber-band/`

### 3.2 Prior art: TopoR has been shipping this architecture since 1996

The topology-first idea was discovered twice, independently. The Western/academic lineage runs
Leiserson–Maley (1985) → Dai/Dayan/Staepelaere's SURF (1991) → Dayan's thesis (1997) → gEDA
`toporouter` (2008). The Russian/industrial lineage is entirely separate: Sergey Luzin and Oleg
Polubasov began work on a flexible topological router in **1988**, shipped the first industrial
version in **1996**, released FreeStyle Router for DOS in 2002, and have sold it as **TopoR** —
now under Eremex — ever since. It is the only commercial autorouter built on this architecture,
and it is the most direct prior art for this project.

**What it actually does**, from the vendor's own internals write-up and the Russian trade
literature:

1. **Delaunay triangulation of the board field.** Vertices sit at pads, barriers, polygons and
   other elements that do not move during routing. Routing happens on the triangulation, not on
   a grid. *This is §6.2 of this document, shipped 30 years ago.*
2. **Initial routing with no layers at all.** Nets are routed sequentially — widest first, and
   among equals shortest first — taking the shortest path across the triangulation subject to
   clearances while minimizing crossings. Every trace lives in a single plane. The topological
   model enforces one invariant: **no more than two conductors may cross at any point.**
3. **Layer assignment afterwards, as a colouring problem.** Only once the planar-with-crossings
   solution exists is each segment assigned a layer such that crossing segments land on different
   layers. For two-layer boards this finds the **exact** via minimum; for multilayer it is NP-hard
   and approximated.
4. **The "metric stage"** converts topology into geometry: arbitrary-angle segments, optionally
   with arcs.
5. **Optimization by single-conductor reroute**, scored by `S = Σ kᵢ·xᵢ` over roughly **one
   hundred** factors — length, vias, rule violations, and subtler events like crossings and
   branch-point connections. The coefficients are not fixed: they are selected per conductor from
   its quality and surroundings, then *oscillated* within ranges to widen the search.
6. **"Layer optimization" as a tunnel move.** A global procedure that radically re-assigns
   segments to layers *without changing the topological paths*. Vias can only decrease, and
   because it jumps between distant points of the search space it shrinks the effective diameter
   of the solution graph and reduces local-minimum trapping.
7. **A Pareto front instead of a single answer.** TopoR maintains multiple variants scored under
   `F = F₁·sin φ + F₂·cos φ`, where `F₁` treats vias as very cheap and `F₂` as very expensive. It
   keeps only the convex hull and reports, for each surviving variant, **the range of via costs
   over which that variant is optimal.** There is no automatic stopping criterion — the user
   decides when to stop.

Point 7 and the framing behind it matter: the documentation states the thesis of §3 explicitly —
the solution space is a graph whose **vertices are topologies** and whose edges are single-net
reroutes, and it is far smaller than a geometric router's space *because each vertex stands for a
large set of concrete geometric realizations.* They also build Steiner branch points by force
equilibrium, inserting a new branch point whenever an angle falls below 120° — the Gilbert–Steiner
rule from §5.3, arrived at independently.

**Reported results:** a 784-contact BGA routed in 7 layers instead of 10; four traces fitted where
a conventional router fits three in the same channel; interconnect area reduced roughly 3× on a
worked example. Treat these as vendor figures — they are illustrative, not benchmarked against a
published suite (§8).

**Two ideas worth taking that are *not* in the architecture proposed in §7:**

- **Deferred layer assignment — planarize first, colour second.** Routing everything in one plane
  with crossings permitted, then colouring the crossing graph into layers, is a cleaner
  decomposition than searching a 3D layered graph directly. It collapses the layer dimension out
  of the topology search entirely, and it makes the 2-layer case exactly solvable. Stage 3 of the
  proposed flow should adopt this.
- **The via-cost Pareto hull.** Carrying a hull of variants parameterized by one contested weight,
  and reporting the weight range over which each is optimal, is better engineering than
  scalarizing to a single objective and hoping the weights were right. It also composes directly
  with multi-objective CBS (§6.1).

**Where TopoR stops — and where this project starts:**

| Gap | Consequence |
|---|---|
| **No optimality guarantee anywhere.** Sequential seeding, single-net local reroute, and oscillating cost coefficients constitute a well-tuned metaheuristic with no bound. | This is exactly the hole CBS fills (§6.1). "Optimal" in the TopoR sense means "best of the variants it happened to find". |
| **~100 hand-weighted proxy factors** instead of a physical objective functional. | No extraction, no field solve, no real coupling model. It cannot optimize §2's `J[layout]` because it cannot evaluate it. |
| **Length *minimizing*, not length *matching*.** Differential pairs and delay-matched buses are the weak spot users report. | Matched-length routing is a constraint pulling against the tool's core objective; this needs first-class support, not a post-pass. |
| **No published parallel/GPU scaling.** Single-machine, interactive-scale. | The GPU results in §5.1 and §5.6 are unexploited here. |
| **No automatic convergence criterion.** | There is no theory saying when to stop, because there is no bound to converge to. |
| **PCB only, closed source, commercial** ($1,990–$9,990 across editions), from a Russian vendor. | Nothing to build on directly; verify procurement and export posture independently before relying on it. Users also report weak auto-placement and support friction. |

> **Net assessment.** TopoR is strong evidence that the §7 architecture works — triangulation
> substrate, topology-then-geometry, arcs, via minimization through layer re-assignment — because
> a company has sold exactly that for three decades. It is *not* evidence that the problem is
> solved. What it lacks is precisely what the last fifteen years of MAPF, GPU routing and
> physical-objective optimization now supply: a guarantee, a real objective, and scale. Build the
> benchmark in §8 first, and make TopoR the baseline to beat.

---

## 4. Complexity map — what is provably out of reach

Stating these plainly keeps the project honest about the word "optimal".

| Problem | Result | Where |
|---|---|---|
| Rectilinear SMT | NP-hard; **but** Hanan grid gives an exact finite witness | Hanan 1966; Garey–Johnson |
| Euclidean SMT | NP-hard; **not known to be in NP** — Steiner points are algebraic numbers of unbounded degree | `03-steiner-trees/2014-history-of-the-euclidean-steiner-tree-problem.pdf` |
| λ-geometry SMT | Multi-level generalized Hanan grid suffices for all finite λ | generalized-Hanan work, §1.1 |
| Weighted-region shortest path | **Unsolvable in the Algebraic Computation Model over ℚ.** Only (1+ε)-approximable | `01-foundations-theory/2013-unsolvability-of-weighted-region-shortest-path-problem.pdf` |
| Bounded-curvature shortest path among obstacles | **NP-hard** (2D, polygonal obstacles); (1+ε) approximation exists; poly-time for "moderate" obstacles | `07-any-angle-continuous/1998-Reif-Wang-complexity-2D-curvature-constrained-shortest-path.pdf` |
| Edge/vertex-disjoint paths | NP-complete, including planar | Robertson–Seymour; Schrijver |
| Disjoint paths **with homotopy given** | **Polynomial** | Schrijver |
| Channel / switchbox routing | NP-complete for nets with ≥3 terminals (knock-knee mode) | Sarrafzadeh; TU-Berlin complexity-gap work |
| Global routing fractional relaxation | FPTAS via min–max resource sharing, + randomized rounding | Raghavan–Thompson 1987; Held–Korte; BonnRoute |

**Reading.** "Actually optimal" is achievable in exactly two senses, and we should claim
only these: (a) **exactly optimal geometry within a chosen topology** (Layer B — and it is
free), and (b) **(1+ε)-optimal, or optimal-with-a-certificate, over topology** within a
bounded search budget (CBS gives this; ECBS/EECBS gives a bounded-suboptimality knob).
Anything stronger is not merely unknown, it is provably unavailable.

---

## 5. Paradigm survey

### 5.1 Grid and graph search — the classical spine

Lee (1961) BFS wavefront → Hadlock (A* with detour numbers) → Soukup (depth-first with line
probes) → Mikami–Tabuchi / Hightower line search (gridless, O(bends) memory). Everything
modern is A* on a sparse grid graph with history costs. The interesting recent work is
**parallelism**: GAMER decomposes multi-source/multi-destination shortest path into
alternating sweeps, dropping a sweep from O(n²) to O(log²n) on GPU (19.9× on coarse-grained
routing inside CUGR); InstantGR maps global routing across thousands of GPU threads;
OrthoRoute (2025, open source) is a KiCad plugin doing GPU wavefront + PathFinder on
17.6k-pad backplanes that FreeRouting could not finish in a month.
→ `papers/02-maze-line-search/`, `papers/04-global-routing/`

**Take:** the grid is the wrong *representation* for us, but the parallel wavefront is the
right *engine*. Its continuous limit is the Eikonal equation (§5.6).

### 5.2 Continuous shortest paths — the geometry we actually want

- **Visibility graph / continuous Dijkstra.** Mitchell–Mount–Papadimitriou's discrete
  geodesic algorithm and the continuous-Dijkstra paradigm are the exact Euclidean analogue
  of Lee. Mitchell's *Handbook of Discrete & Computational Geometry* chapter 31 is the best
  single map of this territory.
- **ANYA (Harabor & Grastien).** The first **optimal** any-angle pathfinder needing no
  preprocessing: it searches over *intervals* of states rather than points, constructing
  them on the fly. This is the correct answer to "A* but any angle" — Theta*, Lazy Theta*
  and Field D* are near-optimal only.
- **Weighted region problem.** Cost varies continuously over the plane; optimal paths obey
  **Snell's law of refraction** at region boundaries. This is *exactly* the right model for
  routing through a congestion / coupling-risk / thermal field. Note the unsolvability
  result in §4: (1+ε) is the ceiling.
- **Bounded curvature.** Reif–Wang NP-hardness; (1+ε)-approximation exists; poly-time for
  moderate obstacles; clothoid/Euler-spiral and Fermat-spiral constructions give **G2**
  (curvature-continuous) transitions, which is what impedance-continuous high-speed traces
  and photonic waveguides actually need.

→ `papers/07-any-angle-continuous/` (23 papers — the densest folder in the corpus)

### 5.3 Steiner trees and their continuous generalization

FLUTE / GeoSteiner / OARSMT for the classical constructions; the more interesting thread is
**branched optimal transport (BOT)** / the **Gilbert–Steiner problem**: cost is
`∫ w(x)^α ds` where `w` is the flow carried. Sub-additive `α` rewards bundling, and the
optimal branching angles are a function of `α` (branch points have degree 3; the Steiner
limit gives 120°). **This is the correct model for any current-carrying net whose width
scales with current — power distribution, wide buses, clock trees.** No EDA router uses it.
Approximate BOT solvers now exist (NeurIPS 2022), plus a "central spanning tree"
formulation (2024) that interpolates between MST and Steiner-like branching under a
robustness parameter.
→ `papers/03-steiner-trees/`

### 5.4 Negotiated congestion — the most important practical algorithm in the field

McMurchie & Ebeling's **PathFinder** (FPGA'95): route everything greedily, allow overuse,
then iterate with a cost carrying a *present* congestion term and a *historical* congestion
term that ratchets up. Signals "negotiate" for resources; the one that needs a resource most
keeps it. It converges, it is anytime, it parallelises trivially over nets, and it defused
the net-ordering problem well enough that every FPGA router and most IC routers still use it
30 years later. OrthoRoute uses it on GPU today.
→ `papers/12-negotiated-congestion-fpga/1995-McMurchie-Ebeling-PathFinder-negotiated-congestion.pdf`

**Take:** PathFinder is our baseline and our anytime fallback. It is *not* optimal — it is a
Lagrangian-flavoured heuristic with no bound. CBS (§6.1) is the upgrade path.

### 5.5 Mathematical programming — where the guarantees live

Concurrent formulations route all nets at once: 0/1 ILP over candidate trees; multicommodity
flow with **randomized rounding** (Raghavan–Thompson 1987 — the origin of provable
approximation in this field); and Held–Korte's **min–max resource sharing** FPTAS, which is
what BonnRoute actually runs (fractional relaxation + randomized rounding + iterative
refinement; reported 2× faster than an industrial router with 5% less netlength and 20%
fewer vias on 32/22 nm IBM chips). The honest caveat, repeated across the surveys:
concurrent methods do not scale to millions of nets, which is why the field retreated to
sequential + negotiation.
→ `papers/04-global-routing/`, `papers/15-surveys-books/2010-Held-Korte-global-routing-VLSI-algorithms-theory-practice.pdf`

### 5.6 Physics-inspired and PDE methods

- **Fast Marching / Eikonal.** The continuous limit of Lee's algorithm: solve
  `|∇T| = 1/F(x)` for arrival time `T` in a speed field `F`, then descend the gradient.
  Gives the exact any-angle geodesic in a continuous anisotropic cost field, is O(N) with
  fast sweeping, and parallelises. **FM2** builds `F` from a clearance/obstacle-potential
  field so the resulting path is simultaneously *short* and *high-clearance* — precisely the
  wire/spacing tradeoff.
- **Voronoi / medial axis.** The generalized Voronoi diagram of free space is the
  maximum-clearance roadmap. Max clearance is directly worth money: lower critical area
  (higher particle-defect yield) and lower coupling.
- **Elastic bands / active contours.** Quinlan & Khatib: a collision-free path deformed by
  internal contraction forces and external obstacle-repulsion forces, in real time. This is
  the *numerical* analogue of the rubber-band sketch, and the right tool for §3 Layer C.
- **Physarum.** Positive-feedback flow reinforcement converging to Steiner-like networks —
  mathematically a mirror-descent/Lagrangian dynamic on a flow problem, and structurally the
  same feedback loop as PathFinder's history cost.
- **Topology optimization and adjoint EM inverse design.** Treat copper as a density field
  ρ(x) ∈ [0,1]; minimize resistance / insertion loss / crosstalk by adjoint gradients (two
  field solves per iteration regardless of parameter count); connectivity-preserving schemes
  using fictitious currents exist to prevent opens and shorts. Curvilinear shapes emerge for
  free. A 2026 *Nature Communications* result (precomputed numerical Green functions) claims
  near-real-time full-wave inverse design, which is what would make this viable beyond a
  handful of critical nets.
- **Wiring optimization in biology.** Chklovskii: brains minimize total wire, and the
  optimum allocates ~3/5 of grey-matter volume to wire. A useful reminder that the right
  macro target is a *volume/density* budget, not a length budget.

→ `papers/09-physics-inspired/`, `papers/13-manufacturing-si-dfm/`

### 5.7 Learning-based methods — what actually replicates

- **Works:** learned *guidance* inside a classical router. GeniusRoute trains a VAE on human
  analog layouts and feeds the resulting routing-region guidance to an A* detailed router
  with hard symmetry constraints — imitating human intent without hard-coding it. GNN/U-Net
  congestion and DRC-hotspot predictors are now standard. Learned net-ordering (transformer
  RL, IJCAI 2025) and offline RL for detailed-routing convergence (2025) target exactly the
  ordering weakness in §5.4. GR-Evolve (2026) uses an LLM to *evolve the routing algorithm*
  per design — a notably different and promising framing.
- **Contested:** end-to-end RL placement. AlphaChip (*Nature* 2021) plus the published
  reassessments and rebuttals are all archived here. Read all of them before betting on
  end-to-end RL.
- **Environments / benchmarks:** XRoute (RL environment on real industrial designs),
  HeuriGym.

→ `papers/10-ml-learning-based/`

### 5.8 The photonics precedent — a working curvilinear detailed router

PIC routing has no choice: waveguides need minimum bend radius, loss grows with curvature
and with each crossing, and ports have fixed orientations. **LiDAR** (ISPD 2025) and
**LiDAR 2.0** (2025) do curvy-aware hierarchical A* with per-net bend radii and spacing,
automatic crossing insertion and port escape — 8,000+ nets in 425 s. **Apollo** (2025)
closes the loop with routing-informed placement. This is the closest existing thing to what
we want to build, in a domain where curvilinear is mandatory rather than optional.
→ `papers/11-analog-photonics-curvilinear/`

---

## 6. The transplants worth making

### 6.1 Conflict-Based Search replaces rip-up-and-reroute

Routing N nets is multi-agent path finding with three twists: there is no time axis (all
wires coexist, so conflicts are purely spatial), agents are *spatially extended* (wires have
width — and MAPF has a literature for that), and agents are trees, not paths.

**CBS** (Sharon et al.) is a two-level algorithm: the low level plans each agent optimally
in isolation; the high level builds a constraint tree, branching only on *actual* conflicts
and adding a constraint to one agent at a time. It returns **provably optimal** solutions
without ever committing to an agent order. Relevant descendants, all archived:

- **CCBS / continuous-time CBS** — removes time discretization. The spatial analogue removes
  *grid* discretization.
- **Optimal and bounded-suboptimal any-angle MAPF** (2024) — CCBS plus an optimal any-angle
  low-level solver (TO-AA-SIPP). **This is literally "optimal multi-net any-angle routing"
  published under a different name.**
- **MAPF with spatially-extended agents** (2020) — agents with geometric footprint.
- **Multi-objective CBS** (2021) — Pareto fronts, which is what §2's functional needs.
- **UGPCB-CBS** (*J. Supercomputing*, 2025) — CBS on a Delaunay mesh applied to PCB routing,
  reported to beat FreeRouting-class tools on speed and outcome. Proof the transplant works.
- **Where Paths Collide** (2025) — comprehensive MAPF survey, classic and learned.

**Take:** replace net ordering with conflict branching; use ECBS/EECBS to trade a bounded
suboptimality factor for scale; keep PathFinder as the anytime seed.
→ `papers/07-any-angle-continuous/`, `papers/15-surveys-books/2025-where-paths-collide-survey-multi-agent-pathfinding.pdf`

### 6.2 Constrained Delaunay triangulation as the routing substrate

A CDT over pads, vias and obstacle vertices gives a **coordinate-free, resolution-free**
representation of free space. A route is a *sequence of triangle edges crossed*, i.e. a
topology — exactly the Layer A object from §3. Properties that matter:

- Space is O(n) in the number of obstacles, independent of any grid pitch.
- Each triangulation edge has an **exact capacity**:
  `floor((len − 2·clearance) / (w + s))`. Congestion becomes an exact count, not an
  estimate on a coarse GCell grid.
- Any-angle is native; there is no preferred direction to choose.
- It is what gEDA `toporouter`, Cadence Q*, TopoR and UGPCB-CBS all use.
- Incremental updates (edge flips) are local, which matters for interactive and ECO routing.

### 6.3 Weighted-region / Eikonal cost fields

Encode congestion, coupling risk, thermal, layer preference and yield as a continuous speed
field `F(x)`; solve the Eikonal equation on GPU; descend. Paths refract at cost boundaries
(Snell), which is the physically correct behaviour and is unreachable on a fixed grid.
Accept (1+ε) — exactness is provably unavailable here.

### 6.4 Branched optimal transport for current-carrying nets

For PDN, wide buses and clock trees, minimize `∫ w^α ds` rather than length. Gives correct
tapering and correct branch angles. Unused in EDA; a genuine differentiator.

### 6.5 Adjoint EM shape optimization for the critical 1%

For the handful of nets where the objective *is* electromagnetic (impedance continuity,
insertion loss, mode purity), run adjoint-based shape optimization on the real solver: two
field solves per gradient regardless of the number of shape parameters.

---

## 7. Proposed architecture for Optimal Prime

A concrete synthesis of the above. Each stage cites the result it rests on.

```
 ┌── 0. OBJECTIVE ──────────────────────────────────────────────────────────┐
 │  Physical functional J[layout] (§2), not Σ wirelength.                    │
 │  Differentiable where possible; Pareto-aware (multi-objective CBS).       │
 └──────────────────────────────────────────────────────────────────────────┘
                                   ↓
 ┌── 1. SUBSTRATE: constrained Delaunay triangulation of free space ────────┐
 │  Coordinate-free topology. Exact per-edge capacity. Any-angle native.     │
 │  [toporouter / TopoR / Q* / UGPCB-CBS]                          §6.2      │
 └──────────────────────────────────────────────────────────────────────────┘
                                   ↓
 ┌── 2. COST FIELD: Eikonal / weighted-region speed field on GPU ───────────┐
 │  Congestion + coupling + thermal + clearance (FM2) → F(x).                │
 │  Paths refract at cost boundaries (Snell). (1+ε) is the provable ceiling. │
 │  [Mitchell WRP; FM2; GAMER-style GPU sweeps]                    §5.6, 6.3 │
 └──────────────────────────────────────────────────────────────────────────┘
                                   ↓
 ┌── 3. TOPOLOGY SEARCH: Conflict-Based Search over homotopy classes ───────┐
 │  Low level: optimal any-angle single-net search on the CDT (ANYA/A*/FMM). │
 │  High level: branch on capacity/spacing conflicts, never on net order.    │
 │  ECBS/EECBS knob for bounded suboptimality at scale.                      │
 │  Seed with PathFinder negotiation for an anytime first solution.          │
 │  [CBS, CCBS, any-angle MAPF, UGPCB-CBS, PathFinder]             §6.1      │
 └──────────────────────────────────────────────────────────────────────────┘
                                   ↓
 ┌── 4. EMBEDDING: rubber-band relaxation — EXACT, POLYNOMIAL ──────────────┐
 │  Taut path in the chosen homotopy class = straight segments + circular    │
 │  arcs tangent to clearance circles. Provably shortest in its class.       │
 │  Also yields a *routability certificate* before embedding.                │
 │  Maps 1:1 to Gerber G01/G02/G03 — manufacturable as-is.                   │
 │  [Leiserson–Maley; Schrijver; Dai/Dayan SURF]                   §3.1      │
 └──────────────────────────────────────────────────────────────────────────┘
                                   ↓
 ┌── 5. CURVATURE FEASIBILITY (only where physics demands it) ──────────────┐
 │  Replace tangent-arc corners with clothoid / Euler-spiral G2 transitions  │
 │  for min-bend-radius, impedance continuity, flex strain, photonics.       │
 │  NB: bounded-curvature shortest path is NP-hard → use (1+ε), or the       │
 │  LiDAR-style bend-radius-feasible expansion.                              │
 │  [Reif–Wang; clothoid G2 fitting; LiDAR 2.0]                    §5.2, 5.8 │
 └──────────────────────────────────────────────────────────────────────────┘
                                   ↓
 ┌── 6. REFINEMENT ─────────────────────────────────────────────────────────┐
 │  Bulk: elastic-band / FM2 spreading — maximize min clearance subject to   │
 │        a length budget (buys yield + crosstalk margin, costs ~nothing).   │
 │  PDN / wide buses / clock: branched-optimal-transport tapering (∫w^α ds). │
 │  Critical 1%: adjoint EM shape optimization on the real solver.           │
 │  [Quinlan–Khatib; BOT/Gilbert–Steiner; adjoint inverse design]  §6.4, 6.5 │
 └──────────────────────────────────────────────────────────────────────────┘
                                   ↓
 ┌── 7. LEARNED GUIDANCE (optimality-preserving only) ──────────────────────┐
 │  Learn A* heuristics, CBS conflict-selection order, cost weights,         │
 │  congestion/DRC predictors, routing-region guidance (GeniusRoute-style).  │
 │  Never let the model *be* the router; keep the exact skeleton.  §5.7      │
 └──────────────────────────────────────────────────────────────────────────┘
```

### Why this is defensible as "actually optimal"

- Stage 4 is **exactly optimal**, unconditionally, given stage 3's output.
- Stage 3 is **optimal, or bounded-suboptimal with an explicit factor** — a real guarantee,
  which no rip-up-and-reroute router provides.
- Stages 2, 5 and 6 are (1+ε) or local, and §4 shows that is provably the best available.
- So the claim is: *optimal geometry, certified-bounded topology, ε-optimal physics.* That
  is stronger than any shipping router claims, and it is honest.

### Highest-risk unknowns

1. **Curvilinear DRC.** No engine exists for arc-and-segment geometry at scale. This is the
   critical path for the whole project and should be prototyped first.
2. **Exact predicates on arcs.** Tangent circles and arc–arc clearance need careful exact or
   interval arithmetic, or the CDT will corrupt at scale.
3. **CBS scaling.** CBS is exponential in the number of conflicts. Real designs have 10⁴–10⁶
   nets. Expect to need EECBS + hierarchical decomposition + PathFinder seeding, and expect
   the optimality claim to be *per-region*, not per-board.
4. **Extraction and timing on curved geometry.** Parasitic extraction is Manhattan-native.
   Without it, the physical objective in §2 cannot be evaluated.

---

## 8. Benchmarks — and a gap worth filling

- **IC global routing:** ISPD 2007/2008 contest suites (BoxRouter, FastRoute, NTHU-Route,
  CUGR, InstantGR all report on these).
- **IC detailed routing:** ISPD 2018 (20 cases, 65/45/32 nm) and ISPD 2019 (advanced rules).
  This is the standard TritonRoute / Dr. CU battleground.
- **FPGA:** VTR/VPR, and the open parallel-routing benchmarks for commercial FPGAs (2024).
- **Photonic:** the LiDAR / Apollo suites.
- **RL environments:** XRoute (industrial designs), HeuriGym.
- **PCB: essentially nothing standard exists.** Papers compare against FreeRouting, ELECTRA,
  DeepPCB and "Optimized 3D-A*" on ad-hoc boards. 3D LineExplore (2026) reports >98% routing
  success against that field, and Cypress (ISPD 2025) did GPU PCB *placement*, but there is
  no agreed benchmark set, no agreed metric and no agreed DRC.
  **Building an open PCB routing benchmark and scorer would be a real contribution and is a
  prerequisite for claiming this router beats anything.**

---

## 9. Open problems this project could actually own

1. **A curvilinear DRC engine** with exact arc predicates. Nothing else unlocks any of this.
2. **Rubber-band embedding at scale, on GPU**, with incremental updates. The 1985–1991
   algorithms were designed for thousands of features, not millions.
3. **CBS for spatially-extended, tree-shaped agents.** MAPF handles extended agents and
   handles paths; nets are *trees* with width. This is an unsolved MAPF variant and it is
   exactly our problem.
4. **Branched optimal transport in EDA.** Nobody has applied `∫ w^α ds` to PDN or bus
   routing. Low-hanging and publishable.
5. **A physically-grounded objective that is differentiable end-to-end**, so stages 2–6 can
   share gradients instead of being a pipeline of proxies.
6. **An open PCB routing benchmark** (§8).
7. **Quantifying the real curvilinear win.** The 21.5% figure (§0.2) is free-space,
   two-terminal, wirelength-only. Nobody has published the congested, multi-terminal,
   via-and-layer-aware number. That measurement is itself a contribution.

---

## 10. Reading order

**Tier 1 — read first (the thesis of this document lives here)**

1. `01-foundations-theory/1990-Schrijver-homotopic-routing-methods.pdf` — why topology-first works
2. `07-any-angle-continuous/2013-Harabor-Grastien-ANYA-optimal-any-angle-pathfinding.pdf` — optimal any-angle search
3. `07-any-angle-continuous/2012-conflict-based-search-optimal-multi-agent-path-finding.pdf` — CBS
4. `07-any-angle-continuous/2024-optimal-bounded-suboptimal-any-angle-multi-agent-pathfinding.pdf` — CBS + any-angle together
5. `12-negotiated-congestion-fpga/1995-McMurchie-Ebeling-PathFinder-negotiated-congestion.pdf` — the baseline to beat
6. `11-analog-photonics-curvilinear/2025-LiDAR2.0-hierarchical-curvy-waveguide-detailed-routing.pdf` — a working curvilinear router

**Tier 2 — the constraints**

7. `07-any-angle-continuous/1998-Reif-Wang-complexity-2D-curvature-constrained-shortest-path.pdf`
8. `01-foundations-theory/2013-unsolvability-of-weighted-region-shortest-path-problem.pdf`
9. `03-steiner-trees/1966-Hanan-on-the-rectilinear-steiner-problem.pdf`
10. `05-detailed-routing/2020-Kahng-TritonRoute-open-source-detailed-router.pdf` — what real DRC costs

**Tier 3 — the state of the art to measure against**

11. `15-surveys-books/2001-Hu-Sapatnekar-survey-multi-net-global-routing.pdf`
12. `15-surveys-books/2010-Held-Korte-global-routing-VLSI-algorithms-theory-practice.pdf`
13. `04-global-routing/2020-CUGR-detailed-routability-driven-3D-global-routing.pdf`
14. `05-detailed-routing/2020-DrCU-detailed-routing-sparse-grid-graph-TCAD.pdf`
15. `04-global-routing/2025-InstantGR-scalable-GPU-global-routing.pdf`

**Tier 4 — the transplants**

16. `03-steiner-trees/2022-branched-optimal-transport-theory-and-solvers-NeurIPS.pdf`
17. `09-physics-inspired/2013-fast-marching-methods-in-path-planning-survey.pdf`
18. `13-manufacturing-si-dfm/2026-near-real-time-full-wave-inverse-design-EM-devices.pdf`
19. `08-nonmanhattan-geometry/2003-Y-architecture-on-chip-interconnect-analysis-methodology.pdf`
20. `10-ml-learning-based/2026-GR-Evolve-LLM-driven-algorithm-evolution-global-routing.pdf`

---

## Appendix A — the λ-geometry detour factor

For λ-geometry (legal directions every `α = π/λ` radians), a straight connection at angle
`θ ∈ [0, α]` from the nearest legal direction is realized by two segments of total length

```
L(θ) = |P| · [ sin(α − θ) + sin θ ] / sin α
```

Averaging over `θ` uniform on `[0, α]`:

```
E[L]/|P| = 2(1 − cos α) / (α · sin α)
```

| geometry | λ | α | detour factor | length vs. Euclidean |
|---|---|---|---|---|
| Manhattan | 2 | π/2 | **1.2732** (= 4/π) | +27.3% |
| Hexagonal / Y | 3 | π/3 | **1.1027** | +10.3% |
| Octilinear / X | 4 | π/4 | **1.0548** | +5.5% |
| λ = 8 | 8 | π/8 | **1.0134** | +1.3% |
| Euclidean | ∞ | → 0 | **1.0000** | — |

Manhattan → Euclidean recovers **21.5%**; Manhattan → octilinear recovers **17.2%**, i.e.
**80% of the total available prize sits in the first 8 directions.** Worst case (rather than
average) is `1/cos(α/2)`: 1.414 for Manhattan, 1.155 for hexagonal, 1.082 for octilinear.

These are free-space, two-terminal, wirelength-only figures. Under congestion, with vias,
layers and multi-terminal nets, they bound only the geometric component — see §9.7.

---

## Appendix B — corpus map

| folder | n | theme |
|---|---|---|
| `01-foundations-theory` | 7 | homotopy, disjoint paths, complexity, randomized rounding |
| `02-maze-line-search` | 2 | Lee-family search and its hardware acceleration |
| `03-steiner-trees` | 9 | Hanan, GeoSteiner, OARSMT, branched optimal transport |
| `04-global-routing` | 8 | BoxRouter → CUGR → GPU (InstantGR, GAP-LA, GANGR), flow methods |
| `05-detailed-routing` | 9 | TritonRoute, Dr. CU, pin access, gridless tiles, Bonn-lineage theses |
| `06-topological-rubber-band` | 5 | rubber-band / topological routing lineage, incl. the TopoR literature |
| `07-any-angle-continuous` | 23 | ANYA, geodesics, weighted regions, bounded curvature, MAPF/CBS |
| `08-nonmanhattan-geometry` | 2 | X and Y architectures, hexagonal interconnect |
| `09-physics-inspired` | 5 | fast marching, Voronoi, Physarum, biological wiring |
| `10-ml-learning-based` | 16 | RL, GNN, offline RL, LLM agents, and the AlphaChip debate |
| `11-analog-photonics-curvilinear` | 9 | LiDAR/Apollo curvy waveguide routing, analog symmetry, inverse design |
| `12-negotiated-congestion-fpga` | 2 | PathFinder and modern parallel FPGA routing |
| `13-manufacturing-si-dfm` | 6 | crosstalk, EM, stretchable curved interconnect, adjoint EM design |
| `14-open-source-tools` | 5 | OpenROAD, 3D LineExplore, GPU PCB placement |
| `15-surveys-books` | 7 | the survey shelf, incl. the MAPF survey and Korte's VLSI optimization |

Full annotated list: [`papers/INDEX.md`](papers/INDEX.md).
