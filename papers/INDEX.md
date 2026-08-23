# Annotated Bibliography — 116 papers

Every PDF in this tree, with a one-line "what it gives us". Read
[`../SYNTHESIS.md`](../SYNTHESIS.md) first — it explains why these are grouped this way.

Section 16 at the bottom lists **key works that are paywalled and therefore not archived
here**, with full citations, because several of them are foundational.

---

## 01 — Foundations & theory
*Homotopy, disjoint paths, complexity, provable approximation.*

| File | Why it matters |
|---|---|
| `1987-Raghavan-Thompson-randomized-rounding.pdf` | The origin of provable approximation in routing: solve the LP relaxation, then round each net's fractional path probabilistically. Chernoff bounds give the congestion guarantee. Everything from BonnRoute back to the ILP formulations descends from this. |
| `1990-Schrijver-homotopic-routing-methods.pdf` | **Tier 1.** Disjoint paths in a planar graph *homotopic to given paths* are findable in polynomial time. This is the theorem that makes topology-first routing viable: pick the homotopy class combinatorially, and the embedding is then easy. |
| `1991-Schrijver-paths-in-graphs-and-curves-on-surfaces.pdf` | The surface-topology generalization — multi-layer routing is routing on a surface with handles (vias). Useful when we extend §3 beyond one layer. |
| `2013-unsolvability-of-weighted-region-shortest-path-problem.pdf` | **Hard limit.** The weighted-region shortest path is *not solvable* in the algebraic computation model over the rationals. Optimal continuous routing through a cost field can only ever be (1+ε)-approximated. Cite this whenever someone says "exactly optimal". |
| `2021-exponential-time-parameterized-planar-disjoint-paths.pdf` | Modern parameterized complexity of planar disjoint paths — bounds on what an exact topology search can hope to achieve as a function of net count. |
| `2022-routing-problems-VLSI-new-mathematical-models.pdf` | Recent survey of mathematical formulations of the VLSI routing problem; useful as a formulation checklist. |
| `2025-uncrossed-multiflows-applications-disjoint-paths.pdf` | Latest theory connecting multicommodity flow relaxations to genuinely disjoint (uncrossed) paths — the gap between the LP we can solve and the integral routing we need. |

## 02 — Maze & line-search routing
*The Lee family and its hardware acceleration.*

| File | Why it matters |
|---|---|
| `1994-simulated-annealing-maze-routing.pdf` | Early attempt at escaping greedy maze routing via stochastic search; historically interesting, and a reminder of what plain metaheuristics buy (not much). |
| `2022-FPGA-accelerated-maze-routing-kernel-VLSI.pdf` | Hardware acceleration of the maze-routing kernel. Read alongside GAMER/InstantGR (§04) for the parallel-wavefront engine we want in the Eikonal stage. |

## 03 — Steiner trees & continuous generalizations
*Where multi-terminal optimality lives — and where it breaks.*

| File | Why it matters |
|---|---|
| `1966-Hanan-on-the-rectilinear-steiner-problem.pdf` | **Tier 2.** The Hanan grid theorem. The single deepest reason routing is rectilinear: the continuum collapses to an O(n²) grid *with no loss of optimality*. There is no Euclidean analogue. |
| `Robins-Zelikovsky-minimum-steiner-tree-construction-chapter.pdf` | Best single survey of Steiner tree construction: approximation ratios, the Steiner ratio, iterated 1-Steiner, and the graph/geometric split. |
| `2014-history-of-the-euclidean-steiner-tree-problem.pdf` | Why Euclidean SMT is not known to be in NP — Steiner points are algebraic numbers of unbounded degree. The formal reason "free-form optimal" is harder than it looks. |
| `2014-GeoSteiner-software-package-computational-study.pdf` | The state of the art in *exact* geometric Steiner trees (thousands of terminals). Full-Steiner-tree generation + concatenation. Our reference for "exact, if you can afford it". |
| `2016-approximate-euclidean-steiner-trees.pdf` | Practical (1+ε) Euclidean Steiner constructions — the tractable middle ground. |
| `2024-hierarchical-heuristic-clustered-steiner-trees-with-obstacles.pdf` | Obstacle-aware clustered Steiner trees — closest published thing to obstacle-avoiding multi-terminal routing in continuous geometry. |
| `2022-branched-optimal-transport-theory-and-solvers-NeurIPS.pdf` | **Tier 4, high value.** Branched optimal transport: cost `∫ w^α ds` where `w` is carried flow. Sub-additivity rewards bundling and *predicts branching angles*. The correct model for PDN, wide buses and clock trees. Unused in EDA. |
| `2023-gilbert-steiner-branching-points-degree-3.pdf` | Proves planar Gilbert–Steiner branch points have degree 3 — the structural theorem behind the above. |
| `2024-central-spanning-tree-problem.pdf` | Interpolates between MST and Steiner-like branching under a robustness parameter; a tunable family for "how much should we bundle?". |

## 04 — Global routing
*Capacity, congestion, flow, and the GPU era.*

| File | Why it matters |
|---|---|
| `2001-Albrecht-global-routing-approximation-multicommodity-flow.pdf` | The min–max resource-sharing FPTAS applied to global routing; the algorithm underneath BonnRoute. Where the real guarantees are. |
| `2005-multicommodity-flow-algorithms-buffered-global-routing.pdf` | Extends flow-based global routing to include buffer insertion — i.e. routing where the objective is delay, not length. |
| `2007-BoxRouter-2.0-hybrid-robust-global-router-layer-assignment.pdf` | Progressive-ILP box expansion plus negotiation; the classic hybrid, and the reference point for the ISPD-2008 era. |
| `2020-CUGR-detailed-routability-driven-3D-global-routing.pdf` | **Tier 3.** Modern academic SOTA global router; 3D, detailed-routability-driven, probabilistic resource estimation. The thing to benchmark against. |
| `2024-VGR-3D-global-routing-via-minimization.pdf` | Explicit via minimization in 3D global routing — vias, not length, are the expensive resource. |
| `2025-InstantGR-scalable-GPU-global-routing.pdf` | **Tier 3.** GPU data partitioning and memory management for global routing across thousands of threads. The scaling template. |
| `2025-GAP-LA-GPU-accelerated-performance-driven-layer-assignment.pdf` | GPU layer assignment driven by timing rather than via count. |
| `2025-GANGR-GAN-assisted-global-routing-parallelization.pdf` | GAN-assisted partitioning for parallel global routing; an example of learning used for *decomposition*, not for the route itself. |

## 05 — Detailed routing
*Where design rules actually bite.*

| File | Why it matters |
|---|---|
| `1995-Contour-tile-based-gridless-router-DEC-WRL.pdf` | **Read for the cost of gridless.** Corner-stitched tile planes, gridless expansion, arbitrary widths/spacings. Shows exactly what machinery non-grid routing demands — before you even add arcs. |
| `2018-ISPD-initial-detailed-routing-contest-slides.pdf` | The benchmark and rule set that defines the modern detailed-routing problem: spacing tables, cut spacing, EOL spacing, min-area. |
| `2020-Kahng-TritonRoute-open-source-detailed-router.pdf` | **Tier 2.** The open-source detailed router (OpenROAD). Pin-access analysis, track assignment, initial detailed routing, search-and-repair, in-memory DRC engine. The honest picture of what real DRC costs. |
| `2021-TritonRoute-WXL-integrated-DRC-engine.pdf` | The integrated-DRC successor; read for the DRC-engine architecture we'll have to reinvent for arcs. |
| `2020-DrCU-detailed-routing-sparse-grid-graph-TCAD.pdf` | **Tier 3.** Two-level sparse grid graph + correct-by-construction minimum-area path search. One-to-two orders of magnitude fewer DRVs than contemporaries. The data-structure lesson. |
| `2020-Tao-of-PAO-pin-access-oracle-detailed-routing.pdf` | Pin access as a first-class subproblem — in advanced nodes, most DRVs originate at pins, not in the channel. |
| `standard-cell-pin-access-physical-design-advanced-lithography.pdf` | Pin accessibility under unidirectional metal and multipatterning; why leading-edge layers cannot go non-Manhattan. |
| `2020-efficient-algorithms-routing-net-subject-to-design-rules-dissertation.pdf` | Bonn-school dissertation: multi-label interval shortest paths and dynamic programs for design-rule-correct single-net routing. Rigorous treatment of "shortest path subject to real DRC". |
| `improved-detailed-placement-and-routing-methodologies-thesis.pdf` | Broad thesis on detailed placement/routing co-optimization; useful context for stage 0 of our flow. |

## 06 — Topological / rubber-band routing
*The lineage the whole synthesis rests on.*

| File | Why it matters |
|---|---|
| `1986-Maley-compaction-with-automatic-jog-introduction-MIT.pdf` | Maley's MS thesis (MIT/LCS/TR-372) — the precursor to *Single-Layer Wire Routing and Compaction*. Topological invariants, homotopic compaction, and the rubber-band viewpoint in its original form. |
| `2017-constrained-routing-between-non-visible-vertices.pdf` | Routing on constrained triangulations with any-angle spanning properties — the theoretical underpinning for using a CDT as the routing substrate (§6.2). |
| `2007-Litvinova-Garkushin-topological-routing-PCB-connections.pdf` | *(Russian, open access)* An independent discrete topological model for multi-layer PCBs — "macro-discretes", per-side crossing subsets, and conflict resolution by diagonal segments. Reports 2–3× less search time and 2–3× fewer vias than shape-based routing. Not the TopoR group; a parallel Russian line of work. |
| `2008-TopoR-new-CAD-tools-russian-electronics.pdf` | *(Russian trade press)* Overview of the TopoR toolchain by the Eremex/FreeStyleTeam lineage. Context for the commercial realization of topology-first routing. |
| `2008-Eremex-TopoR-new-CAD-instrumentation.pdf` | *(Russian, vendor publication)* Companion piece on TopoR's tooling and design flow. |

> **The most useful TopoR document is not archivable** — it is the vendor's own internals
> write-up, *«Трассировка в САПР TopoR – взгляд изнутри»* (Routing in TopoR CAD — a view from
> the inside), Elektronika NTB, at `electronics.ru/journal/article/112`. It documents the
> Delaunay triangulation substrate, the layer-free initial routing, layer assignment as a
> colouring problem, the ~100-factor cost function with oscillating coefficients, the
> "layer optimization" tunnel move, and the via-cost Pareto hull. **Read it before designing
> stage 3 of the proposed flow** — see SYNTHESIS §3.2.

> The core papers of this lineage (Leiserson–Maley 1985; Dai/Dayan/Staepelaere SURF, DAC 1991;
> Dai–Kong–Sato "Routability of a Rubber-Band Sketch", DAC 1991; Dayan's 1997 UCSC thesis) are
> paywalled — see §16.

## 07 — Any-angle, continuous geometry, and multi-agent search
*The densest and most important folder. 23 papers.*

**Optimal any-angle single-path search**

| File | Why it matters |
|---|---|
| `2013-Harabor-Grastien-ANYA-optimal-any-angle-pathfinding.pdf` | **Tier 1.** ANYA: the first optimal any-angle pathfinder with no preprocessing. Searches over *intervals* of states, built on the fly. The right low-level solver. |
| `2016-Harabor-Optimal-any-angle-pathfinding-in-practice-JAIR.pdf` | The journal version with the full experimental comparison against Theta*, Lazy Theta*, Field A*, subgoal graphs. |
| `2026-optimal-any-angle-path-planning-static-dynamic.pdf` | Latest any-angle work extending to dynamic environments — relevant for incremental/ECO routing. |

**Classical computational geometry of shortest paths**

| File | Why it matters |
|---|---|
| `1987-Mitchell-Mount-Papadimitriou-discrete-geodesic-problem.pdf` | The continuous-Dijkstra paradigm and the discrete geodesic algorithm. The exact Euclidean analogue of Lee's algorithm. |
| `2017-Mitchell-shortest-paths-and-networks-handbook-DCG-ch31.pdf` | **The best single map of this territory.** Handbook of Discrete & Computational Geometry ch. 31 — visibility graphs, L1 paths, weighted regions, curvature constraints, all with complexity bounds. |
| `2011-computing-shortest-paths-among-curved-obstacles-plane.pdf` | Shortest paths when the obstacles themselves are curved — our clearance regions are discs, so this is directly applicable. |
| `lubiw-shortest-path-motion-planning-lecture.pdf` | Clean lecture treatment; good onboarding material for the team. |

**Weighted-region / anisotropic cost fields**

| File | Why it matters |
|---|---|
| `2024-exact-solutions-to-the-weighted-region-problem.pdf` | What exactness is available in the weighted-region problem despite the unsolvability result. |
| `2020-computing-close-to-optimal-weighted-shortest-paths.pdf` | Practical (1+ε) algorithms for weighted shortest paths — the realistic target for our cost-field stage. |
| `rowe-optimal-path-maps-weighted-regions.pdf` | Optimal-path *maps* across weighted regions (Snell's law refraction). Precomputed fields rather than per-query search. |

**Curvature-constrained paths**

| File | Why it matters |
|---|---|
| `1998-Reif-Wang-complexity-2D-curvature-constrained-shortest-path.pdf` | **Tier 2, hard limit.** Shortest bounded-curvature path among polygonal obstacles is NP-hard. Any min-bend-radius router is approximating, by necessity. |
| `2010-reachability-paths-bounded-curvature-convex-polygon.pdf` | Tractable special case: bounded-curvature reachability inside a convex region. |
| `2008-sketching-piecewise-clothoid-curves.pdf` | Fitting piecewise clothoids (Euler spirals) with G2 continuity. The mechanism for curvature-continuous corners. |
| `2015-continuous-curvature-path-generation-fermats-spiral.pdf` | Fermat-spiral alternative to clothoids — closed-form, cheaper to evaluate. |
| `2022-continuous-curvature-target-tree-path-planning.pdf` | Continuous-curvature planning in tight environments; practical construction patterns. |

**Multi-agent path finding — the transplant**

| File | Why it matters |
|---|---|
| `2012-conflict-based-search-optimal-multi-agent-path-finding.pdf` | **Tier 1.** CBS. Two-level: optimal single-agent low level, lazy conflict-branching high level. Provably optimal multi-agent solutions *without committing to an agent order*. This replaces rip-up-and-reroute. |
| `2024-optimal-bounded-suboptimal-any-angle-multi-agent-pathfinding.pdf` | **Tier 1.** CCBS + optimal any-angle low-level solver. This is "optimal multi-net any-angle routing" published under a different name. |
| `2025-optimal-multi-agent-path-finding-in-continuous-time.pdf` | Removes time discretization from CBS; the spatial analogue removes grid discretization. |
| `2023-clique-analysis-bypassing-continuous-time-CBS.pdf` | Scaling techniques for continuous-time CBS — bypassing and clique analysis to cut the constraint tree. |
| `2021-CBS-framework-multi-objective-multi-agent-path-finding.pdf` | Multi-objective CBS producing Pareto fronts — exactly what the multi-term objective functional in SYNTHESIS §2 needs. |
| `2020-MAPF-spatially-extended-agents.pdf` | Agents with geometric footprint, i.e. agents with width. Wires have width. Directly needed. |
| `2019-representation-optimal-multi-robot-motion-planning-CBS.pdf` | CBS over continuous configuration spaces with adaptive representation — how to keep CBS tractable off-grid. |
| `2025-passage-traversing-optimal-path-planning-sampling.pdf` | Sampling-based planning that reasons explicitly about *passages* between obstacles — structurally the same question as "which gap does this net take", i.e. topology selection. |

## 08 — Non-Manhattan geometries
*The quantitative case for more directions.*

| File | Why it matters |
|---|---|
| `2003-Y-architecture-on-chip-interconnect-analysis-methodology.pdf` | **Tier 4.** Three-direction (0/60/120°) interconnect, with the analysis methodology for wirelength/via gains. Independently corroborates the ~10% detour factor computed in SYNTHESIS Appendix A. |
| `2003-hierarchical-three-way-interconnect-hexagonal-processors.pdf` | Hexagonal/tri-directional interconnect architecture; the geometric argument in its cleanest form. |

> Teig, *The X Architecture: not your father's diagonal wiring* (DAC 2002) and the NTU
> multilevel X-router are paywalled — see §16.

## 09 — Physics-inspired methods
*PDE, field, and biological approaches.*

| File | Why it matters |
|---|---|
| `2013-fast-marching-methods-in-path-planning-survey.pdf` | **Tier 4.** Fast marching / Eikonal for path planning, including **FM2**, where the speed field is derived from clearance so paths are simultaneously short *and* well-clear. That is precisely the wire-length/spacing tradeoff. |
| `2011-path-planning-voronoi-diagram-and-fast-marching.pdf` | Voronoi/medial-axis roadmap (max clearance) combined with fast marching. Max clearance = lower critical area = higher yield, and lower coupling. |
| `2019-Physarum-inspired-Steiner-tree-in-graphs.pdf` | Slime-mould flow-reinforcement dynamics converging to Steiner-like networks. Structurally the same positive feedback as PathFinder's history cost — a smooth relaxation whose fixed points are trees. |
| `2016-physical-maze-solvers-twelve-prototypes-Lee-algorithm.pdf` | Twelve physical systems that implement Lee's algorithm (chemical, biological, fluid). Useful as an intuition pump for what wavefront routing *is*. |
| `1999-Chklovskii-wiring-optimization-in-the-brain.pdf` | Biological wiring minimization; the result that the optimum allocates ~3/5 of volume to wire. Argues for volume/density budgets rather than length budgets. |

## 10 — Learning-based methods
*What replicates, what is contested.*

| File | Why it matters |
|---|---|
| `2022-ML-for-placement-and-routing-methodological-overview.pdf` | The best orientation survey for ML in physical design. |
| `2022-GNNs-for-design-reliability-security-of-ICs.pdf` | Graph neural networks across IC design tasks, incl. congestion and DRC prediction. |
| `2020-attention-routing-track-assignment-RL.pdf` | Attention-based RL for track-assignment detailed routing — an early credible RL-inside-a-router result. |
| `2023-XRoute-RL-environment-for-routing.pdf` | An RL *environment* built on real industrial designs. If we want to train anything, this is the harness. |
| `2025-transformer-RL-net-ordering-detailed-routing.pdf` | Learned net ordering — attacks exactly the weakness that CBS attacks structurally. Worth comparing head-to-head. |
| `2025-offline-RL-detailed-routing-convergence.pdf` | Offline RL to accelerate detailed-routing convergence; no environment interaction needed. |
| `2024-ML-optimal-ordering-global-routing-semiconductors.pdf` | Learned ordering for global routing. |
| `2024-RoutePlacer-end-to-end-routability-aware-placer-GNN.pdf` | Differentiable congestion penalty via GNN, folded into placement. A template for making our objective differentiable. |
| `2023-FanoutNet-PCB-fanout-automation-deep-RL-AAAI.pdf` | Deep RL for PCB fanout — one of the few peer-reviewed ML-for-PCB results. |
| `2021-Mirhoseini-graph-placement-methodology-fast-chip-design-Nature.pdf` | AlphaChip (*Nature* 2021). Read together with the next two. |
| `2023-updated-assessment-RL-for-macro-placement.pdf` | The reassessment. |
| `2024-that-chip-has-sailed-critique-RL-macro-placement.pdf` | The rebuttal to the reassessment. Read all three before betting on end-to-end RL. |
| `2025-HeuriGym-agentic-benchmark-LLM-heuristics-combinatorial-optimization.pdf` | Benchmark for LLM-generated heuristics on combinatorial problems. |
| `2026-GR-Evolve-LLM-driven-algorithm-evolution-global-routing.pdf` | **Tier 4.** LLM-driven *evolution of the routing algorithm itself*, adapted per design. The most interesting recent framing of ML in routing. |
| `2026-agentic-AI-for-physical-design-RD-status-prospects.pdf` | ISPD 2026 invited: where agentic AI actually helps in physical design R&D. |
| `2026-PDAgent-Bench-LLM-agents-VLSI-physical-design.pdf` | Benchmark and grounding for LLM agents in physical design. |

## 11 — Analog, photonic & curvilinear routing
*The domains where curved routing is already mandatory.*

| File | Why it matters |
|---|---|
| `2024-LiDAR-automated-curvy-waveguide-routing-photonic.pdf` | Curvy-aware hierarchical A* honouring per-net bend radii and spacing, with automatic crossing insertion and port escape. **The closest published thing to what we want to build.** |
| `2025-LiDAR2.0-hierarchical-curvy-waveguide-detailed-routing.pdf` | **Tier 1.** The scaled version: 8,000+ nets in 425 s. Read this for the practical mechanics of a curvilinear detailed router. |
| `2025-Apollo-routing-informed-placement-photonic-IC.pdf` | Routing-informed placement for PICs — closes the loop that stage 0 of our flow needs. |
| `2026-end-to-end-physical-design-inverse-designed-EPIC.pdf` | Full yield-optimized physical design flow for electronic-photonic ICs with inverse-designed components. |
| `2025-AI-agents-photonic-IC-design-automation.pdf` | Agentic automation for PIC design; context for §10. |
| `2019-GeniusRoute-generative-NN-analog-routing.pdf` | **The strongest ML-in-routing result.** A VAE learns routing-region guidance from human analog layouts; an A* detailed router follows the guidance while hard-enforcing symmetry. Learn intent, keep the exact engine. |
| `2020-hierarchical-symmetry-constraints-analog-layout.pdf` | Automatic extraction of hierarchical symmetry constraints — the constraint vocabulary analog routing needs. |
| `2023-graph-attention-symmetry-constraint-extraction-analog.pdf` | GNN version of the same. |
| `2024-flexible-framework-large-scale-FDTD-inverse-design.pdf` | Open FDTD inverse-design framework — the solver tier under stage 6 of our flow. |

## 12 — Negotiated congestion (FPGA lineage)

| File | Why it matters |
|---|---|
| `1995-McMurchie-Ebeling-PathFinder-negotiated-congestion.pdf` | **Tier 1.** PathFinder. Present + historical congestion cost, iterated to convergence. The most-deployed routing algorithm ever written, and our baseline and anytime fallback. |
| `2024-open-source-fast-parallel-routing-commercial-FPGAs.pdf` | Modern parallel PathFinder-family router for commercial FPGAs; the current parallelization state of the art in this lineage. |

## 13 — Manufacturing, signal integrity, DFM

| File | Why it matters |
|---|---|
| `2001-crosstalk-aware-timing-driven-router-FPGA.pdf` | Crosstalk folded directly into the routing cost function — the earliest clean example of routing to a physical rather than geometric objective. |
| `2005-targeting-layer-and-crosstalk-minimization.pdf` | Joint layer assignment and crosstalk minimization. |
| `2024-crosstalk-aware-timing-prediction-in-routing.pdf` | Modern ML-assisted crosstalk-induced delay prediction — needed to evaluate the coupling term in our objective. |
| `electromigration-aware-routing-3D-ICs-stress-aware-EM-modeling.pdf` | EM-aware routing with stress modelling; the reliability term. |
| `2023-vertical-serpentine-interconnect-stretchable-curved-electronics.pdf` | Curved/serpentine interconnect where curvature is chosen for *mechanical strain*, not length. A different reason to be curvilinear, and a real product domain. |
| `2026-near-real-time-full-wave-inverse-design-EM-devices.pdf` | **Tier 4.** Precomputed numerical Green functions give near-real-time full-wave inverse design. If this holds up, adjoint shape optimization stops being a per-net luxury. |

## 14 — Open-source tools and PCB-scale systems

| File | Why it matters |
|---|---|
| `2021-OpenROAD-project-unleashing-hardware-innovation.pdf` | The open RTL-to-GDS flow; where TritonRoute lives, and the most likely integration target for anything we build on the IC side. |
| `2020-bridging-academic-open-source-EDA-to-real-world-usability.pdf` | The honest account of what it takes to make academic EDA usable. Worth reading before designing our interfaces. |
| `2026-3D-LineExplore-multilayer-PCB-geometric-routing.pdf` | 3D line-exploration gridless routing for multi-layer PCBs; reports >98% completion vs FreeRouting, ELECTRA, DeepPCB and optimized 3D-A*. Current published PCB SOTA, and our nearest competitor. |
| `2025-Cypress-VLSI-inspired-PCB-placement-GPU.pdf` | GPU PCB *placement* using VLSI analytical techniques — the placement half of the problem, and evidence GPU works at PCB scale. |
| `2025-double-layer-placement-IC-modules-on-PCB.pdf` | Double-layer PCB placement formulation. |
| `2026-PCBWorld-benchmark-environment-engine-grounded-PCB-design-automation.pdf` | **Tier 1 for MVP-01.** The KiCad-native PCB routing benchmark: 679 real open-source boards plus two synthetic generators, all in `.kicad_pcb`, with eight engine-checked metrics (Clean Pass, routability, DRV count, wirelength, vias, time) and published baselines — Freerouting CP 0.80 on D3-A, 0.78 on D3-B. Built on KiCad 9.0.8 + kicad-python 0.6.0. This supersedes the "build a PCB benchmark from scratch" item in SYNTHESIS §8 and is the measurement spec for [`MVP-PLAN.md`](../MVP-PLAN.md). Board data derives from PCBench (164 boards, MIT, github.com/PCBench/PCBench). |

## 15 — Surveys & books

| File | Why it matters |
|---|---|
| `2001-Hu-Sapatnekar-survey-multi-net-global-routing.pdf` | **Tier 3.** The canonical global routing survey: sequential vs concurrent, rip-up/reroute, multicommodity flow. 49 pages, still the best orientation. |
| `2009-Chen-Chang-global-and-detailed-routing-book-chapter.pdf` | The standard textbook chapter — maze/line-search/Steiner/channel/switchbox/multilevel, all in one place. Best onboarding document in the corpus. |
| `2010-Held-Korte-global-routing-VLSI-algorithms-theory-practice.pdf` | **Tier 3.** Global routing as min–max resource sharing, with the FPTAS and the randomized-rounding step. The most rigorous treatment available. |
| `Korte-combinatorial-optimization-in-VLSI-design.pdf` | Book-length Bonn-school treatment of the whole layout problem as combinatorial optimization. |
| `2017-survey-of-shortest-path-algorithms.pdf` | Broad shortest-path survey; useful for picking the low-level solver. |
| `2022-AI-ML-algorithms-applications-VLSI-design.pdf` | AI/ML across VLSI design. |
| `2025-where-paths-collide-survey-multi-agent-pathfinding.pdf` | **Tier 1 companion.** Comprehensive MAPF survey — search-based (CBS, PBS, LNS), compilation-based, and learned. The map of the field we are transplanting from. |

---

## 16 — Key works NOT archived here (paywalled), with citations

These matter enough that they should be obtained through an institutional library.

**Foundational**
- C. Y. Lee, "An Algorithm for Path Connections and Its Applications," *IRE Trans. Electronic Computers*, EC-10(3):346–365, 1961. — the original maze router.
- F. O. Hadlock, "A shortest path algorithm for grid graphs," *Networks* 7:323–334, 1977.
- J. Soukup, "Fast maze router," *DAC*, 100–102, 1978.
- K. Mikami & K. Tabuchi, "A computer program for optimal routing of printed circuit connectors," *IFIP*, 1968. — line search.
- D. Hightower, "A solution to line-routing problems on the continuous plane," *DAC*, 1969.
- J. K. Ousterhout, "Corner stitching: a data-structuring technique for VLSI layout tools," *IEEE TCAD*, 1984.

**The topological / rubber-band lineage (most important gap)**
- C. E. Leiserson & F. M. Maley, "Algorithms for routing and testing routability of planar VLSI layouts," *STOC*, 69–78, 1985. — **the key paper**: polynomial-time routability testing and wirelength-optimal routing given a sketch.
- F. M. Maley, *Single-Layer Wire Routing and Compaction*, MIT Press, 1989 (PhD thesis MIT/LCS/TR-403, 1987). Borrowable at archive.org: `singlelayerwirer00fmil`.
- W. W.-M. Dai, T. Dayan & D. Staepelaere, "Topological routing in SURF: generating a rubber-band sketch," *DAC*, 1991. `10.1145/127601.127622`
- W. W.-M. Dai, R. Kong & M. Sato, "Routability of a rubber-band sketch," *DAC*, 1991. `10.1145/127601.127623`
- T. Dayan, *Rubber-Band Based Topological Router*, PhD thesis, UC Santa Cruz, 1997. — the basis for gEDA `toporouter`.
- D. Staepelaere et al., "Surf: a rubber-band routing system for multichip modules," *IEEE Design & Test*, 1993.
- "A faster algorithm for rubber-band equivalent transformation for planar VLSI layouts," *IEEE TCAD*, 1996.

**Non-Manhattan**
- S. Teig, "The X Architecture: not your father's diagonal wiring," *SLIP*, 2002. `10.1145/505348.505355`
- T.-Y. Ho, Y.-W. Chang et al., "Multilevel full-chip routing for the X-based architecture," *DAC*, 2005.
- H. Chen, C.-K. Cheng, A. B. Kahng et al., "The Y-architecture: yet another on-chip interconnect solution," *ASP-DAC*, 2003.
- M. Zachariasen, "A catalog of Hanan grid problems," *Networks* 38(2), 2001; and generalized Hanan grids for uniform-orientation metrics, 2015.

**Global / detailed routing systems**
- M. Cho & D. Z. Pan, "BoxRouter: a new global router based on box expansion and progressive ILP," *DAC*, 2006.
- Y. Xu, Y. Zhang & C. Chu, "FastRoute 4.0," *ASP-DAC*, 2009.
- Y.-J. Chang et al., "NTHU-Route 2.0," *ICCAD*, 2008.
- M. M. Ozdal & M. D. F. Wong, "Archer: a history-driven global routing algorithm," *ICCAD*, 2007.
- M. Held, D. Müller, D. Rotter, V. Traub, J. Vygen et al., "BonnRoute: algorithms and data structures for fast and good VLSI routing," *ACM TODAES* 18(2), 2013.
- G. Chen et al., "Dr. CU: detailed routing by sparse grid graph and minimum-area-captured path search," *ASP-DAC* 2019 / *TCAD* 2020. (TCAD version **is** archived here.)
- S. Mantik, G. Posser, W.-K. Chow, Y. Ding, W.-H. Liu, "ISPD 2018 initial detailed routing contest and benchmarks," *ISPD*, 2018; and the ISPD 2019 follow-up with advanced rules.
- S. Lin, J. Liu, E. F. Y. Young et al., "GAMER: GPU-accelerated maze routing," *IEEE TCAD*, 2022.
- S. Liu et al., "FastGR / GPU-accelerated global routing," *DATE*, 2022.
- "DGR: Differentiable Global Router," *DAC*, 2024. `10.1145/3649329.3656530`
- Y.-W. Chang et al., "Challenges and approaches in VLSI routing," *ISPD*, 2022 (invited survey covering GPU routing, PCB routing and AI-driven analog routing). `10.1145/3505170.3511477`

**Steiner trees**
- C. Chu & Y.-C. Wong, "FLUTE: fast lookup table based rectilinear Steiner minimal tree algorithm," *IEEE TCAD*, 2008.
- D. M. Warme, P. Winter & M. Zachariasen, GeoSteiner (software + papers) — `geosteiner.com`.
- J. Liu et al., "REST: constructing rectilinear Steiner minimum tree via reinforcement learning," *DAC*, 2021.

**PCB-specific — the TopoR lineage**
- S. Yu. Luzin & O. B. Polubasov, "Топологическая трассировка: реальность или миф?" (Topological routing: reality or myth?), *EDA Expert* №5, 42–46, 2002. — the manifesto.
- S. Yu. Luzin & O. B. Polubasov, "Advantages of isotropic PCB routing," *Printed Circuit Design & Fab*, №6, 38–40, Feb 2009. — the any-angle argument, with the BGA layer-count and area figures. Mirrored by EDN as "Speed and improve PCB routing".
- O. B. Polubasov, "Routing Concepts of a Topological Router CAD System," *Onboard Technology*, 11–15, May 2011.
- «Трассировка в САПР TopoR – взгляд изнутри», *Электроника НТБ*, `electronics.ru/journal/article/112`. — **the internals paper.** Triangulation, layer-free initial routing, layer colouring, the ~100-factor cost function, the layer-optimization tunnel move, the Pareto hull.
- «Гибкая топологическая трассировка в произвольных направлениях», *Электроника НТБ* №1, 2013, `electronics.ru/journal/article/3556`. — any-angle rationale incl. the FR-4 glass-weave anisotropy and thermal-symmetry arguments, and the 784-pin BGA 10→7 layer result.
- Q. Ma, E. F. Y. Young et al., escape routing and length-matching bus routing series (*IEEE TCAD* / *ICCAD*, 2010–2020).
- "PCB routing on unstructured meshes with conflict-based search" (UGPCB-CBS), *J. Supercomputing*, 2025. `10.1007/s11227-025-07569-0` — **CBS on a Delaunay mesh for PCB. Directly on our path; get this one.**
- LiDAR 3.0, *ISPD* 2026. `10.1145/3764386.3779589`

**Tools worth reading the source of (no paper)**
- `freerouting` (A. Wirtz) — shape-based push-and-shove PCB autorouter, Java, open source.
- KiCad `pns_*` (T. Wlostowski) — the interactive push-and-shove router; `pns_walkaround.cpp` computes convex-hull-hugging paths, which is a rubber-band relaxation in miniature.
- gEDA `toporouter` (A. Blake, GSoC 2008, mentored by DJ Delorie) — CDT-based topological router implementing Dayan's thesis.
- `bbenchoff/OrthoRoute` (2025, MIT licence) — GPU wavefront + PathFinder KiCad autorouter; CuPy/CUDA. Practical proof GPU routing works at PCB scale.
- `cuhk-eda/dr-cu`, `The-OpenROAD-Project/TritonRoute` — the two open detailed routers worth reading.
