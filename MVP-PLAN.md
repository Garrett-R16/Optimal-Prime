# MVP-01 — A Routing Optimizer for KiCad

*Companion to [`SYNTHESIS.md`](SYNTHESIS.md). That document argues what optimal routing could
be. This one narrows it to one buildable, measurable thing.*

---

## 0. The contract

**In scope.** Given a `.kicad_pcb` whose components are already placed and whose stackup and
via rules are already fixed, produce a `.kicad_pcb` in which every net is fully connected and
KiCad's own DRC reports zero errors — optimizing wirelength and via count, with no constraint
that the geometry look human-drawn.

**Frozen (deliberately, for now).**

| Frozen | Why |
|---|---|
| Placement | Placement/route co-optimization is a second research program. Fixing it makes every board a deterministic instance. |
| Layer stackup | Layer count and dielectric are inputs, not variables. |
| Via types — through-hole only | No buried, no blind, no micro. One via geometry, one cost. Layer assignment stays a colouring problem (SYNTHESIS §3.2) rather than a 3D stack search. |
| Differential pairs | Length *matching* pulls against length *minimization*; it is a different objective, and it is TopoR's documented weak spot. Adding it before the base optimizer works would confound every measurement. |
| Length tuning / delay matching | Same reason. |
| Impedance control, crosstalk, EM | The full `J[layout]` functional of SYNTHESIS §2 requires extraction we do not have. MVP-01 optimizes the proxy (length, vias) and is honest that it is a proxy. |
| Copper pours / zones | Treated as fixed obstacles if present, never generated. |
| Compute time | **Explicitly not a goal.** An hour per board is acceptable. This unlocks exact methods, exhaustive search, thousands of restarts, and an oracle call on every candidate — all of which are off the table for a production router and all of which are exactly what a research MVP should exploit. |

**Done means:** on the PCBWorld D3-A open-source board set, Clean Pass ≥ 0.80 — matching
Freerouting, the strongest free baseline — using any-angle geometry, with the result loading
into KiCad and passing DRC unmodified.

**Non-goal:** beating Freerouting on speed. We will lose by two to four orders of magnitude
and that is fine.

---

## Status

| Phase | State | Evidence |
|---|---|---|
| **P0 — Oracle** | **complete** | All four exit criteria verified. `unconnected_items` *is* in the CLI JSON and `kicad_version` comes free, so the risk in section 7.2 did not materialise. |
| **P1 — Floor, bar, machinery** | **complete except the bar** | E2 run: 15 boards x 20 seeds x 2 strategies, 600 cells, 0 crashes. See [`results/E2-findings.md`](results/E2-findings.md). S2 (Freerouting) remains unavailable -- `kicad-cli` has no Specctra export -- so the *bar* is still the published 0.80 rather than a number measured here. |
| P2 onward | not started | |

**Headline results.** S1 beats the S0 floor on 14/15 boards with a geometric-mean DRV ratio
of 0.011, and routes within 37% of the Steiner lower bound against the floor's 867%.
**E2: net ordering alone moves violation count by a median of 50% (up to 140%), while moving
wirelength by 14.4%** -- the measured form of the section 0 claim that the combinatorial
choice dominates the geometric one. Net order never flipped Clean Pass on any board, which is
a negative result P2 should re-test with a better router.

45 tests pass, including the Level 1 differential and Level 3 synthetic instances.

---

## 1. Why this is the right cut

Three reasons this particular slice is worth building first, rather than a smaller or larger one.

1. **It is the smallest scope in which the founding hypothesis is falsifiable.** The claim
   behind this repo is that abandoning straight-lines-and-45s buys real completion, not just
   prettier pictures. That claim needs exactly: fixed placement (so geometry is the only
   variable), a fixed stackup (so via cost is a scalar), and a binary pass/fail oracle. It
   does not need diff pairs, impedance, or timing. Adding those adds confounds, not evidence.

2. **The oracle already exists and we did not have to build it.** `kicad-cli pcb drc` is a
   full production DRC engine with a machine-readable output, and it understands arc tracks
   natively. SYNTHESIS §7 listed "there is no curvilinear DRC engine" as the project's critical
   path. For PCB, at MVP scale, that turned out to be wrong: KiCad is one. We should exploit
   that shamelessly rather than write our own.

3. **The benchmark already exists too, and it is KiCad-native.** PCBWorld (arXiv 2607.05915,
   2026) ships 679 real open-source boards plus two synthetic generators, all in `.kicad_pcb`,
   with eight engine-checked metrics and published baselines. SYNTHESIS §8 proposed building a
   PCB routing benchmark from scratch. That is now redundant. Adopt theirs; the comparability
   is worth far more than a bespoke suite.

Everything below is built around the consequence of (2) and (3): **the hard infrastructure is
already available, so the MVP is mostly a search-strategy experiment.**

---

## 2. Architecture: build the arena first, the router second

The user requirement that different approaches — including deliberately random ones — be
tried and compared dictates the shape of the system. It is not "a router with some options."
It is a **measurement harness with pluggable routers**, where the plugin interface is narrow
enough that a fifty-line random baseline and a full CBS-over-triangulation implementation both
satisfy it.

```
             .kicad_pcb                                     .kicad_pcb
                 |                                              ^
                 v                                              |
        +-----------------+      +-------------+      +-----------------+
        |  problem.py     | ---> |  Strategy   | ---> |    emit.py      |
        |  parse -> IR    |  P   |  (plugin)   |  S   |  IR -> board    |
        +-----------------+      +-------------+      +-----------------+
                                       ^                        |
                                       |                        v
                                 +-----------+          +----------------+
                                 | runner.py |<---------|   oracle.py    |
                                 | matrix,   |  Score   | kicad-cli drc  |
                                 | seeds, DB |          | (ground truth) |
                                 +-----------+          +----------------+
```

### 2.1 Which door into KiCad

Three exist. They are not equivalent and picking wrong costs weeks.

| Door | Mechanism | Verdict |
|---|---|---|
| **Specctra DSN / SES** | KiCad exports `.dsn`, external router returns `.ses`. What Freerouting uses. | **No.** Lossy on design rules (the copper-to-hole clearance bug is well documented), fragile parser on both sides, and it round-trips through a 1990s format that has no concept of our objective. Use it *only* to run Freerouting as a baseline. |
| **IPC API (`kicad-python` / `kipy`)** | Protobuf over a socket to a *running* KiCad 9+. `Board.create_items()` accepts `Track`, `ArcTrack`, `Via`; `begin_commit()`/`push_commit()` group them into one undo step. | **Yes, for the interactive plugin at the end.** Stable, officially supported, and this is what PCBWorld itself is built on (KiCad 9.0.8 + kicad-python 0.6.0). But it requires a live GUI process, which makes a 10,000-run experiment matrix miserable. |
| **Direct `.kicad_pcb` s-expression I/O** | Read and write the board file. Arc tracks are first-class: `(arc (start X Y) (mid X Y) (end X Y) (width W) (layer L) (net N) (uuid ...))`. | **Yes, for the batch loop.** Headless, trivially parallel, perfectly reproducible, no GUI, and a board file is a diffable artifact you can archive with the run. |

**Decision: file I/O for the experiment loop, IPC for the demo.** The `emit.py` boundary is
where the two converge — write it to target the file format, and add a thin `kipy` adapter
later that walks the same `Solution` object.

The one hazard: the SWIG `pcbnew` Python module is deprecated as of KiCad 9 and slated for
removal in KiCad 11. Do not build on it, and be sceptical of any tutorial or plugin that does.

### 2.2 The problem IR

Deliberately small. Everything the frozen scope allows us to ignore, we drop at parse time.

```python
@dataclass(frozen=True)
class Problem:
    board_outline: Polygon             # Edge.Cuts, as a closed region
    layers:        list[LayerId]       # copper only, ordered top -> bottom
    nets:          list[Net]           # Net.terminals: list[Pad]
    pads:          list[Pad]           # position, shape, layers, net, is_smd
    obstacles:     list[Obstacle]      # keepouts, zones, existing locked copper, holes
    rules:         DesignRules         # clearance, track widths, via drill/annulus,
                                       # hole-to-hole, board-edge clearance
    via_stack:     ViaStack            # through-hole only, one geometry
```

A `Solution` is per-net: an ordered list of `Segment | Arc` per layer plus `Via` transitions.
Nothing else. No grid, no tiles, no strategy-specific state — those live inside strategies so
that a grid router and a triangulation router produce comparable output.

**Arcs.** Internally, keep exact arc primitives (centre, radii, angles). Discretize to chords
with bounded sagitta (≤ 1 µm) *only* for our own fast pre-checks. Always emit true `(arc ...)`
to the board so KiCad's DRC evaluates the real geometry, not our approximation.

**Clearance guardband.** KiCad's arc clearance arithmetic has known rounding artifacts —
reports of a 10 mil clearance measuring 9.99996 mil and failing. Route to `clearance + ε` with
ε = 2 µm. This is a two-line fix that will otherwise eat days of phantom debugging.

### 2.3 The strategy interface

One method. That is the point.

```python
class Strategy(Protocol):
    name: str
    def route(self, problem: Problem, rng: Random, budget: Budget) -> Solution: ...
```

`rng` is seeded per run so every stochastic strategy is reproducible. `budget` carries a wall
clock cap (generous — one hour) and an optional oracle-call cap, but strategies are free to
ignore it. Strategies may call the oracle themselves; runs that do are tagged, because a
strategy that queries KiCad's DRC in its inner loop is doing something categorically different
from one that does not, and the comparison must acknowledge it.

### 2.4 The scorer

```
kicad-cli pcb drc --format json --severity-all --units mm -o out.json board.kicad_pcb
```

The JSON carries `violations`, `unconnected_items`, and `schematic_parity`. Metrics, adopted
from PCBWorld so numbers are directly comparable to published baselines:

| Metric | Definition | Role |
|---|---|---|
| **CP** — Clean Pass | 1 iff `unconnected_items` is empty **and** no error-severity violation | **Primary. Binary. Non-negotiable.** |
| **Rout.** — routability | fraction of target connections routed | Partial credit; the gradient CP does not give you |
| **DRV** | count of error-severity violations | Diagnostic — *which* rules a strategy breaks is more informative than how many |
| **WL** | total copper length (arcs by true arc length) | Secondary objective |
| **Via** | via count | Secondary objective |
| **Time** | wall clock | **Recorded, never optimized.** |

Two additions of our own:

- **`WL_ratio` = WL / Σ (net Steiner lower bound).** Absolute wirelength is not comparable
  across boards. The ratio is. Compute the bound with a Euclidean Steiner minimal tree per net
  ignoring obstacles — GeoSteiner solves these exactly at MVP net sizes. This gives every board
  an interpretable "how far above the physical floor are we" number, and it is the metric in
  which the ~21.5% any-angle prize of SYNTHESIS §0.2 should actually show up.
- **`J_lite` = WL_ratio + w_v · vias**, with `w_v` swept rather than fixed — reported as a
  Pareto front over `w_v`, following TopoR's via-cost hull (SYNTHESIS §3.2). Never collapse to
  one scalar; the via/length tradeoff is the one weight nobody agrees on.

**Verify first (Phase 0, day one):** that `unconnected_items` genuinely appears in the CLI JSON
and genuinely catches an unrouted net. Build a board with one net deliberately open and confirm
the oracle fails it. The entire measurement stack rests on this; test it before trusting it.

---

## 3. The strategy portfolio

The research method is a tournament. Every entry below implements the same interface, is scored
identically, and is expected to lose to something — including the ones we believe in.

### Tier 0 — controls and external baselines

| ID | Strategy | What it is for |
|---|---|---|
| **S0** | **Uniform-random geometry.** Random waypoints, connect, ignore everything. | The floor. Establishes what a score of "no algorithm" looks like. Every plot needs this line. |
| **S1** | **Random net order + greedy grid A\*.** Fine grid, first-come-first-served, no rip-up. Run 10,000 seeds per board, keep the best. | The single most informative cheap experiment we can run. The *spread* across seeds measures how much of routing is net ordering and nothing else — which is the empirical version of the SYNTHESIS §0.3 claim that ordering dominates angle. |
| **S2** | **Freerouting** via DSN/SES. | The number to beat. Published: CP 0.80 on D3-A, 0.78 on D3-B. |
| **S3** | **KiCad's own PNS router**, driven through the IPC API. | The engine-native baseline, and the one users would otherwise reach for. |

### Tier 1 — classical, on a grid

| ID | Strategy | Papers |
|---|---|---|
| **S4** | **PathFinder negotiated congestion.** Rip-up and reroute with present + historical congestion cost. | folder 12; the baseline every academic router is measured against |
| **S5** | **Sequential + adaptive rip-up**, ordering by criticality then by failure history. | folder 05 (TritonRoute's search-and-repair) |
| **S6** | **Simulated annealing over net order**, with S1's greedy router as the evaluator. | `1994-simulated-annealing-maze-routing.pdf` |

Tier 1 exists to prove the harness measures something real. If S4 does not beat S1, the harness
is broken, not the algorithm.

### Tier 2 — the thesis

| ID | Strategy | The idea |
|---|---|---|
| **S7** | **CDT + shortest-path-in-triangulation + rubber-band embedding.** Constrained Delaunay triangulation of the board; route each net as a face-sequence through the triangulation; relax the resulting path to its taut geometric realization (segments + arcs). | The SYNTHESIS §7 core, and TopoR's architecture. Any-angle falls out for free — it is not an added feature, it is what relaxation produces. |
| **S8** | **S7 + deferred layer assignment.** Route all nets in one plane allowing crossings, then colour crossings into layers. Exact via minimum for 2 layers. | Stolen from TopoR (SYNTHESIS §3.2). Collapses the layer dimension out of the search. |
| **S9** | **Conflict-Based Search over the triangulation.** High level branches on conflicts between nets; low level is optimal any-angle single-net search. | folder 07. Gives a *bounded-suboptimality guarantee*, which nothing else on this list has. Expected to be too slow for large boards — which is fine, because compute time is not a goal, and the small-board result tells us how far from optimal everything else is. |
| **S10** | **Simulated annealing over topology, not geometry.** A move flips which side of an obstacle a net passes. Geometry is recomputed by exact relaxation after every move. | The cheap version of S9. Searches the same space without the guarantee. If S10 ≈ S9 in quality, S9 is a research instrument rather than a product. |

### Tier 3 — sampling and fields (the deliberately weird ones)

| ID | Strategy | The idea |
|---|---|---|
| **S11** | **RRT\* / PRM per net**, in continuous space with clearance-aware sampling. | Sampling-based motion planning transplanted directly. Naturally any-angle, naturally asymptotically optimal, and completely unused in EDA. Genuinely might be terrible. Worth one week to find out. |
| **S12** | **Eikonal / Fast Marching cost field.** Solve for arrival time under a clearance-derived speed field (FM2), descend the gradient. | folder 09. Produces short-*and*-clear paths in one shot, and generalizes to weighted regions where cost is not uniform. |
| **S13** | **Monte-Carlo Tree Search over net order and topology choices.** | The tree-search framing of the same combinatorial core; a bridge to Tier 4 if we ever get there. |

### Tier 4 — learned (out of MVP scope, listed so the interface accommodates it)

Learned net ordering (`2025-transformer-RL-net-ordering-detailed-routing.pdf`) and learned
guidance plug in as a *reordering of S4/S7's queue*, not as a replacement router. Keep the
interface able to accept a hint vector; do not build this in MVP-01.

---

## 4. How this gets tested

Six levels, bottom to top. Levels 0–2 are where the bugs actually are.

**Level 0 — geometric predicates against a brute-force reference.**
Segment–segment, segment–arc, and arc–arc minimum distance. Test each against a densely sampled
numeric reference on 10⁶ random configurations, including the degenerate cases that will
otherwise bite: concentric arcs, tangent arcs, zero-length segments, arcs subtending > π,
coincident endpoints. This is boring and it is the foundation of everything.

**Level 1 — differential test against KiCad's DRC.**
The highest-value test in the plan. Generate random layouts — valid, marginal, and deliberately
violating — run both our internal checker and `kicad-cli pcb drc`, and assert agreement on
every violation. **Any disagreement is a bug in us until proven otherwise.** Log the marginal
cases; the disagreement distribution tells us exactly how large the guardband ε needs to be.
Target: zero disagreements over 10,000 generated layouts before any strategy is trusted.

**Level 2 — round-trip and invariant properties.**
`parse → emit → parse` is the identity on all 679 benchmark boards. A `Solution` never
self-intersects. Every routed net's geometry induces a connected graph spanning its pads. Every
via lands on a legal layer pair. Property-based (Hypothesis), not example-based.

**Level 3 — synthetic instances with known answers.**
Hand-build boards where the optimum is computable: two pads with one obstacle (answer is a taut
path — check the relaxation reproduces it exactly); three-terminal net in free space (answer is
the Steiner point at 120°); a board where any-angle strictly beats every 45° route (check we
find it); a board that is provably unroutable in 2 layers (check we report failure rather than
emitting a DRC-violating board — **silent failure is worse than reported failure**).

**Level 4 — regression on golden boards.**
A pinned set of ~20 boards spanning the difficulty range. Every commit runs them. Score
regressions block the merge. Results stored as JSON in `results/`, diffable in review.

**Level 5 — the benchmark.**
PCBWorld D2 (synthetic gridless, 2-layer, generator released), D3-A (100 small open-source
boards), D3-B (10 medium). PCBench (164 boards, MIT licence) as the independently-sourced
fallback if PCBWorld's code release is not yet public. Report CP, Rout., DRV, WL_ratio, Via,
Time, per-board, no aggregation-only summaries.

**A test discipline that matters more than any of the above:** the oracle is external and we do
not control it. Pin the KiCad version, record it in every result record, and re-run the full
matrix when it changes. A score is meaningless without the engine version attached.

---

## 5. The experimental method

The user asked for a research method, not a build order. Here is the protocol.

**Design.** Full factorial: strategy × board × seed. Ten seeds minimum for stochastic
strategies, one for deterministic. Same boards, same seeds, same oracle, same machine.

**Reporting.** Per-board paired deltas against the incumbent, never aggregate means alone —
mean CP across boards of wildly different difficulty is close to meaningless. Bootstrap
confidence intervals. Publish the full matrix, including the runs that failed.

**Admission rule.** A strategy enters the incumbent position if it beats the current incumbent
on Clean Pass by a paired sign test at p < 0.05, **or** if it is on the Pareto front of
(CP, WL_ratio, Via) even while losing on CP. The second clause exists to keep S9/S12 alive:
a strategy that wins on quality and loses on completion is telling us something.

**Preregistration.** Fix the metric and the board set before running. The temptation to pick
the board subset where your favourite strategy wins is enormous and it will happen
unconsciously. Write the analysis down first.

**Ablations that are worth more than the tournament itself:**

- **E1 — the any-angle ablation. The founding hypothesis of this repo, tested directly.**
  Take one topology search (S7's). Run it with two embedders: taut any-angle relaxation, versus
  the same topology snapped to 45°. Everything else identical. Measure ΔCP and ΔWL_ratio.
  *Prediction from SYNTHESIS §0.2: ΔWL_ratio ≈ 4–5% (octilinear already captures 17.2% of the
  21.5% prize).* If ΔCP is also near zero, the honest conclusion is that any-angle geometry is
  a rounding error and this project's value lives entirely in topology search. **Run this
  experiment early. It is the one that can kill the thesis, which is exactly why it goes first.**

- **E2 — the ordering ablation.** S1's seed spread, per board. Quantifies the ceiling available
  to any method that only reorders nets, and therefore how much of the win S4/S9 must come from
  elsewhere.

- **E3 — topology vs geometry search.** S10 (SA over topology, exact geometry) versus S6 (SA
  over net order, greedy geometry), matched on evaluation count. Directly tests SYNTHESIS §0.1.

- **E4 — the optimality gap.** On small boards only, S9 (CBS, bounded-suboptimal) gives a
  certified bound. Measure how far every heuristic sits from it. This is the only way to know
  whether we are chasing the last 5% or the first 50%, and no published PCB router reports it.

### 5.1 What makes two runs comparable

**Nothing is validated without KiCad.** `kicad-cli` *is* KiCad — the same C++ binary set, the
same DRC engine, the same `SHAPE` geometry library the GUI uses. The headless/GUI distinction
in §2.1 is about process model, not about which checker runs. Every score on the leaderboard
comes from the engine.

But running the same binary is necessary, not sufficient. Six things have to be pinned before
two numbers mean anything next to each other.

1. **Engine version.** DRC behaviour changes between KiCad releases. Pin one version, record it
   in every result record, and re-run the entire matrix on upgrade. Never mix versions in one
   comparison. A score without an engine version attached is not a score.

2. **The effective rule set — the trap that will bite hardest.** DRC *severities* do not live in
   the `.kicad_pcb`; they live in the project file (`.kicad_pro`), and custom rules live in a
   `.kicad_dru` sidecar. A benchmark board shipped with "clearance violation → warning" would
   sail through Clean Pass while producing an unmanufacturable board, and nothing about the run
   would look wrong. **Normalization is therefore part of board ingestion, not an afterthought:**
   every board gets a canonical `.kicad_pro` with a fixed severity map, its `.kicad_dru` is
   either preserved-and-hashed or the board is rejected from the set, and the SHA-256 of the
   effective rule set goes in the run record.

3. **The instance.** SHA-256 of the input board *after* normalization. Two runs are comparable
   only if this matches. It also catches the case where a benchmark upstream silently updates.

4. **Seed and code version.** The RNG seed plus the git commit of the strategy. Stochastic
   strategies are only reproducible if both are recorded.

5. **Metric definitions.** `score.py` is versioned. Old runs are never silently rescored with new
   metric code — either re-run them or tag them with the old version.

6. **The machine — for `Time` only.** Wall clock is the one metric that does not survive a change
   of hardware. Which is one more reason it is recorded and never optimized.

**The run record.** Every run writes one self-describing JSON, and the leaderboard is regenerated
from these files alone — there is no separate database that can drift out of sync:

```json
{
  "board":       {"name": "...", "sha256": "...", "rules_sha256": "...", "nets": 47, "layers": 2},
  "strategy":    {"name": "S7", "git": "a1b2c3d", "params": {}},
  "seed":        7,
  "engine":      {"kicad": "9.0.8", "cli_path": "..."},
  "score_ver":   3,
  "metrics":     {"cp": 1, "rout": 1.0, "drv": 0, "wl_mm": 812.4,
                  "wl_ratio": 1.19, "vias": 31, "time_s": 214.7},
  "drv_detail":  [{"type": "clearance", "at": [0, 0], "layer": "F.Cu", "nets": [3, 9]}],
  "artifacts":   {"board_out": "routed.kicad_pcb", "drc_json": "drc.json"},
  "host":        {"os": "...", "cpu": "...", "cores": 16}
}
```

**Two checkers, one truth.** Strategies need a fast in-process clearance check in their inner
loop; a subprocess call to `kicad-cli` per candidate path would make even an unbounded compute
budget useless. So there are two checkers, in an asymmetric relationship:

- The **internal checker** (`geometry.py`) is fast, in-process, and **conservative by
  construction** — the ε guardband means it may call a legal layout illegal, but never the
  reverse. A conservative checker can cost us completion. It cannot cost us correctness.
- The **oracle** (`kicad-cli`) is slow and authoritative. **Only oracle scores appear on the
  leaderboard.** If a strategy's internal check is optimistic, it shows up as a Clean Pass loss,
  which is exactly the right punishment.

Level 1's differential test (§4) is the contract between the two. Until it reads zero
disagreements, no strategy result is trusted.

**Cross-checking the oracle itself.** Connectivity and wirelength are computed *independently*
from our own geometry and asserted equal to what KiCad reports. Two implementations that agree
are probably right; two that disagree tell you one of them is wrong — which is information a
single source of truth cannot give you.

---

## 6. Phasing

Each phase has an exit criterion. Do not start the next one until it is met.

| Phase | Build | Exit criterion |
|---|---|---|
| **P0 — Oracle** (~1 wk) | `problem.py`, `emit.py`, `oracle.py`, `score.py`. Board fetch script. | Parse/emit round-trips all benchmark boards. Oracle correctly fails a deliberately-open net and a deliberately-shorted board. Freerouting's published CP reproduced within noise on D3-A. **Zero router code written.** |
| **P1 — Floor, bar, machinery** (~1 wk) | S0, S1, S2, S3. `runner.py`, results DB. | E2 complete: the net-order spread is measured and plotted. We know what random looks like and what Freerouting looks like on our machine. |
| **P2 — A real router** (~2 wk) | S4 (PathFinder), S5. Level 0–2 test suites. | S4 > S1 with statistical significance. Level 1 differential test at zero disagreements. |
| **P3 — Geometry** (~3 wk) | CDT substrate, rubber-band relaxation, arc emission. S7. | Level 3 synthetic instances pass exactly. Arcs survive the KiCad round-trip and pass DRC. **E1 run and reported, whatever it says.** |
| **P4 — Topology** (~4 wk) | S8, S9, S10. | E3 and E4 reported. S9 produces a certified bound on at least the D3-A small boards. |
| **P5 — The long tail** (~2 wk) | S11, S12, S13. | Each either enters the Pareto front or is documented as a negative result and retired. |
| **P6 — Ship the MVP** (~2 wk) | Best strategy wired to the IPC API as a KiCad plugin. | CP ≥ 0.80 on D3-A. A human can click a button in KiCad 9 and get a routed board that passes DRC. |

**Between phases**, the full tournament re-runs and the incumbent is updated. The leaderboard is
regenerated from `results/` — it is a view, never a hand-maintained file.

**Negative results get written down.** A retired strategy gets a paragraph in
`results/NEGATIVE.md` saying what it was, what it scored, and why we think it lost. This is the
part of research-based development that everyone skips and then repeats.

---

### P0 — Oracle (~1 week)

**Goal: prove we can measure, before anything exists that routes.**

**Build**

| File | Substance |
|---|---|
| `fetch_boards.py` | Pull PCBench, filter to the frozen scope (placement complete, through-hole vias only, 2–8 copper layers), **normalize the rule set** per §5.1, hash, freeze into `boards/manifest.json`. |
| `sexpr.py` | Lossless s-expression reader/writer. |
| `problem.py` | Token tree → `Problem` IR. |
| `emit.py` | `Solution` → spliced token tree → `.kicad_pcb`. |
| `oracle.py` | `kicad-cli` wrapper; parses `violations` + `unconnected_items`; records engine version. |
| `score.py` | Oracle JSON → CP / Rout. / DRV / WL / Via, plus independent connectivity and wirelength computed from our own geometry as a cross-check. |

**Three decisions that are hard to reverse later**

- **Write the s-expression layer ourselves rather than using `kiutils` or similar.** Lossless
  round-trip is a hard requirement, and third-party board models lag KiCad releases — anything
  that parses into a semantic model and regenerates will silently drop fields added in a version
  it does not know about. Parse to a **token tree, preserve unknown nodes verbatim, and mutate
  only the track section.**
- **Integer nanometres internally.** The file format stores millimetres as decimal; KiCad's
  internal units are integer nanometres. Working in floats and converting at the end produces
  failures at exactly the tolerance we care about. Convert once, at the I/O boundary.
- **Deterministic UUIDs**, hashed from `(strategy, seed, item index)` rather than random. Reruns
  then produce **byte-identical** board files, which turns reproducibility into a `diff` instead
  of a statistical argument.

**Exit criteria, in order**

1. `parse → emit → parse` is the identity on every benchmark board, and the re-emitted board
   scores identically to the original under DRC.
2. A rerun of any run produces a byte-identical output board.
3. Delete one track from a clean board → oracle reports `unconnected_items`. Overlap two tracks →
   oracle reports a clearance violation. **Verify this before trusting anything downstream.**
4. Freerouting's published CP ≈ 0.80 on D3-A reproduces on our harness within noise.

**Explicitly not done:** no routing code, no geometry predicates, no tests beyond round-trip.

**If criterion 3 fails** — i.e. `unconnected_items` is absent from the CLI JSON — connectivity
becomes our job: compute it in `score.py` from emitted geometry against pad shapes, and validate
that implementation against KiCad's ratsnest over the IPC API on a sample of boards. Cost: a few
days, not a redesign.

---

### P1 — The floor, the bar, and the comparison machinery (~1 week)

**Goal: build the machinery that compares strategies to each other, and calibrate it with the
two reference points every later number is read against.**

**Where comparison actually happens.** P0 answers "can we score *one* run?" P1 answers "can we
score *many* runs and rank them?" — `runner.py` is the comparison machinery, and everything from
P2 onward is a new entrant fed into it. The comparison never moves; only the field of entrants
grows.

**Floor and bar, not floor and ceiling.** The *floor* is S0, uniform-random geometry: what a
score looks like with no algorithm at all. Without it, a Clean Pass of 0.4 is an uninterpretable
number. The *bar* is S2, Freerouting: the best freely available router, and what a user gives up
by switching to us. Neither is an upper bound — a true ceiling requires a certified optimum, and
that only arrives in P4 with S9 and experiment E4.

**Build:** `runner.py` — the strategy × board × seed matrix; multiprocessing, crash-isolated (one
strategy segfaulting must not lose the matrix), resumable (a re-run skips completed cells), one
JSON per cell. `registry.py` for plugin discovery. Then **S0** (uniform-random geometry), **S1**
(random net order + greedy grid A*, 10,000 seeds per board), **S2** (Freerouting headless via
DSN/SES), **S3** (KiCad PNS).

**Two honest caveats found while planning this**

- **S3 may not be buildable.** Stock `kipy` exposes board item CRUD — `create_items`,
  `update_items`, `get_tracks` — not the interactive router. PCBWorld reached `PNS::ROUTER` by
  writing its own 14 bindings against the C++ engine. If PCBWorld's code is not released, S3
  either gets dropped or costs a C++ binding effort disproportionate to a baseline. Not a
  blocker: S2 is the baseline that matters.
- **DSN export may need the GUI.** If `kicad-cli` has no Specctra export subcommand, S2's inputs
  get exported once through the GUI and cached alongside the board. Ugly, one-time, acceptable.

**Exit:** **E2 complete** — the net-order seed spread measured and plotted per board. This single
plot is the most informative cheap result in the plan: the width of that distribution is the
share of the routing problem that is ordering and nothing else.

**Explicitly not done:** no rip-up, no internal DRC, no geometry library.

---

### P2 — A real router (~2 weeks)

**Goal: a router that genuinely works on a grid, and the fast internal checker every later
strategy depends on.**

**Build**

- `geometry.py` — exact predicates, integer arithmetic where possible: point–segment,
  segment–segment, segment–arc, arc–arc minimum distance. This is the foundation of every
  strategy from here on and it is worth over-testing.
- The internal DRC: spatial index (a uniform bucket grid suffices at PCB scale), clearance
  queries, board-edge clearance, hole-to-hole.
- Layered grid graph, A* with an explicit via cost term.
- **S4 — PathFinder.** Cost `(base + h_n)·(1 + p_n)`, with `p_n` the present-congestion term
  updated within an iteration and `h_n` the historical term accumulated across iterations. It is
  a short algorithm; implement it directly from the paper.
- **S5** — sequential with adaptive rip-up ordered by failure history.

**Exit**

1. Level 0: predicates agree with a densely-sampled numeric reference over 10⁶ random
   configurations, including tangent arcs, arcs subtending > π, and zero-length degenerates.
2. **Level 1: zero disagreements with `kicad-cli` over 10,000 generated layouts.** Until this
   holds, every strategy result is provisional.
3. Level 2 property tests pass.
4. **S4 beats S1's best-of-10,000 on Clean Pass, paired sign test, p < 0.05.** If it does not,
   the harness is broken — not the algorithm. Stop and fix the harness.

**Explicitly not done:** no arcs, no triangulation. Everything is still on a grid, deliberately.

---

### P3 — Geometry (~3 weeks)

**Goal: any-angle output, and the experiment that can falsify this repository's premise.**

**Build**

- **The CDT substrate.** Constrained Delaunay triangulation of the board: vertices at pad and
  obstacle geometry plus the outline; constrained edges along obstacle boundaries. Per-edge
  capacity `floor((len − 2·clearance)/(width + spacing))`.
- **Topology as a channel** — a net's topology is the ordered sequence of triangulation edges it
  crosses. Nothing geometric is committed at this stage.
- **The embedding: the funnel algorithm.** Given a channel, the funnel algorithm computes the
  exact taut path through it in linear time. Inflate obstacle vertices by the clearance radius
  and the taut path becomes *tangent segments plus arcs on the inflated circles* — precisely what
  `(arc (start)(mid)(end))` expresses. **This is where any-angle geometry stops being an
  aspiration and becomes a subroutine.** It is exact, it is polynomial, and it is the concrete
  form of the SYNTHESIS §0.1 claim.
- **S7** = sequential shortest-channel search + funnel embedding + rip-up.
- Arc emission with the ε guardband.

**Exit**

1. Level 3 synthetic instances exact: the taut path around a single obstacle matches the closed
   form; the three-terminal free-space Steiner point lands at 120°; the provably-unroutable
   2-layer board is *reported as failed*, not emitted with violations.
2. Arcs survive the KiCad round-trip and pass DRC.
3. **E1 run and reported, whatever it says.**

**Why S7 is deliberately a dumb sequential search:** E1 must compare two *embedders* under the
same topology search. Keeping the search simple removes the confound. Topology search is P4's
job, and mixing the two would make E1 uninterpretable.

---

### P4 — Topology (~4 weeks)

**Goal: attack the part that is actually hard.**

**Build**

- **S8 — deferred layer assignment.** Route every net in one plane with crossings permitted, then
  build the crossing conflict graph and colour it. Two layers: exact via minimum. More than two:
  heuristic, and labelled as such in every result. Taken from TopoR (SYNTHESIS §3.2).
- **S9 — CBS over the triangulation.** The high level branches on a conflict — two nets exceeding
  a triangulation edge's capacity, or violating clearance. A constraint is "net *i* may not use
  face *f*". The low level is a constrained funnel search. Use the bounded-suboptimal variant
  (focal search) so the guarantee degrades gracefully instead of vanishing.
- **S10 — simulated annealing over topology.** A move flips which side of an obstacle a net
  passes, or reroutes one channel. Energy is `J_lite`. Geometry is recomputed *exactly* by the
  funnel after every move — which is what makes searching topology cheap enough to be worth it.

**Exit:** **E3** (topology search vs net-order search, matched on evaluation count) and **E4**
(the optimality gap) both reported, with S9 producing a certified bound on at least the D3-A
small boards.

**Expected failure:** S9 will not scale past small boards. That is by design — it is a measuring
instrument for E4, not the product. Time-box the low-level search and report the gap at
termination rather than pretending it converged.

---

### P5 — The long tail (~2 weeks total, not each)

**S11** (RRT*/PRM per net), **S12** (Eikonal/FM2 cost field), **S13** (MCTS over net order and
topology). Each gets a fixed time box. Each either enters the Pareto front of (CP, WL_ratio, Via)
or is written up in `results/NEGATIVE.md` and retired. No exceptions and no extensions — the
value here is in cheaply eliminating whole families of approach, not in rescuing one.

---

### P6 — Ship the MVP (~2 weeks)

**Build:** the winning strategy wired to the IPC API as a KiCad 9 plugin — `plugin.json` manifest,
`kipy` connection via `KICAD_API_SOCKET` / `KICAD_API_TOKEN`, all geometry pushed inside a single
`begin_commit` / `push_commit` pair so the whole route is one undo step. Progress reporting,
cancellation, and partial-result commit on cancel. Install docs and a demo board.

**Exit:** CP ≥ 0.80 on D3-A, and a human can open a board in KiCad, click one button, and get a
routed board that passes DRC unmodified.

**Note on effort:** the plugin is roughly a hundred lines. `emit.py` already produces the
geometry; the plugin is an alternate output adapter walking the same `Solution` object. This is
exactly why it is last — building it earlier would force every experiment to run against a GUI
and buy nothing.

---

## 7. Risks, in order of how much they would hurt

1. **The any-angle prize is real but small (E1 comes back near zero).** Likelihood: moderate —
   the λ-geometry derivation in SYNTHESIS Appendix A already predicts only ~4–5% over 45°.
   Mitigation: this is not actually a project-killer, it is a redirection. The response is to
   pivot fully onto topology search, where the modelled upside is 3–10×. E1 running in P3 rather
   than P5 is what makes that pivot cheap.
2. **KiCad's DRC disagrees with itself on arcs.** The rounding artifacts are documented. The ε
   guardband handles small ones; systematic disagreement would mean falling back to
   fine-grained polyline approximation of arcs at emission, losing exactness but keeping
   any-angle. Level 1's differential test is what tells us which world we are in, and it runs
   in P2, before we depend on the answer.
3. **PCBWorld's code is not actually released.** The paper is recent (July 2026) and the
   repository URL is not stated in it. Mitigation: PCBench (164 boards, MIT, on GitHub) is
   confirmed available and is the source of PCBWorld's own D3 boards, so the board data is
   reachable either way; only the harness would need rebuilding, and we are building our own
   harness regardless.
4. **CBS does not scale past toy boards.** Near-certain — CBS is exponential in conflict count
   and real boards have thousands of nets. Mitigation: this is *by design*. S9's job is E4, the
   optimality gap on small instances. It is a measuring instrument, not the product.
5. **Pad access dominates the failure modes.** In IC routing, most DRVs originate at pins
   (`2020-Tao-of-PAO-pin-access-oracle-detailed-routing.pdf`); fine-pitch BGAs make PCB no
   different. Mitigation: instrument DRV *location* from run one. If failures cluster at pads,
   pin-access escape routing becomes its own subproblem and the plan gets an extra phase.
6. **The Steiner lower bound for WL_ratio is expensive or wrong.** GeoSteiner is exact but
   obstacle-oblivious, so the ratio is a lower bound on a lower bound. Acceptable — it only
   needs to be *consistent* across strategies, not tight.

---

## 8. The papers that matter for *this* MVP

The full 115-paper corpus is indexed in [`papers/INDEX.md`](papers/INDEX.md). These are the ones
you actually need to have read before writing the corresponding code. Read in phase order.

**P0–P1, the harness and baselines**
- `2026-PCBWorld-benchmark-environment-engine-grounded-PCB-design-automation.pdf` *(new)* —
  the benchmark, the eight metrics, the KiCad integration pattern, the baseline numbers. **Read
  this first.** It is the closest thing to a spec for our harness.
- `2026-3D-LineExplore-multilayer-PCB-geometric-routing.pdf` — current published PCB SOTA,
  reports >98% completion vs Freerouting/ELECTRA/DeepPCB. Our nearest competitor.
- `2018-ISPD-initial-detailed-routing-contest-slides.pdf` — how a routing benchmark is properly
  constructed and scored.

**P2, negotiated congestion and rip-up**
- folder `12-negotiated-congestion-fpga` — PathFinder. The present + historical cost formulation
  is short and you should implement it directly from the paper.
- `2020-Kahng-TritonRoute-open-source-detailed-router.pdf` — search-and-repair, and the honest
  accounting of what a real DRC engine costs. Read for architecture even though our DRC is
  KiCad's.
- `1995-Contour-tile-based-gridless-router-DEC-WRL.pdf` — what gridless actually costs, from
  people who shipped it.

**P3, geometry — the CDT substrate and rubber-band relaxation**
- `1986-Maley-compaction-with-automatic-jog-introduction-MIT.pdf` — the rubber-band-equivalent
  data structure, from the source.
- `2017-constrained-routing-between-non-visible-vertices.pdf` — routing on constrained
  triangulations with any-angle spanning guarantees. The substrate, formally.
- `2011-computing-shortest-paths-among-curved-obstacles-plane.pdf` — shortest paths when
  obstacles are curved, which is what clearance-inflated pads are.
- `2017-Mitchell-shortest-paths-and-networks-handbook-DCG-ch31.pdf` — the map of the whole
  territory; keep it open.
- `2007-Litvinova-Garkushin-topological-routing-PCB-connections.pdf` and the two TopoR pieces in
  folder `06` — the prior art actually shipping this architecture (SYNTHESIS §3.2).

**P4, topology search**
- folder `07-any-angle-continuous`: the CBS papers. Specifically the base CBS paper, the
  continuous-time CBS + optimal any-angle low-level solver (this is our S9, published under
  another name), and `2020-MAPF-spatially-extended-agents.pdf` — because wires have width and
  every naive MAPF formulation forgets that.
- `1988-Schrijver-homotopic-routing` (folder `01`) — the theorem that makes the whole
  topology/geometry split legitimate: disjoint paths homotopic to given paths are polynomial.
- `2021-CBS-framework-multi-objective-multi-agent-path-finding.pdf` — Pareto fronts from CBS,
  which is how `J_lite`'s via-weight sweep should be computed rather than by re-running.
- `2014-GeoSteiner-software-package-computational-study.pdf` — exact Steiner trees, for the
  WL_ratio denominator.

**P5, the weird ones**
- folder `09-physics-inspired` — Fast Marching / FM2 for S12.
- `2025-passage-traversing-optimal-path-planning-sampling.pdf` — sampling-based planning that
  reasons about passages between obstacles; the most promising framing for S11.
- `2013-unsolvability-of-weighted-region-shortest-path-problem.pdf` — read before believing any
  exactness claim about S12. Weighted-region shortest path is provably unsolvable exactly; (1+ε)
  is the ceiling.

**Bounding the claims — read before writing "optimal" in a README**
- `Hanan-1966` (folder `03`) — why the grid was ever a good idea.
- `2014-history-of-the-euclidean-steiner-tree-problem.pdf` — why Euclidean SMT is not known to
  be in NP.
- `2007-Reif-Wang-bounded-curvature-NP-hard` (folder `07`) — minimum-bend-radius routing is
  NP-hard, so every curvature-feasible router is approximating.

Paywalled foundations worth obtaining through Purdue's library, in priority order: Leiserson &
Maley, *Algorithms for routing and testing routability of planar VLSI layouts*, STOC 1985; and
Dayan's 1997 UCSC thesis on rubber-band routing. Both are cited in INDEX §16.

---

## 9. Repository layout

```
optimal-prime/
  SYNTHESIS.md                 # the research synthesis
  MVP-PLAN.md                  # this document
  papers/                      # the corpus + INDEX.md
  arena/                       # the harness — no routing logic lives here
    problem.py                 # .kicad_pcb -> Problem IR
    emit.py                    # Solution -> .kicad_pcb  (arc-native)
    oracle.py                  # kicad-cli drc wrapper, version-pinned
    score.py                   # CP / Rout. / DRV / WL_ratio / Via / J_lite
    geometry.py                # exact segment+arc predicates (Level 0 tested)
    runner.py                  # strategy x board x seed matrix, parallel, resumable
    registry.py                # strategy plugin discovery
  strategies/                  # one file per S-number, all implementing Strategy
  boards/                      # fetched benchmark instances (not committed)
  results/
    runs/<utc>/<strategy>/<board>/<seed>.json
    NEGATIVE.md                # retired strategies and why
  tests/                       # levels 0-3
  plugin/                      # P6: the kipy KiCad plugin
```

**Stack.** Python 3.11+; `shapely` 2.x for polygon work; `triangle` or `CGAL` bindings for the
CDT; `networkx` for graph search; `hypothesis` for property tests; `numpy` throughout. No Rust,
no C++ — compute time is not a goal and a rewrite is cheap once we know which strategy won.
Deliberately resist optimizing anything before P6.

---

## 10. The one-paragraph version

Build the measurement harness before the router, because KiCad already supplies the two hard
pieces — a production DRC engine that understands arcs (`kicad-cli pcb drc --format json`) and,
via PCBWorld and PCBench, a native benchmark of hundreds of real open-source boards with
published baselines. Freeze placement, stackup, via type, and differential pairs so that
geometry and topology are the only variables. Then run a tournament: a random-order control to
establish the floor, Freerouting to establish the bar, PathFinder to prove the harness measures
something, and the constrained-Delaunay-plus-rubber-band architecture from SYNTHESIS §7 as the
challenger — with a CBS variant kept alive purely to report the optimality gap nobody else
reports. Run the any-angle ablation early, because it is the experiment that can falsify the
premise of this whole repository, and finding that out in month two is worth more than any
result found in month six.
