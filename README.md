# Optimal Prime

Development and testing grounds for a next-generation autorouter — one that optimizes for
what is *physically* optimal rather than for what is legible to a human eye. No preferred
directions, no 45-degree dogma, no grid unless the grid earns its place.

## Current state: research phase

| Path | What it is |
|---|---|
| [`SYNTHESIS.md`](SYNTHESIS.md) | **Start here.** The full research synthesis: why routers look the way they do, what "optimal" can provably mean, a survey of every paradigm, the prior art, the cross-domain transplants worth making, and a proposed architecture. |
| [`MVP-PLAN.md`](MVP-PLAN.md) | **The current work.** MVP-01: a routing optimizer for KiCad with placement, stackup and via type frozen and differential pairs deferred. Scope, architecture, the strategy tournament, the six-level test plan, the experimental protocol, and the phase-by-phase reading list. |
| [`papers/INDEX.md`](papers/INDEX.md) | Annotated bibliography of all 116 archived papers, plus full citations for the paywalled works that could not be archived. |
| [`papers/`](papers/) | The corpus itself, 116 PDFs in 15 thematic folders. |

## The three-sentence version

Routing decomposes into topology selection (NP-hard, where all the difficulty lives),
geometric embedding (polynomial and **exactly** solvable — the answer is a taut rubber band,
which is any-angle for free), and physical refinement (continuous, (1+e) at best). Almost
every shipping router conflates the first two by searching in geometry space on a grid,
which throws away the exact result and buys nothing. The proposal is to search topology on a
constrained Delaunay triangulation using Conflict-Based Search from the multi-agent
path-finding literature, then let rubber-band relaxation produce provably optimal
segment-and-arc geometry that Gerber can already express.

One commercial tool — Eremex TopoR, in continuous development since 1988 — already ships the
triangulation-plus-topology-first half of that architecture. See SYNTHESIS section 3.2 for what
it does, the two ideas worth taking from it, and the gaps that leave room for this project.

Full argument, with the complexity results that bound the claim, in [`SYNTHESIS.md`](SYNTHESIS.md).

## What is being built first

MVP-01 is a single, falsifiable thing: given a `.kicad_pcb` with components already placed and
the stackup fixed, route every net so that KiCad's own DRC reports zero errors — with no
constraint that the geometry look human-drawn. Compute time is explicitly not a goal.

Two things turned out to already exist, and MVP-01 is built around exploiting both:
`kicad-cli pcb drc --format json` is a production DRC engine that understands arc tracks
natively, and PCBWorld/PCBench supply hundreds of real open-source boards in native
`.kicad_pcb` format with published baselines. So the harness comes first and the router second,
and the first experiment to run is the one that could falsify the premise of this repository.
See [`MVP-PLAN.md`](MVP-PLAN.md).

## Running the arena

```bash
python scripts/fetch_boards.py --source kicad-demos   # build the benchmark set
python -m pytest tests/ -q                            # 40+ tests, incl. the DRC oracle
python -c "from arena.runner import run_matrix; run_matrix(['S0','S1'], None, list(range(1,21)), tag='e2')"
python scripts/report.py --tag e2                     # leaderboard + E2
```

Requires KiCad 9 (for `kicad-cli`) and Python 3.11+. Set `OPTIMAL_PRIME_KICAD_CLI` if
`kicad-cli` is not on `PATH`.

| Path | What it is |
|---|---|
| `arena/` | The measurement harness. No routing logic lives here. |
| `strategies/` | One file per strategy, all implementing the same one-method interface. |
| `scripts/fetch_boards.py` | Board ingestion: filter, normalise, strip, baseline, hash. |
| `scripts/report.py` | Regenerates the leaderboard from run records. It is a view, not a file. |
| `boards/` | Fetched benchmark instances (gitignored except the manifest). |
| `results/runs/` | One self-describing JSON per (strategy, board, seed). |

## Notes

- `papers/` is ~318 MB of PDFs. If this repository is pushed anywhere, put that directory
  behind Git LFS or exclude it — see `.gitignore`.
- Papers were collected from open-access sources only (arXiv, institutional repositories,
  author pages, open-access journals, vendor and trade publications). Paywalled works are
  cited but not redistributed; see section 16 of [`papers/INDEX.md`](papers/INDEX.md).
