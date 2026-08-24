# Optimal Prime

A taut-string autorouter for KiCad.

Stretch a rubber band between two pads and let it pull tight around whatever is in the way.
The shape it settles into is made of straight lines and circular arcs, and nothing else — no
grid, no preferred direction, no 45-degree rule. Those are conventions for human legibility,
and the copper does not need them.

## The idea in one paragraph

Every obstacle — a pad, a copper label, the board edge — is a convex shape grown outward by
the clearance. The shortest path avoiding such shapes is a theorem, not a heuristic: straight
lines tangent to them, joined by arcs riding on circles at their corners.

The copper is then settled as one arrangement rather than a sequence of decisions. Every band
is re-solved against the *previous* positions of all the others and all of them move at once,
so two bands contending for the same space both give way instead of the first one keeping it.
A `.kicad_pcb` holds exactly two kinds of copper, `segment` and `arc`, so the optimal geometry
and the expressible geometry are the same set. Nothing is snapped, rounded, or flattened on
the way out — the file holds the answer itself.

## Results

Two runs on the same KiCad demo board, judged entirely by KiCad's own DRC.

| board | layers | connections | arcs | copper | DRC errors | unconnected | time |
|---|---|---|---|---|---|---|---|
| `ecc83-pp` | F.Cu | **20 / 20** | 14 | 292.9 mm | **0** | **0** | 9 s |
| `ecc83-pp` | F.Cu + B.Cu | **20 / 20** | 5 | 239.5 mm | **0** | **0** | 16 s |
| `sonde xilinx` | F.Cu + B.Cu | 64 / 66 | 33 | 678.4 mm | **0** | 2 | 335 s |

### One at a time, versus all together

Via `python scripts/compare.py`:

| board | scheme | connections | copper |
|---|---|---|---|
| `ecc83-pp` F.Cu | one at a time | 19 / 20 | 305.3 mm |
| | **all together** | **20 / 20** | **292.9 mm** |
| `ecc83-pp` F.Cu+B.Cu | one at a time | 20 / 20 | 239.5 mm |
| | all together | 20 / 20 | 239.5 mm |
| `sonde xilinx` | one at a time | 63 / 66 | 690.3 mm |
| | **all together** | **64 / 66** | **678.4 mm** |

Where a board has any headroom, relaxing the bands together wins on *both* counts — more
connections finished **and** less copper. That is the tell that it is doing the right thing: a
band that had been taking the long way round something now goes through instead, and the band
it used to detour around gives up a little of its own straightness to let it past. The
two-layer case is unchanged because there was nothing to recover.

Rendered results with the geometry visible: [`examples/results.html`](examples/results.html).

## Running it

```bash
python run.py --board ecc83-pp --layers F.Cu,B.Cu
python scripts/build_report.py          # regenerate the results page
python -m pytest tests/ -q              # 21 tests
```

Needs KiCad 9 (for `kicad-cli`), Python 3.11+, and numpy. Boards come from KiCad's own
installed demos. Set `KICAD_CLI` or `KICAD_DEMOS` if they are not in the default location.

## As a KiCad plugin

Copy [`plugin/`](plugin/) into `Documents/KiCad/9.0/plugins/`, enable the API server in
Preferences → Plugins, and a **Route with taut strings** button appears in the PCB editor.
The whole route lands as a single undo step.

## Layout

| Path | What it is |
|---|---|
| [`taut/tangent.py`](taut/tangent.py) | **The algorithm.** Tangent graph over the obstacles' corner circles, Dijkstra, out come lines and arcs. |
| [`taut/obstacles.py`](taut/obstacles.py) | Pads, tracks and copper graphics as convex shapes inflated by the clearance. |
| [`taut/board.py`](taut/board.py) | Minimal `.kicad_pcb` reader — pads, layers, netclasses, outline. |
| [`taut/relax.py`](taut/relax.py) | Settles every band at once, rather than one after another. |
| [`taut/route.py`](taut/route.py) | Routes a whole board: shape each band alone, relax the set, settle what is left. |
| [`taut/emit.py`](taut/emit.py) | Writes copper back as native `segment` and `arc`. |
| [`taut/geometry.py`](taut/geometry.py) | Exact distance predicates for segments and arcs. |
| [`run.py`](run.py) | Route a demo board, run DRC, render an SVG. |
| [`SYNTHESIS.md`](SYNTHESIS.md) | The research this came out of: 116 papers on automatic routing. |

## What it does not do yet

- **No vias.** A connection goes on whichever allowed layer is shorter, but never changes
  layer part-way. Two surface pads on opposite faces have no solution.
- **Relaxation is slow.** Each step re-solves every contended band against every other, on
  every permitted layer. `sonde xilinx` takes about 5 minutes against 1 minute for settling
  one at a time, and it runs under a wall-clock budget rather than to convergence.
- **It is not guaranteed to converge**, and does not always: 2 connections on `sonde xilinx`
  are still unrouted. Whether a different global arrangement would fit them, or whether they
  genuinely need a via, is not yet known.

None of these can produce an illegal board — they cost completed connections, never
correctness. When no legal path exists the router says so instead of laying copper anyway.
