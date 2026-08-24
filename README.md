# Optimal Prime

A taut-string autorouter for KiCad.

Stretch a rubber band between two pads and let it pull tight around whatever is in the way.
The shape it settles into is made of straight lines and circular arcs, and nothing else — no
grid, no preferred direction, no 45-degree rule. Those are conventions for human legibility,
and the copper does not need them.

## The idea in one paragraph

Every obstacle — a pad, a finished track, a copper label, the board edge — is a convex shape
grown outward by the clearance. The shortest path avoiding such shapes is a theorem, not a
heuristic: straight lines tangent to them, joined by arcs riding on circles at their corners.
A `.kicad_pcb` holds exactly two kinds of copper, `segment` and `arc`, so the optimal geometry
and the expressible geometry are the same set. Nothing is snapped, rounded, or flattened on
the way out — the file holds the answer itself.

## Results

Two runs on the same KiCad demo board, judged entirely by KiCad's own DRC.

| board | layers | connections | arcs | copper | DRC errors | unconnected | time |
|---|---|---|---|---|---|---|---|
| `ecc83-pp` | F.Cu | 19 / 20 | 12 | 305.3 mm | **0** | 1 | 2.6 s |
| `ecc83-pp` | F.Cu + B.Cu | **20 / 20** | 5 | 239.5 mm | **0** | **0** | 1.0 s |
| `sonde xilinx` | F.Cu + B.Cu | 63 / 66 | 40 | 697.8 mm | **0** | 3 | 63 s |

Nothing here is DRC-dirty. What varies is how much gets *finished*. When the router cannot
find a legal path it reports the connection as unrouted rather than laying copper anyway.

### Sequential versus bundle

Both schemes on the same boards, via `python scripts/compare.py`:

| board | scheme | connections | copper | DRC | unconnected |
|---|---|---|---|---|---|
| `ecc83-pp` F.Cu | sequential | 19 / 20 | 305.3 mm | 0 | 1 |
| | **bundle** | 19 / 20 | 305.3 mm | 0 | 1 |
| `ecc83-pp` F.Cu+B.Cu | sequential | 20 / 20 | 239.5 mm | 0 | 0 |
| | **bundle** | 20 / 20 | 239.5 mm | 0 | 0 |
| `sonde xilinx` | sequential | 63 / 66 | 690.3 mm | 0 | 3 |
| | **bundle** | 63 / 66 | 697.8 mm | 0 | 3 |

**The bundle changes nothing on these boards**, and that is worth stating plainly. The
mechanism is correct — `tests/test_bundle.py` proves it directly on the case it exists for:
a gap that holds two tracks, one net alone sitting in the middle of it, and the second unable
to fit until both are seated. None of these boards' remaining failures turn out to be that
case.

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
| [`taut/route.py`](taut/route.py) | Routes a whole board, one connection at a time. |
| [`taut/emit.py`](taut/emit.py) | Writes copper back as native `segment` and `arc`. |
| [`taut/geometry.py`](taut/geometry.py) | Exact distance predicates for segments and arcs. |
| [`run.py`](run.py) | Route a demo board, run DRC, render an SVG. |
| [`SYNTHESIS.md`](SYNTHESIS.md) | The research this came out of: 116 papers on automatic routing. |

## What it does not do yet

- **No vias.** A connection goes on whichever allowed layer is shorter, but never changes
  layer part-way. Two surface pads on opposite faces have no solution.
- **Contention is only modelled inside gaps between two static obstacles.** Two nets running
  alongside each other in open copper share no gap, so no slot assignment covers them and the
  settling pass falls back to routing one against the other. That is why the bundle does not
  yet move these boards.
- **What blocks the remaining connections is unresolved.** They fail during settling, not for
  want of a path in isolation, so they are contention failures — but whether a different
  global arrangement would fit them, or whether they genuinely need a via, is not yet known.

None of these can produce an illegal board — they cost completed connections, never
correctness. When no legal path exists the router says so instead of laying copper anyway.
