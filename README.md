# Optimal Prime

A taut-string autorouter for KiCad.

Stretch a rubber band between two pads and let it pull tight around whatever is in the way.
The shape it settles into is made of straight lines and circular arcs, and nothing else — no
grid, no preferred direction, no 45-degree rule. Those are conventions for human legibility,
and the copper does not need them.

## The idea in one paragraph

Every obstacle — a pad, a finished track, the board edge — is treated as something round to
go around. The shortest path avoiding a set of circles is a theorem, not a heuristic: it is a
sequence of straight lines tangent to those circles, joined by arcs riding on their surfaces.
A `.kicad_pcb` holds exactly two kinds of copper, `segment` and `arc`, so the optimal geometry
and the expressible geometry are the same set. Nothing is snapped, rounded, or flattened on
the way out — the file holds the answer itself.

## Results

Two runs on the same KiCad demo board, judged entirely by KiCad's own DRC.

| board | layers | connections | arcs | copper | DRC errors | unconnected | time |
|---|---|---|---|---|---|---|---|
| `ecc83-pp` | F.Cu | 17 / 20 | 21 | 289.1 mm | **0** | 3 | 33 s |
| `ecc83-pp` | F.Cu + B.Cu | **20 / 20** | 5 | 240.2 mm | **0** | **0** | 5 s |
| `test_pads_inside_pads` | F.Cu | 12 / 12 | 0 | 67.6 mm | **0** | **0** | <1 s |

The three unrouted nets in the first run are not a bug: `ecc83-pp` is a two-layer board and
one layer is not enough for it. The second run is the same board with the back copper allowed,
and it completes.

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
| [`taut/tangent.py`](taut/tangent.py) | **The algorithm.** Tangent graph over discs, Dijkstra, out come lines and arcs. |
| [`taut/obstacles.py`](taut/obstacles.py) | Pads, tracks and board edges reduced to keep-out discs. |
| [`taut/board.py`](taut/board.py) | Minimal `.kicad_pcb` reader — pads, layers, netclasses, outline. |
| [`taut/route.py`](taut/route.py) | Routes a whole board, one connection at a time. |
| [`taut/emit.py`](taut/emit.py) | Writes copper back as native `segment` and `arc`. |
| [`taut/geometry.py`](taut/geometry.py) | Exact distance predicates for segments and arcs. |
| [`run.py`](run.py) | Route a demo board, run DRC, render an SVG. |
| [`SYNTHESIS.md`](SYNTHESIS.md) | The research this came out of: 116 papers on automatic routing. |

## What it does not do yet

- **No vias.** A connection goes on whichever allowed layer is shorter, but never changes
  layer part-way. Two surface pads on opposite faces have no solution.
- **Obstacles are circles.** A long rectangular pad is treated as its enclosing circle, which
  wastes room and occasionally refuses a connection that would fit. Rounded rectangles are the
  honest next step.
- **Nets are routed one after another**, each finished track becoming an obstacle for the
  next. Nothing negotiates, reroutes, or reconsiders an order.

None of these can produce an illegal board — they cost completed connections, never
correctness. When no legal path exists the router says so instead of laying copper anyway.
