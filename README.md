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
A connection that still cannot fit rips up whatever is blocking it and hands the displaced
bands back to the queue, so nothing is dropped merely for arriving last.
A `.kicad_pcb` holds exactly two kinds of copper, `segment` and `arc`, so the optimal geometry
and the expressible geometry are the same set. Nothing is snapped, rounded, or flattened on
the way out — the file holds the answer itself.

## Results

Two runs on the same KiCad demo board, judged entirely by KiCad's own DRC.

| board | layers | connections | arcs | copper | DRC errors | unconnected | time |
|---|---|---|---|---|---|---|---|
| `ecc83-pp` | F.Cu | **20 / 20** | 14 | 292.9 mm | **0** | **0** | 9 s |
| `ecc83-pp` | F.Cu + B.Cu | **20 / 20** | 5 | 239.5 mm | **0** | **0** | 16 s |
| `sonde xilinx` | F.Cu + B.Cu | **66 / 66** | 46 | 785.5 mm | **0** | **0** | 343 s |

Every board fully routed, every board DRC-clean by KiCad's own check.

### How it got there

Three schemes on the same boards, each one fixing what the last could not do:

| board | one at a time | relaxed together | + rip-up |
|---|---|---|---|
| `ecc83-pp` F.Cu | 19 / 20 | 20 / 20 | 20 / 20 |
| `ecc83-pp` F.Cu+B.Cu | 20 / 20 | 20 / 20 | 20 / 20 |
| `sonde xilinx` | 63 / 66 | 64 / 66 | **66 / 66** |

Relaxing the bands together recovers the detours one-at-a-time routing creates. Rip-up covers
the last case: a connection that cannot fit takes the space from whatever is sitting on it and
hands the displaced connections back to the queue. Without it, the last net placed gets
whatever is left, and if that is nothing it is dropped — even when a path plainly exists and
another net is merely sitting on it.

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
- **It is not guaranteed to converge.** It happens to finish every connection on all three
  boards, but nothing about the method promises that, and there is no lower bound to say how
  far the result sits from the best possible arrangement.
- **Rip-up is budgeted, not principled.** Forty rip-ups per board stops two connections
  trading the same space forever; it is not a convergence argument.

None of these can produce an illegal board — they cost completed connections, never
correctness. When no legal path exists the router says so instead of laying copper anyway.
