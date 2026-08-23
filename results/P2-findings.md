# P2 results — a real router, and the checker that makes it possible

*Engine: KiCad 9.0.7. Score version 1. Regenerate with `python scripts/report.py --tag p2`;
raw table in [`P2-report.txt`](P2-report.txt). 79 tests pass.*

---

## 1. S4 clears the bar E2 set

| | S0 (floor) | S1 (random order + grid A*) | **S4 (PathFinder)** |
|---|---|---|---|
| Clean Pass | 0.000 | 0.133 | **0.182** |
| routability | 0.736 | 0.839 | **0.993** |
| median DRV | 457 | 40 | **5** |
| vs the floor | — | 14/15 boards, ratio 0.012 | **11/11 boards, ratio 0.002** |
| median time | 0.00 s | 31 s | 66 s |

Per board, median DRV over seeds:

| board | nets | S1 | S4 | |
|---|---|---|---|---|
| `complex_hierarchy` | 50 | 25 | **2** | 12× |
| `multichannel_mixer` | 79 | 128 | **12** | 11× |
| `interf_u` | 110 | 186 | **16** | 12× |
| `sonde xilinx` | 26 | 40 | **7** | 6× |
| `flat_hierarchy` | 34 | 24 | **6** | 4× |
| `custom_pads_test` | 3 | 8 | **2** | 4× |
| `pic_programmer` | 34 | 26 | **9** | 3× |
| `tinytapeout-demo` | 108 | 230 | **71** | 3× |
| `ecc83-pp` | 9 | 0 | 0 | both clean |
| `ecc83-pp_v2` | 9 | **4** | 5 | S1 |
| `carte_test` | 83 | **76** | 103 | S1 |
| `StickHub` | 45 | **338** | 492 | S1 |

**The bar E2 set was S1's *best-of-20*, not its median** — taking the best of many random
orders is nearly free when compute time is not a goal. S4 clears it on every board where it
converges: `pic_programmer` 9 vs 14, `sonde xilinx` 7 vs 20, `complex_hierarchy` 2 vs 17,
`flat_hierarchy` 6 vs 17, `multichannel_mixer` 12 vs 90, `interf_u` 16 vs 125.

**The two losses are the DRV-rewards-giving-up effect again.** On `StickHub` S4 routes 94% of
connections against S1's 69%, and on `carte_test` 99% against 96%. A router that attempts more
in congested space generates more conflicts. Read `drv` next to `rout` or it lies.

## 2. Negotiation halves order-sensitivity — which is the point

E2's headline was that net ordering swings violation count by a median 50%. PathFinder exists
to make that stop mattering, and it does. Absolute DRV spread across seeds:

| | S1 | S4 |
|---|---|---|
| mean absolute spread | 30.0 | **13.3** |
| median absolute spread | 12 | **8** |

Better on 9 of 13 boards, worse on one, tied on three (all at zero). `interf_u` goes from a
151–204 range to 8–16; `multichannel_mixer` from 89–144 to 11–28.

*(Relative spread appears to rise, but that is a normalisation artifact: when the median drops
from 25 to 2, a swing of ±2 violations reads as 200%. Absolute spread is the honest measure.)*

## 3. The bug that made S4 lose, and what it taught

The first version of S4 **lost to S1** — exactly the tripwire MVP-PLAN puts at P2's exit. The
plan says that means the harness is broken. This time it did not; the algorithm was.

The cause was the choice of *negotiated resource*. PathFinder prices a shared resource until
nets stop wanting it, and I priced the grid **cell centre**. But two nets in adjacent cells
register zero contention while their copper sits at the clearance limit, so the algorithm
converged happily onto boards full of violations. S1 accidentally avoided this by stamping a
track's whole inflated halo as occupied.

Making the resource a track's **clearance envelope** — so that "contested" and "violates
clearance" mean the same thing — turned an 8× loss into an 8× win with no other change.

**The general lesson: in negotiated congestion, the resource has to be the thing the rule is
actually about.** Get that wrong and the algorithm optimises a proxy perfectly.

## 4. The Level 1 differential test holds

Zero unsafe disagreements between our clearance checker and KiCad, over 40 probe trials and
24 in the committed test, half of them deliberately marginal (parallel neighbours placed
within half a clearance of the limit). We are consistently 2–3× stricter, which is the allowed
direction: it costs completions, never correctness.

A companion test asserts a genuinely clean layout is clean on both sides — without it, a
checker that flagged everything would satisfy the one-directional contract while being useless.

Three of these tests failed on first run because their *fixtures* laid tracks across existing
pads. The checker was right and the tests were wrong.

## 5. Where S4 stops

**It does not scale past ~80 nets within a 420 s budget.** On `interf_u` (110 nets) it
completes 9.5% of connections before the wall clock; on `tinytapeout-demo` (108 nets), 0–31%.
The numbers quoted for those boards in §1 come from runs that hit the cap, so they measure a
partial route and should not be read as converged results.

Rerouting only nets involved in contention — standard PathFinder practice — was implemented and
**barely helped**: on `interf_u`, 96 of 110 nets are contested every iteration, so the reroute
set never shrinks. Contention on these boards is board-wide, not local.

The likely cause is over-reservation. The clearance envelope is currently a disc of one grid
cell, but the true envelope radius is ~0.44 cells at the chosen pitch, so S4 reserves roughly
twice the copper it needs and manufactures contention that geometry does not require. Fixing
that means decoupling the congestion resolution from the routing pitch, which is a real change
rather than a tuning knob.

**Nothing converged.** `converged=False` on every board above `ecc83-pp`; S4 runs out its 24
iterations with contested nodes remaining. The results above are what an unconverged
PathFinder produces, and are correspondingly a floor on what it could do.

## 6. What P2 did not deliver

- **S5** (adaptive rip-up) is not implemented. S4 subsumed enough of it to make it low priority
  against the scaling problem.
- **The bar is still unmeasured.** S2 needs a Specctra DSN per board and `kicad-cli` cannot
  export one, so Freerouting's 0.80 remains a published figure rather than one measured here.
- **Clean Pass is still barely discriminating.** Only 2 of 15 boards ever pass, the same two as
  in P1. S4 moved CP from 0.133 to 0.182, but the gain is a bigger share of the *same* easy
  boards, not a new board cracked. Until a board flips, CP cannot separate strategies and DRV
  is doing the work — which is exactly the metric that rewards giving up.
