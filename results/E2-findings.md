# P1 results — the floor, and how much of routing is net ordering

*Engine: KiCad 9.0.7. Score version 1. 15 boards, 20 seeds, 600 cells, 0 crashes.
Regenerate with `python scripts/report.py --tag e2`; raw table in [`E2-report.txt`](E2-report.txt).*

---

## 1. The floor is real, and it makes everything else readable

| | S0 (uniform-random geometry) | S1 (random order + grid A*) |
|---|---|---|
| Clean Pass | 0.003 | 0.133 |
| median DRV | 456.5 | 42.5 |
| median WL / Steiner bound | **9.67×** | **1.37×** |
| median vias | 0 | 19 |

S1 beats the floor on **14 of 15 boards**, geometric-mean DRV ratio **0.011** — roughly
ninety times fewer violations. It routes within **37% of the Steiner lower bound**, against
the floor's 867% over.

This is what the floor is for. On its own, "S1 achieves Clean Pass 0.14" is uninterpretable.
Next to a floor of 0.003 and a wirelength ratio of 9.67, it is legible.

**`rout` turned out to be nearly useless as a discriminator.** S0 scores 0.739 on it —
straight lines drawn through everything really do connect the pads, they just short half the
board doing it. Only Clean Pass separates a routed board from a destroyed one. That is the
justification for making CP the headline and binary.

The one board S1 loses on is `custom_pads_test` (8 DRV vs 7), a 3-net board with custom pad
shapes that the IR approximates by their enclosing circle.

## 2. E2 — net ordering moves violations 50%, and wirelength barely at all

S1 is deterministic given a net order, so the entire spread across 20 seeds is attributable
to ordering alone.

| | median across boards | worst board |
|---|---|---|
| DRV spread (worst − best, as % of median) | **50.0%** | **140%** (`carte_test`, 30 → 126) |
| wirelength spread | **14.4%** | 51.6% (`interf_u`) |

Reordering nets changes how many rules you break by a factor of two, and on the worst board
by a factor of four. It changes how much copper you use by about a seventh.

**This is the empirical form of the SYNTHESIS §0.3 claim**, measured rather than argued: the
combinatorial choice dominates, and the geometric one is a rounding error next to it. It also
sets the bar for P2 — a negotiated-congestion router (S4) has to beat not S1's median but
S1's *best-of-20*, because taking the best of many random orders is nearly free when compute
time is not a goal.

**A negative result worth recording:** net order alone never flipped Clean Pass, on any of the
15 boards (`0/15`). Two boards pass on every order; thirteen fail on every order. At S1's
quality level, boards are either trivially passable or hopeless, and ordering decides only
*how badly* you fail. Whether that survives a better router is a P2 question.

## 3. What the geometry fixes cost, and why that is the right trade

Fixing the pad-halo and keepout bugs (commit `aeb7e04`) **raised** S1's violation count on
several boards — `StickHub` 136 → 312, `interf_u` 89 → 180 — while raising routability from
0.772 to 0.836.

That is the expected direction, not a regression. Before the fix the grid was heavily
over-blocked, so S1 simply refused to route large numbers of nets; unrouted nets generate no
clearance violations. Unblocking the board means it attempts more, and a router with no
rip-up and no negotiation necessarily creates more conflicts as it attempts more.

It is also a warning about DRV as a metric: **DRV alone rewards giving up.** It is only
meaningful read alongside `rout`, which is why the leaderboard prints both and why Clean Pass
— which requires full connectivity *and* zero violations — is the only headline that cannot
be gamed by refusing to work.

## 4. Bugs this measurement found

None of these were visible by reading the code. All were found by asking what DRC actually
complained about.

| Bug | Symptom | Impact |
|---|---|---|
| Stub segments joined a path to the *previously routed* pad rather than the pad owning the path's first cell | copper drawn straight across the board | largest single contributor |
| Grid pitch ignored diagonal adjacency | parallel diagonal tracks sit `pitch/√2` apart, not `pitch` | systematic clearance failures |
| Via footprint claimed one cell | a 1.6 mm via on a sub-mm grid overlaps its neighbours | via-adjacent violations |
| `netclass_patterns` vs `netclass_assignments` | KiCad 9 renamed it; every net silently got Default rules | POWER class routed at 0.5/0.25 mm instead of 0.8/0.28 mm |
| Filled copper pours retained | `kicad-cli` cannot refill zones, so any pour is stale the moment we re-route | **75%** of all violations |
| Pad halos used the enclosing circle | over-covers rectangular pads by up to √2; overlapping halos hard-block | 95% of `multichannel_mixer` blocked |
| Keepouts modelled as bounding circles | four slot keepouts became four 53.7 mm discs on a 110 mm board | board unroutable |
| Keepout flags ignored | copper-pour-only keepouts blocked tracks | 0 → 78/79 nets routed |

Cumulatively: 485 → 25 DRVs on `pic_programmer`, and `multichannel_mixer` from routing
nothing at all to routing 78 of 79 nets.

## 5. Caveats on these numbers

- **Filled pours are stripped at ingestion.** A pour is not a fixed obstacle — KiCad
  recomputes its fill around whatever tracks exist, and `kicad-cli` has no zone-fill command,
  so a retained pour keeps the fill computed for the *original* routing. Removing it makes
  each instance harder (GND now needs real copper: 2302 → 2775 connections) and well-posed
  rather than easier and wrong. Restoring pours needs zone refill over the IPC API.
- **The board set is KiCad's own demos**, not PCBench or PCBWorld. 15 boards, 2–4 layers,
  3–371 nets. Enough to measure a spread; not enough to claim a benchmark result.
- **S2 (Freerouting) has not run**, so the *bar* is still the published 0.80 rather than a
  number measured here. `kicad-cli` has no Specctra export, so the DSN must come from the GUI.
  Every comparison below is against the floor only.
- **WL_ratio's denominator is `(√3/2)·MST`**, a bound on a bound: obstacle-oblivious and
  layer-oblivious. Consistent across strategies, which is all it needs to be, but its absolute
  value should not be quoted as "37% above optimal".
- **`time_s` is not comparable across machines**, and is recorded rather than optimised.
  S1 spent 12,424 s of routing across 300 cells.
