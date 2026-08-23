"""Regenerate the leaderboard and the E2 ordering analysis from run records.

The leaderboard is a *view*. Everything is recomputed from ``results/runs/<tag>/**.json``,
so there is no second database that can drift out of sync with the runs it describes.

Reporting rule from MVP-PLAN section 5: per-board paired deltas, never aggregate means
alone. A mean Clean Pass across boards of wildly different difficulty is close to
meaningless, so per-board numbers are always printed alongside any total.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def load_runs(tag: str) -> list[dict]:
    directory = ROOT / "results" / "runs" / tag
    if not directory.exists():
        raise SystemExit(f"no runs at {directory}")
    # Match only run records: a sweep in progress also leaves transient seedNNNN.drc.json
    # files beside them, and globbing "seed*.json" swallows those too.
    records = [p for p in sorted(directory.rglob("seed*.json"))
               if not p.name.endswith(".drc.json")]
    out = []
    for path in records:
        try:
            out.append(json.loads(path.read_text(encoding="utf-8")))
        except (FileNotFoundError, json.JSONDecodeError):
            continue  # a cell still being written by a concurrent sweep
    return out


def _ok(runs: list[dict]) -> list[dict]:
    return [r for r in runs if r.get("status") == "ok"]


def leaderboard(runs: list[dict]) -> None:
    by_strategy: dict[str, list[dict]] = defaultdict(list)
    for run in runs:
        by_strategy[run["spec"]["strategy"]].append(run)

    print("\n=== LEADERBOARD " + "=" * 74)
    print(f"{'strategy':10s} {'cells':>6s} {'ok':>5s} {'CP':>7s} {'rout':>7s} "
          f"{'DRV med':>8s} {'WL ratio':>9s} {'vias':>7s} {'time':>8s}")
    for name in sorted(by_strategy):
        rows = by_strategy[name]
        good = _ok(rows)
        if not good:
            print(f"{name:10s} {len(rows):6d} {0:5d}   (no successful cells)")
            continue
        metrics = [r["metrics"] for r in good]
        print(f"{name:10s} {len(rows):6d} {len(good):5d} "
              f"{statistics.mean(m['cp'] for m in metrics):7.3f} "
              f"{statistics.mean(m['rout'] for m in metrics):7.3f} "
              f"{statistics.median(m['drv'] for m in metrics):8.1f} "
              f"{statistics.median(m['wl_ratio'] for m in metrics):9.2f} "
              f"{statistics.median(m['vias'] for m in metrics):7.1f} "
              f"{statistics.mean(m['time_s'] for m in metrics):7.2f}s")

    failures = [r for r in runs if r.get("status") != "ok"]
    if failures:
        kinds: dict[str, int] = defaultdict(int)
        for run in failures:
            kinds[f"{run['spec']['strategy']}/{run['status']}"] += 1
        print("\nnon-ok cells: " + ", ".join(f"{k}={v}" for k, v in sorted(kinds.items())))


def per_board(runs: list[dict], strategies: list[str]) -> None:
    print("\n=== PER BOARD (median over seeds) " + "=" * 56)
    boards = sorted({r["spec"]["board"] for r in runs})
    header = f"{'board':30s} {'nets':>5s}"
    for name in strategies:
        header += f" | {name + ' CP':>7s} {name + ' DRV':>8s} {name + ' WL':>9s}"
    print(header)

    for board in boards:
        rows = [r for r in _ok(runs) if r["spec"]["board"] == board]
        if not rows:
            continue
        line = f"{board[:30]:30s} {rows[0]['board']['nets']:5d}"
        for name in strategies:
            subset = [r["metrics"] for r in rows if r["spec"]["strategy"] == name]
            if not subset:
                line += f" | {'-':>7s} {'-':>8s} {'-':>9s}"
                continue
            line += (f" | {statistics.mean(m['cp'] for m in subset):7.2f}"
                     f" {statistics.median(m['drv'] for m in subset):8.1f}"
                     f" {statistics.median(m['wl_mm'] for m in subset):9.1f}")
        print(line)


def experiment_e2(runs: list[dict], strategy: str = "S1") -> None:
    """E2 -- how much of the routing problem is net ordering and nothing else.

    S1 is deterministic given a net order, so the entire spread across seeds is attributable
    to the order alone. The width of that distribution is the ceiling available to any method
    that only reorders nets, and therefore how much of a later win must come from elsewhere.
    """
    print(f"\n=== E2: NET-ORDER SENSITIVITY ({strategy}) " + "=" * 46)
    rows = [r for r in _ok(runs) if r["spec"]["strategy"] == strategy]
    by_board: dict[str, list[dict]] = defaultdict(list)
    for run in rows:
        by_board[run["spec"]["board"]].append(run["metrics"])

    print(f"{'board':28s} {'seeds':>5s} {'CP rate':>8s} | "
          f"{'DRV best':>8s} {'DRV med':>8s} {'DRV worst':>9s} {'spread':>7s} | "
          f"{'WL best':>9s} {'WL worst':>9s} {'WL %':>6s}")

    spreads = []
    for board in sorted(by_board):
        metrics = by_board[board]
        if len(metrics) < 2:
            continue
        drv = sorted(m["drv"] for m in metrics)
        wl = sorted(m["wl_mm"] for m in metrics)
        cp_rate = statistics.mean(m["cp"] for m in metrics)
        spread = (drv[-1] - drv[0]) / max(1, statistics.median(drv))
        wl_pct = (wl[-1] - wl[0]) / wl[0] * 100 if wl[0] > 0 else 0.0
        spreads.append((board, cp_rate, drv, wl_pct, spread))
        print(f"{board[:28]:28s} {len(metrics):5d} {cp_rate:8.2f} | "
              f"{drv[0]:8d} {statistics.median(drv):8.1f} {drv[-1]:9d} {spread:6.1%} | "
              f"{wl[0]:9.1f} {wl[-1]:9.1f} {wl_pct:5.1f}%")

    if not spreads:
        print("  (need at least two seeds per board)")
        return

    decided = [s for s in spreads if 0 < s[1] < 1]
    print(f"\n  boards where net order alone decides Clean Pass: "
          f"{len(decided)}/{len(spreads)}")
    for board, cp_rate, *_ in decided:
        print(f"    {board}: passes on {cp_rate:.0%} of orders")
    print(f"  median DRV spread across orders: "
          f"{statistics.median(s[4] for s in spreads):.1%} of the median")
    print(f"  median wirelength spread:        "
          f"{statistics.median(s[3] for s in spreads):.1f}%")


def floor_and_bar(runs: list[dict]) -> None:
    """Paired per-board comparison of every strategy against the S0 floor."""
    print("\n=== PAIRED vs S0 (the floor) " + "=" * 61)
    by_key: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for run in _ok(runs):
        by_key[(run["spec"]["strategy"], run["spec"]["board"])].append(run["metrics"])

    strategies = sorted({k[0] for k in by_key} - {"S0"})
    for name in strategies:
        wins = losses = ties = 0
        ratios = []
        for (strategy, board), metrics in by_key.items():
            if strategy != name:
                continue
            floor = by_key.get(("S0", board))
            if not floor:
                continue
            ours = statistics.median(m["drv"] for m in metrics)
            theirs = statistics.median(m["drv"] for m in floor)
            wins += ours < theirs
            losses += ours > theirs
            ties += ours == theirs
            if theirs > 0:
                ratios.append(ours / theirs)
        if wins + losses + ties == 0:
            continue
        geo = math.exp(statistics.mean(math.log(max(r, 1e-9)) for r in ratios)) if ratios else 0
        print(f"  {name} vs S0 on DRV: {wins} better, {losses} worse, {ties} tied "
              f"({wins}/{wins + losses + ties} boards); geometric mean DRV ratio {geo:.3f}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tag", default="e2")
    parser.add_argument("--e2-strategy", default="S1")
    args = parser.parse_args()

    runs = load_runs(args.tag)
    strategies = sorted({r["spec"]["strategy"] for r in runs})
    print(f"{len(runs)} run records, {len(_ok(runs))} ok, strategies {strategies}")
    if runs:
        engines = {r.get("engine", {}).get("kicad") for r in _ok(runs)}
        print(f"engine: {engines or 'unknown'}  |  score_ver: "
              f"{ {r['score_ver'] for r in runs} }")

    leaderboard(runs)
    per_board(runs, strategies)
    floor_and_bar(runs)
    experiment_e2(runs, args.e2_strategy)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
