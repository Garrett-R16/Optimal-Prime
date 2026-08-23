"""The comparison machinery.

P0 answered "can we score *one* run?". This answers "can we score *many* and rank them?".
Everything from P2 onward is a new entrant fed into this unchanged; the comparison never
moves, only the field of entrants grows.

Three properties matter more than speed:

* **crash isolation** -- one strategy segfaulting or looping must cost one cell, not the
  matrix;
* **resumability** -- re-running skips completed cells, so an interrupted sweep continues
  rather than restarting;
* **self-describing records** -- the leaderboard is regenerated from the run files alone, so
  there is no second database that can drift out of sync with them.
"""

from __future__ import annotations

import json
import os
import platform
import subprocess
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from random import Random

from . import oracle, strategy as strategy_mod
from .emit import write_solution
from .problem import BoardLoadError, load_problem
from .score import SCORE_VERSION, Score, score_solution
from .oracle import DrcResult, Violation

__all__ = ["RunSpec", "run_one", "run_matrix", "load_manifest", "results_dir"]

ROOT = Path(__file__).resolve().parent.parent
RESULTS = ROOT / "results" / "runs"


@dataclass(frozen=True)
class RunSpec:
    strategy: str
    board: str
    seed: int

    def relative_path(self) -> Path:
        return Path(self.strategy) / self.board / f"seed{self.seed:04d}.json"


def load_manifest(path: Path | None = None) -> dict:
    path = path or ROOT / "boards" / "manifest.json"
    if not path.exists():
        raise FileNotFoundError(
            f"no board manifest at {path}. Run: python scripts/fetch_boards.py --source kicad-demos"
        )
    return json.loads(path.read_text(encoding="utf-8"))


def results_dir(tag: str) -> Path:
    directory = RESULTS / tag
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def _load_strategies() -> None:
    """Import the strategy package so the registry is populated.

    Called inside every worker: a ProcessPoolExecutor child starts from a fresh interpreter
    and does not inherit whatever the parent happened to import.
    """
    if strategy_mod.REGISTRY:
        return
    import importlib
    importlib.import_module("strategies")


def _git_commit() -> str:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT,
                             capture_output=True, text=True, timeout=30)
        return out.stdout.strip() or "unknown"
    except (subprocess.SubprocessError, OSError):
        return "unknown"


def _baseline_from_manifest(entry: dict) -> DrcResult:
    """Rebuild the bare board's DRC result from the manifest, avoiding a re-run per cell."""
    payload = entry["baseline"]
    violations = tuple(
        Violation(
            type=v["type"], severity=v["severity"], description=v["description"],
            positions=tuple(tuple(p) for p in v["positions"]), uuids=tuple(v["uuids"]),
        )
        for v in payload["violations"]
    )
    unconnected = tuple(
        Violation(type="unconnected_items", severity="error", description="")
        for _ in range(payload["unconnected"])
    )
    return DrcResult(kicad_version=payload["kicad_version"], violations=violations,
                     unconnected=unconnected, source=entry["name"], raw=None)


def run_one(spec: RunSpec, entry: dict, out_dir: Path, budget_s: float = 3600.0,
            keep_board: bool = True) -> dict:
    """Execute one (strategy, board, seed) cell and write its self-describing record."""
    record: dict = {
        "spec": {"strategy": spec.strategy, "board": spec.board, "seed": spec.seed},
        "board": {
            "name": entry["name"],
            "sha256": entry["board_sha256"],
            "rules_sha256": entry["rules_sha256"],
            "nets": entry["nets"],
            "pads": entry["pads"],
            "layers": entry["layers"],
        },
        "strategy": {"name": spec.strategy, "git": _git_commit()},
        "score_ver": SCORE_VERSION,
        "host": {
            "os": platform.platform(),
            "python": platform.python_version(),
            "cores": os.cpu_count(),
        },
    }

    target = out_dir / spec.relative_path()
    target.parent.mkdir(parents=True, exist_ok=True)

    try:
        _load_strategies()
        problem = load_problem(ROOT / entry["path"], ROOT / entry["project"])
        impl = strategy_mod.get(spec.strategy)
        budget = strategy_mod.Budget(wall_clock_s=budget_s)

        started = time.monotonic()
        solution = impl.route(problem, Random(spec.seed), budget)
        elapsed = time.monotonic() - started

        board_out = target.with_suffix(".kicad_pcb")
        write_solution(problem, solution, board_out, strategy=spec.strategy, seed=spec.seed)

        drc = oracle.run_drc(board_out)
        baseline = _baseline_from_manifest(entry)
        scored: Score = score_solution(problem, solution, drc, baseline, elapsed)

        record["engine"] = {"kicad": drc.kicad_version}
        record["metrics"] = scored.to_dict()
        record["solution"] = solution.stats()
        record["meta"] = {k: v for k, v in solution.meta.items() if k != "net_order"}
        record["net_order"] = solution.meta.get("net_order")
        record["status"] = "ok"
        record["artifacts"] = {"board_out": board_out.name if keep_board else None}

        if not keep_board:
            board_out.unlink(missing_ok=True)
            board_out.with_suffix(".kicad_pro").unlink(missing_ok=True)
        board_out.with_suffix(".drc.json").unlink(missing_ok=True)

    except strategy_mod.Unavailable as exc:
        record["status"] = "unavailable"
        record["reason"] = str(exc)
    except (BoardLoadError, oracle.OracleError) as exc:
        record["status"] = "error"
        record["reason"] = f"{type(exc).__name__}: {exc}"
    except Exception as exc:  # a strategy bug must cost one cell, not the matrix
        record["status"] = "crash"
        record["reason"] = f"{type(exc).__name__}: {exc}"
        record["traceback"] = traceback.format_exc()[-4000:]

    target.write_text(json.dumps(record, indent=2), encoding="utf-8")
    return record


def _worker(args) -> dict:
    spec, entry, out_dir, budget_s, keep_board = args
    return run_one(spec, entry, Path(out_dir), budget_s, keep_board)


def run_matrix(strategies: list[str], boards: list[str] | None, seeds: list[int],
               tag: str, workers: int | None = None, budget_s: float = 3600.0,
               resume: bool = True, keep_board: bool = False,
               manifest: dict | None = None) -> list[dict]:
    """Run the full strategy x board x seed matrix."""
    _load_strategies()
    manifest = manifest or load_manifest()
    entries = {b["name"]: b for b in manifest["boards"]}
    chosen = boards or sorted(entries)
    missing = [name for name in chosen if name not in entries]
    if missing:
        raise KeyError(f"boards not in manifest: {missing}")

    out_dir = results_dir(tag)
    specs: list[RunSpec] = []
    for strategy_name in strategies:
        for board_name in chosen:
            for seed in seeds:
                spec = RunSpec(strategy_name, board_name, seed)
                if resume and (out_dir / spec.relative_path()).exists():
                    continue
                specs.append(spec)

    if not specs:
        print("nothing to do (all cells already present; pass resume=False to redo)")
        return []

    print(f"{len(specs)} cells -> {out_dir}")
    payload = [(s, entries[s.board], str(out_dir), budget_s, keep_board) for s in specs]

    records: list[dict] = []
    workers = workers or max(1, (os.cpu_count() or 4) - 2)
    if workers == 1:
        for item in payload:
            records.append(_worker(item))
            _report(records[-1], len(records), len(specs))
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futures = {pool.submit(_worker, item): item[0] for item in payload}
            for done in as_completed(futures):
                records.append(done.result())
                _report(records[-1], len(records), len(specs))
    return records


def _report(record: dict, index: int, total: int) -> None:
    spec = record["spec"]
    label = f"[{index}/{total}] {spec['strategy']} {spec['board']} seed{spec['seed']}"
    if record["status"] != "ok":
        print(f"{label}: {record['status'].upper()} {record.get('reason', '')[:90]}")
        return
    metrics = record["metrics"]
    print(f"{label}: cp={metrics['cp']} rout={metrics['rout']:.3f} drv={metrics['drv']:<4d} "
          f"wl={metrics['wl_mm']:8.1f}mm vias={metrics['vias']:<4d} t={metrics['time_s']:.1f}s")
