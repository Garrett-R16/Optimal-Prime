"""P1 -- the comparison machinery and the baseline strategies.

The properties tested here are the ones that make a leaderboard trustworthy: a run record
carries everything needed to reproduce and compare it, a strategy that crashes costs one cell
rather than the sweep, and a stochastic strategy is reproducible from its seed alone.
"""

from __future__ import annotations

import json
from pathlib import Path
from random import Random

import pytest

import strategies  # noqa: F401  -- registers S0 and S1
from arena import runner, strategy as strategy_mod
from arena.problem import load_problem
from arena.strategy import Budget, register

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / "boards" / "manifest.json"

pytestmark = pytest.mark.skipif(
    not MANIFEST.exists(),
    reason="no board manifest; run scripts/fetch_boards.py --source kicad-demos",
)


@pytest.fixture(scope="module")
def manifest() -> dict:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def small_entry(manifest) -> dict:
    """The smallest board in the manifest, so tests stay fast."""
    return min(manifest["boards"], key=lambda b: b["nets"] * b["pads"])


@pytest.fixture(scope="module")
def small_problem(small_entry):
    return load_problem(ROOT / small_entry["path"], ROOT / small_entry["project"])


# --------------------------------------------------------------------------- registry

def test_baselines_are_registered():
    assert {"S0", "S1"} <= set(strategy_mod.available())


def test_unknown_strategy_names_are_rejected():
    with pytest.raises(KeyError):
        strategy_mod.get("S99")


# --------------------------------------------------------------------------- strategies

@pytest.mark.parametrize("name", ["S0", "S1"])
def test_strategy_is_reproducible_from_its_seed(name, small_problem):
    first = strategy_mod.get(name).route(small_problem, Random(11), Budget(120))
    second = strategy_mod.get(name).route(small_problem, Random(11), Budget(120))
    assert first.items == second.items
    assert first.routed_nets == second.routed_nets


def test_different_seeds_give_different_net_orders(small_problem):
    orders = {
        tuple(strategy_mod.get("S1").route(small_problem, Random(seed), Budget(120))
              .meta["net_order"])
        for seed in range(8)
    }
    assert len(orders) > 1, "seed must actually change the net order; E2 measures its effect"


def test_s0_produces_copper_and_s1_produces_less_of_it(small_problem):
    floor = strategy_mod.get("S0").route(small_problem, Random(3), Budget(120))
    grid = strategy_mod.get("S1").route(small_problem, Random(3), Budget(120))
    assert floor.items and grid.items
    assert grid.wirelength_nm < floor.wirelength_nm, \
        "a router that loses to random geometry on wirelength is not a router"


def test_strategy_respects_an_exhausted_budget(small_problem):
    spent = Budget(wall_clock_s=0.0)
    solution = strategy_mod.get("S1").route(small_problem, Random(1), spent)
    assert solution.abandoned_nets, "an exhausted budget must abandon nets, not route them"
    assert not solution.routed_nets


# --------------------------------------------------------------------------- runner

def test_run_record_carries_everything_needed_to_compare_it(small_entry, tmp_path):
    """MVP-PLAN section 5.1: six things must be pinned before two numbers are comparable."""
    spec = runner.RunSpec("S1", small_entry["name"], seed=5)
    record = runner.run_one(spec, small_entry, tmp_path, budget_s=120, keep_board=False)

    assert record["status"] == "ok", record.get("reason")
    assert record["engine"]["kicad"]                      # 1. engine version
    assert record["board"]["rules_sha256"]                # 2. effective rule set
    assert record["board"]["sha256"]                      # 3. the instance
    assert record["spec"]["seed"] == 5                    # 4. seed
    assert record["strategy"]["git"]                      #    and code version
    assert record["score_ver"] == 1                       # 5. metric definitions
    assert record["host"]["os"]                           # 6. the machine, for time only

    metrics = record["metrics"]
    assert set(metrics) >= {"cp", "rout", "drv", "wl_mm", "wl_ratio", "vias", "time_s"}
    assert metrics["cp"] in (0, 1)
    assert 0.0 <= metrics["rout"] <= 1.0

    written = json.loads((tmp_path / spec.relative_path()).read_text(encoding="utf-8"))
    assert written == record, "the record on disk must be the record returned"


def test_a_crashing_strategy_costs_one_cell_not_the_matrix(small_entry, tmp_path):
    @register("S_CRASH_TEST")
    class Exploding:
        def route(self, problem, rng, budget):
            raise ZeroDivisionError("deliberate")

    try:
        spec = runner.RunSpec("S_CRASH_TEST", small_entry["name"], seed=1)
        record = runner.run_one(spec, small_entry, tmp_path, budget_s=60, keep_board=False)
        assert record["status"] == "crash"
        assert "ZeroDivisionError" in record["reason"]
        assert "traceback" in record
        assert (tmp_path / spec.relative_path()).exists(), \
            "a crashed cell must still leave a record, or the sweep silently loses it"
    finally:
        strategy_mod.REGISTRY.pop("S_CRASH_TEST", None)


def test_matrix_resumes_instead_of_redoing_work(small_entry, tmp_path, manifest, monkeypatch):
    monkeypatch.setattr(runner, "results_dir", lambda tag: tmp_path)

    first = runner.run_matrix(["S1"], [small_entry["name"]], [1], tag="t",
                              workers=1, budget_s=120, manifest=manifest)
    assert len(first) == 1

    second = runner.run_matrix(["S1"], [small_entry["name"]], [1], tag="t",
                               workers=1, budget_s=120, manifest=manifest)
    assert second == [], "an already-completed cell must be skipped on resume"


def test_unknown_board_is_rejected_loudly(manifest):
    with pytest.raises(KeyError):
        runner.run_matrix(["S1"], ["no-such-board"], [1], tag="t", manifest=manifest)


def test_baseline_is_reconstructed_from_the_manifest(small_entry):
    """Scoring must not re-run DRC on the bare board for every cell."""
    baseline = runner._baseline_from_manifest(small_entry)
    assert baseline.unconnected_count == small_entry["baseline"]["unconnected"]
    assert baseline.kicad_version == small_entry["baseline"]["kicad_version"]
