"""The strategy plugin interface.

One method. That is the point: a fifty-line random baseline and a full
CBS-over-triangulation implementation must both satisfy it, so that their outputs are
comparable and interchangeable and the comparison machinery never has to know which is which.

``rng`` is seeded per run, so every stochastic strategy is reproducible from its run record.
``budget`` carries a generous wall-clock cap -- compute time is explicitly not a goal in
MVP-01 -- and strategies are free to ignore it.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from random import Random
from typing import Callable, Protocol, runtime_checkable

from .problem import Problem
from .solution import Solution

__all__ = ["Budget", "Strategy", "register", "get", "available", "REGISTRY"]


@dataclass
class Budget:
    """What a strategy is allowed to spend.

    ``oracle_calls`` exists to tag runs, not to constrain them: a strategy that queries
    KiCad's DRC in its inner loop is doing something categorically different from one that
    does not, and the comparison has to acknowledge that rather than hide it.
    """

    wall_clock_s: float = 3600.0
    oracle_calls: int | None = None
    started_at: float = field(default_factory=time.monotonic)

    def elapsed_s(self) -> float:
        return time.monotonic() - self.started_at

    def exhausted(self) -> bool:
        return self.elapsed_s() >= self.wall_clock_s

    def remaining_s(self) -> float:
        return max(0.0, self.wall_clock_s - self.elapsed_s())


@runtime_checkable
class Strategy(Protocol):
    name: str

    def route(self, problem: Problem, rng: Random, budget: Budget) -> Solution: ...


class Unavailable(RuntimeError):
    """Raised by a registered strategy that cannot run in this environment.

    Distinct from a crash: an unavailable strategy is recorded as such in the run matrix,
    with its reason, rather than silently vanishing from the comparison.
    """


REGISTRY: dict[str, Callable[[], Strategy]] = {}


def register(name: str) -> Callable:
    """Register a strategy class under its S-number."""
    def decorate(cls):
        if name in REGISTRY:
            raise ValueError(f"strategy {name} is already registered")
        cls.name = name
        REGISTRY[name] = cls
        return cls
    return decorate


def get(name: str) -> Strategy:
    if name not in REGISTRY:
        raise KeyError(f"unknown strategy {name!r}; known: {sorted(REGISTRY)}")
    return REGISTRY[name]()


def available() -> list[str]:
    return sorted(REGISTRY)
