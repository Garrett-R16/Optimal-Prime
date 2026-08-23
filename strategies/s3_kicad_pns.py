"""S3 -- KiCad's own PNS router. The engine-native baseline.

This is the router a KiCad user would otherwise reach for, so it is the most directly
meaningful comparison there is. It is also, for now, the one baseline this harness cannot
build.

The obstacle is specific. Stock ``kicad-python`` (kipy) exposes board *item* CRUD --
``create_items``, ``update_items``, ``get_tracks`` -- but not the interactive router.
PCBWorld reached ``PNS::ROUTER`` by writing its own 14 bindings against the KiCad C++ engine.
Reproducing that is a C++ binding effort disproportionate to a baseline, so S3 waits on
either PCBWorld's code being released or a future kipy exposing the router.

Registered as unavailable so the gap shows up in every sweep rather than only in prose.
"""

from __future__ import annotations

from random import Random

from arena.problem import Problem
from arena.solution import Solution
from arena.strategy import Budget, Unavailable, register


@register("S3")
class KiCadPNS:
    def route(self, problem: Problem, rng: Random, budget: Budget) -> Solution:
        raise Unavailable(
            "kicad-python exposes board item CRUD, not PNS::ROUTER. Driving KiCad's own "
            "router needs custom C++ bindings (PCBWorld wrote 14 of them). Blocked on that "
            "code being released or on kipy exposing the router. S2 is the baseline that "
            "matters meanwhile."
        )
