"""Strategy plugins. Importing this package registers every strategy.

Strategies that cannot run in a given environment are still registered; they raise
``Unavailable`` and the runner records that, with the reason, in the run matrix. A baseline
that quietly vanishes from the leaderboard is worse than one that is visibly absent.
"""

from . import (  # noqa: F401
    s0_random,
    s1_grid_astar,
    s2_freerouting,
    s3_kicad_pns,
    s4_pathfinder,
)

__all__ = ["s0_random", "s1_grid_astar", "s2_freerouting", "s3_kicad_pns", "s4_pathfinder"]
