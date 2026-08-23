"""Strategy plugins. Importing this package registers every strategy."""

from . import s0_random, s1_grid_astar  # noqa: F401

__all__ = ["s0_random", "s1_grid_astar"]
