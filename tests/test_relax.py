"""Relaxing bands together, and the scenario that needs it.

A gap wide enough for two tracks. Route one net through it alone and a rubber band does what
rubber bands do -- it sits in the middle. The second then finds the gap full, and no amount of
re-ordering helps: the taut path is deterministic, so putting the first net back lands it in
exactly the same place.

Solving them together fixes it, and needs two things that are easy to get wrong. Updates must
be *simultaneous*, or the first band's answer is permanent again. And the clearance between
bands must start *soft*, because the taut solver treats other copper as solid and solid walls
do not yield -- two bands in the middle of a gap each find no way past the other and nothing
moves.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import pytest

from taut.obstacles import Obstacle
from taut.relax import relax
from taut.route import _path_obstacles
from taut.tangent import NoPathFound, TautPath, solve

CLEARANCE = 1.0
WIDTH = 1.0
GAP = 5.5          # holds two tracks: floor((5.5 - 1) / (1 + 1)) = 2


def walls(gap: float = GAP) -> list[Obstacle]:
    """Two blocks with a corridor of ``gap`` between their cores, centred on y = 0."""
    half = gap / 2.0
    return [
        Obstacle(vertices=((-6.0, -20.0), (6.0, -20.0), (6.0, -half), (-6.0, -half)),
                 r=CLEARANCE + WIDTH / 2, net=0, label="lower wall"),
        Obstacle(vertices=((-6.0, half), (6.0, half), (6.0, 20.0), (-6.0, 20.0)),
                 r=CLEARANCE + WIDTH / 2, net=0, label="upper wall"),
    ]


@dataclass
class Band:
    """The shape :func:`taut.relax.relax` expects of an item."""

    net_code: int
    start: tuple[float, float]
    goal: tuple[float, float]
    obstacles: list
    layer: str = "F.Cu"
    layer_options: tuple[str, ...] = ("F.Cu",)
    halo: float = CLEARANCE + WIDTH
    path: TautPath | None = None

    def seed(self):
        self.path = solve(self.start, self.goal, self.obstacles)
        return self


def solve_one(band: Band, layer: str, blockers):
    return solve(band.start, band.goal, band.obstacles + blockers)


def y_in_corridor(path: TautPath) -> float:
    """Where the path sits at x = 0, which is the only place the corridor constrains it."""
    best = None
    for element in path.elements:
        for i in range(41):
            t = i / 40
            if hasattr(element, "y1"):
                x = element.x1 + (element.x2 - element.x1) * t
                y = element.y1 + (element.y2 - element.y1) * t
            else:
                x, y = element.point_at(t)
            if best is None or abs(x) < abs(best[0]):
                best = (x, y)
    return best[1]


# --------------------------------------------------------------------------- the scenario

def test_a_lone_band_sits_in_the_middle():
    """The behaviour that strands the second net, stated as a fact about the geometry."""
    path = solve((-15.0, 0.0), (15.0, 0.0), walls())
    assert abs(y_in_corridor(path)) < 1e-6


def test_sequentially_the_second_band_cannot_fit():
    obstacles = walls()
    first = solve((-15.0, 0.0), (15.0, 0.0), obstacles)
    occupied = _path_obstacles(first, CLEARANCE + WIDTH, net=1)
    with pytest.raises(NoPathFound):
        solve((-15.0, -0.2), (15.0, 0.2), obstacles + occupied)


def contending_bands(obstacles):
    """Two bands that both need the corridor, with endpoints far enough apart to be legal.

    The first runs straight through and so wants the middle. The second starts and ends well
    below the lower wall, which it cannot cross, so its only way across is the same corridor.
    Putting their endpoints close together instead would make the test fail for a boring
    reason: the endpoints themselves would be inside each other's clearance.
    """
    return [
        Band(net_code=1, start=(-15.0, 0.0), goal=(15.0, 0.0), obstacles=obstacles).seed(),
        Band(net_code=2, start=(-15.0, -10.0), goal=(15.0, -10.0),
             obstacles=obstacles).seed(),
    ]


def test_relaxed_together_both_bands_fit():
    """Neither band is pushed. Both are re-solved at once, and both give way."""
    obstacles = walls()
    bands = contending_bands(obstacles)

    report = relax(bands, solve_one, _path_obstacles, steps=6)

    assert report.converged, (
        f"the two bands should end up clear of each other; "
        f"overlaps by step {report.overlaps_by_step}")

    a, b = (y_in_corridor(band.path) for band in bands)
    assert a * b < 0, f"they must take opposite sides of the corridor, got {a} and {b}"
    assert abs(a - b) >= WIDTH + CLEARANCE - 1e-6, \
        f"they must be at least a track pitch apart, got {abs(a - b):.3f}"


def test_one_pass_is_enough_when_there_is_room_to_go_around():
    """Recorded because it is why the annealing was removed.

    An earlier version softened the band-to-band clearance for the first few steps, on the
    argument that solid walls do not yield. They do not -- but bands turn out to have room to
    route *around* each other far more often than they need to pass *through*, so a single
    pass at full clearance already separates them here, and softening changed nothing on a
    real board either.
    """
    obstacles = walls()
    bands = contending_bands(obstacles)
    report = relax(bands, solve_one, _path_obstacles, steps=1)
    assert report.converged


def test_bands_that_do_not_meet_are_left_alone():
    """A band with nothing near it has already found its shape; re-deriving it is waste."""
    far = [
        Band(net_code=1, start=(-15.0, -12.0), goal=(15.0, -12.0), obstacles=[]).seed(),
        Band(net_code=2, start=(-15.0, 12.0), goal=(15.0, 12.0), obstacles=[]).seed(),
    ]
    before = [band.path.length for band in far]
    report = relax(far, solve_one, _path_obstacles, steps=4)
    assert report.moved == 0, "nothing contended, so nothing should have been re-solved"
    assert [band.path.length for band in far] == pytest.approx(before)


def test_relaxation_never_leaves_a_band_overlapping_a_pad():
    """Only band-to-band clearance is softened; the walls are solid throughout."""
    obstacles = walls()
    bands = contending_bands(obstacles)
    relax(bands, solve_one, _path_obstacles, steps=6)
    for band in bands:
        for element in band.path.elements:
            for i in range(21):
                t = i / 20
                if hasattr(element, "y1"):
                    x = element.x1 + (element.x2 - element.x1) * t
                    y = element.y1 + (element.y2 - element.y1) * t
                else:
                    x, y = element.point_at(t)
                for wall in obstacles:
                    assert not wall.contains(x, y, tol=1e-6), \
                        "a band may overlap another band on the way, never a wall"

