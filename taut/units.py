"""Unit conversion between KiCad's file format and our internal representation.

The board file stores millimetres as decimal text; KiCad's internal units are integer
nanometres. We work in integer nanometres everywhere and convert only at the I/O boundary,
because accumulating float error in millimetres produces clearance failures at exactly the
tolerance the rules care about -- a 0.2 mm clearance checked against a value that is
0.19999999997 mm fails, and the cause is invisible.
"""

from __future__ import annotations

__all__ = ["NM_PER_MM", "mm_to_nm", "nm_to_mm", "GUARDBAND_NM"]

NM_PER_MM = 1_000_000

#: Clearance guardband. KiCad's arc clearance arithmetic has documented rounding artifacts --
#: a nominal 10 mil clearance measuring 9.99996 mil and failing DRC. We route to
#: ``clearance + GUARDBAND_NM`` so that a layout our checker calls legal is never rejected by
#: the engine for a sub-micron rounding difference. 2 um is far below any real design rule and
#: far above the artifact.
GUARDBAND_NM = 2_000


def mm_to_nm(value: float) -> int:
    """Millimetres (as read from a board file) to integer nanometres."""
    return int(round(value * NM_PER_MM))


def nm_to_mm(value: int) -> float:
    """Integer nanometres back to millimetres for emission."""
    return value / NM_PER_MM


#: Fractional margin added on top of the design clearance.
#:
#: A taut path rides *exactly* on its keep-out boundary -- that is what taut means -- so
#: without this the copper sits at precisely the clearance limit with nothing to spare for
#: KiCad's own rounding, for solder-mask webbing between adjacent apertures, or for any small
#: difference between our shape model and the engine's. Riding the limit is fragile rather
#: than optimal; 8% of the clearance costs a few microns of routing room and removes a whole
#: class of marginal violation.
CLEARANCE_MARGIN = 0.08
