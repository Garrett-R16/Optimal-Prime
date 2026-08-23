"""S2 -- Freerouting. The bar.

Freerouting is the strongest freely available PCB autorouter and the number MVP-01 has to
beat: PCBWorld reports Clean Pass 0.80 on its D3-A open-source board set and 0.78 on D3-B.

It cannot be driven end to end from this harness yet, and the reason is worth stating
plainly rather than hiding: Freerouting consumes Specctra DSN, and ``kicad-cli`` has no
Specctra export subcommand. KiCad can only produce a ``.dsn`` through the GUI
(File > Export > Specctra DSN). So the input has to be exported once by hand and cached
beside the board, after which this strategy runs unattended.

Registering it as *unavailable* rather than omitting it is deliberate: the run matrix then
records the gap in every sweep, with its reason, instead of the bar quietly vanishing from
the leaderboard.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from random import Random

from arena.problem import Problem
from arena.solution import Solution
from arena.strategy import Budget, Unavailable, register


def find_freerouting() -> Path | None:
    """Locate the Freerouting jar or executable."""
    override = os.environ.get("OPTIMAL_PRIME_FREEROUTING")
    if override and Path(override).exists():
        return Path(override)
    found = shutil.which("freerouting")
    return Path(found) if found else None


def cached_dsn(problem: Problem) -> Path | None:
    """A ``.dsn`` exported by hand and left beside the board."""
    candidate = problem.source_path.with_suffix(".dsn")
    return candidate if candidate.exists() else None


@register("S2")
class Freerouting:
    """Run Freerouting over a pre-exported Specctra DSN and import its session file."""

    def route(self, problem: Problem, rng: Random, budget: Budget) -> Solution:
        dsn = cached_dsn(problem)
        if dsn is None:
            raise Unavailable(
                f"no {problem.source_path.with_suffix('.dsn').name} beside the board. "
                "kicad-cli has no Specctra export subcommand, so the DSN must be exported "
                "once from the KiCad GUI (File > Export > Specctra DSN) and cached next to "
                "the board file."
            )

        jar = find_freerouting()
        if jar is None:
            raise Unavailable(
                "Freerouting not found. Install it and set OPTIMAL_PRIME_FREEROUTING to the "
                "jar, or put 'freerouting' on PATH."
            )

        with tempfile.TemporaryDirectory() as work:
            session = Path(work) / "out.ses"
            command = (["java", "-jar", str(jar)] if jar.suffix == ".jar" else [str(jar)])
            command += ["-de", str(dsn), "-do", str(session)]
            try:
                subprocess.run(command, capture_output=True, text=True,
                               timeout=max(60.0, budget.remaining_s()), check=False)
            except subprocess.TimeoutExpired as exc:
                raise Unavailable(f"Freerouting timed out: {exc}") from exc

            if not session.exists() or session.stat().st_size == 0:
                raise Unavailable("Freerouting produced no session file")

            # Parsing SES back into our Solution IR is the remaining piece of work. It is
            # deliberately not stubbed with something that half-works: a baseline that
            # silently under-reports is worse than one that is honestly absent.
            raise Unavailable(
                "Specctra SES import is not implemented yet; Freerouting ran but its result "
                "cannot be scored. See MVP-PLAN P1."
            )
