"""The oracle: KiCad's own DRC engine, run headless.

``kicad-cli`` *is* KiCad -- the same binaries, the same DRC engine, the same geometry library
the GUI uses. Running headless is a process-model choice, not a different checker. This is
the authoritative source of truth for every score on the leaderboard; a strategy's own
internal clearance check exists only to make its inner loop fast, and is deliberately
conservative so that it can cost us completion but never correctness.

The engine is external and we do not control it, so every result carries the KiCad version
that produced it. A score without an engine version attached is not a score.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

__all__ = ["DrcResult", "Violation", "OracleError", "find_kicad_cli", "run_drc",
           "kicad_version"]


class OracleError(RuntimeError):
    """Raised when the DRC engine cannot be run or its output cannot be understood."""


_SEARCH_HINTS = (
    r"C:\Program Files\KiCad\9.0\bin\kicad-cli.exe",
    r"C:\Program Files\KiCad\8.0\bin\kicad-cli.exe",
    "/usr/bin/kicad-cli",
    "/usr/local/bin/kicad-cli",
    "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli",
)


def find_kicad_cli() -> Path:
    """Locate ``kicad-cli``: explicit env var, then PATH, then the usual install locations."""
    override = os.environ.get("OPTIMAL_PRIME_KICAD_CLI")
    if override:
        path = Path(override)
        if not path.exists():
            raise OracleError(f"OPTIMAL_PRIME_KICAD_CLI points at a missing file: {path}")
        return path

    found = shutil.which("kicad-cli")
    if found:
        return Path(found)

    for hint in _SEARCH_HINTS:
        candidate = Path(hint)
        if candidate.exists():
            return candidate

    raise OracleError(
        "kicad-cli not found. Install KiCad 9, or set OPTIMAL_PRIME_KICAD_CLI to its path."
    )


@dataclass(frozen=True, slots=True)
class Violation:
    type: str
    severity: str
    description: str
    positions: tuple[tuple[float, float], ...] = ()
    uuids: tuple[str, ...] = ()

    def key(self) -> tuple:
        """Identity used to match a violation against the board's pre-existing ones.

        Positions are rounded to the micron: KiCad reports millimetres to six places and we
        do not want a sub-nanometre difference to make a pre-existing violation look new.
        """
        return (self.type, tuple(sorted((round(x, 3), round(y, 3)) for x, y in self.positions)))


@dataclass(frozen=True, slots=True)
class DrcResult:
    kicad_version: str
    violations: tuple[Violation, ...]
    unconnected: tuple[Violation, ...]
    source: str
    raw: dict = field(default=None, repr=False, compare=False)

    @property
    def unconnected_count(self) -> int:
        return len(self.unconnected)

    def by_type(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for violation in self.violations:
            counts[violation.type] = counts.get(violation.type, 0) + 1
        return counts


def _parse_violation(entry: dict) -> Violation:
    positions = []
    uuids = []
    for item in entry.get("items", []):
        pos = item.get("pos")
        if isinstance(pos, dict) and "x" in pos and "y" in pos:
            positions.append((float(pos["x"]), float(pos["y"])))
        if item.get("uuid"):
            uuids.append(str(item["uuid"]))
    return Violation(
        type=str(entry.get("type", "unknown")),
        severity=str(entry.get("severity", "error")),
        description=str(entry.get("description", "")),
        positions=tuple(positions),
        uuids=tuple(uuids),
    )


def run_drc(board_path: str | Path, cli: Path | None = None,
            timeout_s: float = 900.0) -> DrcResult:
    """Run DRC on a board and return the parsed result.

    ``--severity-all`` is used deliberately: filtering happens in :mod:`arena.score` against
    the canonical severity map, so that what counts is a decision recorded in our code rather
    than whatever the board's project file happens to say.
    """
    board_path = Path(board_path)
    if not board_path.exists():
        raise OracleError(f"board not found: {board_path}")

    cli = cli or find_kicad_cli()
    report = board_path.with_suffix(".drc.json")

    command = [
        str(cli), "pcb", "drc",
        "--format", "json",
        "--severity-all",
        "--units", "mm",
        "-o", str(report),
        str(board_path),
    ]

    try:
        completed = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout_s,
        )
    except subprocess.TimeoutExpired as exc:
        raise OracleError(f"DRC timed out after {timeout_s}s on {board_path.name}") from exc

    if not report.exists():
        raise OracleError(
            f"DRC produced no report for {board_path.name} "
            f"(exit {completed.returncode}): {completed.stderr.strip()[:400]}"
        )

    try:
        data = json.loads(report.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise OracleError(f"DRC report for {board_path.name} is not valid JSON") from exc

    return DrcResult(
        kicad_version=str(data.get("kicad_version", "unknown")),
        violations=tuple(_parse_violation(v) for v in data.get("violations", [])),
        unconnected=tuple(_parse_violation(v) for v in data.get("unconnected_items", [])),
        source=str(data.get("source", board_path.name)),
        raw=data,
    )


def kicad_version(cli: Path | None = None) -> str:
    """The engine version, for the run record."""
    cli = cli or find_kicad_cli()
    try:
        completed = subprocess.run(
            [str(cli), "--version"], capture_output=True, text=True, timeout=60,
        )
    except (subprocess.SubprocessError, OSError) as exc:
        raise OracleError(f"could not run {cli}: {exc}") from exc
    return completed.stdout.strip() or "unknown"
