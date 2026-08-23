"""Build the benchmark board set.

Ingestion is not a copy. Each board is:

1. **filtered** to the frozen MVP-01 scope -- placement complete, at least two copper layers,
   at least one routable net, no blind/buried/micro vias;
2. **normalised** -- the canonical DRC severity map is written into its project file, because
   severities live there rather than in the board and a board shipped with ``clearance``
   downgraded to a warning would sail through Clean Pass while being unmanufacturable;
3. **stripped** -- existing routing is removed, so the instance is the bare placement problem;
4. **baselined** -- the bare board is scored once, and that result is stored in the manifest
   so that no strategy run is ever charged for a violation that was already there;
5. **hashed** -- board and rule-set SHA-256 go in the manifest, and a run whose hashes do not
   match is not comparable.

Usage::

    python scripts/fetch_boards.py --source kicad-demos
    python scripts/fetch_boards.py --source pcbench       # downloads ~273 MB
"""

from __future__ import annotations

import argparse
import glob
import json
import shutil
import ssl
import sys
import tarfile
import tempfile
import urllib.request
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arena import oracle, sexpr                                    # noqa: E402
from arena.emit import clear_routing                               # noqa: E402
from arena.problem import BoardLoadError, Problem, load_problem     # noqa: E402
from arena.rules import CANONICAL_SEVERITIES, rules_hash            # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
BOARDS = ROOT / "boards"
MANIFEST = BOARDS / "manifest.json"

PCBENCH_TARBALL = "https://codeload.github.com/PCBench/PCBench/tar.gz/refs/heads/main"
KICAD_DEMOS = Path(r"C:\Program Files\KiCad\9.0\share\kicad\demos")

MIN_LAYERS = 2
MAX_LAYERS = 8
MIN_NETS = 2


# --------------------------------------------------------------------------- sources

def source_kicad_demos() -> list[Path]:
    if not KICAD_DEMOS.exists():
        raise SystemExit(f"KiCad demos not found at {KICAD_DEMOS}")
    return [Path(p) for p in sorted(glob.glob(str(KICAD_DEMOS / "**" / "*.kicad_pcb"),
                                              recursive=True))]


def source_pcbench(cache: Path) -> list[Path]:
    """Download and unpack PCBench (164 boards, MIT).

    Uses the codeload tarball rather than ``git clone``: cloning fails on machines whose git
    cannot verify GitHub's certificate chain, and we only ever want a snapshot anyway.
    """
    cache.mkdir(parents=True, exist_ok=True)
    extracted = cache / "PCBench-main"
    if not extracted.exists():
        archive = cache / "pcbench.tar.gz"
        if not archive.exists():
            print(f"downloading PCBench (~273 MB) -> {archive}")
            context = ssl.create_default_context()
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
            with urllib.request.urlopen(PCBENCH_TARBALL, context=context) as response, \
                    archive.open("wb") as handle:
                shutil.copyfileobj(response, handle)
        print("unpacking...")
        with tarfile.open(archive) as tar:
            tar.extractall(cache, filter="data")
    return [Path(p) for p in sorted(glob.glob(str(extracted / "**" / "*.kicad_pcb"),
                                              recursive=True))]


# --------------------------------------------------------------------------- ingestion

def in_scope(problem: Problem) -> str | None:
    """Return a rejection reason, or None if the board belongs in the set."""
    if not (MIN_LAYERS <= problem.layer_count <= MAX_LAYERS):
        return f"layer count {problem.layer_count} outside [{MIN_LAYERS},{MAX_LAYERS}]"
    if len(problem.routable_nets) < MIN_NETS:
        return f"only {len(problem.routable_nets)} routable nets"
    if not problem.pads:
        return "no pads"
    if problem.rules.allow_blind_buried_vias:
        return "board permits blind/buried vias; MVP-01 freezes via type to through-hole"
    if problem.rules.allow_microvias:
        return "board permits microvias; MVP-01 freezes via type to through-hole"
    if problem.rules.unclassified_severities:
        return f"unclassified DRC rules: {problem.rules.unclassified_severities[:3]}"
    return None


def normalise_project(source: Path, destination: Path) -> None:
    """Copy the project file with the canonical severity map imposed."""
    data = json.loads(source.read_text(encoding="utf-8"))
    settings = data.setdefault("board", {}).setdefault("design_settings", {})
    settings["rule_severities"] = dict(CANONICAL_SEVERITIES)
    rules = settings.setdefault("rules", {})
    rules["allow_blind_buried_vias"] = False
    rules["allow_microvias"] = False
    destination.write_text(json.dumps(data, indent=2), encoding="utf-8")


def strip_filled_zones(tree) -> int:
    """Remove filled copper pours, keeping keepout zones.

    A pour is not a fixed obstacle: KiCad recomputes its fill around whatever tracks exist,
    carving clearance out of the copper. ``kicad-cli`` has no zone-fill command, so a pour
    left in place keeps the fill computed for the *original* routing and every track we lay
    reads as a clearance violation against copper that would not be there in reality.

    Rather than score against a stale fill, the pour is removed and its net becomes something
    the router has to reach with real copper. That makes the instance harder and well-posed
    instead of easier and wrong. Restoring pours needs zone refill over the IPC API, which is
    a P2+ concern; the count is recorded in the manifest so the change is never invisible.
    """
    keep, removed = [], 0
    for child in tree:
        if isinstance(child, list) and sexpr.head(child) == "zone":
            if sexpr.find(child, "keepout") is None:
                removed += 1
                continue
        keep.append(child)
    tree[:] = keep
    return removed


def ingest(board_path: Path, out_dir: Path) -> dict | None:
    """Normalise, strip and baseline one board. Returns its manifest entry, or None."""
    try:
        problem = load_problem(board_path)
    except BoardLoadError as exc:
        return {"name": board_path.stem, "rejected": str(exc)}

    reason = in_scope(problem)
    if reason:
        return {"name": board_path.stem, "rejected": reason}

    if list(board_path.parent.glob("*.kicad_dru")):
        return {"name": board_path.stem,
                "rejected": "board carries custom .kicad_dru rules; not yet supported"}

    target = out_dir / board_path.stem
    target.mkdir(parents=True, exist_ok=True)

    project_out = target / f"{board_path.stem}.kicad_pro"
    normalise_project(problem.project_path, project_out)

    bare_tree = sexpr.parse(sexpr.dumps(problem.tree))
    stripped = clear_routing(bare_tree)
    pours = strip_filled_zones(bare_tree)
    board_out = target / f"{board_path.stem}.kicad_pcb"
    board_out.write_text(sexpr.dumps(bare_tree), encoding="utf-8", newline="\n")

    bare = load_problem(board_out, project_out)
    try:
        baseline = oracle.run_drc(board_out)
    except oracle.OracleError as exc:
        shutil.rmtree(target, ignore_errors=True)
        return {"name": board_path.stem, "rejected": f"DRC failed on bare board: {exc}"}

    return {
        "name": board_path.stem,
        "path": str(board_out.relative_to(ROOT).as_posix()),
        "project": str(project_out.relative_to(ROOT).as_posix()),
        "origin": str(board_path),
        "board_sha256": bare.board_sha256,
        "rules_sha256": rules_hash(project_out),
        "layers": bare.layer_count,
        "copper_layers": list(bare.copper_layers),
        "nets": len(bare.routable_nets),
        "pads": len(bare.pads),
        "stripped_routing_items": stripped,
        "stripped_filled_zones": pours,
        "baseline": {
            "kicad_version": baseline.kicad_version,
            "unconnected": baseline.unconnected_count,
            "violations_by_type": baseline.by_type(),
            "violations": [asdict(v) for v in baseline.violations],
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("kicad-demos", "pcbench"), default="kicad-demos")
    parser.add_argument("--cache", type=Path,
                        default=Path(tempfile.gettempdir()) / "optimal-prime-cache")
    parser.add_argument("--limit", type=int, default=0, help="ingest at most N boards")
    parser.add_argument("--out", type=Path, default=BOARDS)
    args = parser.parse_args()

    boards = source_kicad_demos() if args.source == "kicad-demos" else source_pcbench(args.cache)
    if args.limit:
        boards = boards[:args.limit]
    print(f"{len(boards)} candidate boards from {args.source}")

    args.out.mkdir(parents=True, exist_ok=True)
    accepted, rejected = [], []
    for index, path in enumerate(boards, 1):
        entry = ingest(path, args.out)
        if entry is None or "rejected" in entry:
            rejected.append(entry or {"name": path.stem, "rejected": "unknown"})
            print(f"  [{index}/{len(boards)}] reject {path.stem}: {entry['rejected'][:80]}")
        else:
            accepted.append(entry)
            print(f"  [{index}/{len(boards)}] accept {entry['name']}: "
                  f"{entry['layers']}L {entry['nets']} nets {entry['pads']} pads, "
                  f"{entry['baseline']['unconnected']} connections to make")

    existing = json.loads(MANIFEST.read_text()) if MANIFEST.exists() else {"boards": []}
    known = {b["name"]: b for b in existing.get("boards", [])}
    known.update({b["name"]: b for b in accepted})

    manifest = {
        "score_source": "kicad-cli",
        "kicad_version": accepted[0]["baseline"]["kicad_version"] if accepted else None,
        "boards": sorted(known.values(), key=lambda b: b["name"]),
        "rejected": sorted(rejected, key=lambda b: b["name"]),
    }
    MANIFEST.parent.mkdir(parents=True, exist_ok=True)
    MANIFEST.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    print(f"\naccepted {len(accepted)}, rejected {len(rejected)}")
    print(f"manifest: {MANIFEST}  ({len(manifest['boards'])} boards total)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
