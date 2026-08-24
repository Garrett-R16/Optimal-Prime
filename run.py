"""Route a board with taut strings, check it with KiCad, and render the result.

    python run.py --board ecc83-pp --layers F.Cu
    python run.py --board ecc83-pp --layers F.Cu,B.Cu

Boards come from KiCad's own installed demos; nothing here builds test boards.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from taut.board import load_board                       # noqa: E402
from taut.emit import write_board                       # noqa: E402
from taut.route import route_board                      # noqa: E402

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "examples"

DEMOS = Path(os.environ.get(
    "KICAD_DEMOS", r"C:\Program Files\KiCad\9.0\share\kicad\demos"))

_CLI_HINTS = (
    r"C:\Program Files\KiCad\9.0\bin\kicad-cli.exe",
    "/usr/bin/kicad-cli",
    "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli",
)

#: DRC rules a router is answerable for. Silkscreen, courtyards, library metadata and
#: schematic parity are not routing problems and are excluded from the verdict.
ROUTING_RULES = {
    "clearance", "shorting_items", "tracks_crossing", "track_dangling", "via_dangling",
    "copper_edge_clearance", "hole_clearance", "hole_to_hole", "hole_near_hole",
    "holes_co_located", "track_width", "track_angle", "track_segment_length",
    "annular_width", "drill_out_of_range", "items_not_allowed", "item_on_disabled_layer",
    "copper_sliver", "isolated_copper", "connection_width", "starved_thermal",
    "solder_mask_bridge", "creepage", "too_many_vias", "padstack",
}


def find_cli() -> Path:
    override = os.environ.get("KICAD_CLI")
    if override and Path(override).exists():
        return Path(override)
    found = shutil.which("kicad-cli")
    if found:
        return Path(found)
    for hint in _CLI_HINTS:
        if Path(hint).exists():
            return Path(hint)
    raise SystemExit("kicad-cli not found; set KICAD_CLI to its path")


def find_demo(name: str) -> Path:
    matches = glob.glob(str(DEMOS / "**" / f"{name}.kicad_pcb"), recursive=True)
    if not matches:
        available = sorted(Path(p).stem for p in
                           glob.glob(str(DEMOS / "**" / "*.kicad_pcb"), recursive=True))
        raise SystemExit(f"no demo board {name!r}. Available:\n  " + "\n  ".join(available))
    return Path(matches[0])


def run_drc(cli: Path, board: Path) -> dict:
    report = board.with_suffix(".drc.json")
    subprocess.run([str(cli), "pcb", "drc", "--format", "json", "--severity-all",
                    "--units", "mm", "-o", str(report), str(board)],
                   capture_output=True, text=True, timeout=900)
    if not report.exists():
        raise SystemExit(f"DRC produced no report for {board.name}")
    return json.loads(report.read_text(encoding="utf-8"))


def render(cli: Path, board: Path, out_svg: Path, layers: str) -> bool:
    """Export an SVG so the result can actually be looked at."""
    result = subprocess.run(
        [str(cli), "pcb", "export", "svg", "--layers", layers,
         "--page-size-mode", "2", "--exclude-drawing-sheet",
         "-o", str(out_svg), str(board)],
        capture_output=True, text=True, timeout=600)
    if not out_svg.exists():
        print(f"    (render failed: {result.stderr.strip()[:160]})")
        return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--board", default="ecc83-pp", help="KiCad demo board name")
    parser.add_argument("--layers", default="F.Cu", help="comma-separated copper layers")
    parser.add_argument("--order", default="longest-first",
                        choices=("longest-first", "shortest-first"))
    parser.add_argument("--tag", default=None, help="output name (defaults to board+layers)")
    parser.add_argument("--relax-seconds", type=float, default=240.0,
                        help="wall-clock budget for the joint relaxation")
    args = parser.parse_args()

    cli = find_cli()
    source = find_demo(args.board)
    layers = [layer.strip() for layer in args.layers.split(",") if layer.strip()]
    tag = args.tag or f"{args.board}-{len(layers)}layer"

    work = OUT / tag
    work.mkdir(parents=True, exist_ok=True)

    board = load_board(source)
    print(f"board   {board.name}: {len(board.pads)} pads, {len(board.routable)} nets, "
          f"layers {list(board.copper_layers)}")
    print(f"routing on {layers} ({args.order})")

    started = time.monotonic()
    result = route_board(board, layers=layers, order=args.order,
                         relax_seconds=args.relax_seconds)
    elapsed = time.monotonic() - started

    out_board = work / f"{tag}.kicad_pcb"
    write_board(board, result, out_board, seed=tag)

    drc = run_drc(cli, out_board)
    violations = [v for v in drc.get("violations", []) if v.get("type") in ROUTING_RULES]
    unconnected = drc.get("unconnected_items", [])

    by_type: dict[str, int] = {}
    for violation in violations:
        by_type[violation["type"]] = by_type.get(violation["type"], 0) + 1

    print()
    print(f"  connections   {result.stats['connections']}")
    print(f"  routed        {result.stats['routed']}")
    print(f"  failed        {result.stats['failed']}")
    print(f"  tracks        {result.stats['tracks']}  ({result.stats['arcs']} arcs)")
    print(f"  length        {result.stats['length_mm']} mm")
    print(f"  time          {elapsed:.1f}s")
    print(f"  DRC errors    {len(violations)}   {by_type if by_type else ''}")
    print(f"  unconnected   {len(unconnected)}")
    clean = not violations and not unconnected and not result.failed
    print(f"  VERDICT       {'CLEAN' if clean else 'NOT CLEAN'}")

    if result.failed:
        print("\n  failures:")
        for code, name, reason in result.failed[:10]:
            print(f"    net {code} {name}: {reason[:110]}")

    svg = work / f"{tag}.svg"
    if render(cli, out_board, svg, ",".join(layers + ["Edge.Cuts"])):
        print(f"\n  render        {svg}")
    print(f"  board         {out_board}")

    summary = {
        "board": board.name, "layers": layers, "order": args.order,
        "stats": result.stats, "drc_errors": len(violations),
        "drc_by_type": by_type, "unconnected": len(unconnected),
        "clean": clean, "seconds": round(elapsed, 1),
        "kicad": drc.get("kicad_version"),
        "failures": [{"net": c, "name": n, "reason": r} for c, n, r in result.failed],
    }
    (work / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return 0 if clean else 1


if __name__ == "__main__":
    raise SystemExit(main())
