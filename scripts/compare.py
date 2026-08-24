"""Side-by-side comparison of sequential settling and joint relaxation.

Runs both on the same boards, in the same process, so the only thing that differs is the
scheme. Writes ``results/comparison.json`` and prints the table.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from taut.board import load_board                       # noqa: E402
from taut.emit import write_board                       # noqa: E402
from taut.route import route_board                      # noqa: E402

DEMOS = Path(os.environ.get("KICAD_DEMOS",
                            r"C:\Program Files\KiCad\9.0\share\kicad\demos"))

ROUTING_RULES = {
    "clearance", "shorting_items", "tracks_crossing", "track_dangling", "via_dangling",
    "copper_edge_clearance", "hole_clearance", "hole_to_hole", "hole_near_hole",
    "holes_co_located", "track_width", "track_angle", "track_segment_length",
    "annular_width", "drill_out_of_range", "items_not_allowed", "item_on_disabled_layer",
    "copper_sliver", "isolated_copper", "connection_width", "starved_thermal",
    "solder_mask_bridge", "creepage", "too_many_vias", "padstack",
}

CASES = [
    ("ecc83-pp", ["F.Cu"]),
    ("ecc83-pp", ["F.Cu", "B.Cu"]),
    ("sonde xilinx", ["F.Cu", "B.Cu"]),
]


def find_cli() -> Path:
    override = os.environ.get("KICAD_CLI")
    if override and Path(override).exists():
        return Path(override)
    hint = Path(r"C:\Program Files\KiCad\9.0\bin\kicad-cli.exe")
    if hint.exists():
        return hint
    raise SystemExit("kicad-cli not found; set KICAD_CLI")


def demo(name: str) -> Path:
    hits = glob.glob(str(DEMOS / "**" / f"{name}.kicad_pcb"), recursive=True)
    if not hits:
        raise SystemExit(f"demo board {name!r} not installed")
    return Path(hits[0])


def score(cli: Path, board, result, work: Path, tag: str) -> dict:
    out = work / f"{tag}.kicad_pcb"
    write_board(board, result, out, seed=tag)
    report = out.with_suffix(".drc.json")
    subprocess.run([str(cli), "pcb", "drc", "--format", "json", "--severity-all",
                    "--units", "mm", "-o", str(report), str(out)],
                   capture_output=True, text=True, timeout=900)
    drc = json.loads(report.read_text(encoding="utf-8"))
    violations = [v for v in drc["violations"] if v["type"] in ROUTING_RULES]
    return {"drc": len(violations), "unconnected": len(drc["unconnected_items"])}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT / "results")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    cli = find_cli()
    work = args.out / "_work"
    work.mkdir(exist_ok=True)

    rows = []
    for name, layers in CASES:
        board = load_board(demo(name))
        label = f"{name} [{'+'.join(layers)}]"
        print(f"\n{label}")
        entry = {"board": name, "layers": layers}

        for mode, use_bundle in (("sequential", False), ("relaxed", True)):
            started = time.monotonic()
            result = route_board(board, layers=layers, bundle=use_bundle,
                                 relax_seconds=240)
            elapsed = time.monotonic() - started
            checked = score(cli, board, result, work, f"{name}-{mode}".replace(" ", "_"))
            stats = result.stats
            entry[mode] = {
                "routed": stats["routed"],
                "connections": stats["connections"],
                "arcs": stats["arcs"],
                "length_mm": stats["length_mm"],
                "seconds": round(elapsed, 1),
                **checked,
            }
            print(f"  {mode:10s} routed {stats['routed']:3d}/{stats['connections']:<3d} "
                  f"arcs {stats['arcs']:<3d} {stats['length_mm']:8.1f}mm "
                  f"DRC {checked['drc']}  unconn {checked['unconnected']}  {elapsed:5.1f}s")
        rows.append(entry)

    (args.out / "comparison.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")

    print("\n" + "=" * 96)
    print(f"{'board':26s} {'':6s} {'routed':>10s} {'arcs':>6s} {'copper':>10s} "
          f"{'DRC':>5s} {'unconn':>7s} {'time':>8s}")
    for entry in rows:
        label = f"{entry['board']} [{'+'.join(entry['layers'])}]"
        for mode in ("sequential", "relaxed"):
            m = entry[mode]
            print(f"{label[:26]:26s} {mode[:6]:6s} "
                  f"{m['routed']:>4d}/{m['connections']:<5d} {m['arcs']:>6d} "
                  f"{m['length_mm']:>9.1f}mm {m['drc']:>5d} {m['unconnected']:>7d} "
                  f"{m['seconds']:>7.1f}s")
            label = ""
    print(f"\nwrote {args.out / 'comparison.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
