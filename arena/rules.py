"""Design rules and the canonical DRC severity map.

Two things live here, and both exist because of the same trap: **the rules are not in the
board file**. DRC severities and netclass geometry live in the sibling ``.kicad_pro``, and
custom rules live in a ``.kicad_dru``. A board shipped with ``clearance`` downgraded to a
warning would sail through Clean Pass while being unmanufacturable, and nothing about the run
would look wrong.

So board ingestion normalises: every benchmark board gets :data:`CANONICAL_SEVERITIES`
written into its project file, and the hash of the effective rule set goes into every run
record. See MVP-PLAN.md section 5.1.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path

from .units import mm_to_nm

__all__ = ["CANONICAL_SEVERITIES", "ROUTING_RULES", "NetClass", "DesignRules",
           "load_design_rules", "rules_hash"]


# ---------------------------------------------------------------------------------------
# The canonical severity map.
#
# "error"  -- a geometric or electrical rule our routing can violate. Counts against us.
# "ignore" -- not about routing: silkscreen, text, courtyards, library metadata, schematic
#             parity, or a constraint frozen out of MVP-01 scope (differential pairs,
#             length/skew tuning). Placement rules are ignored because placement is an input
#             we are not allowed to change.
#
# Every key KiCad 9.0 knows about is listed explicitly. Silence is not a decision; if KiCad
# adds a rule, load_design_rules() will flag it as unclassified rather than guessing.
# ---------------------------------------------------------------------------------------
CANONICAL_SEVERITIES: dict[str, str] = {
    # --- copper geometry: ours ---------------------------------------------------------
    "clearance": "error",
    "copper_edge_clearance": "error",
    "copper_sliver": "error",
    "connection_width": "error",
    "creepage": "error",
    "hole_clearance": "error",
    "hole_near_hole": "error",
    "hole_to_hole": "error",
    "holes_co_located": "error",
    "isolated_copper": "error",
    "items_not_allowed": "error",
    "item_on_disabled_layer": "error",
    "shorting_items": "error",
    "solder_mask_bridge": "error",
    "starved_thermal": "error",
    "too_many_vias": "error",
    "track_angle": "error",
    "track_dangling": "error",
    "track_segment_length": "error",
    "track_width": "error",
    "tracks_crossing": "error",
    "unconnected_items": "error",
    "via_dangling": "error",
    "zones_intersect": "error",
    # --- padstack / drill geometry: ours once we place vias -----------------------------
    "annular_width": "error",
    "drill_out_of_range": "error",
    "microvia_drill_out_of_range": "error",
    "padstack": "error",
    "through_hole_pad_without_hole": "error",
    # --- board integrity: not ours, but a board failing these is unusable --------------
    "invalid_outline": "error",
    # --- frozen out of MVP-01 scope ----------------------------------------------------
    "diff_pair_gap_out_of_range": "ignore",
    "diff_pair_uncoupled_length_too_long": "ignore",
    "length_out_of_range": "ignore",
    "skew_out_of_range": "ignore",
    # --- placement: an input we may not change -----------------------------------------
    "courtyards_overlap": "ignore",
    "malformed_courtyard": "ignore",
    "missing_courtyard": "ignore",
    "npth_inside_courtyard": "ignore",
    "overlapping_pads": "ignore",
    "pth_inside_courtyard": "ignore",
    # --- library / schematic parity: not about geometry at all -------------------------
    "duplicate_footprints": "ignore",
    "extra_footprint": "ignore",
    "footprint": "ignore",
    "footprint_filters_mismatch": "ignore",
    "footprint_symbol_mismatch": "ignore",
    "footprint_type_mismatch": "ignore",
    "lib_footprint_issues": "ignore",
    "lib_footprint_mismatch": "ignore",
    "missing_footprint": "ignore",
    "net_conflict": "ignore",
    "unresolved_variable": "ignore",
    # --- cosmetic layers ---------------------------------------------------------------
    "mirrored_text_on_front_layer": "ignore",
    "nonmirrored_text_on_back_layer": "ignore",
    "silk_edge_clearance": "ignore",
    "silk_over_copper": "ignore",
    "silk_overlap": "ignore",
    "text_height": "ignore",
    "text_thickness": "ignore",
}

#: The rule keys a router is answerable for. Used to filter violations when scoring.
ROUTING_RULES = frozenset(k for k, v in CANONICAL_SEVERITIES.items() if v == "error")


@dataclass(frozen=True, slots=True)
class NetClass:
    """Geometry constraints for one netclass, in integer nanometres."""

    name: str
    clearance_nm: int
    track_width_nm: int
    via_diameter_nm: int
    via_drill_nm: int


@dataclass(frozen=True, slots=True)
class DesignRules:
    """The effective rule set for one board.

    Everything is integer nanometres, matching KiCad's own internal units, because working in
    millimetre floats produces failures at exactly the tolerance the clearance rules care
    about.
    """

    classes: dict[str, NetClass]
    net_to_class: dict[str, str]
    min_clearance_nm: int
    min_track_width_nm: int
    min_copper_edge_clearance_nm: int
    min_hole_to_hole_nm: int
    min_through_hole_diameter_nm: int
    min_via_diameter_nm: int
    min_via_annular_width_nm: int
    allow_blind_buried_vias: bool
    allow_microvias: bool
    unclassified_severities: tuple[str, ...] = field(default=())

    @property
    def default(self) -> NetClass:
        return self.classes["Default"]

    def for_net(self, net_name: str) -> NetClass:
        """The netclass governing ``net_name``, falling back to Default."""
        return self.classes.get(self.net_to_class.get(net_name, "Default"), self.default)


def _nm(value: float | None, fallback_nm: int) -> int:
    if value is None:
        return fallback_nm
    nm = mm_to_nm(value)
    return nm if nm > 0 else fallback_nm


def load_design_rules(project_path: Path) -> DesignRules:
    """Read the effective design rules from a ``.kicad_pro`` project file.

    Falls back to KiCad's own defaults for anything the project omits, so a board with a
    sparse project file still yields a usable rule set rather than a crash.
    """
    data = json.loads(project_path.read_text(encoding="utf-8"))
    settings = data.get("board", {}).get("design_settings", {})
    raw_rules = settings.get("rules", {})
    net_settings = data.get("net_settings", {})

    classes: dict[str, NetClass] = {}
    for entry in net_settings.get("classes", []):
        name = entry.get("name", "Default")
        classes[name] = NetClass(
            name=name,
            clearance_nm=_nm(entry.get("clearance"), 200_000),
            track_width_nm=_nm(entry.get("track_width"), 250_000),
            via_diameter_nm=_nm(entry.get("via_diameter"), 800_000),
            via_drill_nm=_nm(entry.get("via_drill"), 400_000),
        )
    if "Default" not in classes:
        classes["Default"] = NetClass("Default", 200_000, 250_000, 800_000, 400_000)

    net_to_class: dict[str, str] = {}
    for cls_name, nets in (net_settings.get("netclass_assignments") or {}).items():
        # KiCad stores this either as {netclass: [nets]} or {net: netclass}; tolerate both.
        if isinstance(nets, list):
            for net in nets:
                net_to_class[net] = cls_name
        elif isinstance(nets, str):
            net_to_class[cls_name] = nets

    known = set(CANONICAL_SEVERITIES)
    unclassified = tuple(sorted(set(settings.get("rule_severities", {})) - known))

    return DesignRules(
        classes=classes,
        net_to_class=net_to_class,
        min_clearance_nm=_nm(raw_rules.get("min_clearance"), classes["Default"].clearance_nm),
        min_track_width_nm=_nm(raw_rules.get("min_track_width"), 150_000),
        min_copper_edge_clearance_nm=_nm(raw_rules.get("min_copper_edge_clearance"), 10_000),
        min_hole_to_hole_nm=_nm(raw_rules.get("min_hole_to_hole"), 250_000),
        min_through_hole_diameter_nm=_nm(raw_rules.get("min_through_hole_diameter"), 300_000),
        min_via_diameter_nm=_nm(raw_rules.get("min_via_diameter"), 500_000),
        min_via_annular_width_nm=_nm(raw_rules.get("min_via_annular_width"), 50_000),
        allow_blind_buried_vias=bool(raw_rules.get("allow_blind_buried_vias", False)),
        allow_microvias=bool(raw_rules.get("allow_microvias", False)),
        unclassified_severities=unclassified,
    )


def rules_hash(project_path: Path) -> str:
    """SHA-256 of the rule-bearing parts of a project file.

    Deliberately narrow: only ``design_settings.rules``, ``rule_severities`` and
    ``net_settings`` participate, so cosmetic project edits (window positions, plot options)
    do not invalidate a run's comparability.
    """
    data = json.loads(project_path.read_text(encoding="utf-8"))
    settings = data.get("board", {}).get("design_settings", {})
    payload = {
        "rules": settings.get("rules", {}),
        "rule_severities": settings.get("rule_severities", {}),
        "net_settings": data.get("net_settings", {}),
    }
    blob = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()
