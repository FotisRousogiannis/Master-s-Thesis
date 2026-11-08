#!/usr/bin/env python3
"""Design rules and net classes in segment_board.kicad_pro (safe to re-run; does not touch the layout).

    python3 gen/rules.py

Rules: JLCPCB 4-layer capabilities with margin (track/space >= 0.1 mm, via >= 0.45/0.2 mm).
Net classes (track width = default when routing; widths for 1 oz outer copper, ~10-20 C rise):
  Power_Servo  VS_*, VSERVO      up to ~10 A  -> 2.5 mm tracks + In2 / polygon pours
  Servo_Ch     SERVO*_GND, SERVO*_SIG is signal; the per-servo ground return carries up to ~4 A (stall)
  Power_Logic  5 V / 3.3 V rails, GND
Clearances are kept at <= 0.2 mm so that the fine-pitch eFuse pads (0.175-0.26 mm gaps) pass DRC.
"""
import json
from pathlib import Path

PRO = Path(__file__).resolve().parent.parent / "segment_board.kicad_pro"

RULES = {
    "min_clearance": 0.1, "min_track_width": 0.1, "min_connection": 0.1,
    "min_via_diameter": 0.45, "min_via_annular_width": 0.1, "min_through_hole_diameter": 0.2,
    "min_hole_to_hole": 0.25, "min_hole_clearance": 0.2, "min_copper_edge_clearance": 0.3,
    "min_silk_clearance": 0.0, "min_text_height": 0.8, "min_text_thickness": 0.12,
    "allow_blind_buried_vias": False, "allow_microvias": False,
}
TRACKS = [0.0, 0.2, 0.3, 0.6, 1.0, 1.5, 2.5]
VIAS = [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.6, "drill": 0.3}, {"diameter": 0.8, "drill": 0.4}]


def cls(name, clearance, track, via, drill):
    return {"name": name, "clearance": clearance, "track_width": track, "via_diameter": via, "via_drill": drill,
            "microvia_diameter": 0.3, "microvia_drill": 0.1, "diff_pair_width": 0.2, "diff_pair_gap": 0.25,
            "diff_pair_via_gap": 0.25, "wire_width": 6, "bus_width": 12, "line_style": 0,
            "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)"}


CLASSES = [cls("Default", 0.15, 0.2, 0.6, 0.3),
           cls("Power_Servo", 0.2, 2.5, 0.8, 0.4),
           cls("Servo_Ch", 0.2, 1.0, 0.8, 0.4),
           cls("Power_Logic", 0.15, 0.6, 0.6, 0.3)]
PATTERNS = [("Power_Servo", "VS_*"), ("Power_Servo", "VSERVO"),
            ("Servo_Ch", "SERVO*_GND"),
            ("Power_Logic", "V5_*"), ("Power_Logic", "V_BUCK"), ("Power_Logic", "+3V3"),
            ("Power_Logic", "VBUS_USB"), ("Power_Logic", "PI_5V*"), ("Power_Logic", "GND")]


def main():
    d = json.loads(PRO.read_text())
    ds = d.setdefault("board", {}).setdefault("design_settings", {})
    ds.setdefault("rules", {}).update(RULES)
    ds["track_widths"] = TRACKS
    ds["via_dimensions"] = VIAS
    ns = d.setdefault("net_settings", {})
    ns["classes"] = CLASSES
    ns["netclass_patterns"] = [{"netclass": c, "pattern": p} for c, p in PATTERNS]
    ns.setdefault("meta", {"version": 3})
    PRO.write_text(json.dumps(d, indent=2) + "\n")
    print(f"updated {PRO.name}: {len(CLASSES)} net classes, {len(PATTERNS)} patterns")


if __name__ == "__main__":
    main()
