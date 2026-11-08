#!/usr/bin/env python3
"""Create the starting PCB (segment_board.kicad_pcb) from the schematic netlist.

    python3 gen/pcb.py            # refuses to overwrite an existing board
    python3 gen/pcb.py --force    # start over (loses manual layout work!)

Run ONCE. After that the board is edited by hand in KiCad; schematic changes are brought in with
"Update PCB from Schematic" (F8), which matches footprints by symbol UUID (stable, see schematic.py).

What it sets up:
- 100 x 100 mm outline, 4 copper layers (F.Cu signal / In1 GND / In2 power / B.Cu signal),
  GND zone on In1.Cu, aux origin at the lower-left corner (JLC position files)
- 4 x M3 holes in the corners, Raspberry Pi socket + its 4 x M2.5 holes (Pi 4/5 face down,
  right half of the board, ports at the bottom edge)
- ESP32-S3 module with its antenna at the top edge, outside the Pi shadow (antenna keep-out
  comes with the footprint)
- arm connectors in 4 columns on the left (outside the Pi shadow, plugs reachable from above),
  power inputs on the top edge
- every other part placed next to the board, grouped by schematic sheet, for manual placement
Needs KiCad's Python module (pcbnew) and kicad-cli; tested with KiCad 7, opens in KiCad 9.
"""
import re
import subprocess
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

import pcbnew

from sexp import find, first, parse

HERE = Path(__file__).resolve().parent
PRJ = HERE.parent
BOARD = PRJ / "segment_board.kicad_pcb"
FPLIB = Path("/usr/share/kicad/footprints")
OX, OY = 50.0, 50.0          # board top-left corner on the page (mm)
W = H = 100.0

# Raspberry Pi (face down): socket pin 1 position, see lib footprint RPi_Socket_2x20_FaceDown
PI_PIN1 = (48.77, 23.37)     # Pi edge x 44.0..100, y 15.0..100 (ports at the bottom edge)
PI_HOLES = [(PI_PIN1[0] - 1.27, PI_PIN1[1] - 4.87), (PI_PIN1[0] + 47.73, PI_PIN1[1] - 4.87),
            (PI_PIN1[0] - 1.27, PI_PIN1[1] + 53.13), (PI_PIN1[0] + 47.73, PI_PIN1[1] + 53.13)]
M3_HOLES = [(4, 4), (96, 4), (4, 96), (96, 96)]
ESP32 = (68.0, 12.75, 0)     # module top edge (antenna) on the board edge y = 0

# connector columns: (x of pin column, y start, [values top -> bottom]); rotated 270 deg (pins downwards)
COLUMNS = [
    (4.5, 17.0, [f"SERVO {n}" for n in (1, 2, 3, 4, 5)]),
    (15.0, 17.0, [f"SERVO {n}" for n in (6, 7, 8, 9)] + ["TOUCH 1", "TOUCH 2"]),
    (25.5, 17.0, [f"ENCODER {n}" for n in (1, 2, 3, 4)] + ["E-STOP (NC contact)"]),
    (36.0, 17.0, [f"ENCODER {n}" for n in (5, 6)] + ["LINK RS-485 (Y Z A B GND)"]),
]
FIXED = {   # value -> (x, y, rotation) for pin 1 / footprint anchor, board coordinates
    "XT30 7.4V SERVO IN": (17.7, 12.5, 0),
    "MSTB 5.1V LOGIC IN": (30.0, 7.0, 0),
    "USB-C": (96.3, 40.0, 90),
}


def mm(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(OX + x), pcbnew.FromMM(OY + y))


def netlist():
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "board.net"
        subprocess.run(["kicad-cli", "sch", "export", "netlist", "-o", str(out), str(PRJ / "segment_board.kicad_sch")],
                       check=True, capture_output=True)
        root = parse(out.read_text(encoding="utf-8"))
    comps = []
    for c in find(first(root, "components"), "comp"):
        sp = first(c, "sheetpath")
        fields = {first(f, "name")[1]: (f[2] if len(f) > 2 else "") for f in find(first(c, "fields") or [], "field")}
        comps.append({"ref": first(c, "ref")[1], "value": first(c, "value")[1], "footprint": first(c, "footprint")[1],
                      "path": first(sp, "tstamps")[1] + first(c, "tstamps")[1], "sheet": first(sp, "names")[1],
                      "fields": fields})
    pins = {}
    for n in find(first(root, "nets"), "net"):
        name = first(n, "name")[1]
        for node in find(n, "node"):
            pins[(first(node, "ref")[1], first(node, "pin")[1])] = name
    return comps, pins


def load_fp(lib_id):
    lib, name = lib_id.split(":")
    path = PRJ / "lib" / "segment_board.pretty" if lib == "segment_board" else FPLIB / f"{lib}.pretty"
    fp = pcbnew.FootprintLoad(str(path), name)
    if fp is None:
        raise SystemExit(f"footprint not found: {lib_id}")
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def outline(board):
    pts = [(0, 0), (W, 0), (W, H), (0, H)]
    for a, b in zip(pts, pts[1:] + pts[:1]):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(mm(*a))
        s.SetEnd(mm(*b))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(pcbnew.FromMM(0.1))
        board.Add(s)


def zone(board, layer, netname, inset=0.3):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(board.FindNet(netname))
    z.SetZoneName(f"{netname} {board.GetLayerName(layer)}")
    z.SetLocalClearance(pcbnew.FromMM(0.25))
    z.SetMinThickness(pcbnew.FromMM(0.2))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    ol = z.Outline()
    ol.NewOutline()
    for x, y in ((inset, inset), (W - inset, inset), (W - inset, H - inset), (inset, H - inset)):
        ol.Append(mm(x, y))
    board.Add(z)


def text(board, s, x, y, size=1.5, layer=pcbnew.Cmts_User):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(s)
    t.SetPosition(mm(x, y))
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
    t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
    board.Add(t)


def main(force):
    if BOARD.exists() and not force:
        raise SystemExit(f"{BOARD.name} exists - edit it in KiCad (F8 to sync). Use --force to start over.")
    comps, pins = netlist()
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(4)
    ds = board.GetDesignSettings()
    ds.SetAuxOrigin(mm(0, H))
    ds.SetGridOrigin(mm(0, H))
    for i, (layer, name) in enumerate(((pcbnew.F_Cu, "F.Cu"), (pcbnew.In1_Cu, "In1.Cu"), (pcbnew.In2_Cu, "In2.Cu"),
                                       (pcbnew.B_Cu, "B.Cu"))):
        board.SetLayerType(layer, pcbnew.LT_POWER if layer in (pcbnew.In1_Cu, pcbnew.In2_Cu) else pcbnew.LT_SIGNAL)
    outline(board)

    for name in sorted(set(pins.values())):
        board.Add(pcbnew.NETINFO_ITEM(board, name))

    by_value = defaultdict(list)
    for c in comps:
        by_value[c["value"]].append(c)
    placed = {}

    def place(c, x, y, rot):
        placed[c["ref"]] = (x, y, rot)

    # fixed mechanical parts
    for c, (x, y) in zip(sorted(by_value["M3"], key=lambda c: c["ref"]), M3_HOLES):
        place(c, x, y, 0)
    for c, (x, y) in zip(sorted(by_value["M2.5 Pi"], key=lambda c: c["ref"]), PI_HOLES):
        place(c, x, y, 0)
    for c in comps:
        if c["footprint"].endswith("RPi_Socket_2x20_FaceDown"):
            place(c, *PI_PIN1, 0)
        if c["footprint"].startswith("RF_Module:ESP32"):
            place(c, *ESP32)
        if c["value"] in FIXED:
            place(c, *FIXED[c["value"]])
    for x, y0, values in COLUMNS:
        y = y0
        for v in values:
            c = by_value[v][0]
            pads = load_fp(c["footprint"]).GetPadCount()
            width = pads * 3.5 + 2.48           # MCV 1,5/n-G-3.5 courtyard (12.98 mm for 3 pins)
            place(c, x, y + 3.01, 270)          # courtyard starts 3.01 mm before pin 1
            y += width

    # everything else: next to the board, one block per sheet
    sheets = defaultdict(list)
    for c in comps:
        if c["ref"] not in placed:
            sheets[c["sheet"]].append(c)
    bx, col_x = W + 15, W + 15
    for sheet, items in sheets.items():
        text(board, sheet.strip("/") or "root", col_x, -6, 2.0)
        x, y, row_h, max_x = col_x, 0.0, 0.0, col_x
        for c in sorted(items, key=lambda c: (c["footprint"], c["ref"])):
            fp = load_fp(c["footprint"])
            bb = fp.GetBoundingBox(False, False)
            w, h = pcbnew.ToMM(bb.GetWidth()) + 1.5, pcbnew.ToMM(bb.GetHeight()) + 1.5
            if x + w > col_x + 60:
                x, y, row_h = col_x, y + row_h, 0.0
            ax = x - pcbnew.ToMM(bb.GetLeft())
            ay = y - pcbnew.ToMM(bb.GetTop())
            place(c, ax, ay, 0)
            x += w
            row_h = max(row_h, h)
            max_x = max(max_x, x)
        col_x = max_x + 10

    for c in comps:
        fp = load_fp(c["footprint"])
        fp.SetReference(c["ref"])
        fp.SetValue(c["value"])
        fp.SetPath(pcbnew.KIID_PATH(c["path"].rstrip("/")))
        for k, v in c["fields"].items():
            if k in ("LCSC", "Note") and v:
                fp.SetProperty(k, v)
        x, y, rot = placed[c["ref"]]
        fp.SetPosition(mm(x, y))
        fp.SetOrientationDegrees(rot)
        for pad in fp.Pads():
            net = pins.get((c["ref"], pad.GetNumber()))
            if net:
                pad.SetNet(board.FindNet(net))
        board.Add(fp)

    zone(board, pcbnew.In1_Cu, "GND")
    text(board, "Placement proposal: connectors left (outside the Pi), Pi face down on the right, "
                "ESP32 antenna at the top edge. Move freely.", 0, H + 4, 1.2)
    # zones are filled in KiCad (B) - filling from the headless API crashes KiCad 7
    pcbnew.SaveBoard(str(BOARD), board)
    on = sum(1 for c in comps if placed[c["ref"]][0] <= W)
    print(f"wrote {BOARD.name}: {len(comps)} footprints ({on} on the board), {len(set(pins.values()))} nets")


if __name__ == "__main__":
    main("--force" in sys.argv)
