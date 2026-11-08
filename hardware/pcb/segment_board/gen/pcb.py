#!/usr/bin/env python3
"""Create the starting PCB (segment_board.kicad_pcb) from the schematic netlist, with an automatic
first placement of every part.

    python3 gen/pcb.py            # refuses to overwrite an existing board
    python3 gen/pcb.py --force    # start over (loses manual layout work!)

After that the board is edited by hand in KiCad; schematic changes come in with F8 (footprints are
matched by symbol UUID, see schematic.py). Close KiCad before running this.

Board: 100 x 120 mm, 4 layers (F.Cu signal / In1 GND zone / In2 power / B.Cu signal).

Floor plan (board coordinates, mm, origin top-left):
- Raspberry Pi 4/5 face down on the right (x 44..100, y 35..120), ports at the bottom edge.
  Under the Pi only SMD parts (>= 15 mm free): power, I2C chips, comparators, RS-485.
- ESP32-S3 module at the top edge (antenna outside the Pi shadow, keep-out from the footprint),
  USB-C on the right edge, buttons/jumpers/expansion headers around it (reachable from above).
- Top-left: power inputs/outputs (XT30, MSTB), fuse, reverse-polarity MOSFET, TVS, power LEDs.
- Left: arm connectors in 4 columns; the connection LED (+ its resistor) sits in the gap after
  each connector.
Automatic placement = fixed parts first, then clusters (each IC with the passives that share a
signal net with it) packed into their region without overlapping courtyards. It is a starting
point: rotate/align by hand afterwards.
"""
import subprocess
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

import numpy as np
import pcbnew

from sexp import find, first, parse

HERE = Path(__file__).resolve().parent
PRJ = HERE.parent
BOARD = PRJ / "segment_board.kicad_pcb"
FPLIB = Path("/usr/share/kicad/footprints")
OX, OY = 50.0, 50.0          # board top-left corner on the page (mm)
W, H = 100.0, 120.0
GRID = 0.25                  # placement grid (mm)
GAP = 0.25                   # extra spacing between courtyards (mm)

PI_PIN1 = (48.77, 43.37)     # Pi edge x 44.0..100, y 35.0..120 (ports at the bottom edge)
PI_HOLES = [(PI_PIN1[0] - 1.27, PI_PIN1[1] - 4.87), (PI_PIN1[0] + 47.73, PI_PIN1[1] - 4.87),
            (PI_PIN1[0] - 1.27, PI_PIN1[1] + 53.13), (PI_PIN1[0] + 47.73, PI_PIN1[1] + 53.13)]
M3_HOLES = [(4, 4), (96, 4), (4, 116), (96, 116)]
ESP32 = (68.0, 12.75, 0)     # module top edge (antenna) on the board edge y = 0
ANTENNA_KEEPOUT = (43.5, 0, 92.5, 7.0)   # footprint courtyard: module + 48 mm wide antenna area

FIXED = {   # connector value -> (x, y, rotation) of the footprint anchor (pin 1)
    "XT30 7.4V SERVO IN": (17.7, 12.5, 0),
    "MSTB 5.1V LOGIC IN": (30.0, 7.0, 0),
    "XT30 7.4V SERVO OUT (next board)": (4.0, 19.0, 0),
    "MSTB 5.1V LOGIC OUT (next board)": (30.0, 22.0, 0),
    "15A blade": (4.5, 24.5, 0),
    "USB-C": (96.3, 20.0, 90),
}
COLUMNS = [   # (x of the pin column, y start, connectors top -> bottom), rotated 270 (pins downwards)
    (4.5, 31.0, [f"SERVO {n}" for n in (1, 2, 3, 4, 5)]),
    (15.0, 31.0, [f"SERVO {n}" for n in (6, 7, 8, 9)] + ["TOUCH 1", "TOUCH 2"]),
    (25.5, 31.0, [f"ENCODER {n}" for n in (1, 2, 3, 4)] + ["E-STOP (NC contact)"]),
    (36.0, 31.0, ["LINK RS-485 (Y Z A B GND)"] + [f"ENCODER {n}" for n in (5, 6)]),
]
LED_GAP = 2.5                # space after each connector for its connection LED + resistor
DETECT = {"SERVO": "SERVO", "ENCODER": "ENC", "TOUCH": "TOUCH"}   # connector prefix -> LED net name

REGIONS = {   # name -> list of rectangles (x0, y0, x1, y1), tried in order
    "topleft": [(0.5, 0.5, 41.5, 30.5)],
    "top": [(42.0, 7.0, 99.5, 34.5)],
    "pi": [(51.0, 35.5, 99.5, 119.5), (41.5, 93.0, 51.0, 119.5)],
}
SHEET_REGION = {"/Power": "pi", "/ESP32": "top", "/I2C": "pi", "/Arm connectors": "pi", "/Raspberry": "pi"}
GLOBAL_NETS = {"GND", "+3V3", "V5_LOGIC", "VSERVO", "I2C_SDA", "I2C_SCL"}
# cluster order inside a region (first = placed first = closest to the region's top-left)
ANCHOR_ORDER = ["Q1", "U1", "U2", "U3", "U4", "U5", "U6", "U7", "U9", "U10", "U11", "U17",
                "U12", "U13", "U14", "U15", "U16", "U8"]


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


def extent(fp):
    """courtyard (or body) box of a footprint as placed: (x0, y0, x1, y1) in page mm"""
    crt = fp.GetCourtyard(pcbnew.F_CrtYd)
    bb = crt.BBox() if crt.OutlineCount() else fp.GetBoundingBox(False, False)
    return (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()), pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))


class Grid:
    def __init__(self):
        self.occ = np.zeros((int(H / GRID) + 1, int(W / GRID) + 1), dtype=bool)

    def _idx(self, x0, y0, x1, y1):
        return (max(0, int(np.floor(y0 / GRID))), min(self.occ.shape[0], int(np.ceil(y1 / GRID))),
                max(0, int(np.floor(x0 / GRID))), min(self.occ.shape[1], int(np.ceil(x1 / GRID))))

    def block(self, x0, y0, x1, y1):
        r0, r1, c0, c1 = self._idx(x0, y0, x1, y1)
        self.occ[r0:r1, c0:c1] = True

    def free(self, x0, y0, x1, y1):
        if x0 < 0 or y0 < 0 or x1 > W or y1 > H:
            return False
        r0, r1, c0, c1 = self._idx(x0, y0, x1, y1)
        return not self.occ[r0:r1, c0:c1].any()

    def find(self, rect, w, h):
        x0, y0, x1, y1 = rect
        y = y0
        while y + h <= y1:
            x = x0
            while x + w <= x1:
                if self.free(x, y, x + w, y + h):
                    return x, y
                x += GRID * 2
            y += GRID * 2
        return None


def main(force):
    if BOARD.exists() and not force:
        raise SystemExit(f"{BOARD.name} exists - edit it in KiCad (F8 to sync). Use --force to start over.")
    comps, pins = netlist()
    by_ref = {c["ref"]: c for c in comps}
    nets_of = defaultdict(set)
    for (ref, _pin), net in pins.items():
        nets_of[ref].add(net)

    board = pcbnew.BOARD()
    board.SetCopperLayerCount(4)
    ds = board.GetDesignSettings()
    ds.SetAuxOrigin(mm(0, H))
    ds.SetGridOrigin(mm(0, H))
    for layer in (pcbnew.In1_Cu, pcbnew.In2_Cu):
        board.SetLayerType(layer, pcbnew.LT_POWER)
    for a, b in zip([(0, 0), (W, 0), (W, H), (0, H)], [(W, 0), (W, H), (0, H), (0, 0)]):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(mm(*a))
        s.SetEnd(mm(*b))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(pcbnew.FromMM(0.1))
        board.Add(s)
    for name in sorted(set(pins.values())):
        board.Add(pcbnew.NETINFO_ITEM(board, name))

    fps = {}
    for c in comps:
        fp = load_fp(c["footprint"])
        fp.SetReference(c["ref"])
        fp.SetValue(c["value"])
        fp.SetPath(pcbnew.KIID_PATH(c["path"].rstrip("/")))
        for k, v in c["fields"].items():
            if k in ("LCSC", "Note") and v:
                fp.SetProperty(k, v)
        for pad in fp.Pads():
            net = pins.get((c["ref"], pad.GetNumber()))
            if net:
                pad.SetNet(board.FindNet(net))
        board.Add(fp)
        fps[c["ref"]] = fp

    grid = Grid()
    grid.block(*ANTENNA_KEEPOUT)
    placed = set()

    def put(ref, x, y, rot=0.0, mark=True):
        fp = fps[ref]
        fp.SetOrientationDegrees(rot)
        fp.SetPosition(mm(x, y))
        placed.add(ref)
        if mark:
            x0, y0, x1, y1 = extent(fp)
            grid.block(x0 - OX - GAP, y0 - OY - GAP, x1 - OX + GAP, y1 - OY + GAP)

    def put_in_box(ref, bx, by):
        """place so that the footprint's courtyard box starts at (bx, by)"""
        fp = fps[ref]
        fp.SetOrientationDegrees(0)
        fp.SetPosition(mm(0, 0))
        x0, y0, _, _ = extent(fp)
        put(ref, bx - (x0 - OX), by - (y0 - OY))

    def size(ref):
        fp = fps[ref]
        fp.SetOrientationDegrees(0)
        x0, y0, x1, y1 = extent(fp)
        return x1 - x0 + GAP, y1 - y0 + GAP

    by_value = defaultdict(list)
    for c in comps:
        by_value[c["value"]].append(c["ref"])

    # ---- fixed mechanics, modules and connectors -------------------------------------------
    for ref, (x, y) in zip(sorted(by_value["M3"], key=lambda r: int(r[1:])), M3_HOLES):
        put(ref, x, y)
    for ref, (x, y) in zip(sorted(by_value["M2.5 Pi"], key=lambda r: int(r[1:])), PI_HOLES):
        put(ref, x, y)
    for c in comps:
        if c["footprint"].endswith("RPi_Socket_2x20_FaceDown"):
            put(c["ref"], *PI_PIN1, 0)
            grid.block(PI_PIN1[0] - 4.3, PI_PIN1[1] - 1.8, PI_PIN1[0] + 1.8, PI_PIN1[1] + 50.1)
        if c["footprint"].startswith("RF_Module:ESP32"):
            put(c["ref"], *ESP32, mark=False)
            grid.block(ESP32[0] - 9.6, 0, ESP32[0] + 9.6, 26.5)
        if c["value"] in FIXED:
            put(c["ref"], *FIXED[c["value"]])

    led_of = {}   # connection LED name -> (LED ref, resistor ref)
    for c in comps:
        for net in nets_of[c["ref"]]:
            if net.startswith("LED_") and net.endswith("_A"):
                name = net[4:-2]
                d = led_of.setdefault(name, [None, None])
                d[0 if c["ref"].startswith("D") else 1] = c["ref"]
    for x, y, values in COLUMNS:
        for v in values:
            ref = by_value[v][0]
            fp = fps[ref]
            if fp.GetFPID().GetLibNickname() == "Connector_Phoenix_MC":
                before, length = 3.01, fp.GetPadCount() * 3.5 + 2.48
            else:
                bb = fp.GetBoundingBox(False, False)
                before, length = -pcbnew.ToMM(bb.GetLeft()) + 0.5, pcbnew.ToMM(bb.GetWidth()) + 1.0
            put(ref, x, y + before, 270)
            y += length
            kind, _, num = v.partition(" ")
            name = f"{DETECT.get(kind, '')}{num}"
            if name in led_of and all(led_of[name]):
                led, res = led_of[name]
                put(led, x - 0.9, y + LED_GAP / 2, 0)
                put(res, x + 2.6, y + LED_GAP / 2, 0)
                y += LED_GAP

    # ---- clusters ---------------------------------------------------------------------------
    anchors = [r for r in ANCHOR_ORDER if r in fps] + sorted(
        r for r in fps if r[0] in "UQ" and r not in ANCHOR_ORDER)
    cluster = defaultdict(list)
    forced = {}
    for c in comps:   # power LEDs and jumpers stay where they can be seen/reached
        r, nets = c["ref"], nets_of[c["ref"]]
        if r in placed or r in anchors:
            continue
        if r.startswith("JP") or c["value"] in ("BOOT", "RESET") or r in ("J7", "J8"):
            forced[r] = "top"
        elif any(n.endswith("_LED") for n in nets):
            forced[r] = "top" if any(n.startswith(("PI_", "STATUS")) for n in nets) else "topleft"
        elif c["value"].startswith("SMBJ") or r == "D1":
            forced[r] = "topleft"
    for c in comps:
        r = c["ref"]
        if r in placed or r in anchors or r in forced:
            continue
        mine = nets_of[r] - GLOBAL_NETS
        best, score = None, 0
        for a in anchors:
            s = len(mine & nets_of[a])
            if s > score:
                best, score = a, s
        cluster[best or f"misc{c['sheet']}"].append(r)

    def region_of(ref_or_key):
        if ref_or_key == "Q1":
            return "topleft"
        sheet = by_ref[ref_or_key]["sheet"] if ref_or_key in by_ref else ref_or_key[4:]
        return next((reg for k, reg in SHEET_REGION.items() if sheet.startswith(k)), "pi")

    def place_block(refs, region):
        """shelf-pack refs (first = anchor) into one block, then find room for the block"""
        sizes = {r: size(r) for r in refs}
        area = sum(w * h for w, h in sizes.values())
        width = max(max(w for w, _ in sizes.values()), (area * 1.4) ** 0.5)
        pos, x, y, row = {}, 0.0, 0.0, 0.0
        for r in refs:
            w, h = sizes[r]
            if x > 0 and x + w > width:
                x, y, row = 0.0, y + row, 0.0
            pos[r] = (x, y)
            x += w
            row = max(row, h)
        bw = max(px + sizes[r][0] for r, (px, _) in pos.items())
        bh = max(py + sizes[r][1] for r, (_, py) in pos.items())
        for reg in [region] + [k for k in ("pi", "top", "topleft") if k != region]:
            for rect in REGIONS[reg]:
                spot = grid.find(rect, bw, bh)
                if spot:
                    for r, (px, py) in pos.items():
                        put_in_box(r, spot[0] + px, spot[1] + py)
                    return reg
        return None

    leftovers = []
    jobs = []
    for a in anchors:
        if a not in placed:
            jobs.append(([a] + sorted(cluster.pop(a, []), key=lambda r: -size(r)[0] * size(r)[1]), region_of(a)))
        elif cluster.get(a):   # anchor already fixed (ESP32): its passives go to its region
            jobs.append((sorted(cluster.pop(a), key=lambda r: -size(r)[0] * size(r)[1]), region_of(a)))
    for key, refs in cluster.items():
        jobs.append((sorted(refs, key=lambda r: -size(r)[0] * size(r)[1]), region_of(key)))
    for reg in ("topleft", "top"):
        refs = sorted((r for r, g in forced.items() if g == reg), key=lambda r: -size(r)[0] * size(r)[1])
        for r in refs:   # one by one: they are small and must find gaps
            jobs.insert(0, ([r], reg))
    for refs, reg in jobs:
        if not place_block(refs, reg):
            # block too big: try the parts one by one
            for r in refs:
                if not place_block([r], reg):
                    leftovers.append(r)

    # anything that did not fit: next to the board
    x = W + 10
    for r in leftovers:
        w, h = size(r)
        put_in_box(r, x, 0)
        x += w + 1

    z = pcbnew.ZONE(board)
    z.SetLayer(pcbnew.In1_Cu)
    z.SetNet(board.FindNet("GND"))
    z.SetZoneName("GND In1.Cu")
    z.SetLocalClearance(pcbnew.FromMM(0.25))
    z.SetMinThickness(pcbnew.FromMM(0.2))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    ol = z.Outline()
    ol.NewOutline()
    for px, py in ((0.3, 0.3), (W - 0.3, 0.3), (W - 0.3, H - 0.3), (0.3, H - 0.3)):
        ol.Append(mm(px, py))
    board.Add(z)

    pcbnew.SaveBoard(str(BOARD), board)
    print(f"wrote {BOARD.name}: {len(comps)} footprints, {len(comps) - len(leftovers)} on the board"
          + (f", did not fit: {' '.join(leftovers)}" if leftovers else ""))


if __name__ == "__main__":
    main("--force" in sys.argv)
