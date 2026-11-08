#!/usr/bin/env python3
"""Project footprints (lib/segment_board.pretty), generated from the manufacturer land patterns.

    python3 gen/footprints.py

TI_RGE0024_2EP     TPS259827 (TI RGE0024M, VQFN-24 4x4, two exposed pads)   tps25982.pdf p.55-57
TI_RPW0010A        TPS259472A (TI RPW0010A, VQFN-HR-10 2x2)                  tps25947.pdf p.72-74
L_4x4mm_SMNR4020   2.2 uH 3.4 A inductor, LCSC C135262 (land pattern from LCSC/EasyEDA)
RPi_Socket_2x20_FaceDown
                   2x20 female socket for a Raspberry Pi mounted face down (upside down) on top of
                   this board: the pin columns are mirrored compared to a normal HAT socket. Pi 4/5
                   outline and its M2.5 holes are drawn on User.Drawings for placement.
Coordinates: mm, KiCad convention (y down), origin = package centre (socket: pin 1).
"""
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "lib" / "segment_board.pretty"

HEAD = """(footprint "{name}" (version 20221018) (generator segment_board_gen)
  (layer "F.Cu")
  (descr "{descr}")
  (tags "{tags}")
  (attr {attr})
  (fp_text reference "REF**" (at 0 {ref_y}) (layer "F.SilkS")
    (effects (font (size 1 1) (thickness 0.15)))
  )
  (fp_text value "{name}" (at 0 {val_y}) (layer "F.Fab")
    (effects (font (size 1 1) (thickness 0.15)))
  )
  (fp_text user "${{REFERENCE}}" (at 0 0) (layer "F.Fab")
    (effects (font (size {fab_font} {fab_font}) (thickness 0.08)))
  )
"""


def f(x):
    return f"{x:.4f}".rstrip("0").rstrip(".")


def line(x1, y1, x2, y2, layer, w):
    return f'  (fp_line (start {f(x1)} {f(y1)}) (end {f(x2)} {f(y2)}) (stroke (width {w}) (type solid)) (layer "{layer}"))\n'


def rect(x1, y1, x2, y2, layer, w):
    return "".join(line(*p, layer, w) for p in ((x1, y1, x2, y1), (x2, y1, x2, y2), (x2, y2, x1, y2), (x1, y2, x1, y1)))


def circle(x, y, r, layer, w):
    return f'  (fp_circle (center {f(x)} {f(y)}) (end {f(x + r)} {f(y)}) (stroke (width {w}) (type solid)) (fill none) (layer "{layer}"))\n'


def smd(num, x, y, w, h, shape="roundrect", extra=""):
    rr = " (roundrect_rratio 0.25)" if shape == "roundrect" else ""
    return (f'  (pad "{num}" smd {shape} (at {f(x)} {f(y)}) (size {f(w)} {f(h)}) '
            f'(layers "F.Cu" "F.Paste" "F.Mask"){rr}{extra})\n')


def smd_poly(num, x, y, pts):
    """custom pad: anchor at (x, y), polygon in coordinates relative to the anchor"""
    p = " ".join(f"(xy {f(a)} {f(b)})" for a, b in pts)
    return (f'  (pad "{num}" smd custom (at {f(x)} {f(y)}) (size 0.2 0.2) (layers "F.Cu" "F.Paste" "F.Mask")\n'
            f'    (options (clearance outline) (anchor rect))\n'
            f'    (primitives (gr_poly (pts {p}) (width 0) (fill yes)))\n  )\n')


def via(num, x, y, drill=0.2, dia=0.45):
    return f'  (pad "{num}" thru_hole circle (at {f(x)} {f(y)}) (size {dia} {dia}) (drill {drill}) (layers "*.Cu"))\n'


def write(name, body, descr, tags, attr="smd", ref_y=-3, val_y=3, fab_font=0.5):
    OUT.mkdir(parents=True, exist_ok=True)
    text = HEAD.format(name=name, descr=descr, tags=tags, attr=attr, ref_y=ref_y, val_y=val_y, fab_font=fab_font)
    (OUT / f"{name}.kicad_mod").write_text(text + body + ")\n")


def qfn_outline(half, pin1_mark, crt):
    """fab body + chamfered pin-1 corner, silk corner marks, courtyard"""
    b = ""
    c = 0.5
    b += line(-half + c, -half, half, -half, "F.Fab", 0.1) + line(half, -half, half, half, "F.Fab", 0.1)
    b += line(half, half, -half, half, "F.Fab", 0.1) + line(-half, half, -half, -half + c, "F.Fab", 0.1)
    b += line(-half, -half + c, -half + c, -half, "F.Fab", 0.1)
    s = half + 0.11
    for sx, sy in ((1, -1), (1, 1), (-1, 1)):
        b += line(sx * s, sy * s, sx * (s - 0.35), sy * s, "F.SilkS", 0.12)
        b += line(sx * s, sy * s, sx * s, sy * (s - 0.35), "F.SilkS", 0.12)
    b += line(-s, -s, -s + 0.35, -s, "F.SilkS", 0.12)
    b += circle(*pin1_mark, 0.12, "F.SilkS", 0.24)
    b += rect(-crt, -crt, crt, crt, "F.CrtYd", 0.05)
    return b


# ------------------------------------------------------------------------------------------
def rge0024():
    """TI RGE0024M land pattern (top view): pins 1-6 left (top->bottom), 7-12 bottom (left->right),
    13-18 right (bottom->top), 19-24 top (right->left). EP25 = IN (2.7 x 1.45, upper),
    EP26 = GND (2.7 x 0.85, lower), 0.39 mm apart. 0.2 mm thermal vias as in the TI example."""
    pitch, row, pl, pw = 0.5, 1.9125, 0.575, 0.24     # pad rows 3.825 mm centre-to-centre (TI), 0.275 mm to the EPs
    body = ""
    for i in range(6):
        t = -1.25 + i * pitch
        body += smd(1 + i, -row, t, pl, pw)            # left
        body += smd(7 + i, t, row, pw, pl)             # bottom
        body += smd(13 + i, row, -t, pl, pw)           # right, bottom -> top
        body += smd(19 + i, -t, -row, pw, pl)          # top, right -> left
    paste = " (solder_paste_margin_ratio -0.2)"
    body += smd(25, 0, -0.625, 2.7, 1.45, "rect", paste)
    body += smd(26, 0, 0.925, 2.7, 0.85, "rect", paste)
    for x in (-1.1, 0, 1.1):
        body += via(25, x, -1.05) + via(25, x, -0.2)   # IN pad: two rows
        body += via(26, x, 0.925)                      # GND pad
    body += qfn_outline(2.0, (-2.6, -1.95), 2.5)
    write("TI_RGE0024_2EP", body,
          "TI RGE0024M VQFN-24 4x4 mm, 0.5 mm pitch, 2 exposed pads (EP25 IN, EP26 GND), TPS25982 eFuse, "
          "https://www.ti.com/lit/ds/symlink/tps25982.pdf", "VQFN QFN 24 TPS25982 eFuse")


def rpw0010a():
    """TI RPW0010A (VQFN-HR) land pattern (top view): 1-4 left (top->bottom), 5/6 = long centre bars
    (left/right), 7-10 right (bottom->top). Corner pins 1, 4, 7, 10 are L-shaped."""
    body = ""
    # side pads 2, 3, 8, 9: 0.6 x 0.25 at x = +-0.9, y = +-0.225
    body += smd(2, -0.9, -0.225, 0.6, 0.25, "rect") + smd(3, -0.9, 0.225, 0.6, 0.25, "rect")
    body += smd(9, 0.9, -0.225, 0.6, 0.25, "rect") + smd(8, 0.9, 0.225, 0.6, 0.25, "rect")
    # centre bars 5 (IN) and 6 (OUT): 0.3 x 2.4 at x = -+0.25
    body += smd(5, -0.25, 0, 0.3, 2.4, "rect") + smd(6, 0.25, 0, 0.3, 2.4, "rect")

    # L-shaped corner pad, drawn for the top-left corner (pin 1) relative to anchor (-0.9, -0.7):
    # horizontal part x -1.2..-0.6, y -0.875..-0.525 ; vertical leg x -0.875..-0.575, y -1.2..-0.525
    def corner(sx, sy):
        pts = [(-0.3, -0.175), (-0.3, 0.175), (0.325, 0.175), (0.325, -0.5), (0.025, -0.5), (0.025, -0.175)]
        return [(sx * -x if sx > 0 else x, sy * -y if sy > 0 else y) for x, y in pts]
    for num, sx, sy in ((1, -1, -1), (4, -1, 1), (7, 1, 1), (10, 1, -1)):
        body += smd_poly(num, sx * 0.9, sy * 0.7, corner(sx, sy))
    body += qfn_outline(1.0, (-1.45, -1.0), 1.5)
    write("TI_RPW0010A", body,
          "TI RPW0010A VQFN-HR-10 2x2 mm, 0.45 mm pitch, TPS25947 eFuse, https://www.ti.com/lit/ds/symlink/tps25947.pdf",
          "VQFN-HR QFN 10 TPS25947 eFuse", ref_y=-2.2, val_y=2.2, fab_font=0.4)


def smnr4020():
    body = smd(1, -1.8, 0, 1.5, 4.0, "rect") + smd(2, 1.8, 0, 1.5, 4.0, "rect")
    body += rect(-2.0, -2.0, 2.0, 2.0, "F.Fab", 0.1)
    body += line(-0.9, -2.11, 0.9, -2.11, "F.SilkS", 0.12) + line(-0.9, 2.11, 0.9, 2.11, "F.SilkS", 0.12)
    body += rect(-2.8, -2.25, 2.8, 2.25, "F.CrtYd", 0.05)
    write("L_4x4mm_SMNR4020", body, "Power inductor 4x4 mm (SMNR4020, LCSC C135262), land pattern from LCSC",
          "inductor 4020 4x4", ref_y=-3.2, val_y=3.2)


def rpi_socket_facedown():
    """Same as KiCad's Raspberry_Pi_Zero_Socketed_THT_FaceDown (pin 1 at origin, odd pins x=0,
    even pins x=-2.54, pin n+2 at +2.54 in y), Pi 4/5 outline: holes at x = -1.27 and 47.73,
    y = -4.87 and 53.13 (58 x 49 mm pattern), board edge 3.5 mm outside the holes."""
    body = ""
    for n in range(1, 41):
        x = 0 if n % 2 else -2.54
        y = (n - 1) // 2 * 2.54
        shape = "rect" if n == 1 else "oval"
        body += (f'  (pad "{n}" thru_hole {shape} (at {f(x)} {f(y)}) (size 1.7 1.7) (drill 1) '
                 f'(layers "*.Cu" "*.Mask"))\n')
    body += rect(-3.81, -1.27, 1.27, 49.53, "F.Fab", 0.1)
    body += rect(-3.92, -1.38, 1.38, 49.64, "F.SilkS", 0.12)
    body += line(1.6, -1.6, 1.6, 0, "F.SilkS", 0.12) + line(1.6, -1.6, 0, -1.6, "F.SilkS", 0.12)
    body += rect(-4.3, -1.8, 1.8, 50.1, "F.CrtYd", 0.05)
    # Raspberry Pi 4/5 outline (85 x 56) and mounting holes, face down above this board
    x0, y0 = -1.27 - 3.5, -4.87 - 3.5
    body += rect(x0, y0, x0 + 56, y0 + 85, "Dwgs.User", 0.15)
    for hx in (-1.27, 47.73):
        for hy in (-4.87, 53.13):
            body += circle(hx, hy, 1.375, "Dwgs.User", 0.15) + circle(hx, hy, 3.0, "Dwgs.User", 0.1)
    body += ('  (fp_text user "Raspberry Pi 4/5, face down (components towards this board)" (at 23 24 90) '
             '(layer "Dwgs.User")\n    (effects (font (size 1.5 1.5) (thickness 0.15)))\n  )\n')
    write("RPi_Socket_2x20_FaceDown", body,
          "2x20 2.54 mm female socket for a Raspberry Pi 4/5 mounted face down on this board (mirrored pin columns). "
          "Pi outline and M2.5 holes (place the 4 MountingHole parts on the circles) on User.Drawings.",
          "raspberry pi socket face down mirrored 2x20", attr="through_hole", ref_y=-3, val_y=51.5, fab_font=1)


def usb_c_hro():
    """Copy of KiCad's USB_C_Receptacle_HRO_TYPE-C-31-M-12 with the shield pads named "S1" (as the pin of the
    Connector:USB_C_Receptacle_USB2.0_16P symbol). KiCad 10 renamed the footprint pads to "SH", which leaves the
    shield unconnected (DRC net_conflict). A project copy keeps symbol and footprint consistent."""
    src = Path("/usr/share/kicad/footprints/Connector_USB.pretty/USB_C_Receptacle_HRO_TYPE-C-31-M-12.kicad_mod")
    dst = OUT / "USB_C_Receptacle_HRO_TYPE-C-31-M-12_S1.kicad_mod"
    if not src.exists():           # keep the committed copy when KiCad 7 libraries are not installed
        return
    text = src.read_text().replace('(pad "SH"', '(pad "S1"').replace(
        '(footprint "USB_C_Receptacle_HRO_TYPE-C-31-M-12"', '(footprint "USB_C_Receptacle_HRO_TYPE-C-31-M-12_S1"')
    dst.write_text(text)


def main():
    usb_c_hro()
    rge0024()
    rpw0010a()
    smnr4020()
    rpi_socket_facedown()
    print(f"wrote {len(list(OUT.glob('*.kicad_mod')))} footprints to {OUT}")


if __name__ == "__main__":
    main()
