"""Write a hierarchical KiCad schematic from a component/net description.

Connectivity is label based: every connected pin gets a short wire stub and a global
label carrying the net name; pins that are deliberately unused get a no-connect flag.
"""

import uuid
from pathlib import Path

from sexp import Q, dump

GRID = 2.54
STUB = 2.54
PROJECT = "segment_board"
PAPERS = [("A3", 420, 297), ("A2", 594, 420), ("A1", 841, 594), ("A0", 1189, 841)]


def uid(*parts):
    return str(uuid.uuid5(uuid.NAMESPACE_URL, "segment_board/" + "/".join(map(str, parts))))


def snap(v):
    return round(round(v / GRID) * GRID, 4)


def font(size=1.27):
    return ["effects", ["font", ["size", size, size]]]


class Part:
    def __init__(self, ref, lib_id, value, nets, footprint="", lcsc="", unit=1, note="", dnp=False):
        self.ref, self.lib_id, self.value = ref, lib_id, value
        self.nets = nets              # {pin number: net name | None (no-connect)}
        self.footprint, self.lcsc, self.unit, self.note, self.dnp = footprint, lcsc, unit, note, dnp
        self.at = (0, 0)


class Sheet:
    def __init__(self, name, filename, title):
        self.name, self.filename, self.title = name, filename, title
        self.parts = []
        self.texts = []

    def add(self, part):
        self.parts.append(part)
        return part

    def text(self, s):
        self.texts.append(s)


class Writer:
    def __init__(self, library, root_title, rev, date):
        self.lib = library
        self.root_title, self.rev, self.date = root_title, rev, date
        self.root_uuid = uid("root")

    # -- geometry ---------------------------------------------------------------------------
    def _extent(self, part):
        """Bounding box of the part including stubs and labels, relative to the symbol origin."""
        pins = self.lib.pins(part.lib_id, part.unit)
        x0, x1, y0, y1 = self.lib.bbox(part.lib_id, part.unit)
        longest = max([len(n) for n in part.nets.values() if n] + [0])
        label = STUB + 3 + longest * 1.1
        left = x0 - (label if any(p[2] == 0 for p in pins.values()) else 6)
        right = x1 + (label if any(p[2] == 180 for p in pins.values()) else 6)
        top = -y1 - (label if any(p[2] == 270 for p in pins.values()) else 8)
        bottom = -y0 + (label if any(p[2] == 90 for p in pins.values()) else 8)
        return left, right, top, bottom

    def _layout(self, sheet):
        for paper, w, h in PAPERS:
            x, y, row_h = 25.4, 40.64, 0
            ok = True
            for part in sheet.parts:
                l, r, t, b = self._extent(part)
                pw, ph = r - l + 5, b - t + 5
                if x + pw > w - 20:
                    x, y, row_h = 25.4, y + row_h, 0
                part.at = (snap(x - l), snap(y - t))
                x += pw
                row_h = max(row_h, ph)
            if y + row_h < h - 30:
                return paper
        return PAPERS[-1][0]

    # -- elements ---------------------------------------------------------------------------
    def _label(self, net, x, y, ang, key):
        just = {0: "left", 180: "right", 90: "left", 270: "right"}[ang]
        return ["global_label", Q(net), ["shape", "passive"], ["at", x, y, ang], ["fields_autoplaced"],
                ["effects", ["font", ["size", 1.27, 1.27]], ["justify", just]], ["uuid", uid("lbl", key)],
                ["property", Q("Intersheetrefs"), Q("${INTERSHEET_REFS}"), ["at", x, y, 0],
                 ["effects", ["font", ["size", 1.27, 1.27]], "hide"]]]

    def _part_items(self, part, sheet_path):
        sx, sy = part.at
        pins = self.lib.pins(part.lib_id, part.unit)
        unknown = set(part.nets) - set(pins)
        if unknown:
            raise ValueError(f"{part.ref}: unknown pins {sorted(unknown)} for {part.lib_id}")
        items = []
        out_dir = {0: (-1, 0, 180), 180: (1, 0, 0), 90: (0, 1, 270), 270: (0, -1, 90)}
        for num, (px, py, ang, etype) in pins.items():
            x, y = round(sx + px, 4), round(sy - py, 4)
            key = (part.ref, part.unit, num)
            net = part.nets.get(num, None)
            if net is None:
                if etype != "no_connect":
                    items.append(["no_connect", ["at", x, y], ["uuid", uid("nc", *key)]])
                continue
            dx, dy, lang = out_dir[ang]
            ex, ey = round(x + dx * STUB, 4), round(y + dy * STUB, 4)
            items.append(["wire", ["pts", ["xy", x, y], ["xy", ex, ey]], ["stroke", ["width", 0], ["type", "default"]],
                          ["uuid", uid("w", *key)]])
            items.append(self._label(net, ex, ey, lang, key))
        x0, x1, y0, y1 = self.lib.bbox(part.lib_id, part.unit)
        props = [("Reference", part.ref, (sx + x1 + 1.27, sy - y1 - 1.27), False),
                 ("Value", part.value, (sx + x1 + 1.27, sy - y1 + 1.27), False),
                 ("Footprint", part.footprint, (sx, sy), True),
                 ("Datasheet", "", (sx, sy), True)]
        if part.lcsc:
            props.append(("LCSC", part.lcsc, (sx, sy), True))
        if part.note:
            props.append(("Note", part.note, (sx, sy), True))
        sym = ["symbol", ["lib_id", Q(part.lib_id)], ["at", sx, sy, 0], ["unit", part.unit],
               ["in_bom", "yes"], ["on_board", "yes"], ["dnp", "yes" if part.dnp else "no"],
               ["uuid", uid("sym", part.ref, part.unit)]]
        for name, val, (px, py), hide in props:
            eff = ["effects", ["font", ["size", 1.27, 1.27]], ["justify", "left"]]
            if hide:
                eff.append("hide")
            sym.append(["property", Q(name), Q(val), ["at", round(px, 4), round(py, 4), 0], eff])
        for num in pins:
            sym.append(["pin", Q(num), ["uuid", uid("pin", part.ref, part.unit, num)]])
        sym.append(["instances", ["project", Q(PROJECT), ["path", Q(sheet_path), ["reference", Q(part.ref)], ["unit", part.unit]]]])
        items.append(sym)
        return items

    def _file(self, title, paper, page, lib_ids, body, sheet_uuid):
        lib_symbols = ["lib_symbols"] + [self.lib.get(l) for l in sorted(lib_ids)]
        return ["kicad_sch", ["version", 20230121], ["generator", "eeschema"], ["uuid", sheet_uuid],
                ["paper", Q(paper)],
                ["title_block", ["title", Q(title)], ["date", Q(self.date)], ["rev", Q(self.rev)],
                 ["company", Q("Fotis Rousogiannis — Diploma Thesis, University of Thessaly")],
                 ["comment", 1, Q(self.root_title)]],
                lib_symbols, *body]

    def write(self, sheets, outdir):
        outdir = Path(outdir)
        root_body = []
        for i, sheet in enumerate(sheets):
            paper = self._layout(sheet)
            sheet_uuid = uid("sheet", sheet.name)
            path = f"/{self.root_uuid}/{sheet_uuid}"
            body = []
            for part in sheet.parts:
                body += self._part_items(part, path)
            for j, t in enumerate(sheet.texts):
                body.append(["text", Q(t), ["at", 25.4, 20.32 + j * 5.08, 0],
                             ["effects", ["font", ["size", 2, 2]], ["justify", "left"]], ["uuid", uid("txt", sheet.name, j)]])
            libs = {p.lib_id for p in sheet.parts}
            content = self._file(sheet.title, paper, i + 2, libs, body, uid("file", sheet.name))
            (outdir / sheet.filename).write_text(dump(content) + "\n", encoding="utf-8")
            col, row = i % 4, i // 4
            x, y = 25.4 + col * 60.96, 60.96 + row * 40.64
            root_body.append(["sheet", ["at", x, y], ["size", 50.8, 25.4], ["fields_autoplaced"],
                              ["stroke", ["width", 0.1524], ["type", "solid"]], ["fill", ["color", 0, 0, 0, 0.0]],
                              ["uuid", sheet_uuid],
                              ["property", Q("Sheetname"), Q(sheet.title), ["at", x, round(y - 0.7, 4), 0],
                               ["effects", ["font", ["size", 1.27, 1.27]], ["justify", "left", "bottom"]]],
                              ["property", Q("Sheetfile"), Q(sheet.filename), ["at", x, round(y + 26, 4), 0],
                               ["effects", ["font", ["size", 1.27, 1.27]], ["justify", "left", "top"]]],
                              ["instances", ["project", Q(PROJECT), ["path", Q(f"/{self.root_uuid}"), ["page", Q(str(i + 2))]]]]])
        root_body.append(["text", Q(self.root_title), ["at", 25.4, 30.48, 0], ["effects", ["font", ["size", 4, 4]], ["justify", "left"]],
                          ["uuid", uid("roottitle")]])
        root_body.append(["sheet_instances", ["path", Q("/"), ["page", Q("1")]]])
        root = self._file(self.root_title, "A4", 1, set(), root_body, self.root_uuid)
        (outdir / f"{PROJECT}.kicad_sch").write_text(dump(root) + "\n", encoding="utf-8")
