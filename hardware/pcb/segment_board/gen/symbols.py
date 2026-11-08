"""Symbol library access: KiCad standard libraries + project-specific symbols.

Project symbols (parts missing from the KiCad libraries) are generated here from their
datasheet pin tables and written to ../lib/segment_board.kicad_sym.
"""

import copy
import os
from pathlib import Path

from sexp import Q, dump, find, first, parse

KICAD_SYMBOLS = Path(os.environ.get("KICAD_SYMBOL_DIR", "/usr/share/kicad/symbols"))
PROJECT_LIB = "segment_board"

# ----------------------------------------------------------------------------------------
# Project symbols: (name, ref prefix, description, datasheet, [left pins], [right pins])
# pin = (number, name, electrical type)
# ----------------------------------------------------------------------------------------
PROJECT_SYMBOLS = [
    ("TPS259827", "U", "15 A eFuse, 2.7-24 V, adjustable current limit, VQFN-24 (2 EP)",
     "https://www.ti.com/lit/ds/symlink/tps25982.pdf",
     [("1", "IN", "power_in"), ("2", "IN", "passive"), ("3", "IN", "passive"), ("16", "IN", "passive"),
      ("25", "IN_EP", "passive"), ("6", "EN/UVLO", "input"), ("15", "dVdt", "passive"),
      ("7", "ITIMER", "passive"), ("12", "LDSTRT", "input"),
      ("4", "GND", "power_in"), ("5", "GND", "passive"), ("14", "GND", "passive"), ("26", "GND_EP", "passive")],
     [("17", "OUT", "passive"), ("18", "OUT", "passive"), ("19", "OUT", "passive"), ("20", "OUT", "passive"),
      ("21", "OUT", "passive"), ("22", "OUT", "passive"), ("23", "OUT", "passive"), ("24", "OUT", "passive"),
      ("13", "PG", "open_collector"), ("9", "IMON", "passive"), ("8", "ILIM", "passive"),
      ("10", "RETRY_DLY", "passive"), ("11", "NRETRY", "passive")]),
    ("TPS259472A", "U", "5.5 A eFuse, 2.7-23 V, reverse-current blocking, overvoltage clamp, VQFN-HR-10",
     "https://www.ti.com/lit/ds/symlink/tps25947.pdf",
     [("5", "IN", "power_in"), ("1", "EN/UVLO", "input"), ("2", "OVCSEL", "input"),
      ("7", "DVDT", "passive"), ("10", "ITIMER", "passive"), ("8", "GND", "power_in")],
     [("6", "OUT", "passive"), ("3", "PG", "open_collector"), ("4", "PGTH", "input"), ("9", "ILM", "passive")]),
    ("LM66100", "U", "Ideal diode, 1.5 A, 1.5-5.5 V, SC-70-6",
     "https://www.ti.com/lit/ds/symlink/lm66100.pdf",
     [("1", "VIN", "power_in"), ("3", "~{CE}", "input"), ("2", "GND", "power_in")],
     [("6", "VOUT", "passive"), ("5", "~{ST}", "open_collector"), ("4", "N/C", "no_connect")]),
    ("AO4407A", "Q", "P-channel MOSFET, -30 V, -12 A, SOIC-8",
     "https://www.aosmd.com/res/datasheets/AO4407A.pdf",
     [("5", "D", "passive"), ("6", "D", "passive"), ("7", "D", "passive"), ("8", "D", "passive"), ("4", "G", "input")],
     [("1", "S", "passive"), ("2", "S", "passive"), ("3", "S", "passive")]),
]

PIN_LEN = 2.54
GRID = 2.54


def _r(x):
    return round(x, 4)


def make_ic(name, ref, desc, datasheet, left, right):
    rows = max(len(left), len(right))
    width = 15.24 if name != "AO4407A" else 10.16
    top = (rows - 1) * GRID / 2
    top = round(top / GRID) * GRID
    height = rows * GRID + GRID
    body = [Q(f"{name}_0_1"), ["rectangle", ["start", _r(-width / 2), _r(top + GRID)], ["end", _r(width / 2), _r(top - rows * GRID)],
                                ["stroke", ["width", 0.254], ["type", "default"]], ["fill", ["type", "background"]]]]
    pins = [Q(f"{name}_1_1")]
    for side, plist in (("L", left), ("R", right)):
        for i, (num, pname, etype) in enumerate(plist):
            y = _r(top - i * GRID)
            x = _r(-width / 2 - PIN_LEN) if side == "L" else _r(width / 2 + PIN_LEN)
            ang = 0 if side == "L" else 180
            pins.append(["pin", etype, "line", ["at", x, y, ang], ["length", PIN_LEN],
                         ["name", Q(pname), ["effects", ["font", ["size", 1.27, 1.27]]]],
                         ["number", Q(num), ["effects", ["font", ["size", 1.27, 1.27]]]]])
    props = [["property", Q("Reference"), Q(ref), ["at", 0, _r(top + GRID * 2), 0], ["effects", ["font", ["size", 1.27, 1.27]]]],
             ["property", Q("Value"), Q(name), ["at", 0, _r(top - rows * GRID - GRID), 0], ["effects", ["font", ["size", 1.27, 1.27]]]],
             ["property", Q("Footprint"), Q(""), ["at", 0, 0, 0], ["effects", ["font", ["size", 1.27, 1.27]], "hide"]],
             ["property", Q("Datasheet"), Q(datasheet), ["at", 0, 0, 0], ["effects", ["font", ["size", 1.27, 1.27]], "hide"]],
             ["property", Q("Description"), Q(desc), ["at", 0, 0, 0], ["effects", ["font", ["size", 1.27, 1.27]], "hide"]]]
    return ["symbol", Q(name), ["in_bom", "yes"], ["on_board", "yes"], *props, ["symbol", *body], ["symbol", *pins]]


def write_project_library(path):
    lib = ["kicad_symbol_lib", ["version", 20220914], ["generator", "segment_board_gen"]]
    lib += [make_ic(*s) for s in PROJECT_SYMBOLS]
    Path(path).write_text(dump(lib) + "\n", encoding="utf-8")


class Library:
    def __init__(self, project_lib_path):
        self._cache = {}
        self.project_lib_path = Path(project_lib_path)

    def _load(self, libname):
        if libname not in self._cache:
            path = self.project_lib_path if libname == PROJECT_LIB else KICAD_SYMBOLS / f"{libname}.kicad_sym"
            root = parse(path.read_text(encoding="utf-8"))
            self._cache[libname] = {s[1]: s for s in find(root, "symbol")}
        return self._cache[libname]

    def get(self, lib_id):
        """Flattened symbol definition named lib_id, ready for a schematic's lib_symbols."""
        libname, name = lib_id.split(":")
        syms = self._load(libname)
        sym = copy.deepcopy(syms[name])
        ext = first(sym, "extends")
        if ext:
            parent = copy.deepcopy(syms[ext[1]])
            own_props = {p[1]: p for p in find(sym, "property")}
            flat = [parent[0], Q(lib_id)]
            for item in parent[2:]:
                if isinstance(item, list) and item[0] == "property" and item[1] in own_props:
                    flat.append(own_props[item[1]])
                elif isinstance(item, list) and item[0] == "symbol":
                    item[1] = Q(item[1].replace(ext[1], name, 1))
                    flat.append(item)
                else:
                    flat.append(item)
            sym = flat
        sym[1] = Q(lib_id)
        return sym

    def pins(self, lib_id, unit=1):
        """{pin number: (x, y, angle, type)} in library coordinates for one unit (plus common unit 0)."""
        sym = self.get(lib_id)
        name = lib_id.split(":")[1]
        out = {}
        for sub in find(sym, "symbol"):
            parts = sub[1].rsplit("_", 2)
            u, style = int(parts[-2]), int(parts[-1])
            if u not in (0, unit) or style not in (0, 1):
                continue
            for p in find(sub, "pin"):
                at = first(p, "at")
                num = first(p, "number")[1]
                out[num] = (float(at[1]), float(at[2]), int(float(at[3])) if len(at) > 3 else 0, p[1])
        return out

    def bbox(self, lib_id, unit=1):
        pins = self.pins(lib_id, unit)
        xs = [p[0] for p in pins.values()] or [0]
        ys = [p[1] for p in pins.values()] or [0]
        return min(xs), max(xs), min(ys), max(ys)

    def units(self, lib_id):
        sym = self.get(lib_id)
        us = {int(s[1].rsplit("_", 2)[-2]) for s in find(sym, "symbol")}
        return max(us) if us else 1
