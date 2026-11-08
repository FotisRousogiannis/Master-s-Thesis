#!/usr/bin/env python3
"""Pick JLCPCB/LCSC parts for the generic components (R, C, LED, misc) and write passives.csv.

    python3 resolve_passives.py

Rules: package must match, the exact value must appear in the description, extra requirements
(voltage, tolerance, ...) must appear too. Preference: JLC *basic* part (no setup fee) >
"preferred extended" > extended, then highest stock. gen/build.py reads passives.csv and puts
the LCSC code on every matching part, so the schematic BOM is complete for JLC assembly.
Run again before ordering (stock changes).
"""
import csv
import json
import re
import urllib.request
from pathlib import Path

from jlc_check import URL

HERE = Path(__file__).resolve().parent

# (value as in build.py, footprint key, search keywords, package, required regexes in description)
R0603 = [("1.2k", "1.2kΩ"), ("1.5k", "1.5kΩ"), ("10", "10Ω"), ("100k", "100kΩ"), ("10k", "10kΩ"),
         ("120", "120Ω"), ("150", "150Ω"), ("1k", "1kΩ"), ("2.2k", "2.2kΩ"), ("2.7k", "2.7kΩ"),
         ("220", "220Ω"), ("330k", "330kΩ"), ("39k", "39kΩ"), ("4.7k", "4.7kΩ"), ("470", "470Ω"),
         ("47k", "47kΩ"), ("5.1k", "5.1kΩ"), ("6.8k", "6.8kΩ"), ("750", "750Ω")]
SPECS = [(v, "R_0603_1608Metric", f"{d} 0603 resistor", "0603", [rf"(?<![\d.]){re.escape(d)}", r"±1%"])
         for v, d in R0603]
SPECS += [
    ("100n", "C_0603_1608Metric", "100nF 0603", "0603", [r"(?<![\d.])100nF", r"X7R", r"(25|50)V"]),
    ("1u", "C_0603_1608Metric", "1uF 0603", "0603", [r"(?<![\d.])1uF", r"(X7R|X5R)", r"(25|50)V"]),
    ("3.3n", "C_0603_1608Metric", "3.3nF 0603", "0603", [r"(?<![\d.])3\.3nF", r"(X7R|C0G|NP0)", r"50V"]),
    ("10u", "C_0805_2012Metric", "10uF 0805", "0805", [r"(?<![\d.])10uF", r"(X5R|X7R)", r"(25|35|50)V"]),
    ("22u", "C_0805_2012Metric", "22uF 0805", "0805", [r"(?<![\d.])22uF", r"(X5R|X7R)", r"(16|25|35)V"]),
    ("1000u 16V", "CP_Radial_D10.0mm_P5.00mm", "1000uF 16V 10x", "", [r"1mF|1000uF", r"(16|25)V", r"10mm|10x"]),
    ("5m 2W", "R_2512_6332Metric", "5mΩ 2512 2W", "2512", [r"(?<![\d.])5mΩ", r"(2W|3W)"]),
    # 4x4 mm, 3.4 A; footprint segment_board:L_4x4mm_SMNR4020 follows the LCSC land pattern
    ("2.2u Isat>=3A", "L_4x4mm_SMNR4020", "SMNR4020-2.2UH", "", [r"SMNR4020\-2\.2UH", r"2\.2uH", r"3\.4A"]),
    ("SMBJ6.0A", "D_SMB", "SMBJ6.0A", "SMB", [r"SMBJ6\.0A"]),
    # LEDs: low Vf (~2 V) so they are bright from 3.3 V through 1 k (see sim/)
    ("green", "LED_0603_1608Metric", "19-217/G7C-AN1P2/6T", "0603", [r"19\-217/G7C\-AN1P2/6T", r"(?i)yellow green"]),
    ("yellow", "LED_0603_1608Metric", "19-217/Y5C-AP1Q2/6T", "0603", [r"19\-217/Y5C\-AP1Q2/6T", r"(?i)yellow"]),
    ("red", "LED_0603_1608Metric", "19-217/R6C-AL1M2VY/6T", "0603", [r"19\-217/R6C\-AL1M2VY/6T", r"(?i)red"]),
    ("BOOT", "SW_SPST_TL3342", "TL3342F", "", [r"(?i)5\.2mm"]),
    ("RESET", "SW_SPST_TL3342", "TL3342F", "", [r"(?i)5\.2mm"]),
]
RANK = {"base": 0}


def query(kw, n=50):
    body = json.dumps({"keyword": kw, "currentPage": 1, "pageSize": n}).encode()
    req = urllib.request.Request(URL, data=body, headers={"Content-Type": "application/json", "User-Agent": "Mozilla/5.0"})
    data = json.load(urllib.request.urlopen(req, timeout=30))
    return (data.get("data") or {}).get("componentPageInfo", {}).get("list") or []


def pick(kw, pkg, req):
    best = None
    for it in query(kw):
        desc = f"{it.get('describe') or ''} {it.get('componentModelEn') or ''}"
        if pkg and pkg not in (it.get("componentSpecificationEn") or ""):
            continue
        if not all(re.search(r, desc) for r in req):
            continue
        if not it.get("stockCount"):
            continue
        lib = it.get("componentLibraryType")
        pref = 0 if lib == "base" else (1 if it.get("preferredComponentFlag") else 2)
        key = (pref, -it["stockCount"])
        if best is None or key < best[0]:
            best = (key, it)
    return best[1] if best else None


def main():
    rows = []
    for value, fp, kw, pkg, req in SPECS:
        it = pick(kw, pkg, req)
        if it is None:
            print(f"NOT FOUND  {value:16s} {fp}")
            rows.append({"value": value, "footprint": fp, "lcsc": "", "mpn": "", "type": "", "stock": "", "description": ""})
            continue
        t = "basic" if it["componentLibraryType"] == "base" else ("preferred" if it.get("preferredComponentFlag") else "extended")
        print(f"{it['componentCode']:10s} {t:9s} {value:16s} {it['componentModelEn']:28s} {it['stockCount']:>9}  {(it.get('describe') or '')[:70]}")
        rows.append({"value": value, "footprint": fp, "lcsc": it["componentCode"], "mpn": it["componentModelEn"],
                     "type": t, "stock": it["stockCount"], "description": (it.get("describe") or "")[:120]})
    with open(HERE / "passives.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


if __name__ == "__main__":
    main()
