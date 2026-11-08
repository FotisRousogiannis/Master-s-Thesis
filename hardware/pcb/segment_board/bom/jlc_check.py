#!/usr/bin/env python3
"""Check JLCPCB stock and price for every part in parts.csv that has an LCSC code.

    python3 jlc_check.py            # all parts
    python3 jlc_check.py C130026    # specific LCSC codes

Uses the public parts search of jlcpcb.com; run it again right before ordering.
"""

import csv
import json
import sys
import urllib.request
from pathlib import Path

URL = "https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/selectSmtComponentList"


def lookup(code):
    body = json.dumps({"keyword": code, "currentPage": 1, "pageSize": 5}).encode()
    req = urllib.request.Request(URL, data=body, headers={"Content-Type": "application/json", "User-Agent": "Mozilla/5.0"})
    data = json.load(urllib.request.urlopen(req, timeout=30))
    items = (data.get("data") or {}).get("componentPageInfo", {}).get("list") or []
    return next((i for i in items if i.get("componentCode") == code), None)


def main(codes):
    rows = list(csv.DictReader(open(Path(__file__).with_name("parts.csv"), encoding="utf-8")))
    if codes:
        rows = [r for r in rows if r["lcsc"] in codes]
    for r in rows:
        code = r["lcsc"]
        if not code.startswith("C"):
            print(f"{r['function'][:40]:40s} {'-':10s} not stocked at JLCPCB ({r['assembly']})")
            continue
        it = lookup(code)
        if not it:
            print(f"{r['function'][:40]:40s} {code:10s} NOT FOUND")
            continue
        prices = it.get("componentPrices") or [{}]
        print(f"{r['function'][:40]:40s} {code:10s} stock {it.get('stockCount'):>8}  "
              f"{it.get('componentLibraryType'):8s} ${prices[0].get('productPrice')}")


if __name__ == "__main__":
    main(sys.argv[1:])
