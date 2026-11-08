#!/usr/bin/env python3
"""Sanity checks on a KiCad netlist (complements ERC).

    kicad-cli sch export netlist -o board.net segment_board.kicad_sch
    python3 check_netlist.py board.net

Fails on: nets with a single connection (usually a typo in a net name),
nets with power-input pins but no possible supply, unconnected component pins.
"""

import sys
from collections import defaultdict

from sexp import find, first, parse

SUPPLY_TYPES = {"power_out", "passive", "output", "tri_state", "bidirectional"}


def main(path):
    root = parse(open(path, encoding="utf-8").read())
    nets = first(root, "nets")
    problems, summary = [], []
    for net in find(nets, "net"):
        name = first(net, "name")[1]
        nodes = [(first(n, "ref")[1], first(n, "pin")[1], (first(n, "pintype") or [None, ""])[1]) for n in find(net, "node")]
        if name.startswith("unconnected-"):
            continue
        if len(nodes) == 1:
            problems.append(f"single-connection net {name}: {nodes[0][0]}.{nodes[0][1]}")
        types = {t.split("+")[0] for _, _, t in nodes}
        if "power_in" in types and not types & SUPPLY_TYPES:
            problems.append(f"power net {name} has no supply: {nodes}")
        summary.append((name, len(nodes)))
    print(f"{len(summary)} nets")
    for p in problems:
        print("PROBLEM:", p)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
