"""Read the logbook from a terminal or from Arm Studio's Tools panel (read-only).

    ros2 run arm_logbook logbook position | calibrations [JOINT] | recordings | faults
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

from .store import Logbook


def default_database() -> Path:
    base = os.environ.get("ARM_DATA_DIR", Path.home() / ".local" / "share" / "arm_studio")
    return Path(base) / "arm_logbook.db"


def _when(t):
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t)) if t else "—"


def report(book: Logbook, what: str, joint=None) -> list[str]:
    if what == "position":
        rows = book.last_positions()
        return [f"{j:10s} {math.degrees(r['position']):9.2f}°   {_when(r['time'])}" for j, r in rows.items()] or [
            "Δεν υπάρχει αποθηκευμένη θέση (γράφεται μόνο με τον πραγματικό βραχίονα)."]
    if what == "calibrations":
        rows = book.calibrations(joint)
        return [f"{_when(r['time'])}  {r['joint']:10s} μηδέν {r['zero_deg']}°  αναφορά {r['reference_deg']}°"
                for r in rows] or ["Κανένα calibration ακόμα."]
    if what == "recordings":
        rows = book.recordings()
        return [f"{_when(r['start'])} → {_when(r['stop'])}  {r['note'] or '(χωρίς σημείωση)'}\n    {r['path']}"
                for r in rows] or ["Καμία καταγραφή ακόμα."]
    if what == "faults":
        rows = book.faults()
        return [f"{_when(r['time'])}  {r['source']}: {r['message']}" for r in rows] or ["Κανένα σφάλμα."]
    raise ValueError(what)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Ημερολόγιο του βραχίονα (μόνο ανάγνωση)")
    ap.add_argument("what", choices=["position", "calibrations", "recordings", "faults"])
    ap.add_argument("joint", nargs="?")
    ap.add_argument("--database", default=str(default_database()))
    args = ap.parse_args(argv)
    try:
        book = Logbook(args.database, readonly=True)
    except FileNotFoundError as exc:
        print(exc)
        return 1
    print(f"Ημερολόγιο: {book.path}")
    for line in report(book, args.what, args.joint):
        print(line)
    book.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
