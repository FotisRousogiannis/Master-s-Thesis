"""The arm's logbook in one SQLite file (ADR-0020). No ROS dependency (unit-tested).

Only the arm_logbook node opens the file; everyone else talks to it through ROS.
Every write is one transaction: after a power cut a record is either complete or absent.
"""

import json
import sqlite3
from pathlib import Path

SCHEMA_VERSION = 1
SCHEMA = """
CREATE TABLE IF NOT EXISTS last_position (
    joint    TEXT PRIMARY KEY,
    position REAL NOT NULL,          -- joint units of /joint_states (rad), unwrapped
    time     REAL NOT NULL           -- unix time, s
);
CREATE TABLE IF NOT EXISTS calibrations (
    id            INTEGER PRIMARY KEY,
    time          REAL NOT NULL,
    joint         TEXT NOT NULL,
    zero_deg      REAL,
    reference_deg REAL,
    invert        INTEGER,
    details       TEXT               -- JSON, anything else
);
CREATE TABLE IF NOT EXISTS recordings (
    id     INTEGER PRIMARY KEY,
    start  REAL NOT NULL,
    stop   REAL,
    path   TEXT NOT NULL UNIQUE,
    note   TEXT NOT NULL DEFAULT '',
    topics TEXT                      -- JSON list
);
CREATE TABLE IF NOT EXISTS faults (
    id      INTEGER PRIMARY KEY,
    time    REAL NOT NULL,
    source  TEXT NOT NULL,
    message TEXT NOT NULL,
    details TEXT                     -- JSON
);
"""


class Logbook:
    def __init__(self, path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._db = sqlite3.connect(str(self.path))
        self._db.execute("PRAGMA synchronous = FULL")  # durable on power loss
        version = self._db.execute("PRAGMA user_version").fetchone()[0]
        if version > SCHEMA_VERSION:
            raise RuntimeError(f"{self.path}: newer logbook schema ({version}) than this program ({SCHEMA_VERSION})")
        with self._db:
            self._db.executescript(SCHEMA)
            self._db.execute(f"PRAGMA user_version = {SCHEMA_VERSION}")

    def close(self):
        self._db.close()

    # ------------------------------------------------------------------ last position
    def save_positions(self, positions: dict, time: float):
        with self._db:
            self._db.executemany(
                "INSERT INTO last_position (joint, position, time) VALUES (?, ?, ?) "
                "ON CONFLICT(joint) DO UPDATE SET position = excluded.position, time = excluded.time",
                [(joint, float(p), time) for joint, p in positions.items()],
            )

    def last_positions(self) -> dict:
        rows = self._db.execute("SELECT joint, position, time FROM last_position ORDER BY joint")
        return {joint: {"position": p, "time": t} for joint, p, t in rows}

    # ------------------------------------------------------------------ history
    def add_calibration(self, time, joint, zero_deg=None, reference_deg=None, invert=None, **details):
        with self._db:
            self._db.execute(
                "INSERT INTO calibrations (time, joint, zero_deg, reference_deg, invert, details) VALUES (?, ?, ?, ?, ?, ?)",
                (time, joint, zero_deg, reference_deg, None if invert is None else int(bool(invert)),
                 json.dumps(details, ensure_ascii=False) if details else None),
            )

    def calibrations(self, joint=None) -> list[dict]:
        sql = "SELECT time, joint, zero_deg, reference_deg, invert FROM calibrations"
        rows = self._db.execute(sql + (" WHERE joint = ?" if joint else "") + " ORDER BY time, id",
                                (joint,) if joint else ())
        return [dict(zip(("time", "joint", "zero_deg", "reference_deg", "invert"), r)) for r in rows]

    def start_recording(self, time, path, note="", topics=()):
        with self._db:
            self._db.execute("INSERT INTO recordings (start, path, note, topics) VALUES (?, ?, ?, ?)",
                             (time, str(path), note, json.dumps(list(topics))))

    def stop_recording(self, time, path):
        with self._db:
            self._db.execute("UPDATE recordings SET stop = ? WHERE path = ? AND stop IS NULL", (time, str(path)))

    def recordings(self) -> list[dict]:
        rows = self._db.execute("SELECT start, stop, path, note, topics FROM recordings ORDER BY start, id")
        return [{"start": s, "stop": e, "path": p, "note": n, "topics": json.loads(t or "[]")} for s, e, p, n, t in rows]

    def add_fault(self, time, source, message, **details):
        with self._db:
            self._db.execute("INSERT INTO faults (time, source, message, details) VALUES (?, ?, ?, ?)",
                             (time, source, message, json.dumps(details, ensure_ascii=False) if details else None))

    def faults(self) -> list[dict]:
        rows = self._db.execute("SELECT time, source, message FROM faults ORDER BY time, id")
        return [dict(zip(("time", "source", "message"), r)) for r in rows]

    # ------------------------------------------------------------------ events from ROS (/logbook/event)
    def handle_event(self, event: dict, time: float):
        """One JSON event: {"kind": "calibration" | "recording_start" | "recording_stop" | "fault", ...}."""
        event = dict(event)
        kind = event.pop("kind", None)
        if kind == "calibration":
            self.add_calibration(time, event.pop("joint"), event.pop("zero_deg", None),
                                 event.pop("reference_deg", None), event.pop("invert", None), **event)
        elif kind == "recording_start":
            self.start_recording(time, event["path"], event.get("note", ""), event.get("topics", ()))
        elif kind == "recording_stop":
            self.stop_recording(time, event["path"])
        elif kind == "fault":
            self.add_fault(time, event.pop("source"), event.pop("message"), **event)
        else:
            raise ValueError(f"unknown logbook event: {kind!r}")


class StillDetector:
    """Says when the arm has stopped at a new position, so the logbook writes only then
    (a few writes per motion, not 100 per second)."""

    def __init__(self, still_s: float = 0.5, tolerance: float = 1e-3):
        self.still_s, self.tolerance = still_s, tolerance
        self._moving_ref, self._since, self._saved = None, None, None

    def _close(self, a, b):
        return a is not None and a.keys() == b.keys() and all(abs(a[k] - b[k]) <= self.tolerance for k in b)

    def update(self, positions: dict, now: float):
        """Positions to save, or None."""
        if not self._close(self._moving_ref, positions):
            self._moving_ref, self._since = dict(positions), now
            return None
        if now - self._since >= self.still_s and not self._close(self._saved, positions):
            self._saved = dict(positions)
            return dict(positions)
        return None
