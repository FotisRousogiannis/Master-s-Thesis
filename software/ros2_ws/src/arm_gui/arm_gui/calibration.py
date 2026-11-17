"""The calibration bench of ADR-0023 without Qt, the way teleop.py works on the bench.

The Pi's identification mode (arm_pi_driver/identify.py) runs the teleop procedures ([6], [10],
[9]) and streams the encoders 5 times a second; the arm moves while an arrow is held (drive). This
class keeps what the Pi said and turns each finished procedure into the parameters to save; the
user confirms once («Σωστό · αποθήκευση») and they go to /arm_calibration all at once, then to
arm.yaml.

    request = bench.identify(group)        -> the GUI publishes it
    follow_ups = bench.handle(reply)       -> the GUI publishes these too (next joint, next servo)
    bench.changes                          -> {parameter: value} waiting for «Σωστό»
"""

import re

from arm_calibration import arm_config


def natural(name: str):
    """joint_2 before joint_10, servo_2 before servo_10 (ROS gives parameters alphabetically)."""
    return [int(part) if part.isdigit() else part for part in re.split(r"(\d+)", name)]


MAGNET_NOTES = {   # what the AS5600 status means for the user (AS5600 datasheet: AGC out of range)
    "TOO_WEAK": "μαγνήτης μακριά ή μικρός: η γωνία μετρά, με θόρυβο· φέρ' τον πιο κοντά (~0.5–3 mm)",
    "TOO_STRONG": "μαγνήτης πολύ κοντά: η γωνία μετρά, με θόρυβο· απομάκρυνέ τον λίγο",
    "NO_MAGNET": "δεν βλέπει μαγνήτη: η γωνία δεν ισχύει",
}
USABLE = ("OK", "TOO_WEAK", "TOO_STRONG")
ORDER = {"joints": lambda n: (n == "gripper", natural(n))}      # the gripper last, as in arm.yaml
MIN_MOVE_DEG = 2.0       # teleop detect_axis: less than this is not a joint moving
COUPLING = 0.5           # a second encoder moving more than half the first: two joints moved


def nest(values: dict) -> dict:
    """Flat parameters of /arm_calibration -> the nested data of arm.yaml."""
    data: dict = {}
    for name, value in values.items():
        node = data
        *parents, leaf = name.split(".")
        for part in parents:
            node = node.setdefault(part, {})
        if leaf in ("channel", "adc_channel") and value == -1:     # -1 = not set (arm_config.NO_CHANNEL)
            value = None
        node[leaf] = list(value) if isinstance(value, (list, tuple)) else value
    for section in ("servos", "encoders", "force_sensors", "joints", "controller"):
        data.setdefault(section, {})
    for section in ("servos", "encoders", "force_sensors"):
        for item in data[section].values():   # a null of the file has no parameter
            item.setdefault("channel" if section != "force_sensors" else "adc_channel", None)
    for joint in data["joints"].values():
        joint.setdefault("encoder", None)
        joint.setdefault("servos", {})
    for section in ("servos", "encoders", "force_sensors", "joints"):
        key = ORDER.get(section, natural)
        data[section] = {name: data[section][name] for name in sorted(data[section], key=key)}
    return data


def wrap(delta: float) -> float:
    return (delta + 180.0) % 360.0 - 180.0


def zero_for(raw_deg: float, invert: bool, angle_deg: float) -> float:
    """Zero such that the joint reads `angle_deg` at the encoder reading `raw_deg` (teleop)."""
    return round((raw_deg - (-1 if invert else 1) * angle_deg) % 360.0, 3)


class Bench:
    """What the Pi said, and what a finished procedure would save."""

    def __init__(self):
        self.data = None
        self.active = False              # the Pi is (we asked it to be) in identification mode
        self.live = {}                   # {channel: {'raw_deg', 'status'}} from the Pi, 5 Hz
        self.log = []                    # [(level, text)] for the user
        self.running = None              # name of the procedure running on the Pi
        self.changes = {}                # {parameter: value} waiting for «Σωστό · αποθήκευση»
        self.summary = []                # what the last procedure found, line by line
        self.identified = {}             # {joint: (channel, moved deg)} of the last [6]
        self._queue, self._current = [], None
        self._next_id = 1
        self._results = {}

    # ---------------------------------------------------------------- the file
    def channel_of(self, joint: str):
        encoder = self.data["joints"][joint].get("encoder")
        return self.data["encoders"].get(encoder, {}).get("channel") if encoder else None

    def owner_of(self, channel: int):
        return next((j for j in self.data["joints"] if self.channel_of(j) == channel), None)

    def position(self, joint: str):
        """The joint angle now (zero and direction of the file), from the live reading."""
        reading = self.live.get(self.channel_of(joint))
        if not reading or reading.get("raw_deg") is None or reading.get("status") not in USABLE:
            return None
        j = self.data["joints"][joint]
        return round((-1 if j.get("invert") else 1) * wrap(reading["raw_deg"] - j.get("zero_deg", 0.0)), 2)

    def groups(self) -> list:
        """Groups whose servos all have a channel (only those can move)."""
        result = []
        for g in arm_config.groups(self.data):
            if all(self.data["servos"][s].get("channel") is not None for s in g.servos):
                result.append(g)
        return result

    # ---------------------------------------------------------------- requests
    def _request(self, request: dict) -> dict:
        request = {"id": self._next_id, **request}
        self._next_id += 1
        if request["op"] != "exit":
            self.active = True
        return request

    def scan(self) -> dict:
        return self._request({"op": "scan"})

    def ping(self) -> dict | None:
        """Every few seconds: keeps the Pi in identification mode (it leaves after 5 s of silence)."""
        return self._request({"op": "ping"}) if self.active else None

    def drive(self, speeds: dict) -> dict:
        """{joint: speed −1..1} → the servos with the signs of the file (sent while held)."""
        servos = {}
        for joint, speed in speeds.items():
            for servo, sign in self.data["joints"][joint]["servos"].items():
                servos[servo] = max(-1.0, min(1.0, servos.get(servo, 0.0) + sign * speed))
        return self._request({"op": "drive", "servos": servos})

    def stop(self) -> dict:
        self._queue, self._current, self.running = [], None, None
        return self._request({"op": "stop"})

    def exit(self) -> dict:
        self._queue, self._current, self.running, self.active = [], None, None, False
        return self._request({"op": "exit"})

    def _start(self, name: str, steps: list) -> dict:
        """Run the routines one after the other: steps = [(label, request)]."""
        self.changes, self.summary, self._results = {}, [], {}
        self.running, self._queue = name, list(steps)
        return self._pop()

    def _pop(self) -> dict | None:
        if not self._queue:
            self._current = None
            return None
        label, request = self._queue.pop(0)
        self._current = label
        self.log.append(("info", f"{self.running}: {label}…"))
        return self._request({"op": "routine", **request})

    def identify(self, group) -> dict:
        """[6] for every joint of the group: which encoder, which way."""
        self.identified = {}
        return self._start("Αναγνώριση", [
            (j, {"name": "identify", "servos": self.data["joints"][j]["servos"]}) for j in group.joints])

    def servo_deadbands(self, group) -> dict:
        """[10] for every servo of the group, the others braking."""
        return self._start("Νεκρή ζώνη servo", [
            (s, {"name": "servo_deadband", "servo": s, "brake": [o for o in group.servos if o != s]})
            for s in group.servos])

    def joint_deadbands(self, group) -> dict:
        """[9] for every joint of the group."""
        steps = []
        for j in group.joints:
            joint = self.data["joints"][j]
            steps.append((j, {"name": "joint_deadband", "servos": joint["servos"],
                              "channel": self.channel_of(j), "invert": joint["invert"]}))
        return self._start("Νεκρή ζώνη άρθρωσης", steps)

    # ---------------------------------------------------------------- replies
    def handle(self, reply: dict) -> list:
        """One reply of the Pi → follow-up requests (the next routine of a procedure)."""
        op = reply.get("op")
        if op == "live":
            self.live = {int(ch): r for ch, r in reply["result"].items()}
            return []
        if op == "exit" and reply.get("id") is None:          # the Pi left on its own
            self.active, self.running, self._queue = False, None, []
            self.log.append(("warning", reply.get("result", "")))
            return []
        if op == "progress":
            self.log.append(("info", f"  {self._current}: {reply['result']}"))
            return []
        if not reply.get("ok"):
            self.log.append(("error", reply.get("error", "σφάλμα")))
            if op in ("routine", "drive"):
                self._queue, self._current, self.running = [], None, None
            return []
        if op == "scan":
            self._scan(reply["result"])
        if op == "routine":
            self._results[self._current] = reply["result"]
            follow = self._pop()
            if follow is not None:
                return [follow]
            self._finish()
        return []

    def _scan(self, result: dict) -> None:
        for ch, entry in sorted(result.items(), key=lambda kv: int(kv[0])):
            devices = ", ".join(entry["devices"]) or "τίποτα"
            reading = entry.get("encoder")
            owner = self.owner_of(int(ch))
            text = f"κανάλι {ch}: {devices}" + (f" · {owner}" if owner else "")
            level = "info"
            if reading:
                text += f" · {reading['raw_deg']}° · μαγνήτης {reading['status']}"
                if reading["status"] != "OK":
                    text += f" ({MAGNET_NOTES.get(reading['status'], '')})"
                    level = "warning" if reading["status"] in USABLE else "error"
            self.log.append((level, text))

    def _finish(self) -> None:
        name, self.running = self.running, None
        {"Αναγνώριση": self._identified, "Νεκρή ζώνη servo": self._servo_bands,
         "Νεκρή ζώνη άρθρωσης": self._joint_bands}[name]()
        self.log += [("info", line) for line in self.summary]
        if self.changes:
            self.log.append(("info", "Αν είναι σωστά: «Σωστό · αποθήκευση»."))

    def _identified(self) -> None:
        """teleop detect_axis: the encoder that moved most is the joint's; two joints, two encoders."""
        problems = []
        for joint, result in self._results.items():
            moved = {int(ch): deg for ch, deg in result["moved"].items()}
            ranked = sorted(moved, key=lambda ch: -abs(moved[ch]))
            best = ranked[0]
            if abs(moved[best]) < MIN_MOVE_DEG:
                problems.append(f"{joint}: κανένας encoder δεν κινήθηκε πάνω από {MIN_MOVE_DEG:g}° "
                                f"(μεγάλωσε την ταχύτητα)")
                continue
            if len(ranked) > 1 and abs(moved[ranked[1]]) > COUPLING * abs(moved[best]):
                problems.append(f"{joint}: κινήθηκαν δύο encoders ({moved}): λάθος πρόσημο servo;")
                continue
            self.identified[joint] = (best, moved[best])
        channels = [ch for ch, _ in self.identified.values()]
        if len(set(channels)) < len(channels):
            problems.append("δύο αρθρώσεις κίνησαν τον ίδιο encoder: έλεγξε τα πρόσημα των servo")
        if problems:
            self.summary = ["✖ " + p for p in problems]
            self.identified = {}
            return
        self._identify_changes()

    def _identify_changes(self) -> None:
        self.changes, self.summary = {}, ["Κατάλαβα:"]
        for joint, (channel, moved) in self.identified.items():
            owner = next((e for e, v in self.data["encoders"].items() if v.get("channel") == channel), None)
            encoder = self.data["joints"][joint].get("encoder")
            if owner is not None and owner != encoder:
                self.changes[f"joints.{joint}.encoder"] = owner
            elif owner is None and encoder:
                self.changes[f"encoders.{encoder}.channel"] = channel
            self.changes[f"joints.{joint}.invert"] = moved < 0
            self.summary.append(f"  {joint} → encoder στο κανάλι {channel}, η γωνία "
                                f"{'ανεβαίνει' if moved > 0 else 'κατεβαίνει'} με θετική εντολή "
                                f"({moved:+.1f}°)")
        self.changes = {k: v for k, v in self.changes.items() if self._differs(k, v)}

    def swap_names(self) -> None:
        """A differential named the other way round: the two joints exchange what [6] found."""
        if len(self.identified) == 2:
            a, b = list(self.identified)
            self.identified = {a: self.identified[b], b: self.identified[a]}
            self._identify_changes()
            self.log += [("info", line) for line in self.summary]

    def flip(self, joint: str) -> None:
        """The «positive» move of this joint was the wrong way: every servo sign flips (teleop W/S)."""
        if joint not in self.identified:
            return
        for servo, sign in self.data["joints"][joint]["servos"].items():
            self.changes[f"joints.{joint}.servos.{servo}"] = -sign
        channel, moved = self.identified[joint]
        self.identified[joint] = (channel, -moved)
        self.changes[f"joints.{joint}.invert"] = -moved < 0
        self.log.append(("info", f"{joint}: αντίθετη φορά (πρόσημα servo και encoder)"))

    def _servo_bands(self) -> None:
        self.changes, self.summary = {}, []
        for servo, r in self._results.items():
            if r["down_us"] is None or r["up_us"] is None:
                self.summary.append(f"✖ {servo}: δεν κινήθηκε προς τη μία φορά ({r})")
                continue
            neutral = round((r["down_us"] + r["up_us"]) / 2)
            self.changes[f"servos.{servo}.deadband_us"] = [r["down_us"], r["up_us"]]
            self.changes[f"servos.{servo}.neutral_us"] = neutral
            self.summary.append(f"  {servo}: κινείται κάτω από {r['down_us']} και πάνω από {r['up_us']} µs → "
                                f"νεκρό σημείο ≈ {neutral} µs")

    def _joint_bands(self) -> None:
        self.changes, self.summary = {}, []
        for joint, r in self._results.items():
            if r["pos"] is None or r["neg"] is None:
                self.summary.append(f"✖ {joint}: δεν κινήθηκε προς τη μία φορά ({r})")
                continue
            self.changes[f"joints.{joint}.deadband"] = [r["pos"], r["neg"]]
            self.summary.append(f"  {joint}: κινείται από {r['pos']} (+) / {r['neg']} (−)")

    def zero(self, joint: str, angle: float, lower: float, upper: float) -> bool:
        """teleop [6], second part: the joint is at `angle`; its encoder reading becomes that angle."""
        reading = self.live.get(self.channel_of(joint))
        if not reading or reading.get("raw_deg") is None or reading.get("status") not in USABLE:
            self.log.append(("error", f"{joint}: ο encoder δεν μετρά τώρα"))
            return False
        zero = zero_for(reading["raw_deg"], self.data["joints"][joint].get("invert", False), angle)
        self.changes = {f"joints.{joint}.zero_deg": zero, f"joints.{joint}.limits_deg": [float(lower), float(upper)]}
        self.summary = [f"  {joint}: μηδέν {zero}° (ένδειξη {reading['raw_deg']}° = {angle:g}°), "
                        f"όρια [{lower:g}, {upper:g}]"]
        self.log += [("info", line) for line in self.summary]
        return True

    def gripper(self, closed_raw: float, open_raw: float) -> None:
        """8α: zero = closed; limits [0, opening] (invert so that opening counts positive)."""
        travel = wrap(open_raw - closed_raw)
        self.changes = {"joints.gripper.zero_deg": round(closed_raw % 360.0, 3),
                        "joints.gripper.invert": travel < 0,
                        "joints.gripper.limits_deg": [0.0, round(abs(travel), 2)]}
        self.summary = [f"  αρπάγη: κλειστή {closed_raw}°, ανοιχτή {open_raw}° → διαδρομή {abs(travel):.1f}°"]
        self.log += [("info", line) for line in self.summary]

    def _differs(self, name: str, value) -> bool:
        try:
            return arm_config.get_value(self.data, name) != value
        except KeyError:
            return True

    def saved(self) -> None:
        self.changes, self.summary = {}, []
