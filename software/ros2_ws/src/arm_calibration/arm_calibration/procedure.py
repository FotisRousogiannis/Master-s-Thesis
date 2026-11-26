"""The calibration procedure of ADR-0023 (steps 2-8), without ROS and without a GUI.

The procedure never writes the file itself: every step turns measurements (the replies of the Pi's
identification mode, arm_pi_driver/identify.py) into Proposals, "file value -> measured value ->
why", that the user approves one by one; approved proposals are applied with `apply()`, which
validates the whole file. The wizard of Arm Studio (step 5 of the implementation) drives these
functions and sends the jog requests they ask for.

    2  check_scan(data, scan)                        -> findings
    3+4 joint_jog(data, joint) / analyze_jog(...)    -> JointMapping (encoder, motion, proposals)
    5  direction_proposals(data, joint, delta, positive)
    6  ServoDeadbandSweep / JointDeadbandSweep       -> requests one by one, then proposals
    7  zero_proposals(data, {joint: raw_deg})
    8  gripper_limit_proposals(...), pressure_proposals(...)
"""

import copy
from dataclasses import dataclass, field

from . import arm_config

USABLE = ("OK", "TOO_WEAK", "TOO_STRONG")
INFO, WARNING, ERROR = "info", "warning", "error"
JOG_SPEED = 0.4          # as teleop [6]
JOG_DURATION_S = 0.5
JOG_STOP_DEG = 3.0
COUPLING = 0.5           # a second encoder moving more than half the first: not one joint alone


@dataclass(frozen=True)
class Proposal:
    """One change of arm.yaml for the user to approve: the table row "file -> measured"."""

    name: str            # flat parameter, e.g. "encoders.enc_1.channel"
    old: object          # value in the file now (None: not set)
    new: object
    why: str


@dataclass(frozen=True)
class Finding:
    """Something the user should see (not a change)."""

    level: str           # INFO, WARNING or ERROR
    text: str


def _old(data: dict, name: str):
    try:
        return arm_config.get_value(data, name)
    except KeyError:
        return None


def propose(data: dict, name: str, new, why: str) -> list:
    """A Proposal, or nothing if the file already has this value."""
    old = _old(data, name)
    return [] if old == new else [Proposal(name, old, new, why)]


def apply(data: dict, proposals: list) -> tuple[dict, list]:
    """(new data, validation errors): the approved proposals on a copy of the file's data.

    A proposal to None removes the entry (e.g. a servo that no longer moves a joint).
    """
    new = copy.deepcopy(data)
    for p in proposals:
        if p.new is None:
            *parents, leaf = p.name.split(".")
            arm_config.get_value(new, ".".join(parents)).pop(leaf, None)
        else:
            arm_config.set_value(new, p.name, p.new)
    return new, arm_config.validate(new)


# ---------------------------------------------------------------------------------------------- step 2


def check_scan(data: dict, scan: dict) -> list:
    """Findings of an I2C scan ({channel: {'devices', 'encoder'}}) against the file."""
    scan = {int(ch): v for ch, v in scan.items()}
    findings = []
    pca = data["controller"]["pca"]["mux_channel"]
    if "PCA9685" not in scan.get(pca, {}).get("devices", []):
        findings.append(Finding(ERROR, f"PCA9685: δεν απαντά στο κανάλι {pca} (servo χωρίς παλμούς)"))
    used = {}
    for name, joint in data["joints"].items():
        encoder = joint.get("encoder")
        channel = data["encoders"].get(encoder, {}).get("channel") if encoder else None
        if channel is not None:
            used[channel] = name
        if not joint.get("enabled"):
            continue
        reading = scan.get(channel, {}).get("encoder") if channel is not None else None
        if reading is None:
            findings.append(Finding(ERROR, f"{name}: κανένας AS5600 στο κανάλι {channel}"))
        elif reading["status"] != "OK":
            level = WARNING if reading["status"] in USABLE else ERROR
            findings.append(Finding(level, f"{name}: μαγνήτης {reading['status']} (κανάλι {channel})"))
        else:
            findings.append(Finding(INFO, f"{name}: encoder OK (κανάλι {channel})"))
    for channel, entry in sorted(scan.items()):
        if "AS5600" in entry.get("devices", []) and channel not in used:
            findings.append(Finding(INFO, f"AS5600 στο κανάλι {channel}: δεν ανήκει σε άρθρωση ακόμα"))
    sensors = [s for j in data["joints"].values() if j.get("enabled") for s in j.get("force_sensors", [])]
    for sensor in sensors:
        channel = data["force_sensors"][sensor].get("adc_channel")
        if "ADS1115" not in scan.get(channel, {}).get("devices", []):
            findings.append(Finding(ERROR, f"{sensor}: κανένας ADS1115 στο κανάλι {channel}"))
    return findings


# ---------------------------------------------------------------------------------------------- steps 3-4


@dataclass
class JointMapping:
    """What one jog of a joint showed."""

    joint: str
    channel: int | None = None       # encoder channel that moved (the largest motion)
    delta: float = 0.0               # its motion [deg], with sign, for the file's servo signs
    findings: list = field(default_factory=list)
    proposals: list = field(default_factory=list)


def joint_jog(data: dict, joint: str, speed: float = JOG_SPEED, duration_s: float = JOG_DURATION_S) -> dict:
    """The identify request that moves `joint` with the servo signs of the file.

    The Pi refuses a duration over controller.bench.max_duration_s (3 s); it stops at the first
    encoder change of JOG_STOP_DEG anyway.
    """
    signs = data["joints"][joint]["servos"]
    return {"op": "jog", "servos": {s: sign * speed for s, sign in signs.items()},
            "duration_s": duration_s, "stop_deg": JOG_STOP_DEG}


def _kind(data: dict, joint: str) -> str:
    return next(g.kind for g in arm_config.groups(data) if joint in g.joints)


def analyze_jog(data: dict, joint: str, result: dict) -> JointMapping:
    """A jog result ({'moved': {channel: deg}, 'reason'}) -> encoder of the joint, problems, proposals."""
    mapping = JointMapping(joint)
    moved = {int(ch): float(d) for ch, d in (result.get("moved") or {}).items()}
    kind, signs = _kind(data, joint), data["joints"][joint]["servos"]
    if not moved:
        mapping.findings.append(Finding(ERROR, f"{joint}: δεν κινήθηκε κανένας encoder"))
        if kind == arm_config.MIRRORED:
            last = list(signs)[-1]
            mapping.findings.append(Finding(WARNING, "κατοπτρικά servo: ίσως «παλεύουν» (αντίθετες φορές)"))
            mapping.proposals += propose(data, f"joints.{joint}.servos.{last}", -signs[last],
                                         "καμία κίνηση με τα δύο servo μαζί: δοκίμασε το αντίθετο πρόσημο")
        else:
            mapping.findings.append(Finding(WARNING, "έλεγξε: τροφοδοσία servo, κανάλι servo, νεκρή ζώνη"))
        return mapping
    ranked = sorted(moved, key=lambda ch: -abs(moved[ch]))
    mapping.channel, mapping.delta = ranked[0], moved[ranked[0]]
    for other in ranked[1:]:
        if abs(moved[other]) > COUPLING * abs(mapping.delta):
            mapping.findings.append(Finding(WARNING, f"κινήθηκε και ο encoder στο κανάλι {other} "
                                                     f"({moved[other]:+.1f}°): ανισορροπία servo ή μηχανική σύζευξη"))
    mapping.proposals += _encoder_proposals(data, joint, mapping.channel)
    return mapping


def _encoder_proposals(data: dict, joint: str, channel: int) -> list:
    encoders = data["encoders"]
    current = data["joints"][joint].get("encoder")
    if current and encoders[current].get("channel") == channel:
        return []
    owner = next((e for e, v in encoders.items() if v.get("channel") == channel), None)
    if owner is not None:
        user = next((j for j, v in data["joints"].items() if v.get("encoder") == owner and j != joint), None)
        why = f"κινήθηκε ο encoder του καναλιού {channel} ({owner})"
        if user is not None:
            why += f", που στο αρχείο είναι του {user}: λάθος κανάλι servo ή encoder"
        return propose(data, f"joints.{joint}.encoder", owner, why)
    if current:
        return propose(data, f"encoders.{current}.channel", channel,
                       f"κινήθηκε ο encoder του καναλιού {channel}")
    return []


def relabel_proposals(data: dict, joint: str, actual: str) -> list:
    """The jog of `joint` moved what the user calls `actual` (e.g. a differential named the other
    way round): servos with signs and encoders are exchanged between the two joints.

    Software alone cannot tell: in a differential, swapped encoders AND swapped sign patterns give
    measurements that agree with each other; only the user knows which motion is "joint_1".
    """
    if actual == joint:
        return []
    a, b = data["joints"][joint], data["joints"][actual]
    result = []
    for name, other in ((joint, b), (actual, a)):
        why = f"η κίνηση του {joint} ήταν στην πραγματικότητα το {actual}"
        for servo in set(data["joints"][name]["servos"]) | set(other["servos"]):
            result += propose(data, f"joints.{name}.servos.{servo}", other["servos"].get(servo), why)
        result += propose(data, f"joints.{name}.encoder", other.get("encoder"), why)
    return result


# ---------------------------------------------------------------------------------------------- step 5


def direction_proposals(data: dict, joint: str, delta: float, positive: bool) -> list:
    """The user saw the jog of `joint` (encoder change `delta`) and says whether it was positive.

    The control loop needs: positive command -> positive position. Seen as negative: every servo
    sign of the joint flips (a positive command then goes the other way). Then `invert` is chosen
    so that the encoder change of a positive command reads as a positive position.
    """
    proposals = []
    signs = data["joints"][joint]["servos"]
    if not positive:
        for servo, sign in signs.items():
            proposals += propose(data, f"joints.{joint}.servos.{servo}", -sign,
                                 "η κίνηση ήταν προς τα αρνητικά: αντίθετο πρόσημο")
        delta = -delta
    proposals += propose(data, f"joints.{joint}.invert", delta < 0,
                         "θετική κίνηση → ο encoder " + ("μειώνεται" if delta < 0 else "αυξάνεται"))
    return proposals


# ---------------------------------------------------------------------------------------------- step 6


class Sweep:
    """Smallest level that moves an encoder, per direction: one jog request at a time.

        sweep = ServoDeadbandSweep(...)
        while (request := sweep.next_request()) is not None:
            sweep.record(<jog result of the Pi>)
        sweep.proposals(data)
    """

    stop_deg = 0.5
    duration_s = 0.4

    def __init__(self, levels: list):
        self.levels = levels
        self.found = {+1: None, -1: None}
        self._direction, self._index = +1, 0

    @property
    def done(self) -> bool:
        return self._direction is None

    def next_request(self) -> dict | None:
        if self.done:
            return None
        return {"op": "jog", "plain": self.plain, "duration_s": self.duration_s, "stop_deg": self.stop_deg,
                "servos": self.servos(self._direction * self.levels[self._index])}

    def record(self, result: dict) -> None:
        moved = any(abs(float(d)) >= self.stop_deg for d in (result.get("moved") or {}).values())
        if moved:
            self.found[self._direction] = self.levels[self._index]
        if moved or self._index == len(self.levels) - 1:
            self._direction, self._index = (-1 if self._direction == +1 else None), 0
        else:
            self._index += 1


class ServoDeadbandSweep(Sweep):
    """Teleop [10]: pulses around the neutral of ONE servo, without compensation, up and down."""

    plain = True

    def __init__(self, data: dict, servo: str, step_us: int = 10):
        pca = data["controller"]["pca"]
        self.servo, self.neutral = servo, data["servos"][servo]["neutral_us"]
        # the Pi's plain_pulse: the same span both ways (the largest that fits both ends and the safety limit)
        span = min(pca["max_offset_us"], pca["pulse_max_us"] - self.neutral, self.neutral - pca["pulse_min_us"])
        self.up_span = self.down_span = span
        super().__init__(list(range(step_us, pca["max_offset_us"] // 2 + 1, step_us)))

    def servos(self, offset_us: int) -> dict:
        speed = offset_us / (self.up_span if offset_us > 0 else self.down_span)   # plain_pulse inverse
        return {self.servo: speed}

    def proposals(self, data: dict) -> list:
        up, down = self.found[+1], self.found[-1]
        if up is None or down is None:
            return []
        edges = [self.neutral - down, self.neutral + up]
        result = propose(data, f"servos.{self.servo}.deadband_us", edges,
                         f"πρώτη κίνηση στα {edges[0]} / {edges[1]} µs")
        middle = round(sum(edges) / 2)
        result += propose(data, f"servos.{self.servo}.neutral_us", middle, "μέση της νεκρής ζώνης")
        return result


class JointDeadbandSweep(Sweep):
    """Teleop [9]: the smallest command (speed fraction, compensated) that moves the joint, + and -."""

    plain = False

    def __init__(self, data: dict, joint: str, step: float = 0.01, top: float = 0.4):
        self.joint, self.signs = joint, data["joints"][joint]["servos"]
        super().__init__([round(step * i, 4) for i in range(1, int(round(top / step)) + 1)])

    def servos(self, speed: float) -> dict:
        return {s: sign * speed for s, sign in self.signs.items()}

    def proposals(self, data: dict) -> list:
        pos, neg = self.found[+1], self.found[-1]
        if pos is None or neg is None:
            return []
        return propose(data, f"joints.{self.joint}.deadband", [pos, neg],
                       f"πρώτη κίνηση με εντολή {pos} (+) / {neg} (−)")


# ---------------------------------------------------------------------------------------------- steps 7-8


def zero_proposals(data: dict, raw_deg: dict) -> list:
    """Step 7: the joints are at their reference pose; their encoders read `raw_deg`."""
    result = []
    for joint, raw in raw_deg.items():
        result += propose(data, f"joints.{joint}.zero_deg", round(float(raw) % 360.0, 3),
                          "θέση αναφοράς = 0")
    return result


def _wrap(delta: float) -> float:
    return (delta + 180.0) % 360.0 - 180.0


def gripper_limit_proposals(data: dict, joint: str, closed_raw: float, open_raw: float) -> tuple:
    """Step 8a: (proposals, findings) from the encoder fully closed and fully open. Zero = closed."""
    findings = []
    travel = _wrap(open_raw - closed_raw)
    invert = data["joints"][joint]["invert"]
    opening = -travel if invert else travel
    proposals = propose(data, f"joints.{joint}.zero_deg", round(closed_raw % 360.0, 3), "κλειστή = 0°")
    if opening < 0:
        proposals += propose(data, f"joints.{joint}.invert", not invert, "το άνοιγμα να μετρά θετικά")
        opening = -opening
    if opening < 5.0:
        findings.append(Finding(ERROR, f"{joint}: διαδρομή μόνο {opening:.1f}° (κινήθηκε η αρπάγη;)"))
    proposals += propose(data, f"joints.{joint}.limits_deg", [0.0, round(opening, 2)],
                         f"πλήρως ανοιχτή στις {opening:.1f}°")
    return proposals, findings


def pressure_proposals(data: dict, volts: dict) -> list:
    """Step 8b: the ZD10 voltages when the user pressed Stop = the maximum pressure of each."""
    result = []
    for sensor, value in volts.items():
        result += propose(data, f"force_sensors.{sensor}.max_v", round(float(value), 3),
                          "τάση τη στιγμή του Stop")
    return result
