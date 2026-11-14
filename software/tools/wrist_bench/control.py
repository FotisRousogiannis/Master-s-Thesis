"""Closed-loop position control of a differential pair on the bench: pure logic, no I/O (unit-tested).

The two axes of a differential are "pitch" (both motors together, keys W/S) and "roll" (motors
opposed, keys A/D). Each axis has its own AS5600 on the joint output (ADR-0019), so the controller
works per axis in degrees and the mixing to motors happens afterwards (teleop.Teleop.pulses):
a motor that runs a little faster than the other, or gear backlash, shows up on the encoders and
the PID corrects it.

    joint angle = sign * (raw - zero)   (unwrapped by arm_calibration.encoders.Tracker)
    u = PID(target - angle)             speed fraction -1..1 for that axis
"""

import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

_ARM_CALIBRATION = Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "arm_calibration"
if str(_ARM_CALIBRATION) not in sys.path:
    sys.path.insert(0, str(_ARM_CALIBRATION))
from arm_calibration.encoders import EncoderJump, Tracker  # noqa: E402  (no ROS dependency)

AXES = ("pitch", "roll")
DEFAULT_CALIBRATION = Path(__file__).with_name("base_calibration.json")

# Starting values, NOT measured: tune on the bench (teleop menu, PID settings) and record what works.
DEFAULT_PID = {
    "kp": 0.02,           # speed fraction per degree of error: 10 deg -> 0.2
    "ki": 0.0,
    "kd": 0.0,
    "max_speed": 0.5,     # cap of the controller output (speed fraction)
    "min_speed": 0.3,     # smallest command that still moves the base (150 us did not, 360 us did)
    "tolerance_deg": 1.0, # inside this the axis is "there": output 0 (outputs off)
    "settle_s": 0.5,      # both axes inside the tolerance this long -> done
    "timeout_s": 60.0,    # give up if not there by then (base: ~4 deg/s at 0.5, 80 deg took >20 s)
    "limit_margin_deg": 5.0,  # stop if an axis goes this far past its limits
    "deadband_margin": 0.03,  # added to a measured deadband to get the smallest command used
}


@dataclass
class AxisCalibration:
    mux_channel: int      # TCA9548A channel of the encoder that measures this axis
    sign: int             # +1 if the encoder angle grows with a + command on this axis, else -1
    zero_deg: float       # raw encoder angle when the axis is at 0
    lower_deg: float = -90.0
    upper_deg: float = 90.0
    label: str = ""                      # physical name, e.g. "yaw (joint_1)"
    deadband_pos: float | None = None    # smallest + command that moves this axis (measured, [9])
    deadband_neg: float | None = None    # same for - (can differ: gravity)

    def name(self, axis: str) -> str:
        return self.label or axis

    def tracker(self) -> Tracker:
        return Tracker(self.zero_deg, self.sign < 0, self.lower_deg, self.upper_deg)


def zero_for(raw_deg: float, sign: int, angle_deg: float) -> float:
    """Zero such that the axis reads `angle_deg` at the encoder reading `raw_deg`."""
    return round((raw_deg - sign * angle_deg) % 360.0, 3)


def detect_axis(before: dict, after: dict, min_move_deg: float = 2.0):
    """(mux channel, sign) of the encoder that moved during a + command on one axis.

    In a differential a pure axis command should move one joint only; if two encoders move about
    as much, the mixing (motor B direction) is wrong for this pair."""
    moves = {ch: (after[ch] - before[ch] + 180.0) % 360.0 - 180.0 for ch in before}
    ranked = sorted(moves, key=lambda ch: abs(moves[ch]), reverse=True)
    best = ranked[0]
    if abs(moves[best]) < min_move_deg:
        raise ValueError(f"κανένας encoder δεν κινήθηκε πάνω από {min_move_deg}° ({moves})")
    if len(ranked) > 1 and abs(moves[ranked[1]]) > 0.5 * abs(moves[best]):
        raise ValueError(f"κινήθηκαν δύο encoders ({moves}): λάθος φορά του μοτέρ B;")
    return best, (1 if moves[best] > 0 else -1)


def load_calibration(path=DEFAULT_CALIBRATION):
    """({axis: AxisCalibration}, pid settings); missing file -> ({}, defaults)."""
    path = Path(path)
    if not path.exists():
        return {}, dict(DEFAULT_PID)
    data = json.loads(path.read_text())
    axes = {ax: AxisCalibration(**data[ax]) for ax in AXES if ax in data}
    return axes, {**DEFAULT_PID, **data.get("pid", {})}


def save_calibration(path, axes: dict, pid: dict, motors: dict):
    data = {"_comment": "Bench calibration of the base differential, written by teleop.py (ADR-0019, ADR-0021)."}
    data.update({ax: asdict(c) for ax, c in axes.items()})
    data["pid"], data["motors"] = pid, motors
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n")


class PID:
    """PID on the error, derivative on the measurement (no kick when the target changes),
    integral frozen while the output is saturated in the same direction (anti-windup)."""

    def __init__(self, kp, ki=0.0, kd=0.0, out_limit=1.0):
        self.kp, self.ki, self.kd, self.out_limit = kp, ki, kd, out_limit
        self.reset()

    def reset(self):
        self.integral, self.prev_measurement = 0.0, None

    def update(self, error: float, measurement: float, dt: float) -> float:
        d = 0.0
        if self.kd and self.prev_measurement is not None and dt > 0:
            d = -self.kd * (measurement - self.prev_measurement) / dt
        self.prev_measurement = measurement
        candidate = self.integral + error * dt
        out = self.kp * error + self.ki * candidate + d
        saturated = abs(out) > self.out_limit and math.copysign(1, out) == math.copysign(1, error)
        if not saturated:
            self.integral = candidate
        out = self.kp * error + self.ki * self.integral + d
        return max(-self.out_limit, min(self.out_limit, out))


class LimitError(RuntimeError):
    pass


class PairController:
    """Two axes, each: angle from its encoder, PID to a target, deadband/limit shaping."""

    def __init__(self, axes: dict, pid: dict):
        missing = [ax for ax in AXES if ax not in axes]
        if missing:
            raise ValueError(f"δεν έχει γίνει calibration: {', '.join(missing)}")
        self.axes, self.cfg = axes, pid
        self.trackers = {ax: c.tracker() for ax, c in axes.items()}
        self.pids = {ax: PID(pid["kp"], pid["ki"], pid["kd"], pid["max_speed"]) for ax in AXES}
        self.targets = {}

    def angles(self, raw: dict) -> dict:
        """{axis: degrees} from {mux channel: raw degrees}; raises EncoderJump on an impossible step."""
        return {ax: self.trackers[ax].update(raw[c.mux_channel]) for ax, c in self.axes.items()}

    def set_targets(self, targets: dict) -> dict:
        """Targets clamped to each axis's limits; returns what was set."""
        for ax, value in targets.items():
            c = self.axes[ax]
            self.targets[ax] = max(c.lower_deg, min(c.upper_deg, value))
        for p in self.pids.values():
            p.reset()
        return dict(self.targets)

    def step(self, angles: dict, dt: float):
        """({axis: speed fraction}, {axis: error}); raises LimitError past the limits + margin."""
        margin = self.cfg["limit_margin_deg"]
        for ax, a in angles.items():
            c = self.axes[ax]
            if not c.lower_deg - margin <= a <= c.upper_deg + margin:
                raise LimitError(f"{ax} στις {a:.1f}°, εκτός ορίων [{c.lower_deg}, {c.upper_deg}]")
        commands, errors = {}, {}
        for ax in AXES:
            error = self.targets[ax] - angles[ax]
            errors[ax] = error
            u = self.pids[ax].update(error, angles[ax], dt)
            if abs(error) <= self.cfg["tolerance_deg"]:
                u = 0.0
            else:
                floor = self.min_command(ax, error)
                if abs(u) < floor:
                    u = math.copysign(floor, error)   # below this the axis does not move
            commands[ax] = max(-self.cfg["max_speed"], min(self.cfg["max_speed"], u))
        return commands, errors

    def min_command(self, axis: str, error: float) -> float:
        """Smallest command that moves `axis` towards `error`: the measured deadband of that direction
        plus a margin, else the general min_speed."""
        c = self.axes[axis]
        measured = c.deadband_pos if error > 0 else c.deadband_neg
        if measured is None:
            return self.cfg["min_speed"]
        return min(self.cfg["max_speed"], measured + self.cfg.get("deadband_margin", 0.03))

    def there(self, errors: dict) -> bool:
        return all(abs(e) <= self.cfg["tolerance_deg"] for e in errors.values())


def first_moving(steps, min_move_deg=0.5):
    """Deadband from a ramp: steps = [(command, axis movement in deg)], increasing command.
    Returns the first command whose step moved the axis at least min_move_deg in its direction."""
    for command, moved in steps:
        if moved * math.copysign(1, command) >= min_move_deg:
            return abs(command)
    return None

