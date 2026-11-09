"""Joint position from an absolute encoder on the joint output axis (AS5600, 1:1), ADR-0019.

No ROS dependency (unit-tested). Angles in degrees.

The AS5600 is absolute within one turn: at power-up it already knows the magnet angle,
so a joint needs no homing motion, only its zero (the encoder reading at the joint's 0).
The joint angle is ``sign * (raw - zero)``, placed in the 360° window centred on the joint's
allowed range: the encoder's 0/360 jump then falls in the part of the circle the joint never
reaches. A joint whose range is a full 360° has no such part (margin 0).
"""

FULL_TURN = 360.0
RAW_KEY = "raw_deg"  # key of the raw encoder angle in the encoder/<joint> diagnostics (ADR-0018)


def window(lower: float, upper: float) -> tuple[float, float]:
    """(start of the 360° window, unreachable margin) for a joint with this range."""
    span = upper - lower
    if not 0 < span <= FULL_TURN:
        raise ValueError(f"range [{lower}, {upper}] does not fit in one encoder turn")
    margin = FULL_TURN - span
    return lower - margin / 2, margin


def joint_angle(raw: float, zero: float, invert: bool, lower: float, upper: float) -> float:
    """Joint angle for an encoder reading ``raw`` (0..360)."""
    start, _ = window(lower, upper)
    sign = -1.0 if invert else 1.0
    return (sign * (raw - zero) - start) % FULL_TURN + start


def zeros_from_reports(joints, reports: dict, now: float, max_age: float = 2.0):
    """Zeros for "the arm is in its reference pose now": all joints or none.

    reports: {joint: (stamp, level, raw or None)} from the encoder/<joint> diagnostics.
    Returns ({joint: zero}, problems); zeros is empty whenever there is any problem,
    so a missing or faulty encoder never leaves the arm half calibrated.
    """
    zeros, problems = {}, []
    for joint in joints:
        if joint not in reports:
            problems.append(f"{joint}: ο encoder δεν έχει αναφέρει")
            continue
        stamp, level, raw = reports[joint]
        if now - stamp > max_age:
            problems.append(f"{joint}: η τελευταία μέτρηση είναι παλιά ({now - stamp:.1f} s)")
        elif level != 0:
            problems.append(f"{joint}: ο encoder αναφέρει πρόβλημα (επίπεδο {level})")
        elif raw is None or not 0.0 <= raw < FULL_TURN:
            problems.append(f"{joint}: δεν υπάρχει έγκυρο {RAW_KEY}")
        else:
            zeros[joint] = round(raw, 2)
    return ({} if problems else zeros), problems


class EncoderJump(ValueError):
    """Two consecutive readings differ more than any real motion allows: encoder fault."""


class Tracker:
    """Continuous joint position from consecutive encoder readings ("unwrapping", ADR-0019).

    The encoder only knows 0..360: after 359.9 it gives 0 again. Between two readings a joint
    moves far less than 180°, so a change of e.g. 358 -> 1 is +3° (it went on), never -357°.
    The first reading uses joint_angle() (the arm must be inside its limits at power-up);
    after that the position keeps counting past ±180° instead of jumping to the other end.

    Reference for the firmware (ESP32, ~500 Hz, ADR-0005), which must see every reading.
    max_step_deg: largest believable change between two readings; above it the reading is
    rejected as an encoder fault. Must be larger than (max joint speed × sample period) and
    well below 180°.
    """

    def __init__(self, zero: float, invert: bool, lower: float, upper: float, max_step_deg: float = 90.0):
        if not 0 < max_step_deg < 180:
            raise ValueError("max_step_deg must be in (0, 180)")
        self.zero, self.invert, self.lower, self.upper = zero, invert, lower, upper
        self.max_step = max_step_deg
        self.position = None  # joint angle, degrees; None until the first reading
        self._raw = None

    def update(self, raw: float) -> float:
        if self.position is None:
            self.position = joint_angle(raw, self.zero, self.invert, self.lower, self.upper)
        else:
            delta = raw - self._raw
            if delta > 180.0:
                delta -= FULL_TURN
            elif delta < -180.0:
                delta += FULL_TURN
            if abs(delta) > self.max_step:
                raise EncoderJump(f"η ένδειξη άλλαξε {delta:+.1f}° σε μία μέτρηση")
            self.position += -delta if self.invert else delta
        self._raw = raw
        return self.position

    @property
    def in_limits(self) -> bool:
        return self.position is not None and self.lower <= self.position <= self.upper
