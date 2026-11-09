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
