"""Νόμοι ελέγχου (ControlLaw).

Νέος νόμος: μια κλάση εδώ, ένα όνομα στο LAWS, και στο YAML `law: {type: <όνομα>, …}`.
"""

import math

from arm_pi_driver.control.interfaces import ControlLaw


def clamp(value: float, low: float, high: float) -> float:
    """Η τιμή μέσα στο [low, high]."""
    return max(low, min(high, value))


class PID(ControlLaw):
    """PID στο σφάλμα, με παράγωγο στη μέτρηση και anti-windup.

    Η παράγωγος στη μέτρηση δεν δίνει «κλωτσιά» όταν αλλάζει ο στόχος· το ολοκλήρωμα παγώνει όσο
    η έξοδος είναι κορεσμένη προς την ίδια φορά.
    """

    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0, out_limit: float = 1.0):
        self.kp, self.ki, self.kd, self.out_limit = kp, ki, kd, out_limit
        self.reset()

    def reset(self) -> None:
        """Μηδενίζει ολοκλήρωμα και ιστορικό."""
        self.integral, self.prev_measurement = 0.0, None

    def update(self, error: float, measurement: float, dt: float) -> float:
        """Έξοδος −out_limit..out_limit για αυτό το σφάλμα."""
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
        return clamp(out, -self.out_limit, self.out_limit)


class BangBang(ControlLaw):
    """Σταθερή ταχύτητα προς τον στόχο, ανεξάρτητα από την απόσταση (ο πιο απλός νόμος)."""

    def __init__(self, speed: float, out_limit: float = 1.0):
        self.speed = clamp(abs(speed), 0.0, out_limit)

    def reset(self) -> None:
        """Δεν θυμάται τίποτα."""

    def update(self, error: float, measurement: float, dt: float) -> float:
        """±speed, με το πρόσημο του σφάλματος."""
        return math.copysign(self.speed, error) if error else 0.0


LAWS = {'pid': PID, 'bang_bang': BangBang}


def make_law(spec: dict, out_limit: float) -> ControlLaw:
    """Νόμος από το YAML: {type: pid, kp: …} → PID(kp=…, out_limit=…)."""
    params = dict(spec)
    kind = params.pop('type', 'pid')
    if kind not in LAWS:
        raise ValueError(f"άγνωστος νόμος ελέγχου '{kind}'· διαθέσιμοι: {', '.join(LAWS)}")
    return LAWS[kind](**params, out_limit=out_limit)
