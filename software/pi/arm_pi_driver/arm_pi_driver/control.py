"""Έλεγχος θέσης ενός διαφορικού, χωρίς ROS και χωρίς υλικό (μεταφορά από τον πάγκο, ADR-0021).

Από software/tools/wrist_bench/control.py και teleop.py, που δούλεψαν στη βάση (run5, 2026-11-14):
ένας PID ανά άρθρωση σε μοίρες → κλάσμα ταχύτητας −1..1 ανά άξονα → μίξη στα δύο μοτέρ →
παλμοί με αντιστάθμιση της νεκρής ζώνης κάθε μοτέρ.

    together = άξονας που κινείται όταν τα μοτέρ πάνε μαζί   (joint_1, yaw)
    opposed  = άξονας που κινείται όταν τα μοτέρ πάνε αντίθετα (joint_2, roll)
    μοτέρ A = together + opposed,  μοτέρ B = together − opposed  (B ανάποδα αν είναι κατοπτρικό)
"""

import math


class LimitError(RuntimeError):
    """Μια άρθρωση βγήκε από τα όρια της + το περιθώριο: στάση."""


def clamp(value: float, low: float, high: float) -> float:
    """Η τιμή μέσα στο [low, high]."""
    return max(low, min(high, value))


class PID:
    """PID στο σφάλμα, με παράγωγο στη μέτρηση και anti-windup.

    Η παράγωγος στη μέτρηση δεν δίνει «κλωτσιά» όταν αλλάζει ο στόχος· το ολοκλήρωμα παγώνει όσο
    η έξοδος είναι κορεσμένη προς την ίδια φορά.
    """

    def __init__(self, kp: float, ki: float = 0.0, kd: float = 0.0, out_limit: float = 1.0):
        self.kp, self.ki, self.kd, self.out_limit = kp, ki, kd, out_limit
        self.reset()

    def reset(self) -> None:
        """Μηδενίζει ολοκλήρωμα και ιστορικό (νέος στόχος)."""
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


def compensation_span(motors: list, min_us: float, max_us: float, max_offset_us: float) -> float:
    """Το εύρος παλμού πάνω από την άκρη της νεκρής ζώνης που έχουν ΟΛΑ τα μοτέρ και οι φορές.

    motors = [(neutral_us, (down_us, up_us))]. Κοινό εύρος: ίδια εντολή → ίδια απόσταση από την
    άκρη του καθενός → περίπου ίδια ταχύτητα (ίδια servo).
    """
    spans = []
    for neutral, (down, up) in motors:
        spans.append(min(max_us, neutral + max_offset_us) - up)
        spans.append(down - max(min_us, neutral - max_offset_us))
    return max(0.0, min(spans))


def compensated_pulse(speed: float, neutral_us: float, edges, span_us: float) -> float:
    """Ταχύτητα −1..1 → παλμός: 0 → νεκρό σημείο (φρένο)· αλλιώς από την άκρη της νεκρής ζώνης."""
    if speed == 0:
        return neutral_us
    speed = clamp(speed, -1.0, 1.0)
    down, up = edges
    return up + speed * span_us if speed > 0 else down + speed * span_us


def plain_pulse(speed: float, neutral_us: float, min_us: float, max_us: float,
                max_offset_us: float) -> float:
    """Χωρίς αντιστάθμιση: −1..1 → min..νεκρό..max, μέσα στο όριο ασφαλείας."""
    speed = clamp(speed, -1.0, 1.0)
    pulse = neutral_us + speed * ((max_us - neutral_us) if speed >= 0 else (neutral_us - min_us))
    return clamp(pulse, neutral_us - max_offset_us, neutral_us + max_offset_us)


class DifferentialController:
    """Δύο αρθρώσεις ενός διαφορικού: στόχοι και γωνίες (μοίρες) → παλμοί των δύο μοτέρ."""

    def __init__(self, config: dict):
        self.joints, self.motors = config['joints'], config['motors']
        self.pca, self.cfg = config['pca'], config['pid']
        axes = sorted(j['axis'] for j in self.joints.values())
        if axes != ['opposed', 'together']:
            raise ValueError(f'ένα διαφορικό θέλει μία άρθρωση together και μία opposed, '
                             f'όχι {axes}')
        cfg = self.cfg
        self.pids = {name: PID(cfg['kp'], cfg['ki'], cfg['kd'], cfg['max_speed'])
                     for name in self.joints}
        self.targets = {}

    def set_targets(self, targets: dict) -> dict:
        """Στόχοι (μοίρες), κομμένοι στα όρια κάθε άρθρωσης· επιστρέφει τι ορίστηκε."""
        for name, value in targets.items():
            joint = self.joints[name]
            clamped = clamp(value, joint['lower_deg'], joint['upper_deg'])
            if self.targets.get(name) != clamped:
                self.pids[name].reset()
            self.targets[name] = clamped
        return dict(self.targets)

    def check_limits(self, angles: dict) -> None:
        """Σκάει με LimitError αν κάποια άρθρωση είναι πέρα από τα όρια της + το περιθώριο."""
        margin = self.cfg['limit_margin_deg']
        for name, angle in angles.items():
            joint = self.joints[name]
            if not joint['lower_deg'] - margin <= angle <= joint['upper_deg'] + margin:
                raise LimitError(f"{name} στις {angle:.1f}°, εκτός ορίων "
                                 f"[{joint['lower_deg']}, {joint['upper_deg']}]")

    def min_command(self, name: str, error: float) -> float:
        """Η μικρότερη εντολή που κινεί την άρθρωση: μετρημένη νεκρή ζώνη + περιθώριο."""
        joint = self.joints[name]
        measured = joint.get('deadband_pos') if error > 0 else joint.get('deadband_neg')
        if measured is None:
            return self.cfg['min_speed']
        return min(self.cfg['max_speed'], measured + self.cfg.get('deadband_margin', 0.03))

    def axis_speeds(self, angles: dict, dt: float) -> tuple[dict, dict]:
        """({άρθρωση: ταχύτητα −1..1}, {άρθρωση: σφάλμα}) για τους τρέχοντες στόχους."""
        self.check_limits(angles)
        speeds, errors = {}, {}
        for name in self.joints:
            target = self.targets.get(name, angles[name])
            error = target - angles[name]
            errors[name] = error
            u = self.pids[name].update(error, angles[name], dt)
            if abs(error) <= self.cfg['tolerance_deg']:
                u = 0.0
            else:
                floor = self.min_command(name, error)
                if abs(u) < floor:
                    u = math.copysign(floor, error)     # κάτω από αυτό η άρθρωση δεν κινείται
            speeds[name] = clamp(u, -self.cfg['max_speed'], self.cfg['max_speed'])
        return speeds, errors

    def pulses(self, speeds: dict) -> dict:
        """{κανάλι: παλμός ή None (καθόλου παλμός)} για τις ταχύτητες των αξόνων."""
        by_axis = {self.joints[name]['axis']: speed for name, speed in speeds.items()}
        together, opposed = by_axis.get('together', 0.0), by_axis.get('opposed', 0.0)
        a, b = self.motors['a'], self.motors['b']
        sa = clamp(together + opposed, -1.0, 1.0) * (-1 if a.get('mirrored') else 1)
        sb = clamp(together - opposed, -1.0, 1.0) * (-1 if b.get('mirrored') else 1)
        if sa == 0 and sb == 0:
            return {a['channel']: None, b['channel']: None}     # και τα δύο σταματημένα
        # Μοτέρ στο 0 ενώ το άλλο τρέχει: παλμός νεκρού σημείου, ώστε να φρενάρει· χωρίς παλμό
        # το γυρίζει το άλλο μέσα από το διαφορικό (πάγκος, 2026-11-14).
        lo, hi = self.pca['pulse_min_us'], self.pca['pulse_max_us']
        offset = self.pca['max_offset_us']
        ea, eb = a['deadband_edges_us'], b['deadband_edges_us']
        if self.cfg.get('motor_compensation', True):
            motors = [(a['neutral_us'], ea), (b['neutral_us'], eb)]
            span = compensation_span(motors, lo, hi, offset)
            return {a['channel']: compensated_pulse(sa, a['neutral_us'], ea, span),
                    b['channel']: compensated_pulse(sb, b['neutral_us'], eb, span)}
        return {a['channel']: plain_pulse(sa, a['neutral_us'], lo, hi, offset),
                b['channel']: plain_pulse(sb, b['neutral_us'], lo, hi, offset)}
