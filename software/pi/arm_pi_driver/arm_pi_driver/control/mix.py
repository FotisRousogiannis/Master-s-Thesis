"""MixController: ομάδα αρθρώσεων που κινείται από μοτέρ με γραμμική μίξη (από τον πάγκο).

Χωρίς μετρημένη νεκρή ζώνη σε κάποιο μοτέρ της ομάδας (calibration.yaml), οι παλμοί βγαίνουν χωρίς
αντιστάθμιση (νεκρό σημείο ± ταχύτητα × εύρος).

Κάθε μοτέρ γράφει στο YAML πόσο συμμετέχει σε κάθε άρθρωση της ομάδας (`mix`):

    ταχύτητα μοτέρ = Σ mix[άρθρωση] × ταχύτητα άρθρωσης      (× −1 αν είναι κατοπτρικό)

    διαφορικό (βάση, καρπός):  a: {joint_1: 1, joint_2: 1}   b: {joint_1: 1, joint_2: -1}
    δύο κατοπτρικά (joint_3):  a: {joint_3: 1}               b: {joint_3: 1}  (b mirrored)
    ένα servo:                 a: {x: 1}

Ανά άρθρωση: νόμος ελέγχου (ControlLaw) → ανοχή, ελάχιστη εντολή (μετρημένη νεκρή ζώνη),
μέγιστο· ανά μοτέρ: αντιστάθμιση νεκρής ζώνης (pulses.py).
"""

import math

from arm_pi_driver.control.interfaces import Controller, LimitError
from arm_pi_driver.control.laws import clamp, make_law
from arm_pi_driver.control.pulses import compensated_pulse, compensation_span, plain_pulse


class MixController(Controller):
    """Ομάδα αρθρώσεων με μοτέρ γραμμικής μίξης (διαφορικό, κατοπτρικά, ένα servo)."""

    def __init__(self, group: dict, control: dict, pca: dict):
        self._joints, self.motors = group['joints'], group['motors']
        self.cfg, self.pca = control, pca
        for name, motor in self.motors.items():
            unknown = set(motor['mix']) - set(self._joints)
            if unknown:
                raise ValueError(f'μοτέρ {name}: mix για αρθρώσεις εκτός ομάδας: '
                                 f'{sorted(unknown)}')
        driven = {j for motor in self.motors.values() for j in motor['mix'] if motor['mix'][j]}
        if driven != set(self._joints):
            raise ValueError(f'αρθρώσεις χωρίς μοτέρ: {sorted(set(self._joints) - driven)}')
        max_speed = control['max_speed']
        self.laws = {name: make_law(joint.get('law', {'type': 'pid'}), max_speed)
                     for name, joint in self._joints.items()}
        self._targets = {}

    @property
    def joints(self) -> tuple:
        """Οι αρθρώσεις της ομάδας."""
        return tuple(self._joints)

    @property
    def channels(self) -> tuple:
        """Τα κανάλια PWM των μοτέρ της ομάδας."""
        return tuple(motor['channel'] for motor in self.motors.values())

    @property
    def targets(self) -> dict:
        """Οι στόχοι που ισχύουν (μοίρες)."""
        return dict(self._targets)

    def set_targets(self, targets: dict) -> dict:
        """Στόχοι (μοίρες), κομμένοι στα όρια κάθε άρθρωσης· επιστρέφει όσους ισχύουν."""
        for name, value in targets.items():
            joint = self._joints[name]
            clamped = clamp(value, joint['lower_deg'], joint['upper_deg'])
            if self._targets.get(name) != clamped:
                self.laws[name].reset()
            self._targets[name] = clamped
        return self.targets

    def clear_targets(self) -> None:
        """Ξεχνά όλους τους στόχους και τη «μνήμη» των νόμων."""
        self._targets = {}
        for law in self.laws.values():
            law.reset()

    def check_limits(self, angles: dict) -> None:
        """Σκάει με LimitError αν κάποια άρθρωση είναι πέρα από τα όρια της + το περιθώριο."""
        margin = self.cfg['limit_margin_deg']
        for name in self._joints:
            joint, angle = self._joints[name], angles[name]
            if not joint['lower_deg'] - margin <= angle <= joint['upper_deg'] + margin:
                raise LimitError(f"{name} στις {angle:.1f}°, εκτός ορίων "
                                 f"[{joint['lower_deg']}, {joint['upper_deg']}]")

    def min_command(self, name: str, error: float) -> float:
        """Η μικρότερη εντολή που κινεί την άρθρωση: μετρημένη νεκρή ζώνη + περιθώριο."""
        joint = self._joints[name]
        measured = joint.get('deadband_pos') if error > 0 else joint.get('deadband_neg')
        if measured is None:
            return self.cfg['min_speed']
        return min(self.cfg['max_speed'], measured + self.cfg.get('deadband_margin', 0.03))

    def joint_speeds(self, angles: dict, dt: float) -> dict:
        """{άρθρωση: ταχύτητα −1..1}: νόμος ελέγχου, μετά ανοχή, ελάχιστο και μέγιστο."""
        self.check_limits(angles)
        speeds = {}
        for name in self._joints:
            error = self._targets.get(name, angles[name]) - angles[name]
            u = self.laws[name].update(error, angles[name], dt)
            if abs(error) <= self.cfg['tolerance_deg']:
                u = 0.0                                 # «έφτασα»
            else:
                floor = self.min_command(name, error)
                if abs(u) < floor:
                    u = math.copysign(floor, error)     # κάτω από αυτό η άρθρωση δεν κινείται
            speeds[name] = clamp(u, -self.cfg['max_speed'], self.cfg['max_speed'])
        return speeds

    def motor_speeds(self, speeds: dict) -> dict:
        """{μοτέρ: ταχύτητα −1..1} από τις ταχύτητες των αρθρώσεων (μίξη)."""
        result = {}
        for name, motor in self.motors.items():
            mixed = sum(k * speeds.get(joint, 0.0) for joint, k in motor['mix'].items())
            result[name] = clamp(mixed, -1.0, 1.0) * (-1 if motor.get('mirrored') else 1)
        return result

    def hold(self) -> dict:
        """Οι παλμοί της στάσης: νεκρό σημείο σε κάθε μοτέρ.

        Φρένο: κρατά φορτίο (π.χ. βαρύτητα στο joint_2). Με `control.stop: off` κανένας παλμός:
        το servo σίγουρα δεν κινείται μόνο του, αλλά δεν κρατά.
        """
        brake = self.cfg.get('stop', 'brake') == 'brake'
        return {m['channel']: (m['neutral_us'] if brake else None) for m in self.motors.values()}

    def pulses(self, motor_speeds: dict) -> dict:
        """{κανάλι: παλμός}: όλα στο 0 → στάση (hold)· αλλιώς όσα είναι στο 0 φρενάρουν."""
        if not any(motor_speeds.values()):
            return self.hold()
        # Μοτέρ στο 0 ενώ άλλο τρέχει: παλμός νεκρού σημείου, ώστε να φρενάρει· χωρίς παλμό το
        # γυρίζει το άλλο μέσα από τη μηχανική σύνδεση (πάγκος, 2026-11-14).
        lo, hi = self.pca['pulse_min_us'], self.pca['pulse_max_us']
        offset = self.pca['max_offset_us']
        measured = all(m.get('deadband_edges_us') for m in self.motors.values())
        if self.cfg.get('motor_compensation', True) and measured:
            span = compensation_span([(m['neutral_us'], m['deadband_edges_us'])
                                      for m in self.motors.values()], lo, hi, offset)
            return {m['channel']: compensated_pulse(motor_speeds[name], m['neutral_us'],
                                                    m['deadband_edges_us'], span)
                    for name, m in self.motors.items()}
        return {m['channel']: plain_pulse(motor_speeds[name], m['neutral_us'], lo, hi, offset)
                for name, m in self.motors.items()}

    def step(self, angles: dict, dt: float) -> dict:
        """Γωνίες → παλμοί: νόμος ανά άρθρωση → μίξη → παλμοί ανά μοτέρ."""
        return self.pulses(self.motor_speeds(self.joint_speeds(angles, dt)))
