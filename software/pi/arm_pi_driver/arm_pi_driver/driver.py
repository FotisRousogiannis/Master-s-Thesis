"""Η λογική του driver χωρίς ROS: encoders → θέσεις, εντολές → παλμοί, με ελέγχους ασφαλείας.

Κάθε tick (π.χ. 50 Hz):
  1. διαβάζει κάθε encoder· η θέση (μοίρες) με unwrapping από το arm_calibration (ADR-0019)
  2. κινεί τα μοτέρ ΜΟΝΟ αν: ήρθε εντολή πρόσφατα (watchdog), όλες οι αρθρώσεις έχουν έγκυρη
     θέση, και καμία δεν είναι εκτός ορίων· αλλιώς καθόλου παλμός (τα servo σταματούν)
  3. επιστρέφει αναφορά (Report) για το node: θέσεις, καταστάσεις, και γεγονότα μόνο όταν
     κάτι αλλάζει

Το node (node.py) μόνο μεταφέρει τα παραπάνω από/προς το ROS 2.
"""

from dataclasses import dataclass
import math

from arm_calibration.encoders import EncoderJump, Tracker
from arm_pi_driver.control import DifferentialController, LimitError

USABLE = ('OK', 'TOO_WEAK', 'TOO_STRONG')   # η γωνία μετρά, ίσως λιγότερο αξιόπιστα
JUMP = 'JUMP'                               # η ένδειξη άλλαξε πιο πολύ απ' όσο γίνεται: βλάβη
IDLE = 'info'                               # επίπεδα γεγονότων, ίδια ονόματα με τον logger του ROS
WARNING = 'warning'
ERROR = 'error'
MOVING = 'κίνηση'


@dataclass
class JointReport:
    """Μια άρθρωση σε ένα tick."""

    status: str                     # OK, TOO_WEAK, TOO_STRONG, NO_MAGNET, NO_RESPONSE, JUMP
    raw_deg: float                  # η ένδειξη του encoder 0–360 (nan αν δεν απαντά)
    position_deg: float | None      # η θέση της άρθρωσης· None αν δεν υπάρχει έγκυρη


@dataclass
class Report:
    """Ένα tick: θέσεις, αν κινείται και γιατί όχι, και τι άλλαξε από το προηγούμενο."""

    joints: dict                    # {άρθρωση: JointReport}
    moving: bool
    reason: str                     # MOVING, 'στον στόχο', ή γιατί δεν κινείται
    level: str                      # IDLE, WARNING ή ERROR για το reason
    events: list                    # [(επίπεδο, κείμενο)] μόνο για όσα άλλαξαν


class Driver:
    """Ένα διαφορικό (δύο αρθρώσεις, δύο μοτέρ): encoders, PwmOutput, έλεγχος και ασφάλεια."""

    def __init__(self, config: dict, encoders: dict, pwm, watchdog_s: float = 0.2):
        self.config, self.encoders, self.pwm = config, encoders, pwm
        self.watchdog_s = watchdog_s
        self.controller = DifferentialController(config)
        self.trackers = {name: self._new_tracker(name) for name in config['joints']}
        self.channels = [m['channel'] for m in config['motors'].values()]
        self.last_command = None
        self.last_tick = None
        self.written = {}               # {κανάλι: τελευταίος παλμός που γράφτηκε}
        self._status = {}
        self._reason = None

    def _new_tracker(self, name: str) -> Tracker:
        joint = self.config['joints'][name]
        inverted = joint['sign'] < 0
        return Tracker(joint['zero_deg'], inverted, joint['lower_deg'], joint['upper_deg'])

    def command(self, targets_deg: dict, now: float) -> dict:
        """Νέοι στόχοι (μοίρες)· επιστρέφει τους στόχους που ισχύουν.

        Άγνωστα ονόματα αγνοούνται· όποια άρθρωση λείπει κρατά τον στόχο της, ή αν δεν έχει,
        μένει εκεί που είναι.
        """
        targets = {n: t for n, t in targets_deg.items() if n in self.config['joints']}
        if not targets:
            return dict(self.controller.targets)
        for name in self.config['joints']:
            position = self.trackers[name].position
            has_target = name in targets or name in self.controller.targets
            if not has_target and position is not None:
                targets[name] = position
        self.last_command = now
        return self.controller.set_targets(targets)

    def read_joints(self) -> dict:
        """{άρθρωση: JointReport} από μία ανάγνωση κάθε encoder."""
        joints = {}
        for name, encoder in self.encoders.items():
            raw, status = encoder.read()
            position = None
            if status in USABLE:
                try:
                    position = self.trackers[name].update(raw)
                except EncoderJump:
                    status = JUMP       # η μέτρηση απορρίφθηκε· η προηγούμενη θέση μένει
            else:
                # Χωρίς μέτρηση η άρθρωση μπορεί να μετακινηθεί (με το χέρι): όταν ξαναβρεθεί,
                # η θέση ξεκινά από την απόλυτη ένδειξη, όχι από την παλιά.
                self.trackers[name] = self._new_tracker(name)
            joints[name] = JointReport(status, raw, position)
        return joints

    def decide(self, joints: dict, now: float, dt: float) -> tuple:
        """(παλμοί, λόγος, επίπεδο): κίνηση μόνο αν όλα είναι ασφαλή."""
        stop = {channel: None for channel in self.channels}
        if self.last_command is None:
            return stop, 'χωρίς εντολή', IDLE
        if now - self.last_command > self.watchdog_s:
            return stop, f'καμία εντολή για > {self.watchdog_s:g} s (watchdog)', IDLE
        for name, joint in joints.items():
            if joint.position_deg is None:
                return stop, f'{name}: {joint.status}', WARNING
        angles = {name: joint.position_deg for name, joint in joints.items()}
        try:
            speeds, _ = self.controller.axis_speeds(angles, dt)
        except LimitError as exc:
            return stop, str(exc), WARNING
        if not any(speeds.values()):
            return stop, 'στον στόχο', IDLE
        return self.controller.pulses(speeds), MOVING, IDLE

    def write(self, pulses: dict) -> str | None:
        """Γράφει όσους παλμούς άλλαξαν· None αν πέτυχε, αλλιώς το σφάλμα."""
        try:
            for channel, pulse in pulses.items():
                if channel in self.written and self.written[channel] == pulse:
                    continue
                if pulse is None:
                    self.pwm.off(channel)
                else:
                    self.pwm.set_pulse(channel, pulse)
                self.written[channel] = pulse
        except OSError:
            self.written = {}               # άγνωστο τι έχει τώρα: ξαναγράφονται όλα
            return 'PCA9685 δεν απαντά'
        return None

    def tick(self, now: float) -> Report:
        """Ένας κύκλος: ανάγνωση, απόφαση, έξοδοι."""
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        joints = self.read_joints()
        pulses, reason, level = self.decide(joints, now, dt)
        failure = self.write(pulses)
        if failure:
            reason, level = failure, ERROR
        moving = reason == MOVING
        if not moving:
            for pid in self.controller.pids.values():
                pid.reset()
        return Report(joints, moving, reason, level, self._events(joints, reason, level))

    def _events(self, joints: dict, reason: str, level: str) -> list:
        events = []
        for name, joint in joints.items():
            if self._status.get(name) != joint.status:
                if joint.status == 'OK':
                    events.append((IDLE, f'encoder {name}: OK'))
                elif joint.status in USABLE:
                    text = f'encoder {name}: {joint.status} (μετρά, με επιφύλαξη)'
                    events.append((WARNING, text))
                else:
                    events.append((ERROR, f'encoder {name}: {joint.status}'))
                self._status[name] = joint.status
        if reason != self._reason:
            events.append((level, f'έξοδοι: {reason}'))
            self._reason = reason
        return events

    def stop(self) -> None:
        """Καθόλου παλμός πουθενά (τερματισμός)· αθόρυβα αν το PCA9685 δεν απαντά."""
        try:
            self.pwm.all_off()
        except OSError:
            pass
        self.written = {}

    @staticmethod
    def angle_or_none(value: float) -> float | None:
        """Για εμφάνιση: None αντί για nan."""
        return None if value is None or math.isnan(value) else value
