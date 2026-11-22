"""Η λογική του driver χωρίς ROS: encoders → θέσεις, εντολές → παλμοί, με ελέγχους ασφαλείας.

Ο Driver έχει μια λίστα από Controller (control/), έναν ανά ομάδα αρθρώσεων του YAML, και δεν ξέρει
ποιος είναι ποιος. Η ασφάλεια είναι κοινή: πρόβλημα σε ΜΙΑ ομάδα σταματά ΟΛΕΣ (ένας βραχίονας).

Κάθε tick (π.χ. 50 Hz):
  1. διαβάζει κάθε encoder· η θέση (μοίρες) με unwrapping από το arm_calibration (ADR-0019)
  2. κινεί τα μοτέρ ΜΟΝΟ αν: ήρθε εντολή πρόσφατα (watchdog), όλες οι αρθρώσεις έχουν έγκυρη
     θέση, και καμία δεν είναι εκτός ορίων· αλλιώς στάση: παλμός νεκρού σημείου (φρένο)
  3. επιστρέφει αναφορά (Report) για το node: θέσεις, καταστάσεις, και γεγονότα μόνο όταν
     κάτι αλλάζει

Το node (node.py) μόνο μεταφέρει τα παραπάνω από/προς το ROS 2.
"""

from dataclasses import dataclass

from arm_calibration.encoders import EncoderJump, Tracker
from arm_pi_driver.arm_file import Change, GRIPPER_GROUP, JOINT, RESTART, SERVO
from arm_pi_driver.control import build_controllers, joint_configs, LimitError

USABLE = ('OK', 'TOO_WEAK', 'TOO_STRONG')   # η γωνία μετρά, ίσως λιγότερο αξιόπιστα
JUMP = 'JUMP'                               # η ένδειξη άλλαξε πιο πολύ απ' όσο γίνεται: βλάβη
IDLE = 'info'                               # επίπεδα γεγονότων, ίδια ονόματα με τον logger του ROS
WARNING = 'warning'
ERROR = 'error'
MOVING = 'κίνηση'
AT_TARGET = 'στον στόχο'
NO_COMMAND = 'χωρίς εντολή'


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
    reason: str                     # MOVING, AT_TARGET, ή γιατί δεν κινείται
    level: str                      # IDLE, WARNING ή ERROR για το reason
    events: list                    # [(επίπεδο, κείμενο)] μόνο για όσα άλλαξαν
    extra: list                     # διαγνωστικά των controllers (π.χ. αισθητήρες της αρπάγης)


class Driver:
    """Όλες οι ομάδες αρθρώσεων ενός ελεγκτή: encoders, PwmOutput, controllers και ασφάλεια."""

    def __init__(self, config: dict, encoders: dict, pwm, watchdog_s: float = 0.2,
                 force_sensors: dict | None = None, controllers: list | None = None):
        self.config, self.encoders, self.pwm = config, encoders, pwm
        self.watchdog_s = watchdog_s
        self.joints = joint_configs(config)
        if controllers is None:
            controllers = build_controllers(config, force_sensors)
        self.controllers = controllers
        self.owner = {joint: c for c in self.controllers for joint in c.joints}
        self.trackers = {name: self._new_tracker(name) for name in self.joints}
        self.channels = [channel for c in self.controllers for channel in c.channels]
        self.last_command = None
        self.last_tick = None
        self.written = {}               # {κανάλι: τελευταίος παλμός που γράφτηκε}
        self._status = {}
        self._reason = None
        self._pending = []              # γεγονότα από αλλαγές βαθμονόμησης, για το επόμενο tick

    def _new_tracker(self, name: str) -> Tracker:
        # Τα όρια διευρυμένα κατά το περιθώριο ασφαλείας: η θέση μένει συνεχής και λίγο έξω από
        # τα όρια (εκεί τη χρειάζεται ο έλεγχος ορίων), και μια αρπάγη χωρίς βαθμονόμηση
        # (όρια [0, 0]) έχει ακόμα εύρος.
        joint, margin = self.joints[name], self.config['control']['limit_margin_deg']
        inverted = joint['sign'] < 0
        return Tracker(joint['zero_deg'], inverted, joint['lower_deg'] - margin,
                       joint['upper_deg'] + margin)

    def update_joint(self, name: str, fields: dict) -> bool:
        """Νέα βαθμονόμηση μιας άρθρωσης (μηδέν, φορά, όρια) ενώ τρέχει· True αν μας αφορά.

        Οι θέσεις «πηδούν» με νέο μηδέν, οπότε για ασφάλεια ξεχνιούνται όλοι οι στόχοι: τα μοτέρ
        σταματούν μέχρι την επόμενη εντολή. Τα controllers βλέπουν τα ίδια λεξικά αρθρώσεων, άρα
        και τα νέα όρια.
        """
        if name not in self.joints or not fields:
            return False
        self.joints[name].update(fields)
        self.trackers[name] = self._new_tracker(name)
        for controller in self.controllers:
            controller.clear_targets()
        text = ', '.join(f'{k} = {v:g}' for k, v in fields.items())
        self._pending.append((WARNING, f'νέα βαθμονόμηση {name}: {text} · στόχοι μηδενίστηκαν'))
        return True

    def apply_change(self, change: Change) -> bool:
        """Μια αλλαγή του arm.yaml (από το Arm Studio) ενώ τρέχει· True αν εφαρμόστηκε."""
        if change.kind == JOINT:
            return self.update_joint(change.name, change.fields)
        if change.kind == SERVO:
            return self.update_servo(change.name, change.fields)
        if change.kind == GRIPPER_GROUP:
            groups = [g for g in self.config['groups'].values()
                      if 'force_sensors' in g and change.name in g['joints']]
            for group in groups:
                group.update(change.fields)
            if groups:
                levels = ', '.join(f'{n} {v:.2f} V' for n, v in change.fields['contact'].items())
                self._pending.append((IDLE, f'αρπάγη: όρια επαφής {levels or "κανένα"}'))
            return bool(groups)
        if change.kind == RESTART:
            # Κανάλια, servo/encoder μιας άρθρωσης, enabled, νόμοι ελέγχου: αλλάζουν τη δομή·
            # ζωντανά θα έστελναν παλμούς σε άλλο servo. Ισχύουν μετά από επανεκκίνηση.
            self._pending.append((WARNING, f'{change.name}: άλλαξε στο arm.yaml· ισχύει μετά από '
                                           f'επανεκκίνηση του driver'))
            return True
        return False

    def update_servo(self, servo: str, fields: dict) -> bool:
        """Νέο νεκρό σημείο / νεκρή ζώνη ενός servo ενώ τρέχει.

        Για ασφάλεια ξεχνιούνται όλοι οι στόχοι, όπως στις αρθρώσεις.
        """
        motors = [m for g in self.config['groups'].values() for m in g['motors'].values()
                  if m.get('servo') == servo]
        if not motors or not fields:
            return False
        for motor in motors:
            motor.update(fields)
        for controller in self.controllers:
            controller.clear_targets()
        self.written = {}                   # ο παλμός στάσης μπορεί να άλλαξε: ξαναγράφεται
        self._pending.append((WARNING, f'νέα βαθμονόμηση servo {servo} · στόχοι μηδενίστηκαν'))
        return True

    def command(self, targets_deg: dict, now: float) -> dict:
        """Νέοι στόχοι (μοίρες)· επιστρέφει τους στόχους που ισχύουν.

        Άγνωστα ονόματα αγνοούνται· όποια άρθρωση λείπει κρατά τον στόχο της, ή αν δεν έχει,
        μένει εκεί που είναι.
        """
        targets = {n: t for n, t in targets_deg.items() if n in self.owner}
        if not targets:
            return self.targets
        for name, controller in self.owner.items():
            position = self.trackers[name].position
            has_target = name in targets or name in controller.targets
            if not has_target and position is not None:
                targets[name] = position
        self.last_command = now
        for controller in self.controllers:
            mine = {n: t for n, t in targets.items() if n in controller.joints}
            if mine:
                controller.set_targets(mine)
        return self.targets

    @property
    def targets(self) -> dict:
        """Οι στόχοι που ισχύουν, από όλες τις ομάδες."""
        return {n: t for c in self.controllers for n, t in c.targets.items()}

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
        stop = {}
        for controller in self.controllers:
            stop.update(controller.hold())          # φρένο (νεκρό σημείο), ή κανένας παλμός
        if self.last_command is None:
            return stop, NO_COMMAND, IDLE
        if now - self.last_command > self.watchdog_s:
            return stop, f'καμία εντολή για > {self.watchdog_s:g} s (watchdog)', IDLE
        for name, joint in joints.items():
            if joint.position_deg is None:
                return stop, f'{name}: {joint.status}', WARNING
        angles = {name: joint.position_deg for name, joint in joints.items()}
        pulses = {}
        try:
            for controller in self.controllers:
                pulses.update(controller.step({n: angles[n] for n in controller.joints}, dt))
        except LimitError as exc:
            return stop, str(exc), WARNING
        if pulses == stop:
            return stop, AT_TARGET, IDLE
        return pulses, MOVING, IDLE

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

    def servo_statuses(self, failure: str | None) -> list:
        """servo/<όνομα> για τις Συσκευές (ADR-0018): ο παλμός που γράφτηκε, ή το σφάλμα του PCA.

        Το servo δεν απαντά (PWM ένα προς ένα): «ενεργό» σημαίνει μόνο ότι ο PCA9685 δέχτηκε τον
        παλμό του, όχι ότι το μοτέρ γυρίζει.
        """
        out = []
        for group in self.config['groups'].values():
            for key, motor in group['motors'].items():
                name, channel = f"servo/{motor.get('servo', key)}", motor['channel']
                pulse = self.written.get(channel)
                shown = 'off' if pulse is None else f'{pulse:.0f}'
                values = {'channel': channel, 'pulse_us': shown}
                if failure:
                    out.append((name, ERROR, failure, values))
                else:
                    text = 'χωρίς παλμό' if pulse is None else f'παλμός {pulse:.0f} µs'
                    out.append((name, IDLE, text, values))
        return out

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
        if reason not in (MOVING, AT_TARGET):
            # Στάση ασφαλείας (watchdog, encoder, όρια, PCA): οι στόχοι ξεχνιούνται. Μετά τη
            # διακοπή, όποια άρθρωση δεν αναφέρει η επόμενη εντολή μένει εκεί που είναι ΤΟΤΕ,
            # όχι εκεί που της είχαν πει πριν (μπορεί να μετακινήθηκε με το χέρι στο μεταξύ).
            for controller in self.controllers:
                controller.clear_targets()
        extra = [item for controller in self.controllers for item in controller.diagnostics()]
        extra += self.servo_statuses(failure)
        return Report(joints, moving, reason, level, self._events(joints, reason, level), extra)

    def _events(self, joints: dict, reason: str, level: str) -> list:
        events, self._pending = self._pending, []
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
        """Καθόλου παλμός πουθενά (τερματισμός)· αθόρυβα αν το PCA9685 δεν απαντά.

        ΟΧΙ φρένο εδώ: μετά τον τερματισμό το PCA9685 θα κρατούσε το νεκρό σημείο για πάντα, χωρίς
        watchdog, και ένα ατελές νεκρό σημείο «σέρνει» το servo. Όσο το node τρέχει, η στάση
        είναι φρένο (hold) και την επιβλέπουν watchdog και encoders.
        """
        try:
            self.pwm.all_off()
        except OSError:
            pass
        self.written = {}
