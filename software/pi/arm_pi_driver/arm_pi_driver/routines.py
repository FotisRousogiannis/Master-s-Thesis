"""Οι διαδικασίες βαθμονόμησης του teleop.py ([6], [9], [10]), στο Pi, χωρίς ROS (ADR-0023).

Ίδια λογική με το εργαλείο πάγκου, που δοκιμάστηκε στη βάση (software/tools/wrist_bench/teleop.py):
μικρός παλμός → στάση → αναμονή να ηρεμήσει → ανάγνωση, ξανά, μέχρι να φανεί κίνηση. Κάθε
διαδικασία είναι γεννήτρια (generator) που ζητά ενέργειες· ο Runner τις εκτελεί μέσα στον κύκλο του
node (50 Hz), ώστε τίποτα να μην «κολλά» το node, και ελέγχει σε κάθε κύκλο την ασφάλεια:

    Pulse(παλμοί, s)   οι παλμοί για τόσο χρόνο, μετά κανένας παλμός (όπως το bench.stop())
    Wait(s)            περιμένει (ο βραχίονας ηρεμεί)
    Read()             → {κανάλι: γωνία} των encoders που παρακολουθούνται
    Note(κείμενο)      πρόοδος για το Arm Studio

Ασφάλεια σε ΚΑΘΕ κύκλο: encoder που χάνεται → στάση· μετακίνηση πάνω από max_travel_deg από
την αρχή της διαδικασίας → στάση. Στο τέλος ή σε διακοπή: κανένας παλμός.
"""

from dataclasses import dataclass
import math

USABLE = ('OK', 'TOO_WEAK', 'TOO_STRONG')


def wrap(delta: float) -> float:
    """Διαφορά γωνιών στο −180..180."""
    return (delta + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class Pulse:
    pulses: dict            # {κανάλι PCA: µs}
    seconds: float


@dataclass(frozen=True)
class Wait:
    seconds: float


@dataclass(frozen=True)
class Read:
    pass


@dataclass(frozen=True)
class Note:
    text: str


class RoutineError(Exception):
    """Η διαδικασία δεν μπορεί να συνεχίσει (το μήνυμα πάει στο Arm Studio)."""


class Runner:
    """Εκτελεί μία διαδικασία-γεννήτρια, ένα βήμα ανά κύκλο του node."""

    def __init__(self, routine, encoders: dict, pwm, max_travel_deg: float):
        self.routine, self.encoders, self.pwm = routine, encoders, pwm
        self.max_travel_deg = max_travel_deg
        self.notes = []                 # πρόοδος που δεν έχει σταλεί ακόμα
        self.result, self.error, self.done = None, None, False
        self._action, self._until, self._written = None, None, set()
        self._start, self._last, self._travel = None, {}, {}

    def _read(self) -> dict:
        readings = {}
        for channel, encoder in self.encoders.items():
            raw, status = encoder.read()
            readings[channel] = None if math.isnan(raw) or status not in USABLE else raw
        return readings

    def _safety(self, readings: dict) -> None:
        if self._start is None:                       # ποιοι encoders μετρούν στην αρχή
            self._start = {ch: r for ch, r in readings.items() if r is not None}
            if not self._start:
                raise RoutineError('κανένας encoder δεν μετρά')
            self._last = dict(self._start)
            self._travel = {ch: 0.0 for ch in self._start}
            return
        for channel in self._start:
            raw = readings.get(channel)
            if raw is None:
                raise RoutineError(f'ο encoder στο κανάλι {channel} σταμάτησε να μετρά')
            self._travel[channel] += wrap(raw - self._last[channel])
            self._last[channel] = raw
            if abs(self._travel[channel]) > self.max_travel_deg:
                raise RoutineError(f'κανάλι {channel}: πάνω από {self.max_travel_deg:g}° από την '
                                   'αρχή· στάση')

    def _off(self) -> None:
        for channel in sorted(self._written):
            self.pwm.off(channel)
        self._written = set()

    def stop(self, reason: str = 'διακοπή από τον χρήστη') -> None:
        """Διακοπή: κανένας παλμός, τέλος με σφάλμα."""
        if not self.done:
            self._off()
            self.error, self.done = reason, True

    def tick(self, now: float) -> None:
        """Ένας κύκλος: ασφάλεια, και η επόμενη ενέργεια όταν τελειώσει η τωρινή."""
        if self.done:
            return
        try:
            readings = self._read()
            self._safety(readings)
            while True:
                action = self._action
                if isinstance(action, Pulse) and now < self._until:
                    return
                if isinstance(action, Pulse):
                    self._off()
                if isinstance(action, Wait) and now < self._until:
                    return
                self._action = self._next(action, readings)
                if self._action is None:
                    return
                if isinstance(self._action, Pulse):
                    for channel, pulse in self._action.pulses.items():
                        self.pwm.set_pulse(channel, pulse)
                        self._written.add(channel)
                    self._until = now + self._action.seconds
                    return
                if isinstance(self._action, Wait):
                    self._until = now + self._action.seconds
                    return
        except (RoutineError, OSError) as exc:
            self._off()
            self.error = str(exc) if isinstance(exc, RoutineError) else 'το I²C δεν απαντά'
            self.done = True

    def _next(self, action, readings: dict):
        """Η επόμενη ενέργεια της γεννήτριας (Read και Note εξυπηρετούνται αμέσως)."""
        value = readings if isinstance(action, Read) else None
        while True:
            try:
                action = self.routine.send(value) if action is not None else next(self.routine)
            except StopIteration as stop:
                self._off()
                self.result, self.done = stop.value, True
                return None
            if isinstance(action, Note):
                self.notes.append(action.text)
                value = None
                continue
            if isinstance(action, Read):
                value = {ch: readings.get(ch) for ch in self._start}
                continue
            return action


# ---------------------------------------------------------------------------------------- [6]


def identify(pulses, speed=0.4, step_s=0.5, max_s=3.0, enough_deg=3.0, settle_s=0.3):
    """Αναγνώριση άρθρωσης, όπως το [6] του teleop.

    Κίνηση με «θετική» εντολή σε βήματα step_s μέχρι κάποιος encoder να κινηθεί enough_deg (το
    πολύ max_s), μετά πίσω στη θέση εκκίνησης παρακολουθώντας εκείνον τον encoder.

    pulses(ταχύτητα) → {κανάλι: µs} για τα servo της άρθρωσης με τα πρόσημα του αρχείου.
    Επιστρέφει {'moved': {κανάλι: °}, 'main': κανάλι, 'seconds', 'returned'}.
    """
    before = yield Read()
    moved, total = {ch: 0.0 for ch in before}, 0.0
    while total < max_s and max(abs(m) for m in moved.values()) < enough_deg:
        yield Pulse(pulses(speed), step_s)
        total += step_s
        yield Wait(settle_s)
        after = yield Read()
        moved = {ch: wrap(after[ch] - before[ch]) for ch in before}
        moves = '  '.join(f'κανάλι {ch} {m:+.1f}°' for ch, m in moved.items())
        yield Note(f'{total:.1f} s: {moves}')
    main = max(moved, key=lambda ch: abs(moved[ch]))
    spent, returned = 0.0, False
    while spent < 2 * total + 1.0:                      # πίσω, όσο κινήθηκε
        now = yield Read()
        offset = wrap(now[main] - before[main])
        if abs(offset) <= 1.0 or offset * moved[main] < 0:
            returned = True
            break
        yield Pulse(pulses(-speed), 0.1)
        spent += 0.1
    return {'moved': {str(ch): round(m, 2) for ch, m in moved.items()}, 'main': main,
            'seconds': round(total, 2), 'returned': returned}


# ---------------------------------------------------------------------------------------- [10]


def servo_deadband(channel, neutral, brake, step_us=10, max_us=400, hold_s=0.5, settle_s=0.2,
                   min_move_deg=0.5):
    """Νεκρή ζώνη ενός servo, όπως το [10] του teleop.

    Παλμός νεκρό ± 10, 20, … µs σε ΕΝΑ servo (τα άλλα της ομάδας φρενάρουν στο νεκρό τους
    σημείο) μέχρι να κινηθεί κάποιος encoder, προς τις δύο φορές.

    brake = {κανάλι: νεκρό σημείο} των άλλων servo. Επιστρέφει {'down_us', 'up_us'} (None όπου δεν
    κινήθηκε ως max_us).
    """
    edges = {}
    for direction in (1, -1):
        edge = None
        for offset in range(step_us, max_us + 1, step_us):
            pulse = neutral + direction * offset
            before = yield Read()
            yield Pulse({**brake, channel: pulse}, hold_s)
            yield Wait(settle_s)
            after = yield Read()
            moved = max(abs(wrap(after[ch] - before[ch])) for ch in before)
            if moved >= min_move_deg:
                edge = pulse
                break
        edges[direction] = edge
        found = f'κινείται από {edge} µs' if edge else f'όχι ως {max_us} µs'
        yield Note(f"{'πάνω' if direction > 0 else 'κάτω'}: {found}")
    return {'down_us': edges[-1], 'up_us': edges[1]}


# ---------------------------------------------------------------------------------------- [9]


def joint_deadband(pulses, channel, sign, start=0.05, step=0.02, top=0.6, hold_s=0.6, settle_s=0.2,
                   min_move_deg=0.5):
    """Νεκρή ζώνη μιας άρθρωσης, όπως το [9] του teleop.

    Εντολή που μεγαλώνει (με αντιστάθμιση νεκρής ζώνης servo) μέχρι η άρθρωση να κινηθεί
    min_move_deg ΠΡΟΣ ΤΗ ΣΩΣΤΗ ΜΕΡΙΑ, για + και −.

    channel = ο encoder της άρθρωσης, sign = −1 αν η γωνία μετρά ανάποδα (invert). Επιστρέφει
    {'pos', 'neg'} (None όπου δεν κινήθηκε ως top).
    """
    found = {}
    for direction in (1, -1):
        command, value = start, None
        while command <= top + 1e-9:
            before = yield Read()
            yield Pulse(pulses(direction * command), hold_s)
            yield Wait(settle_s)
            after = yield Read()
            moved = sign * wrap(after[channel] - before[channel])
            if moved * direction >= min_move_deg:
                value = round(command, 3)
                break
            command = round(command + step, 3)
        found[direction] = value
        text = f'κινείται από {value}' if value else f'όχι ως {top}'
        yield Note(f"{'θετικά' if direction > 0 else 'αρνητικά'}: {text}")
    return {'pos': found[1], 'neg': found[-1]}
