"""Λειτουργία αναγνώρισης (ADR-0023, βήματα 2–3 της βαθμονόμησης), χωρίς ROS.

Όσο ο driver είναι σε αναγνώριση, ο κανονικός έλεγχος θέσης ΔΕΝ τρέχει: μόνο ο Identifier γράφει
στα servo, ώστε να μην κινούν δύο πράγματα το PCA9685 ταυτόχρονα. Τρία πράγματα:

    scan()       ποια chip απαντούν σε κάθε κανάλι του TCA9548A, και η ένδειξη κάθε AS5600
    read()       ακατέργαστη ένδειξη (0–360°) και κατάσταση μαγνήτη ΟΛΩΝ των encoders, ανά κανάλι
    start_jog()  σύντομη κίνηση με όποια servo ζητηθούν (ακόμα κι αν δεν ανήκουν σε άρθρωση)· στάση
                 με την ΠΡΩΤΗ αλλαγή οποιουδήποτε encoder πάνω από stop_deg, ή όταν λήξει ο χρόνος.
                 Το αποτέλεσμα: πόσο (και προς τα πού) κινήθηκε κάθε encoder.

Ασφάλεια: χρόνος ≤ bench.max_duration_s, στάση ≤ bench.max_travel_deg, παλμός μέσα στο
neutral ± max_offset_us· αν ένας encoder που μετρούσε χαθεί στη διάρκεια, στάση αμέσως.
"""

from dataclasses import dataclass
import math

from arm_pi_driver.control.pulses import compensated_pulse, compensation_span, plain_pulse

DEVICES = {0x40: 'PCA9685', 0x36: 'AS5600', 0x48: 'ADS1115'}
USABLE = ('OK', 'TOO_WEAK', 'TOO_STRONG')
TCA_CHANNELS = range(8)
DONE, TIMEOUT, LOST = 'κινήθηκε', 'λήξη χρόνου', 'χάθηκε encoder'


def wrap(delta: float) -> float:
    """Διαφορά γωνιών στο −180..180 (η ένδειξη «γυρίζει» στο 360)."""
    return (delta + 180.0) % 360.0 - 180.0


@dataclass
class JogResult:
    """Το αποτέλεσμα μιας κίνησης αναγνώρισης."""

    moved: dict             # {κανάλι encoder: μετακίνηση σε μοίρες, με πρόσημο}
    reason: str             # DONE, TIMEOUT ή LOST
    duration_s: float
    channels: list          # τα κανάλια PCA που κινήθηκαν

    def as_dict(self) -> dict:
        """Για JSON (το Arm Studio)."""
        return {'moved': {str(k): round(v, 3) for k, v in self.moved.items()},
                'reason': self.reason, 'duration_s': round(self.duration_s, 3),
                'channels': self.channels}


class Identifier:
    """Σάρωση, ενδείξεις και κινήσεις αναγνώρισης πάνω από ένα PwmOutput και 8 encoders."""

    def __init__(self, encoders: dict, pwm, probe, pca: dict, bench: dict):
        """Με encoders = {κανάλι TCA: Encoder} και probe(κανάλι) → [διευθύνσεις που απαντούν]."""
        self.encoders, self.pwm, self.probe = encoders, pwm, probe
        self.pca, self.bench = pca, bench
        self.touched = set()            # κανάλια PCA που έγραψε η αναγνώριση
        self._jog = None

    @property
    def jogging(self) -> bool:
        """Αν τρέχει κίνηση αυτή τη στιγμή."""
        return self._jog is not None

    def read(self) -> dict:
        """{κανάλι: {'raw_deg', 'status'}} για όλους τους encoders (raw_deg None αν δεν μετρά)."""
        result = {}
        for channel, encoder in self.encoders.items():
            raw, status = encoder.read()
            result[channel] = {'raw_deg': None if math.isnan(raw) else round(raw, 3),
                               'status': status}
        return result

    def scan(self) -> dict:
        """{κανάλι: {'devices': [ονόματα], 'encoder': ένδειξη ή None}} για τα 8 κανάλια."""
        readings = self.read()
        result = {}
        for channel in TCA_CHANNELS:
            found = [DEVICES.get(a, hex(a)) for a in self.probe(channel)]
            encoder = readings.get(channel) if 'AS5600' in found else None
            result[channel] = {'devices': found, 'encoder': encoder}
        return result

    def start_jog(self, servos: list, duration_s: float, stop_deg: float, now: float) -> None:
        """Ξεκινά κίνηση: servos = [(κανάλι PCA, νεκρό σημείο µs, νεκρή ζώνη ή None, ταχύτητα)].

        Αν όλα τα servo έχουν μετρημένη νεκρή ζώνη, η ταχύτητα μετρά από την άκρη της (όπως στον
        driver): ίδια ταχύτητα → ίδια κίνηση σε κάθε servo, αλλιώς ένα διαφορικό «ξεφεύγει» και
        στη δεύτερη άρθρωση. ValueError αν κάτι ξεπερνά τα όρια ασφαλείας ή τρέχει ήδη κίνηση.
        """
        if self._jog is not None:
            raise ValueError('τρέχει ήδη κίνηση')
        if not servos:
            raise ValueError('καμία κίνηση: δεν δόθηκε servo')
        if not 0 < duration_s <= self.bench['max_duration_s']:
            raise ValueError(f'διάρκεια {duration_s} s (επιτρέπεται έως '
                             f"{self.bench['max_duration_s']})")
        if not 0 < stop_deg <= self.bench['max_travel_deg']:
            raise ValueError(f'στάση στις {stop_deg}° (επιτρέπεται έως '
                             f"{self.bench['max_travel_deg']})")
        lo, hi, offset = (self.pca[k] for k in ('pulse_min_us', 'pulse_max_us', 'max_offset_us'))
        for channel, _, _, speed in servos:
            if not -1.0 <= speed <= 1.0:
                raise ValueError(f'κανάλι {channel}: ταχύτητα {speed} (επιτρέπεται −1..1)')
        if all(edges for _, _, edges, _ in servos):
            span = compensation_span([(n, e) for _, n, e, _ in servos], lo, hi, offset)
            pulses = {ch: compensated_pulse(v, n, e, span) for ch, n, e, v in servos}
        else:
            pulses = {ch: plain_pulse(v, n, lo, hi, offset) for ch, n, _, v in servos}
        start = {ch: r['raw_deg'] for ch, r in self.read().items()
                 if r['raw_deg'] is not None and r['status'] in USABLE}
        self._jog = {'start': now, 'duration': duration_s, 'stop_deg': stop_deg,
                     'neutral': {ch: n for ch, n, _, _ in servos}, 'last': dict(start),
                     'moved': {ch: 0.0 for ch in start}}
        self._write(pulses)

    def tick(self, now: float) -> JogResult | None:
        """Ένας κύκλος της κίνησης· το αποτέλεσμα όταν τελειώσει, αλλιώς None."""
        jog = self._jog
        if jog is None:
            return None
        reason = None
        for channel, reading in self.read().items():
            if channel not in jog['last']:
                continue
            if reading['raw_deg'] is None or reading['status'] not in USABLE:
                reason = LOST
                continue
            jog['moved'][channel] += wrap(reading['raw_deg'] - jog['last'][channel])
            jog['last'][channel] = reading['raw_deg']
        if reason is None and any(abs(d) >= jog['stop_deg'] for d in jog['moved'].values()):
            reason = DONE
        if reason is None and now - jog['start'] >= jog['duration']:
            reason = TIMEOUT
        if reason is None:
            return None
        self.stop()
        moved = {ch: d for ch, d in jog['moved'].items() if abs(d) >= 0.2}    # όχι θόρυβος
        return JogResult(moved, reason, now - jog['start'], sorted(jog['neutral']))

    def stop(self) -> None:
        """Φρένο (νεκρό σημείο) στα servo της κίνησης· τίποτα αν δεν τρέχει κίνηση."""
        if self._jog is not None:
            neutral, self._jog = self._jog['neutral'], None
            self._write(neutral)

    def release(self) -> None:
        """Έξοδος από την αναγνώριση: κανένας παλμός σε όσα κανάλια έγραψε."""
        self._jog = None
        for channel in sorted(self.touched):
            self.pwm.off(channel)
        self.touched = set()

    def _write(self, pulses: dict) -> None:
        for channel, pulse in pulses.items():
            self.pwm.set_pulse(channel, pulse)
            self.touched.add(channel)


class Session:
    """Τα αιτήματα του Arm Studio προς την αναγνώριση (JSON), χωρίς ROS.

    Κάθε αίτημα {'id', 'op', ...} βάζει τον driver σε αναγνώριση (αν δεν ήταν) και κρατά ζωντανή
    την κατάσταση· χωρίς αίτημα για timeout_s, έξοδος μόνη της (το Arm Studio έκλεισε ή χάθηκε):

        {'op': 'scan'}                                     → σάρωση I²C
        {'op': 'read'}                                     → ενδείξεις encoders (και «είμαι εδώ»)
        {'op': 'jog', 'servos': {'servo_1': 0.4, ...}, 'duration_s': 0.5, 'stop_deg': 3.0}
                                                           → ξεκινά· το αποτέλεσμα έρχεται αργότερα
        {'op': 'stop'}                                     → στάση της κίνησης
        {'op': 'exit'}                                     → πίσω στον κανονικό έλεγχο

    Τα servo με το όνομά τους στο arm.yaml (κανάλι, νεκρό σημείο, νεκρή ζώνη): ακόμα κι αν δεν
    ανήκουν σε άρθρωση, αρκεί να έχουν κανάλι.
    """

    def __init__(self, identifier: Identifier, data: dict, timeout_s: float = 5.0):
        self.identifier, self.data, self.timeout_s = identifier, data, timeout_s
        self.active = False
        self.last_request = None
        self._jog_id = None

    def handle(self, request: dict, now: float) -> dict:
        """Ένα αίτημα → η άμεση απάντηση."""
        op, reply = request.get('op'), {'id': request.get('id'), 'op': request.get('op')}
        try:
            if op == 'exit':
                self.exit()
                return {**reply, 'ok': True, 'result': 'κανονικός έλεγχος'}
            self.active, self.last_request = True, now
            if op == 'scan':
                result = self.identifier.scan()
            elif op == 'read':
                result = self.identifier.read()
            elif op == 'jog':
                self.identifier.start_jog(self._servos(request.get('servos') or {}),
                                          float(request.get('duration_s', 0.5)),
                                          float(request.get('stop_deg', 3.0)), now)
                self._jog_id = request.get('id')
                result = 'ξεκίνησε'
            elif op == 'stop':
                self.identifier.stop()
                result = 'στάση'
            else:
                raise ValueError(f'άγνωστη εντολή {op!r} (scan, read, jog, stop, exit)')
        except (ValueError, TypeError) as exc:
            return {**reply, 'ok': False, 'error': str(exc)}
        except OSError:
            self.identifier.stop()
            return {**reply, 'ok': False, 'error': 'το I²C δεν απαντά'}
        return {**reply, 'ok': True, 'result': result}

    def tick(self, now: float) -> list:
        """Ένας κύκλος: το αποτέλεσμα μιας κίνησης που τελείωσε, ή έξοδος λόγω σιωπής."""
        if not self.active:
            return []
        if now - self.last_request > self.timeout_s:
            self.exit()
            return [{'id': None, 'op': 'exit', 'ok': True,
                     'result': f'καμία εντολή για > {self.timeout_s:g} s: κανονικός έλεγχος'}]
        try:
            result = self.identifier.tick(now)
        except OSError:
            self.identifier.stop()
            return [{'id': self._jog_id, 'op': 'jog', 'ok': False, 'error': 'το I²C δεν απαντά'}]
        if result is None:
            return []
        return [{'id': self._jog_id, 'op': 'jog', 'ok': True, 'result': result.as_dict()}]

    def exit(self) -> None:
        """Πίσω στον κανονικό έλεγχο: κανένας παλμός σε όσα κανάλια κίνησε η αναγνώριση."""
        self.identifier.release()
        self.active = False

    def _servos(self, speeds: dict) -> list:
        servos = []
        for name, speed in speeds.items():
            servo = self.data['servos'].get(name)
            if servo is None:
                raise ValueError(f'άγνωστο servo {name!r}')
            if servo.get('channel') is None:
                raise ValueError(f'το {name} δεν έχει κανάλι (βήμα 1: κανάλια servo)')
            servos.append((servo['channel'], servo['neutral_us'], servo.get('deadband_us'),
                           float(speed)))
        return servos
