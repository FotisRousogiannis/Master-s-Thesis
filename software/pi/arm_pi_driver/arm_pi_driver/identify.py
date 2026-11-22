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
LIVE_PERIOD_S = 0.2       # ζωντανές ενδείξεις προς το Arm Studio (5 Hz)
DRIVE_HOLD_S = 0.3        # «κράτα το κουμπί»: χωρίς νέο drive τόσο → κανένας παλμός


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

    def pulses(self, servos: list) -> dict:
        """Παλμοί {κανάλι: µs} για servos = [(κανάλι, νεκρό σημείο, νεκρή ζώνη|None, ταχύτητα)].

        Αν όλα τα servo έχουν μετρημένη νεκρή ζώνη, η ταχύτητα μετρά από την άκρη της (όπως στον
        driver): ίδια ταχύτητα → ίδια κίνηση σε κάθε servo. ValueError για ταχύτητα εκτός −1..1.
        """
        lo, hi, offset = (self.pca[k] for k in ('pulse_min_us', 'pulse_max_us', 'max_offset_us'))
        for channel, _, _, speed in servos:
            if not -1.0 <= speed <= 1.0:
                raise ValueError(f'κανάλι {channel}: ταχύτητα {speed} (επιτρέπεται −1..1)')
        if all(edges for _, _, edges, _ in servos):
            span = compensation_span([(n, e) for _, n, e, _ in servos], lo, hi, offset)
            return {ch: compensated_pulse(v, n, e, span) for ch, n, e, v in servos}
        return {ch: plain_pulse(v, n, lo, hi, offset) for ch, n, _, v in servos}

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
        pulses = self.pulses(servos)
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
                                                             ('plain': true → χωρίς αντιστάθμιση
                                                             νεκρής ζώνης, για να μετρηθεί)
        {'op': 'drive', 'servos': {'servo_1': 0.4, ...}}   → κίνηση ΟΣΟ ξαναστέλνεται (≤ 0.3 s,
                                                             όπως το «κράτα το πλήκτρο» του teleop)
        {'op': 'routine', 'name': 'identify' | 'servo_deadband' | 'joint_deadband', ...}
                                                           → διαδικασία teleop ([6], [10], [9]):
                                                             πρόοδος {'op': 'progress'} και στο
                                                             τέλος {'op': 'routine'} (routines.py)
        {'op': 'ping'}                                     → «είμαι εδώ»
        {'op': 'stop'}                                     → στάση κάθε κίνησης
        {'op': 'exit'}                                     → πίσω στον κανονικό έλεγχο

    Όσο είναι ενεργή, στέλνει μόνη της ζωντανές ενδείξεις {'op': 'live'} κάθε LIVE_PERIOD_S.

    Τα servo με το όνομά τους στο arm.yaml (κανάλι, νεκρό σημείο, νεκρή ζώνη): ακόμα κι αν δεν
    ανήκουν σε άρθρωση, αρκεί να έχουν κανάλι.
    """

    def __init__(self, identifier: Identifier, data: dict, timeout_s: float = 5.0):
        self.identifier, self.data, self.timeout_s = identifier, data, timeout_s
        self.active = False
        self.last_request = None
        self._jog_id = None
        self.runner, self._routine_id = None, None
        self._drive = None              # {'until': χρόνος, 'start': {κανάλι: °}, 'travel': {…}}
        self._live_at = None

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
                plain = bool(request.get('plain', False))
                self.identifier.start_jog(self._servos(request.get('servos') or {}, plain),
                                          float(request.get('duration_s', 0.5)),
                                          float(request.get('stop_deg', 3.0)), now)
                self._jog_id = request.get('id')
                result = 'ξεκίνησε'
            elif op == 'ping':
                result = 'pong'
            elif op == 'drive':
                result = self._start_drive(request.get('servos') or {}, now)
            elif op == 'routine':
                self._start_routine(request)
                self._routine_id = request.get('id')
                result = 'ξεκίνησε'
            elif op == 'stop':
                self.stop_all()
                result = 'στάση'
            else:
                raise ValueError(f'άγνωστη εντολή {op!r} (scan, read, jog, drive, routine, '
                                 'ping, stop, exit)')
        except (ValueError, TypeError) as exc:
            return {**reply, 'ok': False, 'error': str(exc)}
        except OSError:
            self.identifier.stop()
            return {**reply, 'ok': False, 'error': 'το I²C δεν απαντά'}
        return {**reply, 'ok': True, 'result': result}

    def tick(self, now: float) -> list:
        """Ένας κύκλος: κίνηση/διαδικασία που τρέχει, ζωντανές ενδείξεις, ή έξοδος λόγω σιωπής."""
        if not self.active:
            return []
        if now - self.last_request > self.timeout_s:
            self.exit()
            return [{'id': None, 'op': 'exit', 'ok': True,
                     'result': f'καμία εντολή για > {self.timeout_s:g} s: κανονικός έλεγχος'}]
        replies = []
        try:
            result = self.identifier.tick(now)
            if result is not None:
                replies.append({'id': self._jog_id, 'op': 'jog', 'ok': True,
                                'result': result.as_dict()})
            replies += self._tick_drive(now)
            replies += self._tick_routine(now)
            if self._live_at is None or now - self._live_at >= LIVE_PERIOD_S:
                self._live_at = now
                replies.append({'id': None, 'op': 'live', 'ok': True,
                                'result': self.identifier.read()})
        except OSError:
            self.stop_all()
            replies.append({'id': None, 'op': 'error', 'ok': False, 'error': 'το I²C δεν απαντά'})
        return replies

    def stop_all(self) -> None:
        """Κάθε κίνηση σταματά: jog (φρένο), drive και διαδικασία (κανένας παλμός)."""
        self.identifier.stop()
        if self._drive is not None:
            self._drive_off()
        if self.runner is not None and not self.runner.done:
            self.runner.stop()

    def exit(self) -> None:
        """Πίσω στον κανονικό έλεγχο: κανένας παλμός σε όσα κανάλια κίνησε η αναγνώριση."""
        self.stop_all()
        self.identifier.release()
        self.runner, self._drive = None, None
        self.active = False

    # ---------------------------------------------------------------- drive (κράτα το κουμπί)
    def _start_drive(self, speeds: dict, now: float) -> str:
        if self.runner is not None and not self.runner.done:
            raise ValueError('τρέχει διαδικασία: «Διακοπή» πρώτα')
        pulses = self.identifier.pulses(self._servos(speeds))
        if self._drive is None:
            start = {ch: r['raw_deg'] for ch, r in self.identifier.read().items()
                     if r['raw_deg'] is not None and r['status'] == 'OK'}
            self._drive = {'start': start, 'last': dict(start),
                           'travel': {ch: 0.0 for ch in start}, 'channels': set()}
        for channel, pulse in pulses.items():
            self.identifier.pwm.set_pulse(channel, pulse)
            self._drive['channels'].add(channel)
        self._drive['until'] = now + DRIVE_HOLD_S
        return 'κίνηση'

    def _drive_off(self) -> None:
        for channel in sorted(self._drive['channels']):
            self.identifier.pwm.off(channel)
        self._drive = None

    def _tick_drive(self, now: float) -> list:
        drive = self._drive
        if drive is None:
            return []
        if now > drive['until']:                     # το κουμπί αφέθηκε (ή χάθηκε η σύνδεση)
            self._drive_off()
            return []
        readings = self.identifier.read()
        for channel in drive['start']:               # όριο διαδρομής, όπως το teleop (μαγνήτες OK)
            raw = readings[channel]['raw_deg']
            if raw is None:
                continue
            drive['travel'][channel] += wrap(raw - drive['last'][channel])
            drive['last'][channel] = raw
            limit = self.identifier.bench.get('jog_max_travel_deg', 90.0)
            if abs(drive['travel'][channel]) > limit:
                self._drive_off()
                return [{'id': None, 'op': 'drive', 'ok': False,
                         'error': f'όριο {limit:g}° στο κανάλι {channel}: στάση'}]
        return []

    # ---------------------------------------------------------------- διαδικασίες του teleop
    def _start_routine(self, request: dict) -> None:
        from arm_pi_driver import routines
        if self.runner is not None and not self.runner.done:
            raise ValueError('τρέχει ήδη διαδικασία')
        if self._drive is not None:
            self._drive_off()
        name = request.get('name')

        def pulses_of(signs, plain=False):
            servos = self._servos({s: 1.0 for s in signs}, plain)
            return lambda speed: self.identifier.pulses(
                [(ch, n, e, sign * speed) for (ch, n, e, _), sign in zip(servos, signs.values())])

        if name == 'identify':
            routine = routines.identify(pulses_of(request['servos']),
                                        float(request.get('speed', 0.4)),
                                        float(request.get('step_s', 0.5)),
                                        float(request.get('max_s', 3.0)))
        elif name == 'servo_deadband':
            servo = self.data['servos'][request['servo']]
            if servo.get('channel') is None:
                raise ValueError(f"το {request['servo']} δεν έχει κανάλι")
            others = [self.data['servos'][b] for b in request.get('brake', [])]
            brake = {o['channel']: o['neutral_us'] for o in others if o.get('channel') is not None}
            routine = routines.servo_deadband(servo['channel'], servo['neutral_us'], brake)
        elif name == 'joint_deadband':
            routine = routines.joint_deadband(pulses_of(request['servos']),
                                              int(request['channel']),
                                              -1 if request.get('invert') else 1)
        else:
            raise ValueError(f'άγνωστη διαδικασία {name!r} '
                             '(identify, servo_deadband, joint_deadband)')
        self.runner = routines.Runner(routine, self.identifier.encoders, self.identifier.pwm,
                                      self.identifier.bench['max_travel_deg'])

    def _tick_routine(self, now: float) -> list:
        runner = self.runner
        if runner is None:
            return []
        runner.tick(now)
        replies = [{'id': self._routine_id, 'op': 'progress', 'ok': True, 'result': text}
                   for text in runner.notes]
        runner.notes = []
        if runner.done:
            self.runner = None
            end = {'id': self._routine_id, 'op': 'routine', 'ok': not runner.error}
            end.update({'error': runner.error} if runner.error else {'result': runner.result})
            replies.append(end)
        return replies

    def _servos(self, speeds: dict, plain: bool = False) -> list:
        servos = []
        for name, speed in speeds.items():
            servo = self.data['servos'].get(name)
            if servo is None:
                raise ValueError(f'άγνωστο servo {name!r}')
            if servo.get('channel') is None:
                raise ValueError(f'το {name} δεν έχει κανάλι (βήμα 1: κανάλια servo)')
            edges = None if plain else servo.get('deadband_us')
            servos.append((servo['channel'], servo['neutral_us'], edges, float(speed)))
        return servos
