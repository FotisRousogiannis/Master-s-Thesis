"""Ψεύτικο υλικό για tests: τρέχει χωρίς Pi και χωρίς smbus2."""

from arm_pi_driver.hardware.interfaces import Encoder, PwmOutput


class FakeBus:
    """Ένας δίαυλος I²C στη μνήμη: θυμάται κάθε κλήση και απαντά από ένα λεξικό καταχωρητών.

    registers: {(διεύθυνση, καταχωρητής): byte}. Ό,τι γράφεται με write_byte μπαίνει στο `writes`.
    Διευθύνσεις στο `absent` συμπεριφέρονται σαν αποσυνδεδεμένες (OSError, όπως στο Pi).
    """

    def __init__(self, registers: dict | None = None):
        self.registers = dict(registers or {})
        self.writes = []
        self.calls = []
        self.absent = set()     # διευθύνσεις που «δεν είναι συνδεδεμένες»: δεν δίνουν ACK

    def _ack(self, address: int) -> None:
        if address in self.absent:
            raise OSError(5, 'Input/output error')     # ό,τι κάνει το smbus2 χωρίς ACK

    def write_byte(self, address: int, value: int) -> None:
        """Όπως το smbus2: ένα byte χωρίς καταχωρητή (π.χ. το κανάλι του TCA9548A)."""
        self.calls.append(('write_byte', address, value))
        self._ack(address)
        self.writes.append((address, value))

    def read_byte_data(self, address: int, register: int) -> int:
        """Όπως το smbus2: ένα byte από έναν καταχωρητή."""
        self.calls.append(('read_byte_data', address, register))
        self._ack(address)
        return self.registers[(address, register)]

    def read_i2c_block_data(self, address: int, register: int, length: int) -> list[int]:
        """Όπως το smbus2: `length` διαδοχικά bytes, ξεκινώντας από τον `register`."""
        self.calls.append(('read_i2c_block_data', address, register, length))
        self._ack(address)
        return [self.registers[(address, register + i)] for i in range(length)]

    def write_byte_data(self, address: int, register: int, value: int) -> None:
        """Όπως το smbus2: ένα byte σε έναν καταχωρητή."""
        self.calls.append(('write_byte_data', address, register, value))
        self._ack(address)
        self.registers[(address, register)] = value

    def write_i2c_block_data(self, address: int, register: int, data: list[int]) -> None:
        """Όπως το smbus2: διαδοχικά bytes, ξεκινώντας από τον `register`."""
        self.calls.append(('write_i2c_block_data', address, register, list(data)))
        self._ack(address)
        for i, value in enumerate(data):
            self.registers[(address, register + i)] = value


class FakeEncoder(Encoder):
    """Encoder χωρίς υλικό: επιστρέφει όποια γωνία του ορίσεις (για tests του ελέγχου)."""

    def __init__(self, angle: float = 0.0, status: str = 'OK'):
        self.angle = angle
        self.status = status

    def read(self) -> tuple[float, str]:
        """(γωνία, κατάσταση) όπως ορίστηκαν."""
        return self.angle % 360.0, self.status


class FakePwm(PwmOutput):
    """Έξοδος servo χωρίς υλικό: θυμάται τον παλμό κάθε καναλιού (None = καθόλου παλμός)."""

    def __init__(self):
        self.pulses = {}

    def set_pulse(self, channel: int, pulse_us: float) -> None:
        """Κρατά τον παλμό."""
        self.pulses[channel] = pulse_us

    def off(self, channel: int) -> None:
        """Κανένας παλμός στο κανάλι."""
        self.pulses[channel] = None

    def all_off(self) -> None:
        """Κανένας παλμός πουθενά."""
        self.pulses = {channel: None for channel in self.pulses}


class SimulatedArm:
    """Απλή προσομοίωση του βραχίονα: οι παλμοί του FakePwm κινούν τους FakeEncoder.

    Κάθε μοτέρ: μέσα στη νεκρή του ζώνη στέκεται· έξω από αυτήν, ταχύτητα ανάλογη της απόστασης
    από την άκρη της. Κάθε άρθρωση κινείται με την «αντίστροφη μίξη» των μοτέρ της ομάδας της
    (διαφορικό: (a ± b)/2, κατοπτρικά: (a + b)/2, ένα servo: a). Χωρίς βαρύτητα και τζόγο:
    αρκεί για να δοκιμαστεί η ροή, όχι για να ρυθμιστεί ο έλεγχος.
    """

    def __init__(self, config: dict, deg_per_s: float = 40.0, span_us: float = 400.0):
        self.groups = config['groups']
        self.deg_per_s, self.span_us = deg_per_s, span_us
        self.joints = {name: joint for group in self.groups.values()
                       for name, joint in group['joints'].items()}
        self.pwm = FakePwm()
        self.angles = {name: 0.0 for name in self.joints}
        self.encoders = {name: FakeEncoder() for name in self.joints}
        self._update_encoders()

    def motor_speed(self, motor: dict) -> float:
        """−1..1 από τον παλμό του μοτέρ (0 μέσα στη νεκρή ζώνη ή χωρίς παλμό)."""
        pulse = self.pwm.pulses.get(motor['channel'])
        down, up = motor['deadband_edges_us']
        if pulse is None or down <= pulse <= up:
            return 0.0
        speed = (pulse - up) if pulse > up else (pulse - down)
        speed = max(-1.0, min(1.0, speed / self.span_us))
        return -speed if motor.get('mirrored') else speed

    def step(self, dt: float) -> None:
        """Προχωρά την προσομοίωση κατά dt δευτερόλεπτα."""
        for group in self.groups.values():
            speeds = {name: self.motor_speed(m) for name, m in group['motors'].items()}
            for joint in group['joints']:
                weights = {name: m['mix'].get(joint, 0) for name, m in group['motors'].items()}
                norm = sum(k * k for k in weights.values())
                rate = sum(k * speeds[name] for name, k in weights.items()) / norm
                self.angles[joint] += rate * self.deg_per_s * dt
        self._update_encoders()

    def _update_encoders(self) -> None:
        for name, joint in self.joints.items():
            raw = joint['zero_deg'] + joint['sign'] * self.angles[name]
            self.encoders[name].angle = raw % 360.0
