"""Ψεύτικο υλικό για tests: τρέχει χωρίς Pi και χωρίς smbus2."""

from arm_pi_driver.hardware.interfaces import Encoder


class FakeBus:
    """Ένας δίαυλος I²C στη μνήμη: θυμάται κάθε κλήση και απαντά από ένα λεξικό καταχωρητών.

    registers: {(διεύθυνση, καταχωρητής): byte}. Ό,τι γράφεται με write_byte μπαίνει στο `writes`.
    """

    def __init__(self, registers: dict | None = None):
        self.registers = dict(registers or {})
        self.writes = []
        self.calls = []

    def write_byte(self, address: int, value: int) -> None:
        """Όπως το smbus2: ένα byte χωρίς καταχωρητή (π.χ. το κανάλι του TCA9548A)."""
        self.calls.append(('write_byte', address, value))
        self.writes.append((address, value))

    def read_byte_data(self, address: int, register: int) -> int:
        """Όπως το smbus2: ένα byte από έναν καταχωρητή."""
        self.calls.append(('read_byte_data', address, register))
        return self.registers[(address, register)]

    def read_i2c_block_data(self, address: int, register: int, length: int) -> list[int]:
        """Όπως το smbus2: `length` διαδοχικά bytes, ξεκινώντας από τον `register`."""
        self.calls.append(('read_i2c_block_data', address, register, length))
        return [self.registers[(address, register + i)] for i in range(length)]


class FakeEncoder(Encoder):
    """Encoder χωρίς υλικό: επιστρέφει όποια γωνία του ορίσεις (για tests του ελέγχου)."""

    def __init__(self, angle: float = 0.0, status: str = 'OK'):
        self.angle = angle
        self.status = status

    def read(self) -> tuple[float, str]:
        """(γωνία, κατάσταση) όπως ορίστηκαν."""
        return self.angle % 360.0, self.status
