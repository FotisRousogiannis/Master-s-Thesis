"""AS5600: μαγνητικός encoder 12 bit (διεύθυνση πάντα 0x36, γι' αυτό ένας ανά κανάλι του mux)."""

from arm_pi_driver.hardware.interfaces import Encoder
from arm_pi_driver.hardware.tca9548a import Mux

AS5600_ADDRESS = 0x36
RAW_ANGLE = 0x0C    # 2 bytes: 0x0C (hi) + 0x0D (lo), 12 bits γωνίας
STATUS = 0x0B       # bits του STATUS:
MD = 0x20           # υπάρχει μαγνήτης
ML = 0x10           # μαγνήτης πολύ αδύναμος (μακριά)
MH = 0x08           # μαγνήτης πολύ δυνατός (κοντά)


class AS5600(Encoder):
    """Ένας AS5600 πίσω από ένα κανάλι του mux."""

    def __init__(self, bus, mux: Mux, channel: int):
        self.bus = bus
        self.mux = mux
        self.channel = channel

    @staticmethod
    def magnet_status(status: int) -> str:
        """Το byte STATUS σε κείμενο: OK, NO_MAGNET, TOO_WEAK ή TOO_STRONG."""
        if not status & MD:
            return 'NO_MAGNET'
        if status & ML:
            return 'TOO_WEAK'
        if status & MH:
            return 'TOO_STRONG'
        return 'OK'

    def read(self) -> tuple[float, str]:
        """(γωνία 0–360°, κατάσταση μαγνήτη), από μία ανάγνωση του αισθητήρα."""
        self.mux.select(self.channel)
        # Τα δύο bytes σε ΜΙΑ συναλλαγή: αλλιώς μπορεί να είναι από δύο διαφορετικές μετρήσεις.
        hi, lo = self.bus.read_i2c_block_data(AS5600_ADDRESS, RAW_ANGLE, 2)
        raw = ((hi & 0x0F) << 8) | lo
        angle = raw * 360 / 4096
        status = self.bus.read_byte_data(AS5600_ADDRESS, STATUS)
        return angle, self.magnet_status(status)
