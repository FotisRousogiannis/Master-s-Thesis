"""PCA9685: γεννήτρια PWM 16 καναλιών για τα servo (0x40), πίσω από ένα κανάλι του mux."""

import time

from arm_pi_driver.hardware.interfaces import PwmOutput
from arm_pi_driver.hardware.tca9548a import TCA9548A

MODE1, MODE2, PRESCALE, LED0, ALL_LED = 0x00, 0x01, 0xFE, 0x06, 0xFA
SLEEP, AUTO_INCREMENT, RESTART, OUTDRV = 0x10, 0x20, 0x80, 0x04
FULL_OFF = 0x10     # bit 4 του byte OFF_H: έξοδος μόνιμα 0 (κανένας παλμός)


class PCA9685(PwmOutput):
    """Οι παλμοί των servo, με το όριο της προδιαγραφής τους (pulse_min_us..pulse_max_us)."""

    def __init__(self, bus, mux: TCA9548A, mux_channel: int, address: int = 0x40,
                 freq_hz: float = 50, osc_hz: float = 25_000_000,
                 pulse_min_us: float = 500, pulse_max_us: float = 2500):
        self.bus, self.mux, self.mux_channel, self.address = bus, mux, mux_channel, address
        self.freq_hz, self.osc_hz = freq_hz, osc_hz
        self.pulse_min_us, self.pulse_max_us = pulse_min_us, pulse_max_us
        self.ready = False

    def start(self) -> None:
        """Ρύθμιση συχνότητας· γίνεται μόνη της στην πρώτη εντολή (OSError αν δεν απαντά)."""
        self.mux.select(self.mux_channel)
        prescale = round(self.osc_hz / (4096 * self.freq_hz)) - 1
        # η συχνότητα αλλάζει μόνο σε κατάσταση ύπνου
        self.bus.write_byte_data(self.address, MODE1, SLEEP)
        self.bus.write_byte_data(self.address, PRESCALE, prescale)
        self.bus.write_byte_data(self.address, MODE2, OUTDRV)
        self.bus.write_byte_data(self.address, MODE1, AUTO_INCREMENT)
        time.sleep(0.001)                                           # ο ταλαντωτής ξεκινά
        self.bus.write_byte_data(self.address, MODE1, AUTO_INCREMENT | RESTART)
        self.ready = True

    def ticks(self, pulse_us: float) -> int:
        """Μικροδευτερόλεπτα σε βήματα του PCA9685 (4096 ανά περίοδο)."""
        return round(pulse_us * self.freq_hz * 4096 / 1e6)

    def _write(self, register: int, data: list[int]) -> None:
        try:
            if not self.ready:
                self.start()
            self.mux.select(self.mux_channel)
            self.bus.write_i2c_block_data(self.address, register, data)
        except OSError:
            self.ready = False      # ίσως έχασε ρεύμα: ξαναρύθμιση στην επόμενη εντολή
            raise

    def set_pulse(self, channel: int, pulse_us: float) -> None:
        """Παλμός στο κανάλι, κομμένος στο όριο του servo."""
        pulse_us = max(self.pulse_min_us, min(self.pulse_max_us, pulse_us))
        off = self.ticks(pulse_us)
        self._write(LED0 + 4 * channel, [0, 0, off & 0xFF, off >> 8])

    def off(self, channel: int) -> None:
        """Καθόλου παλμός στο κανάλι."""
        self._write(LED0 + 4 * channel, [0, 0, 0, FULL_OFF])

    def all_off(self) -> None:
        """Καθόλου παλμός σε κανένα κανάλι (μία εντολή για όλα)."""
        self._write(ALL_LED, [0, 0, 0, FULL_OFF])
