"""ADS1115: μετατροπέας αναλογικού → ψηφιακού 16 bit, 4 είσοδοι (0x48 με ADDR στη GND).

Καταχωρητές 16 bit (big-endian): 0x00 μετατροπή, 0x01 ρύθμιση. Κάθε ανάγνωση είναι «single-shot»:
γράφεις τη ρύθμιση με το bit OS (ξεκίνα), περιμένεις να γίνει OS = 1 (τελείωσε), διαβάζεις.
Ρύθμιση από docs/hardware/wiring.md: single-ended (είσοδος ως προς GND), PGA ±4.096 V.
"""

import time

from arm_pi_driver.hardware.tca9548a import TCA9548A

CONVERSION, CONFIG = 0x00, 0x01
OS = 0x8000                     # γράψιμο: ξεκίνα μετατροπή · ανάγνωση: 1 = έτοιμη
SINGLE_SHOT = 0x0100
COMPARATOR_OFF = 0x0003
PGA = {6.144: 0b000, 4.096: 0b001, 2.048: 0b010, 1.024: 0b011, 0.512: 0b100, 0.256: 0b101}
DATA_RATE = {8: 0b000, 16: 0b001, 32: 0b010, 64: 0b011, 128: 0b100, 250: 0b101, 475: 0b110,
             860: 0b111}


class ADS1115:
    """Ένα ADS1115 πίσω από ένα κανάλι του mux· `read_volts(input)` για τις εισόδους 0–3."""

    def __init__(self, bus, mux: TCA9548A, mux_channel: int, address: int = 0x48,
                 full_scale_v: float = 4.096, rate_sps: int = 860):
        if full_scale_v not in PGA or rate_sps not in DATA_RATE:
            raise ValueError(f'ADS1115: PGA {full_scale_v} V / {rate_sps} SPS δεν υπάρχει')
        self.bus, self.mux, self.mux_channel, self.address = bus, mux, mux_channel, address
        self.full_scale_v, self.rate_sps = full_scale_v, rate_sps

    def config_word(self, analog_input: int) -> int:
        """Η λέξη ρύθμισης για single-shot στην είσοδο `analog_input` ως προς GND."""
        if not 0 <= analog_input <= 3:
            raise ValueError(f'είσοδος {analog_input}: το ADS1115 έχει A0–A3')
        return (OS | (0b100 + analog_input) << 12 | PGA[self.full_scale_v] << 9 | SINGLE_SHOT
                | DATA_RATE[self.rate_sps] << 5 | COMPARATOR_OFF)

    def read_volts(self, analog_input: int) -> float:
        """Τάση της εισόδου (V)· σκάει με OSError αν δεν απαντά ή αν δεν τελειώσει η μετατροπή."""
        self.mux.select(self.mux_channel)
        word = self.config_word(analog_input)
        self.bus.write_i2c_block_data(self.address, CONFIG, [word >> 8, word & 0xFF])
        time.sleep(1.0 / self.rate_sps)                  # μία μετατροπή (860 SPS: ~1.2 ms)
        for _ in range(5):
            hi, _lo = self.bus.read_i2c_block_data(self.address, CONFIG, 2)
            if hi & 0x80:                                # OS = 1: έτοιμη
                break
            time.sleep(0.2 / self.rate_sps)
        else:
            raise OSError('ADS1115: η μετατροπή δεν τελείωσε')
        hi, lo = self.bus.read_i2c_block_data(self.address, CONVERSION, 2)
        raw = (hi << 8) | lo
        if raw & 0x8000:                                 # αρνητικός (συμπλήρωμα ως προς 2)
            raw -= 1 << 16
        return raw * self.full_scale_v / 32768
