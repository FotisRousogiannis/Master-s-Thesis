"""ZD10: αισθητήρας πίεσης λεπτού φιλμ (η αντίσταση μικραίνει με τη δύναμη), σε διαιρέτη τάσης.

    3.3 V ──[ ZD10 ]──┬──► είσοδος ADS1115        V = 3.3 · R / (R + R_ZD10)
                      R = 6.8 kΩ (+100 nF)          αυξάνεται με την πίεση
    GND ──────────────┘

Μετρήσεις (docs/hardware/wiring.md): χωρίς πίεση R_ZD10 ≈ 17 kΩ → ≈ 0.94 V·
μέγιστη πίεση ≈ 2 kΩ → ≈ 2.5 V.
Αποσυνδεδεμένος αισθητήρας: η R τραβά την είσοδο στα ~0 V, κάτω από κάθε κανονική ένδειξη.
"""

import math

from arm_pi_driver.hardware.ads1115 import ADS1115
from arm_pi_driver.hardware.interfaces import ForceSensor, NO_RESPONSE

NO_SENSOR = 'NO_SENSOR'         # ~0 V: ο ZD10 δεν είναι συνδεδεμένος (ή κομμένο καλώδιο)


class ZD10(ForceSensor):
    """Ένας ZD10 σε μία είσοδο του ADS1115· η τιμή είναι η τάση στην είσοδο (V)."""

    def __init__(self, adc: ADS1115, analog_input: int, supply_v: float = 3.3,
                 divider_ohm: float = 6800, min_valid_v: float = 0.2):
        self.adc, self.analog_input = adc, analog_input
        self.supply_v, self.divider_ohm, self.min_valid_v = supply_v, divider_ohm, min_valid_v

    def read(self) -> tuple[float, str]:
        """(τάση V, κατάσταση)· (nan, NO_RESPONSE) αν δεν απαντά το ADS1115."""
        try:
            volts = self.adc.read_volts(self.analog_input)
        except OSError:
            return math.nan, NO_RESPONSE
        return volts, ('OK' if volts >= self.min_valid_v else NO_SENSOR)

    def resistance_ohm(self, volts: float) -> float:
        """Η αντίσταση του ZD10 για αυτή την τάση (για διάγνωση): R · (Vcc − V) / V."""
        if not volts > 0:
            return math.inf
        return self.divider_ohm * (self.supply_v - volts) / volts
