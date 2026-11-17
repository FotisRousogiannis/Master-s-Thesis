"""Τα συμβόλαια του υλικού: ό,τι χρησιμοποιούν το node και ο έλεγχος, ανεξάρτητα από το chip."""

from abc import ABC, abstractmethod


class Encoder(ABC):
    """Οτιδήποτε μετρά τη γωνία μιας άρθρωσης (AS5600 σήμερα, άλλο chip ή ψεύτικο αύριο)."""

    @abstractmethod
    def read(self) -> tuple[float, str]:
        """(γωνία 0–360°, κατάσταση: 'OK' ή περιγραφή του προβλήματος).

        Δεν σκάει όταν το υλικό λείπει: επιστρέφει (nan, NO_RESPONSE). Το nan δεν μπορεί να
        μπερδευτεί με αληθινή γωνία, όπως θα μπορούσε το 0.
        """


NO_RESPONSE = 'NO_RESPONSE'     # το υλικό δεν απάντησε (δεν είναι συνδεδεμένο, χαλαρό καλώδιο)


class PwmOutput(ABC):
    """Οτιδήποτε βγάζει παλμούς servo (PCA9685 σήμερα, ESP32 ή ψεύτικο αύριο)."""

    @abstractmethod
    def set_pulse(self, channel: int, pulse_us: float) -> None:
        """Παλμός `pulse_us` μικροδευτερολέπτων στο κανάλι (σκάει με OSError αν δεν απαντά)."""

    @abstractmethod
    def off(self, channel: int) -> None:
        """Καθόλου παλμός στο κανάλι: το servo δεν οδηγείται."""

    @abstractmethod
    def all_off(self) -> None:
        """Καθόλου παλμός σε κανένα κανάλι."""
