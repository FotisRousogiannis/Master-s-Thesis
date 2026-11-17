"""Τα συμβόλαια του υλικού: ό,τι χρησιμοποιούν το node και ο έλεγχος, ανεξάρτητα από το chip."""

from abc import ABC, abstractmethod


class Encoder(ABC):
    """Οτιδήποτε μετρά τη γωνία μιας άρθρωσης (AS5600 σήμερα, άλλο chip ή ψεύτικο αύριο)."""

    @abstractmethod
    def read(self) -> tuple[float, str]:
        """(γωνία 0–360°, κατάσταση: 'OK' ή περιγραφή του προβλήματος)."""
