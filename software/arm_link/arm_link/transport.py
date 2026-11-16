"""Το «ταχυδρομείο» του arm_link: μεταφέρει bytes, χωρίς να ξέρει τι σημαίνουν."""

from abc import ABC, abstractmethod


class Transport(ABC):
    """Συμβόλαιο για κάθε τρόπο σύνδεσης με το Pi (UDP, serial, ψεύτικο για tests)."""

    @abstractmethod
    def send(self, data: bytes) -> None:
        """Στέλνει τα bytes στην άλλη πλευρά."""

    @abstractmethod
    def receive(self, timeout_s: float) -> bytes | None:
        """Περιμένει έως timeout_s δευτερόλεπτα· επιστρέφει τα bytes, ή None αν δεν ήρθε τίποτα."""

    @abstractmethod
    def close(self) -> None:
        """Κλείνει τη σύνδεση και ελευθερώνει ό,τι κρατά (π.χ. τη θύρα)."""

class FakeTransport(Transport):
    def __init__(self):
        self.inbox : list = []
        self.closed = False
    def send(self, data: bytes) -> None:
        self.inbox.append(data)
    def receive(self, timeout_s: float) -> bytes | None:
        if len(self.inbox) == 0:
            return None
        else :
            return self.inbox.pop(0)
    def close(self) -> None:
        self.closed = True