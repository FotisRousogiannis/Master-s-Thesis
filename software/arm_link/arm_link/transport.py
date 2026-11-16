"""Το «ταχυδρομείο» του arm_link: μεταφέρει bytes, χωρίς να ξέρει τι σημαίνουν."""

from abc import ABC, abstractmethod
import socket

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

class UdpTransport(Transport):
    """Μεταφορά με UDP. Στο PC: δίνεις peer (το Pi). Στο Pi: δίνεις local_port και απαντά
    σε όποιον του έγραψε τελευταίος."""

    def __init__(self, local_port: int = 0, peer: tuple[str, int] | None = None,
                 broadcast: bool = False):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if broadcast:
            # το Linux απαγορεύει το broadcast αν δεν το ζητήσεις ρητά
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("0.0.0.0", local_port))
        self.peer = peer

    def send(self, data: bytes) -> None:
        if self.peer is None:
            raise RuntimeError("δεν ξέρω σε ποιον να στείλω: δώσε peer ή λάβε πρώτα κάτι")
        self.sock.sendto(data, self.peer)

    def receive(self, timeout_s: float) -> bytes | None:
        self.sock.settimeout(timeout_s)
        try:
            data, addr = self.sock.recvfrom(1024)
        except TimeoutError:
            return None
        self.peer = addr
        return data

    def close(self) -> None:
        self.sock.close()
