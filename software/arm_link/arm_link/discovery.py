"""Εύρεση του Pi στο δίκτυο, χωρίς να ξέρουμε την IP του.

Το PC στέλνει το MAGIC σε όλο το δίκτυο (UDP broadcast). Κάθε Pi που τρέχει EchoServer το επιστρέφει,
και το recvfrom μάς δίνει τη διεύθυνσή του. Στο Pi δεν χρειάζεται καμία αλλαγή.
"""

import sys
import time

from arm_link.server import DEFAULT_PORT
from arm_link.transport import UdpTransport

MAGIC = b"ARM_DISCOVER"


def discover(broadcast_addr: str, port: int = DEFAULT_PORT, timeout_s: float = 1.0) -> list[str]:
    """IP όσων απάντησαν μέσα σε timeout_s, με τη σειρά που απάντησαν (χωρίς διπλά).

    broadcast_addr: π.χ. "10.42.0.255" για το καλώδιο (όλοι οι 10.42.0.x) ή "255.255.255.255".
    """
    transport = UdpTransport(peer=(broadcast_addr, port), broadcast=True)
    found = []
    try:
        transport.send(MAGIC)
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            data = transport.receive(remaining)
            if data == MAGIC:                     # μόνο απαντήσεις στο δικό μας μήνυμα
                ip = transport.peer[0]            # το «ΑΠΟ» του πακέτου που μόλις ήρθε
                if ip not in found:
                    found.append(ip)
    finally:
        transport.close()                         # κλείνει το socket ακόμα κι αν κάτι σκάσει
    return found


if __name__ == "__main__":
    # python3 -m arm_link.discovery [διεύθυνση broadcast]
    addr = sys.argv[1] if len(sys.argv) > 1 else "10.42.0.255"
    print(f"Αναζήτηση στο {addr}:{DEFAULT_PORT} ...")
    found = discover(addr)
    print("Βρέθηκαν:", ", ".join(found) if found else "κανένα")
