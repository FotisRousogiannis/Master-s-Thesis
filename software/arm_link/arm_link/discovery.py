"""Εύρεση του Pi στο δίκτυο, χωρίς να ξέρουμε την IP του.

Το PC στέλνει το MAGIC σε όλο το δίκτυο (UDP broadcast). Κάθε Pi που τρέχει EchoServer το επιστρέφει,
και το recvfrom μάς δίνει τη διεύθυνσή του. Στο Pi δεν χρειάζεται καμία αλλαγή.

discover(...)     ψάχνει σε συγκεκριμένες διευθύνσεις broadcast (π.χ. "10.42.0.255")
discover_all()    βρίσκει μόνο του τα δίκτυα του PC (Ethernet, WiFi) και ψάχνει σε όλα
"""

import ipaddress
import socket
import sys
import time

from arm_link.server import DEFAULT_PORT
from arm_link.transport import UdpTransport

MAGIC = b"ARM_DISCOVER"

# Κάρτες όπου δεν θα υπάρξει ποτέ Pi: localhost και τα εσωτερικά δίκτυα του Docker.
SKIP_PREFIXES = ("lo", "docker", "br-", "veth")


def local_networks() -> list[tuple[str, ipaddress.IPv4Network]]:
    """[(όνομα κάρτας, δίκτυο)] για κάθε ενεργή κάρτα IPv4 του υπολογιστή, π.χ. ("enp8s0", 10.42.0.0/24)."""
    import psutil   # εδώ και όχι πάνω: το discover() δουλεύει και χωρίς psutil

    stats = psutil.net_if_stats()
    networks = []
    for name, addrs in psutil.net_if_addrs().items():
        if name.startswith(SKIP_PREFIXES) or not (name in stats and stats[name].isup):
            continue
        for a in addrs:
            if a.family != socket.AF_INET or not a.netmask:
                continue
            network = ipaddress.IPv4Network(f"{a.address}/{a.netmask}", strict=False)
            if network.prefixlen < 31:    # /31, /32: σημείο-προς-σημείο, χωρίς broadcast
                networks.append((name, network))
    return networks


def discover(broadcast_addrs: str | list[str], port: int = DEFAULT_PORT,
             timeout_s: float = 1.0) -> list[str]:
    """IP όσων απάντησαν μέσα σε timeout_s, με τη σειρά που απάντησαν (χωρίς διπλά).

    broadcast_addrs: μία διεύθυνση ή λίστα, π.χ. "10.42.0.255" (όλοι οι 10.42.0.x).
    Ένα socket στέλνει σε όλες· το λειτουργικό βγάζει την καθεμία από τη σωστή κάρτα.
    """
    if isinstance(broadcast_addrs, str):
        broadcast_addrs = [broadcast_addrs]
    transport = UdpTransport(broadcast=True)
    found = []
    try:
        for addr in broadcast_addrs:
            transport.peer = (addr, port)
            try:
                transport.send(MAGIC)
            except OSError:              # π.χ. κάρτα που μόλις έπεσε: συνεχίζουμε με τις άλλες
                continue
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


def discover_all(port: int = DEFAULT_PORT, timeout_s: float = 1.0) -> dict[str, str]:
    """{IP του Pi: όνομα κάρτας από όπου βρέθηκε} σε όλα τα δίκτυα του υπολογιστή.

    Το broadcast κάθε δικτύου υπολογίζεται από IP + μάσκα (10.42.0.1/24 -> 10.42.0.255).
    """
    networks = local_networks()
    found = discover([str(net.broadcast_address) for _, net in networks], port, timeout_s)
    result = {}
    for ip in found:
        result[ip] = "?"
        for name, net in networks:
            if ipaddress.IPv4Address(ip) in net:   # σε ποιο δίκτυο ανήκει η IP που απάντησε
                result[ip] = name
                break
    return result


if __name__ == "__main__":
    # python3 -m arm_link.discovery                 -> όλα τα δίκτυα του PC
    # python3 -m arm_link.discovery 10.42.0.255     -> μόνο αυτό
    if len(sys.argv) > 1:
        print(f"Αναζήτηση στο {sys.argv[1]}:{DEFAULT_PORT} ...")
        found = discover(sys.argv[1])
        print("Βρέθηκαν:", ", ".join(found) if found else "κανένα")
    else:
        networks = local_networks()
        print("Αναζήτηση σε:", ", ".join(f"{net.broadcast_address} ({name})" for name, net in networks)
              or "κανένα δίκτυο")
        found = discover_all()
        if found:
            for ip, name in found.items():
                print(f"Βρέθηκε: {ip}  (από {name})")
        else:
            print("Βρέθηκαν: κανένα")
