"""Connection to the arm's Raspberry Pi (ADR-0021): find it, check SSH, install the key.

No Qt and no ROS (unit-tested). Arm Studio runs in Docker with the host network, so these see the
computer's network as it is; changing it (NetworkManager, netplan) stays a one-time step outside the app.
"""

import os
import socket
import struct
import subprocess
from pathlib import Path

PI_DIRECT = "10.42.0.2"   # the Pi on the direct cable
PC_DIRECT = "10.42.0.1"   # this computer on the direct cable (software/launcher/pi_link.sh setup-direct)
PI_NAME = "armpi"         # mDNS name: armpi.local (software/pi/setup_network.sh base)
MDNS = ("224.0.0.251", 5353)


def has_local_address(ip: str) -> bool:
    """Is this address on one of this computer's interfaces? (direct cable configured and plugged in)"""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.bind((ip, 0))
            return True
        except OSError:
            return False


def port_open(ip: str, port: int = 22, timeout: float = 1.0) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


# ---------------------------------------------------------------------- mDNS (how 3D printers are found)
def _encode_name(name: str) -> bytes:
    return b"".join(bytes([len(p)]) + p.encode() for p in name.rstrip(".").split(".")) + b"\0"


def build_query(name: str) -> bytes:
    """One question: A record of name (e.g. armpi.local)."""
    return struct.pack(">HHHHHH", 0, 0, 1, 0, 0, 0) + _encode_name(name) + struct.pack(">HH", 1, 1)


def _read_name(data: bytes, pos: int) -> tuple[str, int]:
    labels, end, jumps = [], None, 0
    while True:
        length = data[pos]
        if length & 0xC0 == 0xC0:  # compression pointer
            if end is None:
                end = pos + 2
            pos = ((length & 0x3F) << 8) | data[pos + 1]
            jumps += 1
            if jumps > 20:
                raise ValueError("pointer loop")
            continue
        if length == 0:
            return ".".join(labels), (end if end is not None else pos + 1)
        labels.append(data[pos + 1:pos + 1 + length].decode(errors="replace"))
        pos += 1 + length


def parse_a_records(data: bytes) -> list[tuple[str, str]]:
    """(name, IPv4) of every A record in the answer, authority and additional sections."""
    _id, _flags, qd, an, ns, ar = struct.unpack(">HHHHHH", data[:12])
    pos = 12
    for _ in range(qd):
        _, pos = _read_name(data, pos)
        pos += 4
    records = []
    for _ in range(an + ns + ar):
        name, pos = _read_name(data, pos)
        rtype, _cls, _ttl, length = struct.unpack(">HHIH", data[pos:pos + 10])
        pos += 10
        if rtype == 1 and length == 4:
            records.append((name.lower(), socket.inet_ntoa(data[pos:pos + 4])))
        pos += length
    return records


def mdns_resolve(name: str, timeout: float = 1.5) -> str | None:
    """IPv4 of <name> (e.g. armpi.local) from mDNS, or None."""
    name = name.lower().rstrip(".")
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
            s.settimeout(timeout)
            s.sendto(build_query(name), MDNS)
            while True:
                data, _ = s.recvfrom(4096)
                for rname, ip in parse_a_records(data):
                    if rname == name:
                        return ip
    except (OSError, ValueError, struct.error):
        return None


def find_pi(name: str = PI_NAME) -> tuple[str | None, str]:
    """(address, how): the direct cable first, then <name>.local on any network."""
    if port_open(PI_DIRECT):
        return PI_DIRECT, "καλώδιο"
    ip = mdns_resolve(f"{name}.local")
    if ip and port_open(ip):
        return ip, f"δίκτυο ({name}.local)"
    return None, ""


# ---------------------------------------------------------------------- SSH
def ssh_options(ssh_dir: Path) -> list[str]:
    return [
        "-o", "BatchMode=yes",  # never wait for a password
        "-o", "ConnectTimeout=5",
        "-o", "StrictHostKeyChecking=accept-new",
        "-o", f"UserKnownHostsFile={ssh_dir / 'known_hosts'}",
        "-i", str(ssh_dir / "id_ed25519"),
    ]


def check_ssh(host: str, user: str, ssh_dir: Path) -> tuple[bool, str]:
    if not (ssh_dir / "id_ed25519").exists():
        return False, "Δεν υπάρχει κλειδί: «Ρύθμιση κλειδιού…»"
    try:
        r = subprocess.run(["ssh", *ssh_options(ssh_dir), f"{user}@{host}", "hostname"],
                           capture_output=True, text=True, timeout=15, stdin=subprocess.DEVNULL)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    if r.returncode == 0:
        return True, f"SSH χωρίς κωδικό προς {r.stdout.strip()}"
    return False, (r.stderr.strip().splitlines() or ["αποτυχία"])[-1]


def install_key(host: str, user: str, password: str, ssh_dir: Path) -> tuple[bool, str]:
    """Create the key (once) and copy it to the Pi with the user's password, without a terminal."""
    ssh_dir.mkdir(parents=True, exist_ok=True)
    key = ssh_dir / "id_ed25519"
    try:
        if not key.exists():
            subprocess.run(["ssh-keygen", "-t", "ed25519", "-N", "", "-C", "arm-studio", "-f", str(key)],
                           check=True, capture_output=True, timeout=30)
        askpass = ssh_dir / "askpass.sh"  # gives ssh the password from the environment, never from a file
        askpass.write_text('#!/bin/sh\nprintf "%s\\n" "$ARM_PI_PASSWORD"\n')
        askpass.chmod(0o700)
        env = dict(os.environ, SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE="force",
                   ARM_PI_PASSWORD=password, DISPLAY=os.environ.get("DISPLAY", ":0"))
        r = subprocess.run(
            ["ssh-copy-id", "-i", f"{key}.pub", "-o", "StrictHostKeyChecking=accept-new",
             "-o", f"UserKnownHostsFile={ssh_dir / 'known_hosts'}", "-o", "ConnectTimeout=5", f"{user}@{host}"],
            env=env, capture_output=True, text=True, timeout=60, stdin=subprocess.DEVNULL,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return False, str(exc)
    if r.returncode != 0:
        return False, (r.stderr.strip().splitlines() or ["αποτυχία"])[-1]
    return check_ssh(host, user, ssh_dir)
