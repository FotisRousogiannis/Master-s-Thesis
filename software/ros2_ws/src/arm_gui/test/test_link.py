import socket
import struct

from arm_gui import link


def test_local_address_and_open_port():
    assert link.has_local_address("127.0.0.1")
    assert not link.has_local_address("192.0.2.123")  # TEST-NET, never local
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen()
    port = server.getsockname()[1]
    assert link.port_open("127.0.0.1", port)
    server.close()
    assert not link.port_open("127.0.0.1", port, timeout=0.2)


def _answer(name, ip, compressed):
    header = struct.pack(">HHHHHH", 0, 0x8400, 1, 1, 0, 0)
    question = link._encode_name(name) + struct.pack(">HH", 1, 1)
    owner = struct.pack(">H", 0xC00C) if compressed else link._encode_name(name)  # pointer to the question name
    return header + question + owner + struct.pack(">HHIH", 1, 0x8001, 120, 4) + socket.inet_aton(ip)


def test_mdns_answer_is_parsed_with_and_without_name_compression():
    for compressed in (True, False):
        assert link.parse_a_records(_answer("armpi.local", "192.168.1.20", compressed)) == [
            ("armpi.local", "192.168.1.20")]
    assert link.build_query("armpi.local")[12:] == b"\x05armpi\x05local\x00\x00\x01\x00\x01"


def test_mdns_without_an_answer_gives_none_quickly():
    assert link.mdns_resolve("no-such-host-for-tests.local", timeout=0.2) is None


def test_ssh_check_without_key_says_what_to_do(tmp_path):
    ok, msg = link.check_ssh("10.42.0.2", "fotis", tmp_path)
    assert not ok and "κλειδί" in msg


def test_measure_link_and_discovery_against_a_local_echo_server(monkeypatch):
    import threading
    from arm_link import discovery, server
    from arm_link.transport import UdpTransport

    echo = UdpTransport(local_port=0)
    port = echo.sock.getsockname()[1]
    threading.Thread(target=server.EchoServer(echo).serve, daemon=True).start()
    monkeypatch.setattr(server, "DEFAULT_PORT", port)

    stats, why = link.measure_link("127.0.0.1", count=5, timeout_s=0.5)
    assert why == "" and stats.sent_count == 5 and stats.loss_percent == 0.0

    monkeypatch.setattr(discovery, "discover_all", lambda timeout_s: {"10.42.0.116": "enp8s0"})
    assert link.discover_pi() == ("10.42.0.116", "εύρεση UDP (enp8s0)")
    monkeypatch.setattr(discovery, "discover_all", lambda timeout_s: {})
    assert link.discover_pi() == (None, "")


def test_without_arm_link_the_app_says_so(monkeypatch):
    import builtins
    real_import = builtins.__import__

    def no_arm_link(name, *args, **kwargs):
        if name.startswith("arm_link"):
            raise ImportError(name)
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", no_arm_link)
    assert link.discover_pi() == (None, link.NO_ARM_LINK)
    assert link.measure_link("10.42.0.116") == (None, link.NO_ARM_LINK)

