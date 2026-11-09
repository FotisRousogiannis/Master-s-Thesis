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
