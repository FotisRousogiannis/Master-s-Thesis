"""Tests for the arm_link classes (Ping, LinkStats, LinkProbe, UdpTransport, discovery), without the Pi."""

import threading

import pytest

from arm_link.discovery import MAGIC, discover
from arm_link.ping import U32_MAX, Ping
from arm_link.probe import LinkProbe
from arm_link.stats import LinkStats
from arm_link.transport import FakeTransport, Transport, UdpTransport


def test_ping_refuses_values_that_do_not_fit():
    with pytest.raises(ValueError):
        Ping(-1, 0)
    with pytest.raises(ValueError):
        Ping(U32_MAX + 1, 0)
    with pytest.raises(ValueError):
        Ping.from_bytes(b"short")
    assert len(Ping(U32_MAX, 2**64 - 1).to_bytes()) == Ping.SIZE == 12


def test_stats_loss_min_max_avg_jitter():
    stats = LinkStats()
    for seq in range(4):
        stats.sent(seq)
    stats.received(Ping(0, 0), 1_000_000)          # 1 ms
    stats.received(Ping(1, 0), 3_000_000)          # 3 ms
    stats.received(Ping(2, 0), 2_000_000)          # 2 ms; seq 3 lost
    assert stats.loss_percent == 25.0
    assert (stats.min_ms, stats.max_ms, stats.avg_ms) == (1.0, 3.0, 2.0)
    assert stats.jitter_ms == pytest.approx(1.5)   # |3-1| and |2-3| → 1.5 ms


def test_stats_ignore_duplicates_and_unknown_answers():
    stats = LinkStats()
    stats.sent(0)
    stats.received(Ping(0, 0), 10)
    stats.received(Ping(0, 0), 20)                 # the same answer twice
    stats.received(Ping(9, 0), 30)                 # never sent
    assert stats.rtts_ns == [10] and stats.loss_percent == 0.0


def test_empty_stats_are_zero():
    stats = LinkStats()
    assert (stats.loss_percent, stats.avg_ms, stats.min_ms, stats.max_ms, stats.jitter_ms) == (0, 0, 0, 0, 0)


def test_probe_over_an_echo_has_no_loss():
    stats = LinkProbe(FakeTransport()).run(10, 0.1)    # FakeTransport gives back what it was sent
    assert stats.sent_count == 10 and stats.loss_percent == 0.0


class Lossy(Transport):
    """Loses every second packet and answers one with garbage."""

    def __init__(self):
        self.count, self.waiting = 0, None

    def send(self, data):
        self.count += 1
        self.waiting = None if self.count % 2 == 0 else (b"garbage" if self.count == 3 else data)

    def receive(self, timeout_s):
        data, self.waiting = self.waiting, None
        return data

    def close(self):
        pass


def test_probe_counts_lost_and_garbage_answers_as_lost():
    stats = LinkProbe(Lossy()).run(6, 0.1)
    assert stats.loss_percent == pytest.approx(100 * 4 / 6)     # 2, 4, 6 lost and 3 garbage


def echo_once(server: UdpTransport, times: int = 1):
    def run():
        for _ in range(times):
            data = server.receive(2.0)
            if data is not None:
                server.send(data)
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def test_udp_echo_on_this_computer():
    server = UdpTransport(local_port=0)
    port = server.sock.getsockname()[1]
    client = UdpTransport(peer=("127.0.0.1", port))
    try:
        echo_once(server, times=3)
        stats = LinkProbe(client).run(3, 1.0)
        assert stats.loss_percent == 0.0 and stats.max_ms < 1000
    finally:
        client.close()
        server.close()


def test_udp_receive_times_out_and_send_needs_a_peer():
    t = UdpTransport()
    try:
        assert t.receive(0.05) is None
        with pytest.raises(RuntimeError):
            t.send(b"x")
    finally:
        t.close()


def test_discover_finds_an_echo_server_once():
    server = UdpTransport(local_port=0)
    port = server.sock.getsockname()[1]
    try:
        echo_once(server, times=2)
        found = discover(["127.0.0.1", "127.0.0.1"], port=port, timeout_s=0.5)   # asked twice, listed once
        assert found == ["127.0.0.1"]
    finally:
        server.close()


def test_discover_ignores_answers_that_are_not_ours():
    server = UdpTransport(local_port=0)
    port = server.sock.getsockname()[1]

    def answer_wrong():
        if server.receive(2.0) == MAGIC:
            server.send(b"something else")
    thread = threading.Thread(target=answer_wrong, daemon=True)
    thread.start()
    try:
        assert discover("127.0.0.1", port=port, timeout_s=0.3) == []
    finally:
        server.close()
