import pytest

from arm_link.ping import Ping


def test_round_trip():
    p = Ping(5, 123)
    assert Ping.from_bytes(p.to_bytes()) == p