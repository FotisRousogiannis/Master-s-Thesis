"""Tests against a simulated I2C bus (no hardware needed)."""

import json
from pathlib import Path

import pytest

import wrist_bench as wb


class FakeBus:
    """Records writes; answers AS5600 reads per selected mux channel."""

    def __init__(self, devices_root=(0x40,), devices_per_channel=None, raw_angles=None):
        self.devices_root = set(devices_root)
        self.devices_per_channel = devices_per_channel or {0: {0x36}, 1: {0x36}, 2: {0x36}}
        self.raw_angles = raw_angles or {0: 0, 1: 1024, 2: 2048}
        self.channel = None
        self.writes = []

    def write_byte(self, addr, value):
        if addr == 0x70:
            self.channel = None if value == 0 else value.bit_length() - 1
        self.writes.append(("byte", addr, value))

    def write_byte_data(self, addr, reg, value):
        self.writes.append(("data", addr, reg, value))

    def write_i2c_block_data(self, addr, reg, data):
        self.writes.append(("block", addr, reg, list(data)))

    def read_byte(self, addr):
        visible = self.devices_root | self.devices_per_channel.get(self.channel, set()) | {0x70}
        if addr not in visible:
            raise OSError("no ack")
        return 0

    def read_i2c_block_data(self, addr, reg, n):
        raw = self.raw_angles[self.channel]
        return [raw >> 8, raw & 0xFF]

    def read_byte_data(self, addr, reg):
        return wb.AS5600_MD if reg == wb.AS5600_STATUS else 128


@pytest.fixture
def config():
    return json.loads(Path(wb.DEFAULT_CONFIG).read_text())


def test_scan_finds_devices_per_mux_channel(config):
    found = wb.Bench(config, FakeBus()).scan()
    assert found["root"] == [0x40]
    assert found["mux0"] == found["mux1"] == found["mux2"] == [0x36]
    assert "mux3" not in found


def test_encoder_angle_and_status(config):
    readings = wb.Bench(config, FakeBus()).read_encoders()
    assert readings["diff_1"][0] == pytest.approx(0.0)
    assert readings["diff_2"][0] == pytest.approx(90.0)
    assert readings["upper"][0] == pytest.approx(180.0)
    assert all(r[2] == "OK" for r in readings.values())


def test_magnet_status_flags():
    assert wb.magnet_status(0) == "NO_MAGNET"
    assert wb.magnet_status(wb.AS5600_MD | wb.AS5600_ML) == "TOO_WEAK"
    assert wb.magnet_status(wb.AS5600_MD | wb.AS5600_MH) == "TOO_STRONG"


def test_pulse_ticks_at_50hz(config):
    bus = FakeBus()
    pca = wb.Bench(config, bus).pca
    assert pca.us_to_ticks(1500) == 307
    pca.set_pulse(9, 1600)
    assert bus.writes[-1] == ("block", 0x40, wb.PCA_LED0 + 36, [0, 0, 328 & 0xFF, 328 >> 8])


def test_prescale_for_50hz(config):
    bus = FakeBus()
    wb.Bench(config, bus).pca
    assert ("data", 0x40, wb.PCA_PRESCALE, 121) in bus.writes


def test_offset_is_clamped(config):
    bench = wb.Bench(config, FakeBus())
    assert bench.clamp_offset(500) == 150
    assert bench.clamp_offset(-500) == -150
    assert bench.clamp_offset(40) == 40


def test_duration_is_clamped(config):
    assert wb.Bench(config, FakeBus()).clamp_duration(10) == 3.0


def test_differential_mixing(config):
    bench = wb.Bench(config, FakeBus())
    assert bench.differential(30, 0) == {"diff_a": 30, "diff_b": 30}
    assert bench.differential(0, 30) == {"diff_a": 30, "diff_b": -30}
    config["differential"]["b_sign"] = -1
    assert bench.differential(30, 0) == {"diff_a": 30, "diff_b": -30}


def test_motion_always_ends_with_outputs_off(config):
    bus = FakeBus()
    bench = wb.Bench(config, bus)
    bench.run_motion({"diff_a": 20}, duration=0.05, rate=100)
    blocks = [w for w in bus.writes if w[0] == "block"]
    assert blocks[-1] == ("block", 0x40, wb.PCA_ALL_OFF, [0, 0, 0, 0x10])


def test_motion_aborts_on_travel_limit(config):
    bus = FakeBus()
    bench = wb.Bench(config, bus)
    angles = iter([0, 0, 0] + [400, 0, 0] * 1000)  # diff_1 jumps ~35 deg after start

    def read_block(addr, reg, n):
        raw = next(angles) if bus.channel == 0 else bus.raw_angles[bus.channel]
        return [raw >> 8, raw & 0xFF]

    bus.read_i2c_block_data = read_block
    bench.run_motion({"diff_a": 20}, duration=2.0, rate=1000)
    blocks = [w for w in bus.writes if w[0] == "block"]
    assert len(blocks) == 2  # one pulse, then all-off: aborted right after the first sample


def test_wrap_delta():
    assert wb.wrap_delta(10, 350) == pytest.approx(20)
    assert wb.wrap_delta(350, 10) == pytest.approx(-20)
