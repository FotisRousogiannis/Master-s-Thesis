"""Tests against a simulated I2C bus (no hardware needed)."""

import json
import sys
from pathlib import Path

import pytest

import wrist_bench as wb


class FakeBus:
    """Records writes; answers AS5600 reads per selected mux channel."""

    def __init__(self, devices_root=(0x40,), devices_per_channel=None, raw_angles=None):
        self.devices_root = set(devices_root)
        self.devices_per_channel = devices_per_channel or {1: {0x36}, 6: {0x36}, 7: {0x36}}
        self.raw_angles = raw_angles or {1: 0, 6: 1024, 7: 2048}
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
    assert found["mux1"] == found["mux6"] == found["mux7"] == [0x36]
    assert "mux2" not in found


def test_encoder_angle_and_status(config):
    readings = wb.Bench(config, FakeBus()).read_encoders()
    assert readings["enc_ch1"][0] == pytest.approx(0.0)
    assert readings["enc_ch6"][0] == pytest.approx(90.0)
    assert readings["enc_ch7"][0] == pytest.approx(180.0)
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
    limit = config["safety"]["max_offset_us"]
    assert bench.clamp_offset(5000) == limit
    assert bench.clamp_offset(-5000) == -limit
    assert bench.clamp_offset(40) == 40


def test_duration_is_clamped(config):
    assert wb.Bench(config, FakeBus()).clamp_duration(10) == 3.0


def test_differential_mixing(config):
    config["differential"]["b_sign"] = 1
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
    angles = iter([0, 0, 0] + [400] * 1000)  # enc_ch1 jumps ~35 deg after start

    def read_block(addr, reg, n):
        raw = next(angles) if bus.channel == 1 else bus.raw_angles[bus.channel]
        return [raw >> 8, raw & 0xFF]

    bus.read_i2c_block_data = read_block
    bench.run_motion({"diff_a": 20}, duration=2.0, rate=1000)
    blocks = [w for w in bus.writes if w[0] == "block"]
    assert len(blocks) == 2  # one pulse, then all-off: aborted right after the first sample


def test_wrap_delta():
    assert wb.wrap_delta(10, 350) == pytest.approx(20)
    assert wb.wrap_delta(350, 10) == pytest.approx(-20)


def test_parse_channels():
    assert wb.parse_channels("0-3,8") == [0, 1, 2, 3, 8]
    with pytest.raises(ValueError):
        wb.parse_channels("14-16")


def test_probe_pulses_each_channel_then_turns_off(config, monkeypatch):
    monkeypatch.setattr(wb.time, "sleep", lambda s: None)
    bus = FakeBus()
    bench = wb.Bench(config, bus)
    results = bench.probe([3, 5], offset_us=30, duration=0.01)
    assert set(results) == {3, 5}
    blocks = [w for w in bus.writes if w[0] == "block"]
    pulse = bench.pca.us_to_ticks(1630)
    assert blocks == [
        ("block", 0x40, wb.PCA_LED0 + 12, [0, 0, pulse & 0xFF, pulse >> 8]),
        ("block", 0x40, wb.PCA_ALL_OFF, [0, 0, 0, 0x10]),
        ("block", 0x40, wb.PCA_LED0 + 20, [0, 0, pulse & 0xFF, pulse >> 8]),
        ("block", 0x40, wb.PCA_ALL_OFF, [0, 0, 0, 0x10]),
    ]


def test_probe_offset_is_clamped(config, monkeypatch):
    monkeypatch.setattr(wb.time, "sleep", lambda s: None)
    bus = FakeBus()
    bench = wb.Bench(config, bus)
    bench.probe([0], offset_us=999, duration=0.01)
    pulse = bench.pca.us_to_ticks(1600 + config["safety"]["max_offset_us"])
    assert ("block", 0x40, wb.PCA_LED0, [0, 0, pulse & 0xFF, pulse >> 8]) in bus.writes


def test_jog_runs_only_while_key_keeps_arriving():
    c = wb.JogController(max_offset_us=150, start_offset_us=30)
    assert c.command(0.0) == 0
    c.on_key("w", 0.0)
    assert c.command(0.3) == 30          # within the auto-repeat delay
    assert c.command(0.7) == 0           # released: no repeat arrived


def test_jog_repeat_stream_uses_short_hold():
    c = wb.JogController(max_offset_us=150)
    for t in (0.0, 0.5, 0.53, 0.56):     # first press, then auto-repeat
        c.on_key("w", t)
    assert c.command(0.65) == 30
    assert c.command(0.75) == 0          # 0.19 s without a repeat -> stop


def test_jog_direction_and_space_stop():
    c = wb.JogController(max_offset_us=150)
    c.on_key("s", 0.0)
    assert c.command(0.1) == -30
    c.on_key(" ", 0.2)
    assert c.command(0.2) == 0


def test_jog_speed_limits():
    c = wb.JogController(max_offset_us=40, start_offset_us=30, step_us=5)
    for _ in range(10):
        c.on_key("d", 0.0)
    assert c.offset_us == 40
    for _ in range(20):
        c.on_key("a", 0.0)
    assert c.offset_us == 5


def test_jog_quit():
    c = wb.JogController(max_offset_us=150)
    c.on_key("Q", 0.0)
    assert c.quit


class NoEncoderBus(FakeBus):
    def read_i2c_block_data(self, addr, reg, n):
        raise AssertionError("encoders must not be read")

    def read_byte_data(self, addr, reg):
        raise AssertionError("encoders must not be read")


def test_jog_without_encoders_never_reads_them(config, monkeypatch):
    import io
    import types

    keys = io.StringIO("q")
    monkeypatch.setattr(wb.sys, "stdin", types.SimpleNamespace(
        fileno=lambda: 0, read=keys.read))
    monkeypatch.setattr(wb.select, "select", lambda r, w, x, t: (r if keys.tell() < 1 else [], [], []))
    fake_termios = types.SimpleNamespace(tcgetattr=lambda fd: None, tcsetattr=lambda *a: None, TCSADRAIN=1)
    monkeypatch.setitem(sys.modules, "termios", fake_termios)
    monkeypatch.setitem(sys.modules, "tty", types.SimpleNamespace(setcbreak=lambda fd: None))
    bus = NoEncoderBus()
    wb.jog(wb.Bench(config, bus), channel=9, neutral_us=1600, label="ch9", use_encoders=False)
    blocks = [w for w in bus.writes if w[0] == "block"]
    assert blocks[-1] == ("block", 0x40, wb.PCA_ALL_OFF, [0, 0, 0, 0x10])


def test_pulse_never_leaves_servo_spec(config):
    bus = FakeBus()
    pca = wb.Bench(config, bus).pca
    pca.set_pulse(0, 4000)
    hi = pca.us_to_ticks(2500)
    assert bus.writes[-1] == ("block", 0x40, wb.PCA_LED0, [0, 0, hi & 0xFF, hi >> 8])
    pca.set_pulse(0, 100)
    lo = pca.us_to_ticks(500)
    assert bus.writes[-1] == ("block", 0x40, wb.PCA_LED0, [0, 0, lo & 0xFF, lo >> 8])
