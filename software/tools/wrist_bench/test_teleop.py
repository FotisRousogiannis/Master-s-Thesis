"""teleop.py against the simulated I2C bus of test_wrist_bench (no hardware)."""

import json
from pathlib import Path

import pytest

import control as ctl
import teleop as t
import wrist_bench as wb
from test_wrist_bench import FakeBus, WeakMagnetBus, _fake_terminal


@pytest.fixture(autouse=True)
def no_bench_calibration(monkeypatch, tmp_path):
    """Tests never read the bench's measured base_calibration.json."""
    monkeypatch.setattr(ctl, "DEFAULT_CALIBRATION", tmp_path / "no_calibration.json")


@pytest.fixture
def config():
    return json.loads(Path(wb.DEFAULT_CONFIG).read_text())


def base_bus(cls=FakeBus):
    """Like the bench on 2026-11-13: PCA9685 on mux 0, AS5600 on mux 2 and 4."""
    return cls(devices_root=(), devices_per_channel={0: {0x40}, 2: {0x36}, 4: {0x36}}, raw_angles={2: 1024, 4: 2048})


def test_speed_mapping_is_the_old_tools_within_the_bench_limit():
    # old servo_test.speed_to_pulse with centre 1600, 500..2500: 0.4 -> 1960 / 1160
    assert t.speed_to_pulse(0.4, 1600, 500, 2500, 900) == pytest.approx(1960)
    assert t.speed_to_pulse(-0.4, 1600, 500, 2500, 900) == pytest.approx(1160)
    assert t.speed_to_pulse(-1.0, 1600, 500, 2500, 900) == 700     # old: 500, bench limit: 1600 - 900
    assert t.speed_to_pulse(2.0, 1600, 500, 2500, 900) == 2500


def test_controller_keys_and_first_press_covers_the_autorepeat_delay():
    c = t.TeleopController()
    c.on_key("w", 0.0)
    assert c.command(0.45) == (0.4, 0.0)     # old tool stopped at 0.25 s, before the auto-repeat starts
    c.on_key("a", 1.0)
    assert c.command(1.1) == (0.0, -0.4)
    c.on_key("+", 1.2)
    assert c.magnitude == 0.5
    for _ in range(10):
        c.on_key("-", 1.2)
    assert c.magnitude == 0.1
    c.on_key("q", 1.3)
    assert c.quit


def test_differential_pulses_with_mirrored_b(config):
    tele = t.Teleop(wb.Bench(config, base_bus()), a=0, b=1, invert_b=True)
    assert tele.pulses(0.4, 0.0) == {0: pytest.approx(1960), 1: pytest.approx(1160)}   # pitch: opposite pulses
    assert tele.pulses(0.0, 0.4) == {0: pytest.approx(1960), 1: pytest.approx(1960)}   # roll: same pulses
    assert tele.pulses(0.0, 0.0) == {0: None, 1: None}                                 # stop = outputs off
    # equal pitch and roll: only A runs; B gets neutral so it brakes instead of turning freely
    assert tele.pulses(0.3, 0.3) == {0: pytest.approx(1600 + 0.6 * 900), 1: 1600}


def test_scan_and_magnets(config):
    lines = []
    tele = t.Teleop(wb.Bench(config, base_bus(WeakMagnetBus)), out=lines.append)
    tele.scan()
    assert "  mux0 : 0x40 PCA9685" in lines and "  mux2 : 0x36 AS5600" in lines
    assert tele.encoder_channels_found() == [2, 4]
    result = tele.read_magnets()
    assert result[2][1] == "NO_MAGNET" and result[4][1] == "OK"


def test_choose_validates_and_keeps_defaults(config):
    lines = []
    tele = t.Teleop(wb.Bench(config, base_bus()), out=lines.append)
    answers = iter(["", "0", "", "1", "", "9", "2,4"])   # A=Enter, B=0 (same as A), redo: A, B=1, invert Enter, enc 9 bad, 2,4
    tele.choose(ask=lambda prompt: next(answers))
    assert (tele.a, tele.b, tele.invert_b, tele.encoder_channels) == (0, 1, True, [2, 4])
    assert any("διαφορετικά" in l for l in lines) and any("εκτός ορίων" in l for l in lines)


def test_drive_moves_both_motors_through_the_mux_records_and_ends_off(config, monkeypatch, tmp_path):
    _fake_terminal(monkeypatch, "wq")
    bus = base_bus(WeakMagnetBus)
    lines = []
    tele = t.Teleop(wb.Bench(config, bus), 0, 1, True, [2, 4], out=lines.append)
    tele.drive(csv_path=str(tmp_path / "run.csv"))
    pca = tele.bench.pca
    pulses = {w[2]: w[3] for w in bus.writes if w[0] == "block" and w[2] in (wb.PCA_LED0, wb.PCA_LED0 + 4)}
    a, b = pca.us_to_ticks(1960), pca.us_to_ticks(1160)
    assert pulses[wb.PCA_LED0] == [0, 0, a & 0xFF, a >> 8] and pulses[wb.PCA_LED0 + 4] == [0, 0, b & 0xFF, b >> 8]
    assert [w for w in bus.writes if w[0] == "block"][-1][2] == wb.PCA_ALL_OFF
    assert any("mux2 μαγνήτης NO_MAGNET" in l for l in lines)
    rows = (tmp_path / "run.csv").read_text().splitlines()
    assert rows[0].startswith("t_s,state,pitch,roll,ch0_us,ch1_us,mux2_deg,mux4_deg") and "PITCH+" in rows[1]


def test_menu_runs_the_chosen_actions(config):
    lines = []
    tele = t.Teleop(wb.Bench(config, base_bus()), out=lines.append)
    answers = iter(["1", "5", "q"])
    tele.menu(ask=lambda prompt: next(answers))
    assert any("Σάρωση I²C" in l for l in lines) and "Όλες οι έξοδοι off." in lines



def test_recordings_go_into_the_measurements_folder(tmp_path):
    folder = tmp_path / "docs" / "measurements" / "base"
    assert t.measurement_path("", folder) is None
    assert t.measurement_path("run5", folder) == str(folder / "run5.csv")             # bare name -> folder + .csv
    assert t.measurement_path("run6.csv", folder) == str(folder / "run6.csv")
    assert folder.is_dir()
    own = tmp_path / "elsewhere" / "x"
    assert t.measurement_path(str(own), folder) == str(own) + ".csv"                   # a path is kept
    (folder / "run5.csv").write_text("old")
    again = t.measurement_path("run5", folder, now=lambda: (2026, 11, 14, 12, 30, 5, 0, 0, 0))
    assert again == str(folder / "run5_20261114_123005.csv")                          # never overwrite
    assert t.MEASUREMENTS.parts[-3:] == ("docs", "measurements", "base")



def test_manual_drive_without_encoders(config, monkeypatch):
    from test_wrist_bench import NoEncoderBus

    _fake_terminal(monkeypatch, "wq")
    bus = NoEncoderBus(devices_root=(), devices_per_channel={0: {0x40}})
    lines = []
    tele = t.Teleop(wb.Bench(config, bus), 0, 1, True, [2, 4], out=lines.append)
    tele.drive(use_encoders=False)                     # never touches an encoder
    assert any("ΧΩΡΙΣ ΟΡΙΟ" in l for l in lines)
    assert [w for w in bus.writes if w[0] == "block"][-1][2] == wb.PCA_ALL_OFF


def test_manual_drive_offers_to_go_on_when_an_encoder_is_missing(config, monkeypatch):
    class OnlyPca(FakeBus):
        def read_i2c_block_data(self, addr, reg, n):
            raise OSError(5, "Input/output error")

    lines = []
    tele = t.Teleop(wb.Bench(config, OnlyPca(devices_root=(), devices_per_channel={0: {0x40}})), 0, 1, True, [2, 4],
                    out=lines.append)
    tele.drive(ask=lambda prompt: "")                  # Enter = no: does not move
    assert not any("ΧΩΡΙΣ ΟΡΙΟ" in l for l in lines)
    _fake_terminal(monkeypatch, "wq")
    tele.drive(ask=lambda prompt: "y")                 # yes: goes on without encoders
    assert any("ΧΩΡΙΣ ΟΡΙΟ" in l for l in lines)


def test_menu_asks_about_encoders_for_manual_drive(config, monkeypatch):
    calls = []
    tele = t.Teleop(wb.Bench(config, base_bus()), 0, 1, True, [2, 4], out=lambda *_: None)
    monkeypatch.setattr(tele, "drive", lambda **kw: calls.append(kw["use_encoders"]))
    answers = iter(["4", "n", "", "4", "", "", "q"])
    tele.menu(ask=lambda prompt: next(answers))
    assert calls == [False, True]
