"""control.py logic and the closed loop of teleop.py against a simulated differential (no hardware)."""

import json
from pathlib import Path

import pytest

import control as ctl
import teleop as t
import wrist_bench as wb
from test_wrist_bench import FakeBus


@pytest.fixture(autouse=True)
def no_bench_calibration(monkeypatch, tmp_path):
    """Tests never read the bench's measured base_calibration.json."""
    monkeypatch.setattr(ctl, "DEFAULT_CALIBRATION", tmp_path / "no_calibration.json")


@pytest.fixture
def config():
    return json.loads(Path(wb.DEFAULT_CONFIG).read_text())


# -- pure logic --------------------------------------------------------------------------------
def test_pid_proportional_and_output_limit():
    pid = ctl.PID(kp=0.02, out_limit=0.5)
    assert pid.update(10.0, 0.0, 0.02) == pytest.approx(0.2)
    assert pid.update(100.0, 0.0, 0.02) == 0.5


def test_pid_integral_does_not_wind_up_while_saturated():
    pid = ctl.PID(kp=0.02, ki=0.01, out_limit=0.5)
    for _ in range(100):
        pid.update(100.0, 0.0, 0.1)       # far away: saturated the whole time
    assert pid.integral == 0.0
    pid.update(5.0, 95.0, 0.1)            # close: integrates again
    assert pid.integral == pytest.approx(0.5)


def test_pid_derivative_acts_on_measurement_not_target():
    pid = ctl.PID(kp=0.0, kd=0.1)
    pid.update(0.0, 10.0, 0.1)
    assert pid.update(50.0, 10.0, 0.1) == 0.0     # target jumped, measurement did not: no kick
    assert pid.update(40.0, 11.0, 0.1) == pytest.approx(-1.0)   # moving up 10 deg/s damps


def test_zero_makes_the_axis_read_the_reference_angle():
    for sign in (1, -1):
        zero = ctl.zero_for(raw_deg=200.0, sign=sign, angle_deg=30.0)
        cal = ctl.AxisCalibration(mux_channel=4, sign=sign, zero_deg=zero)
        assert cal.tracker().update(200.0) == pytest.approx(30.0)


def test_detect_axis():
    assert ctl.detect_axis({2: 100.0, 4: 50.0}, {2: 100.3, 4: 62.0}) == (4, 1)
    assert ctl.detect_axis({2: 1.0, 4: 50.0}, {2: 355.0, 4: 50.2}) == (2, -1)   # across 360 -> 0
    with pytest.raises(ValueError, match="δύο"):
        ctl.detect_axis({2: 0.0, 4: 0.0}, {2: 10.0, 4: 9.0})
    with pytest.raises(ValueError, match="κανένας"):
        ctl.detect_axis({2: 0.0, 4: 0.0}, {2: 0.5, 4: 0.2})


def test_calibration_file_round_trip(tmp_path):
    path = tmp_path / "cal.json"
    axes = {"pitch": ctl.AxisCalibration(4, 1, 12.5), "roll": ctl.AxisCalibration(2, -1, 300.0, -45, 45)}
    ctl.save_calibration(path, axes, dict(ctl.DEFAULT_PID, kp=0.03), {"a": 0, "b": 1, "invert_b": True})
    loaded, pid = ctl.load_calibration(path)
    assert loaded == axes and pid["kp"] == 0.03 and pid["ki"] == 0.0
    assert ctl.load_calibration(tmp_path / "missing.json") == ({}, ctl.DEFAULT_PID)


def test_pair_controller_limits_deadband_and_tolerance():
    axes = {"pitch": ctl.AxisCalibration(4, 1, 0.0), "roll": ctl.AxisCalibration(2, 1, 0.0, -45, 45)}
    pair = ctl.PairController(axes, ctl.DEFAULT_PID)
    assert pair.set_targets({"pitch": 200.0, "roll": -10.0}) == {"pitch": 90.0, "roll": -10.0}   # clamped
    cmd, errors = pair.step({"pitch": 89.5, "roll": -5.0}, 0.02)
    assert cmd["pitch"] == 0.0                         # inside the tolerance
    assert cmd["roll"] == -ctl.DEFAULT_PID["min_speed"]   # small error, but enough to move the servo
    with pytest.raises(ctl.LimitError):
        pair.step({"pitch": 0.0, "roll": 60.0}, 0.02)
    with pytest.raises(ValueError, match="calibration"):
        ctl.PairController({"pitch": axes["pitch"]}, ctl.DEFAULT_PID)


# -- simulated base --------------------------------------------------------------------------
class SimBase(FakeBus):
    """PCA9685 on mux 0; pitch encoder on mux 4 (+), roll encoder on mux 2 (mirrored, -).
    Servo: deadband 0.2 of full speed, 60 deg/s at full speed; motor B mounted mirrored."""

    DEADBAND, SPEED = 0.2, 60.0

    def __init__(self):
        super().__init__(devices_root=(), devices_per_channel={0: {0x40}, 2: {0x36}, 4: {0x36}}, raw_angles={})
        self.pitch, self.roll = 10.0, 200.0          # physical joint angles (deg)
        self.pulse = {0: None, 1: None}

    def write_i2c_block_data(self, addr, reg, data):
        super().write_i2c_block_data(addr, reg, data)
        if addr != 0x40:
            return
        if reg == wb.PCA_ALL_OFF:
            self.pulse = {0: None, 1: None}
        elif reg in (wb.PCA_LED0, wb.PCA_LED0 + 4):
            ch = (reg - wb.PCA_LED0) // 4
            ticks = data[2] | (data[3] << 8)
            self.pulse[ch] = None if data[3] & 0x10 else ticks * 1e6 / (50 * 4096)

    def _motor(self, ch):
        if self.pulse[ch] is None:
            return 0.0
        frac = (self.pulse[ch] - 1600) / 900
        return 0.0 if abs(frac) <= self.DEADBAND else self.SPEED * (frac - self.DEADBAND * (1 if frac > 0 else -1))

    def advance(self, dt):
        a, b = self._motor(0), -self._motor(1)       # B mirrored
        self.pitch += (a + b) / 2 * dt
        self.roll += (a - b) / 2 * dt

    def read_i2c_block_data(self, addr, reg, n):
        deg = {4: self.pitch, 2: -self.roll}[self.channel] % 360
        raw = int(deg * 4096 / 360)
        return [raw >> 8, raw & 0xFF]


class FakeClock:
    def __init__(self, bus):
        self.t, self.bus = 0.0, bus

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.bus.advance(dt)
        self.t += dt


class NoKeys:
    def __init__(self, stop_after=None):
        self.calls, self.stop_after = 0, stop_after

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def stop_pressed(self):
        self.calls += 1
        return self.stop_after is not None and self.calls > self.stop_after


def calibrated_teleop(config, tmp_path):
    bus = SimBase()
    clock = FakeClock(bus)
    lines = []
    tele = t.Teleop(wb.Bench(config, bus), 0, 1, True, [2, 4], out=lines.append, calibration=tmp_path / "cal.json")
    answers = iter(["", "y", "0", "", "", "0", "-45", "45"])   # identify (Enter), confirm; pitch 0°, default limits; roll 0°, ±45
    tele.calibrate(ask=lambda prompt: next(answers), sleep=clock.sleep, jog=lambda: None)
    return tele, bus, clock, lines


def test_calibration_finds_each_axis_encoder_and_direction(config, tmp_path):
    tele, bus, _clock, lines = calibrated_teleop(config, tmp_path)
    assert (tele.axes["pitch"].mux_channel, tele.axes["pitch"].sign) == (4, 1)
    assert (tele.axes["roll"].mux_channel, tele.axes["roll"].sign) == (2, -1)
    assert tele.axes["pitch"].tracker().update(bus.pitch % 360) == pytest.approx(0.0, abs=0.2)  # zero = here
    saved = json.loads((tmp_path / "cal.json").read_text())
    assert saved["roll"]["lower_deg"] == -45 and saved["motors"] == {"a": 0, "b": 1, "invert_b": True}
    assert [w for w in bus.writes if w[0] == "block"][-1][2] == wb.PCA_ALL_OFF


def test_closed_loop_reaches_both_targets_and_ends_off(config, tmp_path):
    tele, bus, clock, lines = calibrated_teleop(config, tmp_path)
    pitch0, roll0 = bus.pitch, bus.roll
    ok = tele.move_to({"pitch": 30.0, "roll": -20.0}, csv_path=str(tmp_path / "move.csv"),
                      clock=clock, sleep=clock.sleep, keys=NoKeys())
    assert ok, lines[-3:]
    assert bus.pitch - pitch0 == pytest.approx(30.0, abs=1.5)
    assert bus.roll - roll0 == pytest.approx(-20.0, abs=1.5)
    assert [w for w in bus.writes if w[0] == "block"][-1][2] == wb.PCA_ALL_OFF
    assert (tmp_path / "move.csv").read_text().startswith("t_s,pitch_target,pitch_deg,pitch_cmd")


def test_closed_loop_stops_on_key_and_without_calibration(config, tmp_path):
    tele, bus, clock, lines = calibrated_teleop(config, tmp_path)
    assert not tele.move_to({"pitch": 80.0}, clock=clock, sleep=clock.sleep, keys=NoKeys(stop_after=3))
    assert "Στάση από τον χρήστη." in "".join(lines)
    fresh = t.Teleop(wb.Bench(config, SimBase()), out=lines.append, calibration=tmp_path / "none.json")
    assert not fresh.move_to({"pitch": 10.0}, keys=NoKeys())
    assert any("calibration" in l for l in lines)


def test_identification_moves_back_and_a_no_changes_nothing(config, tmp_path):
    bus = SimBase()
    clock = FakeClock(bus)
    lines = []
    tele = t.Teleop(wb.Bench(config, bus), 0, 1, True, [2, 4], out=lines.append, calibration=tmp_path / "cal.json")
    pitch0, roll0 = bus.pitch, bus.roll
    answers = iter(["", "n"])                               # identify, then "not right"
    tele.calibrate(ask=lambda prompt: next(answers), sleep=clock.sleep, jog=lambda: None)
    assert any("pitch (W/S) → encoder mux4" in l for l in lines)
    assert any("roll (A/D) → encoder mux2" in l and "κατεβαίνει" in l for l in lines)
    assert "Ακυρώθηκε· τίποτα δεν άλλαξε." in lines and not (tmp_path / "cal.json").exists()
    assert abs(bus.pitch - pitch0) < 5 and abs(bus.roll - roll0) < 5   # went back near the start


def test_weak_magnet_asks_and_no_magnet_refuses(config, tmp_path):
    class WeakRoll(SimBase):
        def read_byte_data(self, addr, reg):
            if reg == wb.AS5600_STATUS:
                return wb.AS5600_MD | (wb.AS5600_ML if self.channel == 2 else 0)
            return 128

    lines = []
    tele = t.Teleop(wb.Bench(config, WeakRoll()), 0, 1, True, [2, 4], out=lines.append, calibration=tmp_path / "c.json")
    asked = []
    tele.calibrate(ask=lambda prompt: asked.append(prompt) or "", jog=lambda: None)   # Enter = no
    assert "TOO_WEAK" in asked[0] and len(asked) == 1 and not (tmp_path / "c.json").exists()

    class NoMagnet(SimBase):
        def read_byte_data(self, addr, reg):
            return 0 if reg == wb.AS5600_STATUS else 128

    tele = t.Teleop(wb.Bench(config, NoMagnet()), 0, 1, True, [2, 4], out=lines.append, calibration=tmp_path / "c.json")
    tele.calibrate(ask=lambda prompt: pytest.fail("must not ask"), jog=lambda: None)
    assert any("Δεν βλέπει μαγνήτη" in l for l in lines)


def test_identification_keeps_moving_a_slow_base_until_it_is_clear(config, tmp_path):
    class SlowBase(SimBase):
        SPEED = 12.0          # like the bench: ~1.4 deg in 0.4 s at speed 0.4

    bus = SlowBase()
    clock = FakeClock(bus)
    lines = []
    tele = t.Teleop(wb.Bench(config, bus), 0, 1, True, [2, 4], out=lines.append, calibration=tmp_path / "c.json")
    mapping = tele.identify([2, 4], ask=lambda prompt: "", sleep=clock.sleep)
    assert mapping == {"pitch": (4, 1), "roll": (2, -1)}
    assert any(l.startswith("  pitch (W, 1.") or l.startswith("  pitch (W, 2.") for l in lines)   # needed more than one step


def test_deadband_floor_per_axis_and_direction():
    axes = {"pitch": ctl.AxisCalibration(4, 1, 0.0, deadband_pos=0.10, deadband_neg=0.12),
            "roll": ctl.AxisCalibration(2, 1, 0.0, deadband_pos=0.35, deadband_neg=0.08)}
    pair = ctl.PairController(axes, ctl.DEFAULT_PID)
    pair.set_targets({"pitch": 0.0, "roll": 0.0})
    cmd, _ = pair.step({"pitch": -3.0, "roll": -3.0}, 0.02)      # both need +: small errors
    assert cmd == {"pitch": pytest.approx(0.13), "roll": pytest.approx(0.38)}
    cmd, _ = pair.step({"pitch": 3.0, "roll": 3.0}, 0.02)        # both need -
    assert cmd == {"pitch": pytest.approx(-0.15), "roll": pytest.approx(-0.11)}


def test_first_moving_and_old_calibration_files(tmp_path):
    assert ctl.first_moving([(0.05, 0.0), (0.07, 0.2), (0.09, 0.8)]) == 0.09
    assert ctl.first_moving([(-0.05, 0.3), (-0.07, -0.6)]) == 0.07          # must move the commanded way
    assert ctl.first_moving([(0.05, 0.1)]) is None
    old = tmp_path / "old.json"   # written before labels / deadbands existed
    old.write_text(json.dumps({"pitch": {"mux_channel": 2, "sign": 1, "zero_deg": 1.0},
                               "roll": {"mux_channel": 1, "sign": -1, "zero_deg": 2.0}}))
    axes, _pid = ctl.load_calibration(old)
    assert axes["pitch"].deadband_pos is None and axes["roll"].name("roll") == "roll"


class GravityBase(SimBase):
    """The roll axis lifts the arm: going up needs more command than going down."""

    def advance(self, dt):
        a, b = self._motor(0), -self._motor(1)
        roll_rate = (a - b) / 2
        roll_rate = max(0.0, roll_rate - 8.0) if roll_rate > 0 else roll_rate
        self.pitch += (a + b) / 2 * dt
        self.roll += roll_rate * dt


def test_deadband_measurement_is_saved_and_used(config, tmp_path):
    bus = GravityBase()
    clock = FakeClock(bus)
    lines = []
    tele = t.Teleop(wb.Bench(config, bus), 0, 1, True, [2, 4], out=lines.append, calibration=tmp_path / "cal.json")
    answers = iter(["", "y", "0", "", "", "0", "-45", "45"])
    tele.calibrate(ask=lambda prompt: next(answers), sleep=clock.sleep, jog=lambda: None)
    results = tele.measure_deadband(ask=lambda prompt: "y", sleep=clock.sleep)
    # servo deadband 0.2; motor B's larger scaling on its - side crosses it a bit earlier
    assert 0.15 <= results[("pitch", 1)] <= 0.25 and 0.15 <= results[("pitch", -1)] <= 0.25
    # roll: sign -1 in the calibration, the + command lowers in this model; lifting needs more
    up, down = results[("roll", -1)], results[("roll", 1)]
    assert up is not None and down is not None and up != down
    saved = json.loads((tmp_path / "cal.json").read_text())
    assert saved["pitch"]["deadband_pos"] == pytest.approx(results[("pitch", 1)])
    assert [w for w in bus.writes if w[0] == "block"][-1][2] == wb.PCA_ALL_OFF


def test_manual_drive_shows_calibrated_joint_angles(config, tmp_path, monkeypatch, capsys):
    from test_wrist_bench import _fake_terminal

    tele, bus, clock, lines = calibrated_teleop(config, tmp_path)
    tele.axes["pitch"].label = "yaw (στροφή, joint_1)"
    _fake_terminal(monkeypatch, "q")
    tele.drive()
    shown = capsys.readouterr().out
    assert "yaw (στροφή, joint_1)" in shown and "| ch0=off" in shown


def test_recalibration_keeps_labels_and_deadbands(config, tmp_path):
    tele, bus, clock, lines = calibrated_teleop(config, tmp_path)
    tele.axes["roll"].label, tele.axes["roll"].deadband_pos = "ανύψωση (joint_2)", 0.33
    answers = iter(["n", "0", "", "", "0", "", ""])          # no identification: same encoders
    tele.calibrate(ask=lambda prompt: next(answers), sleep=clock.sleep, jog=lambda: None)
    assert tele.axes["roll"].label == "ανύψωση (joint_2)" and tele.axes["roll"].deadband_pos == 0.33



class OffCentreMotors(SimBase):
    """Both servos stop around 1620 us, not 1600."""

    def _motor(self, ch):
        if self.pulse[ch] is None:
            return 0.0
        frac = (self.pulse[ch] - 1620) / 900
        return 0.0 if abs(frac) <= self.DEADBAND else self.SPEED * (frac - self.DEADBAND * (1 if frac > 0 else -1))


def test_motor_deadband_finds_the_true_neutral_and_uses_it(config, tmp_path):
    bus = OffCentreMotors()
    clock = FakeClock(bus)
    lines = []
    tele = t.Teleop(wb.Bench(config, bus), 0, 1, True, [2, 4], out=lines.append, calibration=tmp_path / "cal.json")
    result = tele.measure_motor_deadband(ask=lambda prompt: "y", sleep=clock.sleep, csv_path=str(tmp_path / "sweep.csv"))
    for name in ("a", "b"):
        assert abs(result[name]["neutral_us"] - 1620) <= 5          # not the assumed 1600
        assert 340 <= result[name]["width_us"] <= 440                # deadband 0.2 * 900 us each side
    assert tele.neutral == {"a": result["a"]["neutral_us"], "b": result["b"]["neutral_us"]}
    saved = json.loads((tmp_path / "cal.json").read_text())["motors"]
    assert saved["neutral_a_us"] == result["a"]["neutral_us"] and "deadband_edges_us" in saved
    assert tele.pulses(0.3, 0.3)[1] == result["b"]["neutral_us"]      # the braking motor gets its own neutral
    assert (tmp_path / "sweep.csv").read_text().startswith("motor,channel,pulse_us,mux2_moved_deg,mux4_moved_deg")
    assert [w for w in bus.writes if w[0] == "block"][-1][2] == wb.PCA_ALL_OFF
    reloaded = t.Teleop(wb.Bench(config, OffCentreMotors()), calibration=tmp_path / "cal.json")
    assert reloaded.neutral == tele.neutral                             # kept for next time



def test_compensated_pulse_starts_at_each_motors_edge():
    edges_a, edges_b = [1420, 1840], [1510, 1700]          # measured on the base, 2026-11-14
    span = t.compensation_span([(1630, edges_a), (1605, edges_b)], 500, 2500, 900)
    assert span == 660                                     # A upwards: min(2500, 1630 + 900) - 1840
    assert t.compensated_pulse(0.0, 1630, edges_a, span) == 1630          # brake at its own neutral
    assert t.compensated_pulse(0.1, 1630, edges_a, span) == pytest.approx(1840 + 66)
    assert t.compensated_pulse(-0.1, 1605, edges_b, span) == pytest.approx(1510 - 66)   # same distance
    assert t.compensated_pulse(1.0, 1630, edges_a, span) == 2500


class UnequalMotors(SimBase):
    """Motor A needs much more than motor B to move (like the base: about +/-210 vs +/-95 us)."""

    DEADBANDS = {0: 0.23, 1: 0.07}

    def _motor(self, ch):
        if self.pulse[ch] is None:
            return 0.0
        frac, db = (self.pulse[ch] - 1600) / 900, self.DEADBANDS[ch]
        return 0.0 if abs(frac) <= db else self.SPEED * (frac - db * (1 if frac > 0 else -1))


def test_compensation_removes_coupling_at_small_commands(config, tmp_path):
    def roll_drift(compensate):
        bus = UnequalMotors()
        clock = FakeClock(bus)
        tele = t.Teleop(wb.Bench(config, bus), 0, 1, True, [2, 4], out=lambda *_: None,
                        calibration=tmp_path / f"c{compensate}.json")
        tele.measure_motor_deadband(ask=lambda prompt: "y", sleep=clock.sleep)
        tele.pid["motor_compensation"] = compensate
        pitch0, roll0 = bus.pitch, bus.roll
        tele.pulse_axis("pitch", 0.1, 2.0, clock.sleep)    # small pure pitch (motors together)
        return bus.pitch - pitch0, bus.roll - roll0

    pitch_off, roll_off = roll_drift(0.0)
    pitch_on, roll_on = roll_drift(1.0)
    assert abs(roll_off) > 1.0                 # without: only motor B moves -> roll moves too
    assert abs(roll_on) < 0.15 * abs(roll_off)  # with: both motors move -> (almost) pure pitch
    assert abs(pitch_on) > 1.0
