import math
from pathlib import Path

from arm_pi_driver.control import build_controllers
from arm_pi_driver.control.pulses import compensated_pulse, compensation_span
from arm_pi_driver.driver import Driver
from arm_pi_driver.hardware import AS5600, NO_RESPONSE, PCA9685, TCA9548A
from arm_pi_driver.hardware.fake import FakeBus, SimulatedArm
import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'base.yaml'
DT = 0.02


@pytest.fixture
def config():
    return yaml.safe_load(CONFIG.read_text(encoding='utf-8'))


def base(config):
    """Η ομάδα της βάσης στο YAML."""
    return config['groups']['base']


def run(sim, driver, seconds, targets=None, start=0.0):
    """Τρέχει προσομοίωση + driver για `seconds`, με εντολή σε κάθε tick (όπως ένας controller)."""
    now, report = start, None
    for _ in range(round(seconds / DT)):
        if targets is not None:
            driver.command(targets, now)
        sim.step(DT)
        report = driver.tick(now)
        now += DT
    return report, now


def test_config_is_the_bench_differential(config):
    [base] = build_controllers(config)
    assert base.joints == ('joint_1', 'joint_2') and base.channels == (0, 1)


def test_no_command_no_pulse(config):
    sim = SimulatedArm(config)
    driver = Driver(config, sim.encoders, sim.pwm)
    report, _ = run(sim, driver, 0.5)
    assert not report.moving and report.reason == 'χωρίς εντολή'
    assert set(sim.pwm.pulses.values()) == {None}


def test_reaches_targets_in_simulation(config):
    sim = SimulatedArm(config)
    driver = Driver(config, sim.encoders, sim.pwm)
    run(sim, driver, 10.0, {'joint_1': 20.0, 'joint_2': -15.0})
    report, _ = run(sim, driver, 0.1, {'joint_1': 20.0, 'joint_2': -15.0}, start=10.0)
    tolerance = config['control']['tolerance_deg'] + 0.5
    assert sim.angles['joint_1'] == pytest.approx(20.0, abs=tolerance)
    assert sim.angles['joint_2'] == pytest.approx(-15.0, abs=tolerance)
    assert report.reason == 'στον στόχο' and set(sim.pwm.pulses.values()) == {None}


def test_targets_are_clamped_to_the_limits(config):
    sim = SimulatedArm(config)
    driver = Driver(config, sim.encoders, sim.pwm)
    run(sim, driver, 0.1)
    upper = base(config)['joints']['joint_1']['upper_deg']
    assert driver.command({'joint_1': 500.0}, 0.0)['joint_1'] == upper


def test_watchdog_stops_the_motors(config):
    sim = SimulatedArm(config)
    driver = Driver(config, sim.encoders, sim.pwm, watchdog_s=0.2)
    report, now = run(sim, driver, 0.5, {'joint_1': 40.0})
    assert report.moving
    report, _ = run(sim, driver, 0.5, None, start=now)       # οι εντολές σταματούν
    assert not report.moving and 'watchdog' in report.reason
    assert set(sim.pwm.pulses.values()) == {None}


def test_missing_encoder_stops_the_motors(config):
    sim = SimulatedArm(config)
    driver = Driver(config, sim.encoders, sim.pwm)
    report, now = run(sim, driver, 0.3, {'joint_1': 40.0})
    assert report.moving
    sim.encoders['joint_2'].status = NO_RESPONSE
    sim.encoders['joint_2'].angle = math.nan
    report, _ = run(sim, driver, 0.1, {'joint_1': 40.0}, start=now)
    assert not report.moving and report.reason == f'joint_2: {NO_RESPONSE}'
    assert set(sim.pwm.pulses.values()) == {None}
    assert report.joints['joint_2'].status == NO_RESPONSE


def test_out_of_limits_stops_the_motors(config):
    sim = SimulatedArm(config)
    sim.angles['joint_1'] = base(config)['joints']['joint_1']['upper_deg'] + 10
    sim.step(0.0)
    driver = Driver(config, sim.encoders, sim.pwm)
    report, _ = run(sim, driver, 0.1, {'joint_1': 0.0})
    assert not report.moving and 'εκτός ορίων' in report.reason


def test_events_only_when_something_changes(config):
    sim = SimulatedArm(config)
    driver = Driver(config, sim.encoders, sim.pwm)
    first = driver.tick(0.0)
    second = driver.tick(DT)
    assert first.events and not second.events


def test_compensation_starts_each_motor_at_its_own_edge(config):
    a, b = base(config)['motors']['a'], base(config)['motors']['b']
    pca = config['pca']
    span = compensation_span([(a['neutral_us'], a['deadband_edges_us']),
                              (b['neutral_us'], b['deadband_edges_us'])],
                             pca['pulse_min_us'], pca['pulse_max_us'], pca['max_offset_us'])
    assert compensated_pulse(0.0, a['neutral_us'], a['deadband_edges_us'], span) == 1630
    assert compensated_pulse(0.01, a['neutral_us'], a['deadband_edges_us'], span) > 1840
    assert compensated_pulse(-0.01, b['neutral_us'], b['deadband_edges_us'], span) < 1510


def test_one_motor_brakes_while_the_other_runs(config):
    [controller] = build_controllers(config)
    # joint_1 = joint_2: μοτέρ A τρέχει, μοτέρ B στο 0 → παίρνει νεκρό σημείο (φρένο), όχι None
    pulses = controller.pulses(controller.motor_speeds({'joint_1': 0.2, 'joint_2': 0.2}))
    a, b = base(config)['motors']['a'], base(config)['motors']['b']
    assert pulses[b['channel']] == b['neutral_us']
    assert pulses[a['channel']] is not None


def test_pca9685_through_the_mux_and_missing():
    bus = FakeBus()
    pca = PCA9685(bus, TCA9548A(bus), mux_channel=0)
    pca.set_pulse(1, 1600)
    ticks = round(1600 * 50 * 4096 / 1e6)
    assert bus.registers[(0x40, 0x06 + 4 + 2)] == ticks & 0xFF
    assert bus.writes[0] == (0x70, 0b0000_0001)
    bus.absent = {0x40}
    with pytest.raises(OSError):
        pca.off(1)
    assert not pca.ready


def test_real_hardware_missing_is_reported(config):
    bus = FakeBus()
    bus.absent = {0x70}
    mux = TCA9548A(bus)
    joints = base(config)['joints']
    encoders = {n: AS5600(bus, mux, j['mux_channel']) for n, j in joints.items()}
    pwm = PCA9685(bus, mux, 0)
    driver = Driver(config, encoders, pwm)
    report = driver.tick(0.0)
    assert {j.status for j in report.joints.values()} == {NO_RESPONSE}
    assert report.reason == 'PCA9685 δεν απαντά'
    driver.stop()   # δεν σκάει χωρίς υλικό


def test_after_a_safety_stop_old_targets_are_forgotten(config):
    sim = SimulatedArm(config)
    driver = Driver(config, sim.encoders, sim.pwm, watchdog_s=0.2)
    report, now = run(sim, driver, 3.0, {'joint_1': 10.0, 'joint_2': 0.0})
    report, now = run(sim, driver, 0.5, None, start=now)        # διακοπή: watchdog
    assert 'watchdog' in report.reason
    sim.angles['joint_2'] = 30.0                                  # μετακινήθηκε με το χέρι,
    report, now = run(sim, driver, 0.2, None, start=now)          # ενώ ο driver διαβάζει
    report, _ = run(sim, driver, 3.0, {'joint_1': 20.0}, start=now)   # εντολή ΜΟΝΟ joint_1
    tolerance = config['control']['tolerance_deg'] + 0.5
    assert sim.angles['joint_2'] == pytest.approx(30.0, abs=tolerance)
    assert sim.angles['joint_1'] == pytest.approx(20.0, abs=tolerance)


def test_targets_are_kept_while_holding_at_the_target(config):
    sim = SimulatedArm(config)
    driver = Driver(config, sim.encoders, sim.pwm)
    report, now = run(sim, driver, 5.0, {'joint_1': 10.0, 'joint_2': 5.0})
    assert report.reason == 'στον στόχο'
    report, _ = run(sim, driver, 0.5, {'joint_1': 10.0}, start=now)  # μόνο joint_1
    assert driver.targets['joint_2'] == pytest.approx(5.0)
