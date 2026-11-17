import copy

from arm_pi_driver import arm_file
from arm_pi_driver.arm_file import Change, GRIPPER_GROUP, JOINT, RESTART, SERVO
from arm_pi_driver.control import build_controllers
from arm_pi_driver.driver import Driver
from arm_pi_driver.hardware.fake import FakeForceSensor, SimulatedArm
import pytest

DT = 0.02
BASE = 'joint_1+joint_2'


def test_signs_become_the_mix_of_the_base(config):
    """joint_1: {servo_1: +1, servo_2: -1}, joint_2: {servo_1: +1, servo_2: +1} (ADR-0023)."""
    motors = config['groups'][BASE]['motors']
    assert motors['servo_1']['mix'] == {'joint_1': 1, 'joint_2': 1}
    assert motors['servo_2']['mix'] == {'joint_1': -1, 'joint_2': 1}
    assert not motors['servo_1']['mirrored'] and not motors['servo_2']['mirrored']


def test_same_motor_speeds_as_the_old_mirrored_mix(config):
    """Old base.yaml: b mix {joint_1: 1, joint_2: -1}, mirrored → the same speeds as the signs."""
    [base] = build_controllers(config)
    for speeds in ({'joint_1': 0.3, 'joint_2': 0.0}, {'joint_1': 0.0, 'joint_2': -0.2},
                   {'joint_1': 0.1, 'joint_2': 0.25}):
        a = speeds['joint_1'] + speeds['joint_2']
        b = -(speeds['joint_1'] - speeds['joint_2'])
        assert base.motor_speeds(speeds) == pytest.approx({'servo_1': a, 'servo_2': b})


def test_measured_values_of_the_base(config):
    joints, motors = config['groups'][BASE]['joints'], config['groups'][BASE]['motors']
    j1, j2 = joints['joint_1'], joints['joint_2']
    assert (j1['mux_channel'], j1['zero_deg'], j1['sign']) == (2, 267.891, 1)
    assert (j2['mux_channel'], j2['zero_deg'], j2['sign']) == (1, 112.676, -1)
    assert (j1['lower_deg'], j1['upper_deg'], j2['deadband_pos']) == (-30.0, 50.0, 0.07)
    a, b = motors['servo_1'], motors['servo_2']
    assert (a['channel'], a['neutral_us'], a['deadband_edges_us']) == (0, 1630, [1420, 1840])
    assert (b['channel'], b['neutral_us'], b['deadband_edges_us']) == (1, 1605, [1510, 1700])


def enable_gripper(data, max_v=None):
    data['joints']['gripper'].update(enabled=True, limits_deg=[0.0, 60.0])
    data['encoders']['enc_7']['channel'] = 7
    data['servos']['servo_9']['channel'] = 8
    for sensor in data['force_sensors'].values():
        sensor['adc_channel'] = 7
        if max_v is not None:
            sensor['max_v'] = max_v
    return data


def test_gripper_moves_in_the_middle_of_its_travel(data):
    """travel_margin 0.9 of [0, 60] → [3, 57]: 5 % short of each end."""
    config, _ = arm_file.driver_config(enable_gripper(data, max_v=2.0))
    gripper = config['groups']['gripper']
    joint = gripper['joints']['gripper']
    assert (joint['lower_deg'], joint['upper_deg']) == pytest.approx((3.0, 57.0))
    assert gripper['controller'] == 'gripper' and gripper['adc']['mux_channel'] == 7
    assert gripper['contact'] == pytest.approx({'finger_1': 1.8, 'finger_2': 1.8})  # 90 % × 2.0
    assert gripper['force_sensors'] == {'finger_1': {'input': 0}, 'finger_2': {'input': 1}}


def test_gripper_without_measured_pressure_has_no_contact_level(data):
    config, _ = arm_file.driver_config(enable_gripper(data))
    assert config['groups']['gripper']['contact'] == {}


def test_parameter_changes(data):
    change = arm_file.parameter_change
    c = change(data, 'joints.joint_1.zero_deg', 270.5)
    assert c.kind == JOINT and c.fields['zero_deg'] == 270.5
    assert change(data, 'joints.joint_2.invert', False).fields['sign'] == 1
    assert change(data, 'joints.joint_2.deadband', [0.1, 0.2]).fields['deadband_neg'] == 0.2
    assert change(data, 'servos.servo_1.neutral_us', 1625) == \
        Change(SERVO, 'servo_1', {'neutral_us': 1625.0, 'deadband_edges_us': [1420.0, 1840.0]})
    assert change(data, 'servos.servo_1.deadband_us', [1430, 1830]).fields['deadband_edges_us'] \
        == [1430.0, 1830.0]
    gripper = change(data, 'force_sensors.finger_1.max_v', 2.0)
    assert gripper == Change(GRIPPER_GROUP, 'gripper', {'contact': {'finger_1': 1.8}})
    assert change(data, 'joints.gripper.travel_margin', 1.0).kind == JOINT
    for structural in ('servos.servo_1.channel', 'joints.joint_1.servos.servo_2',
                       'joints.joint_3.enabled', 'encoders.enc_1.channel',
                       'joints.joint_1.law.kp'):
        assert change(data, structural, 1).kind == RESTART, structural
    assert change(data, 'joints.joint_1.description', 'x') is None
    assert change(data, 'joints.joint_1.no_such_field', 1) is None


def test_servo_recalibration_while_running(config, data):
    sim = SimulatedArm(copy.deepcopy(config))
    driver = Driver(config, sim.encoders, sim.pwm)
    driver.tick(0.0)
    assert driver.apply_change(arm_file.parameter_change(data, 'servos.servo_1.neutral_us', 1625))
    motor = config['groups'][BASE]['motors']['servo_1']
    assert motor['neutral_us'] == 1625
    report = driver.tick(DT)
    assert sim.pwm.pulses[0] == 1625                          # το φρένο με το νέο νεκρό σημείο
    assert any('servo servo_1' in text for _, text in report.events)
    assert driver.apply_change(arm_file.parameter_change(data, 'servos.servo_1.channel', 5))
    assert motor['channel'] == 0                              # όχι ζωντανά
    assert any('επανεκκίνηση' in text for _, text in driver.tick(2 * DT).events)
    unused = arm_file.parameter_change(data, 'servos.servo_7.neutral_us', 1610)
    assert not driver.apply_change(unused)


def test_new_zero_while_running_moves_the_reading_and_stops(config, data):
    sim = SimulatedArm(copy.deepcopy(config))
    driver = Driver(config, sim.encoders, sim.pwm)
    now = 0.0
    for _ in range(50):
        driver.command({'joint_1': 30.0, 'joint_2': 0.0}, now)
        sim.step(DT)
        report = driver.tick(now)
        now += DT
    assert report.moving
    raw = report.joints['joint_1'].raw_deg                   # «Μηδέν = τρέχουσα θέση»
    assert driver.apply_change(arm_file.parameter_change(data, 'joints.joint_1.zero_deg', raw))
    report = driver.tick(now)
    assert report.joints['joint_1'].position_deg == pytest.approx(0.0, abs=0.2)
    assert not report.moving
    assert any('νέα βαθμονόμηση joint_1' in text for _, text in report.events)


def test_new_pressure_limit_reaches_the_running_gripper(data):
    config, _ = arm_file.driver_config(enable_gripper(data))
    sim = SimulatedArm(copy.deepcopy(config))
    sensors = {'finger_1': FakeForceSensor(1.5), 'finger_2': FakeForceSensor(0.94)}
    driver = Driver(config, sim.encoders, sim.pwm, force_sensors=sensors)
    gripper = next(c for c in driver.controllers if 'gripper' in c.joints)
    assert gripper.contact_levels == {}
    change = arm_file.parameter_change(data, 'force_sensors.finger_1.max_v', 1.6)
    assert driver.apply_change(change)
    assert gripper.contact_levels == pytest.approx({'finger_1': 1.44})
