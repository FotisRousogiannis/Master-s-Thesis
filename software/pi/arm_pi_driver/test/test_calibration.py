import copy

from arm_pi_driver.calibration import (apply_calibration, Change, GRIPPER_GROUP, gripper_limits,
                                       JOINT, parameter_change, SERVO)
from arm_pi_driver.control import build_controllers
from arm_pi_driver.driver import Driver
from arm_pi_driver.hardware.fake import SimulatedArm
import pytest

DT = 0.02


def test_zeros_and_directions_come_from_calibration_yaml(config):
    joints = config['groups']['base']['joints']
    j1, j2 = joints['joint_1'], joints['joint_2']
    assert j1['zero_deg'] == pytest.approx(267.891) and j1['sign'] == 1
    assert j2['zero_deg'] == pytest.approx(112.676) and j2['sign'] == -1


def test_a_joint_missing_from_the_calibration_is_refused(config):
    config['groups']['base']['joints']['joint_9'] = {'mux_channel': 5}
    with pytest.raises(ValueError):
        apply_calibration(config, {})


def test_parameter_changes_of_arm_calibration():
    assert parameter_change('encoders.joint_1.zero_deg', 270.5) == \
        Change(JOINT, 'joint_1', {'zero_deg': 270.5})
    assert parameter_change('encoders.joint_2.invert', True) == \
        Change(JOINT, 'joint_2', {'sign': -1})
    assert parameter_change('encoders.joint_2.deadband_pos', 0.1) == \
        Change(JOINT, 'joint_2', {'deadband_pos': 0.1})
    assert parameter_change('gripper.open_deg', -45.0) == \
        Change(JOINT, 'gripper', gripper_limits(-45.0))
    assert parameter_change('gripper.contact_v', 1.4) == \
        Change(GRIPPER_GROUP, 'gripper', {'contact': 1.4})
    assert parameter_change('servos.base_a.neutral_us', 1625) == \
        Change(SERVO, 'base_a', {'neutral_us': 1625.0})
    assert parameter_change('servos.base_b.invert', False) == \
        Change(SERVO, 'base_b', {'mirrored': False})
    assert parameter_change('servos.base_a.note', 'κάτι') is None
    assert parameter_change('safety.max_offset_us', 800) is None


def test_measured_values_of_the_base_come_from_the_common_file(config):
    base = config['groups']['base']
    assert base['joints']['joint_2']['deadband_pos'] == pytest.approx(0.07)
    a, b = base['motors']['a'], base['motors']['b']
    assert (a['channel'], a['neutral_us'], a['mirrored']) == (0, 1630, False)
    assert (b['channel'], b['neutral_us'], b['mirrored']) == (1, 1605, True)
    assert b['deadband_edges_us'] == [1510, 1700]


def test_a_motor_without_a_calibrated_servo_is_refused(config):
    config['groups']['base']['motors']['a']['servo'] = 'δεν_υπάρχει'
    with pytest.raises(ValueError):
        apply_calibration(config, {
            'encoders.joint_1.zero_deg': 0.0, 'encoders.joint_1.invert': False,
            'encoders.joint_2.zero_deg': 0.0, 'encoders.joint_2.invert': False})


def test_without_measured_deadband_the_pulses_are_not_compensated(config):
    base = config['groups']['base']
    del base['motors']['a']['deadband_edges_us']
    [controller] = build_controllers(config)
    pulses = controller.pulses(controller.motor_speeds({'joint_1': 0.2, 'joint_2': 0.0}))
    assert pulses[0] == pytest.approx(1630 + 0.2 * (2500 - 1630))   # νεκρό + ταχύτητα × εύρος


def test_servo_recalibration_while_running(config):
    sim = SimulatedArm(copy.deepcopy(config))
    driver = Driver(config, sim.encoders, sim.pwm)
    driver.tick(0.0)
    assert driver.apply_change(parameter_change('servos.base_a.neutral_us', 1625))
    assert driver.apply_change(parameter_change('servos.base_a.deadband_up_us', 1830))
    motor = config['groups']['base']['motors']['a']
    assert motor['neutral_us'] == 1625 and motor['deadband_edges_us'] == [1420, 1830]
    report = driver.tick(DT)
    assert sim.pwm.pulses[0] == 1625                          # το φρένο με το νέο νεκρό σημείο
    assert any('servo base_a' in text for _, text in report.events)
    assert driver.apply_change(parameter_change('servos.base_a.channel', 5))   # όχι ζωντανά
    assert motor['channel'] == 0
    assert any('επανεκκίνηση' in text for _, text in driver.tick(2 * DT).events)
    assert not driver.apply_change(parameter_change('servos.wrist_diff_a.neutral_us', 1610))


def test_new_zero_while_running_moves_the_reading_and_stops(config):
    sim = SimulatedArm(copy.deepcopy(config))
    driver = Driver(config, sim.encoders, sim.pwm)
    now = 0.0
    for _ in range(50):
        driver.command({'joint_1': 30.0, 'joint_2': 0.0}, now)
        sim.step(DT)
        report = driver.tick(now)
        now += DT
    assert report.moving
    old = report.joints['joint_1'].position_deg
    # «Μηδέν = τρέχουσα θέση» στο Arm Studio: το νέο μηδέν είναι η τωρινή ένδειξη του encoder
    raw = report.joints['joint_1'].raw_deg
    assert driver.update_joint('joint_1', {'zero_deg': raw})
    report = driver.tick(now)
    assert report.joints['joint_1'].position_deg == pytest.approx(0.0, abs=0.2)
    assert old > 1.0 and not report.moving                   # στόχοι μηδενίστηκαν: στάση
    assert any('νέα βαθμονόμηση joint_1' in text for _, text in report.events)
    assert not driver.update_joint('joint_9', {'zero_deg': 1.0})   # άγνωστη άρθρωση
