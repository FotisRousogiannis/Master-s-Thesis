import copy

from arm_pi_driver.calibration import apply_calibration, gripper_limits, parameter_change
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
    assert parameter_change('encoders.joint_1.zero_deg', 270.5) == ('joint_1', {'zero_deg': 270.5})
    assert parameter_change('encoders.joint_2.invert', True) == ('joint_2', {'sign': -1})
    assert parameter_change('gripper.open_deg', -45.0) == ('gripper', gripper_limits(-45.0))
    assert parameter_change('servos.wrist_diff_a.neutral_us', 1605) is None


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
