import copy

from arm_pi_driver.control import (BangBang, build_controllers, ControlLaw, Controller, LAWS,
                                   make_law, MixController, PID)
from arm_pi_driver.driver import Driver
from arm_pi_driver.hardware.fake import SimulatedArm
import pytest

DT = 0.02

JOINT_3 = {   # δύο κατοπτρικά servo, μία κίνηση (όπως ο Φώτης περιγράφει το joint_3)
    'joints': {'joint_3': {'mux_channel': 3, 'sign': 1, 'zero_deg': 10.0, 'lower_deg': -90.0,
                           'upper_deg': 90.0, 'law': {'type': 'pid', 'kp': 0.02}}},
    'motors': {
        'a': {'channel': 2, 'neutral_us': 1600, 'deadband_edges_us': [1550, 1650],
              'mix': {'joint_3': 1}},
        'b': {'channel': 3, 'neutral_us': 1600, 'deadband_edges_us': [1520, 1680],
              'mix': {'joint_3': 1}, 'mirrored': True},
    },
}
TOOL = {      # ένα servo (π.χ. εργαλείο)
    'joints': {'tool': {'mux_channel': 7, 'sign': 1, 'zero_deg': 0.0, 'lower_deg': 0.0,
                        'upper_deg': 120.0, 'law': {'type': 'bang_bang', 'speed': 0.2}}},
    'motors': {'a': {'channel': 8, 'neutral_us': 1600, 'deadband_edges_us': [1560, 1640],
                     'mix': {'tool': 1}}},
}


@pytest.fixture
def arm(config):
    """Η βάση + joint_3 + ένα εργαλείο ενός servo, όλα σε έναν Driver."""
    config = copy.deepcopy(config)
    config['groups']['joint_3'] = copy.deepcopy(JOINT_3)
    config['groups']['tool'] = copy.deepcopy(TOOL)
    return config


def run(sim, driver, seconds, targets, start=0.0):
    now = start
    for _ in range(round(seconds / DT)):
        driver.command(targets, now)
        sim.step(DT)
        report = driver.tick(now)
        now += DT
    return report


def test_contracts():
    assert issubclass(MixController, Controller)
    assert issubclass(PID, ControlLaw) and issubclass(BangBang, ControlLaw)
    with pytest.raises(TypeError):
        Controller()
    with pytest.raises(TypeError):
        ControlLaw()


def test_laws_from_yaml():
    assert isinstance(make_law({'type': 'pid', 'kp': 0.1}, 0.5), PID)
    law = make_law({'type': 'bang_bang', 'speed': 0.9}, 0.5)
    assert law.update(-3.0, 0.0, DT) == -0.5            # κόπηκε στο μέγιστο
    assert set(LAWS) >= {'pid', 'bang_bang'}
    with pytest.raises(ValueError):
        make_law({'type': 'κάτι'}, 0.5)


def test_mirrored_pair_moves_both_servos_the_same_way(arm):
    controller = build_controllers(arm)[1]
    assert controller.joints == ('joint_3',)
    speeds = controller.motor_speeds({'joint_3': 0.2})
    assert speeds == {'a': 0.2, 'b': -0.2}       # b κατοπτρικό: ίδια κίνηση, αντίθετη εντολή
    pulses = controller.pulses(speeds)
    assert pulses[2] > 1650 and pulses[3] < 1520  # το καθένα από την άκρη της δικής του ζώνης


def test_whole_arm_reaches_its_targets(arm):
    sim = SimulatedArm(arm)
    driver = Driver(arm, sim.encoders, sim.pwm)
    targets = {'joint_1': 15.0, 'joint_2': -10.0, 'joint_3': 25.0, 'tool': 60.0}
    report = run(sim, driver, 12.0, targets)
    tolerance = arm['control']['tolerance_deg'] + 0.5
    for joint, target in targets.items():
        assert sim.angles[joint] == pytest.approx(target, abs=tolerance), joint
    assert report.reason == 'στον στόχο'


def test_a_problem_in_one_group_stops_all_groups(arm):
    sim = SimulatedArm(arm)
    driver = Driver(arm, sim.encoders, sim.pwm)
    report = run(sim, driver, 0.3, {'joint_1': 40.0, 'joint_3': 40.0})
    assert report.moving
    sim.encoders['joint_3'].status = 'NO_RESPONSE'
    report = run(sim, driver, 0.1, {'joint_1': 40.0, 'joint_3': 40.0}, start=1.0)
    assert not report.moving and set(sim.pwm.pulses.values()) == {None}


def test_a_joint_in_two_groups_is_refused(arm):
    arm['groups']['joint_3']['joints']['joint_1'] = arm['groups']['base']['joints']['joint_1']
    arm['groups']['joint_3']['motors']['a']['mix']['joint_1'] = 1
    with pytest.raises(ValueError):
        build_controllers(arm)


def test_a_joint_without_a_motor_is_refused(arm):
    arm['groups']['tool']['motors']['a']['mix'] = {'tool': 0}
    with pytest.raises(ValueError):
        build_controllers(arm)
