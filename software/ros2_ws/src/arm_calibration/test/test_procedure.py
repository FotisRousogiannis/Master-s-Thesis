import pytest

from arm_calibration import arm_config, procedure as p


@pytest.fixture
def data():
    return arm_config.load()


def names(proposals):
    return {x.name: x.new for x in proposals}


SCAN = {0: {"devices": ["PCA9685"], "encoder": None},
        1: {"devices": ["AS5600"], "encoder": {"raw_deg": 112.7, "status": "OK"}},
        2: {"devices": ["AS5600"], "encoder": {"raw_deg": 120.0, "status": "NO_MAGNET"}},
        4: {"devices": ["AS5600"], "encoder": {"raw_deg": 10.0, "status": "OK"}}}


def test_scan_reports_magnets_missing_chips_and_spare_encoders(data):
    texts = [(f.level, f.text) for f in p.check_scan(data, SCAN)]
    assert (p.INFO, "joint_2: encoder OK (κανάλι 1)") in texts
    assert (p.ERROR, "joint_1: μαγνήτης NO_MAGNET (κανάλι 2)") in texts
    assert any("κανάλι 4: δεν ανήκει" in t for _, t in texts)
    no_pca = {**SCAN, 0: {"devices": [], "encoder": None}}
    assert any(level == p.ERROR and "PCA9685" in t for level, t in
               [(f.level, f.text) for f in p.check_scan(data, no_pca)])


def test_jog_request_uses_the_signs_of_the_file(data):
    assert p.joint_jog(data, "joint_1")["servos"] == {"servo_1": 0.4, "servo_2": -0.4}


def test_jog_where_the_file_is_right_proposes_nothing(data):
    m = p.analyze_jog(data, "joint_1", {"moved": {"2": 3.2}, "reason": "κινήθηκε"})
    assert (m.channel, m.delta, m.proposals, m.findings) == (2, 3.2, [], [])


def test_encoder_in_another_channel_is_proposed(data):
    m = p.analyze_jog(data, "joint_1", {"moved": {"5": -3.1}})
    assert names(m.proposals) == {"encoders.enc_1.channel": 5}
    assert m.proposals[0].old == 2


def test_encoders_swapped_between_two_joints(data):
    """joint_1 moved the encoder that the file gives to joint_2: wrong channel of servo or encoder."""
    m = p.analyze_jog(data, "joint_1", {"moved": {"1": 3.0}})
    assert names(m.proposals) == {"joints.joint_1.encoder": "enc_2"}
    assert "joint_2" in m.proposals[0].why


def test_two_encoders_moving_is_reported(data):
    m = p.analyze_jog(data, "joint_1", {"moved": {"2": 3.0, "1": -2.0}})
    assert m.channel == 2 and any("κανάλι 1" in f.text for f in m.findings)


def test_mirrored_servos_that_fight_get_a_sign_proposal(data):
    m = p.analyze_jog(data, "joint_3", {"moved": {}, "reason": "λήξη χρόνου"})
    assert names(m.proposals) == {"joints.joint_3.servos.servo_4": 1}
    assert any("παλεύουν" in f.text for f in m.findings)


@pytest.mark.parametrize("delta, positive, expected", [
    (3.0, True, {"joints.joint_2.invert": False}),
    (-3.0, True, {}),                                                  # joint_2 already invert
    (3.0, False, {"joints.joint_2.servos.servo_1": -1, "joints.joint_2.servos.servo_2": -1}),
])
def test_direction(data, delta, positive, expected):
    assert names(p.direction_proposals(data, "joint_2", delta, positive)) == expected


def test_direction_proposals_keep_the_file_valid(data):
    new, errors = p.apply(data, p.direction_proposals(data, "joint_1", 3.0, False))
    assert errors == [] and new["joints"]["joint_1"]["servos"] == {"servo_1": -1, "servo_2": 1}


def run(sweep, moves_at):
    """moves_at(direction, level) -> True if the encoder moves."""
    requests = 0
    while (request := sweep.next_request()) is not None:
        speed = list(request["servos"].values())[0]
        level = sweep.levels[sweep._index]
        moved = moves_at(1 if speed > 0 else -1, level)
        sweep.record({"moved": {"2": 1.0 if moved else 0.0}})
        requests += 1
    return requests


def test_servo_deadband_sweep(data):
    sweep = p.ServoDeadbandSweep(data, "servo_1")
    run(sweep, lambda d, us: us >= (220 if d > 0 else 200))      # edges 1430 / 1850
    assert sweep.found == {1: 220, -1: 200}
    assert names(sweep.proposals(data)) == {"servos.servo_1.deadband_us": [1430, 1850],
                                            "servos.servo_1.neutral_us": 1640}


def test_servo_deadband_requests_are_plain_pulses(data):
    request = p.ServoDeadbandSweep(data, "servo_1").next_request()
    assert request["plain"] is True
    assert 1630 + request["servos"]["servo_1"] * (2500 - 1630) == pytest.approx(1640)


def test_joint_deadband_sweep_and_no_motion(data):
    sweep = p.JointDeadbandSweep(data, "joint_2")
    run(sweep, lambda d, s: s >= (0.06 if d > 0 else 0.08))
    assert names(sweep.proposals(data)) == {"joints.joint_2.deadband": [0.06, 0.08]}
    still = p.JointDeadbandSweep(data, "joint_2")
    assert run(still, lambda d, s: False) == 2 * len(still.levels) and still.proposals(data) == []


def test_zero_and_gripper(data):
    assert names(p.zero_proposals(data, {"joint_1": 270.5})) == {"joints.joint_1.zero_deg": 270.5}
    proposals, findings = p.gripper_limit_proposals(data, "gripper", 350.0, 50.0)   # opens +60°
    assert names(proposals) == {"joints.gripper.zero_deg": 350.0, "joints.gripper.limits_deg": [0.0, 60.0]}
    proposals, _ = p.gripper_limit_proposals(data, "gripper", 50.0, 350.0)          # opens −60°
    assert names(proposals)["joints.gripper.invert"] is True
    assert p.gripper_limit_proposals(data, "gripper", 10.0, 11.0)[1][0].level == p.ERROR
    assert names(p.pressure_proposals(data, {"finger_1": 2.1})) == {"force_sensors.finger_1.max_v": 2.1}


def test_relabel_a_joint_with_its_own_servos(data):
    """joint_3 (servo_3/4) was actually what the file calls joint_4 (servo_5/6)."""
    new, errors = p.apply(data, p.relabel_proposals(data, "joint_3", "joint_4"))
    assert errors == []
    assert new["joints"]["joint_3"]["servos"] == {"servo_5": 1, "servo_6": -1}
    assert new["joints"]["joint_4"]["servos"] == {"servo_3": 1, "servo_4": -1}
    assert (new["joints"]["joint_3"]["encoder"], new["joints"]["joint_4"]["encoder"]) == ("enc_4", "enc_3")
    assert p.relabel_proposals(data, "joint_1", "joint_1") == []
