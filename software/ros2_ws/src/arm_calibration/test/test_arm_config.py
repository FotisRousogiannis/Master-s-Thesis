import copy

import pytest

from arm_calibration import arm_config as a


@pytest.fixture
def data():
    return a.load()


def test_repository_file_is_valid(data):
    assert a.validate(data) == []


def test_base_values_moved_from_the_old_files(data):
    """Bench values (2026-11-14) survive the move to the one file."""
    s, j = data["servos"], data["joints"]
    assert (s["servo_1"]["channel"], s["servo_1"]["neutral_us"], s["servo_1"]["deadband_us"]) == (0, 1630, [1420, 1840])
    assert (s["servo_2"]["channel"], s["servo_2"]["neutral_us"], s["servo_2"]["deadband_us"]) == (1, 1605, [1510, 1700])
    assert (j["joint_1"]["zero_deg"], j["joint_1"]["invert"]) == (267.891, False)
    assert j["joint_1"]["deadband"] == [0.05, 0.05]
    assert (j["joint_2"]["zero_deg"], j["joint_2"]["invert"], j["joint_2"]["deadband"]) == (112.676, True, [0.07, 0.07])
    assert data["encoders"][j["joint_1"]["encoder"]]["channel"] == 2
    assert data["encoders"][j["joint_2"]["encoder"]]["channel"] == 1
    assert j["joint_1"]["limits_deg"] == [-30.0, 50.0]


def test_groups_and_kinds(data):
    kinds = {g.joints: (g.kind, g.enabled) for g in a.groups(data)}
    assert kinds[("joint_1", "joint_2")] == (a.DIFFERENTIAL, True)
    assert kinds[("joint_3",)] == (a.MIRRORED, False)
    assert kinds[("joint_5", "joint_6")] == (a.DIFFERENTIAL, False)
    assert kinds[("gripper",)] == (a.SINGLE, False)


def test_servo_signs_of_the_base(data):
    base = a.groups(data)[0]
    assert a.servo_signs(data, base) == {"servo_1": {"joint_1": 1, "joint_2": 1},
                                         "servo_2": {"joint_1": -1, "joint_2": 1}}


def errors_after(data, change):
    bad = copy.deepcopy(data)
    change(bad)
    return " | ".join(a.validate(bad))


def test_same_channel_twice_is_refused(data):
    assert "κανάλι PCA 0" in errors_after(data, lambda d: d["servos"]["servo_2"].update(channel=0))
    assert "κανάλι encoder 2" in errors_after(data, lambda d: d["encoders"]["enc_2"].update(channel=2))


def test_unknown_references_are_refused(data):
    assert "άγνωστο servo" in errors_after(data, lambda d: d["joints"]["joint_1"]["servos"].update(servo_99=1))
    assert "άγνωστος encoder" in errors_after(data, lambda d: d["joints"]["joint_1"].update(encoder="enc_99"))


def test_sign_must_be_plus_or_minus_one(data):
    assert "πρόσημο" in errors_after(data, lambda d: d["joints"]["joint_1"]["servos"].update(servo_1=2))


def test_differential_with_equal_signs_is_refused(data):
    """Both joints +1, +1: the two motors cannot tell the joints apart."""
    msg = errors_after(data, lambda d: d["joints"]["joint_1"]["servos"].update(servo_2=1))
    assert "δεν κινούνται ανεξάρτητα" in msg


def test_enabled_joint_needs_its_wiring(data):
    def enable_joint_3(d):
        d["joints"]["joint_3"]["enabled"] = True
    msg = errors_after(data, enable_joint_3)
    assert "servo_3 δεν έχει κανάλι" in msg and "δεν έχει encoder με κανάλι" in msg


def test_differential_is_enabled_as_a_whole(data):
    msg = errors_after(data, lambda d: d["joints"]["joint_2"].update(enabled=False))
    assert "ενεργές ή ανενεργές όλες μαζί" in msg


def test_servo_deadband_must_contain_neutral(data):
    msg = errors_after(data, lambda d: d["servos"]["servo_2"].update(deadband_us=[1510, 1590]))
    assert "έξω από τη νεκρή ζώνη" in msg


def test_save_keeps_header_and_round_trips(data, tmp_path):
    path = tmp_path / "arm.yaml"
    path.write_text(a.FILE.read_text(encoding="utf-8"), encoding="utf-8")
    data["joints"]["joint_1"]["zero_deg"] = 10.5
    a.save(data, path)
    text = path.read_text(encoding="utf-8")
    assert text.startswith("# Ο βραχίονας σε ΕΝΑ αρχείο")
    assert a.load(path) == data


def test_save_refuses_an_invalid_file(data, tmp_path):
    path = tmp_path / "arm.yaml"
    data["servos"]["servo_1"]["channel"] = 99
    with pytest.raises(a.ArmConfigError):
        a.save(data, path)
    assert not path.exists()


def test_parameters_are_flat_and_typed(data):
    params = a.parameters(data)
    assert params["joints.joint_1.zero_deg"] == 267.891
    assert params["joints.joint_1.servos.servo_2"] == -1
    assert params["servos.servo_1.deadband_us"] == [1420, 1840]
    assert params["encoders.enc_3.channel"] == -1                  # null channel: -1
    assert "joints.joint_3.encoder" in params and "force_sensors.finger_1.max_v" not in params
    assert isinstance(params["joints.gripper.limits_deg"][0], float)


def test_set_value_from_ros(data):
    from array import array
    a.set_value(data, "joints.joint_1.limits_deg", array("d", [-20, 40]))
    assert data["joints"]["joint_1"]["limits_deg"] == [-20.0, 40.0]
    a.set_value(data, "encoders.enc_3.channel", 4)                 # a null gets a value
    assert data["encoders"]["enc_3"]["channel"] == 4
    a.set_value(data, "encoders.enc_3.channel", -1)                # and back to null
    assert data["encoders"]["enc_3"]["channel"] is None
    a.set_value(data, "joints.joint_3.servos.servo_10", 1)         # a servo added to a joint
    assert data["joints"]["joint_3"]["servos"]["servo_10"] == 1
    with pytest.raises(KeyError):
        a.set_value(data, "joints.joint_1.typo", 1)
    with pytest.raises(KeyError):
        a.set_value(data, "nothing.here", 1)


def test_unknown_keys_are_refused(data):
    """`{description: βάση, κινητήρας A}` without quotes gives a key «κινητήρας A»."""
    msg = errors_after(data, lambda d: d["servos"]["servo_1"].update({"κινητήρας A": None}))
    assert "άγνωστα κλειδιά κινητήρας A" in msg
