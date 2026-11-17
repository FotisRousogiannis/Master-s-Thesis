import copy

import pytest
import yaml

from arm_calibration import arm_config as a

SRC = a.FILE.parents[2]
OLD_CALIBRATION = a.FILE.parent / "calibration.yaml"
OLD_BASE = SRC.parents[1] / "pi" / "arm_pi_driver" / "config" / "base.yaml"


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


@pytest.mark.skipif(not (OLD_CALIBRATION.exists() and OLD_BASE.exists()), reason="old files removed")
def test_signs_equal_old_mix_times_servo_invert(data):
    """New sign = old mix x (-1 if the servo had invert), for every motor of the old base.yaml."""
    old_cal = yaml.safe_load(OLD_CALIBRATION.read_text(encoding="utf-8"))
    old_base = yaml.safe_load(OLD_BASE.read_text(encoding="utf-8"))
    by_channel = {s["channel"]: name for name, s in data["servos"].items()}
    for group in old_base["groups"].values():
        for motor in group["motors"].values():
            old = old_cal["servos"].get(motor["servo"])
            if old is None:
                continue
            new = by_channel[old["channel"]]
            for joint, mix in motor["mix"].items():
                expected = mix * (-1 if old["invert"] else 1)
                assert data["joints"][joint]["servos"][new] == expected, (joint, new)


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
