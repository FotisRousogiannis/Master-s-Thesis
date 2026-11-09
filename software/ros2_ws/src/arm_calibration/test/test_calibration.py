import shutil
from pathlib import Path

import pytest

from arm_calibration import calibration as c

FILE = Path(__file__).resolve().parents[1] / "config" / "calibration.yaml"


@pytest.fixture
def flat():
    return c.load(FILE)


def test_repository_file_is_valid_and_matches_the_bench_values(flat):
    assert c.validate(flat) == []
    assert flat["spec.pulse_min_us"] == 500 and flat["spec.pulse_max_us"] == 2500
    assert flat["servos.wrist_diff_a.channel"] == 9 and flat["servos.wrist_diff_b.channel"] == 10
    assert flat["servos.wrist_diff_b.invert"] is True  # motor B mirrored (b_sign = -1)
    assert all(flat[f"servos.{s}.neutral_us"] == 1600 for s in c.servo_names(flat))


def test_saving_an_unchanged_file_reproduces_it_exactly(flat):
    assert c.dump(flat) == FILE.read_text(encoding="utf-8")


def test_pulse_outside_the_datasheet_range_is_rejected(flat):
    flat["spec.pulse_max_us"] = 2600
    assert any("εκτός ορίων" in e for e in c.validate(flat))


def test_neutral_must_be_inside_the_pulse_limits(flat):
    flat["spec.pulse_max_us"] = 1500
    flat["safety.max_offset_us"] = 400
    assert any("wrist_diff_a.neutral_us" in e for e in c.validate(flat))


def test_two_servos_on_one_channel_are_rejected(flat):
    flat["servos.wrist_diff_b.channel"] = 9
    assert any("κανάλι 9" in e for e in c.validate(flat))


def test_types_are_enforced():
    assert c.coerce("servos.x.neutral_us", 1600.0) == 1600
    assert c.coerce("safety.max_duration_s", 3) == 3.0
    with pytest.raises(ValueError):
        c.coerce("servos.x.neutral_us", 1600.5)
    with pytest.raises(ValueError):
        c.coerce("servos.x.invert", "yes")
    with pytest.raises(KeyError):
        c.coerce("servos.x.unknown", 1)


def test_save_round_trip_through_a_symlink(tmp_path, flat):
    real = tmp_path / "real.yaml"
    shutil.copy(FILE, real)
    link = tmp_path / "link.yaml"
    link.symlink_to(real)
    flat["servos.wrist_diff_a.neutral_us"] = 1605
    c.save(link, flat)
    assert link.is_symlink()  # the repo file is written, the install symlink stays
    assert c.load(real)["servos.wrist_diff_a.neutral_us"] == 1605


def test_invalid_calibration_is_never_written(tmp_path, flat):
    target = tmp_path / "c.yaml"
    flat["spec.pulse_min_us"] = 3000
    with pytest.raises(ValueError):
        c.save(target, flat)
    assert not target.exists()


def test_every_joint_has_an_encoder_zero(flat):
    assert c.encoder_names(flat) == ["gripper", *(f"joint_{i}" for i in range(1, 7))]
    assert c.field_for("encoders.joint_3.zero_deg").type is float
    flat["encoders.joint_3.zero_deg"] = 360.0  # same as 0: only [0, 360) is allowed
    assert any("joint_3.zero_deg" in e for e in c.validate(flat))
    del flat["encoders.joint_3.invert"]
    assert "λείπει το encoders.joint_3.invert" in c.validate(flat)


def test_gripper_cannot_open_until_calibrated(flat):
    assert flat["gripper.open_deg"] == 0.0  # range [0, 0]: no motion before calibration
