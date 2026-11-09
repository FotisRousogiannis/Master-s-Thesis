import pytest

from arm_calibration import encoders as e


def test_zero_reads_as_zero_whatever_the_direction():
    assert e.joint_angle(123.4, 123.4, False, -90, 90) == pytest.approx(0.0)
    assert e.joint_angle(123.4, 123.4, True, -90, 90) == pytest.approx(0.0)


def test_inverted_encoder_gives_the_opposite_angle():
    assert e.joint_angle(130.0, 100.0, False, -90, 90) == pytest.approx(30.0)
    assert e.joint_angle(130.0, 100.0, True, -90, 90) == pytest.approx(-30.0)


def test_encoder_wrap_is_hidden_when_the_zero_is_near_0_360():
    # zero at 350°: the reading crosses 360 -> 0 inside the range, the joint angle does not jump
    assert e.joint_angle(340.0, 350.0, False, -90, 90) == pytest.approx(-10.0)
    assert e.joint_angle(10.0, 350.0, False, -90, 90) == pytest.approx(20.0)


def test_the_jump_lands_in_the_unreachable_part_of_the_circle():
    start, margin = e.window(-137.5, 137.5)  # a joint with a 275° range
    assert margin == pytest.approx(85.0)
    for raw in range(0, 360):
        angle = e.joint_angle(float(raw), 0.0, False, -137.5, 137.5)
        assert start <= angle < start + 360
    # just outside the limits is still read correctly, not as the other end
    assert e.joint_angle(140.0, 0.0, False, -137.5, 137.5) == pytest.approx(140.0)
    assert e.joint_angle(220.0, 0.0, False, -137.5, 137.5) == pytest.approx(-140.0)


def test_full_turn_joints_have_no_margin():
    assert e.window(-180, 180) == (-180, 0)
    with pytest.raises(ValueError):
        e.window(-200, 200)


def test_zeros_are_set_for_all_joints_or_none():
    reports = {"joint_1": (10.0, 0, 12.345), "joint_2": (10.0, 0, 300.0)}
    zeros, problems = e.zeros_from_reports(["joint_1", "joint_2"], reports, now=10.5)
    assert problems == [] and zeros == {"joint_1": 12.35, "joint_2": 300.0}

    for bad in ({"joint_2": (10.0, 0, 300.0)},                                # joint_1 silent
                {"joint_1": (5.0, 0, 12.0), "joint_2": (10.0, 0, 300.0)},     # stale
                {"joint_1": (10.0, 2, 12.0), "joint_2": (10.0, 0, 300.0)},    # magnet error
                {"joint_1": (10.0, 0, None), "joint_2": (10.0, 0, 300.0)}):   # no raw angle
        zeros, problems = e.zeros_from_reports(["joint_1", "joint_2"], bad, now=10.5)
        assert zeros == {} and len(problems) == 1 and problems[0].startswith("joint_1")
