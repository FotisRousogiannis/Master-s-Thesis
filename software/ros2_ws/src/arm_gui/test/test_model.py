import math

import pytest

from arm_gui.model import MIN_MOVE_DURATION_S, JointInfo, move_duration, parse_joints

URDF = """<robot name="r">
  <link name="a"/><link name="b"/><link name="c"/><link name="d"/><link name="e"/>
  <joint name="fixed" type="fixed"><parent link="a"/><child link="b"/></joint>
  <joint name="j1" type="revolute"><parent link="b"/><child link="c"/>
    <limit lower="-1.5" upper="1.5" effort="10" velocity="0.2"/></joint>
  <joint name="spin" type="continuous"><parent link="c"/><child link="d"/>
    <limit effort="10" velocity="1.0"/></joint>
  <joint name="finger" type="prismatic"><parent link="d"/><child link="e"/>
    <limit lower="0" upper="0.04" effort="10" velocity="0.05"/></joint>
  <joint name="finger_mirror" type="prismatic"><parent link="d"/><child link="e"/>
    <mimic joint="finger"/><limit lower="0" upper="0.04" effort="10" velocity="0.05"/></joint>
</robot>"""


def test_parse_keeps_commandable_joints_in_order_and_skips_fixed_and_mimic():
    joints = parse_joints(URDF)
    assert [j.name for j in joints] == ["j1", "spin", "finger"]
    assert joints[1].lower == -math.pi and joints[1].upper == math.pi
    assert joints[2].is_prismatic and joints[2].unit == "mm"


def test_parse_rejects_missing_velocity_limit():
    with pytest.raises(ValueError):
        parse_joints('<robot><joint name="x" type="revolute"><limit lower="0" upper="1"/></joint></robot>')


def test_display_units_round_trip_and_clamp():
    j = JointInfo("j", "revolute", -1.0, 1.0, 0.1)
    assert j.to_display(math.pi / 2) == pytest.approx(90.0)
    assert j.from_display(90.0) == pytest.approx(math.pi / 2)
    assert j.clamp(5.0) == 1.0 and j.clamp(-5.0) == -1.0
    p = JointInfo("p", "prismatic", 0.0, 0.04, 0.05)
    assert p.to_display(0.02) == pytest.approx(20.0)


def test_move_duration_is_set_by_the_slowest_joint():
    joints = {"a": JointInfo("a", "revolute", -3, 3, 0.1), "b": JointInfo("b", "revolute", -3, 3, 1.0)}
    # a: 1 rad at 0.1 rad/s = 10 s, b: 2 rad at 1 rad/s = 2 s
    assert move_duration({"a": 0, "b": 0}, {"a": 1, "b": 2}, joints, 1.0) == pytest.approx(10.0)
    assert move_duration({"a": 0, "b": 0}, {"a": 1, "b": 2}, joints, 0.5) == pytest.approx(20.0)


def test_move_duration_has_a_minimum_and_validates_scale():
    joints = {"a": JointInfo("a", "revolute", -3, 3, 1.0)}
    assert move_duration({"a": 0.0}, {"a": 0.0}, joints, 1.0) == MIN_MOVE_DURATION_S
    with pytest.raises(ValueError):
        move_duration({}, {}, joints, 0.0)


def test_real_arm_urdf_parses():
    import xacro
    from pathlib import Path

    urdf = Path(__file__).resolve().parents[2] / "arm_description" / "urdf" / "arm.urdf.xacro"
    joints = parse_joints(xacro.process_file(str(urdf)).toxml())
    assert [j.name for j in joints] == [f"joint_{i}" for i in range(1, 7)]
