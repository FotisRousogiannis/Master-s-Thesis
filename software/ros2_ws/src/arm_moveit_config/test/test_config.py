"""The MoveIt files agree with the URDF and the controllers (no ROS needed, docs/decisions/0025)."""

import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
SRDF = ET.parse(PKG / "config" / "arm.srdf").getroot()
URDF = ET.fromstring(xacro.process_file(str(SRC / "arm_description" / "urdf" / "arm.urdf.xacro")).toxml())
ARM_JOINTS = [f"joint_{i}" for i in range(1, 7)]
LIMITS = {j.get("name"): j.find("limit") for j in URDF.findall("joint") if j.get("type") == "revolute"}


def load(name):
    return yaml.safe_load((PKG / "config" / name).read_text())


def test_srdf_names_the_robot_and_links_of_the_urdf():
    links = {link.get("name") for link in URDF.findall("link")}
    assert SRDF.get("name") == URDF.get("name")
    chain = SRDF.find("group").find("chain")
    assert (chain.get("base_link"), chain.get("tip_link")) == ("base_link", "tool0")
    for pair in SRDF.findall("disable_collisions"):
        assert {pair.get("link1"), pair.get("link2")} <= links


def test_only_neighbours_or_parts_fixed_together_skip_collision_checks():
    joint_of = {j.find("child").get("link"): j for j in URDF.findall("joint")}
    parent_of = {child: j.find("parent").get("link") for child, j in joint_of.items()}

    def moving_ancestor(link):          # the nearest link above a movable joint (or itself)
        while link in joint_of and joint_of[link].get("type") == "fixed" and parent_of[link] != "world":
            link = parent_of[link]
        return link

    for pair in SRDF.findall("disable_collisions"):
        a, b = pair.get("link1"), pair.get("link2")
        if pair.get("reason") == "Adjacent":
            assert parent_of.get(a) == b or parent_of.get(b) == a, (a, b)
        else:
            assert pair.get("reason") == "Rigid" and moving_ancestor(a) == moving_ancestor(b), (a, b)


def test_named_poses_are_inside_the_urdf_limits():
    for state in SRDF.findall("group_state"):
        values = {j.get("name"): float(j.get("value")) for j in state.findall("joint")}
        assert sorted(values) == ARM_JOINTS
        for name, value in values.items():
            lower, upper = float(LIMITS[name].get("lower")), float(LIMITS[name].get("upper"))
            assert lower <= value <= upper, (state.get("name"), name)


def test_joint_limits_do_not_exceed_the_urdf():
    limits = load("joint_limits.yaml")["joint_limits"]
    assert sorted(limits) == ARM_JOINTS
    for name, limit in limits.items():
        assert 0 < limit["max_velocity"] <= float(LIMITS[name].get("velocity"))
        assert limit["max_acceleration"] > 0


def test_moveit_executes_through_the_arm_controller():
    moveit = load("moveit_controllers.yaml")["moveit_simple_controller_manager"]
    controllers = yaml.safe_load((SRC / "arm_bringup" / "config" / "controllers.yaml").read_text())
    jtc = controllers["arm_controller"]["ros__parameters"]
    assert moveit["controller_names"] == ["arm_controller"]
    assert moveit["arm_controller"]["type"] == "FollowJointTrajectory"
    assert moveit["arm_controller"]["joints"] == jtc["joints"] == ARM_JOINTS


def test_kinematics_and_rviz_use_the_planning_group():
    assert list(load("kinematics.yaml")) == [SRDF.find("group").get("name")]
    rviz = yaml.safe_load((PKG / "rviz" / "moveit.rviz").read_text())
    displays = rviz["Visualization Manager"]["Displays"]
    planning = [d for d in displays if d["Class"] == "moveit_rviz_plugin/MotionPlanning"]
    assert planning[0]["Planning Request"]["Planning Group"] == "arm"


def test_launch_file_parses():
    ast.parse((PKG / "launch" / "moveit.launch.py").read_text())
