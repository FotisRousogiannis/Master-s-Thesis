"""Structural checks on the arm model: it must expand, form a single tree and be physically valid."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro

XACRO_FILE = Path(__file__).resolve().parents[1] / "urdf" / "arm.urdf.xacro"
ARM_JOINTS = [f"joint_{i}" for i in range(1, 7)]


@pytest.fixture(scope="module")
def robot():
    return ET.fromstring(xacro.process_file(str(XACRO_FILE)).toxml())


def test_has_six_revolute_arm_joints(robot):
    revolute = [j.get("name") for j in robot.findall("joint") if j.get("type") == "revolute"]
    assert revolute == ARM_JOINTS


def test_kinematic_tree_is_a_single_chain_to_tool0(robot):
    parent_of = {j.find("child").get("link"): j.find("parent").get("link") for j in robot.findall("joint")}
    links = {link.get("name") for link in robot.findall("link")}
    roots = links - parent_of.keys()
    assert roots == {"world"}

    chain, link = [], "tool0"
    while link in parent_of:
        chain.append(link)
        link = parent_of[link]
    assert link == "world"
    assert len(chain) == len(links) - 1


def test_joint_limits_are_valid(robot):
    for joint in robot.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        assert float(limit.get("lower")) < float(limit.get("upper")), joint.get("name")
        assert float(limit.get("velocity")) > 0.0, joint.get("name")
        assert float(limit.get("effort")) > 0.0, joint.get("name")


def test_physical_links_have_positive_mass_and_inertia(robot):
    for link in robot.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            # Only frame-only links (world, tool0) may omit inertia.
            assert link.get("name") in {"world", "tool0"}
            continue
        assert float(inertial.find("mass").get("value")) > 0.0
        inertia = inertial.find("inertia")
        for axis in ("ixx", "iyy", "izz"):
            assert float(inertia.get(axis)) > 0.0, f"{link.get('name')} {axis}"
