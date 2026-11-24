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
    assert chain[::-1] == ["base_link", *[f"link_{i}" for i in range(1, 7)], "tool0"]
    # off the chain only the gripper, fixed to the flange for now (docs/decisions/0026)
    for other in links - set(chain) - {"world"}:
        while other in parent_of and other != "gripper_base":
            other = parent_of[other]
        assert other == "gripper_base"


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


@pytest.mark.parametrize("hardware", ["mock", "gazebo", "real"])
def test_ros2_control_covers_every_arm_joint(hardware):
    robot = ET.fromstring(
        xacro.process_file(str(XACRO_FILE), mappings={"hardware": hardware, "controllers_file": "/c.yaml"}).toxml()
    )
    systems = robot.findall("ros2_control")
    assert len(systems) == 1
    assert [j.get("name") for j in systems[0].findall("joint")] == ARM_JOINTS
    for joint in systems[0].findall("joint"):
        assert [c.get("name") for c in joint.findall("command_interface")] == ["position"]
        assert {s.get("name") for s in joint.findall("state_interface")} == {"position", "velocity"}
    # The Gazebo plugin must exist only in simulation and must point at the controllers file.
    plugins = robot.findall("gazebo/plugin")
    if hardware == "gazebo":
        assert len(plugins) == 1 and plugins[0].find("parameters").text == "/c.yaml"
    else:
        assert plugins == []


def test_zero_pose_is_the_vertical_arm(robot):
    """The Fusion export is the vertical arm: at zero every joint lies on the vertical through the base."""
    import numpy as np

    sys_path = str(XACRO_FILE.parents[1] / "scripts")
    import sys
    sys.path.insert(0, sys_path)
    import from_fusion

    joints = {j.get("name"): j for j in robot.findall("joint")}
    pose, heights = np.eye(4), []
    for name in ["world_to_base", *ARM_JOINTS]:
        pose = pose @ from_fusion.transform(joints[name].find("origin"))
        heights.append(pose[2, 3])
        assert abs(pose[0, 3]) < 1e-3 and abs(pose[1, 3]) < 0.08, name   # on the axis (shoulder offset 76 mm)
    assert heights == sorted(heights) and heights[-1] > 0.9


def test_meshes_are_in_the_fusion_zip(robot):
    """Drawn meshes come from the zip that CMake extracts, collision meshes are the light ones (ADR-0026)."""
    import zipfile

    zip_path = XACRO_FILE.parents[4] / "cad_export" / "fusion_urdf" / "URDF.zip"
    names = {Path(n).name for n in zipfile.ZipFile(zip_path).namelist() if n.endswith(".stl")}
    visual = {Path(m.get("filename")).name for m in robot.findall("link/visual/geometry/mesh")}
    assert visual and visual <= names
    assert all(m.get("filename").startswith("package://arm_description/meshes/")
               for m in robot.findall("link/visual/geometry/mesh"))
    # collisions: the light meshes in this package, one for every drawn mesh
    light = {p.name for p in (XACRO_FILE.parents[1] / "meshes_collision").glob("*.stl")}
    collision = robot.findall("link/collision/geometry/mesh")
    assert {Path(m.get("filename")).name for m in collision} == visual <= light
    assert all(m.get("filename").startswith("package://arm_description/meshes_collision/") for m in collision)


def test_generated_model_is_up_to_date():
    """arm_model.xacro is what from_fusion.py makes of the exported URDF: never edited by hand."""
    import sys
    sys.path.insert(0, str(XACRO_FILE.parents[1] / "scripts"))
    import from_fusion

    source = XACRO_FILE.parents[4] / "cad_export" / "fusion_urdf" / "URDF.urdf"
    assert from_fusion.convert(source) == (XACRO_FILE.parent / "arm_model.xacro").read_text(encoding="utf-8")


@pytest.mark.parametrize("hardware, friction", [("gazebo", 10.0), ("mock", 0.0), ("real", 0.0)])
def test_joint_friction_only_in_simulation(hardware, friction):
    """In Gazebo the joints hold like the non-back-drivable servos; elsewhere the model carries no friction."""
    robot = ET.fromstring(xacro.process_file(str(XACRO_FILE), mappings={"hardware": hardware}).toxml())
    for joint in robot.findall("joint"):
        if joint.get("type") == "revolute":
            assert float(joint.find("dynamics").get("friction")) == friction, joint.get("name")
            assert float(joint.find("limit").get("effort")) > 3 * friction       # the motor still wins
