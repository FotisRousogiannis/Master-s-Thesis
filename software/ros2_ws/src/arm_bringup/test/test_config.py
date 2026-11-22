"""Consistency checks between the controllers, the URDF and the simulation world (no ROS needed)."""

import ast
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml

PKG = Path(__file__).resolve().parents[1]
XACRO_FILE = PKG.parent / "arm_description" / "urdf" / "arm.urdf.xacro"
CONTROLLERS = yaml.safe_load((PKG / "config" / "controllers.yaml").read_text())


def _ros2_control_joints(hardware):
    robot = ET.fromstring(xacro.process_file(str(XACRO_FILE), mappings={"hardware": hardware}).toxml())
    return [j.get("name") for j in robot.find("ros2_control").findall("joint")]


def test_every_controller_has_a_type():
    cm = CONTROLLERS["controller_manager"]["ros__parameters"]
    assert cm["joint_state_broadcaster"]["type"] == "joint_state_broadcaster/JointStateBroadcaster"
    assert cm["arm_controller"]["type"] == "joint_trajectory_controller/JointTrajectoryController"


def test_arm_controller_drives_exactly_the_ros2_control_joints():
    params = CONTROLLERS["arm_controller"]["ros__parameters"]
    for hardware in ("mock", "gazebo", "real"):
        assert params["joints"] == _ros2_control_joints(hardware)
    assert params["command_interfaces"] == ["position"]
    assert params["state_interfaces"] == ["position", "velocity"]


def test_world_is_valid_xml_with_physics_and_sensors():
    world = ET.parse(PKG / "worlds" / "arm_world.sdf").getroot().find("world")
    systems = {p.get("name") for p in world.findall("plugin")}
    assert {"gz::sim::systems::Physics", "gz::sim::systems::Sensors"} <= systems


def test_launch_file_parses():
    ast.parse((PKG / "launch" / "bringup.launch.py").read_text())


def test_real_hardware_talks_to_the_pi_topics():
    """hardware:=real: the topics of arm_pi_driver (segment base), a command every cycle (ADR-0024)."""
    robot = ET.fromstring(xacro.process_file(str(XACRO_FILE), mappings={"hardware": "real"}).toxml())
    hw = robot.find("ros2_control").find("hardware")
    params = {p.get("name"): p.text for p in hw.findall("param")}
    assert hw.find("plugin").text == "topic_based_ros2_control/TopicBasedSystem"
    assert params == {"joint_commands_topic": "/base/joint_commands", "joint_states_topic": "/base/joint_states",
                      "trigger_joint_command_threshold": "-1"}
    launch = (PKG / "launch" / "bringup.launch.py").read_text()
    assert 'REAL_STATES_TOPIC = "/base/joint_states"' in launch


def test_pi_driver_uses_the_same_topics():
    node = (PKG.parents[2] / "pi" / "arm_pi_driver" / "arm_pi_driver" / "node.py").read_text()
    assert "/{self.segment}/joint_commands" in node and "/{self.segment}/joint_states" in node
    arm = yaml.safe_load((PKG.parent / "arm_calibration" / "config" / "arm.yaml").read_text())
    assert arm["controller"]["segment"] == "base"
