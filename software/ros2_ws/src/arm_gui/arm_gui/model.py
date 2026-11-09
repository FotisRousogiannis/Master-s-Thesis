"""Robot model helpers with no Qt or ROS dependency (unit-tested on their own).

Joint names and limits are read from the URDF published on /robot_description, so the
application follows the model automatically when the provisional URDF (ADR-0009) is replaced.
"""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

COMMANDABLE_TYPES = ("revolute", "continuous", "prismatic")
MIN_MOVE_DURATION_S = 0.5


@dataclass(frozen=True)
class JointInfo:
    name: str
    type: str
    lower: float  # rad or m
    upper: float  # rad or m
    velocity: float  # rad/s or m/s, > 0

    @property
    def is_prismatic(self) -> bool:
        return self.type == "prismatic"

    def clamp(self, value: float) -> float:
        return min(max(value, self.lower), self.upper)

    def to_display(self, value: float) -> float:
        """SI value -> value shown to the user (degrees or millimetres)."""
        return value * 1000.0 if self.is_prismatic else math.degrees(value)

    def from_display(self, value: float) -> float:
        return value / 1000.0 if self.is_prismatic else math.radians(value)

    @property
    def unit(self) -> str:
        return "mm" if self.is_prismatic else "°"


def parse_joints(urdf_xml: str) -> list[JointInfo]:
    """Commandable joints of a URDF, in file order. Mimic joints follow others and are skipped."""
    robot = ET.fromstring(urdf_xml)
    joints = []
    for joint in robot.findall("joint"):
        kind = joint.get("type")
        if kind not in COMMANDABLE_TYPES or joint.find("mimic") is not None:
            continue
        limit = joint.find("limit")
        if kind == "continuous":
            lower, upper = -math.pi, math.pi
        else:
            lower, upper = float(limit.get("lower", 0.0)), float(limit.get("upper", 0.0))
        velocity = float(limit.get("velocity", 0.0)) if limit is not None else 0.0
        if velocity <= 0.0:
            raise ValueError(f"joint {joint.get('name')} has no positive velocity limit")
        joints.append(JointInfo(joint.get("name"), kind, lower, upper, velocity))
    return joints


def move_duration(
    current: dict[str, float], target: dict[str, float], joints: dict[str, JointInfo], speed_scale: float
) -> float:
    """Time for a synchronous move so that no joint exceeds speed_scale x its velocity limit."""
    if not 0.0 < speed_scale <= 1.0:
        raise ValueError("speed_scale must be in (0, 1]")
    duration = 0.0
    for name, goal in target.items():
        start = current.get(name, goal)
        duration = max(duration, abs(goal - start) / (joints[name].velocity * speed_scale))
    return max(duration, MIN_MOVE_DURATION_S)
