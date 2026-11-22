"""MoveIt goals for Arm Studio, without ROS (docs/decisions/0025).

The widgets describe a goal as a plain dict; ros_bridge turns it into a moveit_msgs MoveGroup goal:
    {"kind": "joints", "joints": {joint: rad}, "speed": 0..1}           e.g. a named pose of the SRDF
    {"kind": "position", "xyz": (x, y, z), "speed": 0..1}              tool0 at a point (m, frame world),
                                                                       any orientation
"""

import xml.etree.ElementTree as ET

GROUP = "arm"
TOOL_LINK = "tool0"
FRAME = "world"
JOINT_TOLERANCE_RAD = 0.01
POSITION_TOLERANCE_M = 0.005

# moveit_msgs/MoveItErrorCodes → what the user reads
ERRORS = {
    1: "ολοκληρώθηκε",
    -1: "δεν βρέθηκε διαδρομή (planning failed)",
    -2: "άκυρη διαδρομή",
    -3: "η διαδρομή ακυρώθηκε από αλλαγή στο περιβάλλον",
    -4: "ο controller δεν την εκτέλεσε (control failed)",
    -6: "λήξη χρόνου",
    -7: "διακόπηκε",
    -10: "η τωρινή θέση είναι σε σύγκρουση",
    -12: "ο στόχος είναι σε σύγκρουση",
    -14: "ο στόχος δεν επιτεύχθηκε",
    -16: "άκυρος στόχος",
    -17: "άκυρη κατάσταση βραχίονα",
    -21: "αποτυχία μετασχηματισμού συστήματος συντεταγμένων",
    -31: "δεν υπάρχει λύση αντίστροφης κινηματικής (εκτός εμβέλειας;)",
}


def named_poses(srdf: str, group: str = GROUP) -> dict:
    """{name: {joint: rad}} of the group_state entries of the SRDF, in file order."""
    root = ET.fromstring(srdf)
    return {state.get("name"): {j.get("name"): float(j.get("value")) for j in state.findall("joint")}
            for state in root.findall("group_state") if state.get("group") == group}


def joints_goal(joints: dict, speed: float) -> dict:
    return {"kind": "joints", "joints": {n: float(v) for n, v in joints.items()}, "speed": clamp_speed(speed)}


def position_goal(x: float, y: float, z: float, speed: float) -> dict:
    return {"kind": "position", "xyz": (float(x), float(y), float(z)), "speed": clamp_speed(speed)}


def clamp_speed(speed: float) -> float:
    """MoveIt scales the limits of joint_limits.yaml by this: 0 means «default», so at least 1 %."""
    return min(1.0, max(0.01, float(speed)))


def error_text(code: int) -> str:
    return ERRORS.get(code, f"κωδικός MoveIt {code}")
