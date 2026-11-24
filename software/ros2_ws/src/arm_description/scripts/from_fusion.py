#!/usr/bin/env python3
"""Fusion 360 URDF export → urdf/arm_model.xacro (docs/decisions/0026).

    python3 scripts/from_fusion.py ../../../cad_export/fusion_urdf/URDF.urdf

Fusion names its links/joints after the components («LINK-1-1», «BASE-v7_Revolute-5») and exports the gripper
as it is drawn: four-bar linkages, i.e. closed loops, which URDF (a tree) cannot hold. This script:
  * renames to the names the rest of the system uses (base_link, link_1…link_6, joint_1…joint_6),
  * keeps the arm chain as exported (zero = the exported, vertical pose; axes and limits from Fusion),
  * freezes the gripper at its exported pose: every part fixed to gripper_base, placed by its mesh (Fusion's
    joints of the loops are wrong, see gripper_poses); the gripper's own joints come later, with mimic,
  * replaces Fusion's placeholder effort/velocity (1e6) with the arm's real velocity limit,
  * writes meshes as package://arm_description/meshes/<file>.

Run it again after every new export; never edit arm_model.xacro by hand.
"""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

LINKS = {"BASE-v7": "base_link", "LINK-1-1": "link_1", "LINK-2-1": "link_2", "LINK-3": "link_3",
         "LINK-4": "link_4", "LINK-5": "link_5", "LINK-6": "link_6", "GRIPPER-BASE": "gripper_base"}
ARM = {"BASE-v7_Revolute-5": "joint_1", "LINK-1-1_Revolute-1": "joint_2", "LINK-2-1_Revolute-16": "joint_3",
       "LINK-3_Revolute-3": "joint_4", "LINK-4_Revolute-4": "joint_5", "LINK-5_Revolute-6": "joint_6"}
FLANGE_TO_GRIPPER = "LINK-6_Rigid-7"
VELOCITY = "${joint_velocity}"
EFFORT = "${joint_effort}"
EPS = 1e-9


def name_of(fusion: str) -> str:
    return LINKS.get(fusion, fusion.lower().replace("-", "_"))


def clean(value: float) -> float:
    value = 0.0 if abs(value) < EPS else value
    return round(value, 9) + 0.0


def numbers(values) -> str:
    return " ".join(f"{clean(v):.9g}" for v in values)


def rotation(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rpy_of(m) -> tuple:
    if abs(m[2, 0]) < 1 - 1e-12:
        p = -math.asin(m[2, 0])
        return math.atan2(m[2, 1], m[2, 2]), p, math.atan2(m[1, 0], m[0, 0])
    p = math.copysign(math.pi / 2, -m[2, 0])             # gimbal lock: roll and yaw share one axis
    return 0.0, p, math.atan2(-m[0, 1], m[1, 1])


def transform(origin) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rotation(*[float(v) for v in origin.get("rpy").split()])
    t[:3, 3] = [float(v) for v in origin.get("xyz").split()]
    return t


def origin_xml(t) -> str:
    return f'<origin xyz="{numbers(t[:3, 3])}" rpy="{numbers(rpy_of(t[:3, :3]))}"/>'


def link_xml(link) -> str:
    name = name_of(link.get("name"))
    out = [f'  <link name="{name}">']
    inertial = link.find("inertial")
    if inertial is not None:
        i = inertial.find("inertia")
        out += ["    <inertial>", f"      {origin_xml(transform(inertial.find('origin')))}",
                f'      <mass value="{clean(float(inertial.find("mass").get("value"))):.9g}"/>',
                "      <inertia " + " ".join(f'{k}="{clean(float(i.get(k))):.9g}"'
                                             for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")) + "/>",
                "    </inertial>"]
    for kind in ("visual", "collision"):
        element = link.find(kind)
        if element is None:
            continue
        mesh = element.find("geometry/mesh")
        file = Path(mesh.get("filename")).name
        out += [f"    <{kind}>", f"      {origin_xml(transform(element.find('origin')))}",
                f'      <geometry><mesh filename="package://arm_description/meshes/{file}" '
                f'scale="{mesh.get("scale", "1 1 1")}"/></geometry>']
        if kind == "visual":
            out.append(f'      <material name="{"arm_orange" if name.startswith("link_") else "arm_grey"}"/>')
        out.append(f"    </{kind}>")
    out.append("  </link>")
    return "\n".join(out)


def arm_joint_xml(joint) -> str:
    axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
    axis = axis / np.linalg.norm(axis)
    limit = joint.find("limit")
    return "\n".join([
        f'  <joint name="{ARM[joint.get("name")]}" type="revolute">',
        f'    <parent link="{name_of(joint.find("parent").get("link"))}"/>',
        f'    <child link="{name_of(joint.find("child").get("link"))}"/>',
        f"    {origin_xml(transform(joint.find('origin')))}",
        f'    <axis xyz="{numbers(np.round(axis, 9))}"/>',
        f'    <limit lower="{clean(float(limit.get("lower"))):.9g}" upper="{clean(float(limit.get("upper"))):.9g}" '
        f'effort="{EFFORT}" velocity="{VELOCITY}"/>',
        "  </joint>"])


def fixed_xml(name, parent, child, t) -> str:
    return "\n".join([f'  <joint name="{name}" type="fixed">', f'    <parent link="{parent}"/>',
                      f'    <child link="{child}"/>', f"    {origin_xml(t)}", "  </joint>"])


def gripper_poses(robot) -> dict:
    """{part: 4×4 pose in GRIPPER-BASE} at the exported pose.

    Not from the joints of the linkage: Fusion writes the joints that close the loops wrongly (the two paths to
    a finger disagree by 34 and 159 mm in the export of 2026-11-24). The STL files of the gripper parts share
    one coordinate frame (the parts joined to GRIPPER-BASE without a loop sit exactly on it), so each part is
    placed where its mesh lands on the mesh of GRIPPER-BASE: pose = visual(base) · visual(part)⁻¹.
    """
    visual = {link.get("name"): transform(link.find("visual/origin")) for link in robot.findall("link")}
    base = visual["GRIPPER-BASE"]
    return {name: base @ np.linalg.inv(v) for name, v in visual.items() if name not in LINKS or name == "GRIPPER-BASE"}


def loop_gaps(joints, poses) -> dict:
    """{joint: mismatch in m} for the joints of the gripper: how far Fusion's joint is from the mesh placement
    (for the record; the joints are not used)."""
    gaps = {}
    for j in joints:
        parent, child = j.find("parent").get("link"), j.find("child").get("link")
        if j.get("name") in ARM or j.get("name") == FLANGE_TO_GRIPPER:
            continue
        gaps[j.get("name")] = float(np.linalg.norm((poses[parent] @ transform(j.find("origin")))[:3, 3]
                                                   - poses[child][:3, 3]))
    return gaps


def convert(source: Path) -> str:
    robot = ET.parse(source).getroot()
    joints = robot.findall("joint")
    by_name = {j.get("name"): j for j in joints}
    missing = [n for n in [*ARM, FLANGE_TO_GRIPPER] if n not in by_name]
    if missing:
        raise SystemExit(f"not in the export: {missing} (names changed in Fusion? update ARM in this script)")
    poses = gripper_poses(robot)
    parts = [link.get("name") for link in robot.findall("link")]
    for name, gap in loop_gaps(joints, poses).items():
        if gap > 1e-4:
            print(f"note: Fusion joint {name} is {gap * 1000:.1f} mm off the meshes (not used)")

    out = ['<?xml version="1.0"?>',
           f"<!-- GENERATED by scripts/from_fusion.py from {source.name}: do not edit (docs/decisions/0026). -->",
           '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
           '<xacro:macro name="arm_model">']
    out += [link_xml(link) for link in robot.findall("link")]
    out += [arm_joint_xml(by_name[n]) for n in ARM]
    out.append(fixed_xml("link_6_to_gripper", "link_6", "gripper_base",
                         transform(by_name[FLANGE_TO_GRIPPER].find("origin"))))
    for part in parts:
        if part in LINKS:
            continue
        out.append(fixed_xml(f"gripper_to_{name_of(part)}", "gripper_base", name_of(part), poses[part]))
    out += ["</xacro:macro>", "</robot>", ""]
    return "\n".join(out)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        raise SystemExit(__doc__)
    target = Path(__file__).resolve().parents[1] / "urdf" / "arm_model.xacro"
    target.write_text(convert(Path(argv[0])), encoding="utf-8")
    print(f"written {target}")


if __name__ == "__main__":
    main()
