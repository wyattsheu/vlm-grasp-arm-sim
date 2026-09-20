#!/usr/bin/env python3
"""Build a traceable Robot 129 description without modifying vendor files."""

from __future__ import annotations

import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
VENDOR = ROOT / "robot/vendor/piper_description"
PKG = ROOT / "ros2_ws/src/robot129_description"


def fixed_link(robot: ET.Element, name: str) -> None:
    ET.SubElement(robot, "link", {"name": name})


def fixed_joint(robot: ET.Element, name: str, parent: str, child: str, xyz: str, rpy: str = "0 0 0") -> None:
    joint = ET.SubElement(robot, "joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})


def main() -> None:
    (PKG / "urdf").mkdir(parents=True, exist_ok=True)
    (PKG / "meshes").mkdir(parents=True, exist_ok=True)
    (PKG / "config").mkdir(parents=True, exist_ok=True)
    for mesh in sorted((VENDOR / "meshes").glob("*.STL")):
        shutil.copy2(mesh, PKG / "meshes" / mesh.name)

    tree = ET.parse(VENDOR / "urdf/piper_description.urdf")
    robot = tree.getroot()
    robot.set("name", "robot129")
    for mesh in robot.findall(".//mesh"):
        mesh.set("filename", mesh.get("filename", "").replace("package://piper_description/", "package://robot129_description/"))

    world = ET.Element("link", {"name": "world"})
    robot.insert(0, world)
    base_joint = ET.Element("joint", {"name": "world_to_base", "type": "fixed"})
    ET.SubElement(base_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(base_joint, "parent", {"link": "world"})
    ET.SubElement(base_joint, "child", {"link": "base_link"})
    robot.insert(1, base_joint)

    joint8 = robot.find("./joint[@name='joint8']")
    ET.SubElement(joint8, "mimic", {"joint": "joint7", "multiplier": "-1", "offset": "0"})

    for name in ("flange", "tool0", "tcp", "camera_mount", "camera_link", "camera_color_optical_frame"):
        fixed_link(robot, name)
    fixed_joint(robot, "link6_to_flange", "link6", "flange", "0 0 0")
    fixed_joint(robot, "flange_to_tool0", "flange", "tool0", "0 0 0")
    fixed_joint(robot, "gripper_to_tcp", "gripper_base", "tcp", "0 0 0.180")
    fixed_joint(robot, "gripper_to_camera_mount", "gripper_base", "camera_mount", "0.055 0 0.075")
    fixed_joint(robot, "camera_mount_to_camera_link", "camera_mount", "camera_link", "0 0 0")
    fixed_joint(robot, "camera_link_to_color_optical", "camera_link", "camera_color_optical_frame", "0 0 0", "-1.57079632679 0 -1.57079632679")

    control = ET.SubElement(robot, "ros2_control", {"name": "Robot129SimulationSystem", "type": "system"})
    hardware = ET.SubElement(control, "hardware")
    plugin = ET.SubElement(hardware, "plugin")
    plugin.text = "mock_components/GenericSystem"
    for index in range(1, 9):
        joint = ET.SubElement(control, "joint", {"name": f"joint{index}"})
        # joint8 follows joint7 through the URDF mimic relation.  ros2_control
        # must never expose a command interface for an activated mimic joint.
        if index != 8:
            ET.SubElement(joint, "command_interface", {"name": "position"})
        state = ET.SubElement(joint, "state_interface", {"name": "position"})
        initial = ET.SubElement(state, "param", {"name": "initial_value"})
        initial.text = "0.035" if index == 7 else "-0.035" if index == 8 else "0.0"
        ET.SubElement(joint, "state_interface", {"name": "velocity"})

    ET.indent(tree, space="  ")
    urdf = PKG / "urdf/robot129.urdf"
    tree.write(urdf, encoding="utf-8", xml_declaration=True)
    shutil.copy2(urdf, PKG / "urdf/robot129.urdf.xacro")

    (PKG / "package.xml").write_text("""<?xml version=\"1.0\"?>
<package format=\"3\"><name>robot129_description</name><version>0.1.0</version>
<description>Simulation-only Robot 129 Piper description.</description>
<maintainer email=\"wyattsheu@example.com\">Robot 129</maintainer><license>Apache-2.0</license>
<buildtool_depend>ament_cmake</buildtool_depend><exec_depend>xacro</exec_depend>
<export><build_type>ament_cmake</build_type></export></package>
""", encoding="utf-8")
    (PKG / "CMakeLists.txt").write_text("""cmake_minimum_required(VERSION 3.8)
project(robot129_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes config DESTINATION share/${PROJECT_NAME})
ament_package()
""", encoding="utf-8")
    (PKG / "config/nominal_frames.yaml").write_text("""status: SIMULATION_NOMINAL_UNVERIFIED_ON_HARDWARE
tcp_parent: gripper_base
tcp_xyz_m: [0.0, 0.0, 0.180]
camera_mount_parent: gripper_base
camera_mount_xyz_m: [0.055, 0.0, 0.075]
camera_mount_rpy_rad: [0.0, 0.0, 0.0]
optical_rotation_rpy_rad: [-1.57079632679, 0.0, -1.57079632679]
warning: Do not use these nominal transforms for real Robot 129 deployment.
""", encoding="utf-8")

    links = [x.get("name") for x in robot.findall("link")]
    joints = []
    children = set()
    mesh_results = []
    for joint in robot.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        children.add(child)
        limit = joint.find("limit")
        axis = joint.find("axis")
        joints.append({"name": joint.get("name"), "type": joint.get("type"), "parent": parent, "child": child,
                       "axis": axis.get("xyz") if axis is not None else None,
                       "limit": limit.attrib if limit is not None else None})
    for mesh in robot.findall(".//mesh"):
        path = PKG / mesh.get("filename").split("/meshes/", 1)[1].joinpath() if False else PKG / "meshes" / Path(mesh.get("filename")).name
        mesh_results.append({"reference": mesh.get("filename"), "exists": path.is_file(), "bytes": path.stat().st_size if path.is_file() else 0})
    roots = sorted(set(links) - children)
    report = {"status": "PASS" if roots == ["world"] and all(x["exists"] for x in mesh_results) else "FAIL",
              "source_sha256": hashlib.sha256((VENDOR / "urdf/piper_description.urdf").read_bytes()).hexdigest(),
              "links": links, "joints": joints, "roots": roots, "mesh_results": mesh_results,
              "hardware_revision": "UNVERIFIED", "nominal_frames": "SIMULATION_ONLY"}
    out = ROOT / "out/lesson_03"
    out.mkdir(parents=True, exist_ok=True)
    (out / "urdf_validation.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"{report['status']}: {len(links)} links, {len(joints)} joints, {len(mesh_results)} mesh references, root={roots}")


if __name__ == "__main__":
    main()
