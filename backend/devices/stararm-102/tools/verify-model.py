#!/usr/bin/env python3
"""Verify the pinned vendor model before and after project patches."""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path


VENDOR_LIMITS = {
    "joint1": (-2.27, 2.27),
    "joint2": (0.0, 3.14),
    "joint3": (-3.14, 0.0),
    "joint4": (-1.57, 2.2),
    "joint5": (-1.2, 1.2),
    "joint6": (-3.14, 3.14),
    "joint7_left": (0.0, 1.57),
    "joint7_right": (-1.57, 0.0),
}

PATCHED_LIMITS = {
    "joint1": (-math.radians(110), math.radians(110)),
    "joint2": (0.0, math.radians(180)),
    "joint3": (-math.radians(270), 0.0),
    "joint4": (-math.radians(90), math.radians(90)),
    "joint5": (-math.radians(65), math.radians(65)),
    "joint6": (-math.radians(150), math.radians(150)),
    "joint7_left": (0.0, 1.570796327),
    "joint7_right": (-1.570796327, 0.0),
}


def vector(element: ET.Element | None, attribute: str) -> tuple[float, ...]:
    if element is None:
        raise ValueError(f"missing element for {attribute}")
    return tuple(float(value) for value in element.attrib[attribute].split())


def assert_close(actual: float, expected: float, label: str) -> None:
    if not math.isclose(actual, expected, abs_tol=1e-9):
        raise ValueError(f"{label}: expected {expected}, got {actual}")


def verify(root: Path, mode: str) -> None:
    urdf = (
        root / "ROS2_HUMBLE/src/stararm102_description/urdf/stararm102_description.urdf"
    )
    joints = {
        joint.attrib["name"]: joint
        for joint in ET.parse(urdf).getroot().findall("joint")
    }
    limits = VENDOR_LIMITS if mode == "vendor" else PATCHED_LIMITS
    for name, (expected_lower, expected_upper) in limits.items():
        joint = joints.get(name)
        if joint is None:
            raise ValueError(f"missing {name}")
        if vector(joint.find("axis"), "xyz") != (0.0, 0.0, -1.0):
            raise ValueError(f"{name}: expected the pinned vendor axis 0 0 -1")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"{name}: missing limit")
        assert_close(float(limit.attrib["lower"]), expected_lower, f"{name} lower")
        assert_close(float(limit.attrib["upper"]), expected_upper, f"{name} upper")
        if mode == "patched":
            assert_close(
                float(limit.attrib["velocity"]), math.tau * 34 / 60, f"{name} velocity"
            )

    right = joints["joint7_right"].find("mimic")
    if right is None or right.attrib != {"joint": "joint7_left", "multiplier": "-1"}:
        raise ValueError("joint7_right: expected a -1 mimic of joint7_left")

    if mode == "patched":
        # Planning inherits the URDF velocity; a second YAML value can let MTC
        # advance while ros2_control is still limiting the physical command.
        planning = (
            root / "ROS2_HUMBLE/src/stararm102_moveit_config/config/joint_limits.yaml"
        )
        if any(
            "velocity" in line
            for line in planning.read_text().splitlines()
            if not line.lstrip().startswith("#") and "scaling" not in line
        ):
            raise ValueError("MoveIt YAML must inherit velocity limits from URDF")
        control = root / (
            "ROS2_HUMBLE/src/stararm102_moveit_config/config/"
            "stararm102_description.ros2_control.xacro"
        )
        controlled = {
            joint.attrib["name"]: joint
            for joint in ET.parse(control).getroot().findall(".//joint")
        }
        for name, (expected_lower, expected_upper) in PATCHED_LIMITS.items():
            if name == "joint7_right":
                continue
            joint = controlled.get(name)
            if joint is None:
                raise ValueError(f"ros2_control: missing {name}")
            command = joint.find("command_interface[@name='position']")
            if command is None:
                raise ValueError(
                    f"ros2_control {name}: missing position command interface"
                )
            parameters = {
                item.attrib["name"]: float(item.text)
                for item in command.findall("param")
            }
            assert_close(parameters["min"], expected_lower, f"ros2_control {name} min")
            assert_close(parameters["max"], expected_upper, f"ros2_control {name} max")

        tcp = joints.get("tcp_joint")
        if tcp is None or tcp.attrib.get("type") != "fixed":
            raise ValueError("patched model: missing fixed tcp_joint")
        if tcp.find("parent").attrib.get("link") != "link6":
            raise ValueError("tcp_joint: expected link6 parent")
        if tcp.find("child").attrib.get("link") != "tcp_link":
            raise ValueError("tcp_joint: expected tcp_link child")
        if vector(tcp.find("origin"), "xyz") != (0.0, 0.0, 0.0):
            raise ValueError("tcp_joint: expected zero translation")
        if vector(tcp.find("origin"), "rpy") != (0.0, 0.0, 0.0):
            raise ValueError("tcp_joint: expected zero rotation")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("mode", choices=("vendor", "patched"))
    arguments = parser.parse_args()
    verify(arguments.root, arguments.mode)
    print(f"StarArm-102 {arguments.mode} model verified")


if __name__ == "__main__":
    main()
