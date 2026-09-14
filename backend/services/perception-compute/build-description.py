#!/usr/bin/env python3
"""Generate GraspGenX assets from a patched robot URDF and a device manifest."""

from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh

SERIALIZED_DECIMALS = 9


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate every GraspGenX gripper declared by a device manifest."
    )
    parser.add_argument("manifest", type=Path)
    parser.add_argument("vendor_root", type=Path)
    parser.add_argument("output_root", type=Path)
    parser.add_argument(
        "--graspgenx-root",
        type=Path,
        required=True,
    )
    return parser.parse_args()


def load_wizard(root: Path):
    script = root / "scripts" / "gripper_config_wizard.py"
    spec = importlib.util.spec_from_file_location(
        "graspgenx_gripper_config_wizard", script
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load GraspGenX wizard from {script}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def clean_numbers(value):
    if isinstance(value, float):
        return round(value, SERIALIZED_DECIMALS)
    if isinstance(value, list):
        return [clean_numbers(item) for item in value]
    if isinstance(value, dict):
        return {key: clean_numbers(item) for key, item in value.items()}
    return value


def radians(joints_degrees: dict[str, float]) -> dict[str, float]:
    return {
        joint: float(np.deg2rad(degrees)) for joint, degrees in joints_degrees.items()
    }


def source_mesh(filename: str, package_root: Path) -> Path:
    prefix = "package://"
    if filename.startswith(prefix):
        parts = Path(filename.removeprefix(prefix)).parts
        if len(parts) < 2:
            raise ValueError(f"invalid package mesh URI: {filename}")
        return package_root.joinpath(*parts[1:])
    path = Path(filename)
    return path if path.is_absolute() else package_root / path


def copy_link_geometry(
    source: ET.Element,
    name: str,
    package_root: Path,
    output_dir: Path,
) -> ET.Element:
    link = ET.Element("link", {"name": name})
    for tag in ("visual", "collision"):
        for child in source.findall(tag):
            geometry = copy.deepcopy(child)
            mesh = geometry.find("geometry/mesh")
            if mesh is not None and mesh.get("filename"):
                source_path = source_mesh(mesh.get("filename", ""), package_root)
                relative = Path("meshes") / source_path.name
                target = output_dir / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source_path, target)
                mesh.set("filename", relative.as_posix())
            link.append(geometry)
    return link


def required(root: ET.Element, tag: str, name: str) -> ET.Element:
    element = root.find(f"{tag}[@name='{name}']")
    if element is None:
        raise ValueError(f"URDF has no {tag} named {name}")
    return element


def fixed_joint_between(
    root: ET.Element, parent_name: str, child_name: str
) -> ET.Element:
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if (
            parent is not None
            and child is not None
            and parent.get("link") == parent_name
            and child.get("link") == child_name
        ):
            if joint.get("type") != "fixed":
                raise ValueError(f"{parent_name} -> {child_name} must be a fixed joint")
            return joint
    raise ValueError(f"URDF has no direct joint from {parent_name} to {child_name}")


def canonical_joint(descriptor: dict, source_base_name: str) -> ET.Element:
    transform = descriptor["canonical_base_to_source_base"]
    xyz = transform["xyz_m"]
    rpy = np.deg2rad(transform["rpy_degrees"])
    joint = ET.Element("joint", {"name": "canonical_to_source_base", "type": "fixed"})
    ET.SubElement(
        joint,
        "origin",
        {
            "xyz": " ".join(f"{float(value):.9f}" for value in xyz),
            "rpy": " ".join(f"{float(value):.9f}" for value in rpy),
        },
    )
    ET.SubElement(joint, "parent", {"link": "gripper_base"})
    ET.SubElement(joint, "child", {"link": source_base_name})
    return joint


def write_canonical_urdf(
    descriptor: dict,
    vendor_root: Path,
    output_dir: Path,
) -> None:
    source_urdf = vendor_root / descriptor["source_urdf"]
    package_root = vendor_root / descriptor["source_package_root"]
    source_robot = ET.parse(source_urdf).getroot()
    source_base_name = descriptor["source_base_link"]
    tcp_link_name = descriptor["tcp_link"]
    source_base = required(source_robot, "link", source_base_name)
    source_tcp = required(source_robot, "link", tcp_link_name)
    source_tcp_joint = fixed_joint_between(
        source_robot, source_base_name, tcp_link_name
    )

    robot = ET.Element("robot", {"name": f"{descriptor['id']}-gripper"})
    robot.append(ET.Element("link", {"name": "gripper_base"}))
    robot.append(
        copy_link_geometry(source_base, source_base_name, package_root, output_dir)
    )
    robot.append(canonical_joint(descriptor, source_base_name))
    robot.append(
        copy_link_geometry(source_tcp, tcp_link_name, package_root, output_dir)
    )
    fixed = copy.deepcopy(source_tcp_joint)
    fixed.set("name", "source_base_to_tcp")
    robot.append(fixed)

    for joint_name in descriptor["finger_joints"]:
        source_joint = required(source_robot, "joint", joint_name)
        child = source_joint.find("child")
        if child is None or not child.get("link"):
            raise ValueError(f"finger joint {joint_name} has no child link")
        child_name = child.get("link", "")
        robot.append(
            copy_link_geometry(
                required(source_robot, "link", child_name),
                child_name,
                package_root,
                output_dir,
            )
        )
        joint = copy.deepcopy(source_joint)
        parent = joint.find("parent")
        if parent is None or parent.get("link") != source_base_name:
            raise ValueError(
                f"finger joint {joint_name} is not attached to {source_base_name}"
            )
        robot.append(joint)

    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(
        output_dir / "gripper.urdf", encoding="utf-8", xml_declaration=True
    )


def build_descriptor(
    descriptor: dict,
    vendor_root: Path,
    output_root: Path,
    wizard,
) -> dict:
    output_dir = output_root / descriptor["id"]
    output_dir.mkdir(parents=True, exist_ok=True)
    write_canonical_urdf(descriptor, vendor_root, output_dir)
    robot = wizard.load_urdf(str(output_dir / "gripper.urdf"))
    open_state = radians(descriptor["drive_joint_work_open_degrees"])
    close_state = radians(descriptor["drive_joint_closed_degrees"])
    finger_geometries = wizard.detect_finger_geoms(robot, open_state)
    sweep_volume = descriptor["sweep_volume"]
    open_extents = sweep_volume["extents"]
    open_offset = sweep_volume["offset"]
    half_extents = sweep_volume["extents2"]
    half_offset = sweep_volume["offset2"]
    closing_axis = descriptor["closing_axis"]
    bbox_min, bbox_max = wizard.compute_gripper_bbox(
        robot, open_state, base_T=np.eye(4)
    )
    fingertip_depth = max(
        float(wizard.compute_geom_bbox(robot, name, base_T=np.eye(4))[1][2])
        for name in finger_geometries
    )
    tcp_transform = robot.get_transform(descriptor["tcp_link"], robot.base_link)

    config = clean_numbers(
        {
            "open": open_state,
            "close": close_state,
            # GraspMoE anchors the open fingertips on the object surface.
            # Measure the actual moving meshes with the upstream wizard;
            # neither a sweep-box centre nor its bound is a physical fingertip.
            # The rigid controller TCP remains independent of this depth.
            "fingertip": [*open_offset[:2], fingertip_depth],
            "tool_tcp_transform": tcp_transform.tolist(),
            "sweep_volume": {
                "extents": open_extents,
                "offset": open_offset,
                "extents2": half_extents,
                "offset2": half_offset,
            },
            "links": wizard.get_link_names(robot),
            "standoff": [0.0, open_extents[2] / 2.0],
            "symmetric": descriptor["symmetric"],
            "type": descriptor["type"],
            "bbox": [bbox_min, bbox_max],
            "base_rotation": np.eye(4).tolist(),
        }
    )
    (output_dir / "config.json").write_text(
        json.dumps(config, indent=2, ensure_ascii=False) + "\n"
    )
    wizard.export_merged_mesh(
        robot, open_state, str(output_dir / "vis_mesh.obj"), base_T=np.eye(4)
    )
    shutil.copy2(output_dir / "vis_mesh.obj", output_dir / "coll_mesh.obj")

    visual_mesh = trimesh.load(output_dir / "vis_mesh.obj", force="mesh")
    report = clean_numbers(
        {
            "id": descriptor["id"],
            "generator": "GraspGenX scripts/gripper_config_wizard.py",
            "base_link": robot.base_link,
            "tcp_link": descriptor["tcp_link"],
            "base_to_tcp": tcp_transform.tolist(),
            "finger_geometries": finger_geometries,
            "sampling_depth_source": "open moving-mesh front along canonical +Z",
            "sampling_depth_m": fingertip_depth,
            "closing_axis": closing_axis,
            "half_closing_axis": closing_axis,
            "sweep_volume_source": "device manifest reviewed with GraspGenX wizard",
            "mesh_bounds": visual_mesh.bounds.tolist(),
            "mesh_faces": len(visual_mesh.faces),
        }
    )
    (output_dir / "description-report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n"
    )
    return report


def main() -> None:
    args = arguments()
    manifest = json.loads(args.manifest.read_text())
    wizard = load_wizard(args.graspgenx_root.resolve())
    reports = [
        build_descriptor(
            descriptor,
            args.vendor_root.resolve(),
            args.output_root.resolve(),
            wizard,
        )
        for descriptor in manifest["descriptors"]
    ]
    print(json.dumps(reports, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
