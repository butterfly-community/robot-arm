"""StarArm model metadata and visualization asset catalog."""

from __future__ import annotations

import hashlib
import math
import mimetypes
import os
from pathlib import Path
from typing import Any
from xml.etree import ElementTree

MODEL_ID = "stararm-102-fl"
MODEL_REVISION = "stararm-102-fl-v1"
JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_KEY = "gripper"
GRIPPER_JOINT = "joint7_left"
NAMED_TARGETS = {
    "start": ((0.0, 0.0, math.radians(-5.0), 0.0, 0.0, 0.0), 0.0),
    "test": ((0.0, 0.0, math.radians(-20.0), 0.0, 0.0, 0.0), 0.0),
}
NAMED_TARGET_LABELS = {"start": "默认位", "test": "测试位"}
LINK_MATERIALS = {
    "base_link": {
        "color_rgb": [0.196, 0.298, 0.365],
        "metalness": 0.16,
        "roughness": 0.56,
    },
    **{
        name: {
            "color_rgb": [0.863, 0.906, 0.929],
            "metalness": 0.16,
            "roughness": 0.56,
        }
        for name in ("link1", "link2", "link3", "link4", "link5", "link6")
    },
    **{
        name: {
            "color_rgb": [0.145, 0.216, 0.275],
            "metalness": 0.42,
            "roughness": 0.48,
        }
        for name in ("link7_left", "link7_right")
    },
}


class ModelCatalog:
    def __init__(self) -> None:
        self._root = Path(
            os.environ.get("STARARM_MODEL_ASSETS", "/opt/robot-arm/model")
        )
        self._files = sorted(
            str(path.relative_to(self._root))
            for path in self._root.rglob("*")
            if path.is_file()
        )
        self._root_path = next(
            (name for name in self._files if name.endswith((".urdf", ".xacro"))),
            None,
        )
        if self._root_path is None:
            raise RuntimeError(f"模型目录中没有 URDF：{self._root}")
        description = ElementTree.fromstring(
            (self._root / self._root_path).read_text(encoding="utf-8")
        )
        self._joint_bounds = {
            joint.get("name"): (float(limit.get("lower")), float(limit.get("upper")))
            for joint in description.findall("joint")
            if joint.get("name") in JOINTS
            and (limit := joint.find("limit")) is not None
        }
        missing = [name for name in JOINTS if name not in self._joint_bounds]
        if missing:
            raise RuntimeError(f"URDF 缺少主动关节范围：{missing}")
        digest = hashlib.sha256()
        for relative in self._files:
            digest.update(relative.encode())
            digest.update((self._root / relative).read_bytes())
        self.manifest_hash = digest.hexdigest()

    def model_info(self) -> dict[str, Any]:
        return {
            "schema_version": 3,
            "model_id": MODEL_ID,
            "model_revision": MODEL_REVISION,
            "display_name": "StarArm-102",
            "base_frame": "base_link",
            "tcp_frame": "link6",
            "joints": [
                {
                    "key": name,
                    "label": f"J{index + 1}",
                    "unit": "rad",
                    "minimum": self._joint_bounds[name][0],
                    "maximum": self._joint_bounds[name][1],
                }
                for index, name in enumerate(JOINTS)
            ],
            "tool_actuators": [
                {
                    "key": GRIPPER_KEY,
                    "label": "夹爪",
                    "unit": "rad",
                    "minimum": 0.0,
                    "maximum": math.radians(90.0),
                    "visualization_joint_key": GRIPPER_JOINT,
                }
            ],
            "named_targets": [
                {
                    "key": key,
                    "label": NAMED_TARGET_LABELS[key],
                    "joint_positions_rad": dict(
                        zip(JOINTS, NAMED_TARGETS[key][0], strict=True)
                    ),
                    "actuator_positions_rad": {GRIPPER_KEY: NAMED_TARGETS[key][1]},
                }
                for key in ("start", "test")
            ],
            "motion_options": [
                {
                    "key": f"{kind}_scaling",
                    "label": f"{label}倍率",
                    "unit": "ratio",
                    "minimum": 0.0,
                    "maximum": 1.0,
                    "required": False,
                }
                for kind, label in (("velocity", "速度"), ("acceleration", "加速度"))
            ],
            "diagnostics": [],
            "visualization": {
                "manifest_hash": self.manifest_hash,
                "root_path": self._root_path,
                "files": self._files,
                "link_materials": LINK_MATERIALS,
            },
        }

    def asset_response(self, request: dict[str, Any]) -> dict[str, Any]:
        relative = Path(str(request.get("relative_path", "")))
        response = {
            "schema_version": 3,
            "request_id": str(request.get("request_id", "")),
            "model_revision": MODEL_REVISION,
            "manifest_hash": self.manifest_hash,
            "relative_path": str(relative),
            "mime_type": None,
            "content_hash": None,
            "content": None,
            "original_error": None,
        }
        if (
            request.get("model_revision") != MODEL_REVISION
            or request.get("manifest_hash") != self.manifest_hash
            or relative.is_absolute()
            or ".." in relative.parts
        ):
            response["original_error"] = (
                "asset request does not match the current manifest"
            )
            return response
        try:
            content = (self._root / relative).read_bytes()
        except OSError as error:
            response["original_error"] = str(error)
            return response
        response.update(
            content=list(content),
            content_hash=hashlib.sha256(content).hexdigest(),
            mime_type=mimetypes.guess_type(relative.name)[0]
            or "application/octet-stream",
        )
        return response
