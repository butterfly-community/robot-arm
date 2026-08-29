"""Transport-free StarArm-102 relative TCP target construction."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

from transforms3d.quaternions import axangle2quat, qmult, quat2mat

PIVOT_TO_TCP_M = (0.0, 0.0, 0.07313)
CONFIG_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class MotionConfig:
    schema_version: int = CONFIG_SCHEMA_VERSION
    config_version: int = 1
    control_mode: str = "relative"


def load_motion_config(path: Path) -> MotionConfig:
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return MotionConfig()
    config = MotionConfig(**raw)
    if config.schema_version != CONFIG_SCHEMA_VERSION:
        raise ValueError(f"unsupported motion config version {config.schema_version}")
    if config.control_mode not in {"relative", "manual"}:
        raise ValueError(f"unknown control mode {config.control_mode}")
    return config


def save_motion_config(path: Path, config: MotionConfig) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(asdict(config), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


@dataclass(frozen=True)
class Pose:
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]


def session_is_frozen(frozen_session: int | None, current_session: int | None) -> bool:
    return frozen_session is not None and frozen_session == current_session


def controller_sync_required(previous_source: str | None, source: str) -> bool:
    """Synchronize the first state and newly acquired hardware feedback."""
    return previous_source is None or (
        previous_source != source and source == "hardware"
    )


def frozen_session_after_servo_status(
    code: int, current_session: int | None, frozen_session: int | None
) -> int | None:
    if code == 6 and current_session is not None:
        return current_session
    return frozen_session


def tool_action_transition(previous: float | None, current: float) -> float | None:
    if previous is None or previous == current:
        return None
    return current


def tool_position_rad(value: float) -> float:
    return math.radians(90.0 * (1.0 - value))


def merge_controller_command(
    names: list[str],
    positions: list[float],
    joint_names: tuple[str, ...],
    actuator_name: str,
    current_joints: list[float],
    current_actuator: float,
) -> tuple[list[float], float] | None:
    """Merge independent controller outputs into one complete current command."""
    if len(names) != len(positions) or len(current_joints) != len(joint_names):
        return None
    indices = {name: index for index, name in enumerate(names)}
    if any(name not in indices for name in (*joint_names, actuator_name)):
        return None
    joints = [
        positions[indices[name]]
        if math.isfinite(positions[indices[name]])
        else current_joints[index]
        for index, name in enumerate(joint_names)
    ]
    actuator_value = positions[indices[actuator_name]]
    actuator = actuator_value if math.isfinite(actuator_value) else current_actuator
    if not all(math.isfinite(value) for value in (*joints, actuator)):
        return None
    return joints, actuator


def target_pose(
    anchor: Pose,
    translation_m: list[float],
    front_pitch_rad: float,
    horizontal_arc_rad: float,
) -> Pose:
    anchor_rotation = _wxyz(anchor.orientation_xyzw)
    pitch = axangle2quat((1.0, 0.0, 0.0), front_pitch_rad)
    turn = axangle2quat((0.0, 0.0, 1.0), horizontal_arc_rad)
    target_rotation = qmult(qmult(turn, anchor_rotation), pitch)
    anchor_offset = quat2mat(anchor_rotation).dot(PIVOT_TO_TCP_M)
    target_offset = quat2mat(target_rotation).dot(PIVOT_TO_TCP_M)
    target_position = tuple(
        anchor.position_m[index]
        + float(translation_m[index])
        + target_offset[index]
        - anchor_offset[index]
        for index in range(3)
    )
    return Pose(target_position, _xyzw(target_rotation))


def _wxyz(
    value: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return value[3], value[0], value[1], value[2]


def _xyzw(
    value: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    return float(value[1]), float(value[2]), float(value[3]), float(value[0])
