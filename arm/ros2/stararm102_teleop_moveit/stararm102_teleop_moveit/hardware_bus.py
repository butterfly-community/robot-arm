"""Star Arm 102-FL bus adapter for the official FashionStar SDK.

This module uses only the official FashionStar SDK's port, ping, monitor read
and synchronized position write operations. MoveIt joint coordinates are the
raw FL bus angles for J1--J6; logical direction conversion cancels at the URDF
boundary.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol, Sequence


MOTOR_IDS = {
    "shoulder_pan": 0,
    "shoulder_lift": 1,
    "elbow_flex": 2,
    "wrist_flex": 3,
    "wrist_yaw": 4,
    "wrist_roll": 5,
}
ARM_MOTOR_NAMES = tuple(MOTOR_IDS)
GRIPPER_NAME = "gripper"
GRIPPER_ID = 6
PRODUCTION_MOTOR_IDS = {**MOTOR_IDS, GRIPPER_NAME: GRIPPER_ID}
# Exact lerobot_motor_starai 0.0.7 Goal_Position defaults.  They are repeated
# here only because this adapter deliberately bypasses that package's
# state-changing connect() path while preserving its normal command encoding.
UPSTREAM_MOTION_TIME_MS = 350
UPSTREAM_ACCELERATION_TIME_MS = 50
UPSTREAM_DECELERATION_TIME_MS = 50


@dataclass(frozen=True)
class GripperFeedback:
    position_rad: float
    power_w: float
    current_a: float
    temperature_c: float
    status: int


@dataclass(frozen=True)
class HardwareFeedback:
    arm_positions_rad: tuple[float, ...]
    gripper: GripperFeedback


class MotorBus(Protocol):
    def connect(self) -> None: ...
    def ping(self, motor_id: int) -> bool: ...
    def sync_read(
        self, register: str, motor_names: list[str], *, normalize: bool
    ) -> dict[str, object | None]: ...
    def sync_write(
        self, register: str, values: dict[str, float], *, normalize: bool
    ) -> None: ...
    def disconnect(self) -> None: ...


def make_bus(port: str, baudrate: int) -> MotorBus:
    from fashionstar_uart_sdk.uart_pocket_handler import (  # type: ignore[import-not-found]
        PortHandler,
        SyncPositionControlOptions,
    )

    class FashionStarBus:
        """Minimal use of the official SDK; no configuration command exists."""

        def __init__(self) -> None:
            self.port_handler = PortHandler(port, baudrate)

        def connect(self) -> None:
            if self.port_handler.is_open:
                raise RuntimeError("Star Arm 总线已经连接")
            self.port_handler.openPort()
            if not self.port_handler.is_open:
                raise RuntimeError(f"无法打开 Star Arm 串口：{port}")

        def ping(self, motor_id: int) -> bool:
            return bool(self.port_handler.ping(motor_id))

        def sync_read(
            self, register: str, motor_names: list[str], *, normalize: bool
        ) -> dict[str, object | None]:
            if register != "Monitor" or normalize:
                raise ValueError("真机只允许读取未归一化 Monitor")
            ids = {name: PRODUCTION_MOTOR_IDS[name] for name in motor_names}
            return self.port_handler.sync_read["Monitor"](ids, realtime=True)

        def sync_write(
            self, register: str, values: dict[str, float], *, normalize: bool
        ) -> None:
            if register != "Goal_Position" or normalize:
                raise ValueError("真机只允许写入未归一化 Goal_Position")
            commands = {
                name: SyncPositionControlOptions(
                    PRODUCTION_MOTOR_IDS[name],
                    int(round(float(value) * 10.0)),
                    UPSTREAM_MOTION_TIME_MS,
                    0,
                    UPSTREAM_ACCELERATION_TIME_MS,
                    UPSTREAM_DECELERATION_TIME_MS,
                )
                for name, value in values.items()
            }
            self.port_handler.sync_write["Goal_Position"](commands)

        def disconnect(self) -> None:
            if self.port_handler.is_open:
                self.port_handler.closePort()

    return FashionStarBus()


class HardwareBus:
    """Typed facade around the official FashionStar SDK adapter."""

    def __init__(self, bus: MotorBus):
        self.__bus = bus

    def connect(self) -> HardwareFeedback:
        self.__bus.connect()
        try:
            missing = [
                name
                for name, motor_id in PRODUCTION_MOTOR_IDS.items()
                if not self.__bus.ping(motor_id)
            ]
            if missing:
                raise RuntimeError(f"舵机无响应：{', '.join(missing)}")
            return self.read_feedback()
        except Exception:
            self.__bus.disconnect()
            raise

    def read_feedback(self) -> HardwareFeedback:
        values = self.__bus.sync_read(
            "Monitor", list(PRODUCTION_MOTOR_IDS), normalize=False
        )
        if set(values) != set(PRODUCTION_MOTOR_IDS):
            raise RuntimeError("Monitor 返回的舵机集合不完整")
        positions_deg: dict[str, float] = {}
        for name, value in values.items():
            position = None if value is None else getattr(value, "current_position", None)
            if position is None or not math.isfinite(float(position)):
                raise RuntimeError(f"{name} 的 Monitor 位置无效")
            positions_deg[name] = float(position)
        gripper = values[GRIPPER_NAME]
        assert gripper is not None
        raw_gripper = [
            getattr(gripper, "power", None),
            getattr(gripper, "current", None),
            getattr(gripper, "temperature", None),
        ]
        if any(
            value is None or not math.isfinite(float(value))
            for value in raw_gripper
        ):
            raise RuntimeError("夹爪 Monitor 功率、电流或温度无效")
        status = getattr(gripper, "status", None)
        if (
            not isinstance(status, int)
            or isinstance(status, bool)
            or not 0 <= status <= 0xFF
        ):
            raise RuntimeError("夹爪 Monitor 状态无效")
        return HardwareFeedback(
            arm_positions_rad=tuple(
                math.radians(positions_deg[name]) for name in ARM_MOTOR_NAMES
            ),
            # The manufacturer ROS driver maps joint7_left to the negated ID 6
            # angle.  Monitor power/current are mW/mA in SDK 1.3.12.
            gripper=GripperFeedback(
                position_rad=math.radians(-positions_deg[GRIPPER_NAME]),
                power_w=float(raw_gripper[0]) / 1000.0,
                current_a=float(raw_gripper[1]) / 1000.0,
                temperature_c=float(raw_gripper[2]),
                status=status,
            ),
        )

    def write_positions(
        self,
        positions_rad: Sequence[float],
        gripper_position_rad: float | None = None,
    ) -> None:
        if len(positions_rad) != 6 or any(
            not math.isfinite(value) for value in positions_rad
        ):
            raise ValueError("机械臂目标必须是六个有限弧度值")
        if gripper_position_rad is not None and not math.isfinite(gripper_position_rad):
            raise ValueError("夹爪目标必须是有限弧度值")
        targets = {
            name: math.degrees(float(positions_rad[index]))
            for index, name in enumerate(ARM_MOTOR_NAMES)
        }
        if gripper_position_rad is not None:
            targets[GRIPPER_NAME] = -math.degrees(gripper_position_rad)
        self.__bus.sync_write(
            "Goal_Position",
            targets,
            normalize=False,
        )

    def disconnect(self) -> None:
        self.__bus.disconnect()
