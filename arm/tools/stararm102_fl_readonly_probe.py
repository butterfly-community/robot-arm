#!/usr/bin/env python3
"""Strictly read-only Star Arm 102-FL identification and feedback probe.

The production Stararm102FL and lerobot_motor_starai connect() paths are
intentionally not used because they change torque and reset multi-turn state.
This tool opens the official FashionStar port directly and exposes only open,
ping, Present_Position and a non-torque-changing close.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Protocol

JOINT_IDS = {
    "shoulder_pan": 0,
    "shoulder_lift": 1,
    "elbow_flex": 2,
    "wrist_flex": 3,
    "wrist_yaw": 4,
    "wrist_roll": 5,
    "gripper": 6,
}
PRESENT_POSITION = "Present_Position"


class MotorBus(Protocol):
    def connect(self) -> None: ...

    def ping(self, motor_id: int) -> bool: ...

    def sync_read(
        self, register: str, motor_names: list[str], *, normalize: bool
    ) -> dict[str, float | None]: ...

    def disconnect(self, *, disable_torque: bool) -> None: ...


@dataclass(frozen=True)
class FeedbackSample:
    monotonic_ns: int
    positions_degrees: dict[str, float]


@dataclass(frozen=True)
class ProbeResult:
    schema_version: int
    hardware_variant: str
    requested_device: str
    resolved_device: str
    stable_device: bool
    baudrate: int
    backend: str
    ping: dict[str, bool]
    samples: list[FeedbackSample]
    valid: bool


class ReadOnlyProbe:
    """Capability-limited facade: there is no write or torque method."""

    def __init__(self, bus: MotorBus):
        self.__bus = bus

    def run(
        self,
        *,
        requested_device: str,
        resolved_device: str,
        stable_device: bool,
        baudrate: int,
        cycles: int,
        interval_seconds: float,
    ) -> ProbeResult:
        ping: dict[str, bool] = {}
        samples: list[FeedbackSample] = []
        self.__bus.connect()
        try:
            ping = {
                name: bool(self.__bus.ping(motor_id))
                for name, motor_id in JOINT_IDS.items()
            }
            if not all(ping.values()):
                return self._result(
                    requested_device,
                    resolved_device,
                    stable_device,
                    baudrate,
                    ping,
                    samples,
                    False,
                )
            for cycle in range(cycles):
                positions = self.__bus.sync_read(
                    PRESENT_POSITION, list(JOINT_IDS), normalize=False
                )
                parsed = parse_positions(positions)
                if parsed is None:
                    return self._result(
                        requested_device,
                        resolved_device,
                        stable_device,
                        baudrate,
                        ping,
                        samples,
                        False,
                    )
                samples.append(FeedbackSample(time.monotonic_ns(), parsed))
                if cycle + 1 < cycles:
                    time.sleep(interval_seconds)
        finally:
            # False is essential: closing a read-only probe must not release
            # torque or otherwise change actuator state.
            self.__bus.disconnect(disable_torque=False)
        return self._result(
            requested_device,
            resolved_device,
            stable_device,
            baudrate,
            ping,
            samples,
            True,
        )

    @staticmethod
    def _result(
        requested_device: str,
        resolved_device: str,
        stable_device: bool,
        baudrate: int,
        ping: dict[str, bool],
        samples: list[FeedbackSample],
        valid: bool,
    ) -> ProbeResult:
        return ProbeResult(
            schema_version=1,
            hardware_variant="Star Arm 102-FL",
            requested_device=requested_device,
            resolved_device=resolved_device,
            stable_device=stable_device,
            baudrate=baudrate,
            backend="fashionstar_uart_sdk",
            ping=ping,
            samples=samples,
            valid=valid,
        )


def parse_positions(values: dict[str, float | None]) -> dict[str, float] | None:
    if set(values) != set(JOINT_IDS):
        return None
    parsed: dict[str, float] = {}
    for name in JOINT_IDS:
        value = values[name]
        if value is None:
            return None
        number = float(value)
        if not math.isfinite(number):
            return None
        parsed[name] = number
    return parsed


def make_bus(port: str, baudrate: int) -> MotorBus:
    try:
        from fashionstar_uart_sdk.uart_pocket_handler import (  # type: ignore[import-not-found]
            PortHandler,
        )
    except ImportError as error:
        raise SystemExit(
            "缺少 fashionstar_uart_sdk；请在独立 Python 环境安装 "
            "fashionstar-uart-sdk==1.3.12"
        ) from error

    class ReadOnlyFashionStarBus:
        def __init__(self) -> None:
            self.port_handler = PortHandler(port, baudrate)

        def connect(self) -> None:
            if self.port_handler.is_open:
                raise RuntimeError("Star Arm bus is already connected")
            self.port_handler.openPort()
            if not self.port_handler.is_open:
                raise RuntimeError(f"failed to open Star Arm serial port: {port}")

        def ping(self, motor_id: int) -> bool:
            return bool(self.port_handler.ping(motor_id))

        def sync_read(
            self, register: str, motor_names: list[str], *, normalize: bool
        ) -> dict[str, float | None]:
            if register != PRESENT_POSITION or normalize:
                raise ValueError("read-only probe only permits raw Present_Position")
            ids = {name: JOINT_IDS[name] for name in motor_names}
            values = self.port_handler.read_positions(ids)
            return {
                name: None if values.get(name) is None else float(values[name])
                for name in motor_names
            }

        def disconnect(self, *, disable_torque: bool) -> None:
            if disable_torque:
                raise ValueError("read-only probe cannot change torque")
            if self.port_handler.is_open:
                self.port_handler.closePort()

    return ReadOnlyFashionStarBus()


def resolve_device(value: str) -> tuple[str, bool]:
    requested = Path(value)
    resolved = str(requested.resolve(strict=True))
    stable = str(requested).startswith("/dev/serial/by-id/")
    return resolved, stable


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        required=True,
        help="UC-01 stable path, preferably /dev/serial/by-id/...",
    )
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--interval-ms", type=float, default=50.0)
    parser.add_argument(
        "--allow-unstable-device",
        action="store_true",
        help="Allow /dev/ttyUSB* only for initial discovery; result remains marked unstable",
    )
    args = parser.parse_args()
    if args.baudrate <= 0 or args.cycles <= 0 or args.interval_ms < 0:
        parser.error("baudrate/cycles must be positive and interval-ms non-negative")
    return args


def main() -> int:
    args = parse_args()
    resolved_device, stable_device = resolve_device(args.port)
    if not stable_device and not args.allow_unstable_device:
        raise SystemExit(
            "拒绝不稳定设备名；请使用 /dev/serial/by-id/...，或仅在发现阶段显式添加 "
            "--allow-unstable-device"
        )
    result = ReadOnlyProbe(make_bus(resolved_device, args.baudrate)).run(
        requested_device=args.port,
        resolved_device=resolved_device,
        stable_device=stable_device,
        baudrate=args.baudrate,
        cycles=args.cycles,
        interval_seconds=args.interval_ms / 1000.0,
    )
    print(json.dumps(asdict(result), ensure_ascii=False, indent=2))
    return 0 if result.valid else 2


if __name__ == "__main__":
    raise SystemExit(main())
