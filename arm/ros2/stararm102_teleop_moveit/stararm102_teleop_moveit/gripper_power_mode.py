"""Ensure the manufacturer ID 6 power-protection mode-two profile."""

from __future__ import annotations

import argparse
import struct
import time
from collections.abc import Callable
from typing import Protocol


GRIPPER_ID = 6
BAUDRATE = 1_000_000
STALL_PROTECTION_DISABLED = 0
STALL_POWER_LIMIT_MW = 2_000
POWER_PROTECTION_MW = 4_000

_PUBLIC_PAYLOAD_SIZE = 33
_PUBLIC_RESPONSE_SIZE = 38
_READ_PUBLIC_PREFIX = bytes([0x12, 0x4C, 0x05, 0x01, GRIPPER_ID])
_WRITE_PUBLIC_HEADER = bytes([0x12, 0x4C, 0x06, 0x21])
_STALL_PROTECTION_OFFSET = 6
_STALL_POWER_LIMIT_OFFSET = 7
_POWER_PROTECTION_OFFSET = 15


class SerialTransport(Protocol):
    def reset_input_buffer(self) -> None: ...
    def write(self, data: bytes) -> int: ...
    def read(self, size: int) -> bytes: ...


def _with_checksum(data: bytes) -> bytes:
    return data + bytes([sum(data) % 256])


def _read_public_parameters(transport: SerialTransport) -> bytearray:
    transport.reset_input_buffer()
    request = _with_checksum(_READ_PUBLIC_PREFIX)
    if transport.write(request) != len(request):
        raise RuntimeError("读取 ID 6 保护参数的请求未完整发送")
    response = transport.read(_PUBLIC_RESPONSE_SIZE)
    if len(response) != _PUBLIC_RESPONSE_SIZE:
        raise RuntimeError("读取 ID 6 保护参数的响应不完整")
    if sum(response[:-1]) % 256 != response[-1]:
        raise RuntimeError("ID 6 保护参数响应校验和错误")
    payload = bytearray(response[4:-1])
    if len(payload) != _PUBLIC_PAYLOAD_SIZE or payload[0] != GRIPPER_ID:
        raise RuntimeError("ID 6 保护参数响应格式错误")
    return payload


def _is_mode_two(payload: bytearray) -> bool:
    return (
        payload[_STALL_PROTECTION_OFFSET] == STALL_PROTECTION_DISABLED
        and struct.unpack_from("<H", payload, _STALL_POWER_LIMIT_OFFSET)[0]
        == STALL_POWER_LIMIT_MW
        and struct.unpack_from("<H", payload, _POWER_PROTECTION_OFFSET)[0]
        == POWER_PROTECTION_MW
    )


def ensure_gripper_power_mode_two(
    transport: SerialTransport,
    *,
    settle: Callable[[float], None] = time.sleep,
) -> bool:
    """Return True only when the three mode-two fields had to be changed."""

    payload = _read_public_parameters(transport)
    if _is_mode_two(payload):
        return False

    payload[_STALL_PROTECTION_OFFSET] = STALL_PROTECTION_DISABLED
    struct.pack_into("<H", payload, _STALL_POWER_LIMIT_OFFSET, STALL_POWER_LIMIT_MW)
    struct.pack_into("<H", payload, _POWER_PROTECTION_OFFSET, POWER_PROTECTION_MW)
    packet = _with_checksum(_WRITE_PUBLIC_HEADER + payload)
    if transport.write(packet) != len(packet):
        raise RuntimeError("ID 6 功率保护模式二配置未完整发送")
    settle(0.2)
    if not _is_mode_two(_read_public_parameters(transport)):
        raise RuntimeError("ID 6 功率保护模式二写入后校验失败")
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="配置并校验夹爪 ID 6 功率保护模式二")
    parser.add_argument("--port", required=True)
    args = parser.parse_args()

    import serial  # type: ignore[import-not-found]

    transport = serial.Serial(args.port, BAUDRATE, timeout=0.5)
    try:
        changed = ensure_gripper_power_mode_two(transport)
    finally:
        transport.close()
    action = "已写入并校验" if changed else "已读取确认"
    print(
        f"夹爪 ID 6 功率保护模式二{action}："
        f"A={POWER_PROTECTION_MW} mW，B={STALL_POWER_LIMIT_MW} mW"
    )


if __name__ == "__main__":
    main()
