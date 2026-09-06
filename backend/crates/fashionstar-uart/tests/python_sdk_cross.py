#!/usr/bin/env python3
"""Bidirectional socat test against fashionstar_uart_sdk 1.3.12."""

from __future__ import annotations

import argparse
import array
import fcntl
import os
import pathlib
import struct
import subprocess
import sys
import tempfile
import threading
import time
import types


IDS = tuple(range(7))
CODE_PING = 1
CODE_SET_MTURN_BY_INTERVAL = 14
CODE_QUERY_MONITOR = 22
CODE_SYNC_COMMAND = 25
MOTION_TIME_MS = 350
ACCELERATION_TIME_MS = 50
DECELERATION_TIME_MS = 50
PROJECT_TEMP = pathlib.Path(__file__).resolve().parents[4] / "temp"


class PtySerial:
    def __init__(self, *, port, baudrate, parity, stopbits, bytesize, timeout):
        del baudrate, parity, stopbits, bytesize, timeout
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.is_open = True

    @property
    def in_waiting(self):
        value = array.array("I", [0])
        fcntl.ioctl(self.fd, 0x541B, value, True)
        return value[0]

    def read(self, size=1):
        try:
            return os.read(self.fd, size)
        except BlockingIOError:
            return b""

    def write(self, data):
        view = memoryview(data)
        total = 0
        while view:
            try:
                written = os.write(self.fd, view)
            except BlockingIOError:
                continue
            total += written
            view = view[written:]
        return total

    def flush(self):
        return None

    def close(self):
        if self.is_open:
            os.close(self.fd)
            self.is_open = False


def load_sdk(wheel):
    serial = types.ModuleType("serial")
    serial.Serial = PtySerial
    serial.PARITY_NONE = "N"
    serial.SerialException = OSError
    sys.modules["serial"] = serial
    sys.path.insert(0, wheel)
    from fashionstar_uart_sdk.uart_pocket_handler import (  # noqa: PLC0415
        PortHandler,
        SyncPositionControlOptions,
    )
    from fashionstar_uart_sdk.uservo import Packet  # noqa: PLC0415

    return PortHandler, SyncPositionControlOptions, Packet


def monitor_position(round_index, motor_id):
    return (round_index - 32) * 10 + motor_id


def command_position(round_index, motor_id):
    if motor_id == 0:
        return -(1 << 31) + round_index
    if motor_id == 6:
        return (1 << 31) - 1 - round_index
    return (round_index - 32) * 100 + motor_id


def response_packet(Packet, code, params):
    checksum = Packet.calc_checksum(code, params, pkt_type=Packet.PKT_TYPE_RESPONSE)
    return b"\x05\x1c" + struct.pack("<BB", code, len(params)) + params + bytes([checksum])


def read_request(fd, Packet):
    header = bytearray()
    while bytes(header) != b"\x12\x4c":
        next_byte = os.read(fd, 1)
        if not next_byte:
            raise EOFError("PTY closed while waiting for a request")
        header = (header + next_byte)[-2:]
    code, size = struct.unpack("<BB", read_exact(fd, 2))
    frame = bytes(header) + bytes([code, size]) + read_exact(fd, size + 1)
    if not Packet.verify(frame, pkt_type=Packet.PKT_TYPE_REQUEST):
        raise AssertionError("Python SDK rejected request checksum")
    return code, frame[4:-1]


def read_exact(fd, size):
    value = bytearray()
    while len(value) < size:
        chunk = os.read(fd, size - len(value))
        if not chunk:
            raise EOFError("PTY closed in a packet")
        value.extend(chunk)
    return bytes(value)


def validate_write(params, round_index):
    assert len(params) == 108
    assert params[:3] == bytes([CODE_SET_MTURN_BY_INTERVAL, 15, 7])
    for motor_id, offset in enumerate(range(3, len(params), 15)):
        values = struct.unpack("<BiIHHH", params[offset : offset + 15])
        assert values == (
            motor_id,
            command_position(round_index, motor_id),
            MOTION_TIME_MS,
            ACCELERATION_TIME_MS,
            DECELERATION_TIME_MS,
            0,
        )


def python_protocol_server(port, rounds, Packet):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    try:
        for motor_id in IDS:
            code, params = read_request(fd, Packet)
            assert (code, params) == (CODE_PING, bytes([motor_id]))
            os.write(fd, response_packet(Packet, CODE_PING, params))
        for round_index in range(rounds):
            code, params = read_request(fd, Packet)
            assert code == CODE_SYNC_COMMAND
            assert params == bytes([CODE_QUERY_MONITOR, 1, 7, *IDS])
            if round_index == 0:
                # The Rust transaction retries one lost Monitor request on the same port.
                code, params = read_request(fd, Packet)
                assert code == CODE_SYNC_COMMAND
                assert params == bytes([CODE_QUERY_MONITOR, 1, 7, *IDS])
                # A legal optional action response may still be pending on the bus.
                os.write(fd, response_packet(Packet, CODE_SET_MTURN_BY_INTERVAL, b"\x00\x01"))
                corrupt = bytearray(response_packet(Packet, CODE_QUERY_MONITOR, bytes(16)))
                corrupt[-1] ^= 1
                os.write(fd, b"\xaa\x05\xaa" + corrupt)
            for motor_id in reversed(IDS):
                monitor = struct.pack(
                    "<BHHHHBih",
                    motor_id,
                    7400 + motor_id,
                    100 + motor_id,
                    200 + motor_id,
                    2048,
                    round_index & 0xFF,
                    monitor_position(round_index, motor_id),
                    motor_id - 3,
                )
                frame = response_packet(Packet, CODE_QUERY_MONITOR, monitor)
                for split in (1, 2, 5, len(frame)):
                    if frame:
                        os.write(fd, frame[:split])
                        frame = frame[split:]
            code, params = read_request(fd, Packet)
            assert code == CODE_SYNC_COMMAND
            validate_write(params, round_index)
    finally:
        os.close(fd)


def python_sdk_client(port, rounds, PortHandler, SyncPositionControlOptions):
    handler = PortHandler(port, 1_000_000)
    handler.openPort()
    try:
        for motor_id in IDS:
            assert handler.ping(motor_id)
        names = {f"joint{motor_id}": motor_id for motor_id in IDS}
        for round_index in range(rounds):
            monitors = handler.sync_read["Monitor"](names, realtime=True)
            for name, motor_id in names.items():
                assert monitors[name].current_position == monitor_position(round_index, motor_id) / 10
            commands = {
                name: SyncPositionControlOptions(
                    motor_id,
                    command_position(round_index, motor_id),
                    MOTION_TIME_MS,
                    0,
                    ACCELERATION_TIME_MS,
                    DECELERATION_TIME_MS,
                )
                for name, motor_id in names.items()
            }
            handler.sync_write["Goal_Position"](commands)
    finally:
        handler.closePort()


def wait_for_links(process, links):
    deadline = time.monotonic() + 5
    while not all(link.exists() for link in links):
        if process.poll() is not None:
            raise RuntimeError("socat exited before creating PTYs")
        if time.monotonic() >= deadline:
            raise RuntimeError("socat did not create PTYs")
        time.sleep(0.01)


def socat_pair(directory):
    left = pathlib.Path(directory) / "left"
    right = pathlib.Path(directory) / "right"
    process = subprocess.Popen(
        [
            "socat",
            f"pty,raw,echo=0,link={left}",
            f"pty,raw,echo=0,link={right}",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    wait_for_links(process, (left, right))
    return process, left, right


def stop(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            process.kill()
    process.wait()


def run_rust_client(binary, rounds, Packet):
    PROJECT_TEMP.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix="fashionstar-rust-client-", dir=PROJECT_TEMP
    ) as directory:
        socat, left, right = socat_pair(directory)
        failure = []
        peer = threading.Thread(
            target=lambda: capture(failure, python_protocol_server, str(right), rounds, Packet)
        )
        peer.start()
        try:
            subprocess.run([binary, "client", str(left), str(rounds)], check=True)
            peer.join()
            if failure:
                raise failure[0]
        finally:
            stop(socat)


def run_python_client(binary, rounds, PortHandler, SyncPositionControlOptions):
    PROJECT_TEMP.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix="fashionstar-python-client-", dir=PROJECT_TEMP
    ) as directory:
        socat, left, right = socat_pair(directory)
        rust = subprocess.Popen([binary, "server", str(right), str(rounds)])
        try:
            python_sdk_client(str(left), rounds, PortHandler, SyncPositionControlOptions)
            if rust.wait(timeout=5) != 0:
                raise RuntimeError("Rust protocol peer failed")
        finally:
            if rust.poll() is None:
                rust.terminate()
                rust.wait(timeout=5)
            stop(socat)


def compare_rounding(binary):
    values = [
        "-214748364.8",
        "-0.049999999999",
        "0",
        "0.049999999999",
        "214748364.7",
    ]
    values.extend(repr((step + 0.5) / 10) for step in range(-1000, 1001))
    output = subprocess.check_output([binary, "rounding", *values], text=True)
    rust = [int(value) for value in output.splitlines()]
    python = [round(float(value) * 10) for value in values]
    assert rust == python


def capture(failure, function, *args):
    try:
        function(*args)
    except BaseException as error:  # surfaced in the main test thread
        failure.append(error)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-wheel", required=True)
    parser.add_argument("--rust-peer", required=True)
    parser.add_argument("--rounds", type=int, default=64)
    args = parser.parse_args()
    PortHandler, SyncPositionControlOptions, Packet = load_sdk(args.sdk_wheel)
    compare_rounding(args.rust_peer)
    run_rust_client(args.rust_peer, args.rounds, Packet)
    run_python_client(args.rust_peer, args.rounds, PortHandler, SyncPositionControlOptions)
    print(f"双向交叉测试通过：{args.rounds} 轮 × 2 个方向")


if __name__ == "__main__":
    main()
