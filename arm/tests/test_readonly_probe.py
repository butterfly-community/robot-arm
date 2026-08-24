from __future__ import annotations

import importlib.util
import pathlib
import sys
import types
import unittest
from unittest.mock import patch

MODULE_PATH = (
    pathlib.Path(__file__).parents[1] / "tools" / "stararm102_fl_readonly_probe.py"
)
SPEC = importlib.util.spec_from_file_location("stararm102_fl_readonly_probe", MODULE_PATH)
assert SPEC and SPEC.loader
probe_module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe_module
SPEC.loader.exec_module(probe_module)


class FakeBus:
    def __init__(self, positions=None):
        self.calls = []
        self.positions = positions or {
            name: float(index) for index, name in enumerate(probe_module.JOINT_IDS)
        }

    def connect(self):
        self.calls.append(("connect",))

    def ping(self, motor_id):
        self.calls.append(("ping", motor_id))
        return True

    def sync_read(self, register, names, *, normalize):
        self.calls.append(("sync_read", register, tuple(names), normalize))
        return self.positions

    def disconnect(self, *, disable_torque):
        self.calls.append(("disconnect", disable_torque))


class ReadOnlyProbeTests(unittest.TestCase):
    def test_transport_open_has_no_reset_or_torque_operation(self):
        class FakePortHandler:
            last = None

            def __init__(self, port, baudrate):
                self.is_open = False
                self.calls = []
                FakePortHandler.last = self

            def openPort(self):
                self.calls.append(("open",))
                self.is_open = True

            def closePort(self):
                self.calls.append(("close",))
                self.is_open = False

            def ping(self, motor_id):
                self.calls.append(("ping", motor_id))
                return True

            def read_positions(self, ids):
                self.calls.append(("read_positions", dict(ids)))
                return {name: 0.0 for name in ids}

        package = types.ModuleType("fashionstar_uart_sdk")
        module = types.ModuleType("fashionstar_uart_sdk.uart_pocket_handler")
        module.PortHandler = FakePortHandler
        with patch.dict(
            sys.modules,
            {
                "fashionstar_uart_sdk": package,
                "fashionstar_uart_sdk.uart_pocket_handler": module,
            },
        ):
            bus = probe_module.make_bus("/dev/ttyUSB0", 1_000_000)
            bus.connect()
            bus.ping(0)
            bus.sync_read(
                "Present_Position", list(probe_module.JOINT_IDS), normalize=False
            )
            bus.disconnect(disable_torque=False)
        self.assertEqual(
            [call[0] for call in FakePortHandler.last.calls],
            ["open", "ping", "read_positions", "close"],
        )

    def test_success_uses_only_ping_present_position_and_non_mutating_close(self):
        bus = FakeBus()
        result = probe_module.ReadOnlyProbe(bus).run(
            requested_device="/dev/serial/by-id/uc-01",
            resolved_device="/dev/ttyUSB0",
            stable_device=True,
            baudrate=1_000_000,
            cycles=2,
            interval_seconds=0,
        )
        self.assertTrue(result.valid)
        self.assertEqual(len(result.samples), 2)
        self.assertEqual(bus.calls[0], ("connect",))
        self.assertEqual(bus.calls[1:8], [("ping", index) for index in range(7)])
        self.assertEqual(bus.calls[-1], ("disconnect", False))
        self.assertEqual(
            {call[1] for call in bus.calls if call[0] == "sync_read"},
            {"Present_Position"},
        )

    def test_missing_feedback_is_invalid_and_never_reuses_a_cache(self):
        positions = {name: 0.0 for name in probe_module.JOINT_IDS}
        positions["wrist_roll"] = None
        bus = FakeBus(positions)
        result = probe_module.ReadOnlyProbe(bus).run(
            requested_device="/dev/ttyUSB0",
            resolved_device="/dev/ttyUSB0",
            stable_device=False,
            baudrate=1_000_000,
            cycles=1,
            interval_seconds=0,
        )
        self.assertFalse(result.valid)
        self.assertEqual(result.samples, [])
        self.assertEqual(bus.calls[-1], ("disconnect", False))


if __name__ == "__main__":
    unittest.main()
