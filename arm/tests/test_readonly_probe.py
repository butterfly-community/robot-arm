from __future__ import annotations

import importlib.util
import pathlib
import sys
import unittest

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
