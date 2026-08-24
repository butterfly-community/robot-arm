from __future__ import annotations

import math
import pathlib
import sys
import types
import unittest
from unittest.mock import patch


PACKAGE_ROOT = pathlib.Path(__file__).parents[1] / "ros2" / "stararm102_teleop_moveit"
PROJECT_ROOT = pathlib.Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))
from stararm102_teleop_moveit.hardware_bus import (  # noqa: E402
    CommandSafetyGate,
    MOTOR_IDS,
    SafeHardwareBus,
    make_bus,
)


class FakeBus:
    def __init__(self) -> None:
        self.calls = []
        self.positions = {name: 0.0 for name in MOTOR_IDS}
        self.missing_id = None

    def connect(self):
        self.calls.append(("connect",))

    def ping(self, motor_id):
        self.calls.append(("ping", motor_id))
        return motor_id != self.missing_id

    def sync_read(self, register, names, *, normalize):
        self.calls.append(("sync_read", register, tuple(names), normalize))
        return self.positions

    def sync_write(self, register, values, *, normalize):
        self.calls.append(("sync_write", register, dict(values), normalize))

    def disconnect(self, *, disable_torque):
        self.calls.append(("disconnect", disable_torque))


class SafeHardwareBusTests(unittest.TestCase):
    def test_official_sdk_adapter_opens_without_reset_or_torque_command(self):
        class FakeOptions:
            def __init__(self, *values):
                self.values = values

        class FakePortHandler:
            last = None

            def __init__(self, port, baudrate):
                self.port = port
                self.baudrate = baudrate
                self.is_open = False
                self.calls = []
                self.sync_write = {"Goal_Position": self._write}
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

            def _write(self, commands):
                self.calls.append(("write_positions", dict(commands)))

        package = types.ModuleType("fashionstar_uart_sdk")
        module = types.ModuleType("fashionstar_uart_sdk.uart_pocket_handler")
        module.PortHandler = FakePortHandler
        module.SyncPositionControlOptions = FakeOptions
        with patch.dict(
            sys.modules,
            {
                "fashionstar_uart_sdk": package,
                "fashionstar_uart_sdk.uart_pocket_handler": module,
            },
        ):
            bus = make_bus("/dev/stararm102", 1_000_000)
            bus.connect()
            self.assertTrue(bus.ping(0))
            bus.sync_read("Present_Position", ["shoulder_pan"], normalize=False)
            bus.sync_write("Goal_Position", {"shoulder_pan": 1.0}, normalize=False)
            bus.disconnect(disable_torque=False)

        calls = FakePortHandler.last.calls
        self.assertEqual(calls[0], ("open",))
        self.assertEqual(calls[-1], ("close",))
        self.assertEqual(
            [call[0] for call in calls],
            ["open", "ping", "read_positions", "write_positions", "close"],
        )
        command = calls[3][1]["shoulder_pan"]
        self.assertEqual(command.values, (0, 10, 350, 0, 50, 50))

    def test_connect_reads_all_feedback_without_configuration_or_torque_writes(self):
        raw = FakeBus()
        bus = SafeHardwareBus(raw)
        self.assertEqual(bus.connect(), (0.0,) * 6)
        bus.write_arm_positions([math.radians(index) for index in range(6)])
        bus.disconnect()

        self.assertEqual(raw.calls[0], ("connect",))
        self.assertEqual(raw.calls[1:7], [("ping", index) for index in range(6)])
        self.assertEqual(raw.calls[7][0:2], ("sync_read", "Present_Position"))
        write = raw.calls[8]
        self.assertEqual(write[0:2], ("sync_write", "Goal_Position"))
        self.assertEqual(write[2]["shoulder_pan"], 0.0)
        self.assertAlmostEqual(write[2]["wrist_roll"], 5.0)
        self.assertEqual(raw.calls[-1], ("disconnect", False))
        self.assertNotIn("gripper", write[2])

    def test_missing_motor_closes_without_disabling_torque(self):
        raw = FakeBus()
        raw.missing_id = 4
        with self.assertRaises(RuntimeError):
            SafeHardwareBus(raw).connect()
        self.assertEqual(raw.calls[-1], ("disconnect", False))

class CommandSafetyGateTests(unittest.TestCase):
    def setUp(self):
        self.gate = CommandSafetyGate()
        self.gate.update_feedback([0.0] * 6, 1.0)
        self.gate.set_enabled(True)

    def test_small_fresh_target_is_accepted(self):
        target = [math.radians(1.0), 0.0, 0.0, 0.0, 0.0, 0.0]
        self.assertEqual(self.gate.accept_target(target, 1.01), tuple(target))
        self.assertIsNone(self.gate.fault)

    def test_standard_controller_owns_jump_and_rate_limits(self):
        target = [math.radians(90.0), 0.0, 0.0, 0.0, 0.0, 0.0]
        self.assertEqual(self.gate.accept_target(target, 1.01), tuple(target))
        self.assertIsNone(self.gate.fault)

    def test_fault_latches_until_release_and_repress(self):
        self.gate.trip("test fault")
        self.gate.set_enabled(True)
        self.assertFalse(self.gate.enabled)
        self.gate.set_enabled(False)
        self.gate.update_feedback([0.0] * 6, 1.02)
        self.gate.set_enabled(True)
        self.assertEqual(self.gate.accept_target([0.0] * 6, 1.03), (0.0,) * 6)

    def test_stale_feedback_and_joint_limit_fail_closed(self):
        self.assertIsNone(self.gate.accept_target([0.0] * 6, 1.2))
        self.assertIn("100 ms", self.gate.fault)

        gate = CommandSafetyGate()
        gate.update_feedback([0.0] * 6, 2.0)
        gate.set_enabled(True)
        target = [0.0] * 6
        target[4] = math.radians(66.0)
        self.assertIsNone(gate.accept_target(target, 2.01))
        self.assertIn("J5", gate.fault)


class StandardControlPathTests(unittest.TestCase):
    def test_hardware_home_uses_ros2_control_trajectory_controller(self):
        node = (PACKAGE_ROOT / "stararm102_teleop_moveit" / "hardware_node.py").read_text()
        bridge = (
            PACKAGE_ROOT / "stararm102_teleop_moveit" / "servo_ipc_bridge.py"
        ).read_text()
        launch = (PACKAGE_ROOT / "launch" / "hardware.launch.py").read_text()
        controllers = (PACKAGE_ROOT / "config" / "hardware_controllers.yaml").read_text()
        hardware_patch = (
            PROJECT_ROOT / "patches" / "star-arm-102-fl-topic-hardware.patch"
        ).read_text()

        self.assertNotIn("FollowJointTrajectory", node)
        self.assertNotIn("positions_at", node)
        self.assertIn('self, ExecuteTrajectory, "/execute_trajectory"', bridge)
        self.assertIn('goal.controller_names = ["arm_controller"]', bridge)
        self.assertIn('executable="ros2_control_node"', launch)
        self.assertIn('arguments=["arm_controller"', launch)
        self.assertIn("joint_trajectory_controller/JointTrajectoryController", controllers)
        self.assertIn(
            "joint_state_topic_hardware_interface/JointStateTopicSystem",
            hardware_patch,
        )
        self.assertIn('-      <state_interface name="velocity"/>', hardware_patch)
        self.assertIn('-    <joint name="joint7_left">', hardware_patch)


if __name__ == "__main__":
    unittest.main()
