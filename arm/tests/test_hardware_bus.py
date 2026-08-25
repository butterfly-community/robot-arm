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
    GRIPPER_NAME,
    HardwareBus,
    PRODUCTION_MOTOR_IDS,
    make_bus,
)


class FakeMonitor:
    def __init__(
        self,
        position=0.0,
        *,
        power=0,
        current=0,
        temperature=25.0,
        status=0,
    ) -> None:
        self.current_position = position
        self.power = power
        self.current = current
        self.temperature = temperature
        self.status = status


class FakeBus:
    def __init__(self) -> None:
        self.calls = []
        self.monitors = {name: FakeMonitor() for name in PRODUCTION_MOTOR_IDS}
        self.missing_id = None

    def connect(self):
        self.calls.append(("connect",))

    def ping(self, motor_id):
        self.calls.append(("ping", motor_id))
        return motor_id != self.missing_id

    def sync_read(self, register, names, *, normalize):
        self.calls.append(("sync_read", register, tuple(names), normalize))
        return {name: self.monitors[name] for name in names}

    def sync_write(self, register, values, *, normalize):
        self.calls.append(("sync_write", register, dict(values), normalize))

    def disconnect(self):
        self.calls.append(("disconnect",))


class HardwareBusTests(unittest.TestCase):
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
                self.sync_read = {"Monitor": self._monitor}
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

            def _monitor(self, ids, *, realtime):
                self.calls.append(("monitor", dict(ids), realtime))
                return {name: FakeMonitor() for name in ids}

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
            bus.sync_read("Monitor", ["shoulder_pan"], normalize=False)
            bus.sync_write("Goal_Position", {"shoulder_pan": 1.0}, normalize=False)
            bus.disconnect()

        calls = FakePortHandler.last.calls
        self.assertEqual(calls[0], ("open",))
        self.assertEqual(calls[-1], ("close",))
        self.assertEqual(
            [call[0] for call in calls],
            ["open", "ping", "monitor", "write_positions", "close"],
        )
        command = calls[3][1]["shoulder_pan"]
        self.assertEqual(command.values, (0, 10, 350, 0, 50, 50))

    def test_connect_reads_all_feedback_without_configuration_or_torque_writes(self):
        raw = FakeBus()
        bus = HardwareBus(raw)
        feedback = bus.connect()
        self.assertEqual(feedback.arm_positions_rad, (0.0,) * 6)
        self.assertEqual(feedback.gripper.position_rad, 0.0)
        bus.write_positions(
            [math.radians(index) for index in range(6)],
            math.pi / 2.0,
        )
        bus.disconnect()

        self.assertEqual(raw.calls[0], ("connect",))
        self.assertEqual(raw.calls[1:8], [("ping", index) for index in range(7)])
        self.assertEqual(raw.calls[8][0:2], ("sync_read", "Monitor"))
        write = raw.calls[9]
        self.assertEqual(write[0:2], ("sync_write", "Goal_Position"))
        self.assertEqual(write[2]["shoulder_pan"], 0.0)
        self.assertAlmostEqual(write[2]["wrist_roll"], 5.0)
        self.assertEqual(write[2][GRIPPER_NAME], -90.0)
        self.assertEqual(raw.calls[-1], ("disconnect",))

    def test_gripper_monitor_uses_sdk_units_and_manufacturer_joint_mapping(self):
        raw = FakeBus()
        raw.monitors[GRIPPER_NAME] = FakeMonitor(
            -45.0,
            power=2000,
            current=750,
            temperature=31.5,
            status=1 << 6,
        )
        feedback = HardwareBus(raw).connect().gripper
        self.assertAlmostEqual(feedback.position_rad, math.pi / 4.0)
        self.assertEqual(feedback.power_w, 2.0)
        self.assertEqual(feedback.current_a, 0.75)
        self.assertEqual(feedback.temperature_c, 31.5)
        self.assertEqual(feedback.status, 1 << 6)

    def test_gripper_monitor_rejects_boolean_status(self):
        raw = FakeBus()
        raw.monitors[GRIPPER_NAME].status = True

        with self.assertRaisesRegex(RuntimeError, "Monitor 状态无效"):
            HardwareBus(raw).connect()

    def test_gripper_adapter_does_not_duplicate_urdf_range_checks(self):
        raw = FakeBus()
        bus = HardwareBus(raw)
        bus.write_positions([0.0] * 6, math.radians(91.0))
        self.assertEqual(raw.calls[-1][2][GRIPPER_NAME], -91.0)

    def test_missing_motor_closes_without_disabling_torque(self):
        raw = FakeBus()
        raw.missing_id = 4
        with self.assertRaises(RuntimeError):
            HardwareBus(raw).connect()
        self.assertEqual(raw.calls[-1], ("disconnect",))


class StandardControlPathTests(unittest.TestCase):
    def test_hardware_home_uses_ros2_control_trajectory_controller(self):
        node = (PACKAGE_ROOT / "stararm102_teleop_moveit" / "hardware_node.py").read_text()
        bridge = (
            PACKAGE_ROOT / "stararm102_teleop_moveit" / "servo_ipc_bridge.py"
        ).read_text()
        launch = (PACKAGE_ROOT / "launch" / "hardware.launch.py").read_text()
        launch_support = (
            PACKAGE_ROOT / "stararm102_teleop_moveit" / "launch_support.py"
        ).read_text()
        launched = launch + launch_support
        controllers = (PACKAGE_ROOT / "config" / "hardware_controllers.yaml").read_text()
        servo_config = (PACKAGE_ROOT / "config" / "servo.yaml").read_text()
        hardware_patch = (
            PROJECT_ROOT / "patches" / "star-arm-102-fl-topic-hardware.patch"
        ).read_text()
        model_patch = (
            PROJECT_ROOT / "patches" / "star-arm-102-fl-moveit-model.patch"
        ).read_text()

        self.assertNotIn("FollowJointTrajectory", node)
        self.assertNotIn("positions_at", node)
        self.assertIn('self, ExecuteTrajectory, "/execute_trajectory"', bridge)
        self.assertIn('SetBool, "/servo_node/pause_servo"', bridge)
        self.assertIn("HOME_ZERO_TOLERANCE_RAD = math.radians(1.0)", bridge)
        self.assertIn("request.data = True", bridge)
        self.assertIn("request.data = False", bridge)
        self.assertIn('GetStateValidity, "/check_state_validity"', bridge)
        self.assertIn('GetPlanningScene, "/get_planning_scene"', bridge)
        self.assertIn("scene.allowed_collision_matrix = allowed_collision_matrix", bridge)
        dynamics_patch = (
            PROJECT_ROOT / "patches" / "star-arm-102-fl-moveit-dynamics.patch"
        ).read_text()
        self.assertIn("max_acceleration: 30.0", dynamics_patch)
        for script in ("run-moveit-simulation.sh", "run-moveit-hardware.sh"):
            self.assertIn(
                "star-arm-102-fl-moveit-dynamics.patch",
                (PROJECT_ROOT / "ros2" / script).read_text(),
            )
        self.assertNotIn('"/arm_controller/joint_trajectory"', bridge)
        self.assertIn('goal.controller_names = ["arm_controller"]', bridge)
        self.assertIn('executable="ros2_control_node"', launched)
        self.assertIn('"arm_controller"', launched)
        self.assertIn('"hand_controller"', launched)
        self.assertIn("joint_trajectory_controller/JointTrajectoryController", controllers)
        self.assertIn("hard_stop_singularity_threshold: .inf", servo_config)
        self.assertNotIn("hard_stop_singularity_threshold: 300.0", servo_config)
        self.assertIn("joints: [joint7_left]", controllers)
        self.assertIn(
            "joint_state_topic_hardware_interface/JointStateTopicSystem",
            hardware_patch,
        )
        self.assertIn('-      <state_interface name="velocity"/>', hardware_patch)
        self.assertNotIn('-    <joint name="joint7_left">', hardware_patch)
        self.assertIn('-    <joint name="joint7_right">', hardware_patch)
        self.assertIn('+      upper="1.5707963268"', model_patch)
        self.assertIn('+        <param name="max">1.5707963268</param>', model_patch)


if __name__ == "__main__":
    unittest.main()
