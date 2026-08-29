import math
import tempfile
import unittest
from pathlib import Path

from stararm_102_motion_node.motion_core import (
    PIVOT_TO_TCP_M,
    MotionConfig,
    Pose,
    controller_sync_required,
    frozen_session_after_servo_status,
    load_motion_config,
    merge_controller_command,
    save_motion_config,
    session_is_frozen,
    target_pose,
    tool_action_transition,
    tool_position_rad,
)
from transforms3d.quaternions import axangle2quat, quat2mat


def axis_angle(
    axis: tuple[float, float, float], angle: float
) -> tuple[float, float, float, float]:
    value = axangle2quat(axis, angle)
    return float(value[1]), float(value[2]), float(value[3]), float(value[0])


def rotate(
    rotation: tuple[float, float, float, float], vector: tuple[float, float, float]
) -> tuple[float, float, float]:
    value = quat2mat((rotation[3], rotation[0], rotation[1], rotation[2])).dot(vector)
    return float(value[0]), float(value[1]), float(value[2])


class MotionCoreTests(unittest.TestCase):
    def test_controllers_sync_on_first_feedback_and_hardware_acquisition(self) -> None:
        self.assertTrue(controller_sync_required(None, "software"))
        self.assertFalse(controller_sync_required("software", "software"))
        self.assertTrue(controller_sync_required("software", "hardware"))
        self.assertFalse(controller_sync_required("hardware", "software"))

    def test_missing_motion_config_uses_defaults(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = load_motion_config(Path(directory) / "missing.json")
        self.assertEqual(config, MotionConfig())

    def test_motion_config_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motion.json"
            expected = MotionConfig(config_version=8, control_mode="manual")
            save_motion_config(path, expected)
            actual = load_motion_config(path)
        self.assertEqual(actual, expected)

    def test_malformed_existing_motion_config_is_an_error(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motion.json"
            path.write_text("{", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_motion_config(path)

    def test_unknown_persisted_control_mode_is_an_error(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "motion.json"
            path.write_text(
                '{"schema_version":1,"config_version":1,"control_mode":"other"}',
                encoding="utf-8",
            )
            with self.assertRaises(ValueError):
                load_motion_config(path)

    def test_arm_command_keeps_actuator_feedback_for_nan_interface(self) -> None:
        merged = merge_controller_command(
            ["j1", "j2", "tool"],
            [0.25, -0.5, float("nan")],
            ("j1", "j2"),
            "tool",
            [0.0, 0.0],
            0.75,
        )
        self.assertEqual(merged, ([0.25, -0.5], 0.75))

    def test_actuator_command_keeps_joint_feedback_for_nan_interfaces(self) -> None:
        merged = merge_controller_command(
            ["j1", "j2", "tool"],
            [float("nan"), float("nan"), 0.5],
            ("j1", "j2"),
            "tool",
            [0.1, -0.2],
            0.0,
        )
        self.assertEqual(merged, ([0.1, -0.2], 0.5))

    def test_interleaved_controllers_keep_the_other_controllers_latest_target(
        self,
    ) -> None:
        arm = merge_controller_command(
            ["j1", "j2", "tool"],
            [0.25, -0.5, float("nan")],
            ("j1", "j2"),
            "tool",
            [0.0, 0.0],
            0.1,
        )
        self.assertIsNotNone(arm)
        actuator = merge_controller_command(
            ["j1", "j2", "tool"],
            [float("nan"), float("nan"), 0.6],
            ("j1", "j2"),
            "tool",
            list(arm[0]),
            arm[1],
        )
        self.assertEqual(actuator, ([0.25, -0.5], 0.6))
        late_arm = merge_controller_command(
            ["j1", "j2", "tool"],
            [0.25, -0.5, float("nan")],
            ("j1", "j2"),
            "tool",
            list(actuator[0]),
            actuator[1],
        )
        self.assertEqual(late_arm, ([0.25, -0.5], 0.6))

    def test_no_active_session_is_not_frozen(self):
        self.assertFalse(session_is_frozen(None, None))
        self.assertFalse(session_is_frozen(None, 4))
        self.assertTrue(session_is_frozen(4, 4))
        self.assertFalse(session_is_frozen(4, 5))

    def test_only_servo_code_six_freezes_the_active_control_session(self):
        for code in (-1, 0, 1, 2, 3, 4, 5):
            self.assertIsNone(frozen_session_after_servo_status(code, 4, None))
        self.assertEqual(frozen_session_after_servo_status(6, 4, None), 4)
        self.assertEqual(frozen_session_after_servo_status(6, None, None), None)
        self.assertEqual(frozen_session_after_servo_status(1, 5, 4), 4)

    def test_tool_action_ignores_the_initial_sample_and_emits_changes(self):
        self.assertIsNone(tool_action_transition(None, 1.0))
        self.assertIsNone(tool_action_transition(None, 0.0))
        self.assertIsNone(tool_action_transition(0.375, 0.375))
        self.assertEqual(tool_action_transition(0.0, 0.375), 0.375)
        self.assertEqual(tool_action_transition(0.375, 0.0), 0.0)

    def test_tool_action_positions_use_the_full_gripper_range(self):
        self.assertEqual(tool_position_rad(1.0), 0.0)
        self.assertEqual(tool_position_rad(0.0), math.radians(90.0))
        self.assertEqual(tool_position_rad(0.5), math.radians(45.0))

    def test_translation_is_added_without_a_second_scale(self) -> None:
        anchor = Pose((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0))
        target = target_pose(anchor, [0.02, -0.03, 0.04], 0.0, 0.0)
        for actual, expected in zip(target.position_m, (0.12, 0.17, 0.34), strict=True):
            self.assertAlmostEqual(actual, expected)

    def test_pitch_moves_tcp_on_vertical_arc_around_rear_pivot(self) -> None:
        anchor = Pose(
            (0.0, PIVOT_TO_TCP_M[2], 0.0), axis_angle((1.0, 0.0, 0.0), -math.pi / 2)
        )
        target = target_pose(anchor, [0.0, 0.0, 0.0], math.radians(8), 0.0)
        anchor_pivot = tuple(
            anchor.position_m[i] - rotate(anchor.orientation_xyzw, PIVOT_TO_TCP_M)[i]
            for i in range(3)
        )
        target_pivot = tuple(
            target.position_m[i] - rotate(target.orientation_xyzw, PIVOT_TO_TCP_M)[i]
            for i in range(3)
        )
        self.assertTrue(all(abs(value) < 1e-12 for value in target_pivot))
        self.assertTrue(all(abs(value) < 1e-12 for value in anchor_pivot))
        self.assertNotEqual(target.position_m, anchor.position_m)

    def test_horizontal_arc_moves_tcp_and_keeps_height(self) -> None:
        anchor = Pose(
            (0.0, PIVOT_TO_TCP_M[2], 0.0), axis_angle((1.0, 0.0, 0.0), -math.pi / 2)
        )
        target = target_pose(anchor, [0.0, 0.0, 0.0], 0.0, math.radians(8))
        self.assertNotEqual(target.position_m, anchor.position_m)
        self.assertLess(abs(target.position_m[2] - anchor.position_m[2]), 1e-12)

    def test_opposite_arc_returns_to_anchor(self) -> None:
        anchor = Pose((0.2, -0.1, 0.3), axis_angle((1.0, 0.0, 0.0), -math.pi / 2))
        left = target_pose(anchor, [0.0, 0.0, 0.0], 0.0, math.radians(8))
        returned = target_pose(anchor, [0.0, 0.0, 0.0], 0.0, 0.0)
        self.assertNotEqual(left.position_m, anchor.position_m)
        for actual, expected in zip(
            returned.position_m, anchor.position_m, strict=True
        ):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(
            returned.orientation_xyzw, anchor.orientation_xyzw, strict=True
        ):
            self.assertAlmostEqual(actual, expected)
