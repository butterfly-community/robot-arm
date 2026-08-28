"""Dora-facing MoveIt/Servo node. ROS is private to this model service."""

from __future__ import annotations

import hashlib
import json
import math
import os
import queue
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

import pyarrow as pa
import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from dora import Node as DoraNode
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    PlanningSceneComponents,
    ServoStatus,
)
from moveit_msgs.srv import GetPlanningScene, GetStateValidity, ServoCommandType
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .motion_core import (
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

SCHEMA_VERSION = 2
MODEL_ID = "stararm-102-fl"
MODEL_REVISION = "stararm-102-fl-v1"
JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_KEY = "gripper"
GRIPPER_JOINT = "joint7_left"
STATE_TOPIC = "/stararm102/joint_states"
COMMAND_TOPIC = "/stararm102/joint_commands"
START_RAD = tuple(math.radians(value) for value in (0.0, 0.0, -3.0, 0.0, 0.0, 0.0))
TEST_RAD = tuple(math.radians(value) for value in (0.0, 0.0, -20.0, 0.0, 0.0, 0.0))


def arrow_encode(value: Any) -> pa.StructArray:
    return pa.StructArray.from_arrays(
        [
            pa.array([SCHEMA_VERSION], type=pa.uint32()),
            pa.array([json.dumps(value, separators=(",", ":"), ensure_ascii=False)]),
        ],
        names=["schema_version", "payload_json"],
    )


def arrow_decode(value: pa.Array) -> Any:
    if not isinstance(value, pa.StructArray) or len(value) != 1:
        raise ValueError("expected one robot-arm Arrow envelope")
    version = value.field("schema_version")[0].as_py()
    if version != SCHEMA_VERSION:
        raise ValueError(f"unsupported schema version {version}")
    return json.loads(value.field("payload_json")[0].as_py())


def finite(values: Any, length: int) -> bool:
    return (
        isinstance(values, list)
        and len(values) == length
        and all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
            for value in values
        )
    )


class MotionNode(Node):
    def __init__(self) -> None:
        super().__init__("stararm_102_motion_node")
        self._lock = threading.RLock()
        self._config_path = Path(
            os.environ.get("STARARM_MOTION_CONFIG", "/config/stararm-102-motion.json")
        )
        self._config = load_motion_config(self._config_path)
        self._outgoing: queue.SimpleQueue[tuple[str, dict[str, Any]]] = (
            queue.SimpleQueue()
        )
        self._stop = threading.Event()
        self._latest_arm_state: dict[str, Any] | None = None
        self._feedback_source: str | None = None
        self._controller_sync_requested = False
        self._controller_sync_phase: str | None = None
        self._controller_output_armed = False
        self._current_tcp: Pose | None = None
        self._anchor_tcp: Pose | None = None
        self._target_tcp: Pose | None = None
        self._control_session_id: int | None = None
        self._control_mode = self._config.control_mode
        self._primary_tool_value: float | None = None
        self._frozen_session: int | None = None
        self._hold_sent = False
        self._sequence = 0
        self._last_controller_command: tuple[tuple[float, ...], float] | None = None
        self._servo_code: int | None = None
        self._servo_message: str | None = None
        self._servo_paused = False
        self._pose_commands_selected = False
        self._command_request_pending = False
        self._motion_status = self._idle_motion_status()
        self._actuator_status: dict[str, Any] | None = None
        self._motion_target: list[float] | None = None
        self._planned_trajectory: Any | None = None
        self._plan_goal_handle: Any | None = None
        self._execute_goal_handle: Any | None = None

        self._pose_publisher = self.create_publisher(
            PoseStamped, "/servo_node/pose_target_cmds", 1
        )
        self._hand_publisher = self.create_publisher(
            JointTrajectory, "/hand_controller/joint_trajectory", 1
        )
        self._state_publisher = self.create_publisher(JointState, STATE_TOPIC, 10)
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        self.create_subscription(
            JointState, COMMAND_TOPIC, self._controller_command_callback, 10
        )
        self.create_subscription(
            ServoStatus, "/servo_node/status", self._servo_status_callback, 10
        )
        self._command_client = self.create_client(
            ServoCommandType, "/servo_node/switch_command_type"
        )
        self._controller_switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self._servo_pause_client = self.create_client(
            SetBool, "/servo_node/pause_servo"
        )
        self._move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self._execute_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self._arm_trajectory_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self._hand_trajectory_client = ActionClient(
            self, FollowJointTrajectory, "/hand_controller/follow_joint_trajectory"
        )
        self._state_validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self._planning_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.2, self._maintain_ros_interfaces)
        self._model_info = self._build_model_info()
        self._enqueue("robot_model_info", self._model_info)
        self._enqueue_motion_state()
        self._dora_thread = threading.Thread(
            target=self._dora_loop, name="stararm-motion-dora", daemon=True
        )
        self._dora_thread.start()

    def destroy_node(self) -> bool:
        self._stop.set()
        self._dora_thread.join()
        return super().destroy_node()

    @staticmethod
    def _idle_motion_status() -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "request_id": "",
            "acknowledged_action": "apply",
            "state": "idle",
            "backend_name": "moveit",
            "result_code": None,
            "result_message": "尚未请求普通运动",
            "trajectory_points": None,
            "planned_duration_s": None,
        }

    def _enqueue(self, output: str, value: dict[str, Any]) -> None:
        self._outgoing.put((output, value))

    def _dora_loop(self) -> None:
        dora = DoraNode()
        while not self._stop.is_set():
            while True:
                try:
                    output, value = self._outgoing.get_nowait()
                except queue.Empty:
                    break
                dora.send_output(output, arrow_encode(value))
            event = dora.next(0.01)
            if event is None:
                continue
            if event["type"] == "STOP":
                self._stop.set()
                if rclpy.ok():
                    rclpy.shutdown()
                break
            if event["type"] != "INPUT":
                continue
            try:
                self._handle_dora_input(event["id"], arrow_decode(event["value"]))
            except Exception as error:
                self.get_logger().error(f"Dora input {event.get('id')} failed: {error}")

    def _handle_dora_input(self, input_id: str, value: dict[str, Any]) -> None:
        with self._lock:
            if input_id == "arm_state":
                self._apply_arm_state(value)
            elif input_id == "relative_motion":
                self._apply_relative_motion(value)
            elif input_id == "set_control_mode":
                self._set_control_mode(value)
            elif input_id == "motion_request":
                self._handle_motion_request(value)
            elif input_id == "tool_actuator_request":
                self._handle_actuator_request(value)
            elif input_id == "model_asset_request":
                self._handle_asset_request(value)
            elif input_id == "snapshot":
                self._enqueue("robot_model_info", self._model_info)
                self._enqueue_motion_state()

    def _apply_arm_state(self, value: dict[str, Any]) -> None:
        if (
            value.get("model_revision") != MODEL_REVISION
            or not finite(value.get("joints_rad"), 6)
            or not finite(value.get("actuators_rad"), 1)
        ):
            raise ValueError(
                "ArmState does not match StarArm-102 model revision and shape"
            )
        source = str(value.get("feedback_source", ""))
        requires_sync = controller_sync_required(self._feedback_source, source)
        self._feedback_source = source
        if requires_sync:
            self._controller_sync_requested = True
            self._controller_output_armed = False
        if not self._controller_output_armed:
            self._last_controller_command = (
                tuple(map(float, value["joints_rad"])),
                float(value["actuators_rad"][0]),
            )
        self._latest_arm_state = value
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [*JOINTS, GRIPPER_JOINT]
        message.position = [
            *map(float, value["joints_rad"]),
            float(value["actuators_rad"][0]),
        ]
        self._state_publisher.publish(message)

    def _joint_state_callback(self, message: JointState) -> None:
        try:
            transform = self._tf_buffer.lookup_transform("base_link", "link6", Time())
        except TransformException:
            return
        p = transform.transform.translation
        q = transform.transform.rotation
        with self._lock:
            self._current_tcp = Pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            self._enqueue_motion_state()

    def _controller_command_callback(self, message: JointState) -> None:
        with self._lock:
            if (
                self._latest_arm_state is None
                or self._controller_sync_requested
                or self._controller_sync_phase is not None
                or not self._controller_output_armed
            ):
                return
            if self._last_controller_command is None:
                fallback_joints = list(self._latest_arm_state["joints_rad"])
                fallback_actuator = float(self._latest_arm_state["actuators_rad"][0])
            else:
                fallback_joints = list(self._last_controller_command[0])
                fallback_actuator = self._last_controller_command[1]
            merged = merge_controller_command(
                list(message.name),
                list(message.position),
                JOINTS,
                GRIPPER_JOINT,
                fallback_joints,
                fallback_actuator,
            )
            if merged is None:
                return
            joints, actuator = merged
            if session_is_frozen(self._frozen_session, self._control_session_id):
                return
            identity = (tuple(joints), actuator)
            if identity == self._last_controller_command:
                return
            self._last_controller_command = identity
            self._sequence += 1
            self._enqueue(
                "arm_command",
                {
                    "schema_version": SCHEMA_VERSION,
                    "sequence": self._sequence,
                    "controller_time_ns": time.time_ns(),
                    "model_revision": MODEL_REVISION,
                    "joints_rad": joints,
                    "actuators_rad": [actuator],
                },
            )

    def _synchronize_controllers(self) -> None:
        with self._lock:
            if (
                not self._controller_sync_requested
                or self._controller_sync_phase is not None
                or self._motion_status["state"] in {"planning", "executing"}
                or not self._controller_switch_client.service_is_ready()
                or not self._arm_trajectory_client.server_is_ready()
                or not self._hand_trajectory_client.server_is_ready()
            ):
                return
            self._controller_sync_requested = False
            self._controller_sync_phase = "deactivate"
        self._switch_controllers([], ["arm_controller", "hand_controller"])

    def _switch_controllers(self, activate: list[str], deactivate: list[str]) -> None:
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.BEST_EFFORT
        self._controller_switch_client.call_async(request).add_done_callback(
            self._controller_switch_response
        )

    def _controller_switch_response(self, future: Any) -> None:
        try:
            response = future.result()
            if not response.ok:
                raise RuntimeError(
                    response.message or "controller manager rejected switch"
                )
        except Exception as error:
            self.get_logger().error(f"同步 ros2_control 控制器失败：{error}")
            with self._lock:
                self._controller_sync_phase = None
                self._controller_sync_requested = True
            return
        with self._lock:
            activate = self._controller_sync_phase == "deactivate"
            self._controller_sync_phase = "activate" if activate else None
            if not activate:
                self._enqueue_motion_state()
        if activate:
            self._switch_controllers(["arm_controller", "hand_controller"], [])

    def _maintain_ros_interfaces(self) -> None:
        self._synchronize_controllers()
        self._select_pose_commands()

    def _servo_status_callback(self, message: ServoStatus) -> None:
        with self._lock:
            self._servo_code = int(message.code)
            self._servo_message = message.message
            frozen = frozen_session_after_servo_status(
                self._servo_code, self._control_session_id, self._frozen_session
            )
            if frozen != self._frozen_session:
                self._frozen_session = frozen
                self._publish_feedback_hold_once()
            self._enqueue_motion_state()

    def _publish_feedback_hold_once(self) -> None:
        if self._hold_sent or self._latest_arm_state is None:
            return
        self._hold_sent = True
        self._sequence += 1
        self._enqueue(
            "arm_command",
            {
                "schema_version": SCHEMA_VERSION,
                "sequence": self._sequence,
                "controller_time_ns": time.time_ns(),
                "model_revision": MODEL_REVISION,
                "joints_rad": list(self._latest_arm_state["joints_rad"]),
                "actuators_rad": list(self._latest_arm_state["actuators_rad"]),
            },
        )

    def _apply_relative_motion(self, value: dict[str, Any]) -> None:
        session_id = value.get("control_session_id")
        open_tool = bool(value.get("primary_tool_open"))
        tool_value = float(value.get("primary_tool_value", 0.0))
        if open_tool:
            self._primary_tool_value = 0.0
            self._publish_actuator("input-action", tool_position_rad(0.0))
        else:
            transition = tool_action_transition(self._primary_tool_value, tool_value)
            self._primary_tool_value = tool_value
            if transition is not None:
                self._publish_actuator("input-action", tool_position_rad(transition))
        if not value.get("active"):
            self._control_session_id = None
            self._anchor_tcp = None
            self._target_tcp = None
            self._frozen_session = None
            self._hold_sent = False
            self._enqueue_motion_state()
            return
        if self._control_mode != "relative" or self._motion_status["state"] in {
            "planning",
            "executing",
        }:
            return
        if session_id != self._control_session_id:
            self._control_session_id = session_id
            self._anchor_tcp = self._current_tcp
            self._frozen_session = None
            self._hold_sent = False
        if self._frozen_session == session_id or self._anchor_tcp is None:
            return
        translation = value.get("translation_m")
        if not finite(translation, 3):
            raise ValueError("RelativeToolMotion translation is invalid")
        target = target_pose(
            self._anchor_tcp,
            translation,
            float(value.get("front_pitch_rad", 0.0)),
            float(value.get("horizontal_arc_rad", 0.0)),
        )
        self._target_tcp = target
        self._controller_output_armed = True
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = (
            target.position_m
        )
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = target.orientation_xyzw
        self._pose_publisher.publish(message)
        self._enqueue_motion_state()

    def _set_control_mode(self, request: dict[str, Any]) -> None:
        request_id = str(request.get("request_id", ""))
        mode = request.get("mode")
        error: str | None = None
        if mode not in {"relative", "manual"}:
            error = f"unknown control mode {mode}"
        else:
            next_config = MotionConfig(
                config_version=self._config.config_version + 1,
                control_mode=mode,
            )
            try:
                save_motion_config(self._config_path, next_config)
            except OSError as write_error:
                error = f"保存 motion 配置失败：{write_error}"
            else:
                self._config = next_config
                self._control_mode = mode
                self._control_session_id = None
                self._anchor_tcp = None
                self._target_tcp = None
        self._enqueue(
            "mode_request_result",
            {
                "schema_version": SCHEMA_VERSION,
                "request_id": request_id,
                "acknowledged_action": "apply",
                "value": self._motion_state(),
                "original_error": error,
            },
        )
        self._enqueue_motion_state()

    def _handle_actuator_request(self, request: dict[str, Any]) -> None:
        request_id = str(request.get("request_id", ""))
        position = request.get("position_rad")
        if (
            request.get("model_revision") != MODEL_REVISION
            or request.get("actuator_key") != GRIPPER_KEY
        ):
            self._actuator_status = {
                "schema_version": SCHEMA_VERSION,
                "request_id": request_id,
                "actuator_key": str(request.get("actuator_key", "")),
                "state": "failed",
                "result_code": "model_mismatch",
                "result_message": "request does not match StarArm-102 gripper",
            }
        elif (
            not isinstance(position, (int, float))
            or isinstance(position, bool)
            or not math.isfinite(position)
        ):
            self._actuator_status = {
                "schema_version": SCHEMA_VERSION,
                "request_id": request_id,
                "actuator_key": GRIPPER_KEY,
                "state": "failed",
                "result_code": "invalid_position",
                "result_message": "actuator position must be a finite number",
            }
        elif not self._hand_trajectory_client.server_is_ready():
            self._actuator_status = {
                "schema_version": SCHEMA_VERSION,
                "request_id": request_id,
                "actuator_key": GRIPPER_KEY,
                "state": "failed",
                "result_code": "controller_unavailable",
                "result_message": "hand_controller 尚未就绪",
            }
        else:
            self._publish_actuator(request_id, float(position))
        self._enqueue("actuator_status", self._actuator_status)
        self._enqueue_motion_state()

    def _publish_actuator(self, request_id: str, position_rad: float) -> None:
        self._controller_output_armed = True
        trajectory = JointTrajectory()
        trajectory.joint_names = [GRIPPER_JOINT]
        point = JointTrajectoryPoint()
        point.positions = [position_rad]
        point.time_from_start.nanosec = 250_000_000
        trajectory.points = [point]
        self._hand_publisher.publish(trajectory)
        self._actuator_status = {
            "schema_version": SCHEMA_VERSION,
            "request_id": request_id,
            "actuator_key": GRIPPER_KEY,
            "state": "succeeded",
            "result_code": None,
            "result_message": None,
        }

    def _handle_motion_request(self, request: dict[str, Any]) -> None:
        request_id = str(request.get("request_id", ""))
        if request.get("action") == "cancel":
            self._cancel_motion(request_id)
            return
        if request.get("action") != "apply":
            self._set_motion_failed(
                request_id,
                f"unsupported motion action {request.get('action')}",
                "apply",
            )
            return
        if (
            request.get("model_revision") != MODEL_REVISION
            or self._latest_arm_state is None
        ):
            self._set_motion_failed(
                request_id, "model revision mismatch or no ArmState", "apply"
            )
            return
        options = request.get("options", {})
        if not isinstance(options, dict):
            self._set_motion_failed(
                request_id, "motion options must be an object", "apply"
            )
            return
        unknown_options = sorted(
            set(options) - {"velocity_scaling", "acceleration_scaling"}
        )
        if unknown_options:
            self._set_motion_failed(
                request_id, f"unknown motion options {unknown_options}", "apply"
            )
            return
        if any(
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(value)
            for value in options.values()
        ):
            self._set_motion_failed(
                request_id, "motion option values must be finite numbers", "apply"
            )
            return
        target_by_key = dict(
            zip(JOINTS, self._latest_arm_state["joints_rad"], strict=True)
        )
        joints = request.get("joints", [])
        if not isinstance(joints, list):
            self._set_motion_failed(request_id, "joints must be a list", "apply")
            return
        seen = set()
        for item in joints:
            if not isinstance(item, dict):
                self._set_motion_failed(
                    request_id, "joint target must be an object", "apply"
                )
                return
            key = item.get("joint_key")
            if key not in target_by_key:
                self._set_motion_failed(request_id, f"unknown joint key {key}", "apply")
                return
            if key in seen:
                self._set_motion_failed(
                    request_id, f"duplicate joint key {key}", "apply"
                )
                return
            position = item.get("position_rad")
            if (
                not isinstance(position, (int, float))
                or isinstance(position, bool)
                or not math.isfinite(position)
            ):
                self._set_motion_failed(
                    request_id, f"joint {key} position must be a finite number", "apply"
                )
                return
            seen.add(key)
            target_by_key[key] = float(position)
        self._begin_motion_plan(
            request_id, [target_by_key[key] for key in JOINTS], options
        )

    def _begin_motion_plan(
        self, request_id: str, target: list[float], options: dict[str, Any]
    ) -> None:
        current = (
            None
            if self._latest_arm_state is None
            else list(self._latest_arm_state["joints_rad"])
        )
        if (
            current is None
            or not self._move_group_client.server_is_ready()
            or not self._arm_trajectory_client.server_is_ready()
            or not self._servo_pause_client.service_is_ready()
        ):
            self._set_motion_failed(
                request_id, "缺少当前关节反馈或 MoveGroup 尚未就绪", "apply"
            )
            return
        self._motion_target = target
        self._motion_status = {
            "schema_version": SCHEMA_VERSION,
            "request_id": request_id,
            "acknowledged_action": "apply",
            "state": "planning",
            "backend_name": "moveit",
            "result_code": None,
            "result_message": "MoveIt 正在规划普通关节目标",
            "trajectory_points": None,
            "planned_duration_s": None,
        }
        self._enqueue("motion_status", self._motion_status)
        request = SetBool.Request()
        request.data = True
        self._servo_pause_client.call_async(request).add_done_callback(
            lambda done: self._servo_plan_pause_response(
                request_id, current, target, options, done
            )
        )

    def _servo_plan_pause_response(
        self,
        request_id: str,
        current: list[float],
        target: list[float],
        options: dict[str, Any],
        future: Any,
    ) -> None:
        try:
            response = future.result()
        except Exception as error:
            self._set_motion_failed(
                request_id, f"暂停 MoveIt Servo 失败：{error}", "apply"
            )
            return
        if not response.success:
            self._set_motion_failed(
                request_id, f"MoveIt Servo 拒绝暂停：{response.message}", "apply"
            )
            return
        self._servo_paused = True
        if (
            self._motion_status["request_id"] != request_id
            or self._motion_status["state"] != "planning"
        ):
            self._resume_servo()
            return
        self._send_motion_plan(request_id, current, target, options, None, ())

    def _send_motion_plan(
        self,
        request_id: str,
        current: list[float],
        target: list[float],
        options: dict[str, Any],
        matrix: Any | None,
        collision_pairs: tuple[tuple[str, str], ...],
    ) -> None:
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.start_state.joint_state.name = list(JOINTS)
        goal.request.start_state.joint_state.position = current
        goal.request.start_state.is_diff = False
        if "velocity_scaling" in options:
            goal.request.max_velocity_scaling_factor = float(
                options["velocity_scaling"]
            )
        if "acceleration_scaling" in options:
            goal.request.max_acceleration_scaling_factor = float(
                options["acceleration_scaling"]
            )
        constraints = Constraints()
        constraints.name = "joint_target"
        for name, position in zip(JOINTS, target, strict=True):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        if matrix is not None:
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.planning_options.planning_scene_diff.allowed_collision_matrix = matrix
        future = self._move_group_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._plan_goal_response(
                request_id, options, collision_pairs, done
            )
        )

    def _plan_goal_response(
        self,
        request_id: str,
        options: dict[str, Any],
        pairs: tuple[tuple[str, str], ...],
        future: Any,
    ) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self._set_motion_failed(request_id, f"提交规划失败：{error}", "apply")
            return
        if (
            self._motion_status["request_id"] != request_id
            or self._motion_status["state"] == "cancelled"
        ):
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._set_motion_failed(request_id, "MoveGroup 拒绝普通运动规划", "apply")
            return
        self._plan_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda done: self._plan_result(request_id, options, pairs, done)
        )

    def _plan_result(
        self,
        request_id: str,
        options: dict[str, Any],
        pairs: tuple[tuple[str, str], ...],
        future: Any,
    ) -> None:
        if (
            self._motion_status["request_id"] != request_id
            or self._motion_status["state"] == "cancelled"
        ):
            return
        self._plan_goal_handle = None
        try:
            result = future.result().result
        except Exception as error:
            self._set_motion_failed(request_id, f"规划结果读取失败：{error}", "apply")
            return
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            if pairs:
                self._set_motion_failed(
                    request_id,
                    f"MoveIt 规划失败，错误码 {result.error_code.val}；本次已临时放行 {self._format_pairs(pairs)}",
                    "apply",
                )
            else:
                self._begin_collision_retry(request_id, options, result.error_code.val)
            return
        points = result.planned_trajectory.joint_trajectory.points
        if not points:
            self._set_motion_failed(request_id, "MoveIt 返回空轨迹", "apply")
            return
        end = points[-1].time_from_start
        self._planned_trajectory = result.planned_trajectory
        self._motion_status.update(
            trajectory_points=len(points),
            planned_duration_s=end.sec + end.nanosec * 1e-9,
            result_message="普通运动规划通过"
            if not pairs
            else f"规划通过；本次临时放行 {self._format_pairs(pairs)}",
        )
        self._begin_motion_execute(request_id)

    def _begin_collision_retry(
        self, request_id: str, options: dict[str, Any], error_code: int
    ) -> None:
        current = (
            None
            if self._latest_arm_state is None
            else self._latest_arm_state["joints_rad"]
        )
        if current is None or not self._state_validity_client.service_is_ready():
            self._set_motion_failed(
                request_id,
                f"MoveIt 规划失败，错误码 {error_code}；状态有效性服务尚未就绪",
                "apply",
            )
            return
        request = GetStateValidity.Request()
        request.robot_state.joint_state.name = list(JOINTS)
        request.robot_state.joint_state.position = list(current)
        request.robot_state.is_diff = False
        request.group_name = "arm"
        self._state_validity_client.call_async(request).add_done_callback(
            lambda done: self._collision_validity_result(
                request_id, options, error_code, done
            )
        )

    def _collision_validity_result(
        self, request_id: str, options: dict[str, Any], error_code: int, future: Any
    ) -> None:
        if self._motion_status["request_id"] != request_id:
            return
        try:
            response = future.result()
        except Exception as error:
            self._set_motion_failed(
                request_id, f"读取 MoveIt 起始碰撞失败：{error}", "apply"
            )
            return
        pairs = tuple(
            sorted(
                {
                    tuple(sorted((c.contact_body_1, c.contact_body_2)))
                    for c in response.contacts
                    if c.body_type_1 == c.ROBOT_LINK
                    and c.body_type_2 == c.ROBOT_LINK
                    and c.contact_body_1 != c.contact_body_2
                }
            )
        )
        if response.valid or not pairs:
            self._set_motion_failed(
                request_id,
                f"MoveIt 规划失败，错误码 {error_code}；未检测到起始自碰撞",
                "apply",
            )
            return
        if not self._planning_scene_client.service_is_ready():
            self._set_motion_failed(request_id, "MoveIt 规划场景服务尚未就绪", "apply")
            return
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        self._planning_scene_client.call_async(request).add_done_callback(
            lambda done: self._collision_scene_result(request_id, options, pairs, done)
        )

    def _collision_scene_result(
        self,
        request_id: str,
        options: dict[str, Any],
        pairs: tuple[tuple[str, str], ...],
        future: Any,
    ) -> None:
        if self._motion_status["request_id"] != request_id:
            return
        try:
            matrix = future.result().scene.allowed_collision_matrix
            self._allow_collision_pairs(matrix, pairs)
        except Exception as error:
            self._set_motion_failed(
                request_id, f"构造 MoveIt 临时碰撞矩阵失败：{error}", "apply"
            )
            return
        if self._latest_arm_state is None or self._motion_target is None:
            self._set_motion_failed(request_id, "重试规划时缺少关节状态", "apply")
            return
        self._send_motion_plan(
            request_id,
            list(self._latest_arm_state["joints_rad"]),
            list(self._motion_target),
            options,
            matrix,
            pairs,
        )

    @staticmethod
    def _allow_collision_pairs(matrix: Any, pairs: tuple[tuple[str, str], ...]) -> None:
        if len(matrix.entry_values) != len(matrix.entry_names) or any(
            len(row.enabled) != len(matrix.entry_names) for row in matrix.entry_values
        ):
            raise ValueError("MoveIt AllowedCollisionMatrix 不是方阵")
        for name in sorted({name for pair in pairs for name in pair}):
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:
                    row.enabled.append(False)
                entry = AllowedCollisionEntry()
                entry.enabled = [False] * len(matrix.entry_names)
                matrix.entry_values.append(entry)
        indices = {name: index for index, name in enumerate(matrix.entry_names)}
        for first, second in pairs:
            matrix.entry_values[indices[first]].enabled[indices[second]] = True
            matrix.entry_values[indices[second]].enabled[indices[first]] = True

    @staticmethod
    def _format_pairs(pairs: tuple[tuple[str, str], ...]) -> str:
        return "、".join(f"{first}↔{second}" for first, second in pairs)

    def _begin_motion_execute(self, request_id: str) -> None:
        if (
            self._planned_trajectory is None
            or not self._execute_client.server_is_ready()
            or not self._servo_paused
        ):
            self._set_motion_failed(request_id, "轨迹或执行服务尚未就绪", "apply")
            return
        self._motion_status.update(
            state="executing", result_message="正在执行普通关节轨迹"
        )
        self._enqueue("motion_status", self._motion_status)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self._planned_trajectory
        goal.controller_names = ["arm_controller"]
        self._execute_client.send_goal_async(goal).add_done_callback(
            lambda done: self._execute_goal_response(request_id, done)
        )

    def _execute_goal_response(self, request_id: str, future: Any) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self._set_motion_failed(request_id, f"提交执行失败：{error}", "apply")
            return
        if self._motion_status["request_id"] != request_id:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._set_motion_failed(request_id, "MoveIt 拒绝执行普通轨迹", "apply")
            return
        self._controller_output_armed = True
        self._execute_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda done: self._execute_result(request_id, done)
        )

    def _execute_result(self, request_id: str, future: Any) -> None:
        if self._motion_status["request_id"] != request_id:
            return
        self._execute_goal_handle = None
        try:
            result = future.result().result
        except Exception as error:
            self._set_motion_failed(request_id, f"执行结果读取失败：{error}", "apply")
            return
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self._set_motion_failed(
                request_id,
                f"普通运动执行失败，MoveIt 错误码 {result.error_code.val}",
                "apply",
            )
            return
        self._resume_servo()
        self._reset_relative_baseline()
        self._planned_trajectory = None
        self._motion_target = None
        self._motion_status.update(
            state="succeeded",
            result_code=str(result.error_code.val),
            result_message="普通运动执行完成",
        )
        self._enqueue("motion_status", self._motion_status)
        self._enqueue("motion_request_result", self._status_result())

    def _cancel_motion(self, request_id: str) -> None:
        cancelled_request_id = self._motion_status["request_id"]
        cancelled_action = self._motion_status["acknowledged_action"]
        cancelled_was_active = self._motion_status["state"] in {"planning", "executing"}
        if self._plan_goal_handle is not None:
            self._plan_goal_handle.cancel_goal_async()
            self._plan_goal_handle = None
        if self._execute_goal_handle is not None:
            self._execute_goal_handle.cancel_goal_async()
            self._execute_goal_handle = None
        self._resume_servo()
        self._reset_relative_baseline()
        self._planned_trajectory = None
        self._motion_target = None
        if cancelled_was_active and cancelled_request_id != request_id:
            cancelled = {
                **self._idle_motion_status(),
                "request_id": cancelled_request_id,
                "acknowledged_action": cancelled_action,
                "state": "cancelled",
                "result_message": "普通运动已由显式取消请求结束；机械臂保持当前位置",
            }
            self._enqueue("motion_request_result", self._status_result_for(cancelled))
        self._motion_status = {
            **self._idle_motion_status(),
            "request_id": request_id,
            "acknowledged_action": "cancel",
            "state": "cancelled",
            "result_message": "普通运动已取消；机械臂保持当前位置",
        }
        self._enqueue("motion_status", self._motion_status)
        self._enqueue("motion_request_result", self._status_result())

    def _set_motion_failed(self, request_id: str, message: str, action: str) -> None:
        self._resume_servo()
        self._reset_relative_baseline()
        self._planned_trajectory = None
        self._motion_target = None
        self._motion_status = {
            **self._idle_motion_status(),
            "request_id": request_id,
            "acknowledged_action": action,
            "state": "failed",
            "result_message": message,
        }
        self._enqueue("motion_status", self._motion_status)
        self._enqueue("motion_request_result", self._status_result())

    def _status_result(self) -> dict[str, Any]:
        return self._status_result_for(self._motion_status)

    @staticmethod
    def _status_result_for(status: dict[str, Any]) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "request_id": status["request_id"],
            "acknowledged_action": status["acknowledged_action"],
            "value": status,
            "original_error": status["result_message"]
            if status["state"] == "failed"
            else None,
        }

    def _resume_servo(self) -> None:
        if not self._servo_paused:
            return
        self._servo_paused = False
        if self._servo_pause_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self._servo_pause_client.call_async(request)

    def _reset_relative_baseline(self) -> None:
        self._control_session_id = None
        self._anchor_tcp = None
        self._target_tcp = None

    def _select_pose_commands(self) -> None:
        if (
            self._pose_commands_selected
            or self._command_request_pending
            or not self._command_client.service_is_ready()
        ):
            return
        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.POSE
        self._command_request_pending = True
        self._command_client.call_async(request).add_done_callback(
            self._pose_mode_response
        )

    def _pose_mode_response(self, future: Any) -> None:
        self._command_request_pending = False
        try:
            self._pose_commands_selected = bool(future.result().success)
        except Exception as error:
            self.get_logger().error(f"选择 MoveIt Servo Pose 模式失败：{error}")

    def _motion_state(self) -> dict[str, Any]:
        def pose(value: Pose | None) -> dict[str, Any] | None:
            if value is None:
                return None
            return {
                "frame": "base_link",
                "position_m": list(value.position_m),
                "orientation_xyzw": list(value.orientation_xyzw),
            }

        controllers_ready = (
            self._move_group_client.server_is_ready()
            and self._arm_trajectory_client.server_is_ready()
            and self._hand_trajectory_client.server_is_ready()
            and self._servo_pause_client.service_is_ready()
            and not self._controller_sync_requested
            and self._controller_sync_phase is None
        )
        return {
            "schema_version": SCHEMA_VERSION,
            "control_mode": self._control_mode,
            "current_tool_pose": pose(self._current_tcp),
            "target_tool_pose": pose(self._target_tcp),
            "control_session_id": self._control_session_id,
            "latest_motion": self._motion_status,
            "latest_actuator": self._actuator_status,
            "diagnostics": [
                {"key": "servo_status_code", "value": self._servo_code},
                {"key": "servo_status_message", "value": self._servo_message},
            ],
            "service": {
                "schema_version": SCHEMA_VERSION,
                "build_version": "0.1.0",
                "config_version": self._config.config_version,
                "running": True,
                "has_input": self._latest_arm_state is not None,
                "has_output": controllers_ready,
                "last_error": None,
                "updated_at_ns": time.time_ns(),
            },
        }

    def _enqueue_motion_state(self) -> None:
        state = self._motion_state()
        self._enqueue("service_state", state["service"])
        self._enqueue("motion_state", state)

    def _build_model_info(self) -> dict[str, Any]:
        asset_root = Path(
            os.environ.get("STARARM_MODEL_ASSETS", "/opt/robot-arm/model")
        )
        description_path = (
            next(iter(sorted(asset_root.glob("*.urdf"))), None)
            if asset_root.exists()
            else None
        )
        if description_path is None:
            raise RuntimeError(f"模型目录中没有 URDF：{asset_root}")
        import xml.etree.ElementTree as ET

        root = ET.fromstring(description_path.read_text())
        limits = {}
        for joint in root.findall("joint"):
            if (
                joint.get("name") in JOINTS
                and (limit := joint.find("limit")) is not None
            ):
                limits[joint.get("name")] = (
                    float(limit.get("lower")),
                    float(limit.get("upper")),
                )
        missing = [name for name in JOINTS if name not in limits]
        if missing:
            raise RuntimeError(f"URDF 缺少主动关节范围：{missing}")
        files = (
            sorted(
                str(path.relative_to(asset_root))
                for path in asset_root.rglob("*")
                if path.is_file()
            )
            if asset_root.exists()
            else []
        )
        digest = hashlib.sha256()
        for relative in files:
            digest.update(relative.encode())
            digest.update((asset_root / relative).read_bytes())
        manifest_hash = digest.hexdigest()
        return {
            "schema_version": SCHEMA_VERSION,
            "model_id": MODEL_ID,
            "model_revision": MODEL_REVISION,
            "display_name": "StarArm-102",
            "base_frame": "base_link",
            "tcp_frame": "link6",
            "joints": [
                {
                    "key": name,
                    "label": f"J{index + 1}",
                    "unit": "rad",
                    "minimum": limits[name][0],
                    "maximum": limits[name][1],
                }
                for index, name in enumerate(JOINTS)
            ],
            "tool_actuators": [
                {
                    "key": GRIPPER_KEY,
                    "label": "夹爪",
                    "unit": "rad",
                    "minimum": 0.0,
                    "maximum": math.radians(90),
                    "visualization_joint_key": GRIPPER_JOINT,
                }
            ],
            "named_targets": [
                {
                    "key": "start",
                    "label": "默认位",
                    "positions_rad": dict(zip(JOINTS, START_RAD, strict=True)),
                },
                {
                    "key": "test",
                    "label": "测试位",
                    "positions_rad": dict(zip(JOINTS, TEST_RAD, strict=True)),
                },
            ],
            "motion_options": [
                {
                    "key": "velocity_scaling",
                    "label": "速度倍率",
                    "unit": "ratio",
                    "minimum": 0.0,
                    "maximum": 1.0,
                    "required": False,
                },
                {
                    "key": "acceleration_scaling",
                    "label": "加速度倍率",
                    "unit": "ratio",
                    "minimum": 0.0,
                    "maximum": 1.0,
                    "required": False,
                },
            ],
            "diagnostics": [],
            "visualization": {
                "manifest_hash": manifest_hash,
                "root_path": next(
                    (name for name in files if name.endswith((".urdf", ".xacro"))), ""
                ),
                "files": files,
                "link_materials": {
                    "base_link": {
                        "color_rgb": [0.196, 0.298, 0.365],
                        "metalness": 0.16,
                        "roughness": 0.56,
                    },
                    **{
                        name: {
                            "color_rgb": [0.863, 0.906, 0.929],
                            "metalness": 0.16,
                            "roughness": 0.56,
                        }
                        for name in (
                            "link1",
                            "link2",
                            "link3",
                            "link4",
                            "link5",
                            "link6",
                        )
                    },
                    **{
                        name: {
                            "color_rgb": [0.145, 0.216, 0.275],
                            "metalness": 0.42,
                            "roughness": 0.48,
                        }
                        for name in ("link7_left", "link7_right")
                    },
                },
            },
        }

    def _handle_asset_request(self, request: dict[str, Any]) -> None:
        response = {
            "schema_version": SCHEMA_VERSION,
            "request_id": str(request.get("request_id", "")),
            "model_revision": MODEL_REVISION,
            "manifest_hash": self._model_info["visualization"]["manifest_hash"],
            "relative_path": str(request.get("relative_path", "")),
            "mime_type": None,
            "content_hash": None,
            "content": None,
            "original_error": None,
        }
        relative = Path(response["relative_path"])
        root = Path(os.environ.get("STARARM_MODEL_ASSETS", "/opt/robot-arm/model"))
        if (
            request.get("model_revision") != MODEL_REVISION
            or request.get("manifest_hash") != response["manifest_hash"]
            or relative.is_absolute()
            or ".." in relative.parts
        ):
            response["original_error"] = (
                "asset request does not match the current manifest"
            )
        else:
            try:
                content = (root / relative).read_bytes()
                response["content"] = list(content)
                response["content_hash"] = hashlib.sha256(content).hexdigest()
                response["mime_type"] = (
                    "model/stl"
                    if relative.suffix.lower() == ".stl"
                    else "application/xml"
                )
            except OSError as error:
                response["original_error"] = str(error)
        self._enqueue("model_asset_response", response)


def main(args: list[str] | None = None) -> None:
    stack = subprocess.Popen(
        ["ros2", "launch", "stararm_102_motion_node", "motion_stack.launch.py"]
    )
    node = None
    try:
        rclpy.init(args=args)
        node = MotionNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        stack.terminate()
        stack.wait()


if __name__ == "__main__":
    main()
