"""Dora-facing MoveIt/Servo node. ROS is private to this model service."""

from __future__ import annotations

import json
import math
import os
import queue
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import pyarrow as pa
import rclpy
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import SwitchController
from dora import Node as DoraNode
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .model_catalog import (
    GRIPPER_JOINT,
    GRIPPER_KEY,
    JOINTS,
    MODEL_REVISION,
    NAMED_TARGETS,
    ModelCatalog,
)
from .motion_core import (
    MotionConfig,
    Pose,
    controller_sync_required,
    load_motion_config,
    merge_controller_command,
    save_motion_config,
    target_pose,
    tool_action_transition,
    tool_position_rad,
)
from .moveit_backend import MoveItBackend, PlanningError

SCHEMA_VERSION = 2
STATE_TOPIC = "/stararm102/joint_states"
COMMAND_TOPIC = "/stararm102/joint_commands"
START_RAD, CLOSED_GRIPPER_RAD = NAMED_TARGETS["start"]


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
        self._latest_outgoing: dict[str, dict[str, Any]] = {}
        self._outgoing_lock = threading.Lock()
        self._last_service_state: dict[str, Any] | None = None
        self._motion_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="stararm-moveit"
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
        self._sequence = 0
        self._last_controller_command: tuple[tuple[float, ...], float] | None = None
        self._servo_code: int | None = None
        self._servo_message: str | None = None
        self._servo_paused = False
        self._pose_commands_selected = False
        self._command_request_pending = False
        self._motion_status = self._idle_motion_status()
        self._actuator_status: dict[str, Any] | None = None
        self._motion_actuator_target: float | None = None

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
        self._arm_trajectory_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self._hand_trajectory_client = ActionClient(
            self, FollowJointTrajectory, "/hand_controller/follow_joint_trajectory"
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_timer(0.2, self._maintain_ros_interfaces)
        self._moveit = MoveItBackend(self)
        self._catalog = ModelCatalog()
        self._model_info = self._catalog.model_info()
        self._enqueue("robot_model_info", self._model_info)
        self._enqueue_motion_state(force_service=True)
        self._dora_thread = threading.Thread(
            target=self._dora_loop, name="stararm-motion-dora", daemon=True
        )
        self._dora_thread.start()

    def destroy_node(self) -> bool:
        self._stop.set()
        self._moveit.cancel()
        self._motion_executor.shutdown(wait=True, cancel_futures=True)
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
        if output in {"motion_state", "service_state", "robot_model_info"}:
            with self._outgoing_lock:
                self._latest_outgoing[output] = value
            return
        self._outgoing.put((output, value))

    def _next_output(self) -> tuple[str, dict[str, Any]] | None:
        try:
            return self._outgoing.get_nowait()
        except queue.Empty:
            pass
        with self._outgoing_lock:
            if not self._latest_outgoing:
                return None
            return self._latest_outgoing.popitem()

    def _dora_loop(self) -> None:
        dora = DoraNode()
        while not self._stop.is_set():
            event = dora.next(0.01)
            if event is not None and event["type"] == "STOP":
                self._stop.set()
                if rclpy.ok():
                    rclpy.shutdown()
                break
            if event is not None and event["type"] == "INPUT":
                try:
                    self._handle_dora_input(event["id"], arrow_decode(event["value"]))
                except Exception as error:
                    self.get_logger().error(
                        f"Dora input {event.get('id')} failed: {error}"
                    )
            if pending := self._next_output():
                dora.send_output(pending[0], arrow_encode(pending[1]))

    def _handle_dora_input(self, input_id: str, value: dict[str, Any]) -> None:
        with self._lock:
            if input_id == "arm_state":
                self._apply_arm_state(value)
            elif input_id == "transformed_control":
                self._apply_transformed_control(value)
            elif input_id == "set_control_mode":
                self._set_control_mode(value)
            elif input_id == "motion_request":
                self._handle_motion_request(value)
            elif input_id == "prepare_relative":
                self._prepare_relative_control(value)
            elif input_id == "tool_actuator_request":
                self._handle_actuator_request(value)
            elif input_id == "model_asset_request":
                self._handle_asset_request(value)
            elif input_id == "snapshot":
                self._enqueue("robot_model_info", self._model_info)
                self._enqueue_motion_state(force_service=True)

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
                current_joints = list(self._latest_arm_state["joints_rad"])
                current_actuator = float(self._latest_arm_state["actuators_rad"][0])
            else:
                current_joints = list(self._last_controller_command[0])
                current_actuator = self._last_controller_command[1]
            merged = merge_controller_command(
                list(message.name),
                list(message.position),
                JOINTS,
                GRIPPER_JOINT,
                current_joints,
                current_actuator,
            )
            if merged is None:
                return
            joints, actuator = merged
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
            self._enqueue_motion_state()

    def _apply_transformed_control(self, value: dict[str, Any]) -> None:
        session_id = value.get("control_session_id")
        if not value.get("active"):
            self._control_session_id = None
            self._anchor_tcp = None
            self._target_tcp = None
            self._enqueue_motion_state()
            return
        actuator_actions = value.get("actuator_actions", {})
        open_tool = actuator_actions.get("primary_tool_open", {})
        tool = actuator_actions.get("primary_tool", {})
        if (
            open_tool.get("is_active")
            and open_tool.get("changed_since_last_sync")
            and open_tool.get("value")
        ):
            self._primary_tool_value = 0.0
            self._publish_actuator("input-action", tool_position_rad(0.0))
        elif tool.get("is_active"):
            tool_value = float(tool.get("value", 0.0))
            transition = tool_action_transition(self._primary_tool_value, tool_value)
            self._primary_tool_value = tool_value
            if transition is not None:
                self._publish_actuator("input-action", tool_position_rad(transition))
        if self._control_mode != "relative" or self._motion_status["state"] in {
            "planning",
            "executing",
        }:
            return
        if session_id != self._control_session_id:
            self._control_session_id = session_id
            self._anchor_tcp = self._current_tcp
        if self._anchor_tcp is None:
            return
        translation = value.get("translation_m")
        if not finite(translation, 3):
            raise ValueError("TransformedControlFrame translation is invalid")
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
        error = self._apply_control_mode(request.get("mode"))
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

    def _apply_control_mode(self, mode: Any) -> str | None:
        if mode not in {"relative", "manual"}:
            return f"unknown control mode {mode}"
        next_config = MotionConfig(
            config_version=self._config.config_version + 1,
            control_mode=mode,
        )
        try:
            save_motion_config(self._config_path, next_config)
        except OSError as write_error:
            return f"保存 motion 配置失败：{write_error}"
        self._config = next_config
        self._control_mode = mode
        self._control_session_id = None
        self._anchor_tcp = None
        self._target_tcp = None
        return None

    def _prepare_relative_control(self, request: dict[str, Any]) -> None:
        request_id = str(request.get("request_id", ""))
        error = self._apply_control_mode("manual")
        if error is not None:
            self._set_motion_failed(request_id, error, "apply")
            return
        self._enqueue_motion_state()
        self._motion_actuator_target = CLOSED_GRIPPER_RAD
        self._begin_motion_plan(
            request_id, list(START_RAD), {}, complete_in_relative_mode=True
        )

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
        point.time_from_start.nanosec = 10_000_000
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
        actuator_by_key = {
            GRIPPER_KEY: float(self._latest_arm_state["actuators_rad"][0])
        }
        actuators = request.get("actuators", [])
        if not isinstance(actuators, list):
            self._set_motion_failed(request_id, "actuators must be a list", "apply")
            return
        seen.clear()
        for item in actuators:
            if not isinstance(item, dict):
                self._set_motion_failed(
                    request_id, "actuator target must be an object", "apply"
                )
                return
            key = item.get("actuator_key")
            if key not in actuator_by_key:
                self._set_motion_failed(
                    request_id, f"unknown actuator key {key}", "apply"
                )
                return
            if key in seen:
                self._set_motion_failed(
                    request_id, f"duplicate actuator key {key}", "apply"
                )
                return
            position = item.get("position_rad")
            if (
                not isinstance(position, (int, float))
                or isinstance(position, bool)
                or not math.isfinite(position)
            ):
                self._set_motion_failed(
                    request_id,
                    f"actuator {key} position must be a finite number",
                    "apply",
                )
                return
            seen.add(key)
            actuator_by_key[key] = float(position)
        self._motion_actuator_target = actuator_by_key[GRIPPER_KEY]
        self._begin_motion_plan(
            request_id, [target_by_key[key] for key in JOINTS], options
        )

    def _begin_motion_plan(
        self,
        request_id: str,
        target: list[float],
        options: dict[str, Any],
        *,
        complete_in_relative_mode: bool = False,
    ) -> None:
        if (
            self._latest_arm_state is None
            or not self._moveit.ready()
            or not self._arm_trajectory_client.server_is_ready()
            or not self._hand_trajectory_client.server_is_ready()
            or not self._servo_pause_client.service_is_ready()
        ):
            self._set_motion_failed(
                request_id, "缺少当前关节反馈或轨迹控制器尚未就绪", "apply"
            )
            return
        current = list(self._latest_arm_state["joints_rad"])
        self._motion_status = {
            **self._idle_motion_status(),
            "request_id": request_id,
            "state": "planning",
            "result_message": "MoveIt 正在规划普通关节目标",
        }
        self._enqueue("motion_status", self._motion_status)
        request = SetBool.Request()
        request.data = True
        self._servo_pause_client.call_async(request).add_done_callback(
            lambda done: self._servo_plan_pause_response(
                request_id,
                current,
                target,
                options,
                complete_in_relative_mode,
                done,
            )
        )

    def _servo_plan_pause_response(
        self,
        request_id: str,
        current: list[float],
        target: list[float],
        options: dict[str, Any],
        complete_in_relative_mode: bool,
        future: Any,
    ) -> None:
        with self._lock:
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
            if not self._motion_is(request_id, "planning"):
                self._resume_servo()
                return
            self._motion_executor.submit(
                self._run_motion,
                request_id,
                current,
                target,
                options,
                complete_in_relative_mode,
            )

    def _run_motion(
        self,
        request_id: str,
        current: list[float],
        target: list[float],
        options: dict[str, Any],
        complete_in_relative_mode: bool,
    ) -> None:
        try:
            plan = self._moveit.plan(current, target, options)
        except PlanningError as error:
            message = str(error)
            if error.collision_pairs:
                message += (
                    f"；本次已临时放行 {self._format_pairs(error.collision_pairs)}"
                )
            self._fail_active_motion(request_id, message)
            return
        except Exception as error:
            self._fail_active_motion(request_id, f"MoveIt 规划失败：{error}")
            return

        with self._lock:
            if not self._motion_is(request_id, "planning"):
                return
            message = "普通运动规划通过"
            if plan.collision_pairs:
                message += f"；本次临时放行 {self._format_pairs(plan.collision_pairs)}"
            self._motion_status.update(
                state="executing",
                trajectory_points=plan.points,
                planned_duration_s=plan.duration_s,
                result_message=message,
            )
            self._enqueue("motion_status", self._motion_status)
            if self._motion_actuator_target is not None:
                self._publish_actuator(request_id, self._motion_actuator_target)
            self._controller_output_armed = True

        try:
            result = self._moveit.execute(plan)
        except Exception as error:
            self._fail_active_motion(request_id, f"普通运动执行失败：{error}")
            return

        with self._lock:
            if not self._motion_is(request_id, "executing"):
                return
            if not result.success:
                self._set_motion_failed(
                    request_id,
                    f"普通运动执行失败，MoveIt 错误码 {result.code}",
                    "apply",
                )
                return
            if complete_in_relative_mode:
                error = self._apply_control_mode("relative")
                if error is not None:
                    self._set_motion_failed(request_id, error, "apply")
                    return
            self._resume_servo()
            self._reset_relative_baseline()
            self._motion_actuator_target = None
            self._motion_status.update(
                state="succeeded",
                result_code=str(result.code),
                result_message="普通运动执行完成",
            )
            self._enqueue("motion_status", self._motion_status)
            self._enqueue("motion_request_result", self._status_result())

    def _motion_is(self, request_id: str, state: str) -> bool:
        return (
            self._motion_status["request_id"] == request_id
            and self._motion_status["state"] == state
        )

    def _fail_active_motion(self, request_id: str, message: str) -> None:
        with self._lock:
            if self._motion_status["request_id"] == request_id:
                self._set_motion_failed(request_id, message, "apply")

    @staticmethod
    def _format_pairs(pairs: tuple[tuple[str, str], ...]) -> str:
        return "、".join(f"{first}↔{second}" for first, second in pairs)

    def _cancel_motion(self, request_id: str) -> None:
        cancelled_request_id = self._motion_status["request_id"]
        cancelled_action = self._motion_status["acknowledged_action"]
        cancelled_was_active = self._motion_status["state"] in {
            "planning",
            "executing",
        }
        self._moveit.cancel()
        self._resume_servo()
        self._reset_relative_baseline()
        self._motion_actuator_target = None
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
        self._motion_actuator_target = None
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
            self._moveit.ready()
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

    def _enqueue_motion_state(self, *, force_service: bool = False) -> None:
        state = self._motion_state()
        service = state["service"]
        identity = {
            key: value for key, value in service.items() if key != "updated_at_ns"
        }
        if force_service or identity != self._last_service_state:
            self._last_service_state = identity
            self._enqueue("service_state", service)
        self._enqueue("motion_state", state)

    def _handle_asset_request(self, request: dict[str, Any]) -> None:
        self._enqueue("model_asset_response", self._catalog.asset_response(request))


def main(args: list[str] | None = None) -> None:
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


if __name__ == "__main__":
    main()
