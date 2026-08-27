"""Thin newline-delimited JSON bridge between Rust and the standard ROS APIs."""

from __future__ import annotations

import json
import math
import socket
import threading
from typing import Any

import rclpy
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

SCHEMA_VERSION = 5
START_POSITION_RAD = tuple(
    math.radians(value) for value in (0.0, 0.0, -3.0, 0.0, 0.0, 0.0)
)
GRIPPER_START_POSITION_RAD = math.radians(1.0)
ROS_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_JOINT_NAME = "joint7_left"
STATE_TOPIC = "/stararm102/joint_states"
COMMAND_TOPIC = "/stararm102/joint_commands"


def _finite_vector(value: Any, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(
            isinstance(item, (int, float))
            and not isinstance(item, bool)
            and math.isfinite(item)
            for item in value
        )
    )


def _idle_motion_status() -> dict[str, Any]:
    return {
        "request_id": 0,
        "state": "idle",
        "message": "尚未请求普通运动",
        "trajectory_points": 0,
        "duration_seconds": None,
    }


class ServoIpcBridge(Node):
    def __init__(self) -> None:
        super().__init__("stararm102_servo_ipc_bridge")
        self.declare_parameter("ipc_path", "/ipc/moveit-servo.sock")
        self._ipc_path = str(self.get_parameter("ipc_path").value)
        self._pose_publisher = self.create_publisher(
            PoseStamped, "/servo_node/pose_target_cmds", 1
        )
        self._hand_publisher = self.create_publisher(
            JointTrajectory, "/hand_controller/joint_trajectory", 1
        )
        self._state_publisher = self.create_publisher(JointState, STATE_TOPIC, 10)
        self._joint_subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        self._controller_subscription = self.create_subscription(
            JointState, COMMAND_TOPIC, self._controller_command_callback, 10
        )
        self._status_subscription = self.create_subscription(
            ServoStatus, "/servo_node/status", self._status_callback, 10
        )
        self._command_client = self.create_client(
            ServoCommandType, "/servo_node/switch_command_type"
        )
        self._servo_pause_client = self.create_client(
            SetBool, "/servo_node/pause_servo"
        )
        self._move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self._execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self._state_validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self._planning_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self._command_timer = self.create_timer(0.2, self._select_pose_commands)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._command_request_pending = False
        self._pose_commands_selected = False
        self._status_code: int | None = None
        self._status_message: str | None = None
        self._latest_joint_positions: list[float] | None = None
        self._controller_positions = list(START_POSITION_RAD)
        self._controller_gripper = GRIPPER_START_POSITION_RAD
        self._motion_status = _idle_motion_status()
        self._motion_target: list[float] | None = None
        self._planned_trajectory: Any | None = None
        self._plan_goal_handle: Any | None = None
        self._execute_goal_handle: Any | None = None
        self._servo_paused = False
        self._last_motion_command: tuple[int, str] | None = None
        self._last_gripper_command: tuple[int, float] | None = None
        self._socket: socket.socket | None = None
        self._socket_lock = threading.Lock()
        self._stop = threading.Event()
        self._reader = threading.Thread(
            target=self._ipc_loop, name="moveit-servo-ipc", daemon=True
        )
        self._reader.start()

    def destroy_node(self) -> bool:
        self._stop.set()
        self._close_socket()
        self._reader.join(timeout=2.0)
        return super().destroy_node()

    def _select_pose_commands(self) -> None:
        if self._pose_commands_selected or self._command_request_pending:
            return
        if not self._command_client.service_is_ready():
            return
        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.POSE
        self._command_request_pending = True
        future = self._command_client.call_async(request)
        future.add_done_callback(self._pose_command_response)

    def _pose_command_response(self, future: Any) -> None:
        self._command_request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"选择 MoveIt Servo Pose 模式失败：{error}")
            return
        if response.success:
            self._pose_commands_selected = True
            self.get_logger().info("MoveIt Servo 已切换到 Pose 命令模式")
        else:
            self.get_logger().error("MoveIt Servo 拒绝切换到 Pose 命令模式")

    def _status_callback(self, message: ServoStatus) -> None:
        self._status_code = int(message.code)
        self._status_message = message.message

    def _controller_command_callback(self, message: JointState) -> None:
        indices = {name: index for index, name in enumerate(message.name)}
        if len(message.position) != len(message.name) or any(
            name not in indices for name in (*ROS_JOINT_NAMES, GRIPPER_JOINT_NAME)
        ):
            return
        joints = [float(message.position[indices[name]]) for name in ROS_JOINT_NAMES]
        gripper = float(message.position[indices[GRIPPER_JOINT_NAME]])
        if _finite_vector(joints, 6) and math.isfinite(gripper):
            self._controller_positions = joints
            self._controller_gripper = gripper

    def _joint_state_callback(self, message: JointState) -> None:
        indices = {name: index for index, name in enumerate(message.name)}
        if len(message.position) != len(message.name) or any(
            name not in indices for name in (*ROS_JOINT_NAMES, GRIPPER_JOINT_NAME)
        ):
            return
        positions = [float(message.position[indices[name]]) for name in ROS_JOINT_NAMES]
        gripper = float(message.position[indices[GRIPPER_JOINT_NAME]])
        if not _finite_vector(positions, 6) or not math.isfinite(gripper):
            return
        self._latest_joint_positions = positions
        try:
            transform = self._tf_buffer.lookup_transform("base_link", "link6", Time())
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        tcp_position = [translation.x, translation.y, translation.z]
        tcp_orientation = [rotation.x, rotation.y, rotation.z, rotation.w]
        if not _finite_vector(tcp_position, 3) or not _finite_vector(
            tcp_orientation, 4
        ):
            return
        self._send_feedback(
            {
                "schema_version": SCHEMA_VERSION,
                "controller_joints_rad": list(self._controller_positions),
                "controller_gripper_position_rad": self._controller_gripper,
                "tcp_pose": {
                    "position_m": tcp_position,
                    "orientation_xyzw": tcp_orientation,
                },
                "servo_status_code": self._status_code,
                "servo_status_message": self._status_message,
                "motion_status": self._motion_status,
            }
        )

    def _send_feedback(self, feedback: dict[str, Any]) -> None:
        encoded = (json.dumps(feedback, separators=(",", ":")) + "\n").encode()
        with self._socket_lock:
            connection = self._socket
            if connection is None:
                return
            try:
                connection.sendall(encoded)
            except OSError:
                self._close_socket_locked()

    def _ipc_loop(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            try:
                connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                connection.settimeout(1.0)
                connection.connect(self._ipc_path)
                connection.settimeout(None)
                self._reset_motion_session()
                with self._socket_lock:
                    self._socket = connection
                self.get_logger().info(f"已连接 NOLO IPC：{self._ipc_path}")
                with connection.makefile("r", encoding="utf-8") as stream:
                    for line in stream:
                        if self._stop.is_set():
                            break
                        self._handle_command(line)
            except (OSError, ValueError, json.JSONDecodeError) as error:
                if not self._stop.is_set():
                    self.get_logger().warning(f"NOLO IPC 暂不可用：{error}；1 秒后重连")
            finally:
                self._close_socket()
            self._stop.wait(1.0)

    def _handle_command(self, line: str) -> None:
        command = json.loads(line)
        if command.get("schema_version") != SCHEMA_VERSION:
            raise ValueError("不支持的 IPC schema_version")
        self._publish_arm_state(command.get("arm_state"))
        self._handle_motion_request(command.get("motion_request"))
        self._handle_gripper_request(command.get("gripper_request"))
        if self._motion_status["state"] in {"planning", "executing"}:
            return
        if not command.get("enabled") or not self._pose_commands_selected:
            return
        if command.get("base_frame") != "base_link" or command.get("tcp_link") != "link6":
            raise ValueError("IPC 坐标系必须是 base_link -> link6")
        target = command.get("target_pose")
        if not isinstance(target, dict):
            raise ValueError("启用的 IPC 命令缺少 target_pose")
        position = target.get("position_m")
        orientation = target.get("orientation_xyzw")
        if not _finite_vector(position, 3) or not _finite_vector(orientation, 4):
            raise ValueError("target_pose 含非法数值")
        norm = math.sqrt(sum(value * value for value in orientation))
        if not math.isfinite(norm) or norm == 0.0:
            raise ValueError("target_pose 四元数为零")
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = position
        normalized = [value / norm for value in orientation]
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = normalized
        self._pose_publisher.publish(message)

    def _publish_arm_state(self, raw: Any) -> None:
        if not isinstance(raw, dict):
            raise ValueError("arm_state 必须是对象")
        joints = raw.get("joints_rad")
        gripper = raw.get("gripper_position_rad")
        if not _finite_vector(joints, 6) or not isinstance(
            gripper, (int, float)
        ) or isinstance(gripper, bool) or not math.isfinite(gripper):
            raise ValueError("arm_state 必须包含 J1–J6 和夹爪有限弧度值")
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [*ROS_JOINT_NAMES, GRIPPER_JOINT_NAME]
        message.position = [*map(float, joints), float(gripper)]
        self._state_publisher.publish(message)

    def _handle_gripper_request(self, raw: Any) -> None:
        if raw is None:
            return
        if not isinstance(raw, dict):
            raise ValueError("gripper_request 必须是对象")
        request_id = raw.get("request_id")
        position = raw.get("position_rad")
        if not isinstance(request_id, int) or request_id < 0:
            raise ValueError("gripper_request.request_id 非法")
        if (
            not isinstance(position, (int, float))
            or isinstance(position, bool)
            or not math.isfinite(position)
        ):
            raise ValueError("gripper_request.position_rad 必须是有限弧度值")
        key = (request_id, float(position))
        if key == self._last_gripper_command:
            return
        self._last_gripper_command = key
        trajectory = JointTrajectory()
        trajectory.joint_names = [GRIPPER_JOINT_NAME]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start.nanosec = 250_000_000
        trajectory.points = [point]
        self._hand_publisher.publish(trajectory)

    def _handle_motion_request(self, raw: Any) -> None:
        if raw is None:
            return
        if not isinstance(raw, dict):
            raise ValueError("motion_request 必须是对象")
        request_id = raw.get("request_id")
        action = raw.get("action")
        if not isinstance(request_id, int) or request_id <= 0:
            raise ValueError("motion_request.request_id 必须为正整数")
        if action not in {"plan", "cancel"}:
            raise ValueError("motion_request.action 非法")
        key = (request_id, action)
        if key == self._last_motion_command:
            return
        self._last_motion_command = key
        if action == "cancel":
            self._cancel_motion(request_id)
            return
        target = raw.get("joints_rad")
        if not _finite_vector(target, 6):
            raise ValueError("motion_request.joints_rad 必须是六个有限弧度值")
        self._begin_motion_plan(request_id, list(map(float, target)))

    def _begin_motion_plan(self, request_id: int, target: list[float]) -> None:
        self._resume_servo()
        self._planned_trajectory = None
        current = self._latest_joint_positions
        if current is None:
            self._set_motion_failed(request_id, "缺少当前关节反馈", "plan")
            return
        if not self._move_group_client.server_is_ready():
            self._set_motion_failed(request_id, "MoveGroup 规划服务尚未就绪", "plan")
            return
        self._motion_target = target
        self._motion_status = {
            "request_id": request_id,
            "acknowledged_action": "plan",
            "state": "planning",
            "message": "MoveIt 正在规划普通关节目标",
            "trajectory_points": 0,
            "duration_seconds": None,
        }
        self._send_motion_plan(request_id, list(current), target, None, ())

    def _send_motion_plan(
        self,
        request_id: int,
        current: list[float],
        target: list[float],
        allowed_collision_matrix: Any | None,
        collision_pairs: tuple[tuple[str, str], ...],
    ) -> None:
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.start_state.joint_state.name = list(ROS_JOINT_NAMES)
        goal.request.start_state.joint_state.position = current
        goal.request.start_state.is_diff = False
        constraints = Constraints()
        constraints.name = "joint_target"
        for name, position in zip(ROS_JOINT_NAMES, target, strict=True):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        if allowed_collision_matrix is not None:
            scene = goal.planning_options.planning_scene_diff
            scene.is_diff = True
            scene.allowed_collision_matrix = allowed_collision_matrix
        future = self._move_group_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self._plan_goal_response(
                request_id, collision_pairs, completed
            )
        )

    def _plan_goal_response(
        self,
        request_id: int,
        collision_pairs: tuple[tuple[str, str], ...],
        future: Any,
    ) -> None:
        try:
            handle = future.result()
        except Exception as error:
            if self._motion_status["request_id"] == request_id:
                self._set_motion_failed(request_id, f"提交规划失败：{error}", "plan")
            return
        if (
            self._motion_status["request_id"] != request_id
            or self._motion_status["state"] == "cancelled"
        ):
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._set_motion_failed(request_id, "MoveGroup 拒绝普通运动规划", "plan")
            return
        self._plan_goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(
            lambda completed: self._plan_result(request_id, collision_pairs, completed)
        )

    def _plan_result(
        self,
        request_id: int,
        collision_pairs: tuple[tuple[str, str], ...],
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
            self._set_motion_failed(request_id, f"规划结果读取失败：{error}", "plan")
            return
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            if collision_pairs:
                self._set_motion_failed(
                    request_id,
                    (
                        f"MoveIt 普通运动规划失败，错误码 {result.error_code.val}；"
                        f"本次已临时放行：{self._format_collision_pairs(collision_pairs)}"
                    ),
                    "plan",
                )
            else:
                self._begin_collision_retry(request_id, result.error_code.val)
            return
        points = result.planned_trajectory.joint_trajectory.points
        if not points:
            self._set_motion_failed(request_id, "MoveIt 返回空轨迹", "plan")
            return
        last = points[-1].time_from_start
        duration = last.sec + last.nanosec * 1.0e-9
        self._planned_trajectory = result.planned_trajectory
        self._motion_status = {
            "request_id": request_id,
            "acknowledged_action": "plan",
            "state": "planning",
            "message": (
                "普通运动规划通过"
                if not collision_pairs
                else (
                    "普通运动规划通过；仅本次临时放行："
                    f"{self._format_collision_pairs(collision_pairs)}"
                )
            ),
            "trajectory_points": len(points),
            "duration_seconds": duration,
        }
        self._begin_motion_execute(request_id)

    def _begin_collision_retry(self, request_id: int, error_code: int) -> None:
        current = self._latest_joint_positions
        if current is None:
            self._set_motion_failed(
                request_id,
                f"MoveIt 规划失败，错误码 {error_code}；缺少当前关节反馈",
                "plan",
            )
            return
        if not self._state_validity_client.service_is_ready():
            self._set_motion_failed(
                request_id,
                f"MoveIt 规划失败，错误码 {error_code}；状态有效性服务尚未就绪",
                "plan",
            )
            return
        request = GetStateValidity.Request()
        request.robot_state.joint_state.name = list(ROS_JOINT_NAMES)
        request.robot_state.joint_state.position = list(current)
        request.robot_state.is_diff = False
        request.group_name = "arm"
        future = self._state_validity_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._collision_validity_result(
                request_id, error_code, completed
            )
        )

    def _collision_validity_result(
        self, request_id: int, error_code: int, future: Any
    ) -> None:
        if self._motion_status["request_id"] != request_id:
            return
        try:
            response = future.result()
        except Exception as error:
            self._set_motion_failed(
                request_id, f"读取 MoveIt 起始碰撞失败：{error}", "plan"
            )
            return
        pairs = tuple(
            sorted(
                {
                    tuple(sorted((contact.contact_body_1, contact.contact_body_2)))
                    for contact in response.contacts
                    if contact.body_type_1 == contact.ROBOT_LINK
                    and contact.body_type_2 == contact.ROBOT_LINK
                    and contact.contact_body_1 != contact.contact_body_2
                }
            )
        )
        if response.valid or not pairs:
            self._set_motion_failed(
                request_id,
                f"MoveIt 规划失败，错误码 {error_code}；未检测到起始自碰撞",
                "plan",
            )
            return
        if not self._planning_scene_client.service_is_ready():
            self._set_motion_failed(request_id, "MoveIt 规划场景服务尚未就绪", "plan")
            return
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        scene_future = self._planning_scene_client.call_async(request)
        scene_future.add_done_callback(
            lambda completed: self._collision_scene_result(
                request_id, pairs, completed
            )
        )

    def _collision_scene_result(
        self,
        request_id: int,
        pairs: tuple[tuple[str, str], ...],
        future: Any,
    ) -> None:
        if self._motion_status["request_id"] != request_id:
            return
        try:
            response = future.result()
            matrix = response.scene.allowed_collision_matrix
            self._allow_collision_pairs(matrix, pairs)
        except Exception as error:
            self._set_motion_failed(
                request_id, f"构造 MoveIt 临时碰撞矩阵失败：{error}", "plan"
            )
            return
        current = self._latest_joint_positions
        target = self._motion_target
        if current is None or target is None:
            self._set_motion_failed(request_id, "重试规划时缺少关节状态", "plan")
            return
        self._send_motion_plan(request_id, list(current), list(target), matrix, pairs)

    @staticmethod
    def _allow_collision_pairs(matrix: Any, pairs: tuple[tuple[str, str], ...]) -> None:
        names = matrix.entry_names
        values = matrix.entry_values
        if len(values) != len(names) or any(
            len(row.enabled) != len(names) for row in values
        ):
            raise ValueError("MoveIt AllowedCollisionMatrix 不是方阵")
        for name in sorted({name for pair in pairs for name in pair}):
            if name in names:
                continue
            names.append(name)
            for row in values:
                row.enabled.append(False)
            entry = AllowedCollisionEntry()
            entry.enabled = [False] * len(names)
            values.append(entry)
        indices = {name: index for index, name in enumerate(names)}
        for first, second in pairs:
            first_index = indices[first]
            second_index = indices[second]
            values[first_index].enabled[second_index] = True
            values[second_index].enabled[first_index] = True

    @staticmethod
    def _format_collision_pairs(pairs: tuple[tuple[str, str], ...]) -> str:
        return "、".join(f"{first}↔{second}" for first, second in pairs)

    def _begin_motion_execute(self, request_id: int) -> None:
        if self._planned_trajectory is None:
            self._set_motion_failed(request_id, "没有可执行的规划轨迹", "plan")
            return
        if not self._execute_trajectory_client.server_is_ready():
            self._set_motion_failed(request_id, "MoveIt 轨迹执行服务尚未就绪", "plan")
            return
        if not self._servo_pause_client.service_is_ready():
            self._set_motion_failed(request_id, "MoveIt Servo 暂停服务尚未就绪", "plan")
            return
        self._motion_status = {
            **self._motion_status,
            "state": "executing",
            "message": "正在执行普通关节轨迹",
        }
        request = SetBool.Request()
        request.data = True
        future = self._servo_pause_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._servo_pause_response(request_id, completed)
        )

    def _servo_pause_response(self, request_id: int, future: Any) -> None:
        try:
            response = future.result()
        except Exception as error:
            self._set_motion_failed(request_id, f"暂停 MoveIt Servo 失败：{error}", "plan")
            return
        if not response.success:
            self._set_motion_failed(
                request_id, f"MoveIt Servo 拒绝暂停：{response.message}", "plan"
            )
            return
        self._servo_paused = True
        if (
            self._motion_status["request_id"] != request_id
            or self._motion_status["state"] != "executing"
        ):
            self._resume_servo()
            return
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self._planned_trajectory
        goal.controller_names = ["arm_controller"]
        future = self._execute_trajectory_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self._execute_goal_response(request_id, completed)
        )

    def _execute_goal_response(self, request_id: int, future: Any) -> None:
        try:
            handle = future.result()
        except Exception as error:
            self._set_motion_failed(request_id, f"提交执行失败：{error}", "plan")
            return
        if self._motion_status["request_id"] != request_id:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._set_motion_failed(request_id, "MoveIt 拒绝执行普通轨迹", "plan")
            return
        self._execute_goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(
            lambda completed: self._execute_result(request_id, completed)
        )

    def _execute_result(self, request_id: int, future: Any) -> None:
        if self._motion_status["request_id"] != request_id:
            return
        self._execute_goal_handle = None
        try:
            result = future.result().result
        except Exception as error:
            self._set_motion_failed(request_id, f"执行结果读取失败：{error}", "plan")
            return
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self._set_motion_failed(
                request_id,
                f"普通运动执行失败，MoveIt 错误码 {result.error_code.val}",
                "plan",
            )
            return
        self._finish_motion("普通运动执行完成")

    def _finish_motion(self, message: str) -> None:
        self._resume_servo()
        self._motion_status = {
            **self._motion_status,
            "state": "succeeded",
            "message": message,
        }
        self._planned_trajectory = None
        self._motion_target = None

    def _cancel_motion(self, request_id: int) -> None:
        if self._plan_goal_handle is not None:
            self._plan_goal_handle.cancel_goal_async()
            self._plan_goal_handle = None
        if self._execute_goal_handle is not None:
            self._execute_goal_handle.cancel_goal_async()
            self._execute_goal_handle = None
        self._resume_servo()
        self._planned_trajectory = None
        self._motion_target = None
        self._motion_status = {
            "request_id": request_id,
            "acknowledged_action": "cancel",
            "state": "cancelled",
            "message": "普通运动已取消；机械臂保持当前位置",
            "trajectory_points": 0,
            "duration_seconds": None,
        }

    def _set_motion_failed(self, request_id: int, message: str, action: Any) -> None:
        self._resume_servo()
        self._motion_status = {
            **self._motion_status,
            "request_id": request_id,
            "acknowledged_action": action,
            "state": "failed",
            "message": message,
        }

    def _resume_servo(self) -> None:
        if not self._servo_paused:
            return
        self._servo_paused = False
        if not self._servo_pause_client.service_is_ready():
            self.get_logger().error("普通运动结束后 MoveIt Servo 恢复服务未就绪")
            return
        request = SetBool.Request()
        request.data = False
        future = self._servo_pause_client.call_async(request)
        future.add_done_callback(self._servo_resume_response)

    def _servo_resume_response(self, future: Any) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"恢复 MoveIt Servo 失败：{error}")
            return
        if not response.success:
            self.get_logger().error(f"MoveIt Servo 拒绝恢复：{response.message}")

    def _close_socket(self) -> None:
        with self._socket_lock:
            self._close_socket_locked()

    def _close_socket_locked(self) -> None:
        if self._plan_goal_handle is not None:
            self._plan_goal_handle.cancel_goal_async()
            self._plan_goal_handle = None
        if self._execute_goal_handle is not None:
            self._execute_goal_handle.cancel_goal_async()
            self._execute_goal_handle = None
        self._resume_servo()
        self._planned_trajectory = None
        self._motion_target = None
        connection = self._socket
        self._socket = None
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()

    def _reset_motion_session(self) -> None:
        self._resume_servo()
        self._planned_trajectory = None
        self._plan_goal_handle = None
        self._execute_goal_handle = None
        self._motion_target = None
        self._last_motion_command = None
        self._last_gripper_command = None
        self._motion_status = _idle_motion_status()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ServoIpcBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
