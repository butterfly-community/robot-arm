"""Bridge newline-delimited JSON IPC to the standard MoveIt Servo ROS API."""

import json
import math
import socket
import threading
import time
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
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

SCHEMA_VERSION = 2
HOME_ZERO_TOLERANCE_RAD = math.radians(1.0)
ROS_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
MODEL_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)


def _finite_vector(value: Any, length: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == length
        and all(isinstance(item, (int, float)) and math.isfinite(item) for item in value)
    )


class ServoIpcBridge(Node):
    def __init__(self) -> None:
        super().__init__("stararm102_servo_ipc_bridge")
        self.declare_parameter("ipc_path", "/ipc/moveit-servo.sock")
        self.declare_parameter("simulation_only", True)
        self._ipc_path = self.get_parameter("ipc_path").value
        self._simulation_only = bool(self.get_parameter("simulation_only").value)
        self._pose_publisher = self.create_publisher(
            PoseStamped, "/servo_node/pose_target_cmds", 1
        )
        self._hand_publisher = self.create_publisher(
            JointTrajectory, "/hand_controller/joint_trajectory", 1
        )
        self._hardware_status_subscription = self.create_subscription(
            String, "/stararm102_hardware/status", self._hardware_status_callback, 10
        ) if not self._simulation_only else None
        self._gripper_status_subscription = self.create_subscription(
            String,
            "/stararm102_hardware/gripper_status",
            self._gripper_status_callback,
            10,
        ) if not self._simulation_only else None
        self._joint_subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
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
        self._hardware_fault: str | None = None
        self._latest_gripper_feedback: dict[str, Any] | None = None
        self._latest_joint_positions: list[float] | None = None
        self._latest_joint_time = 0.0
        self._home_status: dict[str, Any] = {
            "request_id": 0,
            "state": "idle",
            "message": "尚未请求回零",
            "trajectory_points": 0,
            "duration_seconds": None,
            "trajectory_model_joints_rad": [],
        }
        self._planned_home_trajectory: Any | None = None
        self._home_plan_goal_handle: Any | None = None
        self._home_execute_goal_handle: Any | None = None
        self._servo_paused_for_home = False
        self._home_zero_feedback_after: float | None = None
        self._forced_home_pairs: tuple[tuple[str, str], ...] = ()
        self._last_home_command: tuple[int, str] | None = None
        self._sequence = 0
        self._last_gripper_closed: bool | None = None
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
        except Exception as error:  # rclpy transports service errors through Future
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

    def _hardware_status_callback(self, message: String) -> None:
        self._hardware_fault = (
            message.data.removeprefix("fault:")
            if message.data.startswith("fault:")
            else None
        )

    def _gripper_status_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if (
            isinstance(value, dict)
            and _finite_vector(
                [
                    value.get("position_rad"),
                    value.get("power_w"),
                    value.get("current_a"),
                    value.get("temperature_c"),
                ],
                4,
            )
            and isinstance(value.get("status"), int)
            and not isinstance(value.get("status"), bool)
            and 0 <= value["status"] <= 0xFF
        ):
            self._latest_gripper_feedback = value

    def _joint_state_callback(self, message: JointState) -> None:
        indices = {name: index for index, name in enumerate(message.name)}
        if len(message.position) != len(message.name) or any(
            name not in indices for name in ROS_JOINT_NAMES
        ):
            return
        positions = [float(message.position[indices[name]]) for name in ROS_JOINT_NAMES]
        if not _finite_vector(positions, 6):
            return
        self._latest_joint_positions = positions
        self._latest_joint_time = time.monotonic()
        self._check_home_zero_feedback()
        try:
            transform = self._tf_buffer.lookup_transform("base_link", "tool0", Time())
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        tcp_position = [translation.x, translation.y, translation.z]
        tcp_orientation = [rotation.x, rotation.y, rotation.z, rotation.w]
        if not _finite_vector(tcp_position, 3) or not _finite_vector(tcp_orientation, 4):
            return
        self._sequence += 1
        feedback = {
            "schema_version": SCHEMA_VERSION,
            "sequence": self._sequence,
            "model_joint_names": list(MODEL_JOINT_NAMES),
            "model_joints_rad": positions,
            "tcp_pose": {
                "position_m": tcp_position,
                "orientation_xyzw": tcp_orientation,
            },
            "servo_status_code": -1 if self._hardware_fault else self._status_code,
            "servo_status_message": self._hardware_fault or self._status_message,
            "gripper_feedback": self._latest_gripper_feedback,
            "home_status": self._home_status,
        }
        self._send_feedback(feedback)

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
                self._reset_home_session()
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
                    self.get_logger().warning(
                        f"NOLO IPC 暂不可用：{error}；1 秒后重连"
                    )
            finally:
                self._close_socket()
            self._stop.wait(1.0)

    def _handle_command(self, line: str) -> None:
        command = json.loads(line)
        if command.get("schema_version") != SCHEMA_VERSION:
            raise ValueError("不支持的 IPC schema_version")
        self._handle_home_request(command.get("home_request"))
        if self._home_status["state"] in {"planning", "executing"}:
            return
        if not command.get("enabled"):
            return
        if not self._pose_commands_selected:
            return
        if command.get("base_frame") != "base_link" or command.get("tcp_link") != "tool0":
            raise ValueError("IPC 坐标系必须是 base_link -> tool0")
        target = command.get("target_pose")
        if not isinstance(target, dict):
            raise ValueError("启用的 IPC 命令缺少 target_pose")
        position = target.get("position_m")
        orientation = target.get("orientation_xyzw")
        if not _finite_vector(position, 3) or not _finite_vector(orientation, 4):
            raise ValueError("target_pose 含非法数值")
        norm = math.sqrt(sum(value * value for value in orientation))
        if not math.isfinite(norm) or norm <= 1.0e-9:
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
        gripper_closed = command.get("gripper_closed")
        if (
            isinstance(gripper_closed, bool)
            and gripper_closed != self._last_gripper_closed
        ):
            self._publish_gripper(0.0 if gripper_closed else math.pi / 2.0)
            self._last_gripper_closed = gripper_closed

    def _handle_home_request(self, raw: Any) -> None:
        if raw is None:
            return
        if not isinstance(raw, dict):
            raise ValueError("home_request 必须是对象")
        request_id = raw.get("request_id")
        action = raw.get("action")
        if not isinstance(request_id, int) or request_id <= 0:
            raise ValueError("home_request.request_id 必须为正整数")
        if action not in {"plan", "cancel"}:
            raise ValueError("home_request.action 非法")
        key = (request_id, action)
        if key == self._last_home_command:
            return
        self._last_home_command = key
        if action == "plan":
            self._begin_home_plan(request_id)
        else:
            self._cancel_home(request_id)

    def _begin_home_plan(self, request_id: int) -> None:
        self._resume_servo_after_home()
        self._planned_home_trajectory = None
        self._home_zero_feedback_after = None
        self._forced_home_pairs = ()
        current = self._current_joint_positions()
        if current is None:
            self._set_home_failed(request_id, "缺少当前关节反馈", "plan")
            return
        if not self._move_group_client.server_is_ready():
            self._set_home_failed(request_id, "MoveGroup 规划服务尚未就绪", "plan")
            return
        self._home_status = {
            "request_id": request_id,
            "acknowledged_action": "plan",
            "state": "planning",
            "message": "MoveIt 正在规划到命名状态 home",
            "trajectory_points": 0,
            "duration_seconds": None,
            "trajectory_model_joints_rad": [],
        }
        self._send_home_plan(request_id, current, None, ())

    def _send_home_plan(
        self,
        request_id: int,
        current: list[float],
        allowed_collision_matrix: Any | None,
        forced_pairs: tuple[tuple[str, str], ...],
    ) -> None:
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = "ompl"
        goal.request.start_state.joint_state.name = list(ROS_JOINT_NAMES)
        goal.request.start_state.joint_state.position = list(current)
        goal.request.start_state.is_diff = False
        constraints = Constraints()
        constraints.name = "home"
        for name in ROS_JOINT_NAMES:
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = 0.0
            joint.tolerance_above = HOME_ZERO_TOLERANCE_RAD
            joint.tolerance_below = HOME_ZERO_TOLERANCE_RAD
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
            lambda completed: self._home_plan_goal_response(
                request_id, forced_pairs, completed
            )
        )

    def _home_plan_goal_response(
        self,
        request_id: int,
        forced_pairs: tuple[tuple[str, str], ...],
        future: Any,
    ) -> None:
        try:
            handle = future.result()
        except Exception as error:
            if self._home_status["request_id"] == request_id:
                self._set_home_failed(request_id, f"提交规划失败：{error}", "plan")
            return
        if (
            self._home_status["request_id"] != request_id
            or self._home_status["state"] == "cancelled"
        ):
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._set_home_failed(request_id, "MoveGroup 拒绝回零规划", "plan")
            return
        self._home_plan_goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(
            lambda completed: self._home_plan_result(
                request_id, forced_pairs, completed
            )
        )

    def _home_plan_result(
        self,
        request_id: int,
        forced_pairs: tuple[tuple[str, str], ...],
        future: Any,
    ) -> None:
        if (
            self._home_status["request_id"] != request_id
            or self._home_status["state"] == "cancelled"
        ):
            return
        self._home_plan_goal_handle = None
        try:
            result = future.result().result
        except Exception as error:
            self._set_home_failed(request_id, f"规划结果读取失败：{error}", "plan")
            return
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            if forced_pairs:
                pairs = self._format_collision_pairs(forced_pairs)
                self._set_home_failed(
                    request_id,
                    f"MoveIt 强制回零规划失败，错误码 {result.error_code.val}；临时放行：{pairs}",
                    "plan",
                )
            else:
                self._begin_forced_home_plan(request_id, result.error_code.val)
            return
        try:
            preview, duration = self._home_trajectory_preview(
                result.planned_trajectory
            )
        except ValueError as error:
            self._set_home_failed(request_id, f"回零轨迹校验失败：{error}", "plan")
            return
        self._planned_home_trajectory = result.planned_trajectory
        self._forced_home_pairs = forced_pairs
        message = "专用回零规划通过"
        if forced_pairs:
            message = (
                "MoveIt 强制回零规划通过；仅本次规划临时放行起始自碰撞对："
                f"{self._format_collision_pairs(forced_pairs)}"
            )
        self._home_status = {
            "request_id": request_id,
            "acknowledged_action": "plan",
            "state": "planning",
            "message": message,
            "trajectory_points": len(
                result.planned_trajectory.joint_trajectory.points
            ),
            "duration_seconds": duration,
            "trajectory_model_joints_rad": preview,
        }
        self._begin_home_execute(request_id)

    def _begin_forced_home_plan(self, request_id: int, error_code: int) -> None:
        current = self._current_joint_positions()
        if current is None:
            self._set_home_failed(
                request_id,
                f"MoveIt 规划失败，错误码 {error_code}；强制规划前缺少新鲜关节反馈",
                "plan",
            )
            return
        if not self._state_validity_client.service_is_ready():
            self._set_home_failed(
                request_id,
                f"MoveIt 规划失败，错误码 {error_code}；状态有效性服务尚未就绪",
                "plan",
            )
            return
        self._home_status = {
            **self._home_status,
            "state": "planning",
            "message": "MoveIt 正在识别起始自碰撞，准备同一路径强制规划",
        }
        request = GetStateValidity.Request()
        request.robot_state.joint_state.name = list(ROS_JOINT_NAMES)
        request.robot_state.joint_state.position = list(current)
        request.robot_state.is_diff = False
        request.group_name = "arm"
        future = self._state_validity_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._forced_home_validity_result(
                request_id, error_code, completed
            )
        )

    def _forced_home_validity_result(
        self, request_id: int, error_code: int, future: Any
    ) -> None:
        if (
            self._home_status["request_id"] != request_id
            or self._home_status["state"] == "cancelled"
        ):
            return
        try:
            response = future.result()
        except Exception as error:
            self._set_home_failed(
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
            self._set_home_failed(
                request_id,
                f"MoveIt 规划失败，错误码 {error_code}；未检测到可临时放行的起始自碰撞",
                "plan",
            )
            return
        if not self._planning_scene_client.service_is_ready():
            self._set_home_failed(
                request_id, "MoveIt 规划场景服务尚未就绪", "plan"
            )
            return
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        scene_future = self._planning_scene_client.call_async(request)
        scene_future.add_done_callback(
            lambda completed: self._forced_home_scene_result(
                request_id, pairs, completed
            )
        )

    def _forced_home_scene_result(
        self,
        request_id: int,
        pairs: tuple[tuple[str, str], ...],
        future: Any,
    ) -> None:
        if (
            self._home_status["request_id"] != request_id
            or self._home_status["state"] == "cancelled"
        ):
            return
        try:
            response = future.result()
            matrix = response.scene.allowed_collision_matrix
            self._allow_collision_pairs(matrix, pairs)
        except Exception as error:
            self._set_home_failed(
                request_id, f"构造 MoveIt 临时碰撞矩阵失败：{error}", "plan"
            )
            return
        current = self._current_joint_positions()
        if current is None:
            self._set_home_failed(
                request_id, "强制规划前缺少当前关节反馈", "plan"
            )
            return
        self._home_status = {
            **self._home_status,
            "message": (
                "MoveIt 正在强制规划回零；仅本次临时放行："
                f"{self._format_collision_pairs(pairs)}"
            ),
        }
        self._send_home_plan(request_id, current, matrix, pairs)

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

    def _home_trajectory_preview(
        self, trajectory: Any
    ) -> tuple[list[list[float]], float]:
        """Extract a finite J1--J6 preview; MoveIt/JTC validate execution."""
        joint_trajectory = trajectory.joint_trajectory
        indices = {
            name: index for index, name in enumerate(joint_trajectory.joint_names)
        }
        if any(name not in indices for name in ROS_JOINT_NAMES):
            raise ValueError("轨迹缺少 J1–J6")
        if not joint_trajectory.points:
            raise ValueError("轨迹为空")
        ordered: list[list[float]] = []
        for point in joint_trajectory.points:
            if len(point.positions) != len(joint_trajectory.joint_names):
                raise ValueError("关节名和位置长度不一致")
            values = [
                float(point.positions[indices[name]]) for name in ROS_JOINT_NAMES
            ]
            if not _finite_vector(values, 6):
                raise ValueError("轨迹包含非法关节值")
            ordered.append(values)
        last = joint_trajectory.points[-1].time_from_start
        duration = last.sec + last.nanosec * 1.0e-9
        if not math.isfinite(duration) or duration < 0.0:
            raise ValueError("轨迹持续时间非法")
        return ordered, duration

    def _begin_home_execute(self, request_id: int) -> None:
        if self._planned_home_trajectory is None or self._home_status["request_id"] != request_id:
            self._set_home_failed(request_id, "没有可执行的已验证回零轨迹", "plan")
            return
        if not self._execute_trajectory_client.server_is_ready():
            self._set_home_failed(
                request_id, "MoveIt 轨迹执行服务尚未就绪", "plan"
            )
            return
        if not self._servo_pause_client.service_is_ready():
            self._set_home_failed(
                request_id, "MoveIt Servo 暂停服务尚未就绪", "plan"
            )
            return
        self._home_status = {
            **self._home_status,
            "acknowledged_action": "plan",
            "state": "executing",
            "message": "正在暂停 MoveIt Servo 连续输出，准备执行回零轨迹",
        }
        request = SetBool.Request()
        request.data = True
        future = self._servo_pause_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._home_servo_pause_response(
                request_id, completed
            )
        )

    def _home_servo_pause_response(
        self, request_id: int, future: Any
    ) -> None:
        try:
            response = future.result()
        except Exception as error:
            if self._home_status["request_id"] == request_id:
                self._set_home_failed(
                    request_id,
                    f"暂停 MoveIt Servo 失败：{error}",
                    "plan",
                )
            return
        if not response.success:
            if self._home_status["request_id"] == request_id:
                self._set_home_failed(
                    request_id,
                    f"MoveIt Servo 拒绝暂停：{response.message}",
                    "plan",
                )
            return
        self._servo_paused_for_home = True
        if (
            self._home_status["request_id"] != request_id
            or self._home_status["state"] != "executing"
        ):
            self._resume_servo_after_home()
            return
        self._send_home_execute_goal(request_id)

    def _send_home_execute_goal(self, request_id: int) -> None:
        if self._planned_home_trajectory is None:
            self._set_home_failed(
                request_id,
                "暂停 MoveIt Servo 后回零轨迹已失效",
                self._home_status.get("acknowledged_action"),
            )
            return
        self._home_status = {
            **self._home_status,
            "message": (
                "正在执行 MoveIt 强制回零轨迹；可随时点击停止"
                if self._forced_home_pairs
                else "正在执行回零轨迹；可随时点击停止"
            ),
        }
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self._planned_home_trajectory
        goal.controller_names = ["arm_controller"]
        future = self._execute_trajectory_client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self._home_execute_goal_response(request_id, completed)
        )

    def _home_execute_goal_response(self, request_id: int, future: Any) -> None:
        try:
            handle = future.result()
        except Exception as error:
            if self._home_status["request_id"] == request_id:
                self._set_home_failed(
                    request_id,
                    f"提交执行失败：{error}",
                    self._home_status.get("acknowledged_action"),
                )
            return
        if (
            self._home_status["request_id"] != request_id
            or self._home_status["state"] == "cancelled"
        ):
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._set_home_failed(
                request_id,
                "MoveIt 拒绝执行回零轨迹",
                self._home_status.get("acknowledged_action"),
            )
            return
        self._home_execute_goal_handle = handle
        result = handle.get_result_async()
        result.add_done_callback(
            lambda completed: self._home_execute_result(request_id, completed)
        )

    def _home_execute_result(self, request_id: int, future: Any) -> None:
        if (
            self._home_status["request_id"] != request_id
            or self._home_status["state"] == "cancelled"
        ):
            return
        self._home_execute_goal_handle = None
        try:
            result = future.result().result
        except Exception as error:
            self._set_home_failed(
                request_id,
                f"执行结果读取失败：{error}",
                self._home_status.get("acknowledged_action"),
            )
            return
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self._home_status = {
                **self._home_status,
                "message": "轨迹执行完成，正在确认 J1–J6 反馈误差不超过 1°",
            }
            self._home_zero_feedback_after = self._latest_joint_time
        else:
            self._set_home_failed(
                request_id,
                f"回零执行失败，MoveIt 错误码 {result.error_code.val}",
                self._home_status.get("acknowledged_action"),
            )

    def _check_home_zero_feedback(self) -> None:
        if (
            self._home_zero_feedback_after is None
            or self._latest_joint_time <= self._home_zero_feedback_after
            or self._latest_joint_positions is None
            or self._home_status["state"] != "executing"
        ):
            return
        request_id = self._home_status["request_id"]
        positions = self._latest_joint_positions
        self._home_zero_feedback_after = None
        max_index = max(range(len(positions)), key=lambda index: abs(positions[index]))
        max_error = abs(positions[max_index])
        if max_error <= HOME_ZERO_TOLERANCE_RAD:
            self._resume_servo_after_home()
            self._home_status = {
                **self._home_status,
                "state": "succeeded",
                "message": (
                    "已完成 MoveIt 强制回零，J1–J6 反馈均在 ±1° 内"
                    if self._forced_home_pairs
                    else "已完成专用回零，J1–J6 反馈均在 ±1° 内"
                ),
            }
            return
        self._set_home_failed(
            request_id,
            (
                "MoveIt 执行返回成功，但回零反馈超出 ±1°："
                f"J{max_index + 1}={math.degrees(positions[max_index]):.2f}°"
            ),
            self._home_status.get("acknowledged_action"),
        )

    def _cancel_home(self, request_id: int) -> None:
        if self._home_plan_goal_handle is not None:
            self._home_plan_goal_handle.cancel_goal_async()
            self._home_plan_goal_handle = None
        if self._home_execute_goal_handle is not None:
            self._home_execute_goal_handle.cancel_goal_async()
            self._home_execute_goal_handle = None
        self._resume_servo_after_home()
        self._planned_home_trajectory = None
        self._home_zero_feedback_after = None
        self._forced_home_pairs = ()
        self._home_status = {
            "request_id": request_id,
            "acknowledged_action": "cancel",
            "state": "cancelled",
            "message": "回零操作已取消；机械臂保持当前位置",
            "trajectory_points": 0,
            "duration_seconds": None,
            "trajectory_model_joints_rad": [],
        }

    def _set_home_failed(self, request_id: int, message: str, action: Any) -> None:
        self._resume_servo_after_home()
        self._home_zero_feedback_after = None
        self._home_status = {
            **self._home_status,
            "request_id": request_id,
            "acknowledged_action": action,
            "state": "failed",
            "message": message,
        }

    def _resume_servo_after_home(self) -> None:
        if not self._servo_paused_for_home:
            return
        self._servo_paused_for_home = False
        if not self._servo_pause_client.service_is_ready():
            self.get_logger().error("回零结束后 MoveIt Servo 恢复服务未就绪")
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
            self.get_logger().error(
                f"MoveIt Servo 拒绝恢复：{response.message}"
            )

    def _publish_gripper(self, position: float) -> None:
        trajectory = JointTrajectory()
        trajectory.joint_names = ["joint7_left"]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.nanosec = 250_000_000
        trajectory.points = [point]
        self._hand_publisher.publish(trajectory)

    def _current_joint_positions(self) -> list[float] | None:
        return self._latest_joint_positions

    def _close_socket(self) -> None:
        with self._socket_lock:
            self._close_socket_locked()

    def _close_socket_locked(self) -> None:
        if self._home_plan_goal_handle is not None:
            self._home_plan_goal_handle.cancel_goal_async()
            self._home_plan_goal_handle = None
        if self._home_execute_goal_handle is not None:
            self._home_execute_goal_handle.cancel_goal_async()
            self._home_execute_goal_handle = None
        self._resume_servo_after_home()
        self._home_zero_feedback_after = None
        if self._home_status["state"] in {"planning", "executing"}:
            self._home_status = {
                **self._home_status,
                "state": "cancelled",
                "message": "IPC 断开，已取消回零执行",
            }
        self._planned_home_trajectory = None
        connection = self._socket
        self._socket = None
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()

    def _reset_home_session(self) -> None:
        """Invalidate plans from an earlier IPC connection."""
        self._resume_servo_after_home()
        self._planned_home_trajectory = None
        self._home_plan_goal_handle = None
        self._home_execute_goal_handle = None
        self._home_zero_feedback_after = None
        self._last_home_command = None
        self._last_gripper_closed = None
        self._home_status = {
            "request_id": 0,
            "state": "idle",
            "message": "尚未请求回零",
            "trajectory_points": 0,
            "duration_seconds": None,
            "trajectory_model_joints_rad": [],
        }


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
