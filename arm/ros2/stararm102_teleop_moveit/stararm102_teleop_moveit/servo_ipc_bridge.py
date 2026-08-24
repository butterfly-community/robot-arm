"""Bridge newline-delimited JSON IPC to the standard MoveIt Servo ROS API."""

import json
import math
import socket
import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


SCHEMA_VERSION = 2
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
        self._ipc_path = self.get_parameter("ipc_path").value
        self._pose_publisher = self.create_publisher(
            PoseStamped, "/servo_node/pose_target_cmds", 1
        )
        self._joint_subscription = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        self._status_subscription = self.create_subscription(
            ServoStatus, "/servo_node/status", self._status_callback, 10
        )
        self._command_client = self.create_client(
            ServoCommandType, "/servo_node/switch_command_type"
        )
        self._command_timer = self.create_timer(0.2, self._select_pose_commands)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._command_request_pending = False
        self._pose_commands_selected = False
        self._status_code: int | None = None
        self._status_message: str | None = None
        self._sequence = 0
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

    def _joint_state_callback(self, message: JointState) -> None:
        indices = {name: index for index, name in enumerate(message.name)}
        if len(message.position) != len(message.name) or any(
            name not in indices for name in ROS_JOINT_NAMES
        ):
            return
        positions = [float(message.position[indices[name]]) for name in ROS_JOINT_NAMES]
        if len(message.velocity) == len(message.name):
            velocities = [float(message.velocity[indices[name]]) for name in ROS_JOINT_NAMES]
        else:
            velocities = [0.0] * len(ROS_JOINT_NAMES)
        if not _finite_vector(positions, 6) or not _finite_vector(velocities, 6):
            return
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
            "model_joint_velocity_rad_s": velocities,
            "tcp_pose": {
                "position_m": tcp_position,
                "orientation_xyzw": tcp_orientation,
            },
            "servo_status_code": self._status_code,
            "servo_status_message": self._status_message,
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

    def _close_socket(self) -> None:
        with self._socket_lock:
            self._close_socket_locked()

    def _close_socket_locked(self) -> None:
        connection = self._socket
        self._socket = None
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()


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
