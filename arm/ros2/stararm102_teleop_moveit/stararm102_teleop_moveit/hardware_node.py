"""Thin topic adapter between ros2_control and the Star Arm 102-FL SDK."""

from __future__ import annotations

import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .hardware_bus import (
    HardwareBus,
    HardwareFeedback,
    make_bus,
)


ROS_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_JOINT_NAME = "joint7_left"
STATE_TOPIC = "/stararm102_hardware/joint_states"
COMMAND_TOPIC = "/stararm102_hardware/joint_commands"


class StarArm102HardwareNode(Node):
    """Expose only position samples and targets; ros2_control owns trajectories."""

    def __init__(self) -> None:
        super().__init__("stararm102_fl_hardware")
        self._last_feedback_error: str | None = None
        self._last_command_error: str | None = None
        self._last_gripper_target: float | None = None

        self.declare_parameter("port", "")
        self.declare_parameter("baudrate", 1_000_000)
        port = str(self.get_parameter("port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        if not port:
            raise RuntimeError("必须显式设置 Star Arm 102-FL 串口")
        if baudrate <= 0:
            raise RuntimeError("baudrate 必须为正数")

        self._bus = HardwareBus(make_bus(port, baudrate))
        self._state_publisher = self.create_publisher(JointState, STATE_TOPIC, 10)
        self._status_publisher = self.create_publisher(
            String, "/stararm102_hardware/status", 10
        )
        self._gripper_status_publisher = self.create_publisher(
            String, "/stararm102_hardware/gripper_status", 10
        )
        self._command_subscription = self.create_subscription(
            JointState,
            COMMAND_TOPIC,
            self._command_callback,
            10,
        )
        try:
            initial = self._bus.connect()
            self._publish_feedback(initial)
        except Exception:
            try:
                self._bus.disconnect()
            except Exception:
                pass
            raise
        self._feedback_timer = self.create_timer(0.01, self._read_feedback)
        self._publish_status("ready")
        self.get_logger().info(
            "Star Arm 102-FL 已连接；轨迹由 ros2_control JointTrajectoryController 执行"
        )

    def destroy_node(self) -> bool:
        try:
            self._bus.disconnect()
        except Exception as error:
            self.get_logger().error(f"关闭真机串口失败：{error}")
        return super().destroy_node()

    def _read_feedback(self) -> None:
        try:
            feedback = self._bus.read_feedback()
            self._publish_feedback(feedback)
            if self._last_feedback_error is not None:
                self._last_feedback_error = None
                if self._last_command_error is None:
                    self._publish_status("ready")
        except Exception as error:
            reason = f"读取真机关节反馈失败：{error}"
            self._publish_status(f"fault:{reason}")
            if reason != self._last_feedback_error:
                self.get_logger().error(reason)
                self._last_feedback_error = reason

    def _publish_feedback(self, feedback: HardwareFeedback) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [*ROS_JOINT_NAMES, GRIPPER_JOINT_NAME]
        message.position = [
            *feedback.arm_positions_rad,
            feedback.gripper.position_rad,
        ]
        # Monitor has no trustworthy velocity sample.
        message.velocity = []
        self._state_publisher.publish(message)
        status = String()
        status.data = json.dumps(
            {
                "position_rad": feedback.gripper.position_rad,
                "power_w": feedback.gripper.power_w,
                "current_a": feedback.gripper.current_a,
                "temperature_c": feedback.gripper.temperature_c,
                "status": feedback.gripper.status,
            },
            separators=(",", ":"),
        )
        self._gripper_status_publisher.publish(status)

    def _command_callback(self, message: JointState) -> None:
        indices = {name: index for index, name in enumerate(message.name)}
        if len(message.position) != len(message.name) or any(
            name not in indices for name in ROS_JOINT_NAMES
        ):
            self._report_command_error("ros2_control 输出缺少 J1--J6")
            return
        target = [float(message.position[indices[name]]) for name in ROS_JOINT_NAMES]
        try:
            gripper_target = None
            gripper_index = indices.get(GRIPPER_JOINT_NAME)
            if gripper_index is not None:
                gripper_position = float(message.position[gripper_index])
                if gripper_position != self._last_gripper_target:
                    gripper_target = gripper_position
            self._bus.write_positions(target, gripper_target)
            if gripper_target is not None:
                self._last_gripper_target = gripper_target
            if self._last_command_error is not None:
                self._last_command_error = None
                if self._last_feedback_error is None:
                    self._publish_status("ready")
        except Exception as error:
            self._report_command_error(f"写入真机关节目标失败：{error}")

    def _report_command_error(self, reason: str) -> None:
        self._publish_status(f"fault:{reason}")
        if reason != self._last_command_error:
            self.get_logger().error(reason)
            self._last_command_error = reason

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self._status_publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: StarArm102HardwareNode | None = None
    try:
        node = StarArm102HardwareNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
