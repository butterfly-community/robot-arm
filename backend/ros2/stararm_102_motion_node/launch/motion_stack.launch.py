from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from stararm_102_motion_node.moveit_config import build_moveit_configs


def generate_launch_description() -> LaunchDescription:
    moveit = build_moveit_configs()
    servo = (
        ParameterBuilder("stararm_102_motion_node").yaml("config/servo.yaml").to_dict()
    )
    move_group = moveit.to_dict()
    for key in ("sensors", "kinect_pointcloud", "kinect_depthimage"):
        move_group.pop(key, None)
    controllers = str(
        Path(get_package_share_directory("stararm_102_motion_node"))
        / "config"
        / "controllers.yaml"
    )
    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[moveit.robot_description],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit.robot_description, controllers],
            output="screen",
        ),
        Node(
            package="moveit_servo",
            executable="servo_node",
            name="servo_node",
            parameters=[
                servo,
                moveit.robot_description,
                moveit.robot_description_semantic,
                moveit.robot_description_kinematics,
                moveit.joint_limits,
            ],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[
                move_group,
                {
                    "allow_trajectory_execution": True,
                    "trajectory_execution.allowed_start_tolerance": 0.0,
                    "publish_robot_description_semantic": True,
                    "moveit_manage_controllers": False,
                },
            ],
            output="screen",
        ),
        Node(
            package="stararm_102_motion_node",
            executable="dora_motion_node",
            output="screen",
        ),
    ]
    nodes.extend(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "-c", "/controller_manager"],
            output="screen",
        )
        for name in ("joint_state_broadcaster", "arm_controller", "hand_controller")
    )
    return LaunchDescription(nodes)
