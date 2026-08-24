import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from stararm102_teleop_moveit.launch_support import (
    load_moveit_parameters,
    move_group_node,
    robot_state_publisher,
    servo_node,
)


def generate_launch_description():
    package_share = get_package_share_directory("stararm102_teleop_moveit")
    vendor_share = get_package_share_directory("stararm102_moveit_config")
    ipc_path = LaunchConfiguration("ipc_path")

    moveit_config, servo_params, move_group_params = load_moveit_parameters()
    controllers = os.path.join(vendor_share, "config", "ros2_controllers.yaml")

    state_publisher = robot_state_publisher(moveit_config)
    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, controllers],
        output="screen",
    )
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )
    hand_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "-c", "/controller_manager"],
        output="screen",
    )
    servo = servo_node(moveit_config, servo_params)
    move_group = move_group_node(move_group_params)
    bridge = Node(
        package="stararm102_teleop_moveit",
        executable="servo_ipc_bridge",
        parameters=[{"ipc_path": ipc_path}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ipc_path", default_value="/ipc/moveit-servo.sock"),
            state_publisher,
            ros2_control,
            joint_state_spawner,
            arm_spawner,
            hand_spawner,
            move_group,
            servo,
            bridge,
        ]
    )
