import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = get_package_share_directory("stararm102_teleop_moveit")
    vendor_share = get_package_share_directory("stararm102_moveit_config")
    ipc_path = LaunchConfiguration("ipc_path")

    moveit_config = (
        MoveItConfigsBuilder(
            "stararm102_description", package_name="stararm102_moveit_config"
        )
        .robot_description(file_path="config/stararm102_description.urdf.xacro")
        .robot_description_semantic(file_path="config/stararm102_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )
    servo_params = (
        ParameterBuilder("stararm102_teleop_moveit")
        .yaml("config/servo.yaml")
        .to_dict()
    )
    controllers = os.path.join(vendor_share, "config", "ros2_controllers.yaml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )
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
    servo = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            {"update_period": 0.01},
            {"planning_group_name": "arm"},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )
    bridge = Node(
        package="stararm102_teleop_moveit",
        executable="servo_ipc_bridge",
        parameters=[{"ipc_path": ipc_path}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("ipc_path", default_value="/ipc/moveit-servo.sock"),
            robot_state_publisher,
            ros2_control,
            joint_state_spawner,
            arm_spawner,
            servo,
            bridge,
        ]
    )
