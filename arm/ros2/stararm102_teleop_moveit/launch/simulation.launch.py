import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from stararm102_teleop_moveit.launch_support import (
    bridge_node,
    control_nodes,
    load_moveit_parameters,
    move_group_node,
)


def generate_launch_description():
    vendor_share = get_package_share_directory("stararm102_moveit_config")
    ipc_path = LaunchConfiguration("ipc_path")

    moveit_config, servo_params, move_group_params = load_moveit_parameters()
    controllers = os.path.join(vendor_share, "config", "ros2_controllers.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("ipc_path", default_value="/ipc/moveit-servo.sock"),
            *control_nodes(moveit_config, servo_params, controllers),
            move_group_node(move_group_params),
            bridge_node(ipc_path),
        ]
    )
