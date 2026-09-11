from pathlib import Path
import json

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
    move_group.update(ParameterBuilder("stararm_102_motion_node").yaml("config/sensors_3d.yaml").to_dict())
    self_filter = json.loads(Path("/config/robot-self-filter.json").read_text())
    move_group["depth"]["mesh_padding_offset"] = self_filter["mesh_padding_offset_m"]
    # Filter duplicate samples around known geometry before voxelization. This
    # does not pad collision objects or lower the physical ground. Zero padding
    # left quantized ground samples in the 0..5 mm voxel layer, blocking closure.
    move_group["depth"]["padding_offset"] = move_group["octomap_resolution"] / 2
    controllers = str(
        Path(get_package_share_directory("stararm_102_motion_node"))
        / "config"
        / "controllers.yaml"
    )
    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[moveit.robot_description, {"publish_frequency": 100.0}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                str(
                    Path(get_package_share_directory("stararm_102_motion_node"))
                    / "config"
                    / "stararm102.rviz"
                ),
            ],
            parameters=[
                moveit.robot_description,
                moveit.robot_description_semantic,
                moveit.robot_description_kinematics,
                moveit.planning_pipelines,
                moveit.joint_limits,
            ],
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
                    "capabilities": "move_group/ExecuteTaskSolutionCapability",
                    "trajectory_execution.allowed_start_tolerance": 0.0,
                    "publish_robot_description_semantic": True,
                    "moveit_manage_controllers": False,
                },
            ],
            output="screen",
        ),
        Node(
            package="stararm_102_mtc",
            executable="pick_place_server",
            parameters=[
                {"octomap_resolution": move_group["octomap_resolution"],
                 "mesh_padding_offset": self_filter["mesh_padding_offset_m"]},
                moveit.robot_description,
                moveit.robot_description_semantic,
                moveit.robot_description_kinematics,
                moveit.planning_pipelines,
                moveit.joint_limits,
            ],
            output="screen",
        ),
    ]
    nodes.extend(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "-c", "/controller_manager", "--param-file", controllers],
            output="screen",
        )
        for name in ("joint_state_broadcaster", "arm_controller", "hand_controller")
    )
    return LaunchDescription(nodes)
