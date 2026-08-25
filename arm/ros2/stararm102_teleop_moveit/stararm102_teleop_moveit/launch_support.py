"""Shared MoveIt launch construction for simulation and hardware."""

from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def load_moveit_parameters() -> tuple[object, dict, dict]:
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
    move_group_params = moveit_config.to_dict()
    # No depth sensor is installed, so do not configure an occupancy updater.
    # Self-collision and the static planning scene remain enabled.
    for key in ("sensors", "kinect_pointcloud", "kinect_depthimage"):
        move_group_params.pop(key, None)
    return moveit_config, servo_params, move_group_params


def robot_state_publisher(moveit_config: object) -> Node:
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )


def control_nodes(
    moveit_config: object, servo_params: dict, controllers: str
) -> list[Node]:
    nodes = [
        robot_state_publisher(moveit_config),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description, controllers],
            output="screen",
        ),
    ]
    nodes.extend(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller, "-c", "/controller_manager"],
            output="screen",
        )
        for controller in (
            "joint_state_broadcaster",
            "arm_controller",
            "hand_controller",
        )
    )
    nodes.append(servo_node(moveit_config, servo_params))
    return nodes


def bridge_node(ipc_path: object, simulation_only: bool = True) -> Node:
    return Node(
        package="stararm102_teleop_moveit",
        executable="servo_ipc_bridge",
        parameters=[{"ipc_path": ipc_path, "simulation_only": simulation_only}],
        output="screen",
    )


def servo_node(moveit_config: object, servo_params: dict) -> Node:
    return Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )


def move_group_node(move_group_params: dict) -> Node:
    return Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            move_group_params,
            {
                "allow_trajectory_execution": True,
                "publish_robot_description_semantic": True,
                "moveit_manage_controllers": False,
            },
        ],
    )
