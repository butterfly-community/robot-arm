"""Shared MoveIt configuration for the model-owned ROS stack."""

from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_configs():
    config = (
        MoveItConfigsBuilder(
            "stararm102_description", package_name="stararm102_moveit_config"
        )
        .robot_description(file_path="config/stararm102_description.urdf.xacro")
        .robot_description_semantic(file_path="config/stararm102_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    # Do not inherit vendor Kinect drivers. The launch file configures the
    # official updater for explicit task observations from our scene node.
    config.sensors_3d = {}
    return config
