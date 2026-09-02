"""Shared MoveIt configuration for the model-owned ROS stack."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_configs(octomap_resolution_m=None):
    sensors = (
        Path(get_package_share_directory("stararm_102_motion_node"))
        / "config"
        / "sensors_3d.yaml"
    )
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
        .sensors_3d(file_path=str(sensors))
        .to_moveit_configs()
    )
    if octomap_resolution_m is None:
        octomap_resolution_m = float(os.environ["OCTOMAP_RESOLUTION_M"])
    config.sensors_3d["octomap_resolution"] = octomap_resolution_m
    return config
