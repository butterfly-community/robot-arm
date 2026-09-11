"""Shared MoveIt configuration for the model-owned ROS stack."""

from moveit_configs_utils import MoveItConfigsBuilder
from xml.etree import ElementTree as ET


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
    # Completion follows the controller's actual feedback, not nominal timing.
    # Real replay: hand JTC succeeded at .567 s, while TEM cancelled at .595 s
    # before the action result arrived (a 0.538 s budget for a tiny open move).
    # Keep waiting for the action result; do not invent a larger time margin.
    config.trajectory_execution["trajectory_execution.execution_duration_monitoring"] = False
    # ROS transport capabilities belong to this service, not the frozen vendor
    # geometry. JointStateTopicSystem receives measured position derivatives for
    # every named joint; expose them without changing any command interface.
    description = ET.fromstring(config.robot_description["robot_description"])
    for joint in description.findall("./ros2_control/joint"):
        if joint.find("state_interface[@name='velocity']") is None:
            ET.SubElement(joint, "state_interface", name="velocity")
    config.robot_description["robot_description"] = ET.tostring(description, encoding="unicode")
    return config
