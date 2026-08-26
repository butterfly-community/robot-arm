from __future__ import annotations

import pathlib
import re
import unittest


ARM_ROOT = pathlib.Path(__file__).parents[1]
ROS_ROOT = ARM_ROOT / "ros2"
PACKAGE_ROOT = ROS_ROOT / "stararm102_teleop_moveit"


class UnifiedControlPathTests(unittest.TestCase):
    def test_only_one_launch_controller_config_and_service_entry_exist(self):
        self.assertEqual(
            [path.name for path in (PACKAGE_ROOT / "launch").glob("*.launch.py")],
            ["arm.launch.py"],
        )
        controller_configs = sorted(
            path.name for path in (PACKAGE_ROOT / "config").glob("*controllers.yaml")
        )
        self.assertEqual(controller_configs, ["controllers.yaml"])
        self.assertTrue((ROS_ROOT / "start-moveit.sh").is_file())
        self.assertFalse((ROS_ROOT / "run-moveit.sh").exists())
        self.assertFalse((ROS_ROOT / "run-moveit-simulation.sh").exists())
        self.assertFalse((ROS_ROOT / "run-moveit-hardware.sh").exists())

    def test_ros_is_independent_of_serial_connection(self):
        bridge = (
            PACKAGE_ROOT / "stararm102_teleop_moveit" / "servo_ipc_bridge.py"
        ).read_text()
        launch = (PACKAGE_ROOT / "launch" / "arm.launch.py").read_text()
        support = (
            PACKAGE_ROOT / "stararm102_teleop_moveit" / "launch_support.py"
        ).read_text()
        controllers = (PACKAGE_ROOT / "config" / "controllers.yaml").read_text()
        topic_patch = (
            ARM_ROOT / "patches" / "star-arm-102-fl-topic-io.patch"
        ).read_text()

        self.assertIn("SCHEMA_VERSION = 5", bridge)
        self.assertNotIn("controller_joint_names", bridge)
        self.assertNotIn("MODEL_JOINT_NAMES", bridge)
        self.assertNotIn("model_joints_rad", bridge)
        self.assertIn('STATE_TOPIC = "/stararm102/joint_states"', bridge)
        self.assertIn('COMMAND_TOPIC = "/stararm102/joint_commands"', bridge)
        self.assertIn("_handle_motion_request", bridge)
        self.assertIn("_handle_gripper_request", bridge)
        self.assertIn('self, ExecuteTrajectory, "/execute_trajectory"', bridge)
        self.assertIn('SetBool, "/servo_node/pause_servo"', bridge)
        self.assertIn('GetStateValidity, "/check_state_validity"', bridge)
        self.assertIn('GetPlanningScene, "/get_planning_scene"', bridge)
        self.assertIn(
            "math.radians(value) for value in (0.0, 0.0, -3.0, 0.0, 0.0, 0.0)",
            bridge,
        )
        self.assertNotIn("START_POSITION_TOLERANCE_RAD", bridge)
        self.assertNotIn("_check_start_position_feedback", bridge)
        self.assertNotIn("serial", bridge.lower())
        self.assertNotIn("home", bridge.lower())
        self.assertNotIn("backend", bridge.lower())
        self.assertIn('executable="ros2_control_node"', support)
        self.assertIn('"arm_controller"', support)
        self.assertIn('"hand_controller"', support)
        self.assertIn("arm.launch.py", (ROS_ROOT / "start-moveit.sh").read_text())
        self.assertIn("bridge_node(ipc_path)", launch)
        self.assertIn("joints: [joint7_left]", controllers)
        self.assertIn(
            "joint_state_topic_hardware_interface/JointStateTopicSystem", topic_patch
        )
        self.assertEqual(
            re.findall(r'<param name="initial_value">([^<]+)</param>', topic_patch),
            [
                "0",
                "0",
                "-0.05235987755982989",
                "0",
                "0",
                "0",
                "0.017453292519943295",
            ],
        )

    def test_motion_gripper_and_collision_retry_share_the_bridge(self):
        bridge = (
            PACKAGE_ROOT / "stararm102_teleop_moveit" / "servo_ipc_bridge.py"
        ).read_text()
        self.assertIn('goal.request.group_name = "arm"', bridge)
        self.assertIn('goal.controller_names = ["arm_controller"]', bridge)
        self.assertIn('trajectory.joint_names = [GRIPPER_JOINT_NAME]', bridge)
        self.assertIn("point.time_from_start.nanosec = 250_000_000", bridge)
        self.assertIn("scene.allowed_collision_matrix = allowed_collision_matrix", bridge)
        self.assertNotIn("HomeRequest", bridge)
        self.assertNotIn("arm-home", bridge)

    def test_arm_uses_vendor_link6_without_an_unmeasured_tool_frame(self):
        bridge = (
            PACKAGE_ROOT / "stararm102_teleop_moveit" / "servo_ipc_bridge.py"
        ).read_text()
        profile = (ARM_ROOT / "config" / "stararm102-fl.v1.json").read_text()
        model_patch = (
            ARM_ROOT / "patches" / "star-arm-102-fl-moveit-model.patch"
        ).read_text()

        self.assertIn('"tcp_link": "link6"', profile)
        self.assertIn('lookup_transform("base_link", "link6", Time())', bridge)
        self.assertIn('tip_link="link6"', model_patch)
        self.assertNotIn("tool0", profile)
        self.assertNotIn("tool0", bridge)
        self.assertNotIn("tool0", model_patch)

    def test_trac_ik_replaces_kdl_without_duplicate_solver_tuning(self):
        model_patch = (
            ARM_ROOT / "patches" / "star-arm-102-fl-moveit-model.patch"
        ).read_text()
        dockerfile = (ROS_ROOT / "Dockerfile").read_text()
        package = (PACKAGE_ROOT / "package.xml").read_text()
        added_lines = "\n".join(
            line[1:]
            for line in model_patch.splitlines()
            if line.startswith("+") and not line.startswith("+++")
        )

        self.assertIn(
            "kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
            added_lines,
        )
        self.assertNotIn("KDLKinematicsPlugin", added_lines)
        self.assertNotIn("kinematics_solver_search_resolution", added_lines)
        self.assertNotIn("solve_type:", added_lines)
        self.assertNotIn("epsilon:", added_lines)
        self.assertIn("ros-jazzy-trac-ik-kinematics-plugin", dockerfile)
        self.assertIn(
            "<exec_depend>trac_ik_kinematics_plugin</exec_depend>", package
        )

    def test_urdf_position_limits_are_the_single_source(self):
        model_patch = (
            ARM_ROOT / "patches" / "star-arm-102-fl-moveit-model.patch"
        ).read_text()
        topic_patch = (
            ARM_ROOT / "patches" / "star-arm-102-fl-topic-io.patch"
        ).read_text()
        profile = (ARM_ROOT / "config" / "stararm102-fl.v1.json").read_text()
        self.assertNotIn("stararm102_description.urdf b/", model_patch)
        self.assertNotIn("lower_deg", profile)
        self.assertNotIn("upper_deg", profile)
        added_topic_lines = [
            line[1:]
            for line in topic_patch.splitlines()
            if line.startswith("+") and not line.startswith("+++")
        ]
        self.assertEqual(
            [line.strip() for line in added_topic_lines if '<param name="min">' in line],
            ['<param name="min">-2.27</param>'] * 2,
        )
        self.assertEqual(
            [line.strip() for line in added_topic_lines if '<param name="max">' in line],
            ['<param name="max">2.27</param>'] * 2,
        )

    def test_web_model_is_generated_from_the_patched_vendor_package(self):
        root = ARM_ROOT.parent
        dockerfile = (
            root / "vr-xr" / "src" / "nolo-usb-server" / "Dockerfile"
        ).read_text()
        compose = (root / "compose.yaml").read_text()
        models = (
            root
            / "vr-xr"
            / "src"
            / "controller-viewer"
            / "public"
            / "arm-simulator"
            / "models"
        )

        self.assertFalse(models.exists())
        self.assertIn("FROM denoland/deno:", dockerfile)
        self.assertIn("star-arm-102-fl-moveit-model.patch", dockerfile)
        self.assertIn("star-arm-102-fl-moveit-dynamics.patch", dockerfile)
        self.assertIn("star-arm-102-fl-topic-io.patch", dockerfile)
        self.assertIn("prepare-controller-viewer.ts", dockerfile)
        self.assertIn("COPY --from=viewer /tmp/viewer/public", dockerfile)
        self.assertEqual(compose.count("stararm_vendor:"), 2)

    def test_confirmed_acceleration_value_is_preserved(self):
        dynamics = (
            ARM_ROOT / "patches" / "star-arm-102-fl-moveit-dynamics.patch"
        ).read_text()
        self.assertIn("max_acceleration: 30.0", dynamics)

    def test_compose_is_the_only_service_entry_and_owns_ipc(self):
        compose = (ARM_ROOT.parent / "compose.yaml").read_text()

        self.assertFalse((ARM_ROOT.parent / "manage-simulation-services.sh").exists())
        self.assertIn("nolo-usb-server:", compose)
        self.assertIn("moveit:", compose)
        self.assertIn("servo-ipc:/ipc", compose)
        self.assertIn("additional_contexts:", compose)
        self.assertIn("192.168.100.10:8765:8765", compose)
        self.assertIn("--host=0.0.0.0", compose)
        self.assertNotIn("network_mode:", compose)

        server = (
            ARM_ROOT.parent / "vr-xr" / "src" / "nolo-usb-server" / "src" / "main.rs"
        ).read_text()
        self.assertIn("failed removing previous Servo IPC socket", server)
        self.assertNotIn("refusing to replace existing Servo IPC socket", server)

    def test_images_are_built_before_the_services_start(self):
        moveit_image = (ROS_ROOT / "Dockerfile").read_text()
        nolo_image = (
            ARM_ROOT.parent / "vr-xr" / "src" / "nolo-usb-server" / "Dockerfile"
        ).read_text()

        self.assertIn("colcon build", moveit_image)
        self.assertIn("COPY --from=stararm_vendor", moveit_image)
        self.assertIn('ENTRYPOINT ["/usr/local/bin/start-moveit"]', moveit_image)
        self.assertIn("cargo build", nolo_image)
        self.assertIn("COPY arm/src/fashionstar-uart", nolo_image)
        self.assertIn('ENTRYPOINT ["/usr/local/bin/nolo-usb-server"]', nolo_image)

        server_root = ARM_ROOT.parent / "vr-xr" / "src" / "nolo-usb-server"
        server_cargo = (server_root / "Cargo.toml").read_text()
        arm_io = (server_root / "src" / "arm_io.rs").read_text()
        self.assertIn(
            'fashionstar-uart = { path = "../../../arm/src/fashionstar-uart" }',
            server_cargo,
        )
        self.assertIn("use fashionstar_uart::", arm_io)
        self.assertNotIn("fn request_packet", arm_io)


if __name__ == "__main__":
    unittest.main()
