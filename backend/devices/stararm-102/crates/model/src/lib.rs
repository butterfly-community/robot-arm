use std::{
    collections::BTreeMap,
    fs,
    path::{Component, Path, PathBuf},
};

use robot_arm_messages::{
    ActuatorMetadata, ArmCommand, JointMetadata, LinkMaterial, ModelAssetRequest,
    ModelAssetResponse, NamedMotionTarget, NumericFieldSchema, RobotModelInfo, SCHEMA_VERSION,
    VisualizationManifest,
};
use sha2::{Digest, Sha256};
use walkdir::WalkDir;

pub const MODEL_ID: &str = "stararm-102-fl";
pub const MODEL_REVISION: &str = "stararm-102-fl-v2";
pub const BASE_FRAME: &str = "base_link";
pub const TCP_FRAME: &str = "tcp_link";
pub const GRIPPER_ASSET_ID: &str = "stararm-102-fl";
pub const JOINTS: [&str; 6] = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"];
pub const GRIPPER_KEY: &str = "gripper";
pub const GRIPPER_JOINT: &str = "joint7_left";
pub const JOINT_LIMITS_DEGREES: [(f64, f64); 6] = [
    (-110.0, 110.0),
    (0.0, 180.0),
    (-270.0, 0.0),
    (-90.0, 90.0),
    (-65.0, 65.0),
    (-150.0, 150.0),
];
pub const DEFAULT_JOINTS_RAD: [f64; 6] = [0.0; 6];
pub const WORK_JOINT3_DEGREES: f64 = -60.0;
pub const WORK_JOINT4_DEGREES: f64 = 60.0;
pub const WORK_JOINT3_RAD: f64 = WORK_JOINT3_DEGREES.to_radians();
pub const WORK_JOINT4_RAD: f64 = WORK_JOINT4_DEGREES.to_radians();
pub const WORK_JOINTS_RAD: [f64; 6] = [0.0, 0.0, WORK_JOINT3_RAD, WORK_JOINT4_RAD, 0.0, 0.0];
pub const GRIPPER_DRIVE_JOINT_CLOSED_DEGREES: f64 = 0.0;
pub const GRIPPER_DRIVE_JOINT_CLOSED_RAD: f64 = GRIPPER_DRIVE_JOINT_CLOSED_DEGREES.to_radians();
pub const GRIPPER_DRIVE_JOINT_WORK_OPEN_DEGREES: f64 = 60.0;
pub const GRIPPER_DRIVE_JOINT_WORK_OPEN_RAD: f64 =
    GRIPPER_DRIVE_JOINT_WORK_OPEN_DEGREES.to_radians();
pub const GRIPPER_DRIVE_JOINT_MECHANICAL_LIMIT_DEGREES: f64 = 90.0;
pub const GRIPPER_DRIVE_JOINT_MECHANICAL_LIMIT_RAD: f64 =
    GRIPPER_DRIVE_JOINT_MECHANICAL_LIMIT_DEGREES.to_radians();
/// Offset from the gripper jaw pivot to `tcp_link` at the `link6` origin.
///
/// Both finger meshes extend from their z=-73.13 mm pivots back to z=0, so
/// `tcp_link` is the explicit gripper-tip center used by MoveIt. This offset is
/// only used to construct the demonstrated arc motion.
pub const ARC_PIVOT_TO_TCP_M: [f64; 3] = [0.0, 0.0, 0.07313];

pub struct ModelCatalog {
    root: PathBuf,
    files: Vec<String>,
    root_path: String,
    manifest_hash: String,
    joint_bounds: BTreeMap<String, (f64, f64)>,
}

impl ModelCatalog {
    pub fn load(root: impl Into<PathBuf>) -> Result<Self, String> {
        let root = root.into();
        let mut files = WalkDir::new(&root)
            .into_iter()
            .filter_map(Result::ok)
            .filter(|entry| entry.file_type().is_file())
            .map(|entry| {
                entry
                    .path()
                    .strip_prefix(&root)
                    .map(|path| path.to_string_lossy().into_owned())
                    .map_err(|error| error.to_string())
            })
            .collect::<Result<Vec<_>, _>>()?;
        files.sort();
        let root_path = files
            .iter()
            .find(|name| name.ends_with(".urdf") || name.ends_with(".xacro"))
            .cloned()
            .ok_or_else(|| format!("模型目录中没有 URDF：{}", root.display()))?;
        let robot = urdf_rs::read_file(root.join(&root_path)).map_err(|error| error.to_string())?;
        let joint_bounds = robot
            .joints
            .into_iter()
            .filter(|joint| JOINTS.contains(&joint.name.as_str()))
            .map(|joint| (joint.name, (joint.limit.lower, joint.limit.upper)))
            .collect::<BTreeMap<_, _>>();
        let missing = JOINTS
            .iter()
            .filter(|name| !joint_bounds.contains_key(**name))
            .copied()
            .collect::<Vec<_>>();
        if !missing.is_empty() {
            return Err(format!("URDF 缺少主动关节范围：{missing:?}"));
        }
        for ((name, expected), index) in JOINTS.iter().zip(joint_limits_rad()).zip(0..) {
            let actual = joint_bounds[*name];
            if (actual.0 - expected.0).abs() > 1e-9 || (actual.1 - expected.1).abs() > 1e-9 {
                return Err(format!(
                    "URDF {} 范围与型号定义不一致：期望 {:.9}..{:.9} rad，实际 {:.9}..{:.9} rad（J{}）",
                    name,
                    expected.0,
                    expected.1,
                    actual.0,
                    actual.1,
                    index + 1
                ));
            }
        }
        let mut digest = Sha256::new();
        for relative in &files {
            digest.update(relative.as_bytes());
            digest.update(fs::read(root.join(relative)).map_err(|error| error.to_string())?);
        }
        Ok(Self {
            root,
            files,
            root_path,
            manifest_hash: hex::encode(digest.finalize()),
            joint_bounds,
        })
    }

    pub fn model_info(&self) -> RobotModelInfo {
        RobotModelInfo {
            schema_version: SCHEMA_VERSION,
            model_id: MODEL_ID.into(),
            model_revision: MODEL_REVISION.into(),
            display_name: "StarArm-102-FL".into(),
            base_frame: BASE_FRAME.into(),
            tcp_frame: TCP_FRAME.into(),
            gripper_asset_id: Some(GRIPPER_ASSET_ID.into()),
            joints: JOINTS
                .iter()
                .enumerate()
                .map(|(index, name)| {
                    let (minimum, maximum) = self.joint_bounds[*name];
                    JointMetadata {
                        key: (*name).into(),
                        label: format!("J{}", index + 1),
                        unit: "rad".into(),
                        minimum,
                        maximum,
                    }
                })
                .collect(),
            tool_actuators: tool_actuators(),
            named_targets: vec![
                named_target("default", "默认位", DEFAULT_JOINTS_RAD),
                named_target("work", "工作位", WORK_JOINTS_RAD),
            ],
            calibration_targets: calibration_targets(),
            motion_options: [
                ("velocity_scaling", "速度倍率"),
                ("acceleration_scaling", "加速度倍率"),
            ]
            .into_iter()
            .map(|(key, label)| NumericFieldSchema {
                key: key.into(),
                label: label.into(),
                unit: "ratio".into(),
                minimum: Some(0.0),
                maximum: Some(1.0),
                required: false,
            })
            .collect(),
            diagnostics: vec![],
            visualization: VisualizationManifest {
                manifest_hash: self.manifest_hash.clone(),
                root_path: self.root_path.clone(),
                files: self.files.clone(),
                link_materials: link_materials(),
            },
        }
    }

    pub fn asset_response(&self, request: ModelAssetRequest) -> ModelAssetResponse {
        let relative = Path::new(&request.relative_path);
        let mut response = ModelAssetResponse {
            schema_version: SCHEMA_VERSION,
            request_id: request.request_id,
            model_revision: MODEL_REVISION.into(),
            manifest_hash: self.manifest_hash.clone(),
            relative_path: request.relative_path.clone(),
            mime_type: None,
            content_hash: None,
            content: None,
            original_error: None,
        };
        if request.model_revision != MODEL_REVISION
            || request.manifest_hash != self.manifest_hash
            || relative.is_absolute()
            || relative
                .components()
                .any(|part| part == Component::ParentDir)
        {
            response.original_error =
                Some("asset request does not match the current manifest".into());
            return response;
        }
        match fs::read(self.root.join(relative)) {
            Ok(content) => {
                response.mime_type = Some(
                    mime_guess::from_path(relative)
                        .first_or_octet_stream()
                        .essence_str()
                        .into(),
                );
                response.content_hash = Some(hex::encode(Sha256::digest(&content)));
                response.content = Some(content);
            }
            Err(error) => response.original_error = Some(error.to_string()),
        }
        response
    }
}

pub fn joint_limits_rad() -> [(f64, f64); 6] {
    JOINT_LIMITS_DEGREES.map(|(minimum, maximum)| (minimum.to_radians(), maximum.to_radians()))
}

pub fn validate_command(command: &ArmCommand) -> Result<(), String> {
    if command.model_revision != MODEL_REVISION
        || command.joints_rad.len() != JOINTS.len()
        || command.actuators_rad.len() != 1
    {
        return Err(format!(
            "ArmCommand 与执行模型不一致：revision={} joints={} actuators={}",
            command.model_revision,
            command.joints_rad.len(),
            command.actuators_rad.len()
        ));
    }
    for (((name, value), (minimum_deg, maximum_deg)), (minimum, maximum)) in JOINTS
        .iter()
        .zip(&command.joints_rad)
        .zip(JOINT_LIMITS_DEGREES)
        .zip(joint_limits_rad())
    {
        if !value.is_finite() || *value < minimum || *value > maximum {
            return Err(format!(
                "{name} 电机角 {:.3}° 超出范围 {minimum_deg:.0}°..{maximum_deg:.0}°",
                value.to_degrees()
            ));
        }
    }
    let gripper = command.actuators_rad[0];
    if !gripper.is_finite()
        || !(GRIPPER_DRIVE_JOINT_CLOSED_RAD..=GRIPPER_DRIVE_JOINT_MECHANICAL_LIMIT_RAD)
            .contains(&gripper)
    {
        return Err(format!(
            "夹爪驱动关节角 {:.3}° 超出范围 0°..{GRIPPER_DRIVE_JOINT_MECHANICAL_LIMIT_DEGREES:.0}°",
            gripper.to_degrees()
        ));
    }
    Ok(())
}

fn named_target(key: &str, label: &str, joints: [f64; 6]) -> NamedMotionTarget {
    NamedMotionTarget {
        key: key.into(),
        label: label.into(),
        joint_positions_rad: JOINTS
            .into_iter()
            .zip(joints)
            .map(|(name, position)| (name.into(), position))
            .collect(),
        actuator_positions_rad: [(GRIPPER_KEY.into(), GRIPPER_DRIVE_JOINT_CLOSED_RAD)]
            .into_iter()
            .collect(),
    }
}

fn calibration_targets() -> Vec<NamedMotionTarget> {
    // FK-verified board poses for the centre-line RGB-D fixture. The wrist
    // rotates about independent axes; camera/board translation is not observable
    // enough if J6 stays fixed. These are joint angles, never servo bus angles.
    [
        [-37.82, 116.92, -183.36, 4.07, 50.84, -86.63],
        [0.0, 117.89, -117.17, -85.18, 0.0, 0.0],
        [39.86, 91.83, -129.99, -26.48, -51.76, 83.71],
        [-10.49, 134.09, -156.02, -61.93, 15.86, -105.79],
        [36.78, 104.11, -120.57, -65.17, -44.58, -43.16],
        [-40.31, 88.62, -144.82, 13.5, 48.09, 143.51],
        [25.06, 135.68, -166.64, -51.69, -30.24, -150.0],
        [0.0, 76.48, -137.37, 36.43, 0.0, 0.0],
        [-25.46, 122.76, -137.1, -68.94, 30.05, 150.0],
    ]
    .into_iter()
    .enumerate()
    .map(|(index, degrees)| {
        named_target(
            &format!("calibration-{}", index + 1),
            &format!("标定姿态 {}", index + 1),
            degrees.map(f64::to_radians),
        )
    })
    .collect()
}

fn tool_actuators() -> Vec<ActuatorMetadata> {
    vec![ActuatorMetadata {
        key: GRIPPER_KEY.into(),
        label: "夹爪驱动关节角".into(),
        unit: "rad".into(),
        minimum: GRIPPER_DRIVE_JOINT_CLOSED_RAD,
        maximum: GRIPPER_DRIVE_JOINT_MECHANICAL_LIMIT_RAD,
        visualization_joint_key: Some(GRIPPER_JOINT.into()),
    }]
}

fn link_materials() -> BTreeMap<String, LinkMaterial> {
    let mut materials = BTreeMap::new();
    materials.insert(
        BASE_FRAME.into(),
        LinkMaterial {
            color_rgb: [0.196, 0.298, 0.365],
            metalness: 0.16,
            roughness: 0.56,
        },
    );
    for name in ["link1", "link2", "link3", "link4", "link5", "link6"] {
        materials.insert(
            name.into(),
            LinkMaterial {
                color_rgb: [0.863, 0.906, 0.929],
                metalness: 0.16,
                roughness: 0.56,
            },
        );
    }
    for name in ["link7_left", "link7_right"] {
        materials.insert(
            name.into(),
            LinkMaterial {
                color_rgb: [0.145, 0.216, 0.275],
                metalness: 0.42,
                roughness: 0.48,
            },
        );
    }
    materials
}

#[cfg(test)]
mod tests {
    use super::*;
    use quick_xml::{Reader, events::Event, name::QName};

    const GRASPGENX_MANIFEST: &str = include_str!("../../../graspgenx/manifest.json");
    const MOVEIT_SRDF: &str =
        include_str!("../../../ros2/motion-node/config/stararm102_description.srdf");
    const MODEL_PATCH: &str = include_str!("../../../patches/model.patch");
    const TOPIC_IO_PATCH: &str = include_str!("../../../patches/topic-io.patch");
    const SERIALIZED_RAD_TOLERANCE: f64 = 0.5e-9;

    fn assert_serialized_rad(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() <= SERIALIZED_RAD_TOLERANCE,
            "serialized angle {actual} differs from model angle {expected}"
        );
    }

    fn srdf_group_state_joint(group_state: &str, joint: &str) -> f64 {
        let mut reader = Reader::from_str(MOVEIT_SRDF);
        let mut active_group_state = false;
        loop {
            match reader.read_event().expect("SRDF must be valid XML") {
                Event::Start(element) if element.name() == QName(b"group_state") => {
                    active_group_state = element.attributes().flatten().any(|attribute| {
                        attribute.key == QName(b"name")
                            && attribute.value.as_ref() == group_state.as_bytes()
                    });
                }
                Event::Empty(element)
                    if active_group_state && element.name() == QName(b"joint") =>
                {
                    let mut name_matches = false;
                    let mut value = None;
                    for attribute in element.attributes().flatten() {
                        if attribute.key == QName(b"name") {
                            name_matches = attribute.value.as_ref() == joint.as_bytes();
                        } else if attribute.key == QName(b"value") {
                            value = Some(
                                std::str::from_utf8(attribute.value.as_ref())
                                    .expect("joint value must be UTF-8")
                                    .parse::<f64>()
                                    .expect("joint value must be numeric"),
                            );
                        }
                    }
                    if name_matches {
                        return value.expect("matching SRDF joint must have a value");
                    }
                }
                Event::End(element) if element.name() == QName(b"group_state") => {
                    active_group_state = false;
                }
                Event::Eof => panic!("missing {group_state}/{joint} in SRDF"),
                _ => {}
            }
        }
    }

    #[test]
    fn named_targets_share_the_declared_joint_order_and_closed_gripper() {
        let target = named_target("default", "默认位", DEFAULT_JOINTS_RAD);
        assert_eq!(target.joint_positions_rad.len(), JOINTS.len());
        assert_eq!(target.joint_positions_rad["joint3"], 0.0);
        assert_eq!(
            target.actuator_positions_rad[GRIPPER_KEY],
            GRIPPER_DRIVE_JOINT_CLOSED_RAD
        );
    }

    #[test]
    fn work_target_uses_the_declared_software_coordinates() {
        let target = named_target("work", "工作位", WORK_JOINTS_RAD);
        assert_eq!(target.joint_positions_rad["joint3"], WORK_JOINT3_RAD);
        assert_eq!(target.joint_positions_rad["joint4"], WORK_JOINT4_RAD);
    }

    #[test]
    fn calibration_targets_are_complete_and_use_declared_coordinates() {
        let targets = calibration_targets();
        let expected_degrees: [[f64; 6]; 9] = [
            [-37.82, 116.92, -183.36, 4.07, 50.84, -86.63],
            [0.0, 117.89, -117.17, -85.18, 0.0, 0.0],
            [39.86, 91.83, -129.99, -26.48, -51.76, 83.71],
            [-10.49, 134.09, -156.02, -61.93, 15.86, -105.79],
            [36.78, 104.11, -120.57, -65.17, -44.58, -43.16],
            [-40.31, 88.62, -144.82, 13.5, 48.09, 143.51],
            [25.06, 135.68, -166.64, -51.69, -30.24, -150.0],
            [0.0, 76.48, -137.37, 36.43, 0.0, 0.0],
            [-25.46, 122.76, -137.1, -68.94, 30.05, 150.0],
        ];
        assert_eq!(targets.len(), expected_degrees.len());
        for (target, expected) in targets.iter().zip(expected_degrees) {
            for (joint, expected_degrees) in JOINTS.iter().zip(expected) {
                assert_eq!(
                    target.joint_positions_rad[*joint],
                    expected_degrees.to_radians()
                );
            }
        }
        assert!(targets.iter().all(|target| {
            target.joint_positions_rad.len() == JOINTS.len()
                && JOINTS
                    .iter()
                    .all(|joint| target.joint_positions_rad.contains_key(*joint))
        }));
        assert!(
            targets
                .iter()
                .all(|target| { target.joint_positions_rad["joint3"] <= (-10.0_f64).to_radians() })
        );
        assert!(
            targets
                .iter()
                .all(|target| target.joint_positions_rad["joint2"] >= 0.0)
        );
    }

    #[test]
    fn public_actuator_range_uses_the_mechanical_limit() {
        let actuator = &tool_actuators()[0];
        assert_eq!(actuator.minimum, GRIPPER_DRIVE_JOINT_CLOSED_RAD);
        assert_eq!(actuator.maximum, GRIPPER_DRIVE_JOINT_MECHANICAL_LIMIT_RAD);
    }

    #[test]
    fn command_limits_cover_every_motor_and_gripper() {
        let joint_limits_rad = joint_limits_rad();
        let command = ArmCommand {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            controller_time_ns: 2,
            model_revision: MODEL_REVISION.into(),
            joints_rad: joint_limits_rad
                .iter()
                .map(|(_, maximum)| *maximum)
                .collect(),
            actuators_rad: vec![GRIPPER_DRIVE_JOINT_MECHANICAL_LIMIT_RAD],
        };
        validate_command(&command).unwrap();

        for index in 0..JOINTS.len() {
            for invalid_value in [
                joint_limits_rad[index].0 - 0.001,
                joint_limits_rad[index].1 + 0.001,
            ] {
                let mut invalid = command.clone();
                invalid.joints_rad[index] = invalid_value;
                assert!(
                    validate_command(&invalid)
                        .unwrap_err()
                        .contains(JOINTS[index])
                );
            }
        }

        let mut invalid_gripper = command;
        invalid_gripper.actuators_rad[0] = GRIPPER_DRIVE_JOINT_MECHANICAL_LIMIT_RAD + 0.001;
        assert!(
            validate_command(&invalid_gripper)
                .unwrap_err()
                .contains("夹爪")
        );
    }

    #[test]
    fn moveit_and_graspgenx_use_the_model_working_values() {
        let manifest: serde_json::Value = serde_json::from_str(GRASPGENX_MANIFEST).unwrap();
        let descriptor = &manifest["descriptors"][0];
        assert_eq!(
            descriptor["drive_joint_work_open_degrees"]["joint7_left"],
            GRIPPER_DRIVE_JOINT_WORK_OPEN_DEGREES
        );
        assert_eq!(
            descriptor["drive_joint_work_open_degrees"]["joint7_right"],
            -GRIPPER_DRIVE_JOINT_WORK_OPEN_DEGREES
        );
        assert_eq!(
            descriptor["drive_joint_closed_degrees"]["joint7_left"],
            GRIPPER_DRIVE_JOINT_CLOSED_DEGREES
        );
        assert_eq!(
            descriptor["drive_joint_closed_degrees"]["joint7_right"],
            GRIPPER_DRIVE_JOINT_CLOSED_DEGREES
        );

        assert_serialized_rad(srdf_group_state_joint("work", "joint3"), WORK_JOINT3_RAD);
        assert_serialized_rad(srdf_group_state_joint("work", "joint4"), WORK_JOINT4_RAD);
        assert_serialized_rad(
            srdf_group_state_joint("open", GRIPPER_JOINT),
            GRIPPER_DRIVE_JOINT_WORK_OPEN_RAD,
        );
        assert_serialized_rad(
            srdf_group_state_joint("closed", GRIPPER_JOINT),
            GRIPPER_DRIVE_JOINT_CLOSED_RAD,
        );
    }

    #[test]
    fn urdf_and_ros_control_share_the_mechanical_limit() {
        let limit = format!("{GRIPPER_DRIVE_JOINT_MECHANICAL_LIMIT_RAD:.9}");
        assert!(MODEL_PATCH.contains(&format!("upper=\"{limit}\"")));
        assert!(MODEL_PATCH.contains(&format!("lower=\"-{limit}\"")));
        assert!(TOPIC_IO_PATCH.contains(&format!(">{limit}</param>")));
    }
}
