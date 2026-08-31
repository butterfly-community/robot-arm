use std::{
    collections::BTreeMap,
    fs,
    path::{Component, Path, PathBuf},
};

use robot_arm_messages::{
    ActuatorMetadata, JointMetadata, LinkMaterial, ModelAssetRequest, ModelAssetResponse,
    NamedMotionTarget, NumericFieldSchema, RobotModelInfo, SCHEMA_VERSION, VisualizationManifest,
};
use sha2::{Digest, Sha256};
use walkdir::WalkDir;

pub const MODEL_ID: &str = "stararm-102-fl";
pub const MODEL_REVISION: &str = "stararm-102-fl-v1";
pub const BASE_FRAME: &str = "base_link";
pub const TCP_FRAME: &str = "tcp_link";
pub const JOINTS: [&str; 6] = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"];
pub const GRIPPER_KEY: &str = "gripper";
pub const GRIPPER_JOINT: &str = "joint7_left";
pub const DEFAULT_JOINTS_RAD: [f64; 6] = [0.0; 6];
pub const WORK_JOINTS_RAD: [f64; 6] = [
    0.0,
    0.0,
    60.0_f64.to_radians(),
    60.0_f64.to_radians(),
    0.0,
    0.0,
];
pub const CLOSED_GRIPPER_RAD: f64 = 0.0;
pub const OPEN_GRIPPER_RAD: f64 = std::f64::consts::FRAC_PI_2;
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
            tool_actuators: vec![ActuatorMetadata {
                key: GRIPPER_KEY.into(),
                label: "夹爪".into(),
                unit: "rad".into(),
                minimum: 0.0,
                maximum: 90.0_f64.to_radians(),
                visualization_joint_key: Some(GRIPPER_JOINT.into()),
            }],
            named_targets: vec![
                named_target("default", "默认位", DEFAULT_JOINTS_RAD),
                named_target("work", "工作位", WORK_JOINTS_RAD),
            ],
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

fn named_target(key: &str, label: &str, joints: [f64; 6]) -> NamedMotionTarget {
    NamedMotionTarget {
        key: key.into(),
        label: label.into(),
        joint_positions_rad: JOINTS
            .into_iter()
            .zip(joints)
            .map(|(name, position)| (name.into(), position))
            .collect(),
        actuator_positions_rad: [(GRIPPER_KEY.into(), CLOSED_GRIPPER_RAD)]
            .into_iter()
            .collect(),
    }
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

    #[test]
    fn named_targets_share_the_declared_joint_order_and_closed_gripper() {
        let target = named_target("default", "默认位", DEFAULT_JOINTS_RAD);
        assert_eq!(target.joint_positions_rad.len(), JOINTS.len());
        assert_eq!(target.joint_positions_rad["joint3"], 0.0);
        assert_eq!(
            target.actuator_positions_rad[GRIPPER_KEY],
            CLOSED_GRIPPER_RAD
        );
    }

    #[test]
    fn work_target_uses_the_declared_software_coordinates() {
        let target = named_target("work", "工作位", WORK_JOINTS_RAD);
        assert_eq!(target.joint_positions_rad["joint3"], 60.0_f64.to_radians());
        assert_eq!(target.joint_positions_rad["joint4"], 60.0_f64.to_radians());
    }
}
