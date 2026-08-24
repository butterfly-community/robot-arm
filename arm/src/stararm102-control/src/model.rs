use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

const PROFILE_JSON: &str = include_str!("../../../config/stararm102-fl.v1.json");
const ARM_DOF: usize = 6;

#[derive(Clone, Debug, Deserialize)]
pub struct ArmProfile {
    pub schema_version: u32,
    pub model_id: String,
    pub joints: Vec<JointProfile>,
    pub moveit_interface: MoveItInterfaceProfile,
    pub teleoperation: TeleoperationProfile,
}

#[derive(Clone, Debug, Deserialize)]
pub struct JointProfile {
    pub name: String,
    pub direction: f64,
}

#[derive(Clone, Debug, Deserialize)]
pub struct MoveItInterfaceProfile {
    pub base_frame: String,
    pub tcp_link: String,
    pub nominal_joints_deg: Vec<f64>,
    pub gripper_open_deg: f64,
    pub gripper_closed_deg: f64,
}

#[derive(Clone, Debug, Deserialize)]
pub struct TeleoperationProfile {
    pub translation_scale: f64,
    pub robot_from_nolo_position_axes: [[f64; 3]; 3],
    pub robot_from_controller_orientation_axes: [[f64; 3]; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub position_m: [f64; 3],
    /// Unit quaternion in `[x, y, z, w]` order.
    pub orientation_xyzw: [f64; 4],
}

impl Pose {
    pub fn from_isometry(value: &Isometry3<f64>) -> Self {
        let quaternion = value.rotation.quaternion();
        let mut orientation = [quaternion.i, quaternion.j, quaternion.k, quaternion.w];
        if orientation[3] < 0.0 {
            orientation.iter_mut().for_each(|value| *value = -*value);
        }
        Self {
            position_m: value.translation.vector.into(),
            orientation_xyzw: orientation,
        }
    }

    pub fn to_isometry(self) -> Option<Isometry3<f64>> {
        if self.position_m.into_iter().any(|value| !value.is_finite())
            || self
                .orientation_xyzw
                .into_iter()
                .any(|value| !value.is_finite())
        {
            return None;
        }
        let [x, y, z, w] = self.orientation_xyzw;
        let quaternion = nalgebra::Quaternion::new(w, x, y, z);
        let norm_squared = quaternion.norm_squared();
        if !norm_squared.is_finite() || norm_squared <= f64::EPSILON {
            return None;
        }
        Some(Isometry3::from_parts(
            Translation3::from(Vector3::from(self.position_m)),
            UnitQuaternion::new_normalize(quaternion),
        ))
    }
}

#[derive(Clone, Debug)]
pub struct ArmModel {
    profile: ArmProfile,
    joint_model_scale: [f64; ARM_DOF],
    nominal_joints: [f64; ARM_DOF],
    robot_from_nolo_position: Matrix3<f64>,
    robot_from_controller_orientation: Matrix3<f64>,
}

impl ArmModel {
    pub fn embedded() -> Result<Self, String> {
        let profile: ArmProfile =
            serde_json::from_str(PROFILE_JSON).map_err(|error| error.to_string())?;
        Self::from_profile(profile)
    }

    pub fn from_profile(profile: ArmProfile) -> Result<Self, String> {
        validate_profile(&profile)?;
        let joint_model_scale = std::array::from_fn(|index| 1.0 / profile.joints[index].direction);
        let nominal_joints = std::array::from_fn(|index| {
            profile.moveit_interface.nominal_joints_deg[index].to_radians()
        });
        let robot_from_nolo_position = Matrix3::from_row_slice(
            &profile
                .teleoperation
                .robot_from_nolo_position_axes
                .into_iter()
                .flatten()
                .collect::<Vec<_>>(),
        );
        let robot_from_controller_orientation = Matrix3::from_row_slice(
            &profile
                .teleoperation
                .robot_from_controller_orientation_axes
                .into_iter()
                .flatten()
                .collect::<Vec<_>>(),
        );
        Ok(Self {
            profile,
            joint_model_scale,
            nominal_joints,
            robot_from_nolo_position,
            robot_from_controller_orientation,
        })
    }

    pub fn profile(&self) -> &ArmProfile {
        &self.profile
    }

    pub fn nominal_joints(&self) -> [f64; ARM_DOF] {
        self.nominal_joints
    }

    /// Converts logical FL joint coordinates to the joint coordinates used by
    /// the manufacturer URDF. The upstream FL driver defines
    /// `logical = model * direction` and writes `model = logical / direction`.
    pub fn model_joints(&self, logical_joints: [f64; ARM_DOF]) -> [f64; ARM_DOF] {
        std::array::from_fn(|index| logical_joints[index] * self.joint_model_scale[index])
    }

    /// Converts manufacturer URDF coordinates back to the logical FL
    /// coordinates used by the device profile and future hardware driver.
    pub fn logical_joints(&self, model_joints: [f64; ARM_DOF]) -> [f64; ARM_DOF] {
        std::array::from_fn(|index| model_joints[index] * self.profile.joints[index].direction)
    }

    pub fn robot_from_nolo_position(&self) -> Matrix3<f64> {
        self.robot_from_nolo_position
    }

    pub fn robot_from_controller_orientation(&self) -> Matrix3<f64> {
        self.robot_from_controller_orientation
    }
}

fn validate_profile(profile: &ArmProfile) -> Result<(), String> {
    if profile.schema_version != 1 {
        return Err("profile must use schema v1".to_owned());
    }
    if profile.joints.len() != 7 || profile.moveit_interface.nominal_joints_deg.len() != ARM_DOF {
        return Err("profile must define six arm joints plus one gripper".to_owned());
    }
    if profile.joints.iter().any(|joint| joint.name.is_empty())
        || profile.joints.iter().enumerate().any(|(index, joint)| {
            profile.joints[..index]
                .iter()
                .any(|other| other.name == joint.name)
        })
    {
        return Err("joint names must be non-empty and unique".to_owned());
    }
    if profile.moveit_interface.base_frame.is_empty()
        || profile.moveit_interface.tcp_link.is_empty()
        || !profile.moveit_interface.gripper_open_deg.is_finite()
        || !profile.moveit_interface.gripper_closed_deg.is_finite()
        || profile.moveit_interface.gripper_open_deg <= profile.moveit_interface.gripper_closed_deg
    {
        return Err("invalid MoveIt interface profile".to_owned());
    }
    for joint in &profile.joints {
        if joint.direction == 0.0
            || !joint.direction.is_finite()
            || !(1.0 / joint.direction).is_finite()
        {
            return Err(format!("invalid joint profile: {}", joint.name));
        }
    }
    if profile
        .moveit_interface
        .nominal_joints_deg
        .iter()
        .any(|value| !value.is_finite())
    {
        return Err("nominal pose must contain only finite values".to_owned());
    }
    for (name, values) in [
        (
            "robot_from_nolo_position_axes",
            profile.teleoperation.robot_from_nolo_position_axes,
        ),
        (
            "robot_from_controller_orientation_axes",
            profile.teleoperation.robot_from_controller_orientation_axes,
        ),
    ] {
        if values.into_iter().flatten().any(|value| !value.is_finite()) {
            return Err(format!("{name} must contain only finite values"));
        }
        let mapping = Matrix3::from_row_slice(&values.into_iter().flatten().collect::<Vec<_>>());
        if (mapping.transpose() * mapping - Matrix3::identity()).norm() > 1.0e-9
            || (mapping.determinant() - 1.0).abs() > 1.0e-9
        {
            return Err(format!("{name} must be a proper orthonormal rotation"));
        }
    }
    let settings = &profile.teleoperation;
    if !settings.translation_scale.is_finite() || settings.translation_scale <= 0.0 {
        return Err("invalid teleoperation settings".to_owned());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_profile_is_complete() {
        let model = ArmModel::embedded().unwrap();
        assert_eq!(model.profile().joints.len(), 7);
        assert_eq!(model.profile().moveit_interface.gripper_open_deg, 90.0);
        assert_eq!(model.profile().moveit_interface.gripper_closed_deg, 0.0);
    }

    #[test]
    fn position_mapping_is_right_up_forward_to_robot_frame() {
        let model = ArmModel::embedded().unwrap();
        let mapping = model.robot_from_nolo_position();
        assert_eq!(mapping * Vector3::x(), -Vector3::y());
        assert_eq!(mapping * Vector3::y(), Vector3::z());
        assert_eq!(mapping * -Vector3::z(), Vector3::x());
    }

    #[test]
    fn controller_body_rotation_mapping_matches_human_motion() {
        let model = ArmModel::embedded().unwrap();
        let mapping = model.robot_from_controller_orientation();
        assert_eq!(mapping * -Vector3::x(), -Vector3::y());
        assert_eq!(mapping * Vector3::y(), -Vector3::x());
        assert_eq!(mapping * Vector3::z(), Vector3::z());
    }

    #[test]
    fn logical_joint_directions_are_applied_to_working_pose() {
        let model = ArmModel::embedded().unwrap();
        let logical = [
            10.0_f64.to_radians(),
            -90.0_f64.to_radians(),
            -90.0_f64.to_radians(),
            20.0_f64.to_radians(),
            30.0_f64.to_radians(),
            -40.0_f64.to_radians(),
        ];
        let visual = model.model_joints(logical);
        let expected = [
            -10.0_f64.to_radians(),
            std::f64::consts::FRAC_PI_2,
            -std::f64::consts::FRAC_PI_2,
            20.0_f64.to_radians(),
            30.0_f64.to_radians(),
            40.0_f64.to_radians(),
        ];
        for index in 0..ARM_DOF {
            assert!((visual[index] - expected[index]).abs() < 1.0e-12);
        }
        assert_eq!(model.logical_joints(visual), logical);
    }

    #[test]
    fn confirmed_default_pose_is_all_zero() {
        let model = ArmModel::embedded().unwrap();
        assert_eq!(model.nominal_joints(), [0.0; ARM_DOF]);
        assert_eq!(model.model_joints(model.nominal_joints()), [0.0; ARM_DOF]);
    }

    #[test]
    fn profile_validation_rejects_non_finite_or_ambiguous_control_data() {
        let profile = || serde_json::from_str::<ArmProfile>(PROFILE_JSON).unwrap();

        let mut duplicate = profile();
        duplicate.joints[1].name = duplicate.joints[0].name.clone();
        assert!(ArmModel::from_profile(duplicate).is_err());

        let mut invalid_mapping = profile();
        invalid_mapping.teleoperation.robot_from_nolo_position_axes[0][0] = f64::NAN;
        assert!(ArmModel::from_profile(invalid_mapping).is_err());

        let mut invalid_scale = profile();
        invalid_scale.teleoperation.translation_scale = f64::NAN;
        assert!(ArmModel::from_profile(invalid_scale).is_err());
    }

    #[test]
    fn pose_rejects_a_quaternion_whose_norm_overflows() {
        let pose = Pose {
            position_m: [0.0; 3],
            orientation_xyzw: [f64::MAX, f64::MAX, 0.0, 0.0],
        };
        assert!(pose.to_isometry().is_none());
    }
}
