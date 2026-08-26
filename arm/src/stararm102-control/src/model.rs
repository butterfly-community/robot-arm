use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

const PROFILE_JSON: &str = include_str!("../../../config/stararm102-fl.v1.json");
#[derive(Clone, Debug, Deserialize)]
pub struct ArmProfile {
    pub schema_version: u32,
    pub model_id: String,
    pub moveit_interface: MoveItInterfaceProfile,
    pub teleoperation: TeleoperationProfile,
}

#[derive(Clone, Debug, Deserialize)]
pub struct MoveItInterfaceProfile {
    pub base_frame: String,
    pub tcp_link: String,
    pub gripper_pivot_to_tcp_m: [f64; 3],
    pub gripper_open_deg: f64,
    pub gripper_closed_deg: f64,
}

#[derive(Clone, Debug, Deserialize)]
pub struct TeleoperationProfile {
    pub translation_scale: f64,
    pub robot_from_nolo_position_axes: [[f64; 3]; 3],
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
    robot_from_nolo_position: Matrix3<f64>,
}

impl ArmModel {
    pub fn embedded() -> Result<Self, String> {
        let profile: ArmProfile =
            serde_json::from_str(PROFILE_JSON).map_err(|error| error.to_string())?;
        Self::from_profile(profile)
    }

    pub fn from_profile(profile: ArmProfile) -> Result<Self, String> {
        validate_profile(&profile)?;
        let robot_from_nolo_position = Matrix3::from_row_slice(
            &profile
                .teleoperation
                .robot_from_nolo_position_axes
                .into_iter()
                .flatten()
                .collect::<Vec<_>>(),
        );
        Ok(Self {
            profile,
            robot_from_nolo_position,
        })
    }

    pub fn profile(&self) -> &ArmProfile {
        &self.profile
    }

    pub fn robot_from_nolo_position(&self) -> Matrix3<f64> {
        self.robot_from_nolo_position
    }

    pub fn gripper_pivot_to_tcp(&self) -> Vector3<f64> {
        Vector3::from(self.profile.moveit_interface.gripper_pivot_to_tcp_m)
    }
}

fn validate_profile(profile: &ArmProfile) -> Result<(), String> {
    if profile.schema_version != 1 {
        return Err("profile must use schema v1".to_owned());
    }
    if profile.moveit_interface.base_frame.is_empty()
        || profile.moveit_interface.tcp_link.is_empty()
        || profile
            .moveit_interface
            .gripper_pivot_to_tcp_m
            .into_iter()
            .any(|value| !value.is_finite())
        || !profile.moveit_interface.gripper_open_deg.is_finite()
        || !profile.moveit_interface.gripper_closed_deg.is_finite()
        || profile.moveit_interface.gripper_open_deg <= profile.moveit_interface.gripper_closed_deg
    {
        return Err("invalid MoveIt interface profile".to_owned());
    }
    let values = profile.teleoperation.robot_from_nolo_position_axes;
    if values.into_iter().flatten().any(|value| !value.is_finite()) {
        return Err("robot_from_nolo_position_axes must contain only finite values".to_owned());
    }
    let mapping = Matrix3::from_row_slice(&values.into_iter().flatten().collect::<Vec<_>>());
    if (mapping.transpose() * mapping - Matrix3::identity()).norm() > 1.0e-9
        || (mapping.determinant() - 1.0).abs() > 1.0e-9
    {
        return Err(
            "robot_from_nolo_position_axes must be a proper orthonormal rotation".to_owned(),
        );
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
        assert_eq!(model.profile().moveit_interface.gripper_open_deg, 90.0);
        assert_eq!(model.profile().moveit_interface.gripper_closed_deg, 1.0);
        assert_eq!(
            model.gripper_pivot_to_tcp(),
            Vector3::new(0.0, 0.0, 0.07313)
        );
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
    fn profile_validation_rejects_non_finite_or_ambiguous_control_data() {
        let profile = || serde_json::from_str::<ArmProfile>(PROFILE_JSON).unwrap();

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
