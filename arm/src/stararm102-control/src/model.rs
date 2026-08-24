use nalgebra::{Isometry3, Matrix3, Translation3, Unit, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

const PROFILE_JSON: &str = include_str!("../../../config/stararm102-fl.v1.json");
const ARM_DOF: usize = 6;

#[derive(Clone, Debug, Deserialize)]
pub struct ArmProfile {
    pub schema_version: u32,
    pub profile_id: String,
    pub hardware_variant: String,
    pub model_id: String,
    pub simulation_only: bool,
    pub transport: TransportProfile,
    pub joints: Vec<JointProfile>,
    pub kinematics: KinematicsProfile,
    pub teleoperation: TeleoperationProfile,
}

#[derive(Clone, Debug, Deserialize)]
pub struct TransportProfile {
    pub backend: String,
    pub minimum_backend_version: String,
    pub baudrate: u32,
    pub stable_device: Option<String>,
    pub feedback_register: String,
    pub feedback_unit: String,
}

#[derive(Clone, Debug, Deserialize)]
pub struct JointProfile {
    pub name: String,
    pub servo_id: u8,
    pub servo_model: String,
    pub direction: f64,
    pub lower_deg: f64,
    pub upper_deg: f64,
    pub zero_offset_deg: Option<f64>,
    pub firmware: Option<String>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct KinematicsProfile {
    pub base_link: String,
    pub tcp_link: String,
    pub geometry_source: String,
    pub joint_origins: Vec<TransformProfile>,
    pub tcp_origin: TcpTransformProfile,
    pub nominal_joints_deg: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct TransformProfile {
    pub xyz_m: [f64; 3],
    pub rpy_rad: [f64; 3],
    pub axis: [f64; 3],
}

#[derive(Clone, Debug, Deserialize)]
pub struct TcpTransformProfile {
    pub xyz_m: [f64; 3],
    pub rpy_rad: [f64; 3],
}

#[derive(Clone, Debug, Deserialize)]
pub struct TeleoperationProfile {
    pub translation_scale: f64,
    pub robot_from_nolo_position_axes: [[f64; 3]; 3],
    pub robot_from_controller_orientation_axes: [[f64; 3]; 3],
    pub workspace_radius_m: [f64; 2],
    pub workspace_z_m: [f64; 2],
    pub max_joint_speed_rad_s: f64,
    pub max_joint_acceleration_rad_s2: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub position_m: [f64; 3],
    /// Unit quaternion in `[x, y, z, w]` order.
    pub orientation_xyzw: [f64; 4],
}

impl Pose {
    pub fn identity() -> Self {
        Self::from_isometry(&Isometry3::identity())
    }

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
        if quaternion.norm_squared() <= f64::EPSILON {
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
    joint_origins: [Isometry3<f64>; ARM_DOF],
    joint_axes: [Unit<Vector3<f64>>; ARM_DOF],
    joint_model_scale: [f64; ARM_DOF],
    tcp_origin: Isometry3<f64>,
    lower_limits: [f64; ARM_DOF],
    upper_limits: [f64; ARM_DOF],
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
        let joint_origins = std::array::from_fn(|index| {
            let origin = &profile.kinematics.joint_origins[index];
            isometry(origin.xyz_m, origin.rpy_rad)
        });
        let joint_axes = std::array::from_fn(|index| {
            Unit::new_normalize(Vector3::from(profile.kinematics.joint_origins[index].axis))
        });
        let joint_model_scale = std::array::from_fn(|index| 1.0 / profile.joints[index].direction);
        let tcp_origin = isometry(
            profile.kinematics.tcp_origin.xyz_m,
            profile.kinematics.tcp_origin.rpy_rad,
        );
        let lower_limits =
            std::array::from_fn(|index| profile.joints[index].lower_deg.to_radians());
        let upper_limits =
            std::array::from_fn(|index| profile.joints[index].upper_deg.to_radians());
        let nominal_joints =
            std::array::from_fn(|index| profile.kinematics.nominal_joints_deg[index].to_radians());
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
            joint_origins,
            joint_axes,
            joint_model_scale,
            tcp_origin,
            lower_limits,
            upper_limits,
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

    pub fn joint_limits(&self) -> ([f64; ARM_DOF], [f64; ARM_DOF]) {
        (self.lower_limits, self.upper_limits)
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

    pub fn forward(&self, joints: [f64; ARM_DOF]) -> Pose {
        let mut transform = Isometry3::identity();
        let model_joints = self.model_joints(joints);
        for (index, model_joint) in model_joints.into_iter().enumerate() {
            transform *= self.joint_origins[index];
            transform *= Isometry3::from_parts(
                Translation3::identity(),
                UnitQuaternion::from_axis_angle(&self.joint_axes[index], model_joint),
            );
        }
        Pose::from_isometry(&(transform * self.tcp_origin))
    }

    pub fn workspace_contains(&self, position: [f64; 3]) -> bool {
        if position.into_iter().any(|value| !value.is_finite()) {
            return false;
        }
        let radius = Vector3::from(position).norm();
        let [min_radius, max_radius] = self.profile.teleoperation.workspace_radius_m;
        let [min_z, max_z] = self.profile.teleoperation.workspace_z_m;
        (min_radius..=max_radius).contains(&radius) && (min_z..=max_z).contains(&position[2])
    }
}

fn isometry(xyz: [f64; 3], rpy: [f64; 3]) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(Vector3::from(xyz)),
        UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]),
    )
}

fn validate_profile(profile: &ArmProfile) -> Result<(), String> {
    if profile.schema_version != 1 || !profile.simulation_only {
        return Err("profile must be schema v1 and explicitly simulation_only".to_owned());
    }
    if profile.joints.len() != 7
        || profile.kinematics.joint_origins.len() != ARM_DOF
        || profile.kinematics.nominal_joints_deg.len() != ARM_DOF
    {
        return Err("profile must define six arm joints plus one gripper".to_owned());
    }
    let ids: BTreeSet<_> = profile.joints.iter().map(|joint| joint.servo_id).collect();
    if ids != BTreeSet::from([0, 1, 2, 3, 4, 5, 6]) {
        return Err("servo IDs must be exactly 0..=6".to_owned());
    }
    for joint in &profile.joints {
        if joint.direction == 0.0
            || !joint.direction.is_finite()
            || !joint.lower_deg.is_finite()
            || !joint.upper_deg.is_finite()
            || joint.lower_deg >= joint.upper_deg
        {
            return Err(format!("invalid joint profile: {}", joint.name));
        }
    }
    for (index, value) in profile
        .kinematics
        .nominal_joints_deg
        .iter()
        .copied()
        .enumerate()
    {
        let joint = &profile.joints[index];
        if !value.is_finite() || value < joint.lower_deg || value > joint.upper_deg {
            return Err(format!("nominal pose exceeds {} limits", joint.name));
        }
    }
    for origin in &profile.kinematics.joint_origins {
        if origin
            .xyz_m
            .into_iter()
            .chain(origin.rpy_rad)
            .chain(origin.axis)
            .any(|value| !value.is_finite())
            || Vector3::from(origin.axis).norm() <= f64::EPSILON
        {
            return Err("invalid kinematic transform".to_owned());
        }
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
        let mapping = Matrix3::from_row_slice(&values.into_iter().flatten().collect::<Vec<_>>());
        if (mapping.transpose() * mapping - Matrix3::identity()).norm() > 1.0e-9
            || (mapping.determinant() - 1.0).abs() > 1.0e-9
        {
            return Err(format!("{name} must be a proper orthonormal rotation"));
        }
    }
    let settings = &profile.teleoperation;
    let positive = [
        settings.translation_scale,
        settings.max_joint_speed_rad_s,
        settings.max_joint_acceleration_rad_s2,
    ];
    if positive
        .into_iter()
        .any(|value| !value.is_finite() || value <= 0.0)
        || settings.workspace_radius_m[0] >= settings.workspace_radius_m[1]
        || settings.workspace_z_m[0] >= settings.workspace_z_m[1]
    {
        return Err("invalid teleoperation settings".to_owned());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_profile_is_explicitly_simulation_only_and_complete() {
        let model = ArmModel::embedded().unwrap();
        assert!(model.profile().simulation_only);
        assert_eq!(model.profile().joints.len(), 7);
        assert_eq!(
            model
                .profile()
                .joints
                .iter()
                .map(|joint| joint.servo_id)
                .collect::<Vec<_>>(),
            (0..=6).collect::<Vec<_>>()
        );
        assert_eq!(model.profile().transport.stable_device, None);
        assert!(
            model
                .profile()
                .joints
                .iter()
                .all(|joint| joint.zero_offset_deg.is_none() && joint.firmware.is_none())
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
    fn forward_pose_is_finite() {
        let model = ArmModel::embedded().unwrap();
        let joints = model.nominal_joints();
        let pose = model.forward(joints);
        assert!(pose.position_m.into_iter().all(f64::is_finite));
        assert!(
            pose.position_m[2] > 0.0,
            "nominal TCP must be above base plane"
        );
        assert!(pose.orientation_xyzw.into_iter().all(f64::is_finite));
    }
}
