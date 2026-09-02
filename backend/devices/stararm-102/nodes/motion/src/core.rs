use std::path::Path;

use json_config_store::{load_or_default, save};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use robot_arm_messages::{ControlMode, TransformedControlFrame};
use serde::{Deserialize, Serialize};
use stararm_102_model::{ARC_PIVOT_TO_TCP_M, GRIPPER_DRIVE_JOINT_WORK_OPEN_RAD};

const CONFIG_SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MotionConfig {
    pub schema_version: u32,
    pub config_version: u64,
    pub control_mode: ControlMode,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            schema_version: CONFIG_SCHEMA_VERSION,
            config_version: 1,
            control_mode: ControlMode::Relative,
        }
    }
}

impl MotionConfig {
    pub fn load(path: &Path) -> eyre::Result<Self> {
        let config: Self = load_or_default(path)?;
        eyre::ensure!(
            config.schema_version == CONFIG_SCHEMA_VERSION,
            "不支持的 motion 配置版本 {}",
            config.schema_version
        );
        Ok(config)
    }

    pub fn set_mode(&mut self, path: &Path, mode: ControlMode) -> eyre::Result<()> {
        let mut next = self.clone();
        next.config_version += 1;
        next.control_mode = mode;
        save(path, &next)?;
        *self = next;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub position_m: [f64; 3],
    pub orientation_xyzw: [f64; 4],
}

pub fn target_pose(anchor: Pose, frame: &TransformedControlFrame) -> Pose {
    let anchor_rotation = quaternion(anchor.orientation_xyzw);
    let arc_rotation =
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), frame.horizontal_arc_rad)
            * anchor_rotation
            * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), frame.front_pitch_rad);
    let tool_rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), frame.tool_yaw_rad)
        * arc_rotation
        * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), frame.tool_pitch_rad)
        * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), frame.tool_roll_rad);
    let target_rotation = tool_rotation
        * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), frame.tool_helical_roll_rad);
    let pivot = Vector3::from(ARC_PIVOT_TO_TCP_M);
    let anchor_offset = anchor_rotation * pivot;
    let arc_offset = arc_rotation * pivot;
    let tool_axis = tool_rotation * Vector3::z();
    let tool_distance = frame.tool_axis_translation_m + frame.tool_helical_translation_m;
    let position =
        Vector3::from(anchor.position_m) + Vector3::from(frame.translation_m) + arc_offset
            - anchor_offset
            + tool_axis * tool_distance;
    Pose {
        position_m: position.into(),
        orientation_xyzw: xyzw(target_rotation),
    }
}

pub fn merge_controller_command(
    names: &[String],
    positions: &[f64],
    joint_names: &[&str],
    actuator_name: &str,
    current_joints: &[f64],
    current_actuator: f64,
) -> Option<(Vec<f64>, f64)> {
    if names.len() != positions.len() || current_joints.len() != joint_names.len() {
        return None;
    }
    let find = |name: &str| names.iter().position(|candidate| candidate == name);
    let joints = joint_names
        .iter()
        .zip(current_joints)
        .map(|(name, current)| {
            let value = positions[find(name)?];
            Some(if value.is_finite() { value } else { *current })
        })
        .collect::<Option<Vec<_>>>()?;
    let actuator_value = positions[find(actuator_name)?];
    let actuator = if actuator_value.is_finite() {
        actuator_value
    } else {
        current_actuator
    };
    let valid = joints
        .iter()
        .chain([&actuator])
        .all(|value| value.is_finite());
    valid.then_some((joints, actuator))
}

pub fn tool_action_transition(previous: Option<f64>, current: f64) -> Option<f64> {
    previous
        .filter(|previous| *previous != current)
        .map(|_| current)
}

pub fn tool_position_rad(value: f64) -> f64 {
    GRIPPER_DRIVE_JOINT_WORK_OPEN_RAD * (1.0 - value)
}

fn quaternion([x, y, z, w]: [f64; 4]) -> UnitQuaternion<f64> {
    UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z))
}

fn xyzw(value: UnitQuaternion<f64>) -> [f64; 4] {
    let value = value.quaternion();
    [value.i, value.j, value.k, value.w]
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use robot_arm_messages::{ActuatorActions, SCHEMA_VERSION};

    fn anchor() -> Pose {
        Pose {
            position_m: [0.2, -0.1, 0.3],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        }
    }

    fn frame() -> TransformedControlFrame {
        TransformedControlFrame {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            source_time_ns: 0,
            transformed_time_ns: 0,
            control_session_id: None,
            active: true,
            translation_m: [0.0; 3],
            front_pitch_rad: 0.0,
            horizontal_arc_rad: 0.0,
            tool_pitch_rad: 0.0,
            tool_yaw_rad: 0.0,
            tool_roll_rad: 0.0,
            tool_axis_translation_m: 0.0,
            tool_helical_translation_m: 0.0,
            tool_helical_roll_rad: 0.0,
            actuator_actions: ActuatorActions::default(),
        }
    }

    #[test]
    fn translation_moves_tcp_without_rotating_it() {
        let mut frame = frame();
        frame.translation_m = [0.01, 0.02, -0.03];
        let target = target_pose(anchor(), &frame);
        assert_abs_diff_eq!(&target.position_m[..], &[0.21, -0.08, 0.27][..]);
        assert_abs_diff_eq!(&target.orientation_xyzw[..], &anchor().orientation_xyzw[..]);
    }

    #[test]
    fn paired_arc_returns_to_the_anchor() {
        let mut frame = frame();
        frame.front_pitch_rad = 0.2;
        let raised = target_pose(anchor(), &frame);
        assert_ne!(raised.position_m, anchor().position_m);
        frame.front_pitch_rad = 0.0;
        let returned = target_pose(anchor(), &frame);
        assert_abs_diff_eq!(&returned.position_m[..], &anchor().position_m[..]);
        assert_abs_diff_eq!(
            &returned.orientation_xyzw[..],
            &anchor().orientation_xyzw[..]
        );
    }

    #[test]
    fn controller_outputs_merge_into_one_complete_command() {
        let names = ["joint1", "joint2", "gripper"].map(str::to_owned).to_vec();
        let merged = merge_controller_command(
            &names,
            &[1.0, f64::NAN, 0.4],
            &["joint1", "joint2"],
            "gripper",
            &[0.0, 0.2],
            0.0,
        );
        assert_eq!(merged, Some((vec![1.0, 0.2], 0.4)));
    }

    #[test]
    fn gripper_mapping_uses_the_device_working_open_pose() {
        assert_eq!(tool_position_rad(0.0), GRIPPER_DRIVE_JOINT_WORK_OPEN_RAD);
        assert_eq!(tool_position_rad(1.0), 0.0);
        assert_eq!(tool_action_transition(None, 0.5), None);
        assert_eq!(tool_action_transition(Some(0.5), 0.5), None);
        assert_eq!(tool_action_transition(Some(0.5), 0.75), Some(0.75));
    }
}
