//! MoveIt Servo IPC protocol and the simulation output sink.
//!
//! This module performs coordinate conversion and validates Servo feedback. It
//! deliberately contains no inverse kinematics: joint targets come from MoveIt.

use crate::{
    model::{ArmModel, Pose},
    simulation::{
        RelativeIntent, RelativeIntentState, SimulationSnapshot, SimulationState,
        SimulationStopReason,
    },
};
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

pub const SERVO_IPC_SCHEMA_VERSION: u32 = 1;
pub const ARM_DOF: usize = 6;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ServoCommandFrame {
    pub schema_version: u32,
    pub time_ns: u64,
    pub intent_receive_time_ns: u64,
    pub intent_sample_sequence: Option<u8>,
    pub enabled: bool,
    pub base_frame: String,
    pub tcp_link: String,
    pub target_pose: Option<Pose>,
    pub gripper_closed: Option<bool>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ServoFeedbackFrame {
    pub schema_version: u32,
    pub sequence: u64,
    pub model_joint_names: Vec<String>,
    pub model_joints_rad: Vec<f64>,
    pub model_joint_velocity_rad_s: Vec<f64>,
    pub servo_status_code: Option<i8>,
    pub servo_status_message: Option<String>,
}

pub struct MoveItSimulationController {
    model: ArmModel,
    logical_joints: [f64; ARM_DOF],
    logical_joint_velocity: [f64; ARM_DOF],
    anchor_tcp: Option<nalgebra::Isometry3<f64>>,
    gripper_closed: bool,
    gripper_rad: f64,
    gripper_open_rad: f64,
    gripper_closed_rad: f64,
    gripper_velocity: f64,
    latest: SimulationSnapshot,
}

impl MoveItSimulationController {
    pub fn new(model: ArmModel) -> Self {
        let logical_joints = model.nominal_joints();
        let gripper = &model.profile().joints[ARM_DOF];
        let gripper_open_rad = (gripper.lower_deg / gripper.direction).to_radians();
        let gripper_closed_rad = (gripper.upper_deg / gripper.direction).to_radians();
        let gripper_rad = gripper_closed_rad;
        let model_joints = model.model_joints(logical_joints);
        let latest = SimulationSnapshot {
            model_id: model.profile().model_id.clone(),
            simulation_only: true,
            backend: "moveit_servo".to_owned(),
            state: SimulationState::Faulted,
            enabled: false,
            time_ns: 0,
            intent_receive_time_ns: 0,
            intent_sample_sequence: None,
            joints_rad: logical_joints,
            joints_deg: logical_joints.map(f64::to_degrees),
            model_joints_rad: model_joints,
            model_joints_deg: model_joints.map(f64::to_degrees),
            joint_velocity_rad_s: [0.0; ARM_DOF],
            gripper_closed: true,
            gripper_rad,
            gripper_deg: gripper_rad.to_degrees(),
            gripper_velocity_rad_s: 0.0,
            tcp_pose: model.forward(logical_joints),
            desired_tcp_pose: None,
            servo_status_code: None,
            servo_status_message: None,
            servo_feedback_age_ms: None,
            stop_reason: Some(SimulationStopReason::ServoUnavailable),
        };
        Self {
            model,
            logical_joints,
            logical_joint_velocity: [0.0; ARM_DOF],
            anchor_tcp: None,
            gripper_closed: true,
            gripper_rad,
            gripper_open_rad,
            gripper_closed_rad,
            gripper_velocity: 0.0,
            latest,
        }
    }

    pub fn latest(&self) -> &SimulationSnapshot {
        &self.latest
    }

    pub fn step(
        &mut self,
        time_ns: u64,
        dt_seconds: f64,
        intent: RelativeIntent,
        feedback: Option<&ServoFeedbackFrame>,
        feedback_age_ms: Option<u64>,
    ) -> (ServoCommandFrame, SimulationSnapshot) {
        let feedback_error = match feedback.map(|value| self.apply_feedback(value)).transpose() {
            Ok(_) => None,
            Err(()) => Some(SimulationStopReason::ServoInvalidFeedback),
        };
        let freshness_error = if feedback.is_none() {
            Some(SimulationStopReason::ServoUnavailable)
        } else if feedback_age_ms.is_none_or(|age| age > 100) {
            Some(SimulationStopReason::ServoFeedbackStale)
        } else {
            None
        };

        let mut state = SimulationState::Idle;
        let mut enabled = false;
        let mut desired_tcp_pose = None;
        let mut stop_reason = feedback_error.or(freshness_error);
        let mut target_pose = None;

        if stop_reason.is_some() {
            state = SimulationState::Faulted;
            self.anchor_tcp = None;
            self.stop_gripper();
        } else {
            match intent.state {
                RelativeIntentState::Faulted => {
                    state = SimulationState::Faulted;
                    stop_reason = Some(SimulationStopReason::IntentFaulted);
                    self.anchor_tcp = None;
                    self.stop_gripper();
                }
                RelativeIntentState::Idle => {
                    stop_reason = Some(SimulationStopReason::IntentIdle);
                    self.anchor_tcp = None;
                    self.stop_gripper();
                }
                RelativeIntentState::Active => match self.target_from_intent(intent) {
                    Some(target) if self.model.workspace_contains(target.position_m) => {
                        enabled = true;
                        desired_tcp_pose = Some(target);
                        target_pose = Some(target);
                        state = servo_state(feedback.and_then(|value| value.servo_status_code));
                        stop_reason =
                            servo_stop_reason(feedback.and_then(|value| value.servo_status_code));
                        if state == SimulationState::Faulted {
                            enabled = false;
                            target_pose = None;
                            self.anchor_tcp = None;
                            self.stop_gripper();
                        } else if let Some(closed) = intent.gripper_closed {
                            self.limit_gripper(closed, dt_seconds);
                        }
                    }
                    Some(target) => {
                        state = SimulationState::Constrained;
                        enabled = true;
                        desired_tcp_pose = Some(target);
                        stop_reason = Some(SimulationStopReason::WorkspaceViolation);
                        self.stop_gripper();
                    }
                    None => {
                        state = SimulationState::Faulted;
                        stop_reason = Some(SimulationStopReason::InvalidIntent);
                        self.anchor_tcp = None;
                        self.stop_gripper();
                    }
                },
            }
        }

        let status_code = feedback.and_then(|value| value.servo_status_code);
        let status_message = feedback.and_then(|value| value.servo_status_message.clone());
        let model_joints = self.model.model_joints(self.logical_joints);
        self.latest = SimulationSnapshot {
            model_id: self.model.profile().model_id.clone(),
            simulation_only: true,
            backend: "moveit_servo".to_owned(),
            state,
            enabled,
            time_ns,
            intent_receive_time_ns: intent.receive_time_ns,
            intent_sample_sequence: intent.sample_sequence,
            joints_rad: self.logical_joints,
            joints_deg: self.logical_joints.map(f64::to_degrees),
            model_joints_rad: model_joints,
            model_joints_deg: model_joints.map(f64::to_degrees),
            joint_velocity_rad_s: self.logical_joint_velocity,
            gripper_closed: self.gripper_closed,
            gripper_rad: self.gripper_rad,
            gripper_deg: self.gripper_rad.to_degrees(),
            gripper_velocity_rad_s: self.gripper_velocity,
            tcp_pose: self.model.forward(self.logical_joints),
            desired_tcp_pose,
            servo_status_code: status_code,
            servo_status_message: status_message,
            servo_feedback_age_ms: feedback_age_ms,
            stop_reason,
        };

        let command = ServoCommandFrame {
            schema_version: SERVO_IPC_SCHEMA_VERSION,
            time_ns,
            intent_receive_time_ns: intent.receive_time_ns,
            intent_sample_sequence: intent.sample_sequence,
            enabled: enabled && target_pose.is_some(),
            base_frame: self.model.profile().kinematics.base_link.clone(),
            tcp_link: self.model.profile().kinematics.tcp_link.clone(),
            target_pose,
            gripper_closed: enabled.then_some(self.gripper_closed),
        };
        (command, self.latest.clone())
    }

    fn apply_feedback(&mut self, feedback: &ServoFeedbackFrame) -> Result<(), ()> {
        if feedback.schema_version != SERVO_IPC_SCHEMA_VERSION
            || feedback.model_joint_names.len() != ARM_DOF
            || feedback.model_joints_rad.len() != ARM_DOF
            || feedback.model_joint_velocity_rad_s.len() != ARM_DOF
        {
            return Err(());
        }
        let mut model_joints = [0.0; ARM_DOF];
        let mut model_velocity = [0.0; ARM_DOF];
        for (target_index, joint) in self.model.profile().joints[..ARM_DOF].iter().enumerate() {
            let source_index = feedback
                .model_joint_names
                .iter()
                .position(|name| name == &joint.name)
                .ok_or(())?;
            model_joints[target_index] = feedback.model_joints_rad[source_index];
            model_velocity[target_index] = feedback.model_joint_velocity_rad_s[source_index];
        }
        if model_joints
            .into_iter()
            .chain(model_velocity)
            .any(|value| !value.is_finite())
        {
            return Err(());
        }
        let logical = self.model.logical_joints(model_joints);
        let (lower, upper) = self.model.joint_limits();
        if logical
            .iter()
            .enumerate()
            .any(|(index, value)| *value < lower[index] - 1.0e-6 || *value > upper[index] + 1.0e-6)
        {
            return Err(());
        }
        self.logical_joints = logical;
        self.logical_joint_velocity = std::array::from_fn(|index| {
            model_velocity[index] * self.model.profile().joints[index].direction
        });
        Ok(())
    }

    fn target_from_intent(&mut self, intent: RelativeIntent) -> Option<Pose> {
        let relative_position = intent.relative_position_m?;
        let relative_orientation = intent.relative_orientation_xyzw?;
        if relative_position
            .into_iter()
            .any(|value| !value.is_finite())
            || relative_orientation
                .into_iter()
                .any(|value| !value.is_finite())
            || intent.gripper_closed.is_none()
        {
            return None;
        }
        let raw_quaternion = Quaternion::new(
            relative_orientation[3],
            relative_orientation[0],
            relative_orientation[1],
            relative_orientation[2],
        );
        if raw_quaternion.norm_squared() <= f64::EPSILON {
            return None;
        }
        let relative_rotation = UnitQuaternion::new_normalize(raw_quaternion);
        let anchor = *self.anchor_tcp.get_or_insert_with(|| {
            self.model
                .forward(self.logical_joints)
                .to_isometry()
                .expect("validated model FK must be finite")
        });
        let position_mapping = self.model.robot_from_nolo_position();
        let orientation_basis =
            UnitQuaternion::from_matrix(&self.model.robot_from_controller_orientation());
        let translation = position_mapping
            * Vector3::from(relative_position)
            * self.model.profile().teleoperation.translation_scale;
        let rotation = orientation_basis * relative_rotation * orientation_basis.inverse();
        Some(Pose::from_isometry(&nalgebra::Isometry3::from_parts(
            Translation3::from(anchor.translation.vector + translation),
            anchor.rotation * rotation,
        )))
    }

    fn limit_gripper(&mut self, closed: bool, dt: f64) {
        if !dt.is_finite() || !(0.0..=0.1).contains(&dt) || dt == 0.0 {
            self.stop_gripper();
            return;
        }
        let settings = &self.model.profile().teleoperation;
        let target = if closed {
            self.gripper_closed_rad
        } else {
            self.gripper_open_rad
        };
        let error = target - self.gripper_rad;
        let desired_velocity = (error / dt).clamp(
            -settings.max_joint_speed_rad_s,
            settings.max_joint_speed_rad_s,
        );
        let velocity_delta = (desired_velocity - self.gripper_velocity).clamp(
            -settings.max_joint_acceleration_rad_s2 * dt,
            settings.max_joint_acceleration_rad_s2 * dt,
        );
        self.gripper_velocity += velocity_delta;
        let mut step = self.gripper_velocity * dt;
        if step.abs() > error.abs() {
            step = error;
            self.gripper_velocity = 0.0;
        }
        self.gripper_rad += step;
        self.gripper_closed = closed;
    }

    fn stop_gripper(&mut self) {
        self.gripper_velocity = 0.0;
    }
}

fn servo_state(code: Option<i8>) -> SimulationState {
    match code {
        Some(-1) => SimulationState::Faulted,
        Some(1..=6) => SimulationState::Constrained,
        _ => SimulationState::Active,
    }
}

fn servo_stop_reason(code: Option<i8>) -> Option<SimulationStopReason> {
    match code {
        Some(-1) => Some(SimulationStopReason::ServoHalt),
        Some(1..=3) => Some(SimulationStopReason::Singularity),
        Some(4 | 5) => Some(SimulationStopReason::ServoCollision),
        Some(6) => Some(SimulationStopReason::JointLimit),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feedback(model: &ArmModel) -> ServoFeedbackFrame {
        ServoFeedbackFrame {
            schema_version: SERVO_IPC_SCHEMA_VERSION,
            sequence: 1,
            model_joint_names: model.profile().joints[..ARM_DOF]
                .iter()
                .map(|joint| joint.name.clone())
                .collect(),
            model_joints_rad: vec![0.0; ARM_DOF],
            model_joint_velocity_rad_s: vec![0.0; ARM_DOF],
            servo_status_code: Some(0),
            servo_status_message: Some("NO_WARNING".to_owned()),
        }
    }

    fn intent(state: RelativeIntentState, position: Option<[f64; 3]>) -> RelativeIntent {
        RelativeIntent {
            state,
            receive_time_ns: 10,
            sample_sequence: Some(1),
            relative_position_m: position,
            relative_orientation_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
            gripper_closed: Some(false),
        }
    }

    #[test]
    fn no_feedback_never_emits_a_servo_target() {
        let mut controller = MoveItSimulationController::new(ArmModel::embedded().unwrap());
        let (command, snapshot) = controller.step(
            1,
            0.01,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            None,
            None,
        );
        assert!(!command.enabled);
        assert_eq!(command.target_pose, None);
        assert_eq!(snapshot.state, SimulationState::Faulted);
        assert_eq!(
            snapshot.stop_reason,
            Some(SimulationStopReason::ServoUnavailable)
        );
    }

    #[test]
    fn active_target_uses_the_separate_position_mapping() {
        let model = ArmModel::embedded().unwrap();
        let feedback = feedback(&model);
        let mut controller = MoveItSimulationController::new(model);
        controller.step(
            1,
            0.01,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&feedback),
            Some(0),
        );
        let anchor = controller.latest().tcp_pose;
        let (command, snapshot) = controller.step(
            2,
            0.01,
            intent(RelativeIntentState::Active, Some([0.0, 0.0, -0.02])),
            Some(&feedback),
            Some(0),
        );
        let target = command.target_pose.unwrap();
        assert!((target.position_m[0] - anchor.position_m[0] - 0.02).abs() < 1.0e-9);
        assert_eq!(snapshot.backend, "moveit_servo");
    }

    #[test]
    fn feedback_is_reordered_and_converted_to_logical_angles() {
        let model = ArmModel::embedded().unwrap();
        let mut value = feedback(&model);
        value.model_joint_names.reverse();
        value.model_joints_rad = vec![0.1, 0.0, 0.0, 0.0, 0.2, 0.3];
        let mut controller = MoveItSimulationController::new(model);
        let (_, snapshot) = controller.step(
            1,
            0.01,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&value),
            Some(0),
        );
        assert!((snapshot.model_joints_rad[0] - 0.3).abs() < 1.0e-9);
        assert!((snapshot.joints_rad[0] + 0.3).abs() < 1.0e-9);
    }

    #[test]
    fn stale_or_invalid_feedback_disables_output() {
        let model = ArmModel::embedded().unwrap();
        let mut value = feedback(&model);
        let mut controller = MoveItSimulationController::new(model);
        let (stale, snapshot) = controller.step(
            1,
            0.01,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            Some(&value),
            Some(101),
        );
        assert!(!stale.enabled);
        assert_eq!(
            snapshot.stop_reason,
            Some(SimulationStopReason::ServoFeedbackStale)
        );

        value.model_joint_names.pop();
        let (invalid, snapshot) = controller.step(
            2,
            0.01,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            Some(&value),
            Some(0),
        );
        assert!(!invalid.enabled);
        assert_eq!(
            snapshot.stop_reason,
            Some(SimulationStopReason::ServoInvalidFeedback)
        );
    }

    #[test]
    fn servo_halt_is_visible_but_recovery_commands_keep_flowing() {
        let model = ArmModel::embedded().unwrap();
        let mut value = feedback(&model);
        value.servo_status_code = Some(2);
        let mut controller = MoveItSimulationController::new(model);
        let (command, snapshot) = controller.step(
            1,
            0.01,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            Some(&value),
            Some(0),
        );
        assert!(command.enabled);
        assert_eq!(snapshot.state, SimulationState::Constrained);
        assert_eq!(
            snapshot.stop_reason,
            Some(SimulationStopReason::Singularity)
        );
    }

    #[test]
    fn all_moveit_warning_codes_are_exposed() {
        assert_eq!(servo_state(Some(-1)), SimulationState::Faulted);
        assert_eq!(
            servo_stop_reason(Some(-1)),
            Some(SimulationStopReason::ServoHalt)
        );
        for code in [1, 2, 3] {
            assert_eq!(servo_state(Some(code)), SimulationState::Constrained);
            assert_eq!(
                servo_stop_reason(Some(code)),
                Some(SimulationStopReason::Singularity)
            );
        }
        for code in [4, 5] {
            assert_eq!(servo_state(Some(code)), SimulationState::Constrained);
            assert_eq!(
                servo_stop_reason(Some(code)),
                Some(SimulationStopReason::ServoCollision)
            );
        }
        assert_eq!(servo_state(Some(6)), SimulationState::Constrained);
        assert_eq!(
            servo_stop_reason(Some(6)),
            Some(SimulationStopReason::JointLimit)
        );
    }

    #[test]
    fn invalid_moveit_status_suppresses_the_target() {
        let model = ArmModel::embedded().unwrap();
        let mut value = feedback(&model);
        value.servo_status_code = Some(-1);
        let mut controller = MoveItSimulationController::new(model);
        let (command, snapshot) = controller.step(
            1,
            0.01,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            Some(&value),
            Some(0),
        );
        assert!(!command.enabled);
        assert_eq!(command.target_pose, None);
        assert_eq!(snapshot.state, SimulationState::Faulted);
        assert_eq!(snapshot.stop_reason, Some(SimulationStopReason::ServoHalt));
    }
}
