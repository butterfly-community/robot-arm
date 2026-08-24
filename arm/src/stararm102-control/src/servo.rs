//! MoveIt Servo IPC protocol and the simulation output sink.
//!
//! This module performs coordinate conversion and validates Servo feedback. It
//! deliberately contains no forward or inverse kinematics: current TCP comes
//! from ROS TF2 and joint targets come from MoveIt.

use crate::{
    model::{ArmModel, Pose},
    simulation::{
        RelativeIntent, RelativeIntentState, SimulationSnapshot, SimulationState,
        SimulationStopReason,
    },
};
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

pub const SERVO_IPC_SCHEMA_VERSION: u32 = 2;
pub const ARM_DOF: usize = 6;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HomeRequestAction {
    Plan,
    Execute,
    Cancel,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HomeRequestFrame {
    pub request_id: u64,
    pub action: HomeRequestAction,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HomeState {
    #[default]
    Idle,
    Planning,
    Ready,
    Executing,
    Succeeded,
    Failed,
    Cancelled,
}

impl HomeState {
    pub fn blocks_teleop(self) -> bool {
        matches!(self, Self::Planning | Self::Ready | Self::Executing)
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct HomeStatusFrame {
    pub request_id: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub acknowledged_action: Option<HomeRequestAction>,
    pub state: HomeState,
    pub message: String,
    pub trajectory_points: usize,
    pub duration_seconds: Option<f64>,
    #[serde(default)]
    pub trajectory_model_joints_rad: Vec<[f64; ARM_DOF]>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ServoCommandFrame {
    pub schema_version: u32,
    pub enabled: bool,
    pub base_frame: String,
    pub tcp_link: String,
    pub target_pose: Option<Pose>,
    pub gripper_closed: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub home_request: Option<HomeRequestFrame>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ServoFeedbackFrame {
    pub schema_version: u32,
    pub sequence: u64,
    pub model_joint_names: Vec<String>,
    pub model_joints_rad: Vec<f64>,
    pub tcp_pose: Pose,
    pub servo_status_code: Option<i8>,
    pub servo_status_message: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub home_status: Option<HomeStatusFrame>,
}

/// Backend-neutral hand-to-MoveIt command mapper and feedback state.
pub struct MoveItController {
    model: ArmModel,
    simulation_only: bool,
    backend: String,
    logical_joints: [f64; ARM_DOF],
    current_tcp: Option<nalgebra::Isometry3<f64>>,
    anchor_tcp: Option<nalgebra::Isometry3<f64>>,
    rearm_required: bool,
    gripper_closed: bool,
    gripper_rad: f64,
    gripper_open_rad: f64,
    gripper_closed_rad: f64,
    latest: SimulationSnapshot,
}

impl MoveItController {
    pub fn new(model: ArmModel) -> Self {
        Self::new_for_backend(model, true, "moveit_servo_simulation")
    }

    pub fn new_for_backend(
        model: ArmModel,
        simulation_only: bool,
        backend: impl Into<String>,
    ) -> Self {
        let backend = backend.into();
        let logical_joints = model.nominal_joints();
        // The model opening angle is independent from the FL servo's logical
        // multi-turn range and transmission direction.
        let gripper_open_rad = model
            .profile()
            .moveit_interface
            .gripper_open_deg
            .to_radians();
        let gripper_closed_rad = model
            .profile()
            .moveit_interface
            .gripper_closed_deg
            .to_radians();
        let gripper_rad = gripper_closed_rad;
        let model_joints = model.model_joints(logical_joints);
        let latest = SimulationSnapshot {
            model_id: model.profile().model_id.clone(),
            simulation_only,
            backend: backend.clone(),
            state: SimulationState::Faulted,
            enabled: false,
            time_ns: 0,
            joints_rad: logical_joints,
            model_joints_rad: model_joints,
            gripper_closed: true,
            gripper_rad,
            tcp_pose: None,
            desired_tcp_pose: None,
            servo_status_code: None,
            servo_status_message: None,
            servo_feedback_age_ms: None,
            stop_reason: Some(SimulationStopReason::ServoUnavailable),
        };
        Self {
            model,
            simulation_only,
            backend,
            logical_joints,
            current_tcp: None,
            anchor_tcp: None,
            rearm_required: true,
            gripper_closed: true,
            gripper_rad,
            gripper_open_rad,
            gripper_closed_rad,
            latest,
        }
    }

    pub fn latest(&self) -> &SimulationSnapshot {
        &self.latest
    }

    /// Invalidate the current hand-to-TCP anchor.  A backend switch must not
    /// reuse an active command from the previously selected output.
    pub fn require_rearm(&mut self) {
        self.rearm_required = true;
        self.anchor_tcp = None;
    }

    pub fn step(
        &mut self,
        time_ns: u64,
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
        let status_error = match feedback.and_then(|value| value.servo_status_code) {
            None => Some(SimulationStopReason::ServoUnavailable),
            Some(-1) => Some(SimulationStopReason::ServoHalt),
            Some(0..=6) => None,
            Some(_) => Some(SimulationStopReason::ServoInvalidFeedback),
        };

        let mut state = SimulationState::Idle;
        let mut enabled = false;
        let mut desired_tcp_pose = None;
        let mut stop_reason = feedback_error.or(freshness_error).or(status_error);
        let mut target_pose = None;

        if stop_reason.is_some() {
            state = SimulationState::Faulted;
            self.rearm_required = true;
            self.anchor_tcp = None;
        } else {
            match intent.state {
                RelativeIntentState::Faulted => {
                    state = SimulationState::Faulted;
                    stop_reason = Some(SimulationStopReason::IntentFaulted);
                    self.rearm_required = true;
                    self.anchor_tcp = None;
                }
                RelativeIntentState::Idle => {
                    stop_reason = Some(SimulationStopReason::IntentIdle);
                    self.rearm_required = false;
                    self.anchor_tcp = None;
                }
                RelativeIntentState::Active if self.rearm_required => {
                    state = SimulationState::Faulted;
                    stop_reason = Some(SimulationStopReason::AwaitingIntentRelease);
                    self.anchor_tcp = None;
                }
                RelativeIntentState::Active => match self.target_from_intent(intent) {
                    Some(target) => {
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
                        } else if let Some(closed) = intent.gripper_closed {
                            self.gripper_closed = closed;
                            self.gripper_rad = if closed {
                                self.gripper_closed_rad
                            } else {
                                self.gripper_open_rad
                            };
                        }
                    }
                    None => {
                        state = SimulationState::Faulted;
                        stop_reason = Some(SimulationStopReason::InvalidIntent);
                        self.rearm_required = true;
                        self.anchor_tcp = None;
                    }
                },
            }
        }

        let status_code = feedback.and_then(|value| value.servo_status_code);
        let status_message = feedback.and_then(|value| value.servo_status_message.clone());
        let model_joints = self.model.model_joints(self.logical_joints);
        self.latest = SimulationSnapshot {
            model_id: self.model.profile().model_id.clone(),
            simulation_only: self.simulation_only,
            backend: self.backend.clone(),
            state,
            enabled,
            time_ns,
            joints_rad: self.logical_joints,
            model_joints_rad: model_joints,
            gripper_closed: self.gripper_closed,
            gripper_rad: self.gripper_rad,
            tcp_pose: self.current_tcp.as_ref().map(Pose::from_isometry),
            desired_tcp_pose,
            servo_status_code: status_code,
            servo_status_message: status_message,
            servo_feedback_age_ms: feedback_age_ms,
            stop_reason,
        };

        let command_enabled = enabled && target_pose.is_some();
        let command = ServoCommandFrame {
            schema_version: SERVO_IPC_SCHEMA_VERSION,
            enabled: command_enabled,
            base_frame: self.model.profile().moveit_interface.base_frame.clone(),
            tcp_link: self.model.profile().moveit_interface.tcp_link.clone(),
            target_pose,
            gripper_closed: (command_enabled && intent.gripper_closed.is_some())
                .then_some(self.gripper_closed),
            home_request: None,
        };
        (command, self.latest.clone())
    }

    fn apply_feedback(&mut self, feedback: &ServoFeedbackFrame) -> Result<(), ()> {
        if feedback.schema_version != SERVO_IPC_SCHEMA_VERSION
            || feedback.model_joint_names.len() != ARM_DOF
            || feedback.model_joints_rad.len() != ARM_DOF
        {
            return Err(());
        }
        let mut model_joints = [0.0; ARM_DOF];
        for (target_index, joint) in self.model.profile().joints[..ARM_DOF].iter().enumerate() {
            let source_index = feedback
                .model_joint_names
                .iter()
                .position(|name| name == &joint.name)
                .ok_or(())?;
            model_joints[target_index] = feedback.model_joints_rad[source_index];
        }
        if model_joints.into_iter().any(|value| !value.is_finite()) {
            return Err(());
        }
        let logical = self.model.logical_joints(model_joints);
        let tcp = feedback.tcp_pose.to_isometry().ok_or(())?;
        self.logical_joints = logical;
        self.current_tcp = Some(tcp);
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
        let norm_squared = raw_quaternion.norm_squared();
        if !norm_squared.is_finite() || norm_squared <= f64::EPSILON {
            return None;
        }
        let relative_rotation = UnitQuaternion::new_normalize(raw_quaternion);
        let anchor = *self.anchor_tcp.get_or_insert(self.current_tcp?);
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
            tcp_pose: Pose {
                position_m: [0.2, 0.0, 0.2],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            servo_status_code: Some(0),
            servo_status_message: Some("NO_WARNING".to_owned()),
            home_status: None,
        }
    }

    fn intent(state: RelativeIntentState, position: Option<[f64; 3]>) -> RelativeIntent {
        RelativeIntent {
            state,
            relative_position_m: position,
            relative_orientation_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
            gripper_closed: Some(false),
        }
    }

    #[test]
    fn no_feedback_never_emits_a_servo_target() {
        let mut controller = MoveItController::new(ArmModel::embedded().unwrap());
        let (command, snapshot) = controller.step(
            1,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            None,
            None,
        );
        assert!(!command.enabled);
        assert_eq!(command.target_pose, None);
        assert_eq!(snapshot.tcp_pose, None);
        assert_eq!(snapshot.state, SimulationState::Faulted);
        assert_eq!(
            snapshot.stop_reason,
            Some(SimulationStopReason::ServoUnavailable)
        );
    }

    #[test]
    fn gripper_model_endpoints_are_independent_from_servo_transmission_units() {
        let controller = MoveItController::new(ArmModel::embedded().unwrap());
        assert!((controller.gripper_open_rad.to_degrees() - 90.0).abs() < 1.0e-12);
        assert!(controller.gripper_closed_rad.abs() < 1.0e-12);
        assert_eq!(controller.latest().gripper_rad, 0.0);
    }

    #[test]
    fn active_target_maps_all_hand_axes_at_one_to_five_scale() {
        let model = ArmModel::embedded().unwrap();
        let feedback = feedback(&model);
        let mut controller = MoveItController::new(model);
        controller.step(
            1,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&feedback),
            Some(0),
        );
        let anchor = controller.latest().tcp_pose.unwrap();
        let (forward, snapshot) = controller.step(
            2,
            intent(RelativeIntentState::Active, Some([0.0, 0.0, -0.05])),
            Some(&feedback),
            Some(0),
        );
        let target = forward.target_pose.unwrap();
        assert!((target.position_m[0] - anchor.position_m[0] - 0.01).abs() < 1.0e-9);
        assert_eq!(snapshot.backend, "moveit_servo_simulation");

        let (right, _) = controller.step(
            3,
            intent(RelativeIntentState::Active, Some([0.05, 0.0, 0.0])),
            Some(&feedback),
            Some(0),
        );
        let target = right.target_pose.unwrap();
        assert!((target.position_m[1] - anchor.position_m[1] + 0.01).abs() < 1.0e-9);

        let (up, _) = controller.step(
            4,
            intent(RelativeIntentState::Active, Some([0.0, 0.05, 0.0])),
            Some(&feedback),
            Some(0),
        );
        let target = up.target_pose.unwrap();
        assert!((target.position_m[2] - anchor.position_m[2] - 0.01).abs() < 1.0e-9);
    }

    #[test]
    fn relative_hand_orientation_maps_one_to_one_in_tcp_local_frame() {
        let model = ArmModel::embedded().unwrap();
        let feedback = feedback(&model);
        let mut controller = MoveItController::new(model);
        controller.step(
            1,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&feedback),
            Some(0),
        );
        let anchor = controller.latest().tcp_pose.unwrap().to_isometry().unwrap();
        let angle = 0.2;
        for (time_ns, raw_axis, expected_axis) in [
            (2, -Vector3::x(), -Vector3::y()),
            (3, Vector3::y(), -Vector3::x()),
            (4, Vector3::z(), Vector3::z()),
        ] {
            let relative = UnitQuaternion::from_scaled_axis(raw_axis * angle);
            let quaternion = relative.quaternion();
            let mut value = intent(RelativeIntentState::Active, Some([0.0; 3]));
            value.relative_orientation_xyzw =
                Some([quaternion.i, quaternion.j, quaternion.k, quaternion.w]);
            let (command, _) = controller.step(time_ns, value, Some(&feedback), Some(0));
            let target = command.target_pose.unwrap().to_isometry().unwrap();
            let actual = (anchor.rotation.inverse() * target.rotation).scaled_axis();
            assert!((actual - expected_axis * angle).norm() < 1.0e-9);
        }
    }

    #[test]
    fn feedback_is_reordered_and_converted_to_logical_angles() {
        let model = ArmModel::embedded().unwrap();
        let mut value = feedback(&model);
        value.model_joint_names.reverse();
        value.model_joints_rad = vec![0.1, 0.0, 0.0, 0.0, 0.2, 0.3];
        let mut controller = MoveItController::new(model);
        let (_, snapshot) = controller.step(
            1,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&value),
            Some(0),
        );
        assert!((snapshot.model_joints_rad[0] - 0.3).abs() < 1.0e-9);
        assert!((snapshot.joints_rad[0] + 0.3).abs() < 1.0e-9);
        assert_eq!(snapshot.tcp_pose, Some(value.tcp_pose));
    }

    #[test]
    fn feedback_requires_schema_v2_and_a_valid_ros_tcp_pose() {
        for invalid in [
            {
                let model = ArmModel::embedded().unwrap();
                let mut value = feedback(&model);
                value.schema_version = 1;
                value
            },
            {
                let model = ArmModel::embedded().unwrap();
                let mut value = feedback(&model);
                value.tcp_pose.orientation_xyzw = [0.0; 4];
                value
            },
        ] {
            let mut controller = MoveItController::new(ArmModel::embedded().unwrap());
            let (command, snapshot) = controller.step(
                1,
                intent(RelativeIntentState::Active, Some([0.0; 3])),
                Some(&invalid),
                Some(0),
            );
            assert!(!command.enabled);
            assert_eq!(command.target_pose, None);
            assert_eq!(
                snapshot.stop_reason,
                Some(SimulationStopReason::ServoInvalidFeedback)
            );
        }
    }

    #[test]
    fn stale_or_invalid_feedback_disables_output() {
        let model = ArmModel::embedded().unwrap();
        let mut value = feedback(&model);
        let mut controller = MoveItController::new(model);
        let (stale, snapshot) = controller.step(
            1,
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
    fn servo_constraint_is_visible_but_recovery_commands_keep_flowing() {
        let model = ArmModel::embedded().unwrap();
        let mut value = feedback(&model);
        value.servo_status_code = Some(2);
        let mut controller = MoveItController::new(model);
        controller.step(
            0,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&value),
            Some(0),
        );
        let (command, snapshot) = controller.step(
            1,
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
        let mut controller = MoveItController::new(model);
        controller.step(
            0,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&value),
            Some(0),
        );
        value.servo_status_code = Some(-1);
        let (command, snapshot) = controller.step(
            1,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            Some(&value),
            Some(0),
        );
        assert!(!command.enabled);
        assert_eq!(command.target_pose, None);
        assert_eq!(snapshot.state, SimulationState::Faulted);
        assert_eq!(snapshot.stop_reason, Some(SimulationStopReason::ServoHalt));
    }

    #[test]
    fn missing_or_unknown_moveit_status_suppresses_the_target() {
        for code in [None, Some(7), Some(-2)] {
            let model = ArmModel::embedded().unwrap();
            let mut value = feedback(&model);
            value.servo_status_code = code;
            let mut controller = MoveItController::new(model);
            let (command, snapshot) = controller.step(
                1,
                intent(RelativeIntentState::Active, Some([0.0; 3])),
                Some(&value),
                Some(0),
            );
            assert!(!command.enabled);
            assert_eq!(command.target_pose, None);
            assert_eq!(snapshot.state, SimulationState::Faulted);
            let expected = if code.is_none() {
                SimulationStopReason::ServoUnavailable
            } else {
                SimulationStopReason::ServoInvalidFeedback
            };
            assert_eq!(snapshot.stop_reason, Some(expected));
        }
    }

    #[test]
    fn feedback_fault_requires_intent_release_before_rearming() {
        let model = ArmModel::embedded().unwrap();
        let feedback = feedback(&model);
        let mut controller = MoveItController::new(model);
        controller.step(
            1,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&feedback),
            Some(0),
        );
        assert!(
            controller
                .step(
                    2,
                    intent(RelativeIntentState::Active, Some([0.0; 3])),
                    Some(&feedback),
                    Some(0),
                )
                .0
                .enabled
        );

        controller.step(
            3,
            intent(RelativeIntentState::Active, Some([0.01; 3])),
            Some(&feedback),
            Some(101),
        );
        let (held, snapshot) = controller.step(
            4,
            intent(RelativeIntentState::Active, Some([0.01; 3])),
            Some(&feedback),
            Some(0),
        );
        assert!(!held.enabled);
        assert_eq!(held.target_pose, None);
        assert_eq!(
            snapshot.stop_reason,
            Some(SimulationStopReason::AwaitingIntentRelease)
        );

        controller.step(
            5,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&feedback),
            Some(0),
        );
        let (rearmed, _) = controller.step(
            6,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            Some(&feedback),
            Some(0),
        );
        assert!(rearmed.enabled);
    }
}
