//! MoveIt Servo IPC protocol and unified arm state mapper.
//!
//! This module performs coordinate conversion and validates Servo feedback. It
//! deliberately contains no forward or inverse kinematics: current TCP comes
//! from ROS TF2 and joint targets come from MoveIt.

use crate::{
    model::{ArmModel, Pose},
    state::{ArmControlState, ArmSnapshot, ArmStopReason, RelativeIntent, RelativeIntentState},
};
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

pub const SERVO_IPC_SCHEMA_VERSION: u32 = 5;
pub const ARM_DOF: usize = 6;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotionRequestAction {
    Plan,
    Cancel,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionRequestFrame {
    pub request_id: u64,
    pub action: MotionRequestAction,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub joints_rad: Option<[f64; ARM_DOF]>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GripperRequestFrame {
    pub request_id: u64,
    pub position_rad: f64,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MotionState {
    #[default]
    Idle,
    Planning,
    Executing,
    Succeeded,
    Failed,
    Cancelled,
}

impl MotionState {
    pub fn blocks_teleop(self) -> bool {
        matches!(self, Self::Planning | Self::Executing)
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MotionStatusFrame {
    pub request_id: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub acknowledged_action: Option<MotionRequestAction>,
    pub state: MotionState,
    pub message: String,
    pub trajectory_points: usize,
    pub duration_seconds: Option<f64>,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ArmFeedbackSource {
    #[default]
    Software,
    Serial,
}

pub const START_POSITION_JOINTS_RAD: [f64; ARM_DOF] =
    [0.0, 0.0, -3.0 * std::f64::consts::PI / 180.0, 0.0, 0.0, 0.0];
pub const GRIPPER_START_POSITION_RAD: f64 = std::f64::consts::PI / 180.0;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArmStateFrame {
    pub joints_rad: [f64; ARM_DOF],
    pub gripper_position_rad: f64,
    pub feedback_source: ArmFeedbackSource,
}

impl Default for ArmStateFrame {
    fn default() -> Self {
        Self {
            joints_rad: START_POSITION_JOINTS_RAD,
            gripper_position_rad: GRIPPER_START_POSITION_RAD,
            feedback_source: ArmFeedbackSource::Software,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ServoCommandFrame {
    pub schema_version: u32,
    pub enabled: bool,
    pub base_frame: String,
    pub tcp_link: String,
    pub target_pose: Option<Pose>,
    pub arm_state: ArmStateFrame,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gripper_request: Option<GripperRequestFrame>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion_request: Option<MotionRequestFrame>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ServoFeedbackFrame {
    pub schema_version: u32,
    pub controller_joints_rad: [f64; ARM_DOF],
    pub controller_gripper_position_rad: f64,
    pub tcp_pose: Pose,
    pub servo_status_code: Option<i8>,
    pub servo_status_message: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion_status: Option<MotionStatusFrame>,
}

/// Backend-neutral hand-to-MoveIt command mapper and feedback state.
pub struct MoveItController {
    model: ArmModel,
    arm_state: ArmStateFrame,
    current_tcp: Option<nalgebra::Isometry3<f64>>,
    anchor_tcp: Option<nalgebra::Isometry3<f64>>,
    reanchor_required: bool,
    gripper_open_rad: f64,
    gripper_closed_rad: f64,
    previous_gripper_pressed: Option<bool>,
    joint_limit_frozen: bool,
    joint_limit_recovery: bool,
    latest: ArmSnapshot,
}

impl MoveItController {
    pub fn new(model: ArmModel) -> Self {
        let arm_state = ArmStateFrame::default();
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
        let latest = ArmSnapshot {
            model_id: model.profile().model_id.clone(),
            feedback_source: ArmFeedbackSource::Software,
            state: ArmControlState::Faulted,
            enabled: false,
            time_ns: 0,
            joints_rad: arm_state.joints_rad,
            gripper_rad: arm_state.gripper_position_rad,
            tcp_pose: None,
            desired_tcp_pose: None,
            servo_status_code: None,
            servo_status_message: None,
            servo_feedback_age_ms: None,
            stop_reason: Some(ArmStopReason::ServoUnavailable),
        };
        Self {
            model,
            arm_state,
            current_tcp: None,
            anchor_tcp: None,
            reanchor_required: true,
            gripper_open_rad,
            gripper_closed_rad,
            previous_gripper_pressed: None,
            joint_limit_frozen: false,
            joint_limit_recovery: false,
            latest,
        }
    }

    pub fn latest(&self) -> &ArmSnapshot {
        &self.latest
    }

    /// Re-anchor the next active command at the current TCP without requiring
    /// the operator to release and press Trigger again.
    pub fn require_reanchor(&mut self) {
        self.reanchor_required = true;
        self.anchor_tcp = None;
    }

    pub fn apply_arm_state(&mut self, state: &ArmStateFrame) -> bool {
        if state.joints_rad.into_iter().any(|value| !value.is_finite())
            || !state.gripper_position_rad.is_finite()
        {
            return false;
        }
        self.arm_state = state.clone();
        true
    }

    pub fn step(
        &mut self,
        time_ns: u64,
        intent: RelativeIntent,
        feedback: Option<&ServoFeedbackFrame>,
        feedback_age_ms: Option<u64>,
    ) -> (ServoCommandFrame, ArmSnapshot) {
        let feedback_error = match feedback.map(|value| self.apply_feedback(value)).transpose() {
            Ok(_) => None,
            Err(()) => Some(ArmStopReason::ServoInvalidFeedback),
        };
        let availability_error = feedback
            .is_none()
            .then_some(ArmStopReason::ServoUnavailable);
        let status_error = match feedback.and_then(|value| value.servo_status_code) {
            None => Some(ArmStopReason::ServoUnavailable),
            Some(-1..=6) => None,
            Some(_) => Some(ArmStopReason::ServoInvalidFeedback),
        };

        let mut state = ArmControlState::Idle;
        let mut enabled = false;
        let mut desired_tcp_pose = None;
        let mut stop_reason = feedback_error.or(availability_error).or(status_error);
        let mut target_pose = None;
        let mut gripper_target = None;

        if stop_reason.is_some() {
            state = ArmControlState::Faulted;
            self.reanchor_required = true;
            self.anchor_tcp = None;
            self.previous_gripper_pressed = None;
        } else {
            match intent.state {
                RelativeIntentState::Faulted => {
                    state = ArmControlState::Faulted;
                    stop_reason = Some(ArmStopReason::IntentFaulted);
                    self.reanchor_required = true;
                    self.anchor_tcp = None;
                    self.previous_gripper_pressed = None;
                }
                RelativeIntentState::Idle => {
                    if self.joint_limit_frozen {
                        self.joint_limit_frozen = false;
                        self.joint_limit_recovery = true;
                    }
                    stop_reason = Some(ArmStopReason::IntentIdle);
                    self.reanchor_required = false;
                    self.anchor_tcp = None;
                    self.previous_gripper_pressed = None;
                }
                RelativeIntentState::Active
                    if self.joint_limit_frozen
                        || (!self.joint_limit_recovery
                            && feedback.and_then(|value| value.servo_status_code) == Some(6)) =>
                {
                    self.joint_limit_frozen = true;
                    state = ArmControlState::Constrained;
                    stop_reason = Some(ArmStopReason::JointLimit);
                }
                RelativeIntentState::Active => match self.target_from_intent(intent) {
                    Some(target) => {
                        if feedback.and_then(|value| value.servo_status_code) != Some(6) {
                            self.joint_limit_recovery = false;
                        }
                        enabled = true;
                        desired_tcp_pose = Some(target);
                        target_pose = Some(target);
                        state = servo_state(feedback.and_then(|value| value.servo_status_code));
                        stop_reason =
                            servo_stop_reason(feedback.and_then(|value| value.servo_status_code));
                        if state == ArmControlState::Faulted {
                            enabled = false;
                            target_pose = None;
                            self.anchor_tcp = None;
                            self.previous_gripper_pressed = None;
                        } else if let Some(pressed) = intent.gripper_pressed
                            && self
                                .previous_gripper_pressed
                                .replace(pressed)
                                .is_some_and(|previous| previous != pressed)
                        {
                            let position_rad = if pressed {
                                self.gripper_closed_rad
                            } else {
                                self.gripper_open_rad
                            };
                            gripper_target = Some(position_rad);
                        }
                    }
                    None => {
                        state = ArmControlState::Faulted;
                        stop_reason = Some(ArmStopReason::InvalidIntent);
                        self.reanchor_required = true;
                        self.anchor_tcp = None;
                        self.previous_gripper_pressed = None;
                    }
                },
            }
        }

        let status_code = feedback.and_then(|value| value.servo_status_code);
        let status_message = feedback.and_then(|value| value.servo_status_message.clone());
        self.latest = ArmSnapshot {
            model_id: self.model.profile().model_id.clone(),
            feedback_source: self.arm_state.feedback_source,
            state,
            enabled,
            time_ns,
            joints_rad: self.arm_state.joints_rad,
            gripper_rad: self.arm_state.gripper_position_rad,
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
            arm_state: self.arm_state.clone(),
            gripper_request: if command_enabled {
                gripper_target.map(|position_rad| GripperRequestFrame {
                    request_id: 0,
                    position_rad,
                })
            } else {
                None
            },
            motion_request: None,
        };
        (command, self.latest.clone())
    }

    fn apply_feedback(&mut self, feedback: &ServoFeedbackFrame) -> Result<(), ()> {
        if feedback.schema_version != SERVO_IPC_SCHEMA_VERSION
            || feedback
                .controller_joints_rad
                .into_iter()
                .any(|value| !value.is_finite())
        {
            return Err(());
        }
        let tcp = feedback.tcp_pose.to_isometry().ok_or(())?;
        if !feedback.controller_gripper_position_rad.is_finite() {
            return Err(());
        }
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
            || intent.gripper_pressed.is_none()
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
        // The relative rotation vector is expressed in the controller's
        // anchored local frame. Controller X drives the vertical tip arc,
        // controller Z drives the horizontal tip arc, and controller Y is ignored.
        let local_rotation = relative_rotation.scaled_axis();
        let position_mapping = self.model.robot_from_nolo_position();
        let position_translation = if intent.components.position {
            position_mapping
                * Vector3::from(relative_position)
                * self.model.profile().teleoperation.translation_scale
        } else {
            Vector3::zeros()
        };
        let pitch = if intent.components.pitch {
            UnitQuaternion::from_scaled_axis(Vector3::x() * -local_rotation.x)
        } else {
            UnitQuaternion::identity()
        };
        let turn = UnitQuaternion::from_axis_angle(
            &Vector3::z_axis(),
            if intent.components.turn {
                local_rotation.z
            } else {
                0.0
            },
        );
        let pivot_to_tcp = self.model.gripper_pivot_to_tcp();
        let anchor = if self.reanchor_required {
            let current = self.current_tcp?;
            let anchor_rotation = turn.inverse() * current.rotation * pitch.inverse();
            let anchor = nalgebra::Isometry3::from_parts(
                Translation3::from(
                    current.translation.vector
                        - position_translation
                        - current.rotation * pivot_to_tcp
                        + anchor_rotation * pivot_to_tcp,
                ),
                anchor_rotation,
            );
            self.anchor_tcp = Some(anchor);
            self.reanchor_required = false;
            anchor
        } else {
            *self.anchor_tcp.get_or_insert(self.current_tcp?)
        };
        let target_rotation = turn * anchor.rotation * pitch;
        Some(Pose::from_isometry(&nalgebra::Isometry3::from_parts(
            Translation3::from(
                anchor.translation.vector + position_translation + target_rotation * pivot_to_tcp
                    - anchor.rotation * pivot_to_tcp,
            ),
            target_rotation,
        )))
    }
}

fn servo_state(code: Option<i8>) -> ArmControlState {
    match code {
        Some(1..=6) => ArmControlState::Constrained,
        _ => ArmControlState::Active,
    }
}

fn servo_stop_reason(code: Option<i8>) -> Option<ArmStopReason> {
    match code {
        Some(2) => Some(ArmStopReason::Singularity),
        Some(5) => Some(ArmStopReason::ServoCollision),
        Some(6) => Some(ArmStopReason::JointLimit),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feedback(_model: &ArmModel) -> ServoFeedbackFrame {
        ServoFeedbackFrame {
            schema_version: SERVO_IPC_SCHEMA_VERSION,
            controller_joints_rad: [0.0; ARM_DOF],
            controller_gripper_position_rad: 0.0,
            tcp_pose: Pose {
                position_m: [0.2, 0.0, 0.2],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            servo_status_code: Some(0),
            servo_status_message: Some("NO_WARNING".to_owned()),
            motion_status: None,
        }
    }

    fn intent(state: RelativeIntentState, position: Option<[f64; 3]>) -> RelativeIntent {
        RelativeIntent {
            state,
            relative_position_m: position,
            relative_orientation_xyzw: Some([0.0, 0.0, 0.0, 1.0]),
            gripper_pressed: Some(false),
            components: Default::default(),
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
        assert_eq!(snapshot.state, ArmControlState::Faulted);
        assert_eq!(snapshot.stop_reason, Some(ArmStopReason::ServoUnavailable));
    }

    #[test]
    fn gripper_model_endpoints_are_independent_from_servo_transmission_units() {
        let controller = MoveItController::new(ArmModel::embedded().unwrap());
        assert!((controller.gripper_open_rad.to_degrees() - 90.0).abs() < 1.0e-12);
        assert!((controller.gripper_closed_rad - GRIPPER_START_POSITION_RAD).abs() < 1.0e-12);
        assert_eq!(controller.latest().gripper_rad, GRIPPER_START_POSITION_RAD);
    }

    #[test]
    fn active_menu_only_moves_the_gripper_after_a_real_transition() {
        let model = ArmModel::embedded().unwrap();
        let feedback = feedback(&model);
        let mut controller = MoveItController::new(model);
        assert!(controller.apply_arm_state(&ArmStateFrame {
            gripper_position_rad: 0.6,
            ..ArmStateFrame::default()
        }));
        controller.step(
            1,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&feedback),
            Some(0),
        );

        let (initial, snapshot) = controller.step(
            2,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            Some(&feedback),
            Some(0),
        );
        assert_eq!(initial.gripper_request, None);
        assert_eq!(snapshot.gripper_rad, 0.6);

        let mut pressed = intent(RelativeIntentState::Active, Some([0.0; 3]));
        pressed.gripper_pressed = Some(true);
        let (closed, snapshot) = controller.step(3, pressed, Some(&feedback), Some(0));
        assert_eq!(
            closed
                .gripper_request
                .as_ref()
                .map(|request| request.position_rad),
            Some(GRIPPER_START_POSITION_RAD)
        );
        assert_eq!(snapshot.gripper_rad, 0.6);

        let (open, snapshot) = controller.step(
            4,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            Some(&feedback),
            Some(0),
        );
        assert_eq!(snapshot.gripper_rad, 0.6);
        let encoded = serde_json::to_value(&open).unwrap();
        assert_eq!(
            encoded["gripper_request"]["position_rad"].as_f64(),
            Some(std::f64::consts::FRAC_PI_2)
        );
        assert!(encoded.get("gripper_pressed").is_none());

        let (unchanged, snapshot) = controller.step(
            5,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            Some(&feedback),
            Some(0),
        );
        assert_eq!(unchanged.gripper_request, None);
        assert_eq!(snapshot.gripper_rad, 0.6);
    }

    #[test]
    fn gripper_position_uses_standard_joint_feedback() {
        let model = ArmModel::embedded().unwrap();
        let mut value = feedback(&model);
        value.controller_gripper_position_rad = 0.6;
        let mut controller = MoveItController::new(model);
        assert!(controller.apply_arm_state(&ArmStateFrame {
            gripper_position_rad: 0.6,
            ..ArmStateFrame::default()
        }));
        let (_, snapshot) = controller.step(
            1,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&value),
            Some(0),
        );
        assert_eq!(snapshot.gripper_rad, 0.6);
    }

    #[test]
    fn active_target_maps_all_hand_axes_at_one_to_two_scale() {
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
        let (forward, _) = controller.step(
            2,
            intent(RelativeIntentState::Active, Some([0.0, 0.0, -0.05])),
            Some(&feedback),
            Some(0),
        );
        let target = forward.target_pose.unwrap();
        assert!((target.position_m[0] - anchor.position_m[0] - 0.025).abs() < 1.0e-9);

        let (right, _) = controller.step(
            3,
            intent(RelativeIntentState::Active, Some([0.05, 0.0, 0.0])),
            Some(&feedback),
            Some(0),
        );
        let target = right.target_pose.unwrap();
        assert!((target.position_m[1] - anchor.position_m[1] + 0.025).abs() < 1.0e-9);

        let (up, _) = controller.step(
            4,
            intent(RelativeIntentState::Active, Some([0.0, 0.05, 0.0])),
            Some(&feedback),
            Some(0),
        );
        let target = up.target_pose.unwrap();
        assert!((target.position_m[2] - anchor.position_m[2] - 0.025).abs() < 1.0e-9);
    }

    #[test]
    fn pitch_and_turn_move_the_tcp_around_the_gripper_pivot() {
        let model = ArmModel::embedded().unwrap();
        let pivot_to_tcp = model.gripper_pivot_to_tcp();
        let mut feedback = feedback(&model);
        let initial_rotation =
            UnitQuaternion::from_euler_angles(-std::f64::consts::FRAC_PI_2, 0.0, 0.0);
        let initial_quaternion = initial_rotation.quaternion();
        feedback.tcp_pose.orientation_xyzw = [
            initial_quaternion.i,
            initial_quaternion.j,
            initial_quaternion.k,
            initial_quaternion.w,
        ];
        let mut controller = MoveItController::new(model);
        controller.step(
            1,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&feedback),
            Some(0),
        );
        let anchor = controller.latest().tcp_pose.unwrap().to_isometry().unwrap();
        let angle = 0.2;
        let relative = UnitQuaternion::from_scaled_axis(-Vector3::x() * angle);
        let quaternion = relative.quaternion();
        let mut value = intent(RelativeIntentState::Active, Some([0.0; 3]));
        value.relative_orientation_xyzw =
            Some([quaternion.i, quaternion.j, quaternion.k, quaternion.w]);
        let (command, _) = controller.step(2, value, Some(&feedback), Some(0));
        let target = command.target_pose.unwrap().to_isometry().unwrap();
        let anchor_pivot = anchor.translation.vector - anchor.rotation * pivot_to_tcp;
        let target_pivot = target.translation.vector - target.rotation * pivot_to_tcp;
        assert!((target_pivot - anchor_pivot).norm() < 1.0e-9);
        assert!(target.translation.vector.z > anchor.translation.vector.z);
        let pitch = (anchor.rotation.inverse() * target.rotation).scaled_axis();
        assert!((pitch - Vector3::x() * angle).norm() < 1.0e-9);

        let relative = UnitQuaternion::from_scaled_axis(Vector3::z() * angle);
        let quaternion = relative.quaternion();
        value.relative_orientation_xyzw =
            Some([quaternion.i, quaternion.j, quaternion.k, quaternion.w]);
        let (command, _) = controller.step(3, value, Some(&feedback), Some(0));
        let target = command.target_pose.unwrap().to_isometry().unwrap();
        let target_pivot = target.translation.vector - target.rotation * pivot_to_tcp;
        assert!((target_pivot - anchor_pivot).norm() < 1.0e-9);
        assert!((target.translation.vector.z - anchor.translation.vector.z).abs() < 1.0e-9);
        assert!((target.translation.vector - anchor.translation.vector).norm() > 0.0);
        let turn = (target.rotation * anchor.rotation.inverse()).scaled_axis();
        assert!((turn - Vector3::z() * angle).norm() < 1.0e-9);
    }

    #[test]
    fn teleop_component_switches_filter_position_pitch_and_turn_independently() {
        let model = ArmModel::embedded().unwrap();
        let pivot_to_tcp = model.gripper_pivot_to_tcp();
        let mut feedback = feedback(&model);
        let initial_rotation =
            UnitQuaternion::from_euler_angles(-std::f64::consts::FRAC_PI_2, 0.0, 0.0);
        let initial_quaternion = initial_rotation.quaternion();
        feedback.tcp_pose.orientation_xyzw = [
            initial_quaternion.i,
            initial_quaternion.j,
            initial_quaternion.k,
            initial_quaternion.w,
        ];
        let raw = UnitQuaternion::from_scaled_axis(Vector3::new(0.2, -0.3, 0.4));
        let quaternion = raw.quaternion();
        let target_for = |components| {
            let mut controller = MoveItController::new(model.clone());
            controller.step(
                1,
                intent(RelativeIntentState::Idle, Some([0.0; 3])),
                Some(&feedback),
                Some(0),
            );
            let anchor = controller.latest().tcp_pose.unwrap().to_isometry().unwrap();
            let mut value = intent(RelativeIntentState::Active, Some([0.05, 0.05, -0.05]));
            value.relative_orientation_xyzw =
                Some([quaternion.i, quaternion.j, quaternion.k, quaternion.w]);
            value.components = components;
            let (command, _) = controller.step(2, value, Some(&feedback), Some(0));
            (anchor, command.target_pose.unwrap().to_isometry().unwrap())
        };

        let (anchor, target) = target_for(crate::state::TeleopComponents {
            position: false,
            pitch: true,
            turn: false,
        });
        let anchor_pivot = anchor.translation.vector - anchor.rotation * pivot_to_tcp;
        let target_pivot = target.translation.vector - target.rotation * pivot_to_tcp;
        assert!((target_pivot - anchor_pivot).norm() < 1.0e-9);
        let actual = (anchor.rotation.inverse() * target.rotation).scaled_axis();
        assert!((actual + Vector3::x() * 0.2).norm() < 1.0e-9);

        let (anchor, target) = target_for(crate::state::TeleopComponents {
            position: true,
            pitch: false,
            turn: false,
        });
        assert!((target.translation.vector - anchor.translation.vector).norm() > 0.0);
        assert!((target.rotation.inverse() * anchor.rotation).angle() < 1.0e-9);

        let (anchor, target) = target_for(crate::state::TeleopComponents {
            position: false,
            pitch: false,
            turn: false,
        });
        assert!((target.translation.vector - anchor.translation.vector).norm() < 1.0e-9);
        assert!((target.rotation.inverse() * anchor.rotation).angle() < 1.0e-9);

        let (anchor, target) = target_for(crate::state::TeleopComponents {
            position: false,
            pitch: false,
            turn: true,
        });
        let anchor_pivot = anchor.translation.vector - anchor.rotation * pivot_to_tcp;
        let target_pivot = target.translation.vector - target.rotation * pivot_to_tcp;
        assert!((target_pivot - anchor_pivot).norm() < 1.0e-9);
        assert!((target.translation.vector - anchor.translation.vector).norm() > 0.0);
        let actual = (target.rotation * anchor.rotation.inverse()).scaled_axis();
        assert!((actual - Vector3::z() * 0.4).norm() < 1.0e-9);
    }

    #[test]
    fn arm_state_is_exposed_without_a_second_joint_representation() {
        let model = ArmModel::embedded().unwrap();
        let value = feedback(&model);
        let mut controller = MoveItController::new(model);
        assert!(controller.apply_arm_state(&ArmStateFrame {
            joints_rad: [0.3, 0.2, 0.0, 0.0, 0.0, 0.1],
            ..ArmStateFrame::default()
        }));
        let (_, snapshot) = controller.step(
            1,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&value),
            Some(0),
        );
        assert!((snapshot.joints_rad[0] - 0.3).abs() < 1.0e-9);
        assert_eq!(snapshot.tcp_pose, Some(value.tcp_pose));
    }

    #[test]
    fn feedback_requires_current_schema_and_a_valid_ros_tcp_pose() {
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
            {
                let model = ArmModel::embedded().unwrap();
                let mut value = feedback(&model);
                value.controller_gripper_position_rad = f64::NAN;
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
                Some(ArmStopReason::ServoInvalidFeedback)
            );
        }
    }

    #[test]
    fn delayed_feedback_is_diagnostic_but_invalid_feedback_disables_output() {
        let model = ArmModel::embedded().unwrap();
        let mut value = feedback(&model);
        let mut controller = MoveItController::new(model);
        let (delayed, snapshot) = controller.step(
            1,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            Some(&value),
            Some(101),
        );
        assert!(delayed.enabled);
        assert_eq!(snapshot.stop_reason, None);

        value.controller_joints_rad[0] = f64::NAN;
        let (invalid, snapshot) = controller.step(
            2,
            intent(RelativeIntentState::Active, Some([0.0; 3])),
            Some(&value),
            Some(0),
        );
        assert!(!invalid.enabled);
        assert_eq!(
            snapshot.stop_reason,
            Some(ArmStopReason::ServoInvalidFeedback)
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
        assert_eq!(snapshot.state, ArmControlState::Constrained);
        assert_eq!(snapshot.stop_reason, Some(ArmStopReason::Singularity));
    }

    #[test]
    fn joint_limit_freezes_until_trigger_is_released_then_reanchors() {
        let model = ArmModel::embedded().unwrap();
        let mut value = feedback(&model);
        let mut controller = MoveItController::new(model);

        assert!(
            controller
                .step(
                    1,
                    intent(RelativeIntentState::Active, Some([0.0; 3])),
                    Some(&value),
                    Some(0),
                )
                .0
                .enabled
        );

        value.servo_status_code = Some(6);
        let (frozen, snapshot) = controller.step(
            2,
            intent(RelativeIntentState::Active, Some([0.1; 3])),
            Some(&value),
            Some(0),
        );
        assert!(!frozen.enabled);
        assert_eq!(frozen.target_pose, None);
        assert_eq!(snapshot.state, ArmControlState::Constrained);
        assert_eq!(snapshot.stop_reason, Some(ArmStopReason::JointLimit));

        value.servo_status_code = Some(0);
        assert!(
            !controller
                .step(
                    3,
                    intent(RelativeIntentState::Active, Some([0.0; 3])),
                    Some(&value),
                    Some(0),
                )
                .0
                .enabled
        );
        controller.step(
            4,
            intent(RelativeIntentState::Idle, Some([0.0; 3])),
            Some(&value),
            Some(0),
        );

        value.servo_status_code = Some(6);
        assert!(
            controller
                .step(
                    5,
                    intent(RelativeIntentState::Active, Some([0.0; 3])),
                    Some(&value),
                    Some(0),
                )
                .0
                .enabled
        );
        value.servo_status_code = Some(0);
        assert!(
            controller
                .step(
                    6,
                    intent(RelativeIntentState::Active, Some([0.01; 3])),
                    Some(&value),
                    Some(0),
                )
                .0
                .enabled
        );
    }

    #[test]
    fn moveit_deceleration_is_not_reported_as_a_stop_reason() {
        assert_eq!(servo_state(Some(-1)), ArmControlState::Active);
        assert_eq!(servo_stop_reason(Some(-1)), None);
        for code in [1, 3, 4] {
            assert_eq!(servo_state(Some(code)), ArmControlState::Constrained);
            assert_eq!(servo_stop_reason(Some(code)), None);
        }
        assert_eq!(servo_state(Some(2)), ArmControlState::Constrained);
        assert_eq!(servo_stop_reason(Some(2)), Some(ArmStopReason::Singularity));
        assert_eq!(servo_state(Some(5)), ArmControlState::Constrained);
        assert_eq!(
            servo_stop_reason(Some(5)),
            Some(ArmStopReason::ServoCollision)
        );
        assert_eq!(servo_state(Some(6)), ArmControlState::Constrained);
        assert_eq!(servo_stop_reason(Some(6)), Some(ArmStopReason::JointLimit));
    }

    #[test]
    fn moveit_invalid_status_does_not_latch_or_suppress_the_next_target() {
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
        assert!(command.enabled);
        assert!(command.target_pose.is_some());
        assert_eq!(snapshot.state, ArmControlState::Active);
        assert_eq!(snapshot.stop_reason, None);
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
            assert_eq!(snapshot.state, ArmControlState::Faulted);
            let expected = if code.is_none() {
                ArmStopReason::ServoUnavailable
            } else {
                ArmStopReason::ServoInvalidFeedback
            };
            assert_eq!(snapshot.stop_reason, Some(expected));
        }
    }

    #[test]
    fn feedback_recovery_reanchors_without_requiring_intent_release() {
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

        let mut invalid = feedback.clone();
        invalid.controller_joints_rad[0] = f64::NAN;
        assert!(
            !controller
                .step(
                    3,
                    intent(RelativeIntentState::Active, Some([0.01; 3])),
                    Some(&invalid),
                    Some(0),
                )
                .0
                .enabled
        );
        let (held, snapshot) = controller.step(
            4,
            intent(RelativeIntentState::Active, Some([0.01; 3])),
            Some(&feedback),
            Some(0),
        );
        assert!(held.enabled);
        assert_eq!(snapshot.stop_reason, None);
        let target = held.target_pose.unwrap();
        for axis in 0..3 {
            assert!((target.position_m[axis] - feedback.tcp_pose.position_m[axis]).abs() < 1.0e-12);
        }
        for axis in 0..4 {
            assert!(
                (target.orientation_xyzw[axis] - feedback.tcp_pose.orientation_xyzw[axis]).abs()
                    < 1.0e-12
            );
        }
    }
}
