//! Transport-neutral command and telemetry types shared with MoveIt Servo.

use crate::model::Pose;
use serde::Serialize;

const ARM_DOF: usize = 6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RelativeIntentState {
    Idle,
    Active,
    Faulted,
}

#[derive(Clone, Copy, Debug)]
pub struct RelativeIntent {
    pub state: RelativeIntentState,
    pub receive_time_ns: u64,
    pub sample_sequence: Option<u8>,
    pub relative_position_m: Option<[f64; 3]>,
    pub relative_orientation_xyzw: Option<[f64; 4]>,
    pub gripper_closed: Option<bool>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SimulationState {
    Idle,
    Active,
    Constrained,
    Faulted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SimulationStopReason {
    IntentIdle,
    IntentFaulted,
    InvalidIntent,
    WorkspaceViolation,
    ServoUnavailable,
    ServoFeedbackStale,
    ServoInvalidFeedback,
    Singularity,
    JointLimit,
    ServoHalt,
    ServoCollision,
}

#[derive(Clone, Debug, Serialize)]
pub struct SimulationSnapshot {
    pub model_id: String,
    pub simulation_only: bool,
    pub backend: String,
    pub state: SimulationState,
    pub enabled: bool,
    pub time_ns: u64,
    pub intent_receive_time_ns: u64,
    pub intent_sample_sequence: Option<u8>,
    pub joints_rad: [f64; ARM_DOF],
    pub joints_deg: [f64; ARM_DOF],
    pub model_joints_rad: [f64; ARM_DOF],
    pub model_joints_deg: [f64; ARM_DOF],
    pub joint_velocity_rad_s: [f64; ARM_DOF],
    pub gripper_closed: bool,
    pub gripper_rad: f64,
    pub gripper_deg: f64,
    pub gripper_velocity_rad_s: f64,
    pub tcp_pose: Pose,
    pub desired_tcp_pose: Option<Pose>,
    pub servo_status_code: Option<i8>,
    pub servo_status_message: Option<String>,
    pub servo_feedback_age_ms: Option<u64>,
    pub stop_reason: Option<SimulationStopReason>,
}
