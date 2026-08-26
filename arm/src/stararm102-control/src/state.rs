//! Transport-neutral command and telemetry types shared with MoveIt Servo.

use crate::{model::Pose, servo::ArmFeedbackSource};
use serde::{Deserialize, Serialize};

const ARM_DOF: usize = 6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RelativeIntentState {
    Idle,
    Active,
    Faulted,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct TeleopComponents {
    pub position: bool,
    pub pitch: bool,
    pub turn: bool,
}

impl Default for TeleopComponents {
    fn default() -> Self {
        Self {
            position: true,
            pitch: true,
            turn: true,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RelativeIntent {
    pub state: RelativeIntentState,
    pub relative_position_m: Option<[f64; 3]>,
    pub relative_orientation_xyzw: Option<[f64; 4]>,
    pub gripper_pressed: Option<bool>,
    pub components: TeleopComponents,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ArmControlState {
    Idle,
    Active,
    Constrained,
    Faulted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ArmStopReason {
    IntentIdle,
    IntentFaulted,
    InvalidIntent,
    ServoUnavailable,
    ServoInvalidFeedback,
    Singularity,
    JointLimit,
    ServoCollision,
}

#[derive(Clone, Debug, Serialize)]
pub struct ArmSnapshot {
    pub model_id: String,
    pub feedback_source: ArmFeedbackSource,
    pub state: ArmControlState,
    pub enabled: bool,
    pub time_ns: u64,
    pub joints_rad: [f64; ARM_DOF],
    pub gripper_rad: f64,
    pub tcp_pose: Option<Pose>,
    pub desired_tcp_pose: Option<Pose>,
    pub servo_status_code: Option<i8>,
    pub servo_status_message: Option<String>,
    pub servo_feedback_age_ms: Option<u64>,
    pub stop_reason: Option<ArmStopReason>,
}
