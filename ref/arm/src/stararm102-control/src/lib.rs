//! Transport-free Star Arm 102-FL profile and MoveIt Servo IPC types.
//!
//! This crate contains no serial-port or motor-write code. Rust's `ArmJointIo`
//! owns the optional serial transport outside this protocol crate.

pub mod model;
pub mod servo;
pub mod state;

pub use model::{ArmModel, ArmProfile, Pose};
pub use servo::{
    ArmFeedbackSource, ArmStateFrame, GRIPPER_START_POSITION_RAD, GripperRequestFrame,
    MotionRequestAction, MotionRequestFrame, MotionState, MotionStatusFrame, MoveItController,
    SERVO_IPC_SCHEMA_VERSION, START_POSITION_JOINTS_RAD, ServoCommandFrame, ServoFeedbackFrame,
};
pub use state::{
    ArmControlState, ArmSnapshot, ArmStopReason, RelativeIntent, RelativeIntentState,
    TeleopComponents,
};
