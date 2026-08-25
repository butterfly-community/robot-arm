//! Transport-free Star Arm 102-FL profile and MoveIt Servo IPC types.
//!
//! This crate contains no serial-port or motor-write code. Hardware transport
//! is provided separately by the ROS adapter.

pub mod model;
pub mod servo;
pub mod simulation;

pub use model::{ArmModel, ArmProfile, Pose};
pub use servo::{
    HomeRequestAction, HomeRequestFrame, HomeState, HomeStatusFrame, MoveItController,
    SERVO_IPC_SCHEMA_VERSION, ServoCommandFrame, ServoFeedbackFrame,
};
pub use simulation::{
    RelativeIntent, RelativeIntentState, SimulationSnapshot, SimulationState, SimulationStopReason,
};
