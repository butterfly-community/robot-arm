//! Transport-free Star Arm 102-FL profile and MoveIt Servo IPC types.
//!
//! This crate deliberately contains no serial-port or motor-write code.  The
//! embedded geometry is a versioned simulation candidate and must not be used
//! to release a powered robot without the hardware checks documented by the
//! project.

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
