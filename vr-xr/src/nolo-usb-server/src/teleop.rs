//! Controller 0 teleoperation intent state, independent of any robot backend.

use nalgebra::{Quaternion, UnitQuaternion};
use serde::Serialize;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TeleopSample {
    pub receive_time_ns: u64,
    pub sample_sequence: u8,
    pub raw_position: [f32; 3],
    pub filtered_position: Option<[f32; 3]>,
    /// Fusion quaternion in `[x, y, z, w]` order.
    pub orientation: [f32; 4],
    pub communication_fresh: bool,
    pub fusion_initialising: bool,
    pub gyro_calibration_active: bool,
    pub trigger_pressed: bool,
    pub squeeze_pressed: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TeleopIntentState {
    Idle,
    Active,
    Faulted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TeleopStopReason {
    NoSample,
    AwaitingSqueeze,
    AwaitingSqueezeRelease,
    SqueezeReleased,
    CommunicationStale,
    CalibrationActive,
    FilteredPositionUnavailable,
    NonFinitePosition,
    InvalidOrientation,
    InvalidUsbReport,
    UsbDisconnected,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct TeleopIntent {
    pub state: TeleopIntentState,
    pub receive_time_ns: u64,
    pub sample_sequence: Option<u8>,
    pub squeeze_pressed: bool,
    pub gripper_closed: Option<bool>,
    pub relative_position: Option<[f32; 3]>,
    pub relative_orientation: Option<[f32; 4]>,
    pub stop_reason: Option<TeleopStopReason>,
}

pub struct TeleopIntentMachine {
    state: TeleopIntentState,
    release_observed: bool,
    anchor_position: Option<[f32; 3]>,
    anchor_orientation: Option<UnitQuaternion<f32>>,
    latest: TeleopIntent,
}

impl Default for TeleopIntentMachine {
    fn default() -> Self {
        Self::new()
    }
}

impl TeleopIntentMachine {
    pub fn new() -> Self {
        Self {
            state: TeleopIntentState::Idle,
            release_observed: false,
            anchor_position: None,
            anchor_orientation: None,
            latest: TeleopIntent {
                state: TeleopIntentState::Idle,
                receive_time_ns: 0,
                sample_sequence: None,
                squeeze_pressed: false,
                gripper_closed: None,
                relative_position: None,
                relative_orientation: None,
                stop_reason: Some(TeleopStopReason::NoSample),
            },
        }
    }

    pub fn latest(&self) -> &TeleopIntent {
        &self.latest
    }

    pub fn update(&mut self, sample: TeleopSample) -> TeleopIntent {
        let validated = match validate_sample(sample) {
            Ok(validated) => validated,
            Err(reason) => {
                self.enter_fault();
                self.latest = intent_from_sample(
                    sample,
                    TeleopIntentState::Faulted,
                    None,
                    None,
                    Some(reason),
                );
                return self.latest.clone();
            }
        };

        match self.state {
            TeleopIntentState::Faulted => {
                if sample.squeeze_pressed {
                    self.latest = intent_from_sample(
                        sample,
                        TeleopIntentState::Faulted,
                        None,
                        None,
                        Some(TeleopStopReason::AwaitingSqueezeRelease),
                    );
                } else {
                    self.state = TeleopIntentState::Idle;
                    self.release_observed = true;
                    self.latest = intent_from_sample(
                        sample,
                        TeleopIntentState::Idle,
                        None,
                        None,
                        Some(TeleopStopReason::AwaitingSqueeze),
                    );
                }
            }
            TeleopIntentState::Idle => {
                if !sample.squeeze_pressed {
                    self.release_observed = true;
                    self.clear_anchor();
                    self.latest = intent_from_sample(
                        sample,
                        TeleopIntentState::Idle,
                        None,
                        None,
                        Some(TeleopStopReason::AwaitingSqueeze),
                    );
                } else if self.release_observed {
                    self.state = TeleopIntentState::Active;
                    self.release_observed = false;
                    self.anchor_position = Some(validated.filtered_position);
                    self.anchor_orientation = Some(validated.orientation);
                    self.latest = intent_from_sample(
                        sample,
                        TeleopIntentState::Active,
                        Some([0.0; 3]),
                        Some([0.0, 0.0, 0.0, 1.0]),
                        None,
                    );
                } else {
                    self.latest = intent_from_sample(
                        sample,
                        TeleopIntentState::Idle,
                        None,
                        None,
                        Some(TeleopStopReason::AwaitingSqueezeRelease),
                    );
                }
            }
            TeleopIntentState::Active => {
                if !sample.squeeze_pressed {
                    self.state = TeleopIntentState::Idle;
                    self.release_observed = true;
                    self.clear_anchor();
                    self.latest = intent_from_sample(
                        sample,
                        TeleopIntentState::Idle,
                        None,
                        None,
                        Some(TeleopStopReason::SqueezeReleased),
                    );
                } else {
                    let anchor_position = self
                        .anchor_position
                        .expect("active intent must have a position anchor");
                    let anchor_orientation = self
                        .anchor_orientation
                        .as_ref()
                        .expect("active intent must have an orientation anchor");
                    let relative_position = std::array::from_fn(|axis| {
                        validated.filtered_position[axis] - anchor_position[axis]
                    });
                    let relative_orientation =
                        quaternion_array(&(anchor_orientation.inverse() * validated.orientation));
                    self.latest = intent_from_sample(
                        sample,
                        TeleopIntentState::Active,
                        Some(relative_position),
                        Some(relative_orientation),
                        None,
                    );
                }
            }
        }

        self.latest.clone()
    }

    /// Immediately invalidates an active or recoverable intent when no
    /// controller sample exists to carry the failure (for example a USB read
    /// error). A valid released sample is required before activation can occur
    /// again.
    pub fn force_fault(&mut self, receive_time_ns: u64, reason: TeleopStopReason) -> TeleopIntent {
        self.enter_fault();
        self.latest.state = TeleopIntentState::Faulted;
        self.latest.receive_time_ns = receive_time_ns;
        self.latest.relative_position = None;
        self.latest.relative_orientation = None;
        self.latest.gripper_closed = None;
        self.latest.stop_reason = Some(reason);
        self.latest.clone()
    }

    fn enter_fault(&mut self) {
        self.state = TeleopIntentState::Faulted;
        self.release_observed = false;
        self.clear_anchor();
    }

    fn clear_anchor(&mut self) {
        self.anchor_position = None;
        self.anchor_orientation = None;
    }
}

struct ValidatedSample {
    filtered_position: [f32; 3],
    orientation: UnitQuaternion<f32>,
}

fn validate_sample(sample: TeleopSample) -> Result<ValidatedSample, TeleopStopReason> {
    if !sample.communication_fresh {
        return Err(TeleopStopReason::CommunicationStale);
    }
    if sample.fusion_initialising || sample.gyro_calibration_active {
        return Err(TeleopStopReason::CalibrationActive);
    }
    if !finite_vector(sample.raw_position) {
        return Err(TeleopStopReason::NonFinitePosition);
    }
    let Some(filtered_position) = sample.filtered_position else {
        return Err(TeleopStopReason::FilteredPositionUnavailable);
    };
    if !finite_vector(filtered_position) {
        return Err(TeleopStopReason::NonFinitePosition);
    }
    let orientation =
        unit_quaternion(sample.orientation).ok_or(TeleopStopReason::InvalidOrientation)?;

    Ok(ValidatedSample {
        filtered_position,
        orientation,
    })
}

fn finite_vector(vector: [f32; 3]) -> bool {
    vector.into_iter().all(f32::is_finite)
}

fn unit_quaternion(value: [f32; 4]) -> Option<UnitQuaternion<f32>> {
    if value.into_iter().any(|component| !component.is_finite()) {
        return None;
    }
    let quaternion = Quaternion::new(value[3], value[0], value[1], value[2]);
    let norm_squared = quaternion.norm_squared();
    if !norm_squared.is_finite() || norm_squared <= f32::EPSILON {
        return None;
    }
    Some(UnitQuaternion::new_normalize(quaternion))
}

fn quaternion_array(value: &UnitQuaternion<f32>) -> [f32; 4] {
    let quaternion = value.quaternion();
    let mut result = [quaternion.i, quaternion.j, quaternion.k, quaternion.w];
    if result[3] < 0.0 {
        result
            .iter_mut()
            .for_each(|component| *component = -*component);
    }
    result
}

fn intent_from_sample(
    sample: TeleopSample,
    state: TeleopIntentState,
    relative_position: Option<[f32; 3]>,
    relative_orientation: Option<[f32; 4]>,
    stop_reason: Option<TeleopStopReason>,
) -> TeleopIntent {
    TeleopIntent {
        state,
        receive_time_ns: sample.receive_time_ns,
        sample_sequence: Some(sample.sample_sequence),
        squeeze_pressed: sample.squeeze_pressed,
        gripper_closed: (state == TeleopIntentState::Active).then_some(sample.trigger_pressed),
        relative_position,
        relative_orientation,
        stop_reason,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(sequence: u8, squeeze_pressed: bool) -> TeleopSample {
        TeleopSample {
            receive_time_ns: u64::from(sequence) * 1_000_000,
            sample_sequence: sequence,
            raw_position: [1.0, 2.0, 3.0],
            filtered_position: Some([1.0, 2.0, 3.0]),
            orientation: [0.0, 0.0, 0.0, 1.0],
            communication_fresh: true,
            fusion_initialising: false,
            gyro_calibration_active: false,
            trigger_pressed: false,
            squeeze_pressed,
        }
    }

    fn assert_vector_near(actual: [f32; 3], expected: [f32; 3]) {
        for axis in 0..3 {
            assert!((actual[axis] - expected[axis]).abs() < 1.0e-6);
        }
    }

    fn assert_quaternion_near(actual: [f32; 4], expected: [f32; 4]) {
        for axis in 0..4 {
            assert!((actual[axis] - expected[axis]).abs() < 1.0e-5);
        }
    }

    fn prepare(machine: &mut TeleopIntentMachine) {
        let idle = machine.update(sample(1, false));
        assert_eq!(idle.state, TeleopIntentState::Idle);
        assert_eq!(idle.stop_reason, Some(TeleopStopReason::AwaitingSqueeze));
    }

    #[test]
    fn requires_an_observed_release_before_first_activation() {
        let mut machine = TeleopIntentMachine::new();
        let held_at_startup = machine.update(sample(1, true));
        assert_eq!(held_at_startup.state, TeleopIntentState::Idle);
        assert_eq!(
            held_at_startup.stop_reason,
            Some(TeleopStopReason::AwaitingSqueezeRelease)
        );

        machine.update(sample(2, false));
        let active = machine.update(sample(3, true));
        assert_eq!(active.state, TeleopIntentState::Active);
        assert_eq!(active.relative_position, Some([0.0; 3]));
        assert_eq!(active.relative_orientation, Some([0.0, 0.0, 0.0, 1.0]));
    }

    #[test]
    fn active_intent_reports_relative_translation_and_rotation() {
        let mut machine = TeleopIntentMachine::new();
        prepare(&mut machine);
        machine.update(sample(2, true));

        let angle = std::f32::consts::FRAC_PI_2;
        let mut moved = sample(3, true);
        moved.filtered_position = Some([1.2, 1.9, 3.3]);
        moved.orientation = [0.0, 0.0, (angle / 2.0).sin(), (angle / 2.0).cos()];
        let intent = machine.update(moved);

        assert_eq!(intent.state, TeleopIntentState::Active);
        assert_vector_near(intent.relative_position.unwrap(), [0.2, -0.1, 0.3]);
        assert_quaternion_near(
            intent.relative_orientation.unwrap(),
            [0.0, 0.0, (angle / 2.0).sin(), (angle / 2.0).cos()],
        );
    }

    #[test]
    fn trigger_controls_gripper_without_controlling_activation() {
        let mut machine = TeleopIntentMachine::new();
        let mut trigger_only = sample(1, false);
        trigger_only.trigger_pressed = true;
        let idle = machine.update(trigger_only);
        assert_eq!(idle.state, TeleopIntentState::Idle);
        assert_eq!(idle.gripper_closed, None);

        let open = machine.update(sample(2, true));
        assert_eq!(open.state, TeleopIntentState::Active);
        assert_eq!(open.gripper_closed, Some(false));

        let mut closed_sample = sample(3, true);
        closed_sample.trigger_pressed = true;
        let closed = machine.update(closed_sample);
        assert_eq!(closed.state, TeleopIntentState::Active);
        assert_eq!(closed.gripper_closed, Some(true));
    }

    #[test]
    fn squeeze_release_ends_the_intent() {
        let mut machine = TeleopIntentMachine::new();
        prepare(&mut machine);
        machine.update(sample(2, true));
        let idle = machine.update(sample(3, false));
        assert_eq!(idle.state, TeleopIntentState::Idle);
        assert_eq!(idle.stop_reason, Some(TeleopStopReason::SqueezeReleased));
        assert_eq!(idle.relative_position, None);
    }

    #[test]
    fn stale_data_faults_and_recovery_cannot_auto_reenable() {
        let mut machine = TeleopIntentMachine::new();
        prepare(&mut machine);
        machine.update(sample(2, true));

        let mut stale = sample(3, true);
        stale.communication_fresh = false;
        let fault = machine.update(stale);
        assert_eq!(fault.state, TeleopIntentState::Faulted);
        assert_eq!(
            fault.stop_reason,
            Some(TeleopStopReason::CommunicationStale)
        );

        let still_held = machine.update(sample(4, true));
        assert_eq!(still_held.state, TeleopIntentState::Faulted);
        assert_eq!(
            still_held.stop_reason,
            Some(TeleopStopReason::AwaitingSqueezeRelease)
        );
        let released = machine.update(sample(5, false));
        assert_eq!(released.state, TeleopIntentState::Idle);
        let active = machine.update(sample(6, true));
        assert_eq!(active.state, TeleopIntentState::Active);
        assert_eq!(active.relative_position, Some([0.0; 3]));
    }

    #[test]
    fn calibration_faults_an_active_intent() {
        let mut machine = TeleopIntentMachine::new();
        prepare(&mut machine);
        machine.update(sample(2, true));
        let mut calibrating = sample(3, true);
        calibrating.fusion_initialising = true;
        let fault = machine.update(calibrating);
        assert_eq!(fault.state, TeleopIntentState::Faulted);
        assert_eq!(fault.stop_reason, Some(TeleopStopReason::CalibrationActive));
    }

    #[test]
    fn non_finite_position_and_orientation_are_rejected() {
        let mut position_machine = TeleopIntentMachine::new();
        let mut bad_position = sample(1, false);
        bad_position.raw_position[0] = f32::NAN;
        let position_fault = position_machine.update(bad_position);
        assert_eq!(
            position_fault.stop_reason,
            Some(TeleopStopReason::NonFinitePosition)
        );

        let mut orientation_machine = TeleopIntentMachine::new();
        let mut bad_orientation = sample(1, false);
        bad_orientation.orientation = [0.0; 4];
        let orientation_fault = orientation_machine.update(bad_orientation);
        assert_eq!(
            orientation_fault.stop_reason,
            Some(TeleopStopReason::InvalidOrientation)
        );
    }

    #[test]
    fn missing_filtered_position_is_rejected() {
        let mut machine = TeleopIntentMachine::new();
        let mut missing = sample(1, false);
        missing.filtered_position = None;
        let fault = machine.update(missing);
        assert_eq!(
            fault.stop_reason,
            Some(TeleopStopReason::FilteredPositionUnavailable)
        );
    }

    #[test]
    fn quaternion_sign_does_not_change_relative_rotation() {
        let mut machine = TeleopIntentMachine::new();
        prepare(&mut machine);
        machine.update(sample(2, true));
        let mut equivalent = sample(3, true);
        equivalent.orientation = [0.0, 0.0, 0.0, -1.0];
        let intent = machine.update(equivalent);
        assert_quaternion_near(intent.relative_orientation.unwrap(), [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn force_fault_clears_active_output_and_requires_release() {
        let mut machine = TeleopIntentMachine::new();
        prepare(&mut machine);
        machine.update(sample(2, true));
        let disconnected = machine.force_fault(3_000_000, TeleopStopReason::UsbDisconnected);
        assert_eq!(disconnected.state, TeleopIntentState::Faulted);
        assert_eq!(disconnected.relative_position, None);

        assert_eq!(
            machine.update(sample(4, true)).state,
            TeleopIntentState::Faulted
        );
        assert_eq!(
            machine.update(sample(5, false)).state,
            TeleopIntentState::Idle
        );
    }
}
