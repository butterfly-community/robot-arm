//! Controller IMU conversion and Fusion AHRS state.

use fusion_ahrs::{Ahrs, Offset, OffsetSettings};
use nalgebra::Vector3;
use std::time::Instant;

use crate::protocol::RawFrame;

const CONTROLLER_SAMPLE_RATE_HZ: f32 = 120.0;
const HMD_SAMPLE_RATE_HZ: f32 = 240.0;
// NOLO reports a signed i16 over a +/-2000 degrees/second range.  Keep the
// source unit because fusion-ahrs consumes gyroscope values in degrees/second.
const GYRO_DEGREES_PER_SECOND_PER_COUNT: f32 = 2000.0 / 32768.0;
const ACCEL_COUNTS_PER_G: f32 = 1024.0;

#[derive(Clone, Copy, Debug)]
pub struct FusionDiagnostics {
    pub acceleration_error_degrees: f32,
    pub accelerometer_ignored: bool,
    pub acceleration_recovery: bool,
    pub initialising: bool,
    pub gyro_bias_dps: [f32; 3],
    pub gyro_calibration_active: bool,
    pub gyro_calibration_complete: bool,
    pub gyro_calibration_progress: f32,
}

struct ManualGyroCalibration {
    active: bool,
    complete: bool,
    accepted_samples: u32,
    required_samples: u32,
    gyroscope_limit_dps: f32,
    sum: Vector3<f32>,
    bias: Vector3<f32>,
}

impl ManualGyroCalibration {
    fn new(sample_rate_hz: f32) -> Self {
        let settings = OffsetSettings::default();
        Self {
            active: false,
            complete: false,
            accepted_samples: 0,
            required_samples: (settings.timeout * sample_rate_hz) as u32,
            gyroscope_limit_dps: settings.threshold,
            sum: Vector3::zeros(),
            bias: Vector3::zeros(),
        }
    }

    fn start(&mut self) {
        self.active = true;
        self.complete = false;
        self.accepted_samples = 0;
        self.sum = Vector3::zeros();
    }

    fn load_bias(&mut self, bias: [f32; 3]) -> bool {
        let threshold = OffsetSettings::default().threshold;
        if bias
            .into_iter()
            .any(|value| !value.is_finite() || value.abs() > threshold)
        {
            return false;
        }
        self.active = false;
        self.complete = true;
        self.accepted_samples = self.required_samples;
        self.sum = Vector3::zeros();
        self.bias = Vector3::from(bias);
        true
    }

    fn completed_bias(&self) -> Option<[f32; 3]> {
        self.complete.then(|| self.bias.into())
    }

    fn correct(&mut self, gyroscope: Vector3<f32>) -> Vector3<f32> {
        if self.active {
            let gyro_stationary = gyroscope
                .iter()
                .all(|value| value.is_finite() && value.abs() <= self.gyroscope_limit_dps);
            if gyro_stationary {
                self.sum += gyroscope;
                self.accepted_samples += 1;
                if self.accepted_samples >= self.required_samples {
                    self.bias = self.sum / self.accepted_samples as f32;
                    self.active = false;
                    self.complete = true;
                }
            } else {
                // Require a contiguous stationary window so deliberate slow
                // controller motion cannot be learned as gyroscope bias.
                self.accepted_samples = 0;
                self.sum = Vector3::zeros();
            }
        }
        gyroscope - self.bias
    }

    fn progress(&self) -> f32 {
        if self.complete {
            1.0
        } else {
            self.accepted_samples as f32 / self.required_samples as f32
        }
    }
}

struct ImuFusion {
    ahrs: Ahrs,
    offset: Offset,
    manual_calibration: ManualGyroCalibration,
    previous_update: Option<Instant>,
    sample_rate_hz: f32,
    accel_counts_per_g: f32,
}

impl ImuFusion {
    fn new(sample_rate_hz: f32, accel_counts_per_g: f32) -> Self {
        let ahrs = Ahrs::new();
        let offset = Offset::new(OffsetSettings::default(), sample_rate_hz);
        Self {
            ahrs,
            offset,
            manual_calibration: ManualGyroCalibration::new(sample_rate_hz),
            previous_update: None,
            sample_rate_hz,
            accel_counts_per_g,
        }
    }

    fn update(&mut self, gyroscope: [i16; 3], accelerometer: [i16; 3], now: Instant) -> [f32; 4] {
        let delta = self
            .previous_update
            .replace(now)
            .map(|previous| now.duration_since(previous).as_secs_f32())
            .filter(|delta| *delta > 0.0 && *delta < 0.1)
            .unwrap_or(1.0 / self.sample_rate_hz);

        // The axis reflection was verified on the physical Controller 0.  The
        // scale follows nolo-teleop's +/-2000 degrees/second interpretation.
        let gyro = Vector3::new(
            -f32::from(gyroscope[0]),
            -f32::from(gyroscope[1]),
            f32::from(gyroscope[2]),
        ) * GYRO_DEGREES_PER_SECOND_PER_COUNT;
        let accelerometer = Vector3::new(
            f32::from(accelerometer[0]),
            f32::from(accelerometer[1]),
            -f32::from(accelerometer[2]),
        ) / self.accel_counts_per_g;

        let calibration_was_active = self.manual_calibration.active;
        let manually_corrected_gyro = self.manual_calibration.correct(gyro);
        if calibration_was_active && self.manual_calibration.complete {
            // The explicit stationary-window average is now the sole persisted
            // bias.  Do not retain a second estimate learned by Fusion Offset
            // while that same window was being collected.
            self.offset.reset();
        }
        let corrected_gyro = self.offset.update(manually_corrected_gyro);
        self.ahrs
            .update_no_magnetometer(corrected_gyro, accelerometer, delta);
        let quaternion = self.ahrs.quaternion();
        let value = quaternion.quaternion();
        [value.i, value.j, value.k, value.w]
    }

    fn start_gyro_calibration(&mut self) {
        self.manual_calibration.start();
        self.offset.reset();
    }

    fn start_pose_calibration(&mut self) {
        if !self.manual_calibration.complete {
            self.start_gyro_calibration();
            self.ahrs.initialise();
            self.previous_update = None;
        }
    }

    fn load_gyro_bias(&mut self, bias: [f32; 3]) -> bool {
        let loaded = self.manual_calibration.load_bias(bias);
        if loaded {
            // A loaded manual bias and an old adaptive Offset estimate must
            // never be applied at the same time.
            self.offset.reset();
        }
        loaded
    }

    fn completed_gyro_bias(&self) -> Option<[f32; 3]> {
        self.manual_calibration.completed_bias()
    }

    fn diagnostics(&self) -> FusionDiagnostics {
        let states = self.ahrs.internal_states();
        let flags = self.ahrs.flags();
        let bias = self.manual_calibration.bias + self.offset.offset();
        FusionDiagnostics {
            acceleration_error_degrees: states.acceleration_error,
            accelerometer_ignored: states.accelerometer_ignored,
            acceleration_recovery: flags.acceleration_recovery,
            initialising: flags.initialising,
            gyro_bias_dps: bias.into(),
            gyro_calibration_active: self.manual_calibration.active,
            gyro_calibration_complete: self.manual_calibration.complete,
            gyro_calibration_progress: self.manual_calibration.progress(),
        }
    }
}

pub struct ControllerFusion(ImuFusion);

impl ControllerFusion {
    pub fn new() -> Self {
        Self(ImuFusion::new(
            CONTROLLER_SAMPLE_RATE_HZ,
            ACCEL_COUNTS_PER_G,
        ))
    }

    /// Return a quaternion in the API's `[x, y, z, w]` order.
    pub fn update(&mut self, frame: RawFrame, now: Instant) -> [f32; 4] {
        self.0.update(frame.gyroscope, frame.accelerometer, now)
    }

    pub fn start_gyro_calibration(&mut self) {
        self.0.start_gyro_calibration();
    }

    pub fn start_pose_calibration(&mut self) {
        self.0.start_pose_calibration();
    }

    pub fn load_gyro_bias(&mut self, bias: [f32; 3]) -> bool {
        self.0.load_gyro_bias(bias)
    }

    pub fn completed_gyro_bias(&self) -> Option<[f32; 3]> {
        self.0.completed_gyro_bias()
    }

    pub fn diagnostics(&self) -> FusionDiagnostics {
        self.0.diagnostics()
    }
}

pub struct HmdFusion(ImuFusion);

impl HmdFusion {
    pub fn new() -> Self {
        Self(ImuFusion::new(HMD_SAMPLE_RATE_HZ, 16384.0))
    }

    /// Return a quaternion in the API's `[x, y, z, w]` order.
    pub fn update(&mut self, frame: RawFrame, now: Instant) -> [f32; 4] {
        self.0
            .update(frame.hmd_gyroscope, frame.hmd_accelerometer, now)
    }

    pub fn start_gyro_calibration(&mut self) {
        self.0.start_gyro_calibration();
    }

    pub fn start_pose_calibration(&mut self) {
        self.0.start_pose_calibration();
    }

    pub fn load_gyro_bias(&mut self, bias: [f32; 3]) -> bool {
        self.0.load_gyro_bias(bias)
    }

    pub fn completed_gyro_bias(&self) -> Option<[f32; 3]> {
        self.0.completed_gyro_bias()
    }

    pub fn diagnostics(&self) -> FusionDiagnostics {
        self.0.diagnostics()
    }
}

impl Default for ControllerFusion {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for HmdFusion {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fusion_uses_library_default_settings() {
        let controller = ControllerFusion::new();
        let settings = controller.0.ahrs.get_settings();
        let expected = fusion_ahrs::AhrsSettings::default();
        assert_eq!(settings.convention, expected.convention);
        assert_eq!(settings.gain, expected.gain);
        assert_eq!(settings.gyroscope_range, expected.gyroscope_range);
        assert_eq!(
            settings.acceleration_rejection,
            expected.acceleration_rejection
        );
        assert_eq!(settings.magnetic_rejection, expected.magnetic_rejection);
        assert_eq!(
            settings.recovery_trigger_period,
            expected.recovery_trigger_period
        );
    }

    #[test]
    fn stationary_samples_produce_a_finite_unit_quaternion() {
        let frame = RawFrame {
            controller_id: 0,
            position: [0.0; 3],
            accelerometer: [0, 0, -1024],
            gyroscope: [0; 3],
            buttons: 0,
            touchpad: None,
            unknown_22: 0,
            unknown_23: 0,
            controller_sequence: 0,
            hmd_position: [0.0; 3],
            unknown_31_36: [0; 6],
            hmd_gyroscope: [0; 3],
            unknown_43_48: [0; 6],
            hmd_accelerometer: [0, 0, -16384],
            unknown_55_58: [0; 4],
            hmd_sequence: 0,
            unknown_60: 0,
            unknown_61_63: [0; 3],
        };
        let start = Instant::now();
        let mut fusion = ControllerFusion::new();
        let mut quaternion = [0.0; 4];
        for index in 0..600 {
            quaternion = fusion.update(
                frame,
                start
                    + std::time::Duration::from_secs_f32(index as f32 / CONTROLLER_SAMPLE_RATE_HZ),
            );
        }
        assert!(quaternion.into_iter().all(f32::is_finite));
        let length_squared: f32 = quaternion.into_iter().map(|value| value * value).sum();
        assert!((length_squared - 1.0).abs() < 1e-4);
    }

    #[test]
    fn stationary_hmd_samples_produce_a_finite_unit_quaternion() {
        let mut frame = RawFrame {
            controller_id: 0,
            position: [0.0; 3],
            accelerometer: [0, 0, -1024],
            gyroscope: [0; 3],
            buttons: 0,
            touchpad: None,
            unknown_22: 0,
            unknown_23: 0,
            controller_sequence: 0,
            hmd_position: [0.0; 3],
            unknown_31_36: [0; 6],
            hmd_gyroscope: [0; 3],
            unknown_43_48: [0; 6],
            hmd_accelerometer: [0, 0, -16384],
            unknown_55_58: [0; 4],
            hmd_sequence: 0,
            unknown_60: 0,
            unknown_61_63: [0; 3],
        };
        let start = Instant::now();
        let mut fusion = HmdFusion::new();
        let mut quaternion = [0.0; 4];
        for index in 0..1200 {
            frame.hmd_sequence = index as u8;
            quaternion = fusion.update(
                frame,
                start + std::time::Duration::from_secs_f32(index as f32 / HMD_SAMPLE_RATE_HZ),
            );
        }
        assert!(quaternion.into_iter().all(f32::is_finite));
        let length_squared: f32 = quaternion.into_iter().map(|value| value * value).sum();
        assert!((length_squared - 1.0).abs() < 1e-4);
    }

    #[test]
    fn ninety_degree_stationary_tilt_converges_to_measured_gravity() {
        let frame = RawFrame {
            controller_id: 0,
            position: [0.0; 3],
            accelerometer: [0, 1024, 0],
            gyroscope: [0; 3],
            buttons: 0,
            touchpad: None,
            unknown_22: 0,
            unknown_23: 0,
            controller_sequence: 0,
            hmd_position: [0.0; 3],
            unknown_31_36: [0; 6],
            hmd_gyroscope: [0; 3],
            unknown_43_48: [0; 6],
            hmd_accelerometer: [0, 0, -16384],
            unknown_55_58: [0; 4],
            hmd_sequence: 0,
            unknown_60: 0,
            unknown_61_63: [0; 3],
        };
        let start = Instant::now();
        let mut fusion = ControllerFusion::new();
        let mut quaternion = [0.0; 4];
        for index in 0..1200 {
            quaternion = fusion.update(
                frame,
                start
                    + std::time::Duration::from_secs_f32(index as f32 / CONTROLLER_SAMPLE_RATE_HZ),
            );
        }
        let [x, y, z, w] = quaternion;
        let gravity = Vector3::new(
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        );
        assert!(gravity.normalize().dot(&Vector3::y()) > 0.9999);
    }

    #[test]
    fn explicit_calibration_requires_three_contiguous_stationary_seconds() {
        let mut frame = RawFrame {
            controller_id: 0,
            position: [0.0; 3],
            accelerometer: [0, 0, -1024],
            gyroscope: [10, -20, 5],
            buttons: 0,
            touchpad: None,
            unknown_22: 0,
            unknown_23: 0,
            controller_sequence: 0,
            hmd_position: [0.0; 3],
            unknown_31_36: [0; 6],
            hmd_gyroscope: [0; 3],
            unknown_43_48: [0; 6],
            hmd_accelerometer: [0, 0, -16384],
            unknown_55_58: [0; 4],
            hmd_sequence: 0,
            unknown_60: 0,
            unknown_61_63: [0; 3],
        };
        let start = Instant::now();
        let mut fusion = ControllerFusion::new();
        fusion.start_gyro_calibration();

        for index in 0..120 {
            frame.controller_sequence = index as u8;
            fusion.update(
                frame,
                start
                    + std::time::Duration::from_secs_f32(index as f32 / CONTROLLER_SAMPLE_RATE_HZ),
            );
        }
        assert!(fusion.diagnostics().gyro_calibration_active);

        frame.gyroscope = [100, 0, 0];
        fusion.update(frame, start + std::time::Duration::from_secs(1));
        assert_eq!(fusion.diagnostics().gyro_calibration_progress, 0.0);

        frame.gyroscope = [10, -20, 5];
        for index in 0..360 {
            fusion.update(
                frame,
                start
                    + std::time::Duration::from_secs_f32(
                        2.0 + index as f32 / CONTROLLER_SAMPLE_RATE_HZ,
                    ),
            );
        }
        let diagnostics = fusion.diagnostics();
        assert!(!diagnostics.gyro_calibration_active);
        assert!(diagnostics.gyro_calibration_complete);
        assert_eq!(diagnostics.gyro_calibration_progress, 1.0);
        assert_eq!(fusion.0.offset.offset(), Vector3::zeros());
        let expected = [-10.0, 20.0, 5.0].map(|value| value * GYRO_DEGREES_PER_SECOND_PER_COUNT);
        for (actual, expected) in diagnostics.gyro_bias_dps.into_iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-3);
        }
    }

    #[test]
    fn first_pose_calibration_collects_bias_but_later_requests_reuse_it() {
        let frame = RawFrame {
            controller_id: 0,
            position: [0.0; 3],
            accelerometer: [0, 0, -1024],
            gyroscope: [0; 3],
            buttons: 0,
            touchpad: None,
            unknown_22: 0,
            unknown_23: 0,
            controller_sequence: 0,
            hmd_position: [0.0; 3],
            unknown_31_36: [0; 6],
            hmd_gyroscope: [0; 3],
            unknown_43_48: [0; 6],
            hmd_accelerometer: [0, 0, -16384],
            unknown_55_58: [0; 4],
            hmd_sequence: 0,
            unknown_60: 0,
            unknown_61_63: [0; 3],
        };
        let start = Instant::now();
        let mut fusion = ControllerFusion::new();
        for index in 0..600 {
            fusion.update(
                frame,
                start
                    + std::time::Duration::from_secs_f32(index as f32 / CONTROLLER_SAMPLE_RATE_HZ),
            );
        }
        assert!(!fusion.diagnostics().initialising);

        fusion.start_pose_calibration();
        let restarted = fusion.diagnostics();
        assert!(restarted.initialising);
        assert!(restarted.gyro_calibration_active);
        assert!(!restarted.gyro_calibration_complete);

        for index in 0..600 {
            fusion.update(
                frame,
                start
                    + std::time::Duration::from_secs_f32(
                        6.0 + index as f32 / CONTROLLER_SAMPLE_RATE_HZ,
                    ),
            );
        }
        let completed = fusion.diagnostics();
        assert!(!completed.initialising);
        assert!(!completed.gyro_calibration_active);
        assert!(completed.gyro_calibration_complete);

        let saved_bias = fusion.completed_gyro_bias().unwrap();
        fusion.start_pose_calibration();
        let reused = fusion.diagnostics();
        assert!(!reused.initialising);
        assert!(!reused.gyro_calibration_active);
        assert!(reused.gyro_calibration_complete);
        assert_eq!(fusion.completed_gyro_bias(), Some(saved_bias));
    }

    #[test]
    fn loaded_bias_skips_pose_calibration() {
        let mut fusion = ControllerFusion::new();
        let saved = [0.1, -0.2, 0.3];
        assert!(fusion.load_gyro_bias(saved));
        fusion.start_pose_calibration();

        let diagnostics = fusion.diagnostics();
        assert!(diagnostics.gyro_calibration_complete);
        assert!(!diagnostics.gyro_calibration_active);
        assert_eq!(fusion.completed_gyro_bias(), Some(saved));
        assert_eq!(fusion.0.offset.offset(), Vector3::zeros());
    }
}
