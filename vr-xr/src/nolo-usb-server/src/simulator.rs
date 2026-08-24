//! Deterministic NOLO CV1 encrypted-report simulator.
//!
//! The simulator stops at the same 64-byte report boundary as the physical
//! HID transport.  Consumers must pass [`VirtualNolo::next_report`] back
//! through the production decoder and normal Fusion/filter/state pipeline.

use anyhow::Result;

use crate::protocol::{REPORT_SIZE, RawFrame, encode_report};

pub const REPORT_RATE_HZ: f64 = 240.0;
pub const REPORT_PERIOD_SECONDS: f64 = 1.0 / REPORT_RATE_HZ;
// The browser may request the three-second Fusion calibration only after it
// has received a valid virtual frame and observed one second of Menu hold.
// Keep Controller 0 released until ten seconds so calibration completion and
// the mandatory post-fault Squeeze release are deterministic before pre-lift.
pub const CALIBRATION_SECONDS: f64 = 10.0;
pub const STARTUP_LIFT_SECONDS: f64 = 3.0;
pub const MOTION_START_SECONDS: f64 = CALIBRATION_SECONDS + STARTUP_LIFT_SECONDS;
pub const ACTION_SECONDS: f64 = 3.0;
pub const ACTION_COUNT: usize = 12;
pub const LOOP_SECONDS: f64 = ACTION_SECONDS * ACTION_COUNT as f64;
pub const DEFAULT_REPORT_COUNT: u64 =
    ((MOTION_START_SECONDS + LOOP_SECONDS) * REPORT_RATE_HZ) as u64;

const POSITION_AMPLITUDE_METERS: f64 = 0.10;
const STARTUP_LIFT_METERS: f64 = 0.20;
const CONTROLLER_HEAD_TO_TAIL_METERS: f64 = 0.205;
const HEAD_DISPLACEMENT_METERS: f64 = 0.03;
const TURN_HEAD_DISPLACEMENT_METERS: f64 = 0.05;
const GYRO_DPS_PER_COUNT: f64 = 2000.0 / 32768.0;
const CONTROLLER_ACCEL_COUNTS_PER_G: f64 = 1024.0;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VirtualSample {
    pub frame: RawFrame,
    pub elapsed_seconds: f64,
    pub phase: &'static str,
}

#[derive(Debug, Default)]
pub struct VirtualNolo {
    report_index: u64,
    controller_sequences: [u8; 2],
    hmd_sequences: [u8; 2],
}

impl VirtualNolo {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn next_sample(&mut self) -> VirtualSample {
        let controller_id = (self.report_index & 1) as usize;
        let elapsed_seconds = self.report_index as f64 * REPORT_PERIOD_SECONDS;
        self.controller_sequences[controller_id] =
            self.controller_sequences[controller_id].wrapping_add(1);
        self.hmd_sequences[controller_id] = self.hmd_sequences[controller_id].wrapping_add(1);

        let (position, quaternion, angular_velocity, phase) = if controller_id == 0 {
            controller_zero_motion(elapsed_seconds)
        } else {
            (
                [0.25, 1.0, 1.0],
                [0.0, 0.0, 0.0, 1.0],
                [0.0; 3],
                "手柄 2 静止",
            )
        };
        let buttons = if controller_id == 0 {
            if elapsed_seconds < 6.25 {
                1 << 2 // Menu: exercise the normal six-second browser calibration.
            } else if elapsed_seconds >= CALIBRATION_SECONDS {
                1 << 4 // Squeeze: drive one continuous lift and motion loop.
            } else {
                0
            }
        } else {
            0
        };
        let frame = RawFrame {
            controller_id: controller_id as u8,
            position: position.map(|value| value as f32),
            accelerometer: controller_accelerometer(quaternion),
            gyroscope: controller_gyroscope(angular_velocity),
            buttons,
            touchpad: None,
            unknown_22: if controller_id == 0 { 35 } else { 93 },
            unknown_23: 6 + (self.controller_sequences[controller_id] & 1),
            controller_sequence: self.controller_sequences[controller_id],
            hmd_position: [0.0, 1.55, 1.0],
            unknown_31_36: [0; 6],
            hmd_gyroscope: [0; 3],
            unknown_43_48: [0; 6],
            hmd_accelerometer: [0, 0, -16384],
            unknown_55_58: [0; 4],
            hmd_sequence: self.hmd_sequences[controller_id],
            unknown_60: 0,
            unknown_61_63: [0; 3],
        };
        self.report_index = self.report_index.saturating_add(1);
        VirtualSample {
            frame,
            elapsed_seconds,
            phase,
        }
    }

    pub fn next_report(&mut self) -> Result<[u8; REPORT_SIZE]> {
        encode_report(self.next_sample().frame)
    }
}

fn controller_zero_motion(elapsed_seconds: f64) -> ([f64; 3], [f64; 4], [f64; 3], &'static str) {
    let base_position = [0.0, 1.0, 1.0];
    if elapsed_seconds < CALIBRATION_SECONDS {
        return (
            base_position,
            [0.0, 0.0, 0.0, 1.0],
            [0.0; 3],
            "自动标定（保持静止）",
        );
    }

    let startup_lift_end = CALIBRATION_SECONDS + STARTUP_LIFT_SECONDS;
    if elapsed_seconds < startup_lift_end {
        let progress = (elapsed_seconds - CALIBRATION_SECONDS) / STARTUP_LIFT_SECONDS;
        let mut position = base_position;
        position[1] += STARTUP_LIFT_METERS * smooth_step(progress);
        return (
            position,
            [0.0, 0.0, 0.0, 1.0],
            [0.0; 3],
            "启动：向上抬起 20 cm",
        );
    }

    let mut working_position = base_position;
    working_position[1] += STARTUP_LIFT_METERS;
    let loop_time = (elapsed_seconds - MOTION_START_SECONDS) % LOOP_SECONDS;
    let action = (loop_time / ACTION_SECONDS) as usize;
    let local_time = loop_time % ACTION_SECONDS;
    let (amount, rate) = smooth_transition(local_time);
    let mut position = working_position;
    let mut axis = [0.0; 3];
    let mut angle = 0.0;
    let mut angle_rate = 0.0;
    let orientation_amplitude = (HEAD_DISPLACEMENT_METERS / CONTROLLER_HEAD_TO_TAIL_METERS).asin();
    let turn_amplitude = (TURN_HEAD_DISPLACEMENT_METERS / CONTROLLER_HEAD_TO_TAIL_METERS).asin();
    let phase = match action {
        0 => {
            position[1] += POSITION_AMPLITUDE_METERS * amount;
            "向上移动 10 cm"
        }
        1 => {
            position[1] += POSITION_AMPLITUDE_METERS * (1.0 - amount);
            "向下移动 10 cm"
        }
        2 => {
            position[0] -= POSITION_AMPLITUDE_METERS * amount;
            "向左移动 10 cm"
        }
        3 => {
            position[0] -= POSITION_AMPLITUDE_METERS * (1.0 - amount);
            "向右移动 10 cm"
        }
        4 => {
            // At [0, 1, +1], facing the base-station origin means forward -Z.
            position[2] -= POSITION_AMPLITUDE_METERS * amount;
            "向前移动 10 cm"
        }
        5 => {
            position[2] -= POSITION_AMPLITUDE_METERS * (1.0 - amount);
            "向后移动 10 cm"
        }
        6 => {
            axis = [-1.0, 0.0, 0.0];
            angle = orientation_amplitude * amount;
            angle_rate = orientation_amplitude * rate;
            "手柄头部抬起 3 cm"
        }
        7 => {
            axis = [-1.0, 0.0, 0.0];
            angle = orientation_amplitude * (1.0 - amount);
            angle_rate = -orientation_amplitude * rate;
            "手柄头部下压 3 cm"
        }
        8 => {
            axis = [0.0, -1.0, 0.0];
            angle = orientation_amplitude * amount;
            angle_rate = orientation_amplitude * rate;
            "手柄向右侧倾 3 cm"
        }
        9 => {
            axis = [0.0, -1.0, 0.0];
            angle = orientation_amplitude * (1.0 - amount);
            angle_rate = -orientation_amplitude * rate;
            "手柄向左侧倾 3 cm"
        }
        10 => {
            axis = [0.0, 0.0, 1.0];
            angle = turn_amplitude * amount;
            angle_rate = turn_amplitude * rate;
            "手柄向左旋转 5 cm"
        }
        11 => {
            axis = [0.0, 0.0, 1.0];
            angle = turn_amplitude * (1.0 - amount);
            angle_rate = -turn_amplitude * rate;
            "手柄向右旋转 5 cm"
        }
        _ => unreachable!("action index must be smaller than ACTION_COUNT"),
    };
    (
        position,
        axis_angle(axis, angle),
        axis.map(|value| value * angle_rate),
        phase,
    )
}

fn smooth_step(progress: f64) -> f64 {
    let progress = progress.clamp(0.0, 1.0);
    0.5 - 0.5 * (std::f64::consts::PI * progress).cos()
}

fn smooth_transition(local_seconds: f64) -> (f64, f64) {
    let progress = (local_seconds / ACTION_SECONDS).clamp(0.0, 1.0);
    let amount = smooth_step(progress);
    let rate =
        0.5 * std::f64::consts::PI / ACTION_SECONDS * (std::f64::consts::PI * progress).sin();
    (amount, rate)
}

fn axis_angle(axis: [f64; 3], angle: f64) -> [f64; 4] {
    if angle == 0.0 {
        return [0.0, 0.0, 0.0, 1.0];
    }
    let half = angle * 0.5;
    let sine = half.sin();
    [axis[0] * sine, axis[1] * sine, axis[2] * sine, half.cos()]
}

fn controller_accelerometer([x, y, z, w]: [f64; 4]) -> [i16; 3] {
    let gravity = [
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        1.0 - 2.0 * (x * x + y * y),
    ];
    [
        quantize_i16(gravity[0] * CONTROLLER_ACCEL_COUNTS_PER_G),
        quantize_i16(gravity[1] * CONTROLLER_ACCEL_COUNTS_PER_G),
        quantize_i16(-gravity[2] * CONTROLLER_ACCEL_COUNTS_PER_G),
    ]
}

fn controller_gyroscope(angular_velocity_rad_s: [f64; 3]) -> [i16; 3] {
    let fusion_dps = angular_velocity_rad_s.map(f64::to_degrees);
    [
        quantize_i16(-fusion_dps[0] / GYRO_DPS_PER_COUNT),
        quantize_i16(-fusion_dps[1] / GYRO_DPS_PER_COUNT),
        quantize_i16(fusion_dps[2] / GYRO_DPS_PER_COUNT),
    ]
}

fn quantize_i16(value: f64) -> i16 {
    value
        .round()
        .clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        fusion::ControllerFusion,
        protocol::decode_report,
        teleop::{TeleopIntentMachine, TeleopIntentState, TeleopSample},
    };
    use std::time::{Duration, Instant};

    fn controller_zero_at(generator: &mut VirtualNolo, seconds: f64) -> VirtualSample {
        let target = (seconds * REPORT_RATE_HZ) as u64;
        loop {
            let sample = generator.next_sample();
            if sample.frame.controller_id == 0 && generator.report_index >= target {
                return sample;
            }
        }
    }

    #[test]
    fn encrypted_reports_use_the_production_decoder_and_increment_sequences() {
        let mut generator = VirtualNolo::new();
        let first = decode_report(&generator.next_report().unwrap())
            .unwrap()
            .unwrap();
        let second = decode_report(&generator.next_report().unwrap())
            .unwrap()
            .unwrap();
        let third = decode_report(&generator.next_report().unwrap())
            .unwrap()
            .unwrap();
        assert_eq!(first.controller_id, 0);
        assert_eq!(second.controller_id, 1);
        assert_eq!(third.controller_id, 0);
        assert_eq!(third.controller_sequence, first.controller_sequence + 1);
        assert_eq!(third.hmd_sequence, first.hmd_sequence + 1);
    }

    #[test]
    fn calibration_is_stationary_and_holds_menu_for_six_seconds() {
        let mut generator = VirtualNolo::new();
        let at_start = controller_zero_at(&mut generator, 0.0);
        let at_six = controller_zero_at(&mut generator, 6.0);
        assert_eq!(at_start.frame.position, at_six.frame.position);
        assert!(at_start.frame.menu_pressed());
        assert!(at_six.frame.menu_pressed());
        assert_eq!(at_six.frame.gyroscope, [0; 3]);
        assert_eq!(at_six.frame.accelerometer, [0, 0, -1024]);
    }

    #[test]
    fn default_capture_contains_setup_and_exactly_one_motion_loop() {
        assert_eq!(MOTION_START_SECONDS, 13.0);
        assert_eq!(LOOP_SECONDS, 36.0);
        assert_eq!(DEFAULT_REPORT_COUNT, 49 * 240);
    }

    #[test]
    fn startup_continuously_lifts_twenty_centimetres_into_the_loop() {
        let mut generator = VirtualNolo::new();
        let lifted = controller_zero_at(
            &mut generator,
            CALIBRATION_SECONDS + STARTUP_LIFT_SECONDS - 0.01,
        );
        let motion = controller_zero_at(&mut generator, MOTION_START_SECONDS);

        assert!((lifted.frame.position[1] - 1.2).abs() < 1.0e-4);
        assert!(lifted.frame.squeeze_pressed());
        assert!(motion.frame.squeeze_pressed());
        assert!((motion.frame.position[1] - 1.2).abs() < 1.0e-5);
    }

    #[test]
    fn position_actions_follow_requested_ten_centimetre_order() {
        let mut generator = VirtualNolo::new();
        let endpoint = ACTION_SECONDS - 0.01;
        let up = controller_zero_at(&mut generator, MOTION_START_SECONDS + endpoint);
        let down = controller_zero_at(
            &mut generator,
            MOTION_START_SECONDS + ACTION_SECONDS + endpoint,
        );
        let left = controller_zero_at(
            &mut generator,
            MOTION_START_SECONDS + 2.0 * ACTION_SECONDS + endpoint,
        );
        let right = controller_zero_at(
            &mut generator,
            MOTION_START_SECONDS + 3.0 * ACTION_SECONDS + endpoint,
        );
        let forward = controller_zero_at(
            &mut generator,
            MOTION_START_SECONDS + 4.0 * ACTION_SECONDS + endpoint,
        );
        let backward = controller_zero_at(
            &mut generator,
            MOTION_START_SECONDS + 5.0 * ACTION_SECONDS + endpoint,
        );
        assert!((up.frame.position[1] - 1.3).abs() < 1.0e-5);
        assert!((down.frame.position[1] - 1.2).abs() < 1.0e-5);
        assert!((left.frame.position[0] + 0.1).abs() < 1.0e-5);
        assert!(right.frame.position[0].abs() < 1.0e-5);
        assert!((forward.frame.position[2] - 0.9).abs() < 1.0e-5);
        assert!((backward.frame.position[2] - 1.0).abs() < 1.0e-5);
    }

    #[test]
    fn loop_up_down_pair_stays_twenty_centimetres_above_original_zero() {
        let mut generator = VirtualNolo::new();
        let end = MOTION_START_SECONDS + 2.0 * ACTION_SECONDS;
        loop {
            let sample = generator.next_sample();
            if sample.elapsed_seconds > end {
                break;
            }
            if sample.frame.controller_id == 0 && sample.elapsed_seconds >= MOTION_START_SECONDS {
                assert!(sample.frame.position[1] >= 1.2 - 1.0e-5);
            }
        }
    }

    #[test]
    fn delayed_fusion_calibration_still_allows_pre_lift_to_drive_teleop() {
        let mut generator = VirtualNolo::new();
        let mut fusion = ControllerFusion::new();
        let mut teleop = TeleopIntentMachine::new();
        let start = Instant::now();
        let mut calibration_started = false;
        let mut active_pre_lift_seen = false;
        let mut final_pre_lift_relative_y = None;
        let end = CALIBRATION_SECONDS + STARTUP_LIFT_SECONDS - 0.01;

        loop {
            let sample = generator.next_sample();
            if sample.elapsed_seconds > end {
                break;
            }
            if sample.frame.controller_id != 0 {
                continue;
            }
            if !calibration_started && sample.elapsed_seconds >= 4.0 {
                fusion.start_pose_calibration();
                calibration_started = true;
            }
            let now = start + Duration::from_secs_f64(sample.elapsed_seconds);
            let orientation = fusion.update(sample.frame, now);
            let diagnostics = fusion.diagnostics();
            let intent = teleop.update(TeleopSample {
                receive_time_ns: (sample.elapsed_seconds * 1.0e9) as u64,
                sample_sequence: sample.frame.controller_sequence,
                raw_position: sample.frame.position,
                filtered_position: Some(sample.frame.position),
                orientation,
                communication_fresh: true,
                fusion_initialising: diagnostics.initialising,
                gyro_calibration_active: diagnostics.gyro_calibration_active,
                trigger_pressed: sample.frame.trigger_pressed(),
                squeeze_pressed: sample.frame.squeeze_pressed(),
            });
            if sample.elapsed_seconds >= CALIBRATION_SECONDS
                && intent.state == TeleopIntentState::Active
            {
                active_pre_lift_seen = true;
                final_pre_lift_relative_y = intent.relative_position.map(|value| value[1]);
            }
        }

        assert!(calibration_started);
        assert!(active_pre_lift_seen);
        assert!(final_pre_lift_relative_y.is_some_and(|value| value > 0.19));
    }

    #[test]
    fn attitude_actions_encode_requested_tip_displacements() {
        let expected = (HEAD_DISPLACEMENT_METERS / CONTROLLER_HEAD_TO_TAIL_METERS).asin();
        let displacement = expected.sin() * CONTROLLER_HEAD_TO_TAIL_METERS;
        assert!((displacement - HEAD_DISPLACEMENT_METERS).abs() < 2.0e-4);
        assert!((expected.to_degrees() - 8.415).abs() < 0.001);

        let expected_turn = (TURN_HEAD_DISPLACEMENT_METERS / CONTROLLER_HEAD_TO_TAIL_METERS).asin();
        let turn_displacement = expected_turn.sin() * CONTROLLER_HEAD_TO_TAIL_METERS;
        assert!((turn_displacement - TURN_HEAD_DISPLACEMENT_METERS).abs() < 2.0e-4);
        assert!((expected_turn.to_degrees() - 14.117).abs() < 0.001);
    }

    #[test]
    fn generated_imu_drives_fusion_in_the_expected_attitude_directions() {
        let mut generator = VirtualNolo::new();
        let mut fusion = ControllerFusion::new();
        let start = Instant::now();
        let head_up_peak = MOTION_START_SECONDS + 7.0 * ACTION_SECONDS - 0.01;
        let right_tilt_peak = MOTION_START_SECONDS + 9.0 * ACTION_SECONDS - 0.01;
        let left_turn_peak = MOTION_START_SECONDS + 11.0 * ACTION_SECONDS - 0.01;
        let right_turn_end = MOTION_START_SECONDS + 12.0 * ACTION_SECONDS - 0.01;
        let mut head_up = None;
        let mut right_tilt = None;
        let mut left_turn = None;
        let mut right_turn = None;
        while right_turn.is_none() {
            let sample = generator.next_sample();
            if sample.frame.controller_id != 0 {
                continue;
            }
            let orientation = fusion.update(
                sample.frame,
                start + Duration::from_secs_f64(sample.elapsed_seconds),
            );
            if (sample.elapsed_seconds - head_up_peak).abs() < REPORT_PERIOD_SECONDS {
                head_up = Some(orientation);
            }
            if (sample.elapsed_seconds - right_tilt_peak).abs() < REPORT_PERIOD_SECONDS {
                right_tilt = Some(orientation);
            }
            if (sample.elapsed_seconds - left_turn_peak).abs() < REPORT_PERIOD_SECONDS {
                left_turn = Some(orientation);
            }
            if (sample.elapsed_seconds - right_turn_end).abs() < REPORT_PERIOD_SECONDS {
                right_turn = Some(orientation);
            }
        }
        let head_up = head_up.expect("missing head-up peak");
        let right_tilt = right_tilt.expect("missing right-tilt peak");
        let left_turn = left_turn.expect("missing left-turn peak");
        let right_turn = right_turn.expect("missing right-turn endpoint");
        assert!(
            head_up[0] < -0.04,
            "unexpected head-up quaternion: {head_up:?}"
        );
        assert!(
            right_tilt[1] < -0.04,
            "unexpected right-tilt quaternion: {right_tilt:?}"
        );
        let expected_turn_z =
            (0.5 * (TURN_HEAD_DISPLACEMENT_METERS / CONTROLLER_HEAD_TO_TAIL_METERS).asin()).sin();
        assert!(
            (left_turn[2] - expected_turn_z as f32).abs() < 0.01,
            "unexpected left-turn quaternion: {left_turn:?}"
        );
        assert!(
            right_turn[2].abs() < 0.01,
            "right turn did not return to neutral: {right_turn:?}"
        );
        for orientation in [head_up, right_tilt, left_turn, right_turn] {
            let norm_squared: f32 = orientation.into_iter().map(|value| value * value).sum();
            assert!((norm_squared - 1.0).abs() < 1.0e-3);
        }
    }
}
