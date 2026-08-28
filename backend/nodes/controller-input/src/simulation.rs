use robot_arm_messages::{
    AbsolutePoseFrame, BooleanActionSample, ControlInputFrame, FloatActionSample,
    InputSimulationState, PoseFlags, SCHEMA_VERSION,
};

const SOURCE_ID: &str = "simulation:standard-spatial-cycle";
const SAMPLE_RATE_HZ: u64 = 100;
const PHASE_SAMPLES: u64 = SAMPLE_RATE_HZ * 3;
const STARTUP_SAMPLES: u64 = PHASE_SAMPLES;
const PHASE_COUNT: u64 = 10;
const VERTICAL_AND_DEPTH_M: f64 = 0.05;
const LATERAL_M: f64 = 0.02;
const STARTUP_LIFT_M: f64 = 0.10;
const ORIENTATION_RAD: f64 = 8.0_f64.to_radians();

#[derive(Default)]
pub struct SimulationPlayback {
    sample_index: u64,
}

pub struct SimulationSample {
    pub pose: AbsolutePoseFrame,
    pub input: ControlInputFrame,
    pub state: InputSimulationState,
}

impl SimulationPlayback {
    pub fn sample(&mut self, sequence: u64, now_ns: i64) -> SimulationSample {
        let (position_m, orientation_xyzw, phase) = sample_pose(self.sample_index);
        let elapsed_s = self.sample_index as f64 / SAMPLE_RATE_HZ as f64;
        self.sample_index += 1;
        let active = BooleanActionSample {
            is_active: true,
            changed_since_last_sync: self.sample_index == 1,
            value: true,
        };
        SimulationSample {
            pose: AbsolutePoseFrame {
                schema_version: SCHEMA_VERSION,
                sequence,
                source_time_ns: now_ns,
                received_time_ns: now_ns,
                source_id: SOURCE_ID.into(),
                reference_space: "simulation_local".into(),
                position_m,
                orientation_xyzw,
                flags: PoseFlags {
                    position_valid: true,
                    position_tracked: true,
                    orientation_valid: true,
                    orientation_tracked: true,
                },
            },
            input: ControlInputFrame {
                schema_version: SCHEMA_VERSION,
                sequence,
                source_time_ns: now_ns,
                received_time_ns: now_ns,
                source_id: SOURCE_ID.into(),
                control_active: active,
                confirm_origin: BooleanActionSample::default(),
                primary_tool: FloatActionSample::default(),
                move_forward_back: FloatActionSample::default(),
                move_left_right: FloatActionSample::default(),
                move_up_down: FloatActionSample::default(),
                front_pitch: FloatActionSample::default(),
                horizontal_arc: FloatActionSample::default(),
            },
            state: InputSimulationState {
                schema_version: SCHEMA_VERSION,
                active: true,
                phase: Some(phase.into()),
                elapsed_s: Some(elapsed_s),
            },
        }
    }
}

pub fn inactive_input(sequence: u64, now_ns: i64) -> ControlInputFrame {
    ControlInputFrame {
        schema_version: SCHEMA_VERSION,
        sequence,
        source_time_ns: now_ns,
        received_time_ns: now_ns,
        source_id: SOURCE_ID.into(),
        ..Default::default()
    }
}

fn sample_pose(sample_index: u64) -> ([f64; 3], [f64; 4], &'static str) {
    if sample_index < STARTUP_SAMPLES {
        let amount = smooth(sample_index as f64 / STARTUP_SAMPLES as f64);
        return (
            [0.0, STARTUP_LIFT_M * amount, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            "启动：向上抬起 10 cm",
        );
    }
    let cycle_sample = (sample_index - STARTUP_SAMPLES) % (PHASE_SAMPLES * PHASE_COUNT);
    let phase = cycle_sample / PHASE_SAMPLES;
    let amount = smooth((cycle_sample % PHASE_SAMPLES) as f64 / PHASE_SAMPLES as f64);
    let mut position = [0.0, STARTUP_LIFT_M, 0.0];
    let mut orientation = [0.0, 0.0, 0.0, 1.0];
    let name = match phase {
        0 => {
            position[1] += VERTICAL_AND_DEPTH_M * amount;
            "向上移动 5 cm"
        }
        1 => {
            position[1] += VERTICAL_AND_DEPTH_M * (1.0 - amount);
            "向下移动 5 cm"
        }
        2 => {
            position[0] -= LATERAL_M * amount;
            "向左移动 2 cm"
        }
        3 => {
            position[0] -= LATERAL_M * (1.0 - amount);
            "向右移动 2 cm"
        }
        4 => {
            position[2] -= VERTICAL_AND_DEPTH_M * amount;
            "向前移动 5 cm"
        }
        5 => {
            position[2] -= VERTICAL_AND_DEPTH_M * (1.0 - amount);
            "向后移动 5 cm"
        }
        6 => {
            orientation = axis_angle([0.0, 0.0, 1.0], ORIENTATION_RAD * amount);
            "手柄左旋 8°"
        }
        7 => {
            orientation = axis_angle([0.0, 0.0, 1.0], ORIENTATION_RAD * (1.0 - amount));
            "手柄右旋 8°"
        }
        8 => {
            orientation = axis_angle([-1.0, 0.0, 0.0], ORIENTATION_RAD * amount);
            "手柄前部抬起 8°"
        }
        _ => {
            orientation = axis_angle([-1.0, 0.0, 0.0], ORIENTATION_RAD * (1.0 - amount));
            "手柄前部往下 8°"
        }
    };
    (position, orientation, name)
}

fn smooth(progress: f64) -> f64 {
    0.5 - 0.5 * (std::f64::consts::PI * progress).cos()
}

fn axis_angle(axis: [f64; 3], angle: f64) -> [f64; 4] {
    let half = angle * 0.5;
    let sine = half.sin();
    [axis[0] * sine, axis[1] * sine, axis[2] * sine, half.cos()]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_components_close<const N: usize>(actual: [f64; N], expected: [f64; N]) {
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert!((actual - expected).abs() <= f64::EPSILON * 16.0);
        }
    }

    #[test]
    fn cycle_preserves_all_phases_and_returns() {
        let endpoints = [
            (STARTUP_SAMPLES, "向上移动 5 cm"),
            (STARTUP_SAMPLES + PHASE_SAMPLES, "向下移动 5 cm"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 2, "向左移动 2 cm"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 3, "向右移动 2 cm"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 4, "向前移动 5 cm"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 5, "向后移动 5 cm"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 6, "手柄左旋 8°"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 7, "手柄右旋 8°"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 8, "手柄前部抬起 8°"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 9, "手柄前部往下 8°"),
        ];
        for (sample, expected) in endpoints {
            assert_eq!(sample_pose(sample).2, expected);
        }
        let (position, orientation, _) = sample_pose(STARTUP_SAMPLES + PHASE_SAMPLES * PHASE_COUNT);
        assert_eq!(position, [0.0, STARTUP_LIFT_M, 0.0]);
        assert_eq!(orientation, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn cycle_keeps_configured_distances_and_angles() {
        let cycle = STARTUP_SAMPLES;
        assert_components_close(sample_pose(cycle + PHASE_SAMPLES).0, [0.0, 0.15, 0.0]);
        assert_components_close(sample_pose(cycle + PHASE_SAMPLES * 2).0, [0.0, 0.10, 0.0]);
        assert_components_close(sample_pose(cycle + PHASE_SAMPLES * 3).0, [-0.02, 0.10, 0.0]);
        assert_components_close(sample_pose(cycle + PHASE_SAMPLES * 4).0, [0.0, 0.10, 0.0]);
        assert_components_close(sample_pose(cycle + PHASE_SAMPLES * 5).0, [0.0, 0.10, -0.05]);
        assert_components_close(sample_pose(cycle + PHASE_SAMPLES * 6).0, [0.0, 0.10, 0.0]);
        assert_components_close(
            sample_pose(cycle + PHASE_SAMPLES * 7).1,
            axis_angle([0.0, 0.0, 1.0], ORIENTATION_RAD),
        );
        assert_components_close(
            sample_pose(cycle + PHASE_SAMPLES * 9).1,
            axis_angle([-1.0, 0.0, 0.0], ORIENTATION_RAD),
        );
    }

    #[test]
    fn return_phases_retrace_the_rotation_phases() {
        for sample in [30, 75, 150, 225, 270] {
            let reverse_sample = PHASE_SAMPLES - sample;
            let left = sample_pose(STARTUP_SAMPLES + PHASE_SAMPLES * 6 + sample).1;
            let right = sample_pose(STARTUP_SAMPLES + PHASE_SAMPLES * 7 + reverse_sample).1;
            assert_components_close(left, right);

            let up = sample_pose(STARTUP_SAMPLES + PHASE_SAMPLES * 8 + sample).1;
            let down = sample_pose(STARTUP_SAMPLES + PHASE_SAMPLES * 9 + reverse_sample).1;
            assert_components_close(up, down);
        }
    }

    #[test]
    fn playback_uses_fixed_generated_time_and_never_activates_tool_action() {
        let mut playback = SimulationPlayback::default();
        let first = playback.sample(4, 100);
        let second = playback.sample(5, 200);
        assert_eq!(first.pose.sequence, 4);
        assert_eq!(first.pose.source_id, first.input.source_id);
        assert!(first.input.control_active.value);
        assert_eq!(first.input.primary_tool.value, 0.0);
        assert_eq!(first.state.elapsed_s, Some(0.0));
        assert_eq!(second.state.elapsed_s, Some(0.01));
    }
}
