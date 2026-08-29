use std::collections::BTreeMap;

use robot_arm_messages::{
    ActionType, InputComponentInfo, InputDriverInfo, InputSimulationState, InputSourceInfo,
    SCHEMA_VERSION,
};

use super::RawSample;

pub(super) const DRIVER_ID: &str = "generated-test-input";
pub(super) const DEVICE_ID: &str = "generic-6dof-cycle";
pub(super) const SOURCE_ID: &str = "simulation:generic-6dof-cycle";
pub(super) const CONTROL_ACTIVE_COMPONENT: &str = "button/control_active";
pub(super) const PRIMARY_TOOL_COMPONENT: &str = "axis/primary_tool";
const SAMPLE_RATE_HZ: u64 = 100;
const PHASE_SAMPLES: u64 = SAMPLE_RATE_HZ * 3;
const STARTUP_SAMPLES: u64 = PHASE_SAMPLES;
const PHASE_COUNT: u64 = 14;
const VERTICAL_AND_DEPTH_M: f64 = 0.05;
const LATERAL_M: f64 = 0.02;
const STARTUP_LIFT_M: f64 = 0.10;
const ORIENTATION_RAD: f64 = 8.0_f64.to_radians();

#[derive(Default)]
pub(super) struct SimulationPlayback {
    sample_index: u64,
}

pub(super) struct SimulationSample {
    pub(super) raw: RawSample,
    pub(super) state: InputSimulationState,
}

pub(super) fn driver_info() -> InputDriverInfo {
    InputDriverInfo {
        driver_id: DRIVER_ID.into(),
        display_name: "生成式测试输入".into(),
        version: env!("CARGO_PKG_VERSION").into(),
        original_error: None,
    }
}

pub(super) fn source_info() -> InputSourceInfo {
    InputSourceInfo {
        source_id: SOURCE_ID.into(),
        driver_id: DRIVER_ID.into(),
        device_id: DEVICE_ID.into(),
        display_name: "通用六自由度测试源".into(),
        vendor_id: None,
        product_id: None,
        serial: None,
        position_capable: true,
        orientation_capable: true,
        action_capable: true,
        active: true,
        available_components: vec![
            InputComponentInfo {
                path: CONTROL_ACTIVE_COMPONENT.into(),
                action_type: ActionType::Boolean,
                localized_name: Some("接管控制".into()),
                definition_source: "generated-test-input".into(),
            },
            InputComponentInfo {
                path: PRIMARY_TOOL_COMPONENT.into(),
                action_type: ActionType::Float,
                localized_name: Some("夹爪连续控制".into()),
                definition_source: "generated-test-input".into(),
            },
        ],
        available_feedback_capabilities: vec![],
        original_error: None,
    }
}

impl SimulationPlayback {
    pub(super) fn sample(&mut self, sequence: u64, now_ns: i64) -> SimulationSample {
        let (position_m, orientation_xyzw, phase, primary_tool) = sample_pose(self.sample_index);
        let elapsed_s = self.sample_index as f64 / SAMPLE_RATE_HZ as f64;
        self.sample_index += 1;
        SimulationSample {
            raw: RawSample {
                sequence,
                source_time_ns: now_ns,
                received_time_ns: now_ns,
                source_id: SOURCE_ID.into(),
                position_m: Some(position_m),
                orientation_xyzw: Some(orientation_xyzw),
                components: BTreeMap::from([
                    (CONTROL_ACTIVE_COMPONENT.into(), 1.0),
                    (PRIMARY_TOOL_COMPONENT.into(), primary_tool),
                ]),
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

fn sample_pose(sample_index: u64) -> ([f64; 3], [f64; 4], &'static str, f64) {
    if sample_index < STARTUP_SAMPLES {
        let amount = smooth(sample_index as f64 / STARTUP_SAMPLES as f64);
        return (
            [0.0, STARTUP_LIFT_M * amount, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            "启动：向上抬起 10 cm",
            0.0,
        );
    }
    let cycle_sample = (sample_index - STARTUP_SAMPLES) % (PHASE_SAMPLES * PHASE_COUNT);
    let phase = cycle_sample / PHASE_SAMPLES;
    let amount = smooth((cycle_sample % PHASE_SAMPLES) as f64 / PHASE_SAMPLES as f64);
    let mut position = [0.0, STARTUP_LIFT_M, 0.0];
    let mut orientation = [0.0, 0.0, 0.0, 1.0];
    let mut primary_tool = 0.0;
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
            "姿态：向左转向 8°"
        }
        7 => {
            orientation = axis_angle([0.0, 0.0, 1.0], ORIENTATION_RAD * (1.0 - amount));
            "姿态：向右转向回正"
        }
        8 => {
            orientation = axis_angle([-1.0, 0.0, 0.0], ORIENTATION_RAD * amount);
            "姿态：前部抬起 8°"
        }
        9 => {
            orientation = axis_angle([-1.0, 0.0, 0.0], ORIENTATION_RAD * (1.0 - amount));
            "姿态：前部往下回正"
        }
        10 => {
            orientation = axis_angle([0.0, 1.0, 0.0], ORIENTATION_RAD * amount);
            "姿态：向右侧倾 8°"
        }
        11 => {
            orientation = axis_angle([0.0, 1.0, 0.0], ORIENTATION_RAD * (1.0 - amount));
            "姿态：向左侧倾回正"
        }
        12 => {
            primary_tool = amount;
            "夹爪：连续闭合"
        }
        _ => {
            primary_tool = 1.0 - amount;
            "夹爪：连续张开"
        }
    };
    (position, orientation, name, primary_tool)
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
            (STARTUP_SAMPLES + PHASE_SAMPLES * 6, "姿态：向左转向 8°"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 7, "姿态：向右转向回正"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 8, "姿态：前部抬起 8°"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 9, "姿态：前部往下回正"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 10, "姿态：向右侧倾 8°"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 11, "姿态：向左侧倾回正"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 12, "夹爪：连续闭合"),
            (STARTUP_SAMPLES + PHASE_SAMPLES * 13, "夹爪：连续张开"),
        ];
        for (sample, expected) in endpoints {
            assert_eq!(sample_pose(sample).2, expected);
        }
        let (position, orientation, _, primary_tool) =
            sample_pose(STARTUP_SAMPLES + PHASE_SAMPLES * PHASE_COUNT);
        assert_eq!(position, [0.0, STARTUP_LIFT_M, 0.0]);
        assert_eq!(orientation, [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(primary_tool, 0.0);
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
        assert_components_close(
            sample_pose(cycle + PHASE_SAMPLES * 11).1,
            axis_angle([0.0, 1.0, 0.0], ORIENTATION_RAD),
        );
        assert_eq!(sample_pose(cycle + PHASE_SAMPLES * 12).3, 0.0);
        assert!(
            (sample_pose(cycle + PHASE_SAMPLES * 12 + PHASE_SAMPLES / 2).3 - 0.5).abs() < 1e-12
        );
        assert_eq!(sample_pose(cycle + PHASE_SAMPLES * 13).3, 1.0);
        assert!(
            (sample_pose(cycle + PHASE_SAMPLES * 13 + PHASE_SAMPLES / 2).3 - 0.5).abs() < 1e-12
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

            let roll = sample_pose(STARTUP_SAMPLES + PHASE_SAMPLES * 10 + sample).1;
            let roll_return = sample_pose(STARTUP_SAMPLES + PHASE_SAMPLES * 11 + reverse_sample).1;
            assert_components_close(roll, roll_return);
        }
    }

    #[test]
    fn playback_emits_one_declared_raw_sample_contract() {
        let mut playback = SimulationPlayback::default();
        let first = playback.sample(4, 100);
        let second = playback.sample(5, 200);
        assert_eq!(first.raw.sequence, 4);
        assert_eq!(first.raw.source_id, SOURCE_ID);
        assert!(first.raw.position_m.is_some());
        assert!(first.raw.orientation_xyzw.is_some());
        assert_eq!(
            first.raw.components,
            BTreeMap::from([
                (CONTROL_ACTIVE_COMPONENT.into(), 1.0),
                (PRIMARY_TOOL_COMPONENT.into(), 0.0),
            ])
        );
        assert_eq!(source_info().available_components.len(), 2);
        assert_eq!(
            source_info().available_components[0].path,
            CONTROL_ACTIVE_COMPONENT
        );
        assert_eq!(
            source_info().available_components[1].path,
            PRIMARY_TOOL_COMPONENT
        );
        assert_eq!(first.state.elapsed_s, Some(0.0));
        assert_eq!(second.state.elapsed_s, Some(0.01));
    }
}
