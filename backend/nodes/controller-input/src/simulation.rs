use std::collections::BTreeMap;

use robot_arm_messages::{
    ActionFeedback, DEFAULT_ACTION_ARC_RAD_PER_S, DEFAULT_ACTION_TRANSLATION_M_PER_S,
    INPUT_ACTIONS, InputComponentInfo, InputDriverInfo, InputSimulationItem, InputSimulationState,
    InputSourceInfo, SCHEMA_VERSION,
};

use super::RawSample;

pub(super) const DRIVER_ID: &str = "generated-test-input";
pub(super) const DEVICE_ID: &str = "generic-action-source";
pub(super) const SOURCE_ID: &str = "simulation:generic-action-source";
pub(super) const START_STOP_COMPONENT: &str = "action/start_stop";
const SAMPLE_RATE_HZ: u64 = 100;
const PHASE_SAMPLES: u64 = SAMPLE_RATE_HZ * 3;
const ORIENTATION_RAD: f64 = 8.0_f64.to_radians();
const LIFT_M: f64 = 0.05;

pub(super) struct SimulationPlayback {
    item: InputSimulationItem,
    sample_index: u64,
}

pub(super) struct SimulationSample {
    pub(super) raw: Option<RawSample>,
    pub(super) state: InputSimulationState,
    pub(super) feedback: Option<ActionFeedback>,
    pub(super) complete: bool,
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
        display_name: "通用动作测试源".into(),
        custom_name: None,
        vendor_id: None,
        product_id: None,
        serial: None,
        position_capable: false,
        orientation_capable: false,
        action_capable: true,
        active: true,
        available_components: INPUT_ACTIONS
            .iter()
            .map(|action| InputComponentInfo {
                path: format!("action/{}", action.key),
                action_type: action.action_type,
                localized_name: Some(action.label.into()),
                definition_source: DRIVER_ID.into(),
            })
            .collect(),
        available_feedback_capabilities: vec![],
        original_error: None,
    }
}

impl SimulationPlayback {
    pub(super) fn new(item: InputSimulationItem) -> Self {
        Self {
            item,
            sample_index: 0,
        }
    }

    pub(super) fn sample(&mut self, sequence: u64, now_ns: i64) -> SimulationSample {
        let generated = generate(self.item, self.sample_index);
        let elapsed_s = self.sample_index as f64 / SAMPLE_RATE_HZ as f64;
        self.sample_index += 1;
        SimulationSample {
            raw: Some(RawSample {
                sequence,
                source_time_ns: now_ns,
                received_time_ns: now_ns,
                source_id: SOURCE_ID.into(),
                position_m: None,
                orientation_xyzw: None,
                components: generated.components,
            }),
            state: InputSimulationState {
                schema_version: SCHEMA_VERSION,
                active: !generated.complete,
                item: Some(self.item),
                phase: Some(generated.phase.into()),
                elapsed_s: Some(elapsed_s),
            },
            feedback: generated.feedback.map(|strength_percent| ActionFeedback {
                schema_version: SCHEMA_VERSION,
                sequence,
                sample_time_ns: now_ns,
                action: "primary_tool".into(),
                strength_percent,
            }),
            complete: generated.complete,
        }
    }
}

struct GeneratedSample {
    components: BTreeMap<String, f64>,
    feedback: Option<f64>,
    phase: &'static str,
    complete: bool,
}

fn generate(item: InputSimulationItem, sample_index: u64) -> GeneratedSample {
    if item == InputSimulationItem::PrimaryToolFeedback {
        return feedback_sample(sample_index);
    }
    if sample_index <= PHASE_SAMPLES {
        return lift_sample(item, sample_index);
    }
    if item == InputSimulationItem::MoveUpDown {
        return GeneratedSample {
            components: BTreeMap::from([
                (START_STOP_COMPONENT.into(), 1.0),
                ("action/move_up_down".into(), 0.0),
            ]),
            feedback: None,
            phase: "complete",
            complete: true,
        };
    }
    let selected_index = sample_index - PHASE_SAMPLES - 1;
    let mut selected = selected_sample(item, selected_index);
    if selected_index == 0 {
        selected.components.insert(START_STOP_COMPONENT.into(), 0.0);
    }
    selected
}

fn selected_sample(item: InputSimulationItem, sample_index: u64) -> GeneratedSample {
    match item {
        InputSimulationItem::PrimaryToolFeedback => unreachable!("feedback has no arm prelude"),
        InputSimulationItem::StartStop => control_sample(sample_index, false),
        InputSimulationItem::EmergencyStop => control_sample(sample_index, true),
        InputSimulationItem::PrimaryToolOpen => open_tool_sample(sample_index),
        InputSimulationItem::PrimaryTool => continuous_tool_sample(sample_index),
        _ => continuous_motion_sample(item, sample_index),
    }
}

fn lift_sample(item: InputSimulationItem, sample_index: u64) -> GeneratedSample {
    let phase_seconds = PHASE_SAMPLES as f64 / SAMPLE_RATE_HZ as f64;
    let lift_amplitude = LIFT_M / (DEFAULT_ACTION_TRANSLATION_M_PER_S * phase_seconds);
    let lift = if sample_index == 0 {
        0.0
    } else {
        lift_amplitude * motion_profile(sample_index - 1)
    };
    let mut components = BTreeMap::from([
        (
            START_STOP_COMPONENT.into(),
            if sample_index == 0 { 1.0 } else { 0.0 },
        ),
        ("action/move_up_down".into(), lift),
    ]);
    if item == InputSimulationItem::PrimaryTool {
        components.insert("action/primary_tool".into(), 1.0);
    }
    GeneratedSample {
        components,
        feedback: None,
        phase: "lifting",
        complete: false,
    }
}

fn continuous_motion_sample(item: InputSimulationItem, sample_index: u64) -> GeneratedSample {
    let action_path = component_path(item).expect("motion item has a component");
    let amplitude = input_amplitude(item);
    let (value, phase, complete, start_stop) = if sample_index == 0 {
        (0.0, "starting", false, 1.0)
    } else if sample_index <= PHASE_SAMPLES {
        (
            amplitude * motion_profile(sample_index - 1),
            "outbound",
            false,
            0.0,
        )
    } else if sample_index <= PHASE_SAMPLES * 2 {
        (
            -amplitude * motion_profile(sample_index - PHASE_SAMPLES - 1),
            "return",
            false,
            0.0,
        )
    } else {
        (0.0, "complete", true, 1.0)
    };
    GeneratedSample {
        components: BTreeMap::from([
            (START_STOP_COMPONENT.into(), start_stop),
            (action_path.into(), value),
        ]),
        feedback: None,
        phase,
        complete,
    }
}

fn continuous_tool_sample(sample_index: u64) -> GeneratedSample {
    let (value, phase, complete, start_stop) = if sample_index == 0 {
        (1.0, "starting", false, 1.0)
    } else if sample_index <= PHASE_SAMPLES {
        (
            1.0 - smooth(progress(sample_index - 1)),
            "opening",
            false,
            0.0,
        )
    } else if sample_index <= PHASE_SAMPLES * 2 {
        (
            smooth(progress(sample_index - PHASE_SAMPLES - 1)),
            "closing",
            false,
            0.0,
        )
    } else {
        (1.0, "complete", true, 1.0)
    };
    GeneratedSample {
        components: BTreeMap::from([
            (START_STOP_COMPONENT.into(), start_stop),
            ("action/primary_tool".into(), value),
        ]),
        feedback: None,
        phase,
        complete,
    }
}

fn open_tool_sample(sample_index: u64) -> GeneratedSample {
    let (start_stop, open, phase, complete) = match sample_index {
        0 => (1.0, 0.0, "starting", false),
        1 => (0.0, 1.0, "opening", false),
        2 => (0.0, 0.0, "opened", false),
        _ => (1.0, 0.0, "complete", true),
    };
    GeneratedSample {
        components: BTreeMap::from([
            (START_STOP_COMPONENT.into(), start_stop),
            ("action/primary_tool_open".into(), open),
        ]),
        feedback: None,
        phase,
        complete,
    }
}

fn control_sample(sample_index: u64, emergency: bool) -> GeneratedSample {
    let complete = sample_index >= PHASE_SAMPLES;
    let start_stop = (sample_index == 0 || (complete && !emergency)) as u8 as f64;
    let emergency_stop = (complete && emergency) as u8 as f64;
    GeneratedSample {
        components: BTreeMap::from([
            (START_STOP_COMPONENT.into(), start_stop),
            ("action/emergency_stop".into(), emergency_stop),
        ]),
        feedback: None,
        phase: if complete { "complete" } else { "active" },
        complete,
    }
}

fn feedback_sample(sample_index: u64) -> GeneratedSample {
    let complete = sample_index > PHASE_SAMPLES * 2;
    let strength = if sample_index <= PHASE_SAMPLES {
        smooth(progress(sample_index)) * 100.0
    } else if sample_index <= PHASE_SAMPLES * 2 {
        (1.0 - smooth(progress(sample_index - PHASE_SAMPLES))) * 100.0
    } else {
        0.0
    };
    GeneratedSample {
        components: BTreeMap::new(),
        feedback: Some(strength),
        phase: if complete {
            "complete"
        } else if sample_index <= PHASE_SAMPLES {
            "increasing"
        } else {
            "decreasing"
        },
        complete,
    }
}

pub(super) fn component_path(item: InputSimulationItem) -> Option<&'static str> {
    Some(match item {
        InputSimulationItem::MoveForwardBack => "action/move_forward_back",
        InputSimulationItem::MoveLeftRight => "action/move_left_right",
        InputSimulationItem::MoveUpDown => "action/move_up_down",
        InputSimulationItem::ToolPitch => "action/tool_pitch",
        InputSimulationItem::ToolYaw => "action/tool_yaw",
        InputSimulationItem::ToolRoll => "action/tool_roll",
        InputSimulationItem::FrontPitch => "action/front_pitch",
        InputSimulationItem::HorizontalArc => "action/horizontal_arc",
        InputSimulationItem::PrimaryToolOpen => "action/primary_tool_open",
        InputSimulationItem::PrimaryTool => "action/primary_tool",
        InputSimulationItem::StartStop => START_STOP_COMPONENT,
        InputSimulationItem::EmergencyStop => "action/emergency_stop",
        InputSimulationItem::ToolAxisTranslation => "action/tool_axis_translation",
        InputSimulationItem::ToolHelicalMotion => "action/tool_helical_motion",
        InputSimulationItem::PrimaryToolFeedback => return None,
    })
}

pub(super) fn action_key(item: InputSimulationItem) -> Option<&'static str> {
    component_path(item).map(|path| path.trim_start_matches("action/"))
}

fn input_amplitude(item: InputSimulationItem) -> f64 {
    let phase_seconds = PHASE_SAMPLES as f64 / SAMPLE_RATE_HZ as f64;
    match item {
        InputSimulationItem::MoveLeftRight | InputSimulationItem::ToolAxisTranslation => {
            0.02 / (DEFAULT_ACTION_TRANSLATION_M_PER_S * phase_seconds)
        }
        InputSimulationItem::MoveForwardBack | InputSimulationItem::MoveUpDown => {
            0.05 / (DEFAULT_ACTION_TRANSLATION_M_PER_S * phase_seconds)
        }
        InputSimulationItem::ToolPitch
        | InputSimulationItem::ToolYaw
        | InputSimulationItem::ToolRoll
        | InputSimulationItem::FrontPitch
        | InputSimulationItem::HorizontalArc => {
            ORIENTATION_RAD / (DEFAULT_ACTION_ARC_RAD_PER_S * phase_seconds)
        }
        InputSimulationItem::ToolHelicalMotion => {
            0.02 / (DEFAULT_ACTION_TRANSLATION_M_PER_S * phase_seconds)
        }
        _ => 0.0,
    }
}

fn progress(sample_index: u64) -> f64 {
    sample_index.min(PHASE_SAMPLES) as f64 / PHASE_SAMPLES as f64
}

fn motion_profile(sample_index: u64) -> f64 {
    1.0 - (std::f64::consts::TAU * progress(sample_index)).cos()
}

fn smooth(progress: f64) -> f64 {
    0.5 - 0.5 * (std::f64::consts::PI * progress).cos()
}

#[cfg(test)]
mod tests {
    use super::*;

    const ITEMS: [InputSimulationItem; 15] = [
        InputSimulationItem::MoveForwardBack,
        InputSimulationItem::MoveLeftRight,
        InputSimulationItem::MoveUpDown,
        InputSimulationItem::ToolPitch,
        InputSimulationItem::ToolYaw,
        InputSimulationItem::ToolRoll,
        InputSimulationItem::FrontPitch,
        InputSimulationItem::HorizontalArc,
        InputSimulationItem::PrimaryToolOpen,
        InputSimulationItem::PrimaryTool,
        InputSimulationItem::StartStop,
        InputSimulationItem::EmergencyStop,
        InputSimulationItem::PrimaryToolFeedback,
        InputSimulationItem::ToolAxisTranslation,
        InputSimulationItem::ToolHelicalMotion,
    ];

    #[test]
    fn simulation_source_reports_exactly_the_fourteen_input_actions() {
        let source = source_info();
        assert!(!source.position_capable);
        assert!(!source.orientation_capable);
        assert_eq!(source.available_components.len(), 14);
        assert_eq!(
            source
                .available_components
                .iter()
                .map(|component| component.path.as_str())
                .collect::<Vec<_>>(),
            INPUT_ACTIONS
                .iter()
                .map(|action| format!("action/{}", action.key))
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn all_fifteen_items_generate_their_own_contract() {
        for item in ITEMS {
            let mut playback = SimulationPlayback::new(item);
            let first = playback.sample(1, 10);
            assert_eq!(first.state.item, Some(item));
            if item == InputSimulationItem::PrimaryToolFeedback {
                assert!(first.feedback.is_some());
            } else {
                assert!(component_path(item).is_some());
                assert!(first.feedback.is_none());
            }
        }
    }

    #[test]
    fn continuous_motion_has_a_smooth_opposite_return() {
        let item = InputSimulationItem::MoveLeftRight;
        for sample in [0, 30, 75, 150, 225, 300] {
            let outbound = selected_sample(item, sample + 1);
            let returned = selected_sample(item, PHASE_SAMPLES + sample + 1);
            let path = component_path(item).unwrap();
            assert!((outbound.components[path] + returned.components[path]).abs() < 1e-12);
        }
    }

    #[test]
    fn continuous_items_finish_with_a_stop_toggle() {
        let sample = selected_sample(
            InputSimulationItem::ToolHelicalMotion,
            PHASE_SAMPLES * 2 + 1,
        );
        assert!(sample.complete);
        assert_eq!(sample.components[START_STOP_COMPONENT], 1.0);
        assert_eq!(sample.components["action/tool_helical_motion"], 0.0);
    }

    #[test]
    fn every_input_demo_begins_with_a_five_centimeter_lift() {
        for item in ITEMS {
            if item == InputSimulationItem::PrimaryToolFeedback {
                continue;
            }
            let first = generate(item, 0);
            assert_eq!(first.phase, "lifting");
            assert_eq!(first.components[START_STOP_COMPONENT], 1.0);

            let lifted_distance_m = (1..=PHASE_SAMPLES)
                .map(|sample| generate(item, sample).components["action/move_up_down"])
                .sum::<f64>()
                * DEFAULT_ACTION_TRANSLATION_M_PER_S
                / SAMPLE_RATE_HZ as f64;
            assert!((lifted_distance_m - LIFT_M).abs() < 1e-12);

            let selected = generate(item, PHASE_SAMPLES + 1);
            assert_ne!(selected.phase, "lifting");
            if item == InputSimulationItem::MoveUpDown {
                assert!(selected.complete);
            } else {
                assert_eq!(selected.components[START_STOP_COMPONENT], 0.0);
            }
        }
    }

    #[test]
    fn vertical_translation_demo_is_only_the_five_centimeter_lift() {
        let completed = generate(InputSimulationItem::MoveUpDown, PHASE_SAMPLES + 1);
        assert!(completed.complete);
        assert_eq!(completed.phase, "complete");
        assert_eq!(completed.components["action/move_up_down"], 0.0);
    }

    #[test]
    fn feedback_runs_zero_to_one_hundred_to_zero() {
        assert_eq!(feedback_sample(0).feedback, Some(0.0));
        assert_eq!(feedback_sample(PHASE_SAMPLES).feedback, Some(100.0));
        assert_eq!(feedback_sample(PHASE_SAMPLES * 2).feedback, Some(0.0));
        assert!(feedback_sample(PHASE_SAMPLES * 2 + 1).complete);
    }
}
