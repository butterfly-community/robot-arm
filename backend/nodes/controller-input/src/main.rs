mod simulation;

use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    path::PathBuf,
    sync::mpsc::{self, Receiver, RecvTimeoutError, Sender},
    thread,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result, eyre};
use fusion_ahrs::{Ahrs, Offset, OffsetSettings};
use hidapi::HidApi;
use json_config_store::{load_or_default, save};
use nalgebra::Vector3;
use nolo_cv1::protocol::{REPORT_SIZE, SUPPORTED_DEVICES, decode_report};
use robot_arm_messages::{
    AbsolutePoseFrame, ActionBinding, ActionFeedback, ActionFeedbackBinding,
    ActionFeedbackBindingState, ActionType, ActuatorActions, ApplyInputBindingsRequest,
    BooleanActionSample, ControlInputFrame, FloatActionSample, InputBindingState,
    InputComponentInfo, InputDiscoveryState, InputDriverInfo, InputFeedbackCapabilityInfo,
    InputSimulationRequest, InputSimulationState, InputSourceInfo, InputStreamDiagnostics,
    PoseComponent, PoseFlags, RequestAction, RequestResult, SCHEMA_VERSION,
    SelectPoseSourceRequest, SelectedInputSourceState, ServiceState, from_arrow, to_arrow,
};
use sdl3::{
    event::Event as SdlEvent,
    gamepad::{Axis, Button, Gamepad},
    sensor::SensorType,
};
use serde::{Deserialize, Serialize};
use simulation::{
    DRIVER_ID as SIMULATION_DRIVER_ID, PRIMARY_TOOL_COMPONENT as SIMULATION_PRIMARY_TOOL_COMPONENT,
    SOURCE_ID as SIMULATION_SOURCE_ID, START_STOP_COMPONENT_A as SIMULATION_START_STOP_COMPONENT_A,
    START_STOP_COMPONENT_B as SIMULATION_START_STOP_COMPONENT_B, SimulationPlayback,
};

const NOLO_DRIVER_ID: &str = "nolo-cv1-hid";
const SDL_DRIVER_ID: &str = "sdl3-gamepad";
const CONFIG_SCHEMA_VERSION: u32 = 2;
const HAPTIC_DURATION_MS: u32 = 50;
const VIRTUAL_FEEDBACK_SOURCE_ID: &str = "virtual-feedback";
const VIRTUAL_FEEDBACK_CAPABILITY_PATH: &str = "feedback/virtual";
const BINDING_CALIBRATION_DURATION: Duration = Duration::from_secs(3);

const ACTIONS: [(&str, ActionType); 9] = [
    ("start_stop", ActionType::Boolean),
    ("emergency_stop", ActionType::Boolean),
    ("primary_tool_open", ActionType::Boolean),
    ("primary_tool", ActionType::Float),
    ("move_forward_back", ActionType::Float),
    ("move_left_right", ActionType::Float),
    ("move_up_down", ActionType::Float),
    ("front_pitch", ActionType::Float),
    ("horizontal_arc", ActionType::Float),
];

const SDL_AXES: [(Axis, &str, &str); 6] = [
    (Axis::LeftX, "axis/left_x", "左摇杆横向"),
    (Axis::LeftY, "axis/left_y", "左摇杆纵向"),
    (Axis::RightX, "axis/right_x", "右摇杆横向"),
    (Axis::RightY, "axis/right_y", "右摇杆纵向"),
    (Axis::TriggerLeft, "axis/left_trigger", "左扳机行程"),
    (Axis::TriggerRight, "axis/right_trigger", "右扳机行程"),
];

const SDL_BUTTONS: [(Button, &str, &str); 26] = [
    (Button::South, "button/south", "右侧按键组下方按钮"),
    (Button::East, "button/east", "右侧按键组右方按钮"),
    (Button::West, "button/west", "右侧按键组左方按钮"),
    (Button::North, "button/north", "右侧按键组上方按钮"),
    (Button::Back, "button/back", "返回按钮"),
    (Button::Guide, "button/guide", "主菜单按钮"),
    (Button::Start, "button/start", "开始按钮"),
    (Button::LeftStick, "button/left_stick", "左摇杆按下"),
    (Button::RightStick, "button/right_stick", "右摇杆按下"),
    (Button::LeftShoulder, "button/left_shoulder", "左肩键"),
    (Button::RightShoulder, "button/right_shoulder", "右肩键"),
    (Button::DPadUp, "button/dpad_up", "方向键上"),
    (Button::DPadDown, "button/dpad_down", "方向键下"),
    (Button::DPadLeft, "button/dpad_left", "方向键左"),
    (Button::DPadRight, "button/dpad_right", "方向键右"),
    (Button::Misc1, "button/misc1", "附加按钮 1"),
    (Button::Misc2, "button/misc2", "附加按钮 2"),
    (Button::Misc3, "button/misc3", "附加按钮 3"),
    (Button::Misc4, "button/misc4", "附加按钮 4"),
    (Button::Misc5, "button/misc5", "附加按钮 5"),
    (Button::Misc6, "button/misc6", "附加按钮 6"),
    (Button::LeftPaddle1, "button/left_paddle1", "左背键 1"),
    (Button::RightPaddle1, "button/right_paddle1", "右背键 1"),
    (Button::LeftPaddle2, "button/left_paddle2", "左背键 2"),
    (Button::RightPaddle2, "button/right_paddle2", "右背键 2"),
    (Button::Touchpad, "button/touchpad", "触摸板按下"),
];

const NOLO_BUTTONS: [(u8, &str, &str); 6] = [
    (0, "button/touchpad", "触摸板按下"),
    (1, "button/trigger", "扳机"),
    (2, "button/menu", "菜单键"),
    (3, "button/system", "系统键"),
    (4, "button/grip", "侧握键"),
    (5, "button/touchpad_touch", "正在触摸触摸板"),
];

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let (driver_tx, driver_rx) = mpsc::channel();
    let (haptic_tx, haptic_rx) = mpsc::channel();
    spawn_nolo_driver(driver_tx.clone());
    spawn_sdl_driver(driver_tx, haptic_rx);

    let mut input = ControllerInput::load(driver_rx, haptic_tx)?;
    publish_snapshot(&mut node, &mut input)?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => match id.as_str() {
                "tick" => {
                    if let Some((pose, control)) = input.tick() {
                        if let Some(pose) = pose {
                            send(&mut node, "absolute_pose", &pose)?;
                        }
                        send(&mut node, "control_input", &control)?;
                    }
                    publish_discovery(&mut node, &input)?;
                }
                "snapshot" => publish_snapshot(&mut node, &mut input)?,
                "select_pose_source" => {
                    let request: SelectPoseSourceRequest =
                        from_arrow(data.as_array()).context("decode select_pose_source")?;
                    let result = input.select_pose_source(request);
                    send(&mut node, "pose_source_request_result", &result)?;
                    publish_snapshot(&mut node, &mut input)?;
                }
                "apply_bindings" => {
                    let request: ApplyInputBindingsRequest =
                        from_arrow(data.as_array()).context("decode apply_bindings")?;
                    let result = input.apply_bindings(request);
                    send(&mut node, "bindings_request_result", &result)?;
                    publish_snapshot(&mut node, &mut input)?;
                }
                "set_simulation" => {
                    let request: InputSimulationRequest =
                        from_arrow(data.as_array()).context("decode set_simulation")?;
                    let result = input.set_simulation(request);
                    send(&mut node, "simulation_request_result", &result)?;
                    publish_snapshot(&mut node, &mut input)?;
                }
                "action_feedback" => {
                    let feedback: ActionFeedback =
                        from_arrow(data.as_array()).context("decode action_feedback")?;
                    input.apply_feedback(feedback);
                }
                _ => {}
            },
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}

#[derive(Clone, Debug)]
struct RawSample {
    sequence: u64,
    source_time_ns: i64,
    received_time_ns: i64,
    source_id: String,
    position_m: Option<[f64; 3]>,
    orientation_xyzw: Option<[f64; 4]>,
    components: BTreeMap<String, f64>,
}

enum DriverEvent {
    Sources {
        driver: InputDriverInfo,
        sources: Vec<InputSourceInfo>,
    },
    Sample(RawSample),
    Error {
        driver_id: String,
        message: String,
    },
}

struct HapticCommand {
    source_id: String,
    capability_path: String,
    intensity: u16,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct PoseSourceSelection {
    driver_id: String,
    device_id: String,
    source_id: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct InputConfig {
    #[serde(default = "config_schema_version")]
    schema_version: u32,
    position_source: Option<PoseSourceSelection>,
    orientation_source: Option<PoseSourceSelection>,
    bindings: Vec<ActionBinding>,
    feedback_bindings: Vec<ActionFeedbackBinding>,
    #[serde(default)]
    component_offsets: BTreeMap<String, BTreeMap<String, f64>>,
    config_version: u64,
}

const fn config_schema_version() -> u32 {
    CONFIG_SCHEMA_VERSION
}

impl Default for InputConfig {
    fn default() -> Self {
        Self {
            schema_version: CONFIG_SCHEMA_VERSION,
            position_source: None,
            orientation_source: None,
            bindings: vec![],
            feedback_bindings: vec![],
            component_offsets: BTreeMap::new(),
            config_version: 1,
        }
    }
}

struct ControllerInput {
    config_path: PathBuf,
    config: InputConfig,
    drivers: BTreeMap<String, InputDriverInfo>,
    sources: BTreeMap<String, InputSourceInfo>,
    diagnostics: BTreeMap<String, InputStreamDiagnostics>,
    samples: BTreeMap<String, RawSample>,
    last_published_sequences: BTreeMap<String, u64>,
    pose_dirty: bool,
    previous_control: Option<ControlInputFrame>,
    simulation: Option<SimulationPlayback>,
    config_before_simulation: Option<InputConfig>,
    simulation_state: InputSimulationState,
    next_sequence: u64,
    receiver: Receiver<DriverEvent>,
    haptic: Sender<HapticCommand>,
    virtual_feedback: Option<ActionFeedback>,
    last_error: Option<String>,
}

impl ControllerInput {
    fn load(receiver: Receiver<DriverEvent>, haptic: Sender<HapticCommand>) -> Result<Self> {
        let config_path = std::env::var("CONTROLLER_INPUT_CONFIG")
            .map(PathBuf::from)
            .unwrap_or_else(|_| PathBuf::from("/config/controller-input.json"));
        let config: InputConfig = load_or_default(&config_path)?;
        if config.schema_version != CONFIG_SCHEMA_VERSION {
            return Err(eyre!("不支持的输入配置版本 {}", config.schema_version));
        }
        Ok(Self {
            config_path,
            config,
            drivers: BTreeMap::new(),
            sources: BTreeMap::new(),
            diagnostics: BTreeMap::new(),
            samples: BTreeMap::new(),
            last_published_sequences: BTreeMap::new(),
            pose_dirty: true,
            previous_control: None,
            simulation: None,
            config_before_simulation: None,
            simulation_state: InputSimulationState {
                schema_version: SCHEMA_VERSION,
                ..Default::default()
            },
            next_sequence: 1,
            receiver,
            haptic,
            virtual_feedback: None,
            last_error: None,
        })
    }

    fn commit_config(&mut self, config: InputConfig) -> Result<()> {
        save(&self.config_path, &config)?;
        self.config = config;
        self.last_published_sequences.clear();
        self.pose_dirty = true;
        self.previous_control = None;
        if self.virtual_feedback.as_ref().is_some_and(|feedback| {
            !self
                .config
                .feedback_bindings
                .iter()
                .any(|binding| binding.action == feedback.action && is_virtual_feedback(binding))
        }) {
            self.virtual_feedback = None;
        }
        Ok(())
    }

    fn drain(&mut self) {
        while let Ok(event) = self.receiver.try_recv() {
            self.accept_driver_event(event);
        }
    }

    fn accept_driver_event(&mut self, event: DriverEvent) {
        match event {
            DriverEvent::Sources { driver, sources } => {
                self.replace_driver_sources(driver, sources);
            }
            DriverEvent::Sample(sample) => self.accept_sample(sample),
            DriverEvent::Error { driver_id, message } => {
                self.last_error = Some(message.clone());
                self.drivers
                    .entry(driver_id.clone())
                    .and_modify(|driver| driver.original_error = Some(message.clone()))
                    .or_insert(InputDriverInfo {
                        driver_id,
                        display_name: "Input driver".into(),
                        version: env!("CARGO_PKG_VERSION").into(),
                        original_error: Some(message),
                    });
            }
        }
    }

    fn replace_driver_sources(&mut self, driver: InputDriverInfo, sources: Vec<InputSourceInfo>) {
        let driver_id = driver.driver_id.clone();
        self.drivers.insert(driver_id.clone(), driver);
        self.sources
            .retain(|_, source| source.driver_id != driver_id);
        for source in sources {
            self.diagnostics
                .entry(source.source_id.clone())
                .or_insert_with(|| InputStreamDiagnostics {
                    source_id: source.source_id.clone(),
                    ..Default::default()
                });
            self.sources.insert(source.source_id.clone(), source);
        }
        self.samples
            .retain(|source_id, _| self.sources.contains_key(source_id));
        self.last_published_sequences
            .retain(|source_id, _| self.sources.contains_key(source_id));
        self.pose_dirty = true;
    }

    fn accept_sample(&mut self, sample: RawSample) {
        let diagnostics = self
            .diagnostics
            .entry(sample.source_id.clone())
            .or_insert_with(|| InputStreamDiagnostics {
                source_id: sample.source_id.clone(),
                ..Default::default()
            });
        diagnostics.received_frames += 1;
        diagnostics.last_source_time_ns = Some(sample.source_time_ns);
        diagnostics.last_received_time_ns = Some(sample.received_time_ns);
        if let Some(previous) = self.samples.get(&sample.source_id) {
            let elapsed = sample.source_time_ns - previous.source_time_ns;
            if elapsed > 0 {
                diagnostics.observed_rate_hz = Some(1_000_000_000.0 / elapsed as f64);
            }
            diagnostics.sequence_gaps += sample.sequence.saturating_sub(previous.sequence + 1);
        }
        self.samples.insert(sample.source_id.clone(), sample);
    }

    fn tick(&mut self) -> Option<(Option<AbsolutePoseFrame>, ControlInputFrame)> {
        self.drain();
        let now = now_ns();
        if let Some(simulation) = self.simulation.as_mut() {
            let sample = simulation.sample(self.next_sequence, now);
            self.simulation_state = sample.state;
            self.accept_sample(sample.raw);
        }

        let relevant = |source_id: &str| {
            self.config
                .position_source
                .as_ref()
                .is_some_and(|source| source.source_id == source_id)
                || self
                    .config
                    .orientation_source
                    .as_ref()
                    .is_some_and(|source| source.source_id == source_id)
                || self
                    .config
                    .bindings
                    .iter()
                    .any(|binding| binding.source_id == source_id)
        };
        let changed = self.samples.iter().any(|(source_id, sample)| {
            relevant(source_id)
                && self.last_published_sequences.get(source_id) != Some(&sample.sequence)
        });
        if !changed && !self.pose_dirty {
            return None;
        }
        let pose_changed = [
            &self.config.position_source,
            &self.config.orientation_source,
        ]
        .into_iter()
        .flatten()
        .any(|selected| {
            self.samples.get(&selected.source_id).is_some_and(|sample| {
                self.last_published_sequences.get(&selected.source_id) != Some(&sample.sequence)
            })
        });
        let pose = (pose_changed || self.pose_dirty).then(|| {
            combined_pose_frame(
                self.config.position_source.as_ref(),
                self.config.orientation_source.as_ref(),
                &self.sources,
                &self.samples,
                self.next_sequence,
                now,
            )
        });
        self.pose_dirty = false;
        for (source_id, sample) in &self.samples {
            self.last_published_sequences
                .insert(source_id.clone(), sample.sequence);
        }
        let control = evaluate_actions(
            &self.samples,
            &self.config.bindings,
            &self.config.component_offsets,
            self.previous_control.as_ref(),
            self.next_sequence,
            now,
        );
        self.next_sequence += 1;
        self.previous_control = Some(control.clone());
        Some((pose, control))
    }

    fn select_pose_source(
        &mut self,
        request: SelectPoseSourceRequest,
    ) -> RequestResult<SelectedInputSourceState> {
        let selection = self
            .sources
            .get(&request.source_id)
            .filter(|source| {
                source.driver_id == request.driver_id && source.device_id == request.device_id
            })
            .filter(|source| supports_pose_component(source, request.component))
            .map(|source| PoseSourceSelection {
                driver_id: source.driver_id.clone(),
                device_id: source.device_id.clone(),
                source_id: source.source_id.clone(),
            });
        let change = match request.action {
            RequestAction::Select => selection
                .ok_or_else(|| "选择的输入 source 不存在或不提供对应位姿能力".to_owned())
                .and_then(|selection| {
                    let mut config = self.config.clone();
                    match request.component {
                        PoseComponent::Position => config.position_source = Some(selection),
                        PoseComponent::Orientation => config.orientation_source = Some(selection),
                    }
                    config.config_version += 1;
                    self.commit_config(config)
                        .map_err(|error| error.to_string())
                }),
            RequestAction::Unselect => {
                let mut config = self.config.clone();
                match request.component {
                    PoseComponent::Position => config.position_source = None,
                    PoseComponent::Orientation => config.orientation_source = None,
                }
                config.config_version += 1;
                self.commit_config(config)
                    .map_err(|error| error.to_string())
            }
            action => Err(format!("input 节点不处理 {action:?} pose source 请求")),
        };
        let value = match request.component {
            PoseComponent::Position => self.source_state(self.config.position_source.as_ref()),
            PoseComponent::Orientation => {
                self.source_state(self.config.orientation_source.as_ref())
            }
        };
        request_result(request, value, change.err())
    }

    fn apply_bindings(
        &mut self,
        request: ApplyInputBindingsRequest,
    ) -> RequestResult<Vec<InputBindingState>> {
        let error = validate_bindings(&request.bindings)
            .err()
            .map(|error| error.to_string());
        if error.is_none() {
            let component_offsets = if self.config.bindings == request.bindings {
                self.config.component_offsets.clone()
            } else {
                self.detect_component_offsets(&request.bindings)
            };
            let mut config = self.config.clone();
            config.bindings = request.bindings.clone();
            config.feedback_bindings = request.feedback_bindings.clone();
            config.component_offsets = component_offsets;
            config.config_version += 1;
            if let Err(save_error) = self.commit_config(config) {
                return RequestResult {
                    schema_version: SCHEMA_VERSION,
                    request_id: request.request_id,
                    acknowledged_action: RequestAction::Apply,
                    value: Some(self.binding_states()),
                    original_error: Some(save_error.to_string()),
                };
            }
        }
        RequestResult {
            schema_version: SCHEMA_VERSION,
            request_id: request.request_id,
            acknowledged_action: RequestAction::Apply,
            value: Some(self.binding_states()),
            original_error: error,
        }
    }

    fn detect_component_offsets(
        &mut self,
        bindings: &[ActionBinding],
    ) -> BTreeMap<String, BTreeMap<String, f64>> {
        let targets = bindings
            .iter()
            .filter(|binding| binding.action_type == ActionType::Float)
            .flat_map(|binding| {
                binding.component_paths.iter().filter_map(|component| {
                    self.sources
                        .get(&binding.source_id)
                        .and_then(|source| {
                            source
                                .available_components
                                .iter()
                                .find(|available| available.path == *component)
                        })
                        .filter(|available| available.action_type == ActionType::Float)
                        .map(|_| (binding.source_id.clone(), component.clone()))
                })
            })
            .collect::<BTreeSet<_>>();
        if targets.is_empty() {
            return BTreeMap::new();
        }

        let started = Instant::now();
        let mut totals = BTreeMap::<(String, String), (f64, u64)>::new();
        while let Some(remaining) = BINDING_CALIBRATION_DURATION.checked_sub(started.elapsed()) {
            match self.receiver.recv_timeout(remaining) {
                Ok(event) => {
                    if let DriverEvent::Sample(sample) = &event {
                        for (source_id, component) in targets
                            .iter()
                            .filter(|(source_id, _)| *source_id == sample.source_id)
                        {
                            if let Some(value) = sample.components.get(component) {
                                let total = totals
                                    .entry((source_id.clone(), component.clone()))
                                    .or_default();
                                total.0 += value;
                                total.1 += 1;
                            }
                        }
                    }
                    self.accept_driver_event(event);
                }
                Err(RecvTimeoutError::Timeout | RecvTimeoutError::Disconnected) => break,
            }
        }

        let mut offsets = BTreeMap::<String, BTreeMap<String, f64>>::new();
        for ((source_id, component), (sum, count)) in totals {
            if count > 0 {
                offsets
                    .entry(source_id)
                    .or_default()
                    .insert(component, sum / count as f64);
            }
        }
        offsets
    }

    fn set_simulation(
        &mut self,
        request: InputSimulationRequest,
    ) -> RequestResult<InputSimulationState> {
        if request.enabled && self.simulation.is_none() {
            self.config_before_simulation = Some(self.config.clone());
            self.config = simulation_config(&self.config);
            self.replace_driver_sources(simulation::driver_info(), vec![simulation::source_info()]);
            self.simulation = Some(SimulationPlayback::default());
            self.simulation_state = InputSimulationState {
                schema_version: SCHEMA_VERSION,
                active: true,
                phase: Some("starting".into()),
                elapsed_s: Some(0.0),
            };
            self.last_published_sequences.clear();
            self.previous_control = None;
            self.virtual_feedback = None;
        } else if !request.enabled && self.simulation.take().is_some() {
            self.replace_driver_sources(simulation::driver_info(), vec![]);
            if let Some(config) = self.config_before_simulation.take() {
                self.config = config;
            }
            self.simulation_state = InputSimulationState {
                schema_version: SCHEMA_VERSION,
                ..Default::default()
            };
            self.last_published_sequences.clear();
            self.pose_dirty = true;
            self.previous_control = None;
            if !self
                .config
                .feedback_bindings
                .iter()
                .any(is_virtual_feedback)
            {
                self.virtual_feedback = None;
            }
        }
        RequestResult {
            schema_version: SCHEMA_VERSION,
            request_id: request.request_id,
            acknowledged_action: RequestAction::Apply,
            value: Some(self.simulation_state.clone()),
            original_error: None,
        }
    }

    fn apply_feedback(&mut self, feedback: ActionFeedback) {
        if self
            .config
            .feedback_bindings
            .iter()
            .any(|binding| binding.action == feedback.action && is_virtual_feedback(binding))
        {
            self.virtual_feedback = Some(feedback.clone());
        }
        for binding in
            self.config.feedback_bindings.iter().filter(|binding| {
                binding.action == feedback.action && !is_virtual_feedback(binding)
            })
        {
            let _ = self.haptic.send(HapticCommand {
                source_id: binding.source_id.clone(),
                capability_path: binding.capability_path.clone(),
                intensity: haptic_intensity(feedback.strength_percent),
            });
        }
    }

    fn binding_states(&self) -> Vec<InputBindingState> {
        ACTIONS
            .iter()
            .map(|(action, action_type)| {
                let binding = self
                    .config
                    .bindings
                    .iter()
                    .find(|value| value.action == *action);
                let sample = binding.and_then(|value| self.samples.get(&value.source_id));
                let components = binding
                    .map(|value| value.component_paths.clone())
                    .unwrap_or_default();
                let applicable = binding.is_some()
                    && sample.is_some_and(|value| {
                        components
                            .iter()
                            .all(|component| value.components.contains_key(component))
                    });
                let value = binding
                    .and_then(|binding| {
                        sample.map(|sample| {
                            binding_value(sample, binding, &self.config.component_offsets)
                        })
                    })
                    .unwrap_or(0.0);
                InputBindingState {
                    action: (*action).into(),
                    action_type: *action_type,
                    source_id: binding.map(|value| value.source_id.clone()),
                    invert: binding.is_some_and(|value| value.invert),
                    configured_components: components,
                    active: binding.is_some() && sample.is_some(),
                    value,
                    applicable,
                    original_error: None,
                }
            })
            .collect()
    }

    fn feedback_binding_states(&self) -> Vec<ActionFeedbackBindingState> {
        self.config
            .feedback_bindings
            .iter()
            .map(|binding| {
                let applicable = is_virtual_feedback(binding)
                    || self.sources.get(&binding.source_id).is_some_and(|source| {
                        source
                            .available_feedback_capabilities
                            .iter()
                            .any(|capability| capability.path == binding.capability_path)
                    });
                ActionFeedbackBindingState {
                    action: binding.action.clone(),
                    source_id: Some(binding.source_id.clone()),
                    capability_path: Some(binding.capability_path.clone()),
                    applicable,
                }
            })
            .collect()
    }

    fn source_state(
        &self,
        selection: Option<&PoseSourceSelection>,
    ) -> Option<SelectedInputSourceState> {
        selection.map(|selected| {
            let active = self.sources.get(&selected.source_id).is_some_and(|source| {
                source.active
                    && source.driver_id == selected.driver_id
                    && source.device_id == selected.device_id
            });
            SelectedInputSourceState {
                driver_id: selected.driver_id.clone(),
                device_id: selected.device_id.clone(),
                source_id: selected.source_id.clone(),
                active,
            }
        })
    }

    fn discovery_state(&self) -> InputDiscoveryState {
        InputDiscoveryState {
            schema_version: SCHEMA_VERSION,
            drivers: self.drivers.values().cloned().collect(),
            sources: self.sources.values().cloned().collect(),
            position_source_id: self
                .config
                .position_source
                .as_ref()
                .map(|value| value.source_id.clone()),
            position_source: self.source_state(self.config.position_source.as_ref()),
            orientation_source_id: self
                .config
                .orientation_source
                .as_ref()
                .map(|value| value.source_id.clone()),
            orientation_source: self.source_state(self.config.orientation_source.as_ref()),
            bindings: self.binding_states(),
            feedback_bindings: self.feedback_binding_states(),
            live_component_values: live_component_values(&self.samples),
            virtual_feedback: self.virtual_feedback.clone(),
            diagnostics: self.diagnostics.values().cloned().collect(),
            simulation: self.simulation_state.clone(),
            service: self.service_state(),
        }
    }

    fn service_state(&self) -> ServiceState {
        ServiceState {
            schema_version: SCHEMA_VERSION,
            build_version: env!("CARGO_PKG_VERSION").into(),
            config_version: self.config.config_version,
            running: true,
            has_input: !self.sources.is_empty() || self.simulation.is_some(),
            has_output: self.previous_control.is_some(),
            last_error: self.last_error.clone(),
            updated_at_ns: now_ns(),
        }
    }
}

fn simulation_config(config: &InputConfig) -> InputConfig {
    let mut config = config.clone();
    let selection = PoseSourceSelection {
        driver_id: SIMULATION_DRIVER_ID.into(),
        device_id: simulation::DEVICE_ID.into(),
        source_id: SIMULATION_SOURCE_ID.into(),
    };
    config.position_source = Some(selection.clone());
    config.orientation_source = Some(selection);
    config.bindings = vec![
        ActionBinding {
            action: "start_stop".into(),
            action_type: ActionType::Boolean,
            source_id: SIMULATION_SOURCE_ID.into(),
            component_paths: vec![
                SIMULATION_START_STOP_COMPONENT_A.into(),
                SIMULATION_START_STOP_COMPONENT_B.into(),
            ],
            invert: false,
        },
        ActionBinding {
            action: "primary_tool".into(),
            action_type: ActionType::Float,
            source_id: SIMULATION_SOURCE_ID.into(),
            component_paths: vec![SIMULATION_PRIMARY_TOOL_COMPONENT.into()],
            invert: false,
        },
    ];
    config.feedback_bindings = vec![ActionFeedbackBinding {
        action: "primary_tool".into(),
        source_id: VIRTUAL_FEEDBACK_SOURCE_ID.into(),
        capability_path: VIRTUAL_FEEDBACK_CAPABILITY_PATH.into(),
    }];
    config
}

fn is_virtual_feedback(binding: &ActionFeedbackBinding) -> bool {
    binding.source_id == VIRTUAL_FEEDBACK_SOURCE_ID
        && binding.capability_path == VIRTUAL_FEEDBACK_CAPABILITY_PATH
}

fn supports_pose_component(source: &InputSourceInfo, component: PoseComponent) -> bool {
    match component {
        PoseComponent::Position => source.position_capable,
        PoseComponent::Orientation => source.orientation_capable,
    }
}

fn request_result(
    request: SelectPoseSourceRequest,
    value: Option<SelectedInputSourceState>,
    error: Option<String>,
) -> RequestResult<SelectedInputSourceState> {
    RequestResult {
        schema_version: SCHEMA_VERSION,
        request_id: request.request_id,
        acknowledged_action: request.action,
        value,
        original_error: error,
    }
}

fn validate_bindings(bindings: &[ActionBinding]) -> Result<()> {
    for binding in bindings {
        let expected = ACTIONS
            .iter()
            .find(|(action, _)| *action == binding.action)
            .ok_or_else(|| eyre!("未知 Action {}", binding.action))?
            .1;
        if binding.action_type != expected {
            return Err(eyre!("Action {} 类型不匹配", binding.action));
        }
        if binding.action == "start_stop" && binding.component_paths.len() != 2 {
            return Err(eyre!("启动和停止控制必须绑定两个按钮"));
        }
    }
    Ok(())
}

fn live_component_values(
    samples: &BTreeMap<String, RawSample>,
) -> BTreeMap<String, BTreeMap<String, f64>> {
    samples
        .iter()
        .map(|(source_id, sample)| (source_id.clone(), sample.components.clone()))
        .collect()
}

fn combined_pose_frame(
    position_source: Option<&PoseSourceSelection>,
    orientation_source: Option<&PoseSourceSelection>,
    sources: &BTreeMap<String, InputSourceInfo>,
    samples: &BTreeMap<String, RawSample>,
    sequence: u64,
    now: i64,
) -> AbsolutePoseFrame {
    let position_sample = position_source.and_then(|source| samples.get(&source.source_id));
    let orientation_sample = orientation_source.and_then(|source| samples.get(&source.source_id));
    let source_time_ns = [position_sample, orientation_sample]
        .into_iter()
        .flatten()
        .map(|sample| sample.source_time_ns)
        .max()
        .unwrap_or(now);
    let received_time_ns = [position_sample, orientation_sample]
        .into_iter()
        .flatten()
        .map(|sample| sample.received_time_ns)
        .max()
        .unwrap_or(now);
    let position_m = position_sample.and_then(|sample| sample.position_m);
    let orientation_xyzw = orientation_sample.and_then(|sample| sample.orientation_xyzw);
    AbsolutePoseFrame {
        schema_version: SCHEMA_VERSION,
        sequence,
        source_time_ns,
        received_time_ns,
        position_source_id: position_source.map(|source| source.source_id.clone()),
        orientation_source_id: orientation_source.map(|source| source.source_id.clone()),
        position_source_capable: position_source
            .and_then(|source| sources.get(&source.source_id))
            .is_some_and(|source| source.position_capable),
        orientation_source_capable: orientation_source
            .and_then(|source| sources.get(&source.source_id))
            .is_some_and(|source| source.orientation_capable),
        reference_space: "local".into(),
        position_m: position_m.unwrap_or([0.0; 3]),
        orientation_xyzw: orientation_xyzw.unwrap_or([0.0, 0.0, 0.0, 1.0]),
        flags: PoseFlags {
            position_valid: position_m.is_some(),
            position_tracked: position_m.is_some(),
            orientation_valid: orientation_xyzw.is_some(),
            orientation_tracked: orientation_xyzw.is_some(),
        },
    }
}

fn evaluate_actions(
    samples: &BTreeMap<String, RawSample>,
    bindings: &[ActionBinding],
    component_offsets: &BTreeMap<String, BTreeMap<String, f64>>,
    previous: Option<&ControlInputFrame>,
    sequence: u64,
    now: i64,
) -> ControlInputFrame {
    let binding = |action: &str| bindings.iter().find(|binding| binding.action == action);
    let value = |action: &str| {
        binding(action)
            .and_then(|binding| {
                samples
                    .get(&binding.source_id)
                    .map(|sample| binding_value(sample, binding, component_offsets))
            })
            .unwrap_or(0.0)
    };
    let active = |action: &str| {
        binding(action).is_some_and(|binding| samples.contains_key(&binding.source_id))
    };
    let boolean = |action: &str, previous_value: bool| {
        let current = value(action) != 0.0;
        BooleanActionSample {
            is_active: active(action),
            changed_since_last_sync: current != previous_value,
            value: current,
        }
    };
    let float = |action: &str, previous_value: f64| {
        let current = value(action);
        FloatActionSample {
            is_active: active(action),
            changed_since_last_sync: current != previous_value,
            value: current,
        }
    };
    ControlInputFrame {
        schema_version: SCHEMA_VERSION,
        sequence,
        source_time_ns: now,
        received_time_ns: now,
        start_stop: boolean(
            "start_stop",
            previous.is_some_and(|frame| frame.start_stop.value),
        ),
        emergency_stop: boolean(
            "emergency_stop",
            previous.is_some_and(|frame| frame.emergency_stop.value),
        ),
        actuator_actions: ActuatorActions {
            primary_tool_open: boolean(
                "primary_tool_open",
                previous.is_some_and(|frame| frame.actuator_actions.primary_tool_open.value),
            ),
            primary_tool: float(
                "primary_tool",
                previous
                    .map(|frame| frame.actuator_actions.primary_tool.value)
                    .unwrap_or(0.0),
            ),
        },
        move_forward_back: float(
            "move_forward_back",
            previous
                .map(|frame| frame.move_forward_back.value)
                .unwrap_or(0.0),
        ),
        move_left_right: float(
            "move_left_right",
            previous
                .map(|frame| frame.move_left_right.value)
                .unwrap_or(0.0),
        ),
        move_up_down: float(
            "move_up_down",
            previous
                .map(|frame| frame.move_up_down.value)
                .unwrap_or(0.0),
        ),
        front_pitch: float(
            "front_pitch",
            previous.map(|frame| frame.front_pitch.value).unwrap_or(0.0),
        ),
        horizontal_arc: float(
            "horizontal_arc",
            previous
                .map(|frame| frame.horizontal_arc.value)
                .unwrap_or(0.0),
        ),
    }
}

fn binding_value(
    sample: &RawSample,
    binding: &ActionBinding,
    component_offsets: &BTreeMap<String, BTreeMap<String, f64>>,
) -> f64 {
    let offset = |component: &str| {
        component_offsets
            .get(&binding.source_id)
            .and_then(|offsets| offsets.get(component))
            .copied()
            .unwrap_or(0.0)
    };
    let mut value =
        if binding.action_type == ActionType::Boolean {
            (!binding.component_paths.is_empty()
                && binding.component_paths.iter().all(|component| {
                    sample.components.get(component).copied().unwrap_or(0.0) != 0.0
                })) as u8 as f64
        } else {
            match binding.component_paths.as_slice() {
                [component] => {
                    sample.components.get(component).copied().unwrap_or(0.0) - offset(component)
                }
                [negative, positive] => {
                    sample.components.get(positive).copied().unwrap_or(0.0)
                        - offset(positive)
                        - sample.components.get(negative).copied().unwrap_or(0.0)
                        + offset(negative)
                }
                _ => 0.0,
            }
        };
    if binding.invert {
        value = -value;
    }
    value
}

struct ImuFusion {
    ahrs: Ahrs,
    offset: Offset,
    previous_time_ns: Option<u64>,
    nominal_rate_hz: f32,
}

impl ImuFusion {
    fn new(nominal_rate_hz: f32) -> Self {
        Self {
            ahrs: Ahrs::new(),
            offset: Offset::new(OffsetSettings::default(), nominal_rate_hz),
            previous_time_ns: None,
            nominal_rate_hz,
        }
    }

    fn update(
        &mut self,
        gyroscope_degrees_s: [f32; 3],
        accelerometer_g: [f32; 3],
        time_ns: u64,
    ) -> [f64; 4] {
        let delta = self
            .previous_time_ns
            .replace(time_ns)
            .map(|previous| time_ns.saturating_sub(previous) as f32 / 1_000_000_000.0)
            .filter(|value| *value > 0.0)
            .unwrap_or(1.0 / self.nominal_rate_hz);
        let gyro = self.offset.update(Vector3::from(gyroscope_degrees_s));
        self.ahrs
            .update_no_magnetometer(gyro, Vector3::from(accelerometer_g), delta);
        let quaternion = self.ahrs.quaternion();
        let value = quaternion.quaternion();
        [
            f64::from(value.i),
            f64::from(value.j),
            f64::from(value.k),
            f64::from(value.w),
        ]
    }
}

fn spawn_nolo_driver(sender: Sender<DriverEvent>) {
    thread::spawn(move || {
        if let Err(error) = run_nolo_driver(&sender) {
            let _ = sender.send(DriverEvent::Error {
                driver_id: NOLO_DRIVER_ID.into(),
                message: error.to_string(),
            });
        }
    });
}

fn run_nolo_driver(sender: &Sender<DriverEvent>) -> Result<()> {
    let mut api = HidApi::new()?;
    loop {
        api.refresh_devices()?;
        let Some(info) = api
            .device_list()
            .find(|info| SUPPORTED_DEVICES.contains(&(info.vendor_id(), info.product_id())))
        else {
            sender.send(DriverEvent::Sources {
                driver: driver_info(NOLO_DRIVER_ID, "NOLO CV1 direct HID", None),
                sources: vec![],
            })?;
            thread::sleep(Duration::from_secs(1));
            continue;
        };
        let device_id = format!(
            "{:04x}:{:04x}:{}",
            info.vendor_id(),
            info.product_id(),
            info.serial_number().unwrap_or("receiver")
        );
        let sources = (0..=1)
            .map(|controller| nolo_source(info, &device_id, controller))
            .collect();
        sender.send(DriverEvent::Sources {
            driver: driver_info(NOLO_DRIVER_ID, "NOLO CV1 direct HID", None),
            sources,
        })?;
        let device = info.open_device(&api)?;
        let mut fusion = [ImuFusion::new(120.0), ImuFusion::new(120.0)];
        let mut sequence = [0_u64; 2];
        let mut report = [0_u8; REPORT_SIZE];
        loop {
            match device.read_timeout(&mut report, 100) {
                Ok(0) => {}
                Ok(REPORT_SIZE) => {
                    let Some(frame) = decode_report(&report)? else {
                        continue;
                    };
                    let index = usize::from(frame.controller_id);
                    sequence[index] += 1;
                    let time_ns = monotonic_ns();
                    let gyro = [
                        -f32::from(frame.gyroscope[0]) * (2000.0 / 32768.0),
                        -f32::from(frame.gyroscope[1]) * (2000.0 / 32768.0),
                        f32::from(frame.gyroscope[2]) * (2000.0 / 32768.0),
                    ];
                    let acceleration = [
                        f32::from(frame.accelerometer[0]) / 1024.0,
                        f32::from(frame.accelerometer[1]) / 1024.0,
                        -f32::from(frame.accelerometer[2]) / 1024.0,
                    ];
                    let orientation = fusion[index].update(gyro, acceleration, time_ns);
                    let mut components = BTreeMap::new();
                    for (bit, path, _) in NOLO_BUTTONS {
                        components.insert(path.into(), f64::from((frame.buttons >> bit) & 1));
                    }
                    let [x, y] = frame.touchpad.unwrap_or([0, 0]);
                    components.insert("axis/touchpad_x".into(), f64::from(x) / 255.0);
                    components.insert("axis/touchpad_y".into(), f64::from(y) / 255.0);
                    let received = now_ns();
                    sender.send(DriverEvent::Sample(RawSample {
                        sequence: sequence[index],
                        source_time_ns: received,
                        received_time_ns: received,
                        source_id: format!("nolo:{device_id}:controller-{}", frame.controller_id),
                        position_m: Some(frame.position.map(f64::from)),
                        orientation_xyzw: Some(orientation),
                        components,
                    }))?;
                }
                Ok(size) => {
                    sender.send(DriverEvent::Error {
                        driver_id: NOLO_DRIVER_ID.into(),
                        message: format!("NOLO HID report 长度为 {size}，期望 {REPORT_SIZE}"),
                    })?;
                }
                Err(error) => {
                    sender.send(DriverEvent::Error {
                        driver_id: NOLO_DRIVER_ID.into(),
                        message: error.to_string(),
                    })?;
                    break;
                }
            }
        }
    }
}

fn nolo_source(info: &hidapi::DeviceInfo, device_id: &str, controller: u8) -> InputSourceInfo {
    let mut components = NOLO_BUTTONS
        .into_iter()
        .map(|(_, path, localized_name)| InputComponentInfo {
            path: path.into(),
            action_type: ActionType::Boolean,
            localized_name: Some(localized_name.into()),
            definition_source: "nolo-cv1-report".into(),
        })
        .collect::<Vec<_>>();
    components.extend(
        [
            ("axis/touchpad_x", "触摸板横向位置"),
            ("axis/touchpad_y", "触摸板纵向位置"),
        ]
        .map(|(path, localized_name)| InputComponentInfo {
            path: path.into(),
            action_type: ActionType::Float,
            localized_name: Some(localized_name.into()),
            definition_source: "nolo-cv1-report".into(),
        }),
    );
    InputSourceInfo {
        source_id: format!("nolo:{device_id}:controller-{controller}"),
        driver_id: NOLO_DRIVER_ID.into(),
        device_id: device_id.into(),
        display_name: format!(
            "{} / Controller {controller}",
            info.product_string().unwrap_or("NOLO CV1")
        ),
        vendor_id: Some(format!("{:04x}", info.vendor_id())),
        product_id: Some(format!("{:04x}", info.product_id())),
        serial: info.serial_number().map(str::to_owned),
        position_capable: true,
        orientation_capable: true,
        action_capable: true,
        active: true,
        available_components: components,
        available_feedback_capabilities: vec![],
        original_error: None,
    }
}

struct SdlGamepadState {
    gamepad: Gamepad,
    source: InputSourceInfo,
    sequence: u64,
    fusion: Option<ImuFusion>,
    latest_acceleration: Option<[f32; 3]>,
    orientation: Option<[f64; 4]>,
}

fn spawn_sdl_driver(sender: Sender<DriverEvent>, haptic: Receiver<HapticCommand>) {
    thread::spawn(move || {
        if let Err(error) = run_sdl_driver(&sender, &haptic) {
            let _ = sender.send(DriverEvent::Error {
                driver_id: SDL_DRIVER_ID.into(),
                message: error.to_string(),
            });
        }
    });
}

fn run_sdl_driver(sender: &Sender<DriverEvent>, haptic: &Receiver<HapticCommand>) -> Result<()> {
    let sdl = sdl3::init().map_err(|error| eyre!(error))?;
    let subsystem = sdl.gamepad().map_err(|error| eyre!(error))?;
    let mut events = sdl.event_pump().map_err(|error| eyre!(error))?;
    let mut gamepads = HashMap::<u32, SdlGamepadState>::new();
    for id in subsystem.gamepads().map_err(|error| eyre!(error))? {
        open_sdl_gamepad(&subsystem, id, &mut gamepads)?;
    }
    publish_sdl_sources(sender, &gamepads)?;

    loop {
        if let Some(event) = events.wait_event_timeout(Duration::from_millis(10)) {
            match event {
                SdlEvent::ControllerDeviceAdded { which, .. } => {
                    open_sdl_gamepad(
                        &subsystem,
                        sdl3::joystick::JoystickId::new(which),
                        &mut gamepads,
                    )?;
                    publish_sdl_sources(sender, &gamepads)?;
                }
                SdlEvent::ControllerDeviceRemoved { which, .. } => {
                    gamepads.remove(&which);
                    publish_sdl_sources(sender, &gamepads)?;
                }
                SdlEvent::ControllerSensorUpdated {
                    timestamp,
                    which,
                    sensor,
                    data,
                } => {
                    if let Some(state) = gamepads.get_mut(&which) {
                        match sensor {
                            SensorType::Accelerometer => state.latest_acceleration = Some(data),
                            SensorType::Gyroscope => {
                                if let (Some(acceleration), Some(fusion)) =
                                    (state.latest_acceleration, state.fusion.as_mut())
                                {
                                    let gyro = data.map(|value| value.to_degrees());
                                    let acceleration = acceleration.map(|value| value / 9.80665);
                                    state.orientation =
                                        Some(fusion.update(gyro, acceleration, timestamp));
                                }
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }

        while let Ok(command) = haptic.try_recv() {
            if let Some(state) = gamepads
                .values_mut()
                .find(|state| state.source.source_id == command.source_id)
            {
                apply_haptic(state, command);
            }
        }

        let received = now_ns();
        for state in gamepads.values_mut() {
            state.sequence += 1;
            let mut components = BTreeMap::new();
            for (axis, path, _) in SDL_AXES {
                if state.gamepad.has_axis(axis) {
                    let raw = state.gamepad.axis(axis);
                    let value = match axis {
                        Axis::TriggerLeft | Axis::TriggerRight => f64::from(raw) / 32767.0,
                        _ if raw < 0 => f64::from(raw) / 32768.0,
                        _ => f64::from(raw) / 32767.0,
                    };
                    components.insert(path.into(), value);
                }
            }
            for (button, path, _) in SDL_BUTTONS {
                if state.gamepad.has_button(button) {
                    components.insert(path.into(), f64::from(state.gamepad.button(button)));
                }
            }
            sender.send(DriverEvent::Sample(RawSample {
                sequence: state.sequence,
                source_time_ns: received,
                received_time_ns: received,
                source_id: state.source.source_id.clone(),
                position_m: None,
                orientation_xyzw: state.orientation,
                components,
            }))?;
        }
    }
}

fn open_sdl_gamepad(
    subsystem: &sdl3::GamepadSubsystem,
    id: sdl3::joystick::JoystickId,
    gamepads: &mut HashMap<u32, SdlGamepadState>,
) -> Result<()> {
    let raw_id = id.value();
    if gamepads.contains_key(&raw_id) {
        return Ok(());
    }
    let gamepad = subsystem.open(id).map_err(|error| eyre!(error))?;
    let has_gyro = unsafe { gamepad.has_sensor(SensorType::Gyroscope) };
    let has_acceleration = unsafe { gamepad.has_sensor(SensorType::Accelerometer) };
    if has_gyro && has_acceleration {
        gamepad
            .sensor_set_enabled(SensorType::Gyroscope, true)
            .map_err(|error| eyre!(error.to_string()))?;
        gamepad
            .sensor_set_enabled(SensorType::Accelerometer, true)
            .map_err(|error| eyre!(error.to_string()))?;
    }
    let source_id = format!("sdl3:{raw_id}");
    let has_rumble = unsafe { gamepad.has_rumble() };
    let has_trigger_rumble = unsafe { gamepad.has_rumble_triggers() };
    let components = SDL_AXES
        .iter()
        .filter(|(axis, _, _)| gamepad.has_axis(*axis))
        .map(|(_, path, localized_name)| InputComponentInfo {
            path: (*path).into(),
            action_type: ActionType::Float,
            localized_name: Some((*localized_name).into()),
            definition_source: "sdl3-gamepad".into(),
        })
        .chain(
            SDL_BUTTONS
                .iter()
                .filter(|(button, _, _)| gamepad.has_button(*button))
                .map(|(_, path, localized_name)| InputComponentInfo {
                    path: (*path).into(),
                    action_type: ActionType::Boolean,
                    localized_name: Some((*localized_name).into()),
                    definition_source: "sdl3-gamepad".into(),
                }),
        )
        .collect();
    let device_id = gamepad
        .serial_number()
        .or_else(|| gamepad.path())
        .unwrap_or_else(|| source_id.clone());
    let feedback_capabilities = sdl_feedback_capabilities(has_trigger_rumble, has_rumble);
    let source = InputSourceInfo {
        source_id,
        driver_id: SDL_DRIVER_ID.into(),
        device_id,
        display_name: gamepad.name().unwrap_or_else(|| "SDL3 gamepad".into()),
        vendor_id: gamepad.vendor_id().map(|value| format!("{value:04x}")),
        product_id: gamepad.product_id().map(|value| format!("{value:04x}")),
        serial: gamepad.serial_number(),
        position_capable: false,
        orientation_capable: has_gyro && has_acceleration,
        action_capable: true,
        active: true,
        available_components: components,
        available_feedback_capabilities: feedback_capabilities,
        original_error: None,
    };
    let fusion = (has_gyro && has_acceleration)
        .then(|| ImuFusion::new(gamepad.sensor_get_data_rate(SensorType::Gyroscope)));
    gamepads.insert(
        raw_id,
        SdlGamepadState {
            gamepad,
            source,
            sequence: 0,
            fusion,
            latest_acceleration: None,
            orientation: None,
        },
    );
    Ok(())
}

fn sdl_feedback_capabilities(
    has_trigger_feedback: bool,
    has_controller_rumble: bool,
) -> Vec<InputFeedbackCapabilityInfo> {
    let mut feedback_capabilities = vec![];
    if has_trigger_feedback {
        feedback_capabilities.extend([
            InputFeedbackCapabilityInfo {
                path: "feedback/trigger_left".into(),
                localized_name: Some("左扳机力度反馈".into()),
            },
            InputFeedbackCapabilityInfo {
                path: "feedback/trigger_right".into(),
                localized_name: Some("右扳机力度反馈".into()),
            },
        ]);
    }
    if has_controller_rumble {
        feedback_capabilities.push(InputFeedbackCapabilityInfo {
            path: "feedback/rumble".into(),
            localized_name: Some("手柄振动".into()),
        });
    }
    feedback_capabilities
}

fn haptic_intensity(strength_percent: f64) -> u16 {
    (strength_percent.clamp(0.0, 100.0) * f64::from(u16::MAX) / 100.0).round() as u16
}

fn apply_haptic(state: &mut SdlGamepadState, command: HapticCommand) {
    let intensity = command.intensity;
    match command.capability_path.as_str() {
        "feedback/trigger_left" => {
            let _ = state
                .gamepad
                .set_rumble_triggers(intensity, 0, HAPTIC_DURATION_MS);
        }
        "feedback/trigger_right" => {
            let _ = state
                .gamepad
                .set_rumble_triggers(0, intensity, HAPTIC_DURATION_MS);
        }
        "feedback/rumble" => {
            let _ = state
                .gamepad
                .set_rumble(intensity, intensity, HAPTIC_DURATION_MS);
        }
        _ => {}
    }
}

fn publish_sdl_sources(
    sender: &Sender<DriverEvent>,
    gamepads: &HashMap<u32, SdlGamepadState>,
) -> Result<()> {
    sender.send(DriverEvent::Sources {
        driver: driver_info(SDL_DRIVER_ID, "SDL3 gamepad", None),
        sources: gamepads
            .values()
            .map(|state| state.source.clone())
            .collect(),
    })?;
    Ok(())
}

fn driver_info(driver_id: &str, display_name: &str, error: Option<String>) -> InputDriverInfo {
    InputDriverInfo {
        driver_id: driver_id.into(),
        display_name: display_name.into(),
        version: env!("CARGO_PKG_VERSION").into(),
        original_error: error,
    }
}

fn publish_snapshot(node: &mut DoraNode, input: &mut ControllerInput) -> Result<()> {
    input.drain();
    publish_discovery(node, input)
}

fn publish_discovery(node: &mut DoraNode, input: &ControllerInput) -> Result<()> {
    send(node, "discovery_state", &input.discovery_state())?;
    send(node, "service_state", &input.service_state())
}

fn send<T: serde::Serialize>(node: &mut DoraNode, id: &str, value: &T) -> Result<()> {
    node.send_output(
        DataId::from(id.to_owned()),
        MetadataParameters::default(),
        to_arrow(value)?,
    )?;
    Ok(())
}

fn monotonic_ns() -> u64 {
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    START.get_or_init(Instant::now).elapsed().as_nanos() as u64
}

fn now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as i64
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    fn source(
        source_id: &str,
        position_capable: bool,
        orientation_capable: bool,
    ) -> InputSourceInfo {
        InputSourceInfo {
            source_id: source_id.into(),
            driver_id: "fixture-driver".into(),
            device_id: source_id.into(),
            display_name: source_id.into(),
            vendor_id: None,
            product_id: None,
            serial: None,
            position_capable,
            orientation_capable,
            action_capable: true,
            active: true,
            available_components: vec![],
            available_feedback_capabilities: vec![],
            original_error: None,
        }
    }

    fn sample(components: &[(&str, f64)]) -> BTreeMap<String, RawSample> {
        let sample = RawSample {
            sequence: 1,
            source_time_ns: 1,
            received_time_ns: 1,
            source_id: "fixture".into(),
            position_m: None,
            orientation_xyzw: None,
            components: components
                .iter()
                .map(|(key, value)| ((*key).into(), *value))
                .collect(),
        };
        BTreeMap::from([("fixture".into(), sample)])
    }

    #[test]
    fn declared_component_tables_cover_sdl3_and_named_nolo_inputs() {
        assert_eq!(SDL_AXES.len(), 6);
        assert_eq!(SDL_BUTTONS.len(), 26);
        assert_eq!(
            SDL_BUTTONS
                .iter()
                .map(|(_, path, _)| *path)
                .collect::<BTreeSet<_>>()
                .len(),
            SDL_BUTTONS.len()
        );
        assert!(SDL_AXES.iter().all(|(_, _, name)| !name.is_empty()));
        assert!(SDL_BUTTONS.iter().all(|(_, _, name)| !name.is_empty()));
        assert!(
            SDL_BUTTONS
                .iter()
                .any(|(_, path, _)| *path == "button/misc6")
        );
        assert!(
            SDL_BUTTONS
                .iter()
                .any(|(_, path, _)| *path == "button/left_paddle2")
        );
        assert!(
            SDL_BUTTONS
                .iter()
                .any(|(_, path, _)| *path == "button/right_paddle2")
        );

        assert_eq!(NOLO_BUTTONS.map(|(bit, _, _)| bit), [0, 1, 2, 3, 4, 5]);
        assert_eq!(
            NOLO_BUTTONS.map(|(_, path, _)| path),
            [
                "button/touchpad",
                "button/trigger",
                "button/menu",
                "button/system",
                "button/grip",
                "button/touchpad_touch",
            ]
        );
    }

    #[test]
    fn discovery_keeps_exact_live_component_values() {
        let values = live_component_values(&sample(&[
            ("button/south", 1.0),
            ("axis/right_trigger", 0.0),
        ]));
        assert_eq!(values["fixture"]["button/south"], 1.0);
        assert_eq!(values["fixture"]["axis/right_trigger"], 0.0);
    }

    #[test]
    fn simulation_uses_declared_pose_action_and_virtual_feedback_bindings() {
        let original = InputConfig {
            position_source: Some(PoseSourceSelection {
                driver_id: "physical-driver".into(),
                device_id: "physical-device".into(),
                source_id: "physical-source".into(),
            }),
            bindings: vec![ActionBinding {
                action: "primary_tool".into(),
                action_type: ActionType::Float,
                source_id: "physical-source".into(),
                component_paths: vec!["axis/trigger".into()],
                invert: false,
            }],
            ..Default::default()
        };
        let config = simulation_config(&original);
        let selection = config.position_source.as_ref().unwrap();
        assert_eq!(selection.source_id, SIMULATION_SOURCE_ID);
        assert_eq!(
            config
                .orientation_source
                .as_ref()
                .map(|source| source.source_id.as_str()),
            Some(SIMULATION_SOURCE_ID)
        );
        assert_eq!(config.bindings.len(), 2);
        assert_eq!(config.bindings[0].action, "start_stop");
        assert_eq!(
            config.bindings[0].component_paths,
            [
                SIMULATION_START_STOP_COMPONENT_A,
                SIMULATION_START_STOP_COMPONENT_B
            ]
        );
        assert_eq!(config.bindings[1].action, "primary_tool");
        assert_eq!(
            config.bindings[1].component_paths,
            [SIMULATION_PRIMARY_TOOL_COMPONENT]
        );
        assert_eq!(config.feedback_bindings.len(), 1);
        assert!(is_virtual_feedback(&config.feedback_bindings[0]));
        assert_eq!(
            original.position_source.unwrap().source_id,
            "physical-source"
        );

        let source = simulation::source_info();
        let sample = SimulationPlayback::default().sample(1, 1).raw;
        let sources = BTreeMap::from([(source.source_id.clone(), source)]);
        let samples = BTreeMap::from([(sample.source_id.clone(), sample)]);
        let pose = combined_pose_frame(
            config.position_source.as_ref(),
            config.orientation_source.as_ref(),
            &sources,
            &samples,
            1,
            1,
        );
        let control = evaluate_actions(
            &samples,
            &config.bindings,
            &config.component_offsets,
            None,
            1,
            1,
        );
        assert_eq!(
            pose.position_source_id.as_deref(),
            Some(SIMULATION_SOURCE_ID)
        );
        assert_eq!(
            pose.orientation_source_id.as_deref(),
            Some(SIMULATION_SOURCE_ID)
        );
        assert!(control.start_stop.is_active);
        assert!(control.start_stop.value);
        assert!(control.actuator_actions.primary_tool.is_active);
        assert_eq!(control.actuator_actions.primary_tool.value, 1.0);
    }

    #[test]
    fn persisted_selection_contains_identifiers_not_online_state() {
        let path = std::env::temp_dir().join(format!(
            "controller-input-config-{}-{}.json",
            std::process::id(),
            now_ns()
        ));
        let config = InputConfig {
            position_source: Some(PoseSourceSelection {
                driver_id: "driver".into(),
                device_id: "device".into(),
                source_id: "source".into(),
            }),
            ..Default::default()
        };

        save(&path, &config).unwrap();
        let stored = std::fs::read_to_string(&path).unwrap();
        assert!(stored.contains("driver_id"));
        assert!(stored.contains("device_id"));
        assert!(stored.contains("source_id"));
        assert!(!stored.contains("active"));
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn tool_open_button_emits_a_boolean_press_edge() {
        let binding = ActionBinding {
            action: "primary_tool_open".into(),
            action_type: ActionType::Boolean,
            source_id: "fixture".into(),
            component_paths: vec!["button/south".into()],
            invert: false,
        };
        let bindings = [binding];
        let pressed = evaluate_actions(
            &sample(&[("button/south", 1.0)]),
            &bindings,
            &BTreeMap::new(),
            None,
            1,
            1,
        );
        assert!(pressed.actuator_actions.primary_tool_open.value);
        assert!(
            pressed
                .actuator_actions
                .primary_tool_open
                .changed_since_last_sync
        );
        let held = evaluate_actions(
            &sample(&[("button/south", 1.0)]),
            &bindings,
            &BTreeMap::new(),
            Some(&pressed),
            2,
            2,
        );
        assert!(held.actuator_actions.primary_tool_open.value);
        assert!(
            !held
                .actuator_actions
                .primary_tool_open
                .changed_since_last_sync
        );
    }

    #[test]
    fn continuous_component_offset_is_subtracted_without_a_dead_zone() {
        let binding = ActionBinding {
            action: "move_up_down".into(),
            action_type: ActionType::Float,
            source_id: "fixture".into(),
            component_paths: vec!["axis/left_y".into()],
            invert: false,
        };
        let offsets = BTreeMap::from([(
            "fixture".into(),
            BTreeMap::from([("axis/left_y".into(), -0.06)]),
        )]);
        let centered = evaluate_actions(
            &sample(&[("axis/left_y", -0.06)]),
            std::slice::from_ref(&binding),
            &offsets,
            None,
            1,
            1,
        );
        assert_eq!(centered.move_up_down.value, 0.0);

        let moved = evaluate_actions(
            &sample(&[("axis/left_y", 0.44)]),
            &[binding],
            &offsets,
            None,
            1,
            1,
        );
        assert!((moved.move_up_down.value - 0.5).abs() < f64::EPSILON);
    }

    #[test]
    fn analog_trigger_remains_continuous() {
        let binding = ActionBinding {
            action: "primary_tool".into(),
            action_type: ActionType::Float,
            source_id: "fixture".into(),
            component_paths: vec!["axis/right_trigger".into()],
            invert: false,
        };
        let frame = evaluate_actions(
            &sample(&[("axis/right_trigger", 0.375)]),
            &[binding],
            &BTreeMap::new(),
            None,
            1,
            1,
        );
        assert_eq!(frame.actuator_actions.primary_tool.value, 0.375);
    }

    #[test]
    fn boolean_chord_requires_every_bound_button() {
        let binding = ActionBinding {
            action: "start_stop".into(),
            action_type: ActionType::Boolean,
            source_id: "fixture".into(),
            component_paths: vec!["button/a".into(), "button/b".into()],
            invert: false,
        };
        let one = evaluate_actions(
            &sample(&[("button/a", 1.0), ("button/b", 0.0)]),
            std::slice::from_ref(&binding),
            &BTreeMap::new(),
            None,
            1,
            1,
        );
        assert!(!one.start_stop.value);
        let both = evaluate_actions(
            &sample(&[("button/a", 1.0), ("button/b", 1.0)]),
            &[binding],
            &BTreeMap::new(),
            Some(&one),
            2,
            2,
        );
        assert!(both.start_stop.value);
        assert!(both.start_stop.changed_since_last_sync);
    }

    #[test]
    fn paired_buttons_form_a_signed_action_without_a_dead_zone() {
        let binding = ActionBinding {
            action: "move_left_right".into(),
            action_type: ActionType::Float,
            source_id: "fixture".into(),
            component_paths: vec!["button/left".into(), "button/right".into()],
            invert: false,
        };
        let frame = evaluate_actions(
            &sample(&[("button/left", 0.0), ("button/right", 1.0)]),
            &[binding],
            &BTreeMap::new(),
            None,
            1,
            1,
        );
        assert_eq!(frame.move_left_right.value, 1.0);
    }

    #[test]
    fn pose_components_and_actions_can_come_from_different_devices() {
        let mut samples = sample(&[("button/one", 1.0)]);
        let mut position = samples.remove("fixture").unwrap();
        position.source_id = "position-device".into();
        position.position_m = Some([1.0, 2.0, 3.0]);
        let mut orientation = position.clone();
        orientation.source_id = "orientation-device".into();
        orientation.position_m = None;
        orientation.orientation_xyzw = Some([0.0, 0.0, 0.5, 0.5]);
        orientation.components = BTreeMap::from([("axis/value".into(), 0.75)]);
        samples.insert(position.source_id.clone(), position);
        samples.insert(orientation.source_id.clone(), orientation);

        let sources = BTreeMap::from([
            (
                "position-device".into(),
                source("position-device", true, false),
            ),
            (
                "orientation-device".into(),
                source("orientation-device", false, true),
            ),
        ]);
        let pose = combined_pose_frame(
            Some(&PoseSourceSelection {
                driver_id: "driver-a".into(),
                device_id: "device-a".into(),
                source_id: "position-device".into(),
            }),
            Some(&PoseSourceSelection {
                driver_id: "driver-b".into(),
                device_id: "device-b".into(),
                source_id: "orientation-device".into(),
            }),
            &sources,
            &samples,
            1,
            1,
        );
        assert_eq!(pose.position_m, [1.0, 2.0, 3.0]);
        assert_eq!(pose.orientation_xyzw, [0.0, 0.0, 0.5, 0.5]);
        assert_eq!(pose.position_source_id.as_deref(), Some("position-device"));
        assert_eq!(
            pose.orientation_source_id.as_deref(),
            Some("orientation-device")
        );
        assert!(pose.position_source_capable);
        assert!(pose.orientation_source_capable);

        let control = evaluate_actions(
            &samples,
            &[
                ActionBinding {
                    action: "start_stop".into(),
                    action_type: ActionType::Boolean,
                    source_id: "position-device".into(),
                    component_paths: vec!["button/one".into()],
                    invert: false,
                },
                ActionBinding {
                    action: "front_pitch".into(),
                    action_type: ActionType::Float,
                    source_id: "orientation-device".into(),
                    component_paths: vec!["axis/value".into()],
                    invert: false,
                },
            ],
            &BTreeMap::new(),
            None,
            1,
            1,
        );
        assert!(control.start_stop.value);
        assert_eq!(control.front_pitch.value, 0.75);
    }

    #[test]
    fn pose_source_candidates_follow_declared_capabilities() {
        let action_only_a = source("action-only-a", false, false);
        let action_only_b = source("action-only-b", false, false);
        let orientation_only = source("orientation-only", false, true);
        let full_pose = source("full-pose", true, true);

        assert!(!supports_pose_component(
            &action_only_a,
            PoseComponent::Position
        ));
        assert!(!supports_pose_component(
            &action_only_a,
            PoseComponent::Orientation
        ));
        assert!(!supports_pose_component(
            &action_only_b,
            PoseComponent::Orientation
        ));
        assert!(!supports_pose_component(
            &orientation_only,
            PoseComponent::Position
        ));
        assert!(supports_pose_component(
            &orientation_only,
            PoseComponent::Orientation
        ));
        assert!(supports_pose_component(&full_pose, PoseComponent::Position));
        assert!(supports_pose_component(
            &full_pose,
            PoseComponent::Orientation
        ));
    }

    #[test]
    fn disconnected_pose_source_emits_no_stale_capability_or_sample() {
        let selection = PoseSourceSelection {
            driver_id: "fixture-driver".into(),
            device_id: "disconnected".into(),
            source_id: "disconnected".into(),
        };
        let pose = combined_pose_frame(
            Some(&selection),
            Some(&selection),
            &BTreeMap::new(),
            &BTreeMap::new(),
            1,
            1,
        );

        assert!(!pose.position_source_capable);
        assert!(!pose.orientation_source_capable);
        assert!(!pose.flags.position_valid);
        assert!(!pose.flags.orientation_valid);
    }

    #[test]
    fn haptic_intensity_maps_percent_action_feedback() {
        assert_eq!(haptic_intensity(0.0), 0);
        assert_eq!(haptic_intensity(50.0), 32_768);
        assert_eq!(haptic_intensity(100.0), u16::MAX);
    }

    #[test]
    fn trigger_feedback_reports_two_independent_bindable_capabilities() {
        let capabilities = sdl_feedback_capabilities(true, false);
        assert_eq!(
            capabilities
                .iter()
                .map(|capability| capability.path.as_str())
                .collect::<Vec<_>>(),
            ["feedback/trigger_left", "feedback/trigger_right"]
        );
        assert!(sdl_feedback_capabilities(false, false).is_empty());
    }

    #[test]
    fn virtual_feedback_is_an_always_applicable_binding_target() {
        let binding = ActionFeedbackBinding {
            action: "primary_tool".into(),
            source_id: VIRTUAL_FEEDBACK_SOURCE_ID.into(),
            capability_path: VIRTUAL_FEEDBACK_CAPABILITY_PATH.into(),
        };

        assert!(is_virtual_feedback(&binding));
    }
}
