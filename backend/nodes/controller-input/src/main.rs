mod simulation;

use std::{
    collections::{BTreeMap, HashMap},
    path::PathBuf,
    sync::mpsc::{self, Receiver, Sender, TryRecvError},
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
    AbsolutePoseFrame, ActionBinding, ActionType, ApplyInputBindingsRequest, ArmTelemetry,
    BooleanActionSample, ControlInputFrame, FloatActionSample, InputBindingState,
    InputComponentInfo, InputDiscoveryState, InputDriverInfo, InputSimulationRequest,
    InputSimulationState, InputSourceInfo, InputStreamDiagnostics, PoseFlags, RequestAction,
    RequestResult, SCHEMA_VERSION, SelectInputSourceRequest, SelectedInputSourceState,
    ServiceState, from_arrow, to_arrow,
};
use sdl3::{
    event::Event as SdlEvent,
    gamepad::{Axis, Button, Gamepad},
    sensor::SensorType,
};
use serde::{Deserialize, Serialize};
use simulation::{SimulationPlayback, inactive_input};

const NOLO_DRIVER_ID: &str = "nolo-cv1-hid";
const SDL_DRIVER_ID: &str = "sdl3-gamepad";
const CONFIG_SCHEMA_VERSION: u32 = 1;
const HAPTIC_DURATION_MS: u32 = u32::MAX;

const ACTIONS: [(&str, ActionType); 8] = [
    ("control_active", ActionType::Boolean),
    ("confirm_origin", ActionType::Boolean),
    ("primary_tool", ActionType::Float),
    ("move_forward_back", ActionType::Float),
    ("move_left_right", ActionType::Float),
    ("move_up_down", ActionType::Float),
    ("front_pitch", ActionType::Float),
    ("horizontal_arc", ActionType::Float),
];

const SDL_AXES: [(Axis, &str); 6] = [
    (Axis::LeftX, "axis/left_x"),
    (Axis::LeftY, "axis/left_y"),
    (Axis::RightX, "axis/right_x"),
    (Axis::RightY, "axis/right_y"),
    (Axis::TriggerLeft, "axis/left_trigger"),
    (Axis::TriggerRight, "axis/right_trigger"),
];

const SDL_BUTTONS: [(Button, &str); 21] = [
    (Button::South, "button/south"),
    (Button::East, "button/east"),
    (Button::West, "button/west"),
    (Button::North, "button/north"),
    (Button::Back, "button/back"),
    (Button::Guide, "button/guide"),
    (Button::Start, "button/start"),
    (Button::LeftStick, "button/left_stick"),
    (Button::RightStick, "button/right_stick"),
    (Button::LeftShoulder, "button/left_shoulder"),
    (Button::RightShoulder, "button/right_shoulder"),
    (Button::DPadUp, "button/dpad_up"),
    (Button::DPadDown, "button/dpad_down"),
    (Button::DPadLeft, "button/dpad_left"),
    (Button::DPadRight, "button/dpad_right"),
    (Button::Misc1, "button/misc1"),
    (Button::Misc2, "button/misc2"),
    (Button::Misc3, "button/misc3"),
    (Button::LeftPaddle1, "button/left_paddle1"),
    (Button::RightPaddle1, "button/right_paddle1"),
    (Button::Touchpad, "button/touchpad"),
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
                "select_source" => {
                    let request: SelectInputSourceRequest =
                        from_arrow(data.as_array()).context("decode select_source")?;
                    let result = input.select(request);
                    send(&mut node, "source_request_result", &result)?;
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
                    let (result, inactive) = input.set_simulation(request);
                    send(&mut node, "simulation_request_result", &result)?;
                    if let Some(control) = inactive {
                        send(&mut node, "control_input", &control)?;
                    }
                    publish_snapshot(&mut node, &mut input)?;
                }
                "arm_telemetry" => {
                    let telemetry: ArmTelemetry =
                        from_arrow(data.as_array()).context("decode arm_telemetry")?;
                    input.apply_telemetry(telemetry);
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
    component: Option<String>,
    intensity: u16,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct InputSelection {
    driver_id: String,
    device_id: String,
    source_id: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct InputConfig {
    #[serde(default = "config_schema_version")]
    schema_version: u32,
    selected: Option<InputSelection>,
    bindings: BTreeMap<String, Vec<ActionBinding>>,
    config_version: u64,
}

const fn config_schema_version() -> u32 {
    CONFIG_SCHEMA_VERSION
}

impl Default for InputConfig {
    fn default() -> Self {
        Self {
            schema_version: CONFIG_SCHEMA_VERSION,
            selected: None,
            bindings: BTreeMap::new(),
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
    last_published_sequence: Option<u64>,
    previous_control: Option<ControlInputFrame>,
    simulation: Option<SimulationPlayback>,
    simulation_state: InputSimulationState,
    next_sequence: u64,
    receiver: Receiver<DriverEvent>,
    haptic: Sender<HapticCommand>,
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
            last_published_sequence: None,
            previous_control: None,
            simulation: None,
            simulation_state: InputSimulationState {
                schema_version: SCHEMA_VERSION,
                ..Default::default()
            },
            next_sequence: 1,
            receiver,
            haptic,
            last_error: None,
        })
    }

    fn commit_config(&mut self, config: InputConfig) -> Result<()> {
        save(&self.config_path, &config)?;
        self.config = config;
        self.last_published_sequence = None;
        self.previous_control = None;
        Ok(())
    }

    fn drain(&mut self) {
        loop {
            match self.receiver.try_recv() {
                Ok(DriverEvent::Sources { driver, sources }) => {
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
                }
                Ok(DriverEvent::Sample(sample)) => {
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
                        diagnostics.sequence_gaps +=
                            sample.sequence.saturating_sub(previous.sequence + 1);
                    }
                    self.samples.insert(sample.source_id.clone(), sample);
                }
                Ok(DriverEvent::Error { driver_id, message }) => {
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
                Err(TryRecvError::Empty | TryRecvError::Disconnected) => break,
            }
        }
    }

    fn tick(&mut self) -> Option<(Option<AbsolutePoseFrame>, ControlInputFrame)> {
        self.drain();
        let now = now_ns();
        if let Some(simulation) = self.simulation.as_mut() {
            let sample = simulation.sample(self.next_sequence, now);
            self.next_sequence += 1;
            self.simulation_state = sample.state;
            self.previous_control = Some(sample.input.clone());
            return Some((Some(sample.pose), sample.input));
        }

        let selected = self.config.selected.as_ref()?;
        let sample = self.samples.get(&selected.source_id)?.clone();
        if self.last_published_sequence == Some(sample.sequence) {
            return None;
        }
        self.last_published_sequence = Some(sample.sequence);
        let bindings = self
            .config
            .bindings
            .get(&sample.source_id)
            .map(Vec::as_slice)
            .unwrap_or_default();
        let control = evaluate_actions(&sample, bindings, self.previous_control.as_ref());
        self.previous_control = Some(control.clone());
        let pose = sample
            .orientation_xyzw
            .map(|orientation_xyzw| AbsolutePoseFrame {
                schema_version: SCHEMA_VERSION,
                sequence: sample.sequence,
                source_time_ns: sample.source_time_ns,
                received_time_ns: sample.received_time_ns,
                source_id: sample.source_id.clone(),
                reference_space: if sample.position_m.is_some() {
                    "nolo_tracking"
                } else {
                    "controller_orientation"
                }
                .into(),
                position_m: sample.position_m.unwrap_or([0.0; 3]),
                orientation_xyzw,
                flags: PoseFlags {
                    position_valid: sample.position_m.is_some(),
                    position_tracked: sample.position_m.is_some(),
                    orientation_valid: true,
                    orientation_tracked: true,
                },
            });
        Some((pose, control))
    }

    fn select(
        &mut self,
        request: SelectInputSourceRequest,
    ) -> RequestResult<SelectedInputSourceState> {
        let change = match request.action {
            RequestAction::Select => {
                let selected = self
                    .sources
                    .get(&request.source_id)
                    .filter(|source| {
                        source.driver_id == request.driver_id
                            && source.device_id == request.device_id
                    })
                    .map(|source| InputSelection {
                        driver_id: source.driver_id.clone(),
                        device_id: source.device_id.clone(),
                        source_id: source.source_id.clone(),
                    })
                    .ok_or_else(|| "选择的输入 source 当前不存在".to_owned());
                selected.and_then(|selected| {
                    let mut config = self.config.clone();
                    config.selected = Some(selected);
                    config.config_version += 1;
                    self.commit_config(config)
                        .map_err(|error| error.to_string())
                })
            }
            RequestAction::Unselect => {
                let mut config = self.config.clone();
                config.selected = None;
                config.config_version += 1;
                self.commit_config(config)
                    .map_err(|error| error.to_string())
            }
            action => Err(format!("input 节点不处理 {action:?} source 请求")),
        };
        request_result(request, self.selected_source_state(), change.err())
    }

    fn apply_bindings(
        &mut self,
        request: ApplyInputBindingsRequest,
    ) -> RequestResult<Vec<InputBindingState>> {
        let error = validate_bindings(&request.bindings)
            .err()
            .map(|error| error.to_string());
        if error.is_none() {
            let mut config = self.config.clone();
            config
                .bindings
                .insert(request.source_id.clone(), request.bindings.clone());
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

    fn set_simulation(
        &mut self,
        request: InputSimulationRequest,
    ) -> (
        RequestResult<InputSimulationState>,
        Option<ControlInputFrame>,
    ) {
        let inactive = if request.enabled {
            self.simulation = Some(SimulationPlayback::default());
            self.simulation_state = InputSimulationState {
                schema_version: SCHEMA_VERSION,
                active: true,
                phase: Some("starting".into()),
                elapsed_s: Some(0.0),
            };
            None
        } else {
            self.simulation = None;
            self.simulation_state = InputSimulationState {
                schema_version: SCHEMA_VERSION,
                ..Default::default()
            };
            let control = inactive_input(self.next_sequence, now_ns());
            self.next_sequence += 1;
            self.previous_control = Some(control.clone());
            Some(control)
        };
        (
            RequestResult {
                schema_version: SCHEMA_VERSION,
                request_id: request.request_id,
                acknowledged_action: RequestAction::Apply,
                value: Some(self.simulation_state.clone()),
                original_error: None,
            },
            inactive,
        )
    }

    fn apply_telemetry(&self, telemetry: ArmTelemetry) {
        let Some(selected) = self.config.selected.as_ref() else {
            return;
        };
        let Some(gripper) = telemetry
            .actuators
            .iter()
            .find(|actuator| actuator.actuator_key == "gripper")
        else {
            return;
        };
        let component = self
            .config
            .bindings
            .get(&selected.source_id)
            .and_then(|bindings| {
                bindings
                    .iter()
                    .find(|binding| binding.action == "primary_tool")
            })
            .and_then(|binding| binding.component_paths.first())
            .cloned();
        let Some(limit) = gripper.command_power_limit_mw else {
            return;
        };
        let _ = self.haptic.send(HapticCommand {
            source_id: selected.source_id.clone(),
            component,
            intensity: haptic_intensity(gripper.power_mw, limit),
        });
    }

    fn binding_states(&self) -> Vec<InputBindingState> {
        let selected = self.config.selected.as_ref();
        let sample = selected.and_then(|value| self.samples.get(&value.source_id));
        let bindings = selected
            .and_then(|value| self.config.bindings.get(&value.source_id))
            .map(Vec::as_slice)
            .unwrap_or_default();
        ACTIONS
            .iter()
            .map(|(action, action_type)| {
                let binding = bindings.iter().find(|value| value.action == *action);
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
                    .and_then(|binding| sample.map(|sample| binding_value(sample, binding)))
                    .unwrap_or(0.0);
                InputBindingState {
                    action: (*action).into(),
                    action_type: *action_type,
                    invert: binding.is_some_and(|value| value.invert),
                    configured_components: components,
                    active: applicable,
                    value,
                    applicable,
                    original_error: None,
                }
            })
            .collect()
    }

    fn selected_source_state(&self) -> Option<SelectedInputSourceState> {
        self.config.selected.as_ref().map(|selected| {
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
            selected_source_id: self
                .config
                .selected
                .as_ref()
                .map(|value| value.source_id.clone()),
            selected_source: self.selected_source_state(),
            bindings: self.binding_states(),
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

fn request_result(
    request: SelectInputSourceRequest,
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
    }
    Ok(())
}

fn evaluate_actions(
    sample: &RawSample,
    bindings: &[ActionBinding],
    previous: Option<&ControlInputFrame>,
) -> ControlInputFrame {
    let value = |action: &str| {
        bindings
            .iter()
            .find(|binding| binding.action == action)
            .map(|binding| binding_value(sample, binding))
            .unwrap_or(0.0)
    };
    let boolean = |action: &str, previous_value: bool| {
        let current = value(action) != 0.0;
        BooleanActionSample {
            is_active: bindings.iter().any(|binding| binding.action == action),
            changed_since_last_sync: current != previous_value,
            value: current,
        }
    };
    let float = |action: &str, previous_value: f64| {
        let current = value(action);
        FloatActionSample {
            is_active: bindings.iter().any(|binding| binding.action == action),
            changed_since_last_sync: current != previous_value,
            value: current,
        }
    };
    ControlInputFrame {
        schema_version: SCHEMA_VERSION,
        sequence: sample.sequence,
        source_time_ns: sample.source_time_ns,
        received_time_ns: sample.received_time_ns,
        source_id: sample.source_id.clone(),
        control_active: boolean(
            "control_active",
            previous.is_some_and(|frame| frame.control_active.value),
        ),
        confirm_origin: boolean(
            "confirm_origin",
            previous.is_some_and(|frame| frame.confirm_origin.value),
        ),
        primary_tool: float(
            "primary_tool",
            previous
                .map(|frame| frame.primary_tool.value)
                .unwrap_or(0.0),
        ),
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

fn binding_value(sample: &RawSample, binding: &ActionBinding) -> f64 {
    let mut value = match binding.component_paths.as_slice() {
        [component] => sample.components.get(component).copied().unwrap_or(0.0),
        [negative, positive] => {
            sample.components.get(positive).copied().unwrap_or(0.0)
                - sample.components.get(negative).copied().unwrap_or(0.0)
        }
        _ => 0.0,
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
                    for bit in 0..8 {
                        components.insert(
                            format!("button/{bit}"),
                            f64::from((frame.buttons >> bit) & 1),
                        );
                    }
                    if let Some([x, y]) = frame.touchpad {
                        components.insert("touch/x".into(), f64::from(x) / 255.0);
                        components.insert("touch/y".into(), f64::from(y) / 255.0);
                    }
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
    let mut components = (0..8)
        .map(|bit| InputComponentInfo {
            path: format!("button/{bit}"),
            action_type: ActionType::Boolean,
            localized_name: Some(format!("Button bit {bit}")),
            definition_source: "nolo-cv1-report".into(),
        })
        .collect::<Vec<_>>();
    components.extend(["touch/x", "touch/y"].map(|path| InputComponentInfo {
        path: path.into(),
        action_type: ActionType::Float,
        localized_name: Some(path.into()),
        definition_source: "nolo-cv1-report".into(),
    }));
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
        rumble_capable: false,
        trigger_rumble_capable: false,
        active: true,
        available_components: components,
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
    has_rumble: bool,
    has_trigger_rumble: bool,
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
            for (axis, path) in SDL_AXES {
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
            for (button, path) in SDL_BUTTONS {
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
        .filter(|(axis, _)| gamepad.has_axis(*axis))
        .map(|(_, path)| InputComponentInfo {
            path: (*path).into(),
            action_type: ActionType::Float,
            localized_name: Some((*path).into()),
            definition_source: "sdl3-gamepad".into(),
        })
        .chain(
            SDL_BUTTONS
                .iter()
                .filter(|(button, _)| gamepad.has_button(*button))
                .map(|(_, path)| InputComponentInfo {
                    path: (*path).into(),
                    action_type: ActionType::Boolean,
                    localized_name: Some((*path).into()),
                    definition_source: "sdl3-gamepad".into(),
                }),
        )
        .collect();
    let device_id = gamepad
        .serial_number()
        .or_else(|| gamepad.path())
        .unwrap_or_else(|| source_id.clone());
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
        rumble_capable: has_rumble,
        trigger_rumble_capable: has_trigger_rumble,
        active: true,
        available_components: components,
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
            has_rumble,
            has_trigger_rumble,
        },
    );
    Ok(())
}

fn haptic_intensity(power_mw: u16, reference_mw: u16) -> u16 {
    ((u32::from(power_mw) * u32::from(u16::MAX)) / u32::from(reference_mw)).min(u32::from(u16::MAX))
        as u16
}

fn apply_haptic(state: &mut SdlGamepadState, command: HapticCommand) {
    let intensity = command.intensity;
    if state.has_trigger_rumble {
        let (left, right) = match command.component.as_deref() {
            Some("axis/left_trigger") => (intensity, 0),
            _ => (0, intensity),
        };
        let _ = state
            .gamepad
            .set_rumble_triggers(left, right, HAPTIC_DURATION_MS);
    } else if state.has_rumble {
        let _ = state
            .gamepad
            .set_rumble(intensity, intensity, HAPTIC_DURATION_MS);
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

    fn sample(components: &[(&str, f64)]) -> RawSample {
        RawSample {
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
        }
    }

    #[test]
    fn persisted_selection_contains_identifiers_not_online_state() {
        let path = std::env::temp_dir().join(format!(
            "controller-input-config-{}-{}.json",
            std::process::id(),
            now_ns()
        ));
        let config = InputConfig {
            selected: Some(InputSelection {
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
    fn analog_trigger_remains_continuous() {
        let binding = ActionBinding {
            action: "primary_tool".into(),
            action_type: ActionType::Float,
            component_paths: vec!["axis/right_trigger".into()],
            invert: false,
        };
        let frame = evaluate_actions(&sample(&[("axis/right_trigger", 0.375)]), &[binding], None);
        assert_eq!(frame.primary_tool.value, 0.375);
    }

    #[test]
    fn paired_buttons_form_a_signed_action_without_a_dead_zone() {
        let binding = ActionBinding {
            action: "move_left_right".into(),
            action_type: ActionType::Float,
            component_paths: vec!["button/left".into(), "button/right".into()],
            invert: false,
        };
        let frame = evaluate_actions(
            &sample(&[("button/left", 0.0), ("button/right", 1.0)]),
            &[binding],
            None,
        );
        assert_eq!(frame.move_left_right.value, 1.0);
    }

    #[test]
    fn haptic_intensity_uses_the_selected_gripper_power() {
        assert_eq!(haptic_intensity(1_000, 2_000), u16::MAX / 2);
        assert_eq!(haptic_intensity(2_000, 2_000), u16::MAX);
        assert_eq!(haptic_intensity(0, 2_000), 0);
    }
}
