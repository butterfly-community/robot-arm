use std::{
    collections::BTreeMap,
    ffi::{CStr, CString, c_char, c_int, c_void},
    fs,
    path::{Path, PathBuf},
    time::{SystemTime, UNIX_EPOCH},
};

use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result, eyre};
use openxr as xr;
use robot_arm_messages::{
    AbsolutePoseFrame, ActionBinding, ActionType, ApplyInputBindingsRequest, BooleanActionSample,
    ControlInputFrame, FloatActionSample, HostDeviceInfo, InputBindingState, InputComponentInfo,
    InputDiscoveryState, InputSimulationRequest, InputSimulationState, InputStreamDiagnostics,
    OpenXrInputSource, OpenXrRuntimeInfo, PoseFlags, RequestAction, RequestResult, SCHEMA_VERSION,
    SelectInputSourceRequest, SelectedInputSourceState, ServiceState, from_arrow, to_arrow,
};
use serde::{Deserialize, Serialize};

mod simulation;

use simulation::{SimulationPlayback, inactive_input};

const USER_PATHS: [&str; 3] = ["/user/hand/left", "/user/hand/right", "/user/gamepad"];
const HAND_PATHS: [&str; 2] = ["/user/hand/left", "/user/hand/right"];
const SIMPLE_CONTROLLER_PROFILE: &str = "/interaction_profiles/khr/simple_controller";
const BOOLEAN_ACTIONS: [&str; 3] = ["control_active", "confirm_origin", "primary_tool_active"];
const DIRECTION_ACTIONS: [&str; 5] = [
    "move_forward_back",
    "move_left_right",
    "move_up_down",
    "front_pitch",
    "horizontal_arc",
];

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct SourceConfig {
    version: u64,
    selected: Option<SelectedSource>,
    bindings_by_profile: BTreeMap<String, Vec<ActionBinding>>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct SelectedSource {
    runtime_identity: String,
    user_path: String,
    interaction_profile: Option<String>,
    confirmed_host_association: Option<String>,
}

struct DirectionActions {
    float: xr::Action<f32>,
    negative: xr::Action<bool>,
    positive: xr::Action<bool>,
}

struct RuntimeActions {
    // OpenXR child handles must be dropped before their parents. Rust drops
    // struct fields in declaration order, so ActionSet stays last.
    booleans: BTreeMap<String, xr::Action<bool>>,
    directions: BTreeMap<String, DirectionActions>,
    discovery: xr::Action<bool>,
    pose: xr::Action<xr::Posef>,
    set: xr::ActionSet,
}

#[derive(Clone, Debug, Default, Deserialize)]
struct InteractionProfileCatalog {
    profiles: Vec<InteractionProfileDefinition>,
}

#[derive(Clone, Debug, Deserialize)]
struct InteractionProfileDefinition {
    profile: String,
    user_paths: Vec<String>,
    components: Vec<InteractionComponentDefinition>,
    pose_suffixes: Vec<String>,
}

#[derive(Clone, Debug, Deserialize)]
struct InteractionComponentDefinition {
    suffix: String,
    action_type: ActionType,
    localized_name: Option<String>,
}

struct RuntimeSession {
    // Spaces -> Session -> Actions -> Instance is the required drop order.
    pose_space: Option<xr::Space>,
    reference_space: xr::Space,
    session: xr::Session<xr::Headless>,
    actions: RuntimeActions,
    instance: xr::Instance,
    running: bool,
    event_buffer: xr::EventDataBuffer,
    view_configuration: xr::ViewConfigurationType,
    pose_binding_error: Option<String>,
    last_pose_error: Option<String>,
    profile_catalog: InteractionProfileCatalog,
}

#[derive(Clone, Debug)]
struct RuntimeDeviceInfo {
    name: String,
    serial: Option<String>,
    supports_position: bool,
    supports_orientation: bool,
}

type MndRootCreate = unsafe extern "C" fn(*mut *mut c_void) -> c_int;
type MndRootDestroy = unsafe extern "C" fn(*mut *mut c_void);
type MndGetDeviceFromRole = unsafe extern "C" fn(*mut c_void, *const c_char, *mut i32) -> c_int;
type MndGetDeviceInfoString =
    unsafe extern "C" fn(*mut c_void, u32, c_int, *mut *const c_char) -> c_int;
type MndGetDeviceInfoBool = unsafe extern "C" fn(*mut c_void, u32, c_int, *mut bool) -> c_int;

#[derive(Default)]
struct StreamObservations {
    received_frames: u64,
    last_sequence: Option<u64>,
    last_source_time_ns: Option<i64>,
    last_received_time_ns: Option<i64>,
    interval_count: u64,
    mean_interval_ns: f64,
    interval_m2_ns: f64,
    sequence_gaps: u64,
    runtime_error_count: u64,
    host_error_count: u64,
}

struct RuntimeSample {
    frame: Option<(Option<AbsolutePoseFrame>, ControlInputFrame)>,
    profile_changed: bool,
}

struct SourceNode {
    config_path: PathBuf,
    config: SourceConfig,
    runtime_info: OpenXrRuntimeInfo,
    runtime: Option<RuntimeSession>,
    host_devices: Vec<HostDeviceInfo>,
    sequence: u64,
    latest_pose: Option<AbsolutePoseFrame>,
    latest_input: Option<ControlInputFrame>,
    observations: StreamObservations,
    last_error: Option<String>,
    simulation: Option<SimulationPlayback>,
    simulation_state: InputSimulationState,
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut source = SourceNode::load();
    source.rebuild_runtime();
    publish_discovery(&mut node, &mut source)?;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => match id.as_str() {
                "tick" => match source.sample(&mut node) {
                    Ok(()) if source.runtime.is_some() => source.last_error = None,
                    Ok(()) => {}
                    Err(error) => source.last_error = Some(error.to_string()),
                },
                "scan" => {
                    if source.runtime.is_none() && source.simulation.is_none() {
                        source.rebuild_runtime();
                    }
                    publish_discovery(&mut node, &mut source)?;
                }
                "snapshot" => publish_discovery(&mut node, &mut source)?,
                "select_source" => {
                    let request: SelectInputSourceRequest =
                        from_arrow(data.as_array()).context("decode select_source")?;
                    if matches!(
                        request.action,
                        RequestAction::Select | RequestAction::Unselect
                    ) {
                        source.publish_inactive(&mut node)?;
                    }
                    let result = source.select(request);
                    send(&mut node, "source_request_result", &result)?;
                    publish_discovery(&mut node, &mut source)?;
                }
                "apply_bindings" => {
                    let request: ApplyInputBindingsRequest =
                        from_arrow(data.as_array()).context("decode apply_bindings")?;
                    if validate_bindings(&request.bindings).is_ok() {
                        source.publish_inactive(&mut node)?;
                    }
                    let result = source.apply_bindings(request);
                    send(&mut node, "bindings_request_result", &result)?;
                    publish_discovery(&mut node, &mut source)?;
                }
                "set_simulation" => {
                    let request: InputSimulationRequest =
                        from_arrow(data.as_array()).context("decode set_simulation")?;
                    let result = source.set_simulation(request, &mut node)?;
                    send(&mut node, "simulation_request_result", &result)?;
                    publish_discovery(&mut node, &mut source)?;
                }
                _ => {}
            },
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}

impl SourceNode {
    fn load() -> Self {
        let path = std::env::var_os("OPENXR_SOURCE_CONFIG")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("/config/openxr-source.json"));
        let (config, last_error) = match fs::read(&path) {
            Ok(bytes) => match serde_json::from_slice(&bytes) {
                Ok(config) => (config, None),
                Err(error) => (
                    SourceConfig::default(),
                    Some(format!("读取采集配置失败：{error}")),
                ),
            },
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
                (SourceConfig::default(), None)
            }
            Err(error) => (
                SourceConfig::default(),
                Some(format!("读取采集配置失败：{error}")),
            ),
        };
        Self {
            config_path: path,
            config,
            runtime_info: OpenXrRuntimeInfo::default(),
            runtime: None,
            host_devices: scan_host_devices(Path::new("/sys/bus/usb/devices")),
            sequence: 0,
            latest_pose: None,
            latest_input: None,
            observations: StreamObservations::default(),
            last_error,
            simulation: None,
            simulation_state: InputSimulationState {
                schema_version: SCHEMA_VERSION,
                ..Default::default()
            },
        }
    }

    fn rebuild_runtime(&mut self) {
        self.runtime = None;
        match RuntimeSession::create(&self.config) {
            Ok((runtime, info)) => {
                self.runtime_info = info;
                self.runtime = Some(runtime);
                self.last_error = None;
            }
            Err(error) => {
                self.runtime_info = probe_runtime();
                self.last_error = Some(error.to_string());
            }
        }
    }

    fn save(&mut self) -> Result<(), String> {
        if let Some(parent) = self.config_path.parent() {
            fs::create_dir_all(parent).map_err(|error| format!("创建采集配置目录失败：{error}"))?;
        }
        fs::write(
            &self.config_path,
            serde_json::to_vec_pretty(&self.config).map_err(|error| error.to_string())?,
        )
        .map_err(|error| format!("保存采集配置失败：{error}"))
    }

    fn select(&mut self, request: SelectInputSourceRequest) -> RequestResult<InputDiscoveryState> {
        let action = request.action;
        let request_id = request.request_id;
        let error = match action {
            RequestAction::Select => {
                self.config.selected = Some(SelectedSource {
                    runtime_identity: request.runtime_identity,
                    user_path: request.user_path,
                    interaction_profile: request.interaction_profile,
                    confirmed_host_association: request.confirmed_host_association,
                });
                self.config.version += 1;
                self.save().err()
            }
            RequestAction::Unselect => {
                self.config.selected = None;
                self.config.version += 1;
                self.save().err()
            }
            _ => Some(format!("采集节点不处理 {action:?} 输入源请求")),
        };
        if error.is_none() {
            self.rebuild_runtime();
        }
        RequestResult {
            schema_version: SCHEMA_VERSION,
            request_id,
            acknowledged_action: action,
            value: Some(self.discovery_state()),
            original_error: error,
        }
    }

    fn apply_bindings(
        &mut self,
        request: ApplyInputBindingsRequest,
    ) -> RequestResult<InputDiscoveryState> {
        let request_id = request.request_id;
        if let Err(error) = validate_bindings(&request.bindings) {
            return RequestResult {
                schema_version: SCHEMA_VERSION,
                request_id,
                acknowledged_action: RequestAction::Apply,
                value: Some(self.discovery_state()),
                original_error: Some(error),
            };
        }
        self.config
            .bindings_by_profile
            .insert(request.interaction_profile, request.bindings);
        self.config.version += 1;
        let mut error = self.save().err();
        if error.is_none() {
            self.rebuild_runtime();
            error = self.last_error.clone();
        }
        RequestResult {
            schema_version: SCHEMA_VERSION,
            request_id,
            acknowledged_action: RequestAction::Apply,
            value: Some(self.discovery_state()),
            original_error: error,
        }
    }

    fn sample(&mut self, node: &mut DoraNode) -> Result<()> {
        if let Some(simulation) = self.simulation.as_mut() {
            self.sequence += 1;
            let sample = simulation.sample(self.sequence, now_ns());
            self.latest_pose = Some(sample.pose.clone());
            self.latest_input = Some(sample.input.clone());
            self.simulation_state = sample.state;
            send(node, "absolute_pose", &sample.pose)?;
            send(node, "control_input", &sample.input)?;
            return Ok(());
        }
        let Some(runtime) = self.runtime.as_mut() else {
            return Ok(());
        };
        let sampled = runtime.sample(
            self.config.selected.as_ref(),
            &self.config.bindings_by_profile,
            &mut self.sequence,
        );
        let sampled = match sampled {
            Ok(value) => value,
            Err(error) => {
                self.observations.runtime_error_count += 1;
                self.publish_inactive(node)?;
                return Err(error);
            }
        };
        if let Some((pose, input)) = sampled.frame {
            if let Some(pose) = pose {
                self.latest_pose = Some(pose.clone());
                send(node, "absolute_pose", &pose)?;
            }
            self.observations.observe(&input);
            self.latest_input = Some(input.clone());
            send(node, "control_input", &input)?;
        }
        if sampled.profile_changed {
            publish_discovery(node, self)?;
        }
        Ok(())
    }

    fn set_simulation(
        &mut self,
        request: InputSimulationRequest,
        node: &mut DoraNode,
    ) -> Result<RequestResult<InputSimulationState>> {
        if request.enabled {
            self.simulation = Some(SimulationPlayback::default());
            self.simulation_state = InputSimulationState {
                schema_version: SCHEMA_VERSION,
                active: true,
                phase: Some("准备模拟输入".into()),
                elapsed_s: Some(0.0),
            };
        } else {
            if self.simulation.take().is_some() {
                self.sequence += 1;
                let input = inactive_input(self.sequence, now_ns());
                self.latest_pose = None;
                self.latest_input = Some(input.clone());
                send(node, "control_input", &input)?;
            }
            self.simulation_state = InputSimulationState {
                schema_version: SCHEMA_VERSION,
                ..Default::default()
            };
        }
        Ok(RequestResult {
            schema_version: SCHEMA_VERSION,
            request_id: request.request_id,
            acknowledged_action: RequestAction::Apply,
            value: Some(self.simulation_state.clone()),
            original_error: None,
        })
    }

    fn publish_inactive(&mut self, node: &mut DoraNode) -> Result<()> {
        let Some(selected) = self.config.selected.as_ref() else {
            return Ok(());
        };
        self.sequence += 1;
        let now = now_ns();
        let input = ControlInputFrame {
            schema_version: SCHEMA_VERSION,
            sequence: self.sequence,
            source_time_ns: now,
            received_time_ns: now,
            source_id: format!("openxr:{}", selected.user_path),
            ..Default::default()
        };
        self.latest_pose = None;
        self.latest_input = Some(input.clone());
        send(node, "control_input", &input)
    }

    fn discovery_state(&self) -> InputDiscoveryState {
        InputDiscoveryState {
            schema_version: SCHEMA_VERSION,
            runtime: self.runtime_info.clone(),
            host_devices: self.host_devices.clone(),
            runtime_sources: self
                .runtime
                .as_ref()
                .map(RuntimeSession::sources)
                .unwrap_or_default(),
            selected_source_id: self
                .config
                .selected
                .as_ref()
                .map(|selected| format!("openxr:{}", selected.user_path)),
            selected_source: self.config.selected.as_ref().map(|selected| {
                let source_id = format!("openxr:{}", selected.user_path);
                let active = self.runtime.as_ref().is_some_and(|runtime| {
                    runtime
                        .sources()
                        .iter()
                        .any(|source| source.source_id == source_id && source.active)
                });
                SelectedInputSourceState {
                    runtime_identity: selected.runtime_identity.clone(),
                    source_id,
                    user_path: selected.user_path.clone(),
                    interaction_profile: selected.interaction_profile.clone(),
                    active,
                }
            }),
            confirmed_host_association: self
                .config
                .selected
                .as_ref()
                .and_then(|value| value.confirmed_host_association.clone()),
            bindings: self
                .runtime
                .as_ref()
                .map(|runtime| runtime.binding_states(&self.config, self.latest_input.as_ref()))
                .unwrap_or_default(),
            diagnostics: self
                .config
                .selected
                .as_ref()
                .map(|selected| {
                    vec![
                        self.observations
                            .snapshot(&format!("openxr:{}", selected.user_path)),
                    ]
                })
                .unwrap_or_default(),
            simulation: self.simulation_state.clone(),
            service: ServiceState {
                schema_version: SCHEMA_VERSION,
                build_version: env!("CARGO_PKG_VERSION").into(),
                config_version: self.config.version,
                running: true,
                has_input: self.latest_pose.is_some() || self.latest_input.is_some(),
                has_output: true,
                last_error: self.last_error.clone(),
                updated_at_ns: now_ns(),
            },
        }
    }
}

impl StreamObservations {
    fn observe(&mut self, frame: &ControlInputFrame) {
        self.received_frames += 1;
        if let Some(previous) = self.last_sequence
            && frame.sequence > previous + 1
        {
            self.sequence_gaps += frame.sequence - previous - 1;
        }
        if let Some(previous) = self.last_received_time_ns {
            let interval = (frame.received_time_ns - previous) as f64;
            self.interval_count += 1;
            let delta = interval - self.mean_interval_ns;
            self.mean_interval_ns += delta / self.interval_count as f64;
            self.interval_m2_ns += delta * (interval - self.mean_interval_ns);
        }
        self.last_sequence = Some(frame.sequence);
        self.last_source_time_ns = Some(frame.source_time_ns);
        self.last_received_time_ns = Some(frame.received_time_ns);
    }

    fn snapshot(&self, source_id: &str) -> InputStreamDiagnostics {
        let observed_rate_hz =
            (self.mean_interval_ns > 0.0).then(|| 1_000_000_000.0 / self.mean_interval_ns);
        let observed_jitter_ms = (self.interval_count > 0)
            .then(|| (self.interval_m2_ns / self.interval_count as f64).sqrt() / 1_000_000.0);
        InputStreamDiagnostics {
            source_id: source_id.into(),
            received_frames: self.received_frames,
            last_source_time_ns: self.last_source_time_ns,
            last_received_time_ns: self.last_received_time_ns,
            observed_rate_hz,
            observed_jitter_ms,
            sequence_gaps: self.sequence_gaps,
            runtime_error_count: self.runtime_error_count,
            host_error_count: self.host_error_count,
        }
    }
}

impl RuntimeSession {
    fn create(config: &SourceConfig) -> Result<(Self, OpenXrRuntimeInfo)> {
        let entry = unsafe { xr::Entry::load()? };
        let available_extensions = entry.enumerate_extensions()?;
        if !available_extensions.mnd_headless {
            return Err(eyre!("OpenXR Runtime 未提供 XR_MND_headless"));
        }
        if !available_extensions.khr_convert_timespec_time {
            return Err(eyre!("OpenXR Runtime 未提供 XR_KHR_convert_timespec_time"));
        }
        let mut extensions = xr::ExtensionSet::default();
        extensions.mnd_headless = true;
        extensions.khr_convert_timespec_time = true;
        let instance = entry.create_instance(
            &xr::ApplicationInfo {
                application_name: "robot-arm-openxr-source",
                application_version: 1,
                engine_name: "none",
                engine_version: 0,
                api_version: xr::Version::new(1, 0, 0),
            },
            &extensions,
            &[],
        )?;
        let system = instance
            .system(xr::FormFactor::HEAD_MOUNTED_DISPLAY)
            .or_else(|_| instance.system(xr::FormFactor::HANDHELD_DISPLAY))?;
        let properties = instance.system_properties(system)?;
        let runtime_properties = instance.properties()?;
        let info = OpenXrRuntimeInfo {
            runtime_name: runtime_properties.runtime_name,
            runtime_version: runtime_properties.runtime_version.to_string(),
            openxr_version: xr::Version::new(1, 0, 0).to_string(),
            system_id: Some(system.into_raw()),
            system_name: Some(properties.system_name),
            vendor_id: Some(properties.vendor_id),
            position_tracking: Some(properties.tracking_properties.position_tracking),
            orientation_tracking: Some(properties.tracking_properties.orientation_tracking),
            original_error: None,
        };
        let view_configuration = instance
            .enumerate_view_configurations(system)?
            .into_iter()
            .find(|value| {
                matches!(
                    *value,
                    xr::ViewConfigurationType::PRIMARY_MONO
                        | xr::ViewConfigurationType::PRIMARY_STEREO
                )
            })
            .ok_or_else(|| eyre!("OpenXR Runtime 未提供 primary view configuration"))?;
        let actions = create_actions(&instance)?;
        let profile_catalog = load_profile_catalog()?;
        let pose_binding_error = suggest_bindings(&instance, &actions, config, &profile_catalog)?;
        let (session, _, _) = unsafe {
            instance.create_session::<xr::Headless>(system, &xr::headless::SessionCreateInfo {})?
        };
        session.attach_action_sets(&[&actions.set])?;
        let reference_space =
            session.create_reference_space(xr::ReferenceSpaceType::LOCAL, xr::Posef::IDENTITY)?;
        let pose_space = config
            .selected
            .as_ref()
            .filter(|selected| HAND_PATHS.contains(&selected.user_path.as_str()))
            .and_then(|selected| instance.string_to_path(&selected.user_path).ok())
            .and_then(|path| {
                actions
                    .pose
                    .create_space(&session, path, xr::Posef::IDENTITY)
                    .ok()
            });
        Ok((
            Self {
                instance,
                session,
                actions,
                reference_space,
                pose_space,
                running: false,
                event_buffer: xr::EventDataBuffer::new(),
                view_configuration,
                pose_binding_error,
                last_pose_error: None,
                profile_catalog,
            },
            info,
        ))
    }

    fn poll_events(&mut self) -> bool {
        let mut profile_changed = false;
        while let Ok(Some(event)) = self.instance.poll_event(&mut self.event_buffer) {
            match event {
                xr::Event::SessionStateChanged(value) => match value.state() {
                    xr::SessionState::READY => {
                        if self.session.begin(self.view_configuration).is_ok() {
                            self.running = true;
                        }
                    }
                    xr::SessionState::STOPPING => {
                        let _ = self.session.end();
                        self.running = false;
                    }
                    xr::SessionState::EXITING | xr::SessionState::LOSS_PENDING => {
                        self.running = false
                    }
                    _ => {}
                },
                xr::Event::InteractionProfileChanged(_) => profile_changed = true,
                _ => {}
            }
        }
        profile_changed
    }

    fn sources(&self) -> Vec<OpenXrInputSource> {
        let native_devices = monado_role_devices().unwrap_or_default();
        USER_PATHS
            .into_iter()
            .filter_map(|user_path| {
                let path = self.instance.string_to_path(user_path).ok()?;
                let profile = self.session.current_interaction_profile(path).ok()?;
                if profile == xr::Path::NULL {
                    return None;
                }
                let interaction_profile = self.instance.path_to_string(profile).ok();
                let native =
                    role_for_user_path(user_path).and_then(|role| native_devices.get(role));
                let localized_name = self
                    .session
                    .input_source_localized_name(
                        path,
                        xr::InputSourceLocalizedNameFlags::USER_PATH
                            | xr::InputSourceLocalizedNameFlags::INTERACTION_PROFILE,
                    )
                    .ok();
                Some(OpenXrInputSource {
                    source_id: format!("openxr:{user_path}"),
                    user_path: user_path.into(),
                    interaction_profile: interaction_profile.clone(),
                    localized_name,
                    runtime_device_name: native.map(|device| device.name.clone()),
                    runtime_serial: native.and_then(|device| device.serial.clone()),
                    pose_capable: native.is_some_and(|device| {
                        device.supports_position || device.supports_orientation
                    }) || self
                        .actions
                        .pose
                        .bound_sources(&self.session)
                        .unwrap_or_default()
                        .into_iter()
                        .filter_map(|path| self.instance.path_to_string(path).ok())
                        .any(|path| path.starts_with(user_path)),
                    action_capable: true,
                    active: true,
                    available_components: interaction_profile
                        .as_deref()
                        .map(|profile| {
                            profile_components(&self.profile_catalog, profile, user_path)
                        })
                        .unwrap_or_default(),
                    original_error: self
                        .last_pose_error
                        .clone()
                        .or_else(|| self.pose_binding_error.clone()),
                })
            })
            .collect()
    }

    fn binding_states(
        &self,
        config: &SourceConfig,
        latest: Option<&ControlInputFrame>,
    ) -> Vec<InputBindingState> {
        let configured = config
            .selected
            .as_ref()
            .and_then(|selected| selected.interaction_profile.as_ref())
            .and_then(|profile| config.bindings_by_profile.get(profile));
        let mut states = BOOLEAN_ACTIONS
            .into_iter()
            .map(|action| {
                let mut state = binding_state(
                    action,
                    ActionType::Boolean,
                    configured,
                    self.actions.booleans[action].bound_sources(&self.session),
                    &self.session,
                    &self.instance,
                );
                state.value = semantic_value(latest, action);
                state
            })
            .collect::<Vec<_>>();
        for action in DIRECTION_ACTIONS {
            let kind = configured
                .and_then(|values| values.iter().find(|value| value.action == action))
                .map(|value| value.action_type)
                .unwrap_or(ActionType::Float);
            let result = match kind {
                ActionType::Float => self.actions.directions[action]
                    .float
                    .bound_sources(&self.session),
                ActionType::Boolean => {
                    let mut paths = self.actions.directions[action]
                        .negative
                        .bound_sources(&self.session)
                        .unwrap_or_default();
                    paths.extend(
                        self.actions.directions[action]
                            .positive
                            .bound_sources(&self.session)
                            .unwrap_or_default(),
                    );
                    Ok(paths)
                }
            };
            let mut state = binding_state(
                action,
                kind,
                configured,
                result,
                &self.session,
                &self.instance,
            );
            state.value = semantic_value(latest, action);
            states.push(state);
        }
        states
    }

    fn sample(
        &mut self,
        selected: Option<&SelectedSource>,
        configured: &BTreeMap<String, Vec<ActionBinding>>,
        sequence: &mut u64,
    ) -> Result<RuntimeSample> {
        let profile_changed = self.poll_events();
        if !self.running {
            return Ok(RuntimeSample {
                frame: None,
                profile_changed,
            });
        }
        self.session
            .sync_actions(&[xr::ActiveActionSet::new(&self.actions.set)])?;
        let Some(selected) = selected else {
            return Ok(RuntimeSample {
                frame: None,
                profile_changed,
            });
        };
        *sequence += 1;
        let sample_time = self.instance.now()?;
        let source_time_ns = sample_time.as_nanos();
        let received_time_ns = now_ns();
        let source_id = format!("openxr:{}", selected.user_path);
        let subaction_path = self.instance.string_to_path(&selected.user_path)?;
        let binding_config = selected
            .interaction_profile
            .as_ref()
            .and_then(|profile| configured.get(profile));
        let boolean = |name: &str| -> Result<BooleanActionSample> {
            let state = self.actions.booleans[name].state(&self.session, subaction_path)?;
            let invert = binding_config
                .and_then(|values| values.iter().find(|value| value.action == name))
                .is_some_and(|value| value.invert);
            Ok(BooleanActionSample {
                is_active: state.is_active,
                changed_since_last_sync: state.changed_since_last_sync,
                value: boolean_value(state.is_active, state.current_state, invert),
            })
        };
        let direction = |name: &str| -> Result<FloatActionSample> {
            let family = &self.actions.directions[name];
            let binding =
                binding_config.and_then(|values| values.iter().find(|value| value.action == name));
            let invert = binding.is_some_and(|value| value.invert);
            let (active, changed, value) =
                if binding.map(|value| value.action_type) == Some(ActionType::Boolean) {
                    let negative = family.negative.state(&self.session, subaction_path)?;
                    let positive = family.positive.state(&self.session, subaction_path)?;
                    (
                        negative.is_active || positive.is_active,
                        negative.changed_since_last_sync || positive.changed_since_last_sync,
                        direction_value(negative.current_state, positive.current_state, invert),
                    )
                } else {
                    let state = family.float.state(&self.session, subaction_path)?;
                    (
                        state.is_active,
                        state.changed_since_last_sync,
                        if invert {
                            -f64::from(state.current_state)
                        } else {
                            f64::from(state.current_state)
                        },
                    )
                };
            Ok(FloatActionSample {
                is_active: active,
                changed_since_last_sync: changed,
                value,
            })
        };
        let input = ControlInputFrame {
            schema_version: SCHEMA_VERSION,
            sequence: *sequence,
            source_time_ns,
            received_time_ns,
            source_id: source_id.clone(),
            control_active: boolean("control_active")?,
            confirm_origin: boolean("confirm_origin")?,
            primary_tool_active: boolean("primary_tool_active")?,
            move_forward_back: direction("move_forward_back")?,
            move_left_right: direction("move_left_right")?,
            move_up_down: direction("move_up_down")?,
            front_pitch: direction("front_pitch")?,
            horizontal_arc: direction("horizontal_arc")?,
        };
        let pose_location = self
            .pose_space
            .as_ref()
            .map(|space| space.locate(&self.reference_space, sample_time));
        let pose = match pose_location {
            Some(Ok(location)) => {
                self.last_pose_error = None;
                Some(AbsolutePoseFrame {
                    schema_version: SCHEMA_VERSION,
                    sequence: *sequence,
                    source_time_ns,
                    received_time_ns,
                    source_id,
                    user_path: selected.user_path.clone(),
                    reference_space: "local".into(),
                    position_m: [
                        location.pose.position.x.into(),
                        location.pose.position.y.into(),
                        location.pose.position.z.into(),
                    ],
                    orientation_xyzw: [
                        location.pose.orientation.x.into(),
                        location.pose.orientation.y.into(),
                        location.pose.orientation.z.into(),
                        location.pose.orientation.w.into(),
                    ],
                    flags: PoseFlags {
                        position_valid: location
                            .location_flags
                            .contains(xr::SpaceLocationFlags::POSITION_VALID),
                        position_tracked: location
                            .location_flags
                            .contains(xr::SpaceLocationFlags::POSITION_TRACKED),
                        orientation_valid: location
                            .location_flags
                            .contains(xr::SpaceLocationFlags::ORIENTATION_VALID),
                        orientation_tracked: location
                            .location_flags
                            .contains(xr::SpaceLocationFlags::ORIENTATION_TRACKED),
                    },
                })
            }
            Some(Err(error)) => {
                self.last_pose_error = Some(error.to_string());
                None
            }
            None => None,
        };
        Ok(RuntimeSample {
            frame: Some((pose, input)),
            profile_changed,
        })
    }
}

fn boolean_value(active: bool, current: bool, invert: bool) -> bool {
    active && (current ^ invert)
}

fn direction_value(negative: bool, positive: bool, invert: bool) -> f64 {
    let value = f64::from(positive) - f64::from(negative);
    if invert { -value } else { value }
}

fn role_for_user_path(user_path: &str) -> Option<&'static str> {
    match user_path {
        "/user/hand/left" => Some("left"),
        "/user/hand/right" => Some("right"),
        "/user/gamepad" => Some("gamepad"),
        _ => None,
    }
}

fn monado_role_devices() -> Result<BTreeMap<String, RuntimeDeviceInfo>> {
    const NAME_PROPERTY: c_int = 0;
    const SERIAL_PROPERTY: c_int = 1;
    const SUPPORTS_POSITION_PROPERTY: c_int = 3;
    const SUPPORTS_ORIENTATION_PROPERTY: c_int = 4;

    unsafe {
        let library = libloading::Library::new("libmonado.so")?;
        let create: MndRootCreate = *library.get(b"mnd_root_create\0")?;
        let destroy: MndRootDestroy = *library.get(b"mnd_root_destroy\0")?;
        let get_role: MndGetDeviceFromRole = *library.get(b"mnd_root_get_device_from_role\0")?;
        let get_string: MndGetDeviceInfoString =
            *library.get(b"mnd_root_get_device_info_string\0")?;
        let get_bool: MndGetDeviceInfoBool = *library.get(b"mnd_root_get_device_info_bool\0")?;
        let mut root = std::ptr::null_mut();
        if create(&mut root) != 0 || root.is_null() {
            return Err(eyre!("libmonado 无法连接 Runtime"));
        }
        let mut devices = BTreeMap::new();
        for role in ["left", "right", "gamepad"] {
            let role_c = CString::new(role)?;
            let mut index = -1;
            if get_role(root, role_c.as_ptr(), &mut index) != 0 || index < 0 {
                continue;
            }
            let index = index as u32;
            let name = mnd_string(root, index, NAME_PROPERTY, get_string)
                .unwrap_or_else(|| role.to_owned());
            let serial = mnd_string(root, index, SERIAL_PROPERTY, get_string)
                .filter(|value| !value.is_empty());
            let supports_position = mnd_bool(root, index, SUPPORTS_POSITION_PROPERTY, get_bool);
            let supports_orientation =
                mnd_bool(root, index, SUPPORTS_ORIENTATION_PROPERTY, get_bool);
            devices.insert(
                role.to_owned(),
                RuntimeDeviceInfo {
                    name,
                    serial,
                    supports_position,
                    supports_orientation,
                },
            );
        }
        destroy(&mut root);
        Ok(devices)
    }
}

unsafe fn mnd_string(
    root: *mut c_void,
    index: u32,
    property: c_int,
    getter: MndGetDeviceInfoString,
) -> Option<String> {
    let mut value = std::ptr::null();
    if unsafe { getter(root, index, property, &mut value) } != 0 || value.is_null() {
        return None;
    }
    Some(
        unsafe { CStr::from_ptr(value) }
            .to_string_lossy()
            .into_owned(),
    )
}

unsafe fn mnd_bool(
    root: *mut c_void,
    index: u32,
    property: c_int,
    getter: MndGetDeviceInfoBool,
) -> bool {
    let mut value = false;
    unsafe { getter(root, index, property, &mut value) == 0 && value }
}

fn create_actions(instance: &xr::Instance) -> Result<RuntimeActions> {
    let set = instance.create_action_set("robot_control", "Robot control", 0)?;
    let user_paths = USER_PATHS
        .into_iter()
        .map(|path| instance.string_to_path(path))
        .collect::<xr::Result<Vec<_>>>()?;
    let hand_paths = HAND_PATHS
        .into_iter()
        .map(|path| instance.string_to_path(path))
        .collect::<xr::Result<Vec<_>>>()?;
    let pose = set.create_action::<xr::Posef>("absolute_pose", "Absolute pose", &hand_paths)?;
    let discovery = set.create_action::<bool>(
        "input_source_discovery",
        "Input source discovery",
        &user_paths,
    )?;
    let mut booleans = BTreeMap::new();
    for name in BOOLEAN_ACTIONS {
        booleans.insert(
            name.into(),
            set.create_action::<bool>(name, name, &user_paths)?,
        );
    }
    let mut directions = BTreeMap::new();
    for name in DIRECTION_ACTIONS {
        directions.insert(
            name.into(),
            DirectionActions {
                float: set.create_action::<f32>(name, name, &user_paths)?,
                negative: set.create_action::<bool>(
                    &format!("{name}_negative"),
                    &format!("{name} negative"),
                    &user_paths,
                )?,
                positive: set.create_action::<bool>(
                    &format!("{name}_positive"),
                    &format!("{name} positive"),
                    &user_paths,
                )?,
            },
        );
    }
    Ok(RuntimeActions {
        set,
        pose,
        discovery,
        booleans,
        directions,
    })
}

fn suggest_bindings(
    instance: &xr::Instance,
    actions: &RuntimeActions,
    config: &SourceConfig,
    catalog: &InteractionProfileCatalog,
) -> Result<Option<String>> {
    let selected_profile = config
        .selected
        .as_ref()
        .and_then(|selected| selected.interaction_profile.as_deref());
    let mut errors = Vec::new();
    for profile in &catalog.profiles {
        if profile.profile != SIMPLE_CONTROLLER_PROFILE
            && selected_profile != Some(profile.profile.as_str())
        {
            continue;
        }
        let mut bindings = Vec::new();
        for user_path in profile
            .user_paths
            .iter()
            .filter(|path| USER_PATHS.contains(&path.as_str()))
        {
            if let Some(component) = profile
                .components
                .iter()
                .find(|component| component.action_type == ActionType::Boolean)
            {
                bindings.push(xr::Binding::new(
                    &actions.discovery,
                    instance.string_to_path(&format!("{user_path}{}", component.suffix))?,
                ));
            }
            if HAND_PATHS.contains(&user_path.as_str())
                && let Some(suffix) = profile.pose_suffixes.first()
            {
                bindings.push(xr::Binding::new(
                    &actions.pose,
                    instance.string_to_path(&format!("{user_path}{suffix}"))?,
                ));
            }
        }
        if selected_profile == Some(profile.profile.as_str()) {
            append_configured_bindings(
                instance,
                actions,
                config.bindings_by_profile.get(&profile.profile),
                &mut bindings,
            )?;
        }
        if bindings.is_empty() {
            continue;
        }
        if let Err(error) = instance.suggest_interaction_profile_bindings(
            instance.string_to_path(&profile.profile)?,
            &bindings,
        ) && selected_profile == Some(profile.profile.as_str())
        {
            errors.push(format!("{}: {error}", profile.profile));
        }
    }

    Ok((!errors.is_empty()).then(|| errors.join("; ")))
}

fn append_configured_bindings<'a>(
    instance: &xr::Instance,
    actions: &'a RuntimeActions,
    configured: Option<&Vec<ActionBinding>>,
    bindings: &mut Vec<xr::Binding<'a>>,
) -> Result<()> {
    if let Some(configured) = configured {
        for binding in configured {
            match (
                binding.action_type,
                BOOLEAN_ACTIONS.contains(&binding.action.as_str()),
                DIRECTION_ACTIONS.contains(&binding.action.as_str()),
            ) {
                (ActionType::Boolean, true, _) if binding.component_paths.len() == 1 => bindings
                    .push(xr::Binding::new(
                        &actions.booleans[&binding.action],
                        instance.string_to_path(&binding.component_paths[0])?,
                    )),
                (ActionType::Float, _, true) if binding.component_paths.len() == 1 => bindings
                    .push(xr::Binding::new(
                        &actions.directions[&binding.action].float,
                        instance.string_to_path(&binding.component_paths[0])?,
                    )),
                (ActionType::Boolean, _, true) if binding.component_paths.len() == 2 => {
                    bindings.push(xr::Binding::new(
                        &actions.directions[&binding.action].negative,
                        instance.string_to_path(&binding.component_paths[0])?,
                    ));
                    bindings.push(xr::Binding::new(
                        &actions.directions[&binding.action].positive,
                        instance.string_to_path(&binding.component_paths[1])?,
                    ));
                }
                _ => {}
            }
        }
    }
    Ok(())
}

fn validate_bindings(bindings: &[ActionBinding]) -> Result<(), String> {
    for binding in bindings {
        let boolean = BOOLEAN_ACTIONS.contains(&binding.action.as_str());
        let direction = DIRECTION_ACTIONS.contains(&binding.action.as_str());
        let valid = (boolean
            && binding.action_type == ActionType::Boolean
            && binding.component_paths.len() == 1)
            || (direction
                && binding.action_type == ActionType::Float
                && binding.component_paths.len() == 1)
            || (direction
                && binding.action_type == ActionType::Boolean
                && binding.component_paths.len() == 2);
        if !valid {
            return Err(format!(
                "功能 Action {} 的类型或 component 数量不匹配",
                binding.action
            ));
        }
    }
    Ok(())
}

fn semantic_value(frame: Option<&ControlInputFrame>, action: &str) -> f64 {
    let Some(frame) = frame else { return 0.0 };
    match action {
        "control_active" => f64::from(frame.control_active.value),
        "confirm_origin" => f64::from(frame.confirm_origin.value),
        "primary_tool_active" => f64::from(frame.primary_tool_active.value),
        "move_forward_back" => frame.move_forward_back.value,
        "move_left_right" => frame.move_left_right.value,
        "move_up_down" => frame.move_up_down.value,
        "front_pitch" => frame.front_pitch.value,
        "horizontal_arc" => frame.horizontal_arc.value,
        _ => 0.0,
    }
}

fn load_profile_catalog() -> Result<InteractionProfileCatalog> {
    let path = std::env::var_os("OPENXR_PROFILE_CATALOG")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/opt/openxr/share/robot-arm/interaction-profiles.json"));
    Ok(serde_json::from_slice(&fs::read(&path).with_context(
        || {
            format!(
                "读取 OpenXR interaction profile 目录失败：{}",
                path.display()
            )
        },
    )?)?)
}

fn profile_components(
    catalog: &InteractionProfileCatalog,
    profile: &str,
    user_path: &str,
) -> Vec<InputComponentInfo> {
    catalog
        .profiles
        .iter()
        .find(|definition| {
            definition.profile == profile
                && definition.user_paths.iter().any(|path| path == user_path)
        })
        .into_iter()
        .flat_map(|definition| &definition.components)
        .map(|component| InputComponentInfo {
            path: format!("{user_path}{}", component.suffix),
            action_type: component.action_type,
            localized_name: component.localized_name.clone(),
            definition_source: "Runtime image interaction profile catalog".into(),
        })
        .collect()
}

fn binding_state(
    action: &str,
    action_type: ActionType,
    configured: Option<&Vec<ActionBinding>>,
    result: xr::Result<Vec<xr::Path>>,
    session: &xr::Session<xr::Headless>,
    instance: &xr::Instance,
) -> InputBindingState {
    let configured_components = configured
        .and_then(|values| values.iter().find(|value| value.action == action))
        .map(|value| value.component_paths.clone())
        .unwrap_or_default();
    let invert = configured
        .and_then(|values| values.iter().find(|value| value.action == action))
        .is_some_and(|value| value.invert);
    match result {
        Ok(paths) => {
            let bound_sources = paths
                .iter()
                .filter_map(|path| instance.path_to_string(*path).ok())
                .collect::<Vec<_>>();
            let flags = xr::InputSourceLocalizedNameFlags::USER_PATH
                | xr::InputSourceLocalizedNameFlags::INTERACTION_PROFILE
                | xr::InputSourceLocalizedNameFlags::COMPONENT;
            let localized_names = paths
                .iter()
                .filter_map(|path| session.input_source_localized_name(*path, flags).ok())
                .collect();
            InputBindingState {
                action: action.into(),
                action_type,
                invert,
                configured_components,
                active: !bound_sources.is_empty(),
                applicable: true,
                bound_sources,
                localized_names,
                value: 0.0,
                original_error: None,
            }
        }
        Err(error) => InputBindingState {
            action: action.into(),
            action_type,
            invert,
            configured_components,
            bound_sources: vec![],
            localized_names: vec![],
            active: false,
            value: 0.0,
            applicable: false,
            original_error: Some(error.to_string()),
        },
    }
}

fn publish_discovery(node: &mut DoraNode, source: &mut SourceNode) -> Result<()> {
    source.host_devices = scan_host_devices(Path::new("/sys/bus/usb/devices"));
    let discovery = source.discovery_state();
    send(node, "service_state", &discovery.service)?;
    send(node, "discovery_state", &discovery)
}
fn probe_runtime() -> OpenXrRuntimeInfo {
    probe_runtime_inner().unwrap_or_else(|error| OpenXrRuntimeInfo {
        openxr_version: xr::Version::new(1, 0, 0).to_string(),
        original_error: Some(error.to_string()),
        ..Default::default()
    })
}
fn probe_runtime_inner() -> Result<OpenXrRuntimeInfo> {
    let entry = unsafe { xr::Entry::load()? };
    let instance = entry.create_instance(
        &xr::ApplicationInfo {
            application_name: "robot-arm-openxr-probe",
            application_version: 1,
            engine_name: "none",
            engine_version: 0,
            api_version: xr::Version::new(1, 0, 0),
        },
        &xr::ExtensionSet::default(),
        &[],
    )?;
    let runtime = instance.properties()?;
    let mut info = OpenXrRuntimeInfo {
        runtime_name: runtime.runtime_name,
        runtime_version: runtime.runtime_version.to_string(),
        openxr_version: xr::Version::new(1, 0, 0).to_string(),
        ..Default::default()
    };
    match instance
        .system(xr::FormFactor::HEAD_MOUNTED_DISPLAY)
        .or_else(|_| instance.system(xr::FormFactor::HANDHELD_DISPLAY))
    {
        Ok(system) => {
            let value = instance.system_properties(system)?;
            info.system_id = Some(system.into_raw());
            info.system_name = Some(value.system_name);
            info.vendor_id = Some(value.vendor_id);
            info.position_tracking = Some(value.tracking_properties.position_tracking);
            info.orientation_tracking = Some(value.tracking_properties.orientation_tracking);
        }
        Err(error) => info.original_error = Some(error.to_string()),
    }
    Ok(info)
}

fn scan_host_devices(root: &Path) -> Vec<HostDeviceInfo> {
    let Ok(entries) = fs::read_dir(root) else {
        return vec![];
    };
    let mut values = entries
        .flatten()
        .filter_map(|entry| usb_device(entry.path()))
        .collect::<Vec<_>>();
    values.sort_by(|left, right| left.bus_path.cmp(&right.bus_path));
    values
}
fn usb_device(path: PathBuf) -> Option<HostDeviceInfo> {
    let vendor_id = read_trimmed(path.join("idVendor"))?;
    let product_id = read_trimmed(path.join("idProduct"))?;
    let device_nodes = match (
        read_trimmed(path.join("busnum")),
        read_trimmed(path.join("devnum")),
    ) {
        (Some(bus), Some(device)) => vec![format!(
            "/dev/bus/usb/{:0>3}/{:0>3}",
            bus.trim_start_matches('0'),
            device.trim_start_matches('0')
        )],
        _ => vec![],
    };
    let interfaces = fs::read_dir(&path)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|entry| {
            let name = entry.file_name().to_string_lossy().into_owned();
            name.contains(':').then_some(name)
        })
        .collect();
    Some(HostDeviceInfo {
        connection_type: "usb".into(),
        bus_path: path
            .file_name()
            .map(|name| name.to_string_lossy().into_owned()),
        vendor_id: Some(vendor_id),
        product_id: Some(product_id),
        manufacturer: read_trimmed(path.join("manufacturer")),
        product: read_trimmed(path.join("product")),
        serial: read_trimmed(path.join("serial")),
        interfaces,
        device_nodes,
        present: true,
    })
}
fn read_trimmed(path: PathBuf) -> Option<String> {
    fs::read_to_string(path)
        .ok()
        .map(|value| value.trim().to_owned())
        .filter(|value| !value.is_empty())
}
fn send<T: Serialize>(node: &mut DoraNode, output: &str, value: &T) -> Result<()> {
    node.send_output(
        DataId::from(output.to_owned()),
        MetadataParameters::default(),
        to_arrow(value)?,
    )?;
    Ok(())
}
fn now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|value| value.as_nanos() as i64)
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn missing_sysfs_root_is_empty() {
        assert!(scan_host_devices(Path::new("/path/that/does/not/exist")).is_empty());
    }
    #[test]
    fn sysfs_fixture_preserves_missing_strings_and_device_identity() {
        let root = std::env::temp_dir().join(format!("openxr-host-fixture-{}", now_ns()));
        for (name, vendor, product) in [("1-2", "1234", "abcd"), ("1-3", "1234", "abcd")] {
            let device = root.join(name);
            fs::create_dir_all(device.join(format!("{name}:1.0"))).unwrap();
            fs::write(device.join("idVendor"), vendor).unwrap();
            fs::write(device.join("idProduct"), product).unwrap();
            fs::write(device.join("busnum"), "1").unwrap();
            fs::write(device.join("devnum"), if name == "1-2" { "7" } else { "8" }).unwrap();
        }
        fs::write(root.join("1-2/product"), "Input device").unwrap();
        let devices = scan_host_devices(&root);
        assert_eq!(devices.len(), 2);
        assert_eq!(devices[0].bus_path.as_deref(), Some("1-2"));
        assert_eq!(devices[0].product.as_deref(), Some("Input device"));
        assert_eq!(devices[1].product, None);
        assert_eq!(devices[1].serial, None);
        fs::remove_dir_all(root.join("1-2")).unwrap();
        let after_remove = scan_host_devices(&root);
        assert_eq!(after_remove.len(), 1);
        assert_eq!(after_remove[0].bus_path.as_deref(), Some("1-3"));
        fs::remove_dir_all(root).unwrap();
    }
    #[test]
    fn runtime_probe_keeps_loader_error() {
        assert!(!probe_runtime().openxr_version.is_empty());
    }
    #[test]
    fn semantic_direction_accepts_float_or_boolean_pair() {
        for action in DIRECTION_ACTIONS {
            assert!(
                validate_bindings(&[ActionBinding {
                    action: action.into(),
                    action_type: ActionType::Float,
                    component_paths: vec!["/user/input/value".into()],
                    invert: false
                }])
                .is_ok()
            );
            assert!(
                validate_bindings(&[ActionBinding {
                    action: action.into(),
                    action_type: ActionType::Boolean,
                    component_paths: vec![
                        "/user/input/negative".into(),
                        "/user/input/positive".into()
                    ],
                    invert: false
                }])
                .is_ok()
            );
        }
        for action in BOOLEAN_ACTIONS {
            assert!(
                validate_bindings(&[ActionBinding {
                    action: action.into(),
                    action_type: ActionType::Boolean,
                    component_paths: vec!["/user/input/click".into()],
                    invert: false,
                }])
                .is_ok()
            );
        }
    }
    #[test]
    fn semantic_values_apply_inversion_without_activating_missing_input() {
        assert!(!boolean_value(false, false, true));
        assert!(!boolean_value(false, true, false));
        assert!(boolean_value(true, false, true));
        assert_eq!(direction_value(false, true, false), 1.0);
        assert_eq!(direction_value(true, false, false), -1.0);
        assert_eq!(direction_value(false, true, true), -1.0);
        assert_eq!(direction_value(true, true, false), 0.0);
    }
    #[test]
    fn binding_shape_rejects_wrong_action_types_and_component_counts() {
        for binding in [
            ActionBinding {
                action: "control_active".into(),
                action_type: ActionType::Float,
                component_paths: vec!["/user/input/value".into()],
                invert: false,
            },
            ActionBinding {
                action: "move_up_down".into(),
                action_type: ActionType::Boolean,
                component_paths: vec!["/user/input/click".into()],
                invert: false,
            },
            ActionBinding {
                action: "unknown".into(),
                action_type: ActionType::Boolean,
                component_paths: vec!["/user/input/click".into()],
                invert: false,
            },
        ] {
            assert!(validate_bindings(&[binding]).is_err());
        }
    }
    #[test]
    fn interaction_components_come_from_the_runtime_catalog() {
        let catalog = InteractionProfileCatalog {
            profiles: vec![InteractionProfileDefinition {
                profile: "/interaction_profiles/example/controller".into(),
                user_paths: vec!["/user/hand/right".into()],
                components: vec![InteractionComponentDefinition {
                    suffix: "/input/example/click".into(),
                    action_type: ActionType::Boolean,
                    localized_name: Some("Example".into()),
                }],
                pose_suffixes: vec!["/input/pose/pose".into()],
            }],
        };
        let components = profile_components(
            &catalog,
            "/interaction_profiles/example/controller",
            "/user/hand/right",
        );
        assert_eq!(components.len(), 1);
        assert_eq!(components[0].path, "/user/hand/right/input/example/click");
        assert_eq!(components[0].localized_name.as_deref(), Some("Example"));
        assert!(
            profile_components(
                &catalog,
                "/interaction_profiles/example/controller",
                "/user/hand/left"
            )
            .is_empty()
        );
    }
    #[test]
    fn openxr_user_paths_map_to_monado_roles_without_device_names() {
        assert_eq!(role_for_user_path("/user/hand/left"), Some("left"));
        assert_eq!(role_for_user_path("/user/hand/right"), Some("right"));
        assert_eq!(role_for_user_path("/user/gamepad"), Some("gamepad"));
    }
    #[test]
    fn host_and_runtime_sources_are_not_paired_by_order() {
        let source = SourceNode {
            config_path: PathBuf::new(),
            config: SourceConfig::default(),
            runtime_info: OpenXrRuntimeInfo::default(),
            runtime: None,
            host_devices: vec![HostDeviceInfo {
                connection_type: "usb".into(),
                bus_path: Some("1-1".into()),
                vendor_id: None,
                product_id: None,
                manufacturer: None,
                product: None,
                serial: None,
                interfaces: vec![],
                device_nodes: vec![],
                present: true,
            }],
            sequence: 0,
            latest_pose: None,
            latest_input: None,
            observations: StreamObservations::default(),
            last_error: None,
            simulation: None,
            simulation_state: InputSimulationState::default(),
        };
        assert!(
            source
                .discovery_state()
                .confirmed_host_association
                .is_none()
        );
    }
}
