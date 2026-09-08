use std::{
    collections::BTreeMap,
    path::PathBuf,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result};
use fashionstar_uart::{
    FashionStarBus, InternalParameters, Monitor, PositionCommand, degrees_to_tenths,
};
use json_config_store::{load_or_default, save};
use robot_arm_messages::{
    ActionFeedback, ActuatorTelemetry, ArmCommand, ArmState, ArmTelemetry, ConnectionFieldSchema,
    ExecutionEndpoint, ExecutionInfo, ExecutionRequest, ExecutionTransportState, FeedbackSource,
    ModelAssetRequest, NumericFieldSchema, ParameterValue, RequestAction, RequestResult,
    SCHEMA_VERSION, ServiceState, from_arrow, to_arrow,
};
use serde::{Deserialize, Serialize};
use stararm_102_model::{
    DEFAULT_JOINTS_RAD, GRIPPER_DRIVE_JOINT_CLOSED_RAD, JOINTS, MODEL_REVISION, ModelCatalog,
    validate_command,
};

mod gripper_feedback;
use gripper_feedback::GripperFeedbackController;

const ADAPTER_REVISION: &str = "stararm-102-fashionstar-v1";
const SERVO_IDS: [u8; 7] = [0, 1, 2, 3, 4, 5, 6];
const ALL_SERVOS_ID: u8 = 0xff;
const MOTION_TIME_MS: u32 = 100;
const ACCELERATION_TIME_MS: u16 = 50;
const DECELERATION_TIME_MS: u16 = 50;
const GRIPPER_COMMAND_POWER_MW: u16 = 2_000;
const GRIPPER_IDLE_POWER_MW: u16 = 400;
const CONFIG_SCHEMA_VERSION: u32 = 1;
const DEFAULT_FEEDBACK_INTERVAL_MS: u64 = 100;

fn default_feedback_interval_ms() -> u64 {
    DEFAULT_FEEDBACK_INTERVAL_MS
}

fn default_gripper_strength_percent() -> f64 {
    50.0 // User-requested measured feedback target, not a power command.
}

fn changed_servo_commands(
    previous: Option<&[PositionCommand; 7]>,
    commands: &[PositionCommand; 7],
) -> Vec<PositionCommand> {
    commands
        .iter()
        .enumerate()
        .filter_map(|(index, command)| {
            (previous.is_none_or(|last| last[index] != *command)).then_some(*command)
        })
        .collect()
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut execution = StarArmExecution::load()?;
    let model_root = std::env::var_os("STARARM_MODEL_ASSETS")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            PathBuf::from("/opt/devices/stararm-102/vendor/ROS2_HUMBLE/src/stararm102_description")
        });
    let model = ModelCatalog::load(model_root).map_err(eyre::Report::msg)?;
    publish_snapshot(&mut node, &execution, &model)?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => match id.as_str() {
                "arm_command" => {
                    let command: ArmCommand =
                        from_arrow(data.as_array()).context("decode arm_command")?;
                    execution.apply_command(command);
                    publish_transport(&mut node, &execution)?;
                    publish_state(&mut node, &execution)?;
                }
                "execution_request" => {
                    let request: ExecutionRequest =
                        from_arrow(data.as_array()).context("decode execution_request")?;
                    let result = execution.handle_request(request);
                    send(&mut node, "request_result", &result)?;
                    publish_transport(&mut node, &execution)?;
                    publish_state(&mut node, &execution)?;
                }
                "model_asset_request" => {
                    let request: ModelAssetRequest =
                        from_arrow(data.as_array()).context("decode model_asset_request")?;
                    send(
                        &mut node,
                        "model_asset_response",
                        &model.asset_response(request),
                    )?;
                }
                "tick" => {
                    if execution.poll_hardware() {
                        publish_transport(&mut node, &execution)?;
                        publish_state(&mut node, &execution)?;
                    }
                }
                "snapshot" => publish_snapshot(&mut node, &execution, &model)?,
                _ => {}
            },
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}

struct StarArmBus {
    bus: FashionStarBus,
    last_commands: Option<[PositionCommand; 7]>,
}

impl StarArmBus {
    fn open(path: &str) -> Result<(Self, Vec<ParameterValue>, ArmState, ArmTelemetry), String> {
        let mut bus =
            FashionStarBus::open(path).map_err(|error| format!("无法打开串口 {path}：{error}"))?;
        for id in SERVO_IDS {
            bus.ping(id)
                .map_err(|error| format!("Ping 舵机 ID {id} 失败：{error}"))?;
        }
        let parameters = read_parameters(&mut bus);
        let monitors = read_sorted_monitors(&mut bus)?;
        let state = state_from_monitors(&monitors, 1);
        let telemetry = telemetry_from_monitors(&monitors, 1, state.sample_time_ns, None);
        Ok((
            Self {
                bus,
                last_commands: None,
            },
            parameters,
            state,
            telemetry,
        ))
    }

    fn read_state(&mut self, sequence: u64) -> Result<(ArmState, ArmTelemetry), String> {
        let monitors = read_sorted_monitors(&mut self.bus)?;
        let state = state_from_monitors(&monitors, sequence);
        let telemetry = telemetry_from_monitors(
            &monitors,
            sequence,
            state.sample_time_ns,
            self.last_commands
                .as_ref()
                .map(|commands| commands[6].power_mw),
        );
        Ok((state, telemetry))
    }

    fn read_parameters(&mut self) -> Vec<ParameterValue> {
        read_parameters(&mut self.bus)
    }

    fn write_gains(&mut self, id: u8, kp: u16, hold_kp: u16) -> Result<(), String> {
        let mut parameters = self
            .bus
            .read_internal_parameters(id)
            .map_err(|error| error.to_string())?;
        parameters.kp = kp;
        parameters.hold_kp = hold_kp;
        self.bus
            .write_internal_parameters(parameters)
            .map_err(|error| error.to_string())?;
        let actual = self
            .bus
            .read_internal_parameters(id)
            .map_err(|error| error.to_string())?;
        if actual.kp != kp || actual.hold_kp != hold_kp {
            return Err(format!(
                "舵机 ID {id} 参数回读不一致：kp={} hold_kp={}",
                actual.kp, actual.hold_kp
            ));
        }
        self.last_commands = None;
        Ok(())
    }

    fn set_torque(&mut self, hold: bool) -> Result<(), String> {
        let result = if hold {
            self.bus.hold_torque(ALL_SERVOS_ID)
        } else {
            self.bus.release_torque(ALL_SERVOS_ID)
        };
        result.map_err(|error| error.to_string())?;
        self.last_commands = None;
        Ok(())
    }

    fn write(&mut self, command: &ArmCommand, gripper_power_mw: u16) -> Result<(), String> {
        let mut commands = encode_command(command)?;
        commands[6].power_mw = gripper_power_mw;
        let changed = changed_servo_commands(self.last_commands.as_ref(), &commands);
        if changed.is_empty() {
            return Ok(());
        }
        self.bus
            .write_positions(&changed)
            .map_err(|error| format!("串口写入失败：{error}"))?;
        self.last_commands = Some(commands);
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct ExecutionConfig {
    schema_version: u32,
    config_version: u64,
    selected_endpoint: Option<String>,
    #[serde(default = "default_feedback_interval_ms")]
    feedback_interval_ms: u64,
    #[serde(default = "default_gripper_strength_percent")]
    gripper_strength_percent: f64,
}

impl Default for ExecutionConfig {
    fn default() -> Self {
        Self {
            schema_version: CONFIG_SCHEMA_VERSION,
            config_version: 1,
            selected_endpoint: None,
            feedback_interval_ms: DEFAULT_FEEDBACK_INTERVAL_MS,
            gripper_strength_percent: default_gripper_strength_percent(),
        }
    }
}

struct StarArmExecution {
    config_path: PathBuf,
    config: ExecutionConfig,
    info: ExecutionInfo,
    transport: ExecutionTransportState,
    state: ArmState,
    telemetry: ArmTelemetry,
    bus: Option<StarArmBus>,
    next_sequence: u64,
    last_feedback_poll: Option<Instant>,
    gripper_feedback: GripperFeedbackController,
}

impl StarArmExecution {
    fn load() -> Result<Self> {
        let config_path = std::env::var_os("STARARM_EXECUTION_CONFIG")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("/config/stararm-102-execution.json"));
        let config: ExecutionConfig = load_or_default(&config_path)?;
        if config.schema_version != CONFIG_SCHEMA_VERSION {
            return Err(eyre::eyre!(
                "不支持的执行配置版本 {}",
                config.schema_version
            ));
        }
        let selected_endpoint = config.selected_endpoint.clone();
        let mut execution = Self::with_config(config_path, config);
        if let Some(path) = selected_endpoint {
            let _ = execution.connect(path);
        }
        Ok(execution)
    }

    #[cfg(test)]
    fn new() -> Self {
        Self::with_config(PathBuf::new(), ExecutionConfig::default())
    }

    fn with_config(config_path: PathBuf, config: ExecutionConfig) -> Self {
        let feedback_interval_ms = config.feedback_interval_ms;
        let gripper_strength_percent = config.gripper_strength_percent;
        Self {
            config_path,
            config,
            info: execution_info(),
            transport: ExecutionTransportState {
                schema_version: SCHEMA_VERSION,
                feedback_interval_ms,
                gripper_strength_percent: Some(gripper_strength_percent),
                ..Default::default()
            },
            state: ArmState {
                schema_version: SCHEMA_VERSION,
                sequence: 0,
                sample_time_ns: now_ns(),
                model_revision: MODEL_REVISION.into(),
                joints_rad: DEFAULT_JOINTS_RAD.to_vec(),
                actuators_rad: vec![GRIPPER_DRIVE_JOINT_CLOSED_RAD],
                feedback_source: FeedbackSource::Software,
            },
            telemetry: ArmTelemetry {
                schema_version: SCHEMA_VERSION,
                sequence: 0,
                sample_time_ns: now_ns(),
                model_revision: MODEL_REVISION.into(),
                actuators: vec![],
            },
            bus: None,
            next_sequence: 1,
            last_feedback_poll: None,
            gripper_feedback: GripperFeedbackController::default(),
        }
    }

    fn service_state(&self) -> ServiceState {
        ServiceState {
            schema_version: SCHEMA_VERSION,
            build_version: env!("CARGO_PKG_VERSION").into(),
            config_version: self.config.config_version,
            running: true,
            has_input: self.transport.last_command.is_some()
                || self.transport.latest_request_id.is_some(),
            has_output: true,
            last_error: self.transport.last_error.clone(),
            updated_at_ns: now_ns(),
        }
    }

    fn discover(&mut self) {
        self.transport.discovered_endpoints = serialport::available_ports()
            .map(|ports| {
                ports
                    .into_iter()
                    .map(|port| {
                        let mut properties = BTreeMap::new();
                        if let serialport::SerialPortType::UsbPort(usb) = port.port_type {
                            properties.insert("vid".into(), format!("{:04x}", usb.vid));
                            properties.insert("pid".into(), format!("{:04x}", usb.pid));
                            if let Some(value) = usb.manufacturer {
                                properties.insert("manufacturer".into(), value);
                            }
                            if let Some(value) = usb.product {
                                properties.insert("product".into(), value);
                            }
                            if let Some(value) = usb.serial_number {
                                properties.insert("serial".into(), value);
                            }
                        }
                        ExecutionEndpoint {
                            key: port.port_name.clone(),
                            label: port.port_name,
                            properties,
                        }
                    })
                    .collect()
            })
            .unwrap_or_default();
    }

    fn handle_request(
        &mut self,
        request: ExecutionRequest,
    ) -> RequestResult<ExecutionTransportState> {
        self.transport.latest_request_id = Some(request.request_id.clone());
        let error = match request.action {
            RequestAction::Connect => request
                .fields
                .get("port")
                .ok_or_else(|| "连接请求缺少 port 字段".to_owned())
                .and_then(|port| self.configure_endpoint(Some(port.clone())))
                .err(),
            RequestAction::Disconnect => self.configure_endpoint(None).err(),
            RequestAction::Discover => {
                self.discover();
                None
            }
            RequestAction::Refresh => {
                if let Some(bus) = self.bus.as_mut() {
                    self.transport.parameter_values = bus.read_parameters();
                    self.transport.parameter_error = self
                        .transport
                        .parameter_values
                        .iter()
                        .find_map(|value| value.original_error.clone());
                }
                None
            }
            RequestAction::Apply => {
                if request.fields.contains_key("torque_mode") {
                    self.apply_torque(&request.fields).err()
                } else if request.fields.contains_key("feedback_interval_ms")
                    || request.fields.contains_key("gripper_strength_percent")
                {
                    self.apply_execution_config(&request.fields).err()
                } else {
                    self.apply_parameters(&request.fields).err()
                }
            }
            action => Some(format!("execution 节点不处理 {action:?} 请求")),
        };
        RequestResult {
            schema_version: SCHEMA_VERSION,
            request_id: request.request_id,
            acknowledged_action: request.action,
            value: Some(self.transport.clone()),
            original_error: error,
        }
    }

    fn apply_torque(&mut self, fields: &BTreeMap<String, String>) -> Result<(), String> {
        let hold = match fields.get("torque_mode").map(String::as_str) {
            Some("hold") => true,
            Some("release") => false,
            Some(value) => return Err(format!("未知力矩模式 {value}")),
            None => return Err("力矩请求缺少 torque_mode".into()),
        };
        self.bus
            .as_mut()
            .ok_or_else(|| "真机串口未连接".to_owned())?
            .set_torque(hold)?;
        if !hold {
            self.gripper_feedback = GripperFeedbackController::default();
            self.transport.gripper_control_power_mw = None;
        }
        Ok(())
    }

    fn apply_parameters(&mut self, fields: &BTreeMap<String, String>) -> Result<(), String> {
        let actuator = fields
            .get("actuator_key")
            .ok_or_else(|| "参数写入请求缺少 actuator_key".to_owned())?;
        let id = JOINTS
            .iter()
            .position(|key| key == actuator)
            .map(|index| index as u8)
            .or_else(|| (actuator == "gripper").then_some(6))
            .ok_or_else(|| format!("未知执行器 {actuator}"))?;
        let parse = |key: &str| {
            fields
                .get(key)
                .ok_or_else(|| format!("参数写入请求缺少 {key}"))?
                .parse::<u16>()
                .map_err(|error| format!("参数 {key} 无效：{error}"))
        };
        let bus = self
            .bus
            .as_mut()
            .ok_or_else(|| "真机串口未连接".to_owned())?;
        bus.write_gains(id, parse("kp")?, parse("hold_kp")?)?;
        self.transport.parameter_values = bus.read_parameters();
        self.transport.parameter_error = self
            .transport
            .parameter_values
            .iter()
            .find_map(|value| value.original_error.clone());
        Ok(())
    }

    fn apply_execution_config(&mut self, fields: &BTreeMap<String, String>) -> Result<(), String> {
        let mut next = self.config.clone();
        if let Some(value) = fields.get("feedback_interval_ms") {
            next.feedback_interval_ms = value
                .parse::<u64>()
                .map_err(|error| format!("feedback_interval_ms 无效：{error}"))?;
        }
        if let Some(value) = fields.get("gripper_strength_percent") {
            let strength = value
                .parse::<f64>()
                .map_err(|error| format!("夹爪力度无效：{error}"))?;
            if !strength.is_finite() || !(0.0..=100.0).contains(&strength) {
                return Err("夹爪力度使用已有的 0–100 反馈刻度".into());
            }
            next.gripper_strength_percent = strength;
        }
        next.config_version += 1;
        save(&self.config_path, &next).map_err(|error| error.to_string())?;
        self.config = next;
        self.transport.feedback_interval_ms = self.config.feedback_interval_ms;
        self.transport.gripper_strength_percent = Some(self.config.gripper_strength_percent);
        self.last_feedback_poll = None;
        Ok(())
    }

    fn configure_endpoint(&mut self, selected_endpoint: Option<String>) -> Result<(), String> {
        let mut next = self.config.clone();
        next.selected_endpoint = selected_endpoint.clone();
        next.config_version += 1;
        save(&self.config_path, &next).map_err(|error| error.to_string())?;
        self.config = next;

        if let Some(path) = selected_endpoint {
            self.connect(path)
        } else {
            self.disconnect();
            self.transport.selected_endpoint = None;
            Ok(())
        }
    }

    fn connect(&mut self, path: String) -> Result<(), String> {
        self.bus = None;
        self.last_feedback_poll = None;
        self.transport.connected = false;
        self.transport.selected_endpoint = Some(path.clone());
        match StarArmBus::open(&path) {
            Ok((bus, parameters, state, telemetry)) => {
                self.bus = Some(bus);
                self.transport.connected = true;
                self.transport.last_error = None;
                self.transport.parameter_values = parameters;
                self.transport.parameter_error = self
                    .transport
                    .parameter_values
                    .iter()
                    .find_map(|value| value.original_error.clone());
                self.state = state;
                self.telemetry = telemetry;
                self.next_sequence = self.state.sequence + 1;
                Ok(())
            }
            Err(error) => {
                self.transport.last_error = Some(error.clone());
                Err(error)
            }
        }
    }

    fn disconnect(&mut self) {
        self.gripper_feedback = GripperFeedbackController::default();
        self.transport.gripper_control_power_mw = None;
        self.transport.gripper_strength_feedback_percent = None;
        self.transport.gripper_feedback_telemetry = None;
        self.transport.gripper_feedback_time_ns = None;
        self.bus = None;
        self.last_feedback_poll = None;
        self.transport.connected = false;
        self.transport.last_error = None;
    }

    fn apply_command(&mut self, command: ArmCommand) {
        if let Err(error) = validate_command(&command) {
            self.transport.last_error = Some(error);
            return;
        }
        let gripper_angle = match degrees_to_tenths(command.actuators_rad[0].to_degrees()) {
            Ok(angle) => angle,
            Err(error) => {
                self.transport.last_error = Some(error.to_string());
                return;
            }
        };
        self.gripper_feedback.request(gripper_angle);
        self.transport.gripper_control_power_mw = self.gripper_feedback.regulated_power_mw;
        if let Some(bus) = self.bus.as_mut() {
            self.transport.last_command = Some(command.clone());
            if let Err(error) = bus.write(&command, self.gripper_feedback.command_power_mw()) {
                self.reopen_after_io_error(error);
            } else {
                self.transport.last_error = None;
            }
        } else if self.transport.selected_endpoint.is_none() {
            self.transport.last_command = Some(command.clone());
            self.state = ArmState {
                schema_version: SCHEMA_VERSION,
                sequence: self.next_sequence,
                sample_time_ns: now_ns(),
                model_revision: command.model_revision,
                joints_rad: command.joints_rad,
                actuators_rad: command.actuators_rad,
                feedback_source: FeedbackSource::Software,
            };
            self.next_sequence += 1;
        }
    }

    fn poll_hardware(&mut self) -> bool {
        let now = Instant::now();
        if self.bus.is_none()
            || self.last_feedback_poll.is_some_and(|last| {
                now.duration_since(last) < Duration::from_millis(self.config.feedback_interval_ms)
            })
        {
            return false;
        }
        self.last_feedback_poll = Some(now);
        let result = self
            .bus
            .as_mut()
            .map(|bus| bus.read_state(self.next_sequence));
        match result {
            Some(Ok((state, telemetry))) => {
                self.state = state;
                self.telemetry = telemetry;
                self.transport.gripper_feedback_telemetry = self
                    .telemetry
                    .actuators
                    .iter()
                    .find(|actuator| actuator.actuator_key == "gripper")
                    .cloned();
                self.transport.gripper_feedback_time_ns = Some(self.telemetry.sample_time_ns);
                let strength = primary_tool_feedback(&self.telemetry).strength_percent;
                self.transport.gripper_strength_feedback_percent = Some(strength);
                if self
                    .gripper_feedback
                    .observe(strength, self.config.gripper_strength_percent)
                {
                    self.transport.gripper_control_power_mw =
                        self.gripper_feedback.regulated_power_mw;
                    if let (Some(bus), Some(requested)) =
                        (&mut self.bus, &self.transport.last_command)
                        && let Err(error) =
                            bus.write(requested, self.gripper_feedback.command_power_mw())
                    {
                        self.reopen_after_io_error(error);
                        return true;
                    }
                }
                self.next_sequence += 1;
                self.transport.last_error = None;
                self.transport.feedback_summary =
                    Some(format!("sequence={} source=hardware", self.state.sequence));
                true
            }
            Some(Err(error)) => {
                self.reopen_after_io_error(error);
                true
            }
            None => false,
        }
    }

    fn reopen_after_io_error(&mut self, error: String) {
        let path = self.transport.selected_endpoint.clone();
        self.bus = None;
        self.last_feedback_poll = None;
        self.transport.connected = false;
        self.transport.last_error = Some(error);
        if let Some(path) = path {
            let io_error = self.transport.last_error.clone().unwrap_or_default();
            if let Err(reopen_error) = self.connect(path) {
                self.transport.last_error =
                    Some(format!("{io_error}；重新打开同一串口失败：{reopen_error}"));
            }
        }
    }
}

fn execution_info() -> ExecutionInfo {
    let parameter = |key: &str, label: &str| NumericFieldSchema {
        key: key.into(),
        label: label.into(),
        unit: String::new(),
        minimum: None,
        maximum: None,
        required: false,
    };
    ExecutionInfo {
        schema_version: SCHEMA_VERSION,
        adapter_id: "stararm-102-fashionstar".into(),
        adapter_revision: ADAPTER_REVISION.into(),
        model_revision: MODEL_REVISION.into(),
        connection_fields: vec![ConnectionFieldSchema {
            key: "port".into(),
            label: "串口".into(),
            field_type: "endpoint".into(),
            required: true,
            options: vec![],
        }],
        actuator_labels: vec![
            "J1 / ID 0",
            "J2 / ID 1",
            "J3 / ID 2",
            "J4 / ID 3",
            "J5 / ID 4",
            "J6 / ID 5",
            "夹爪 / ID 6",
        ]
        .into_iter()
        .map(str::to_owned)
        .collect(),
        parameter_fields: vec![
            parameter("kp", "Kp"),
            parameter("kd", "Kd"),
            parameter("ki", "Ki"),
            parameter("bias", "偏置"),
            parameter("hold_kp", "保持 Kp"),
            parameter("hold_kd", "保持 Kd"),
            parameter("hold_bias", "保持偏置"),
            parameter("full_deg", "满量程"),
            parameter("reserved", "保留值"),
            parameter("pwm_limit", "PWM 上限"),
            parameter("direction", "方向"),
            parameter("pwm_frequency", "PWM 频率"),
            parameter("dead_zone", "死区"),
            parameter("motor_direction", "电机方向"),
            parameter("version_info", "内部版本"),
        ],
    }
}

fn actuator_key(id: u8) -> String {
    if id < 6 {
        JOINTS[usize::from(id)].into()
    } else {
        "gripper".into()
    }
}

fn parameter_values(value: InternalParameters, time: i64) -> Vec<ParameterValue> {
    [
        ("kp", value.kp),
        ("kd", value.kd),
        ("ki", value.ki),
        ("bias", value.bias),
        ("hold_kp", value.hold_kp),
        ("hold_kd", value.hold_kd),
        ("hold_bias", value.hold_bias),
        ("full_deg", value.full_deg),
        ("reserved", value.reserved),
        ("pwm_limit", value.pwm_limit),
        ("direction", u16::from(value.direction)),
        ("pwm_frequency", u16::from(value.pwm_frequency)),
        ("dead_zone", u16::from(value.dead_zone)),
        ("motor_direction", u16::from(value.motor_direction)),
        ("version_info", u16::from(value.version_info)),
    ]
    .into_iter()
    .map(|(field, number)| ParameterValue {
        actuator_key: actuator_key(value.id),
        field_key: field.into(),
        value: Some(f64::from(number)),
        unit: String::new(),
        read_time_ns: time,
        original_error: None,
    })
    .collect()
}

fn read_parameters(bus: &mut FashionStarBus) -> Vec<ParameterValue> {
    let read_time_ns = now_ns();
    let mut parameters = Vec::new();
    for id in SERVO_IDS {
        match bus.read_internal_parameters(id) {
            Ok(value) => parameters.extend(parameter_values(value, read_time_ns)),
            Err(error) => parameters.push(ParameterValue {
                actuator_key: actuator_key(id),
                field_key: "internal_parameters".into(),
                value: None,
                unit: String::new(),
                read_time_ns,
                original_error: Some(format!("读取舵机 ID {id} 内部参数失败：{error}")),
            }),
        }
    }
    parameters
}

fn read_sorted_monitors(bus: &mut FashionStarBus) -> Result<[Monitor; 7], String> {
    let monitors = bus
        .read_monitors(&SERVO_IDS)
        .map_err(|error| error.to_string())?;
    let mut sorted: [Option<Monitor>; 7] = [None; 7];
    for monitor in monitors {
        let index = usize::from(monitor.id);
        if index >= sorted.len() || sorted[index].is_some() {
            return Err(format!("Monitor 返回了无效或重复的 ID {}", monitor.id));
        }
        sorted[index] = Some(monitor);
    }
    if sorted.iter().any(Option::is_none) {
        return Err("Monitor 未返回 ID 0–6".into());
    }
    Ok(sorted.map(Option::unwrap))
}

fn state_from_monitors(monitors: &[Monitor; 7], sequence: u64) -> ArmState {
    ArmState {
        schema_version: SCHEMA_VERSION,
        sequence,
        sample_time_ns: now_ns(),
        model_revision: MODEL_REVISION.into(),
        joints_rad: monitors[..6]
            .iter()
            .map(|value| value.position_degrees().to_radians())
            .collect(),
        actuators_rad: vec![monitors[6].position_degrees().to_radians()],
        feedback_source: FeedbackSource::Hardware,
    }
}

fn telemetry_from_monitors(
    monitors: &[Monitor; 7],
    sequence: u64,
    sample_time_ns: i64,
    gripper_command_power_mw: Option<u16>,
) -> ArmTelemetry {
    ArmTelemetry {
        schema_version: SCHEMA_VERSION,
        sequence,
        sample_time_ns,
        model_revision: MODEL_REVISION.into(),
        actuators: monitors
            .iter()
            .map(|monitor| ActuatorTelemetry {
                actuator_key: actuator_key(monitor.id),
                voltage_mv: monitor.voltage_mv,
                current_ma: monitor.current_ma,
                power_mw: monitor.power_mw,
                command_power_limit_mw: if monitor.id == 6 {
                    gripper_command_power_mw
                } else {
                    None
                },
                temperature_raw: monitor.temperature_raw,
                status: monitor.status,
            })
            .collect(),
    }
}

fn primary_tool_feedback(telemetry: &ArmTelemetry) -> ActionFeedback {
    let strength_percent = telemetry
        .actuators
        .iter()
        .find(|actuator| actuator.actuator_key == "gripper")
        .map(|actuator| {
            let load_power_mw = actuator.power_mw.saturating_sub(GRIPPER_IDLE_POWER_MW);
            f64::from(load_power_mw) / f64::from(GRIPPER_COMMAND_POWER_MW - GRIPPER_IDLE_POWER_MW)
        })
        .unwrap_or(0.0)
        .min(1.0)
        * 100.0;
    ActionFeedback {
        schema_version: SCHEMA_VERSION,
        sequence: telemetry.sequence,
        sample_time_ns: telemetry.sample_time_ns,
        action: "primary_tool".into(),
        strength_percent,
    }
}

fn encode_command(command: &ArmCommand) -> Result<[PositionCommand; 7], String> {
    let positions = command
        .joints_rad
        .iter()
        .copied()
        .chain(command.actuators_rad.iter().copied());
    let commands = positions
        .enumerate()
        .map(|(id, radians)| {
            Ok(PositionCommand {
                id: id as u8,
                position_tenths_degree: degrees_to_tenths(radians.to_degrees())
                    .map_err(|error| error.to_string())?,
                motion_time_ms: MOTION_TIME_MS,
                acceleration_time_ms: ACCELERATION_TIME_MS,
                deceleration_time_ms: DECELERATION_TIME_MS,
                power_mw: if id == 6 { GRIPPER_COMMAND_POWER_MW } else { 0 },
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    commands
        .try_into()
        .map_err(|_| "StarArm-102 命令必须包含六个关节和一个夹爪".into())
}

fn publish_snapshot(
    node: &mut DoraNode,
    execution: &StarArmExecution,
    model: &ModelCatalog,
) -> Result<()> {
    send(node, "robot_model_info", &model.model_info())?;
    send(node, "execution_info", &execution.info)?;
    publish_transport(node, execution)?;
    publish_state(node, execution)
}
fn publish_transport(node: &mut DoraNode, execution: &StarArmExecution) -> Result<()> {
    send(node, "transport_state", &execution.transport)?;
    send(node, "service_state", &execution.service_state())
}
fn publish_state(node: &mut DoraNode, execution: &StarArmExecution) -> Result<()> {
    send(node, "arm_state", &execution.state)?;
    if execution.transport.connected && execution.state.feedback_source == FeedbackSource::Hardware
    {
        send(
            node,
            "action_feedback",
            &primary_tool_feedback(&execution.telemetry),
        )?;
    }
    send(node, "service_state", &execution.service_state())
}
fn send<T: serde::Serialize>(node: &mut DoraNode, id: &str, value: &T) -> Result<()> {
    node.send_output(
        DataId::from(id.to_owned()),
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
    fn command(joints: Vec<f64>, actuator: f64) -> ArmCommand {
        ArmCommand {
            schema_version: SCHEMA_VERSION,
            sequence: 4,
            controller_time_ns: 5,
            model_revision: MODEL_REVISION.into(),
            joints_rad: joints,
            actuators_rad: vec![actuator],
        }
    }
    fn config_path() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../../../../temp")
            .join(format!(
                "stararm-execution-config-{}-{}.json",
                std::process::id(),
                now_ns()
            ))
    }

    #[test]
    fn software_and_hardware_share_one_state_type() {
        let mut execution = StarArmExecution::new();
        let joints = vec![0.1, 0.1, -0.1, 0.1, 0.1, 0.1];
        execution.apply_command(command(joints.clone(), 0.2));
        assert_eq!(execution.state.feedback_source, FeedbackSource::Software);
        assert_eq!(execution.state.joints_rad, joints);
    }

    #[test]
    fn strength_config_persists_without_changing_feedback_period() {
        let path = config_path();
        let mut execution = StarArmExecution::with_config(path.clone(), ExecutionConfig::default());
        let result = execution.handle_request(ExecutionRequest {
            schema_version: SCHEMA_VERSION,
            request_id: "strength-config".into(),
            action: RequestAction::Apply,
            fields: BTreeMap::from([("gripper_strength_percent".into(), "50".into())]),
        });
        assert!(result.original_error.is_none());
        let saved: ExecutionConfig = load_or_default(&path).unwrap();
        assert_eq!(saved.gripper_strength_percent, 50.0);
        assert_eq!(saved.feedback_interval_ms, DEFAULT_FEEDBACK_INTERVAL_MS);
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn sub_uart_roundoff_is_not_a_release_command() {
        let mut execution = StarArmExecution::new();
        execution.apply_command(command(DEFAULT_JOINTS_RAD.to_vec(), 1.0));
        execution.apply_command(command(DEFAULT_JOINTS_RAD.to_vec(), 0.0));
        let holding = execution.gripper_feedback.regulated_power_mw;
        assert!(holding.is_some());
        // Native trajectory endpoint seen in execution 9; it encodes to 0°.
        execution.apply_command(command(
            DEFAULT_JOINTS_RAD.to_vec(),
            6.811_975_009_831_89e-17,
        ));
        assert_eq!(execution.gripper_feedback.regulated_power_mw, holding);
        execution.apply_command(command(DEFAULT_JOINTS_RAD.to_vec(), 0.1_f64.to_radians()));
        assert_eq!(execution.gripper_feedback.regulated_power_mw, None);
    }

    #[test]
    fn disconnect_removes_regulation_and_stale_strength() {
        let mut execution = StarArmExecution::new();
        execution.gripper_feedback.request(10);
        execution.gripper_feedback.request(0);
        execution.gripper_feedback.observe(50.0, 50.0);
        execution.transport.gripper_strength_feedback_percent = Some(50.0);
        execution.disconnect();
        assert!(!execution.gripper_feedback.observe(20.0, 50.0));
        assert_eq!(execution.gripper_feedback.regulated_power_mw, None);
        assert_eq!(execution.transport.gripper_strength_feedback_percent, None);
    }
    #[test]
    fn selected_but_disconnected_hardware_freezes_the_visible_state() {
        let mut execution = StarArmExecution::new();
        execution.transport.selected_endpoint = Some("/dev/disconnected".into());
        let before = execution.state.clone();
        execution.apply_command(command(vec![0.2, 0.2, -0.2, 0.2, 0.2, 0.2], 0.4));
        assert_eq!(execution.state, before);
        assert!(execution.transport.last_command.is_none());
    }

    #[test]
    fn execution_rejects_a_command_outside_the_model_motor_limits() {
        let mut execution = StarArmExecution::new();
        execution.apply_command(command(
            vec![0.0, 0.0, 0.01, 0.0, 0.0, 0.0],
            GRIPPER_DRIVE_JOINT_CLOSED_RAD,
        ));
        assert!(execution.transport.last_command.is_none());
        assert!(
            execution
                .transport
                .last_error
                .as_deref()
                .is_some_and(|error| error.contains("joint3"))
        );
    }
    #[test]
    fn all_model_angles_keep_their_sign_at_the_uart_boundary() {
        let encoded = encode_command(&command(vec![0.1, -0.2, 0.3, -0.4, 0.5, -0.6], 0.7)).unwrap();
        for (actual, expected) in encoded[..6]
            .iter()
            .zip([0.1_f64, -0.2, 0.3, -0.4, 0.5, -0.6])
        {
            assert_eq!(
                actual.position_tenths_degree,
                degrees_to_tenths(expected.to_degrees()).unwrap()
            );
        }
        assert_eq!(
            encoded[6].position_tenths_degree,
            degrees_to_tenths(0.7_f64.to_degrees()).unwrap()
        );
        assert!(encoded[..6].iter().all(|command| command.power_mw == 0));
        assert_eq!(encoded[6].power_mw, GRIPPER_COMMAND_POWER_MW);

        let monitors = std::array::from_fn(|index| Monitor {
            id: index as u8,
            voltage_mv: 0,
            current_ma: 0,
            power_mw: 0,
            temperature_raw: 0,
            status: 0,
            position_tenths_degree: (index as i32 + 1) * 10,
            turns: 0,
        });
        let state = state_from_monitors(&monitors, 1);
        assert_eq!(state.actuators_rad, [7.0_f64.to_radians()]);
    }
    #[test]
    fn primary_tool_feedback_keeps_zero_as_a_valid_sample() {
        let telemetry = |power_mw| ArmTelemetry {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            sample_time_ns: 2,
            model_revision: MODEL_REVISION.into(),
            actuators: vec![ActuatorTelemetry {
                actuator_key: "gripper".into(),
                voltage_mv: 12_000,
                current_ma: 30,
                power_mw,
                command_power_limit_mw: Some(GRIPPER_COMMAND_POWER_MW),
                temperature_raw: 0,
                status: 0,
            }],
        };
        let stable = primary_tool_feedback(&telemetry(364));
        assert_eq!(stable.sequence, 1);
        assert_eq!(stable.sample_time_ns, 2);
        assert_eq!(stable.strength_percent, 0.0);
        assert_eq!(primary_tool_feedback(&telemetry(400)).strength_percent, 0.0);
        assert_eq!(
            primary_tool_feedback(&telemetry(1_200)).strength_percent,
            50.0
        );
        assert_eq!(
            primary_tool_feedback(&telemetry(2_000)).strength_percent,
            100.0
        );
    }
    #[test]
    fn identical_encoded_command_is_the_deduplication_identity() {
        let first = encode_command(&command(vec![0.1; 6], 0.2)).unwrap();
        let same_tenths = encode_command(&command(vec![0.1001; 6], 0.2001)).unwrap();
        assert_eq!(first, same_tenths);
    }
    #[test]
    fn arm_transport_does_not_reissue_an_unchanged_gripper_command() {
        let first = encode_command(&command(vec![0.1; 6], 0.0)).unwrap();
        let mut next = first;
        next[0].position_tenths_degree += 1;
        assert_eq!(changed_servo_commands(Some(&first), &next), vec![next[0]]);
        assert_eq!(changed_servo_commands(None, &next), next.to_vec());
        assert!(changed_servo_commands(Some(&next), &next).is_empty());
        let previous = next;
        next[6].power_mw -= 1;
        assert_eq!(
            changed_servo_commands(Some(&previous), &next),
            vec![next[6]]
        );
        assert_eq!(
            next[6].position_tenths_degree,
            previous[6].position_tenths_degree
        );
    }
    #[test]
    fn parameter_refresh_does_not_enumerate_serial_endpoints() {
        let mut execution = StarArmExecution::new();
        execution.transport.discovered_endpoints = vec![ExecutionEndpoint {
            key: "fixture".into(),
            label: "fixture".into(),
            properties: BTreeMap::new(),
        }];
        let result = execution.handle_request(ExecutionRequest {
            schema_version: SCHEMA_VERSION,
            request_id: "refresh-parameters".into(),
            action: RequestAction::Refresh,
            fields: BTreeMap::new(),
        });
        assert_eq!(result.acknowledged_action, RequestAction::Refresh);
        assert_eq!(execution.transport.discovered_endpoints[0].key, "fixture");
    }

    #[test]
    fn initial_state_matches_the_model_start_only_before_any_input() {
        let execution = StarArmExecution::new();
        assert_eq!(execution.state.joints_rad, DEFAULT_JOINTS_RAD);
        assert_eq!(
            execution.state.actuators_rad,
            [GRIPPER_DRIVE_JOINT_CLOSED_RAD]
        );
    }

    #[test]
    fn io_error_releases_the_connection_and_reopens_only_the_selected_endpoint() {
        let mut execution = StarArmExecution::new();
        let before = execution.state.clone();
        execution.transport.connected = true;
        execution.transport.selected_endpoint = Some("/path/that/does/not/exist".into());
        execution.reopen_after_io_error("operation timed out".into());

        assert!(!execution.transport.connected);
        assert!(execution.bus.is_none());
        assert_eq!(
            execution.transport.selected_endpoint.as_deref(),
            Some("/path/that/does/not/exist")
        );
        let error = execution.transport.last_error.as_deref().unwrap();
        assert!(error.contains("operation timed out"));
        assert!(error.contains("重新打开同一串口失败"));
        assert_eq!(execution.state, before);
        assert!(execution.transport.last_command.is_none());
    }

    #[test]
    fn endpoint_selection_is_persisted_even_when_opening_it_fails() {
        let path = config_path();
        let mut execution = StarArmExecution::with_config(path.clone(), ExecutionConfig::default());

        assert!(
            execution
                .configure_endpoint(Some("/path/that/does/not/exist".into()))
                .is_err()
        );
        let selected: ExecutionConfig = load_or_default(&path).unwrap();
        assert_eq!(
            selected.selected_endpoint.as_deref(),
            Some("/path/that/does/not/exist")
        );
        assert_eq!(selected.config_version, 2);

        execution.configure_endpoint(None).unwrap();
        let disconnected: ExecutionConfig = load_or_default(&path).unwrap();
        assert_eq!(disconnected.selected_endpoint, None);
        assert_eq!(disconnected.config_version, 3);
        assert_eq!(execution.transport.selected_endpoint, None);
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn feedback_interval_is_persisted_without_reconnecting() {
        let path = config_path();
        let mut execution = StarArmExecution::with_config(path.clone(), ExecutionConfig::default());
        let fields = BTreeMap::from([("feedback_interval_ms".into(), "250".into())]);
        execution.apply_execution_config(&fields).unwrap();

        let persisted: ExecutionConfig = load_or_default(&path).unwrap();
        assert_eq!(persisted.feedback_interval_ms, 250);
        assert_eq!(execution.transport.feedback_interval_ms, 250);
        assert!(execution.last_feedback_poll.is_none());
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn missing_feedback_interval_uses_the_default_period() {
        let config: ExecutionConfig = serde_json::from_str(
            r#"{"schema_version":1,"config_version":4,"selected_endpoint":null}"#,
        )
        .unwrap();
        assert_eq!(config.feedback_interval_ms, DEFAULT_FEEDBACK_INTERVAL_MS);
    }
}
