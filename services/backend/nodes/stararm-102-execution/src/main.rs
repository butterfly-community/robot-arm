use std::{
    collections::BTreeMap,
    time::{SystemTime, UNIX_EPOCH},
};

use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result};
use fashionstar_uart::{
    FashionStarBus, InternalParameters, Monitor, PositionCommand, degrees_to_tenths,
};
use robot_arm_messages::{
    ArmCommand, ArmState, ConnectionFieldSchema, ExecutionEndpoint, ExecutionInfo,
    ExecutionRequest, ExecutionTransportState, FeedbackSource, NumericFieldSchema, ParameterValue,
    RequestAction, RequestResult, SCHEMA_VERSION, ServiceState, from_arrow, to_arrow,
};

const MODEL_REVISION: &str = "stararm-102-fl-v1";
const ADAPTER_REVISION: &str = "stararm-102-fashionstar-v1";
const SERVO_IDS: [u8; 7] = [0, 1, 2, 3, 4, 5, 6];
const JOINT_KEYS: [&str; 6] = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"];
const INITIAL_JOINTS_RAD: [f64; 6] = [0.0, 0.0, -3.0_f64.to_radians(), 0.0, 0.0, 0.0];
const INITIAL_GRIPPER_RAD: f64 = 1.0_f64.to_radians();
const MOTION_TIME_MS: u32 = 100;
const ACCELERATION_TIME_MS: u16 = 50;
const DECELERATION_TIME_MS: u16 = 50;

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let mut execution = StarArmExecution::new();
    publish_snapshot(&mut node, &mut execution)?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => match id.as_str() {
                "arm_command" => {
                    let command: ArmCommand =
                        from_arrow(data.as_array()).context("decode arm_command")?;
                    execution.apply_command(command);
                    publish_snapshot(&mut node, &mut execution)?;
                }
                "execution_request" => {
                    let request: ExecutionRequest =
                        from_arrow(data.as_array()).context("decode execution_request")?;
                    let result = execution.handle_request(request);
                    send(&mut node, "request_result", &result)?;
                    publish_snapshot(&mut node, &mut execution)?;
                }
                "tick" => {
                    if execution.poll_hardware() {
                        publish_state(&mut node, &execution)?;
                    }
                }
                "scan" => {
                    execution.discover();
                    publish_transport(&mut node, &execution)?;
                }
                "snapshot" => publish_snapshot(&mut node, &mut execution)?,
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
    fn open(path: &str) -> Result<(Self, Vec<ParameterValue>, ArmState), String> {
        let mut bus =
            FashionStarBus::open(path).map_err(|error| format!("无法打开串口 {path}：{error}"))?;
        for id in SERVO_IDS {
            bus.ping(id)
                .map_err(|error| format!("Ping 舵机 ID {id} 失败：{error}"))?;
        }
        let parameters = read_parameters(&mut bus);
        let monitors = read_sorted_monitors(&mut bus)?;
        let state = state_from_monitors(monitors, 1);
        Ok((
            Self {
                bus,
                last_commands: None,
            },
            parameters,
            state,
        ))
    }

    fn read_state(&mut self, sequence: u64) -> Result<ArmState, String> {
        Ok(state_from_monitors(
            read_sorted_monitors(&mut self.bus)?,
            sequence,
        ))
    }

    fn read_parameters(&mut self) -> Vec<ParameterValue> {
        read_parameters(&mut self.bus)
    }

    fn write(&mut self, command: &ArmCommand) -> Result<bool, String> {
        let commands = encode_command(command)?;
        if self.last_commands.as_ref() == Some(&commands) {
            return Ok(false);
        }
        self.bus
            .write_positions(&commands)
            .map_err(|error| format!("串口写入失败：{error}"))?;
        self.last_commands = Some(commands);
        Ok(true)
    }
}

struct StarArmExecution {
    info: ExecutionInfo,
    transport: ExecutionTransportState,
    state: ArmState,
    bus: Option<StarArmBus>,
    next_sequence: u64,
}

impl StarArmExecution {
    fn new() -> Self {
        Self {
            info: execution_info(),
            transport: ExecutionTransportState {
                schema_version: SCHEMA_VERSION,
                ..Default::default()
            },
            state: ArmState {
                schema_version: SCHEMA_VERSION,
                sequence: 0,
                sample_time_ns: now_ns(),
                model_revision: MODEL_REVISION.into(),
                joints_rad: INITIAL_JOINTS_RAD.to_vec(),
                actuators_rad: vec![INITIAL_GRIPPER_RAD],
                feedback_source: FeedbackSource::Software,
            },
            bus: None,
            next_sequence: 1,
        }
    }

    fn service_state(&self) -> ServiceState {
        ServiceState {
            schema_version: SCHEMA_VERSION,
            build_version: env!("CARGO_PKG_VERSION").into(),
            config_version: 0,
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
                .and_then(|port| self.connect(port.clone()))
                .err(),
            RequestAction::Disconnect => {
                self.disconnect();
                None
            }
            RequestAction::Refresh => {
                self.discover();
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

    fn connect(&mut self, path: String) -> Result<(), String> {
        self.bus = None;
        self.transport.connected = false;
        self.transport.selected_endpoint = Some(path.clone());
        match StarArmBus::open(&path) {
            Ok((bus, parameters, state)) => {
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
        self.bus = None;
        self.transport.connected = false;
        self.transport.last_error = None;
    }

    fn apply_command(&mut self, command: ArmCommand) {
        self.transport.last_command = Some(command.clone());
        if command.model_revision != MODEL_REVISION
            || command.joints_rad.len() != 6
            || command.actuators_rad.len() != 1
        {
            self.transport.last_error = Some(format!(
                "ArmCommand 与执行模型不一致：revision={} joints={} actuators={}",
                command.model_revision,
                command.joints_rad.len(),
                command.actuators_rad.len()
            ));
            return;
        }
        if let Some(bus) = self.bus.as_mut() {
            if let Err(error) = bus.write(&command) {
                self.reopen_after_io_error(error);
            } else {
                self.transport.last_error = None;
            }
        } else {
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
        let result = self
            .bus
            .as_mut()
            .map(|bus| bus.read_state(self.next_sequence));
        match result {
            Some(Ok(state)) => {
                let changed = feedback_payload_changed(&self.state, &state);
                self.state = state;
                self.next_sequence += 1;
                self.transport.last_error = None;
                self.transport.feedback_summary =
                    Some(format!("sequence={} source=hardware", self.state.sequence));
                changed
            }
            Some(Err(error)) => {
                self.reopen_after_io_error(error);
                false
            }
            None => false,
        }
    }

    fn reopen_after_io_error(&mut self, error: String) {
        let path = self.transport.selected_endpoint.clone();
        self.bus = None;
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

fn feedback_payload_changed(previous: &ArmState, current: &ArmState) -> bool {
    previous.model_revision != current.model_revision
        || previous.joints_rad != current.joints_rad
        || previous.actuators_rad != current.actuators_rad
        || previous.feedback_source != current.feedback_source
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
            parameter("direction", "方向"),
            parameter("dead_zone", "死区"),
        ],
    }
}

fn actuator_key(id: u8) -> String {
    if id < 6 {
        JOINT_KEYS[usize::from(id)].into()
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
        ("direction", u16::from(value.direction)),
        ("dead_zone", u16::from(value.dead_zone)),
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

fn state_from_monitors(monitors: [Monitor; 7], sequence: u64) -> ArmState {
    ArmState {
        schema_version: SCHEMA_VERSION,
        sequence,
        sample_time_ns: now_ns(),
        model_revision: MODEL_REVISION.into(),
        joints_rad: monitors[..6]
            .iter()
            .map(|value| value.position_degrees().to_radians())
            .collect(),
        actuators_rad: vec![-monitors[6].position_degrees().to_radians()],
        feedback_source: FeedbackSource::Hardware,
    }
}

fn encode_command(command: &ArmCommand) -> Result<[PositionCommand; 7], String> {
    let positions = command
        .joints_rad
        .iter()
        .copied()
        .chain(command.actuators_rad.iter().map(|value| -*value));
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
                power_mw: 0,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    commands
        .try_into()
        .map_err(|_| "StarArm-102 命令必须包含六个关节和一个夹爪".into())
}

fn publish_snapshot(node: &mut DoraNode, execution: &mut StarArmExecution) -> Result<()> {
    execution.discover();
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
    #[test]
    fn software_and_hardware_share_one_state_type() {
        let mut execution = StarArmExecution::new();
        execution.apply_command(command(vec![0.1; 6], 0.2));
        assert_eq!(execution.state.feedback_source, FeedbackSource::Software);
        assert_eq!(execution.state.joints_rad, vec![0.1; 6]);
    }
    #[test]
    fn j1_through_j6_keep_their_sign_and_only_gripper_uses_hardware_transmission_sign() {
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
            degrees_to_tenths((-0.7_f64).to_degrees()).unwrap()
        );
    }
    #[test]
    fn identical_encoded_command_is_the_deduplication_identity() {
        let first = encode_command(&command(vec![0.1; 6], 0.2)).unwrap();
        let same_tenths = encode_command(&command(vec![0.1001; 6], 0.2001)).unwrap();
        assert_eq!(first, same_tenths);
    }
    #[test]
    fn initial_state_matches_the_model_start_only_before_any_input() {
        let execution = StarArmExecution::new();
        assert_eq!(execution.state.joints_rad, INITIAL_JOINTS_RAD);
        assert_eq!(execution.state.actuators_rad, [INITIAL_GRIPPER_RAD]);
    }

    #[test]
    fn feedback_is_published_only_when_its_payload_changes() {
        let previous = StarArmExecution::new().state;
        let mut sampled_again = previous.clone();
        sampled_again.sequence += 1;
        sampled_again.sample_time_ns += 1;
        assert!(!feedback_payload_changed(&previous, &sampled_again));

        sampled_again.joints_rad[0] = 0.1;
        assert!(feedback_payload_changed(&previous, &sampled_again));
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
}
