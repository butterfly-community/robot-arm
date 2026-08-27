use fashionstar_uart::{
    FashionStarBus, InternalParameters, Monitor, PositionCommand, degrees_to_tenths,
};
use serde::Serialize;
use stararm102_control::{ArmFeedbackSource, ArmStateFrame};
#[cfg(test)]
use stararm102_control::{GRIPPER_START_POSITION_RAD, START_POSITION_JOINTS_RAD};

const ARM_DOF: usize = 6;
const SERVO_COUNT: usize = 7;
const SERVO_IDS: [u8; SERVO_COUNT] = [0, 1, 2, 3, 4, 5, 6];
const MOTION_TIME_MS: u32 = 100;
const ACCELERATION_TIME_MS: u16 = 50;
const DECELERATION_TIME_MS: u16 = 50;

pub struct FashionStarArmBus {
    bus: FashionStarBus,
    last_commands: Option<[PositionCommand; SERVO_COUNT]>,
    last_monitors: Option<[ServoMonitorState; SERVO_COUNT]>,
    internal_parameters: Option<[ServoInternalParametersState; SERVO_COUNT]>,
    parameter_error: Option<String>,
}

impl FashionStarArmBus {
    pub fn open(path: &str) -> Result<Self, String> {
        let mut bus =
            FashionStarBus::open(path).map_err(|error| format!("无法打开串口 {path}：{error}"))?;
        for id in SERVO_IDS {
            bus.ping(id)
                .map_err(|error| format!("Ping 舵机 ID {id} 失败：{error}"))?;
        }
        let (internal_parameters, parameter_error) = read_internal_parameters(&mut bus);
        Ok(Self {
            bus,
            last_commands: None,
            last_monitors: None,
            internal_parameters,
            parameter_error,
        })
    }

    fn monitors(&mut self) -> Result<[Monitor; SERVO_COUNT], String> {
        let monitors = self
            .bus
            .read_monitors(&SERVO_IDS)
            .map_err(|error| error.to_string())?;
        sort_monitors(monitors)
    }
}

fn read_internal_parameters(
    bus: &mut FashionStarBus,
) -> (
    Option<[ServoInternalParametersState; SERVO_COUNT]>,
    Option<String>,
) {
    let mut values: [Option<ServoInternalParametersState>; SERVO_COUNT] = [None; SERVO_COUNT];
    for id in SERVO_IDS {
        match bus.read_internal_parameters(id) {
            Ok(parameters) => values[usize::from(id)] = Some(parameters.into()),
            Err(error) => {
                return (
                    None,
                    Some(format!("读取舵机 ID {id} 内部参数失败：{error}")),
                );
            }
        }
    }
    (Some(values.map(Option::unwrap)), None)
}

fn sort_monitors(monitors: Vec<Monitor>) -> Result<[Monitor; SERVO_COUNT], String> {
    let mut values: [Option<Monitor>; SERVO_COUNT] = [None; SERVO_COUNT];
    for monitor in monitors {
        let index = usize::from(monitor.id);
        if index >= SERVO_COUNT || values[index].is_some() {
            return Err(format!("Monitor 返回了无效或重复的 ID {}", monitor.id));
        }
        values[index] = Some(monitor);
    }
    if values.iter().any(Option::is_none) {
        return Err("Monitor 未返回 ID 0–6".to_owned());
    }
    Ok(values.map(Option::unwrap))
}

impl ArmBus for FashionStarArmBus {
    fn read_state(&mut self) -> Result<ArmStateFrame, String> {
        let monitors = self.monitors()?;
        self.last_monitors = Some(monitors.map(ServoMonitorState::from));
        Ok(arm_state_from_monitors(monitors))
    }

    fn write_positions(
        &mut self,
        joints_rad: [f64; ARM_DOF],
        gripper_position_rad: f64,
    ) -> Result<(), String> {
        let commands = arm_position_commands(joints_rad, gripper_position_rad)?;
        if !commands_changed(self.last_commands.as_ref(), &commands) {
            return Ok(());
        }
        self.bus
            .write_positions(&commands)
            .map_err(|error| format!("串口写入失败：{error}"))?;
        self.last_commands = Some(commands);
        Ok(())
    }

    fn monitor_states(&self) -> Option<[ServoMonitorState; SERVO_COUNT]> {
        self.last_monitors
    }

    fn internal_parameter_states(&self) -> Option<[ServoInternalParametersState; SERVO_COUNT]> {
        self.internal_parameters
    }

    fn parameter_error(&self) -> Option<String> {
        self.parameter_error.clone()
    }
}

fn commands_changed(
    previous: Option<&[PositionCommand; SERVO_COUNT]>,
    next: &[PositionCommand; SERVO_COUNT],
) -> bool {
    previous != Some(next)
}

fn arm_state_from_monitors(monitors: [Monitor; SERVO_COUNT]) -> ArmStateFrame {
    ArmStateFrame {
        joints_rad: [
            monitors[0].position_degrees().to_radians(),
            monitors[1].position_degrees().to_radians(),
            monitors[2].position_degrees().to_radians(),
            monitors[3].position_degrees().to_radians(),
            monitors[4].position_degrees().to_radians(),
            monitors[5].position_degrees().to_radians(),
        ],
        gripper_position_rad: -monitors[6].position_degrees().to_radians(),
        feedback_source: ArmFeedbackSource::Serial,
    }
}

fn arm_position_commands(
    joints_rad: [f64; ARM_DOF],
    gripper_position_rad: f64,
) -> Result<[PositionCommand; SERVO_COUNT], String> {
    let positions = [
        joints_rad[0],
        joints_rad[1],
        joints_rad[2],
        joints_rad[3],
        joints_rad[4],
        joints_rad[5],
        -gripper_position_rad,
    ];
    let commands = positions
        .into_iter()
        .enumerate()
        .map(|(id, position)| {
            Ok(PositionCommand {
                id: id as u8,
                position_tenths_degree: degrees_to_tenths(position.to_degrees())
                    .map_err(|error| error.to_string())?,
                motion_time_ms: MOTION_TIME_MS,
                acceleration_time_ms: ACCELERATION_TIME_MS,
                deceleration_time_ms: DECELERATION_TIME_MS,
                power_mw: 0,
            })
        })
        .collect::<Result<Vec<_>, String>>()?;
    Ok(commands.try_into().expect("fixed seven positions"))
}

pub trait ArmBus: Send {
    fn read_state(&mut self) -> Result<ArmStateFrame, String>;
    fn write_positions(
        &mut self,
        joints_rad: [f64; ARM_DOF],
        gripper_position_rad: f64,
    ) -> Result<(), String>;

    fn monitor_states(&self) -> Option<[ServoMonitorState; SERVO_COUNT]> {
        None
    }

    fn internal_parameter_states(&self) -> Option<[ServoInternalParametersState; SERVO_COUNT]> {
        None
    }

    fn parameter_error(&self) -> Option<String> {
        None
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct ServoMonitorState {
    pub id: u8,
    pub voltage_mv: u16,
    pub current_ma: u16,
    pub power_mw: u16,
    pub temperature_raw: u16,
    pub status: u8,
    pub position_tenths_degree: i32,
    pub turns: i16,
}

impl From<Monitor> for ServoMonitorState {
    fn from(monitor: Monitor) -> Self {
        Self {
            id: monitor.id,
            voltage_mv: monitor.voltage_mv,
            current_ma: monitor.current_ma,
            power_mw: monitor.power_mw,
            temperature_raw: monitor.temperature_raw,
            status: monitor.status,
            position_tenths_degree: monitor.position_tenths_degree,
            turns: monitor.turns,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct ServoInternalParametersState {
    pub id: u8,
    pub kp: u16,
    pub kd: u16,
    pub ki: u16,
    pub bias: u16,
    pub hold_kp: u16,
    pub hold_kd: u16,
    pub hold_bias: u16,
    pub direction: u8,
    pub dead_zone: u8,
}

impl From<InternalParameters> for ServoInternalParametersState {
    fn from(parameters: InternalParameters) -> Self {
        Self {
            id: parameters.id,
            kp: parameters.kp,
            kd: parameters.kd,
            ki: parameters.ki,
            bias: parameters.bias,
            hold_kp: parameters.hold_kp,
            hold_kd: parameters.hold_kd,
            hold_bias: parameters.hold_bias,
            direction: parameters.direction,
            dead_zone: parameters.dead_zone,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct SerialState {
    pub selected_port: Option<String>,
    pub connected: bool,
    pub error: Option<String>,
    pub last_command_joints_rad: Option<[f64; ARM_DOF]>,
    pub last_command_gripper_rad: Option<f64>,
    pub monitors: Option<[ServoMonitorState; SERVO_COUNT]>,
    pub internal_parameters: Option<[ServoInternalParametersState; SERVO_COUNT]>,
    pub parameter_error: Option<String>,
}

#[derive(Default)]
pub struct ArmJointIo {
    state: ArmStateFrame,
    serial: SerialState,
    bus: Option<Box<dyn ArmBus>>,
}

impl ArmJointIo {
    pub fn state(&self) -> &ArmStateFrame {
        &self.state
    }

    pub fn serial_state(&self) -> &SerialState {
        &self.serial
    }

    pub fn apply_controller_output(
        &mut self,
        joints_rad: [f64; ARM_DOF],
        gripper_position_rad: f64,
    ) {
        if let Some(bus) = self.bus.as_mut() {
            self.serial.last_command_joints_rad = Some(joints_rad);
            self.serial.last_command_gripper_rad = Some(gripper_position_rad);
            match bus.write_positions(joints_rad, gripper_position_rad) {
                Ok(()) => self.serial.error = None,
                Err(error) => self.serial.error = Some(error),
            }
            return;
        }
        self.state.joints_rad = joints_rad;
        self.state.gripper_position_rad = gripper_position_rad;
        self.state.feedback_source = ArmFeedbackSource::Software;
    }

    pub fn hold_current_position(&mut self) {
        self.poll();
        if self.serial.error.is_some() {
            return;
        }
        self.apply_controller_output(self.state.joints_rad, self.state.gripper_position_rad);
    }

    pub fn reconnect_after_runtime_error(&mut self) {
        if !self.serial.connected || self.serial.error.is_none() {
            return;
        }
        if let Some(port) = self.serial.selected_port.clone() {
            let _ = self.connect_port(port);
        }
    }

    pub fn poll(&mut self) {
        if let Some(bus) = self.bus.as_mut() {
            match bus.read_state() {
                Ok(mut state) => {
                    self.serial.monitors = bus.monitor_states();
                    state.feedback_source = ArmFeedbackSource::Serial;
                    self.state = state;
                    self.serial.error = None;
                }
                Err(error) => self.serial.error = Some(error),
            }
        }
    }

    pub fn connect_port(&mut self, port: String) -> Result<(), String> {
        self.release_bus();
        let bus = match FashionStarArmBus::open(&port) {
            Ok(bus) => bus,
            Err(error) => {
                self.serial = SerialState {
                    selected_port: Some(port),
                    connected: false,
                    error: Some(error.clone()),
                    ..SerialState::default()
                };
                return Err(error);
            }
        };
        self.attach(port, Box::new(bus))
    }

    #[cfg(test)]
    fn connect(&mut self, port: String, bus: Box<dyn ArmBus>) -> Result<(), String> {
        self.release_bus();
        self.attach(port, bus)
    }

    fn attach(&mut self, port: String, mut bus: Box<dyn ArmBus>) -> Result<(), String> {
        let mut hardware = match bus.read_state() {
            Ok(state) => state,
            Err(error) => {
                self.serial = SerialState {
                    selected_port: Some(port),
                    connected: false,
                    error: Some(error.clone()),
                    ..SerialState::default()
                };
                return Err(error);
            }
        };
        let monitors = bus.monitor_states();
        let internal_parameters = bus.internal_parameter_states();
        let parameter_error = bus.parameter_error();
        hardware.feedback_source = ArmFeedbackSource::Serial;
        self.state = hardware;
        self.serial = SerialState {
            selected_port: Some(port),
            connected: true,
            error: None,
            monitors,
            internal_parameters,
            parameter_error,
            ..SerialState::default()
        };
        self.bus = Some(bus);
        Ok(())
    }

    pub fn disconnect(&mut self) {
        self.release_bus();
    }

    fn release_bus(&mut self) {
        self.bus = None;
        self.state.feedback_source = ArmFeedbackSource::Software;
        self.serial.connected = false;
        self.serial.error = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    #[derive(Default)]
    struct FakeData {
        state: ArmStateFrame,
        writes: Vec<([f64; ARM_DOF], f64)>,
        read_error: Option<String>,
        write_error: Option<String>,
        internal_parameters: Option<[ServoInternalParametersState; SERVO_COUNT]>,
        parameter_error: Option<String>,
    }

    struct FakeBus(Arc<Mutex<FakeData>>);

    impl ArmBus for FakeBus {
        fn read_state(&mut self) -> Result<ArmStateFrame, String> {
            let data = self.0.lock().unwrap();
            match &data.read_error {
                Some(error) => Err(error.clone()),
                None => Ok(data.state.clone()),
            }
        }

        fn write_positions(
            &mut self,
            joints_rad: [f64; ARM_DOF],
            gripper_position_rad: f64,
        ) -> Result<(), String> {
            let mut data = self.0.lock().unwrap();
            if let Some(error) = &data.write_error {
                return Err(error.clone());
            }
            data.writes.push((joints_rad, gripper_position_rad));
            Ok(())
        }

        fn internal_parameter_states(&self) -> Option<[ServoInternalParametersState; SERVO_COUNT]> {
            self.0.lock().unwrap().internal_parameters
        }

        fn parameter_error(&self) -> Option<String> {
            self.0.lock().unwrap().parameter_error.clone()
        }
    }

    #[test]
    fn software_state_starts_at_the_signed_pose_and_gripper_start() {
        let io = ArmJointIo::default();
        assert_eq!(io.state().joints_rad, START_POSITION_JOINTS_RAD);
        assert_eq!(io.state().gripper_position_rad, GRIPPER_START_POSITION_RAD);
    }

    #[test]
    fn software_output_is_the_feedback_for_all_seven_controls() {
        let mut io = ArmJointIo::default();
        io.apply_controller_output([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], 0.7);
        assert_eq!(io.state().joints_rad, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]);
        assert_eq!(io.state().gripper_position_rad, 0.7);
        assert_eq!(io.state().feedback_source, ArmFeedbackSource::Software);
    }

    #[test]
    fn hardware_hold_reads_then_commands_the_measured_position() {
        let data = Arc::new(Mutex::new(FakeData {
            state: ArmStateFrame {
                joints_rad: [0.1, 0.2, -0.3, 0.4, 0.5, 0.6],
                gripper_position_rad: 0.7,
                feedback_source: ArmFeedbackSource::Serial,
            },
            ..FakeData::default()
        }));
        let mut io = ArmJointIo::default();
        io.connect("/dev/fake".to_owned(), Box::new(FakeBus(data.clone())))
            .unwrap();
        data.lock().unwrap().state = ArmStateFrame {
            joints_rad: [-0.6, -0.5, -0.4, 0.3, 0.2, 0.1],
            gripper_position_rad: 0.8,
            feedback_source: ArmFeedbackSource::Serial,
        };

        io.hold_current_position();

        assert_eq!(
            data.lock().unwrap().writes,
            vec![([-0.6, -0.5, -0.4, 0.3, 0.2, 0.1], 0.8)]
        );
    }

    #[test]
    fn connection_monitor_replaces_state_and_disconnect_preserves_it() {
        let data = Arc::new(Mutex::new(FakeData {
            state: ArmStateFrame {
                joints_rad: START_POSITION_JOINTS_RAD,
                gripper_position_rad: 1.2,
                feedback_source: ArmFeedbackSource::Serial,
            },
            writes: Vec::new(),
            read_error: None,
            write_error: None,
            internal_parameters: None,
            parameter_error: None,
        }));
        let mut io = ArmJointIo::default();
        io.connect("/dev/fake".to_owned(), Box::new(FakeBus(data.clone())))
            .unwrap();
        assert!(io.serial_state().connected);
        assert_eq!(io.state().gripper_position_rad, 1.2);

        io.apply_controller_output([0.1; ARM_DOF], 0.8);
        assert_eq!(data.lock().unwrap().writes, vec![([0.1; ARM_DOF], 0.8)]);
        data.lock().unwrap().state.joints_rad = [0.2; ARM_DOF];
        io.poll();
        assert_eq!(io.state().joints_rad, [0.2; ARM_DOF]);

        io.disconnect();
        assert_eq!(io.state().joints_rad, [0.2; ARM_DOF]);
        assert_eq!(io.state().gripper_position_rad, 1.2);
        assert_eq!(io.state().feedback_source, ArmFeedbackSource::Software);
    }

    #[test]
    fn runtime_serial_errors_release_the_bus_and_attempt_reconnect() {
        for fail_write in [false, true] {
            let data = Arc::new(Mutex::new(FakeData::default()));
            let mut io = ArmJointIo::default();
            io.connect("/dev/fake".to_owned(), Box::new(FakeBus(data.clone())))
                .unwrap();
            if fail_write {
                data.lock().unwrap().write_error = Some("write failed".to_owned());
                io.apply_controller_output([0.2; ARM_DOF], 0.3);
                assert_eq!(io.serial_state().error.as_deref(), Some("write failed"));
            } else {
                data.lock().unwrap().read_error = Some("read failed".to_owned());
                io.poll();
                assert_eq!(io.serial_state().error.as_deref(), Some("read failed"));
            }
            io.reconnect_after_runtime_error();
            assert!(!io.serial_state().connected);
            assert_eq!(
                io.serial_state().selected_port.as_deref(),
                Some("/dev/fake")
            );
            assert!(io.serial_state().error.is_some());
            assert_eq!(io.state().feedback_source, ArmFeedbackSource::Software);
        }
    }

    #[test]
    fn arbitrary_software_and_hardware_poses_can_connect() {
        let data = Arc::new(Mutex::new(FakeData {
            state: ArmStateFrame {
                joints_rad: [0.1, 0.2, -0.3, 0.4, 0.5, 0.6],
                gripper_position_rad: 0.7,
                feedback_source: ArmFeedbackSource::Serial,
            },
            ..FakeData::default()
        }));
        let mut io = ArmJointIo::default();
        io.apply_controller_output([0.8, 0.9, 1.0, 1.1, 1.2, 1.3], 1.4);
        io.connect("/dev/fake".to_owned(), Box::new(FakeBus(data)))
            .unwrap();
        assert!(io.serial_state().connected);
        assert_eq!(io.state().joints_rad, [0.1, 0.2, -0.3, 0.4, 0.5, 0.6]);
        assert_eq!(io.state().gripper_position_rad, 0.7);
        assert_eq!(io.state().feedback_source, ArmFeedbackSource::Serial);
    }

    #[test]
    fn connection_exposes_read_only_internal_parameters_without_gating_connection() {
        let parameters = std::array::from_fn(|index| ServoInternalParametersState {
            id: index as u8,
            kp: 100 + index as u16,
            kd: 200 + index as u16,
            ki: 300 + index as u16,
            bias: 0,
            hold_kp: 400 + index as u16,
            hold_kd: 500 + index as u16,
            hold_bias: 0,
            direction: 0,
            dead_zone: 3,
        });
        let data = Arc::new(Mutex::new(FakeData {
            internal_parameters: Some(parameters),
            ..FakeData::default()
        }));
        let mut io = ArmJointIo::default();
        io.connect("/dev/fake".to_owned(), Box::new(FakeBus(data)))
            .unwrap();
        assert_eq!(io.serial_state().internal_parameters, Some(parameters));
        assert_eq!(io.serial_state().parameter_error, None);

        let data = Arc::new(Mutex::new(FakeData {
            parameter_error: Some("query unavailable".to_owned()),
            ..FakeData::default()
        }));
        io.connect("/dev/fake".to_owned(), Box::new(FakeBus(data)))
            .unwrap();
        assert!(io.serial_state().connected);
        assert_eq!(io.serial_state().internal_parameters, None);
        assert_eq!(
            io.serial_state().parameter_error.as_deref(),
            Some("query unavailable")
        );
    }

    #[test]
    fn reconnect_replaces_the_existing_bus_from_the_current_hardware_state() {
        let first = Arc::new(Mutex::new(FakeData {
            state: ArmStateFrame {
                joints_rad: START_POSITION_JOINTS_RAD,
                ..ArmStateFrame::default()
            },
            ..FakeData::default()
        }));
        let mut io = ArmJointIo::default();
        io.connect("/dev/fake".to_owned(), Box::new(FakeBus(first.clone())))
            .unwrap();
        first.lock().unwrap().state.joints_rad = [0.1, 0.2, -0.3, 0.4, 0.5, 0.6];
        io.poll();

        let second = Arc::new(Mutex::new(FakeData {
            state: ArmStateFrame {
                joints_rad: [0.11, 0.21, -0.31, 0.41, 0.51, 0.61],
                gripper_position_rad: 0.7,
                feedback_source: ArmFeedbackSource::Serial,
            },
            ..FakeData::default()
        }));
        io.connect("/dev/fake".to_owned(), Box::new(FakeBus(second)))
            .unwrap();
        assert!(io.serial_state().connected);
        assert_eq!(io.state().joints_rad, [0.11, 0.21, -0.31, 0.41, 0.51, 0.61]);
        assert_eq!(io.state().gripper_position_rad, 0.7);
    }

    #[test]
    fn arm_adapter_keeps_arm_joint_signs_and_negates_only_the_gripper() {
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
        let state = arm_state_from_monitors(monitors);
        assert_eq!(
            state.joints_rad,
            [1.0_f64, 2.0, 3.0, 4.0, 5.0, 6.0].map(f64::to_radians)
        );
        assert_eq!(state.gripper_position_rad, (-7.0_f64).to_radians());

        let commands = arm_position_commands(
            [
                0.05_f64.to_radians(),
                1.0_f64.to_radians(),
                2.0_f64.to_radians(),
                3.0_f64.to_radians(),
                4.0_f64.to_radians(),
                5.0_f64.to_radians(),
            ],
            90.0_f64.to_radians(),
        )
        .unwrap();
        assert_eq!(commands.map(|command| command.id), [0, 1, 2, 3, 4, 5, 6]);
        assert_eq!(commands[0].position_tenths_degree, 0);
        assert_eq!(commands[3].position_tenths_degree, 30);
        assert_eq!(commands[6].position_tenths_degree, -900);
        assert!(commands.iter().all(|command| {
            command.motion_time_ms == MOTION_TIME_MS
                && command.acceleration_time_ms == ACCELERATION_TIME_MS
                && command.deceleration_time_ms == DECELERATION_TIME_MS
                && command.power_mw == 0
        }));

        let mut reversed = monitors.to_vec();
        reversed.reverse();
        assert_eq!(sort_monitors(reversed).unwrap(), monitors);
        let mut duplicate = monitors.to_vec();
        duplicate[6] = duplicate[0];
        assert!(sort_monitors(duplicate).is_err());
    }

    #[test]
    fn serial_output_only_restarts_motion_when_the_encoded_command_changes() {
        let first = arm_position_commands([0.0; ARM_DOF], 0.0).unwrap();
        assert!(commands_changed(None, &first));
        assert!(!commands_changed(Some(&first), &first));

        let same_tenths = arm_position_commands([0.001_f64.to_radians(); ARM_DOF], 0.0).unwrap();
        assert_eq!(same_tenths, first);
        assert!(!commands_changed(Some(&first), &same_tenths));

        let changed = arm_position_commands([0.1_f64.to_radians(); ARM_DOF], 0.0).unwrap();
        assert_ne!(changed, first);
        assert!(commands_changed(Some(&first), &changed));
    }
}
