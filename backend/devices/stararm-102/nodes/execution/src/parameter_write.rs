//! Nonblocking parameter writes on the production UART owner. ACK alone is
//! never acceptance: both public registers and internal PID require readback.
use super::*;
use fashionstar_uart::{CommandReply, REGISTERS, Register, ServoCommand, StopMode};

pub const PID_KEYS: &[&str] = &["kp", "kd", "ki", "bias", "hold_kp", "hold_kd", "hold_bias"];
#[derive(Clone, Copy)]
enum Phase {
    WriteRegister,
    ReadRegister,
    ReadInternal,
    Release,
    WriteInternal,
    Hold,
    VerifyInternal,
}
pub(super) struct ParameterWrite {
    register: Option<Register>,
    key: String,
    id: u8,
    value: i64,
    phase: Phase,
    expected: Option<InternalParameters>,
    ready_at: Option<Instant>,
    released: bool,
}
fn update_pid(p: &mut InternalParameters, key: &str, value: i64) -> Result<(), String> {
    let value = u16::try_from(value).map_err(|_| "PID 参数必须能编码为 u16".to_owned())?;
    match key {
        "kp" => p.kp = value,
        "kd" => p.kd = value,
        "ki" => p.ki = value,
        "bias" => p.bias = value,
        "hold_kp" => p.hold_kp = value,
        "hold_kd" => p.hold_kd = value,
        "hold_bias" => p.hold_bias = value,
        _ => return Err("未知 PID 字段".into()),
    }
    Ok(())
}
impl ParameterWrite {
    /// Internal PID writes issue Stop commands. After success, failure or
    /// cancellation, the next normal target must not be deduplicated away.
    pub fn stops_motion(&self) -> bool {
        self.register.is_none()
    }

    pub fn new(request: ExecutionRequest) -> Result<Self, String> {
        let key = request
            .fields
            .get("field_key")
            .ok_or("缺少 field_key")?
            .clone();
        let register = REGISTERS.iter().find(|r| r.key == key).copied();
        let actuator = request
            .fields
            .get("actuator_key")
            .ok_or("缺少 actuator_key")?;
        let id = JOINTS
            .iter()
            .position(|key| key == actuator)
            .map(|i| i as u8)
            .or_else(|| (actuator == "gripper").then_some(6))
            .ok_or("未知执行器")?;
        let value = request
            .fields
            .get("value")
            .ok_or("缺少 value")?
            .parse::<i64>()
            .map_err(|e| e.to_string())?;
        let phase = if let Some(r) = register {
            r.encode(value).map_err(|e| e.to_string())?;
            if matches!(r.address, 34 | 36) {
                return Err("ID / 波特率需连同机械臂设备映射修改；通用驱动已支持，当前机械臂参数页不更改连接契约".into());
            }
            Phase::WriteRegister
        } else {
            if !PID_KEYS.contains(&key.as_str()) {
                return Err("未知可写参数".into());
            }
            u16::try_from(value).map_err(|_| "PID 参数必须能编码为 u16".to_owned())?;
            Phase::ReadInternal
        };
        Ok(Self {
            register,
            key,
            id,
            value,
            phase,
            expected: None,
            ready_at: None,
            released: false,
        })
    }
    pub fn poll(&mut self, bus: &mut FashionStarBus) -> Result<Option<ParameterValue>, String> {
        let result = self.poll_inner(bus);
        if let Err(error) = &result
            && self.released
        {
            self.released = false;
            if let Err(restore) = bus.hold_torque(self.id) {
                return Err(format!("{}；恢复上力也失败：{restore}", error));
            }
        }
        result
    }
    pub fn restore(&mut self, bus: &mut FashionStarBus) -> Result<(), String> {
        if self.released {
            bus.hold_torque(self.id).map_err(|e| e.to_string())?;
            self.released = false;
        }
        Ok(())
    }
    fn poll_inner(&mut self, bus: &mut FashionStarBus) -> Result<Option<ParameterValue>, String> {
        if bus.monitor_read_pending() || bus.data_read_pending() {
            return Ok(None);
        }
        if self.ready_at.is_some_and(|t| Instant::now() < t) {
            return Ok(None);
        }
        if !bus.command_pending() {
            let command = match self.phase {
                Phase::WriteRegister => ServoCommand::WriteRegister {
                    id: self.id,
                    register: self.register.unwrap(),
                    value: self.value,
                },
                Phase::ReadRegister => ServoCommand::ReadRegister {
                    id: self.id,
                    register: self.register.unwrap(),
                },
                Phase::ReadInternal | Phase::VerifyInternal => ServoCommand::ReadInternal(self.id),
                Phase::Release => ServoCommand::Stop {
                    id: self.id,
                    mode: StopMode::Release,
                    power_mw: 0,
                },
                Phase::WriteInternal => ServoCommand::WriteInternal(self.expected.unwrap()),
                Phase::Hold => ServoCommand::Stop {
                    id: self.id,
                    mode: StopMode::Hold,
                    power_mw: 0,
                },
            };
            bus.begin_command(command).map_err(|e| e.to_string())?;
            if matches!(self.phase, Phase::Release) {
                self.released = true;
            }
            return Ok(None);
        }
        let Some(reply) = bus.poll_command().map_err(|e| e.to_string())? else {
            return Ok(None);
        };
        self.phase = match self.phase {
            Phase::WriteRegister => Phase::ReadRegister,
            Phase::ReadInternal => {
                let CommandReply::Internal(mut p) = reply else {
                    return Err("内部参数响应类型不符".into());
                };
                update_pid(&mut p, &self.key, self.value)?;
                self.expected = Some(p);
                Phase::Release
            }
            Phase::Release => {
                // Vendor private-write settling time, not a motion limit.
                self.ready_at =
                    Some(Instant::now() + fashionstar_uart::INTERNAL_PARAMETERS_WRITE_DELAY);
                Phase::WriteInternal
            }
            Phase::WriteInternal => Phase::Hold,
            Phase::Hold => {
                self.released = false;
                Phase::VerifyInternal
            }
            Phase::ReadRegister | Phase::VerifyInternal => {
                let matches = match reply {
                    CommandReply::Register(v) => v == self.value as f64,
                    CommandReply::Internal(p) => Some(p) == self.expected,
                    _ => false,
                };
                if !matches {
                    return Err(format!(
                        "{} 写入后回读不一致：请求 {}，返回 {reply:?}",
                        self.key, self.value
                    ));
                }
                return Ok(Some(ParameterValue {
                    actuator_key: actuator_key(self.id),
                    field_key: self.key.clone(),
                    value: Some(self.value as f64),
                    unit: self.register.map_or("", |r| r.unit).into(),
                    read_time_ns: now_ns(),
                    original_error: None,
                }));
            }
        };
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(unix)]
    #[test]
    fn pid_write_preserves_the_block_and_restores_torque_on_error() {
        use serialport::SerialPort;
        use std::io::{Read, Write};
        fn packet(peer: &mut serialport::TTYPort) -> (u8, Vec<u8>) {
            let mut header = [0; 4];
            peer.read_exact(&mut header).unwrap();
            assert!(header[..2] == [0x12, 0x4c] || header[..2] == [0x13, 0x4d]);
            let mut data = vec![0; usize::from(header[3]) + 1];
            peer.read_exact(&mut data).unwrap();
            let checksum = header
                .iter()
                .chain(data[..data.len() - 1].iter())
                .fold(0u8, |a, b| a.wrapping_add(*b));
            assert_eq!(data.pop().unwrap(), checksum);
            (header[2], data)
        }
        for failure in [false, true] {
            let (mut peer, port) = serialport::TTYPort::pair().unwrap();
            peer.set_timeout(Duration::from_secs(2)).unwrap();
            let bus = FashionStarBus::open(&port.name().unwrap()).unwrap();
            let thread = std::thread::spawn(move || {
                let mut original = vec![0u8; 26];
                original[1..3].copy_from_slice(&200u16.to_le_bytes());
                original[3..5].copy_from_slice(&20u16.to_le_bytes());
                original[25] = 1;
                assert_eq!(packet(&mut peer), (0xc5, vec![0]));
                peer.write_all(&fashionstar_uart::response_packet(0xc5, &original).unwrap())
                    .unwrap();
                assert_eq!(packet(&mut peer), (24, vec![0, 0x10, 0, 0]));
                peer.write_all(&fashionstar_uart::response_packet(24, &[0, 1]).unwrap())
                    .unwrap();
                let (code, actual) = packet(&mut peer);
                assert_eq!(code, 0xc4);
                let mut expected = original;
                expected[1..3].copy_from_slice(&201u16.to_le_bytes());
                assert_eq!(actual, expected);
                peer.write_all(
                    &fashionstar_uart::response_packet(0xc4, &[0, u8::from(!failure)]).unwrap(),
                )
                .unwrap();
                assert_eq!(packet(&mut peer), (24, vec![0, 0x11, 0, 0]));
                if failure {
                    return;
                }
                peer.write_all(&fashionstar_uart::response_packet(24, &[0, 1]).unwrap())
                    .unwrap();
                assert_eq!(packet(&mut peer), (0xc5, vec![0]));
                peer.write_all(&fashionstar_uart::response_packet(0xc5, &expected).unwrap())
                    .unwrap();
                // Keep the PTY alive until the reader consumes the final reply.
                std::thread::sleep(Duration::from_millis(20));
            });
            let write = ParameterWrite::new(ExecutionRequest {
                schema_version: SCHEMA_VERSION,
                request_id: "pid-test".into(),
                action: RequestAction::Apply,
                fields: BTreeMap::from([
                    ("actuator_key".into(), JOINTS[0].into()),
                    ("field_key".into(), "kp".into()),
                    ("value".into(), "201".into()),
                ]),
            })
            .unwrap();
            let mut execution = StarArmExecution::new();
            execution.bus = Some(StarArmBus {
                bus,
                last_commands: Some(cached_commands()),
            });
            execution.parameter_write = Some(write);
            execution.transport.parameter_write_pending = true;
            execution.transport.parameter_values = vec![ParameterValue {
                actuator_key: JOINTS[0].into(),
                field_key: "kp".into(),
                value: None,
                unit: String::new(),
                read_time_ns: 0,
                original_error: Some("previous read failed".into()),
            }];
            execution.transport.parameter_error = Some("previous read failed".into());
            let started = Instant::now();
            while execution.parameter_write.is_some() {
                assert!(started.elapsed() < Duration::from_secs(2));
                execution.poll_hardware();
                std::thread::sleep(Duration::from_millis(1));
            }
            assert_eq!(execution.transport.parameter_write_error.is_some(), failure);
            assert_eq!(execution.transport.parameter_write_verified, !failure);
            assert!(!execution.transport.parameter_write_pending);
            assert!(execution.bus.as_ref().unwrap().last_commands.is_none());
            if !failure {
                assert_eq!(execution.transport.parameter_values[0].value, Some(201.0));
                assert!(execution.transport.parameter_error.is_none());
            }
            thread.join().unwrap();
        }
    }
    #[cfg(unix)]
    fn cached_commands() -> [PositionCommand; 7] {
        [PositionCommand {
            id: 0,
            position_tenths_degree: 0,
            motion_time_ms: 100,
            acceleration_time_ms: 0,
            deceleration_time_ms: 0,
            power_mw: 0,
        }; 7]
    }
    #[cfg(unix)]
    #[test]
    fn cancelling_pid_invalidates_motion_cache_but_register_write_does_not() {
        use serialport::SerialPort;
        for (key, stops_motion) in [("kp", true), ("soft_start_time", false)] {
            let (_peer, port) = serialport::TTYPort::pair().unwrap();
            let mut execution = StarArmExecution::new();
            execution.bus = Some(StarArmBus {
                bus: FashionStarBus::open(&port.name().unwrap()).unwrap(),
                last_commands: Some(cached_commands()),
            });
            execution
                .apply_parameters(&BTreeMap::from([
                    ("actuator_key".into(), JOINTS[0].into()),
                    ("field_key".into(), key.into()),
                    ("value".into(), "100".into()),
                ]))
                .unwrap();
            execution.cancel_parameter_write();
            assert_eq!(
                execution.bus.as_ref().unwrap().last_commands.is_none(),
                stops_motion
            );
            assert!(!execution.transport.parameter_write_pending);
            assert!(
                execution
                    .transport
                    .parameter_write_error
                    .as_deref()
                    .unwrap()
                    .contains("已取消")
            );
        }
    }
    #[test]
    fn editable_fields_are_typed_and_not_read_only_metadata() {
        let request = |key: &str, value: &str| ExecutionRequest {
            schema_version: SCHEMA_VERSION,
            request_id: "test".into(),
            action: RequestAction::Apply,
            fields: BTreeMap::from([
                ("actuator_key".into(), JOINTS[0].into()),
                ("field_key".into(), key.into()),
                ("value".into(), value.into()),
            ]),
        };
        assert!(ParameterWrite::new(request("angle_limit_low", "-650")).is_ok());
        for key in PID_KEYS {
            assert!(ParameterWrite::new(request(key, "200")).is_ok());
        }
        for key in [
            "version_info",
            "reserved",
            "firmware_version",
            "serial_number",
        ] {
            assert!(ParameterWrite::new(request(key, "1")).is_err());
        }
        assert!(ParameterWrite::new(request("response_switch", "256")).is_err());
        assert!(ParameterWrite::new(request("kp", "-1")).is_err());
    }
}
