//! Public UART commands, cross-checked against C#, Arduino and MicroPython SDKs.
//! Values are protocol units, never robot joint coordinates. See ../README.md.
use crate::{Error, FashionStarBus, Monitor, Packet, PacketDecoder, Register, request_packet};
use std::{
    io::{Read, Write},
    time::Instant,
};

#[derive(Clone, Copy, Debug)]
pub enum StopMode {
    Release = 0x10,
    Hold = 0x11,
    Damping = 0x12,
}
#[derive(Clone, Copy, Debug)]
pub enum MotionProfile {
    Single {
        position: i16,
        time_ms: u16,
        power_mw: u16,
    },
    SingleInterval {
        position: i16,
        time_ms: u16,
        accel_ms: u16,
        decel_ms: u16,
        power_mw: u16,
    },
    SingleVelocity {
        position: i16,
        speed_tenths_dps: u16,
        accel_ms: u16,
        decel_ms: u16,
        power_mw: u16,
    },
    Multi {
        position: i32,
        time_ms: u32,
        power_mw: u16,
    },
    MultiInterval(crate::PositionCommand),
    MultiVelocity {
        position: i32,
        speed_tenths_dps: u16,
        accel_ms: u16,
        decel_ms: u16,
        power_mw: u16,
    },
}
impl MotionProfile {
    pub fn command(self, id: u8) -> Packet {
        let mut p = vec![id];
        macro_rules! append { ($($x:expr),*) => { $(p.extend_from_slice(&$x.to_le_bytes());)* }; }
        let code = match self {
            Self::Single {
                position,
                time_ms,
                power_mw,
            } => {
                append!(position, time_ms, power_mw);
                8
            }
            Self::SingleInterval {
                position,
                time_ms,
                accel_ms,
                decel_ms,
                power_mw,
            } => {
                append!(position, time_ms, accel_ms, decel_ms, power_mw);
                11
            }
            Self::SingleVelocity {
                position,
                speed_tenths_dps,
                accel_ms,
                decel_ms,
                power_mw,
            } => {
                append!(position, speed_tenths_dps, accel_ms, decel_ms, power_mw);
                12
            }
            Self::Multi {
                position,
                time_ms,
                power_mw,
            } => {
                append!(position, time_ms, power_mw);
                13
            }
            Self::MultiInterval(mut c) => {
                c.id = id;
                p.clear();
                c.append_to(&mut p);
                14
            }
            Self::MultiVelocity {
                position,
                speed_tenths_dps,
                accel_ms,
                decel_ms,
                power_mw,
            } => {
                append!(position, speed_tenths_dps, accel_ms, decel_ms, power_mw);
                15
            }
        };
        Packet { code, params: p }
    }
}

/// Legacy batch block: exactly 32 bytes, including reserved/identity bytes.
/// Obtain it by reading the device; never manufacture a block of defaults.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UserData(pub [u8; 32]);

#[derive(Clone, Debug)]
pub enum ServoCommand {
    ReadInternal(u8),
    WriteInternal(crate::InternalParameters),
    Ping(u8),
    ResetUserData(u8),
    ReadRegister {
        id: u8,
        register: Register,
    },
    WriteRegister {
        id: u8,
        register: Register,
        value: i64,
    },
    ReadUserData(u8),
    WriteUserData {
        id: u8,
        data: UserData,
    },
    Motion {
        id: u8,
        profile: MotionProfile,
    },
    Damping {
        id: u8,
        power_mw: u16,
    },
    ReadAngle(u8),
    ReadMultiAngle(u8),
    ResetTurns(u8),
    BeginAsync,
    EndAsync {
        cancel: bool,
    },
    Monitor(u8),
    Origin {
        id: u8,
        restore: bool,
    },
    Stop {
        id: u8,
        mode: StopMode,
        power_mw: u16,
    },
}
impl ServoCommand {
    pub fn packet(&self) -> Result<Packet, Error> {
        let (code, params) = match self {
            Self::ReadInternal(id) => (0xc5, vec![*id]),
            Self::WriteInternal(value) => {
                let frame = value.write_request();
                (0xc4, frame[4..frame.len() - 1].to_vec())
            }
            Self::Ping(id) => (1, vec![*id]),
            Self::ResetUserData(id) => (2, vec![*id]),
            Self::ReadRegister { id, register } => (3, vec![*id, register.address]),
            Self::WriteRegister {
                id,
                register,
                value,
            } => {
                let mut p = vec![*id, register.address];
                p.extend(register.encode(*value)?);
                (4, p)
            }
            Self::ReadUserData(id) => (5, vec![*id]),
            Self::WriteUserData { id, data } => {
                let mut p = vec![*id];
                p.extend(data.0);
                (6, p)
            }
            Self::Motion { id, profile } => return Ok(profile.command(*id)),
            Self::Damping { id, power_mw } => {
                let mut p = vec![*id];
                p.extend(power_mw.to_le_bytes());
                (9, p)
            }
            Self::ReadAngle(id) => (10, vec![*id]),
            Self::ReadMultiAngle(id) => (16, vec![*id]),
            Self::ResetTurns(id) => (17, vec![*id]),
            Self::BeginAsync => (18, vec![]),
            Self::EndAsync { cancel } => (19, vec![u8::from(*cancel)]),
            Self::Monitor(id) => (22, vec![*id]),
            Self::Origin { id, restore } => (23, vec![*id, u8::from(*restore)]),
            Self::Stop { id, mode, power_mw } => {
                let mut p = vec![*id, *mode as u8];
                p.extend(power_mw.to_le_bytes());
                (24, p)
            }
        };
        Ok(Packet { code, params })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum CommandReply {
    Internal(crate::InternalParameters),
    Acknowledged,
    /// UART sent, but firmware optional ACK is absent. NOT verified success.
    SentUnconfirmed,
    Register(f64),
    UserData(UserData),
    Angle {
        position_tenths_degree: i32,
        turns: Option<i16>,
    },
    Monitor(Monitor),
}
pub(crate) struct PendingCommand {
    command: ServoCommand,
    packet: Packet,
    started: Instant,
}

impl FashionStarBus {
    pub fn command_pending(&self) -> bool {
        self.command.is_some()
    }
    pub fn cancel_command(&mut self) -> Result<(), Error> {
        if self.command.take().is_some() {
            self.clear_input()?;
        }
        Ok(())
    }
    /// Nonblocking maintenance transaction. No automatic retry of writes.
    pub fn begin_command(&mut self, command: ServoCommand) -> Result<(), Error> {
        if self.command_pending() || self.data_read_pending() || self.monitor_read_pending() {
            return Err(Error::Protocol("串口已有事务".into()));
        }
        let packet = command.packet()?;
        self.clear_input()?;
        if matches!(
            command,
            ServoCommand::ReadInternal(_) | ServoCommand::WriteInternal(_)
        ) {
            let mut bytes = request_packet(packet.code, &packet.params)?;
            bytes[0] = 0x13;
            bytes[1] = 0x4d;
            let end = bytes.len() - 1;
            bytes[end] = crate::checksum(&bytes[..end]);
            self.port.write_all(&bytes)?;
        } else {
            self.send(packet.code, &packet.params)?;
        }
        self.command = Some(PendingCommand {
            command,
            packet,
            started: Instant::now(),
        });
        Ok(())
    }
    pub fn poll_command(&mut self) -> Result<Option<CommandReply>, Error> {
        let Some(pending) = self.command.take() else {
            return Ok(None);
        };
        if matches!(
            pending.command,
            ServoCommand::BeginAsync | ServoCommand::EndAsync { .. }
        ) {
            return Ok(Some(CommandReply::SentUnconfirmed));
        }
        let mut bytes = [0; 256];
        let available = (self.port.bytes_to_read()? as usize).min(bytes.len());
        if available > 0 {
            let n = self.port.read(&mut bytes[..available])?;
            for b in &bytes[..n] {
                if let Some(Ok(p)) = self.decoder.push(*b)
                    && p.code == pending.packet.code
                    && p.params.first() == pending.packet.params.first()
                {
                    // Other register responses cannot complete this transaction.
                    if matches!(p.code, 3 | 4) && p.params.get(1) != pending.packet.params.get(1) {
                        continue;
                    }
                    return parse_reply(&pending.command, &p).map(Some);
                }
            }
        }
        let timeout = crate::DEFAULT_TIMEOUT
            + if pending.packet.code == 0xc5 {
                crate::INTERNAL_PARAMETERS_RESPONSE_DELAY
            } else {
                std::time::Duration::ZERO
            };
        if pending.started.elapsed() >= timeout {
            self.decoder = PacketDecoder::responses();
            return if matches!(
                pending.packet.code,
                1 | 2 | 3 | 5 | 10 | 16 | 22 | 0xc4 | 0xc5
            ) {
                Err(Error::Protocol(format!(
                    "指令 0x{:02x} 响应超时",
                    pending.packet.code
                )))
            } else {
                Ok(Some(CommandReply::SentUnconfirmed))
            };
        }
        self.command = Some(pending);
        Ok(None)
    }
    /// Send synchronized motion using the same encoder as individual motion.
    pub fn write_synchronized(&mut self, commands: &[(u8, MotionProfile)]) -> Result<(), Error> {
        self.port.write_all(&synchronized_packet(commands)?)?;
        Ok(())
    }
    /// Host serial speed, not a servo EEPROM write. Caller owns bus reconfiguration.
    pub fn set_host_baud_rate(&mut self, baud: u32) -> Result<(), Error> {
        self.port.set_baud_rate(baud)?;
        Ok(())
    }
}

pub fn synchronized_packet(commands: &[(u8, MotionProfile)]) -> Result<Vec<u8>, Error> {
    let packets: Vec<_> = commands.iter().map(|(id, c)| c.command(*id)).collect();
    let first = packets
        .first()
        .ok_or_else(|| Error::Protocol("同步指令不能为空".into()))?;
    let count = u8::try_from(packets.len())
        .map_err(|_| Error::Protocol("同步指令数量超出协议 u8".into()))?;
    let mut p = vec![first.code, first.params.len() as u8, count];
    for packet in &packets {
        if packet.code != first.code || packet.params.len() != first.params.len() {
            return Err(Error::Protocol("同步指令必须使用相同控制模式".into()));
        }
        p.extend(&packet.params);
    }
    if p.len() < 255 {
        return request_packet(25, &p);
    }
    // C# RequestHeaderEx / STM32 FSUS_SendPackage_Common(isSync=1).
    let length =
        u16::try_from(p.len()).map_err(|_| Error::Protocol("扩展同步指令超出 u16".into()))?;
    let mut frame = vec![0x12, 0x4c, 25, 255];
    frame.extend(length.to_le_bytes());
    frame.extend(p);
    frame.push(crate::checksum(&frame));
    Ok(frame)
}

fn parse_reply(command: &ServoCommand, p: &Packet) -> Result<CommandReply, Error> {
    let malformed =
        || Error::Protocol(format!("指令 0x{:02x} 响应无效：{:02x?}", p.code, p.params));
    Ok(match command {
        ServoCommand::ReadInternal(id) => {
            CommandReply::Internal(crate::InternalParameters::from_response(
                *id,
                &crate::response_packet(p.code, &p.params)?,
            )?)
        }
        ServoCommand::WriteInternal(_) if p.params.len() == 2 && p.params[1] == 1 => {
            CommandReply::Acknowledged
        }
        ServoCommand::Ping(_) if p.params.len() == 1 => CommandReply::Acknowledged,
        ServoCommand::ReadRegister { register, .. } => {
            CommandReply::Register(register.decode(&p.params[2..])?)
        }
        ServoCommand::ReadUserData(_) => {
            CommandReply::UserData(UserData(p.params[1..].try_into().map_err(|_| malformed())?))
        }
        ServoCommand::ReadAngle(_) => CommandReply::Angle {
            position_tenths_degree: i32::from(i16::from_le_bytes(
                p.params[1..].try_into().map_err(|_| malformed())?,
            )),
            turns: None,
        },
        ServoCommand::ReadMultiAngle(_) if p.params.len() == 7 => CommandReply::Angle {
            position_tenths_degree: i32::from_le_bytes(p.params[1..5].try_into().unwrap()),
            turns: Some(i16::from_le_bytes(p.params[5..7].try_into().unwrap())),
        },
        ServoCommand::Monitor(_) => CommandReply::Monitor(Monitor::from_params(&p.params)?),
        ServoCommand::WriteRegister { .. } if p.params.len() == 3 && p.params[2] == 1 => {
            CommandReply::Acknowledged
        }
        ServoCommand::ResetUserData(_)
        | ServoCommand::WriteUserData { .. }
        | ServoCommand::Motion { .. }
        | ServoCommand::Damping { .. }
        | ServoCommand::ResetTurns(_)
        | ServoCommand::Origin { .. }
        | ServoCommand::Stop { .. }
            if p.params.len() == 2 && p.params[1] == 1 =>
        {
            CommandReply::Acknowledged
        }
        _ => return Err(malformed()),
    })
}
