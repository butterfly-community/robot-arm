use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::{
    fmt,
    io::{self, Read, Write},
    thread,
    time::Duration,
};

pub const DEFAULT_BAUD_RATE: u32 = 1_000_000;
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(100);
pub const CODE_PING: u8 = 1;
pub const CODE_SET_MTURN_BY_INTERVAL: u8 = 14;
pub const CODE_QUERY_MONITOR: u8 = 22;
pub const CODE_SYNC_COMMAND: u8 = 25;
const CODE_STOP_CONTROL: u8 = 0x18;

const REQUEST_HEADER: [u8; 2] = [0x12, 0x4c];
const RESPONSE_HEADER: [u8; 2] = [0x05, 0x1c];
const FRAME_OVERHEAD: usize = 5;
const INVALID_MONITOR_POSITION: i32 = -235_929_599;
const INTERNAL_PARAMETERS_REQUEST_HEADER: [u8; 4] = [0x13, 0x4d, 0xc5, 0x01];
const INTERNAL_PARAMETERS_RESPONSE_HEADER: [u8; 4] = [0x05, 0x1c, 0xc5, 0x1a];
const INTERNAL_PARAMETERS_WRITE_HEADER: [u8; 4] = [0x13, 0x4d, 0xc4, 0x1a];
const INTERNAL_PARAMETERS_RESPONSE_SIZE: usize = 31;
const INTERNAL_PARAMETERS_RESPONSE_DELAY: Duration = Duration::from_millis(200);
const INTERNAL_PARAMETERS_WRITE_DELAY: Duration = Duration::from_millis(60);

#[derive(Debug)]
pub enum Error {
    Serial(serialport::Error),
    Io(io::Error),
    Protocol(String),
}

impl fmt::Display for Error {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Serial(error) => error.fmt(formatter),
            Self::Io(error) => error.fmt(formatter),
            Self::Protocol(error) => formatter.write_str(error),
        }
    }
}

impl std::error::Error for Error {}
impl From<serialport::Error> for Error {
    fn from(value: serialport::Error) -> Self {
        Self::Serial(value)
    }
}
impl From<io::Error> for Error {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Packet {
    pub code: u8,
    pub params: Vec<u8>,
}

pub fn request_packet(code: u8, params: &[u8]) -> Result<Vec<u8>, Error> {
    encode_packet(REQUEST_HEADER, code, params)
}

pub fn response_packet(code: u8, params: &[u8]) -> Result<Vec<u8>, Error> {
    encode_packet(RESPONSE_HEADER, code, params)
}

fn encode_packet(header: [u8; 2], code: u8, params: &[u8]) -> Result<Vec<u8>, Error> {
    let size = u8::try_from(params.len())
        .map_err(|_| Error::Protocol("协议参数长度超过一个字节".to_owned()))?;
    let mut packet = Vec::with_capacity(params.len() + FRAME_OVERHEAD);
    packet.extend_from_slice(&header);
    packet.extend_from_slice(&[code, size]);
    packet.extend_from_slice(params);
    packet.push(checksum(&packet));
    Ok(packet)
}

fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0_u8, |sum, byte| sum.wrapping_add(*byte))
}

#[derive(Debug)]
pub struct PacketDecoder {
    header: [u8; 2],
    buffer: Vec<u8>,
}

impl PacketDecoder {
    pub fn requests() -> Self {
        Self::new(REQUEST_HEADER)
    }
    pub fn responses() -> Self {
        Self::new(RESPONSE_HEADER)
    }
    fn new(header: [u8; 2]) -> Self {
        Self {
            header,
            buffer: Vec::new(),
        }
    }
    pub fn push(&mut self, byte: u8) -> Option<Result<Packet, Error>> {
        self.buffer.push(byte);
        self.discard_before_header();
        if self.buffer.len() < 4 {
            return None;
        }
        let length = usize::from(self.buffer[3]) + FRAME_OVERHEAD;
        if self.buffer.len() < length {
            return None;
        }
        let frame = self.buffer.drain(..length).collect::<Vec<_>>();
        if checksum(&frame[..length - 1]) != frame[length - 1] {
            self.discard_before_header();
            return Some(Err(Error::Protocol("数据帧校验和错误".to_owned())));
        }
        Some(Ok(Packet {
            code: frame[2],
            params: frame[4..length - 1].to_vec(),
        }))
    }
    fn discard_before_header(&mut self) {
        if self.buffer.starts_with(&self.header) {
            return;
        }
        if let Some(start) = self
            .buffer
            .windows(2)
            .position(|bytes| bytes == self.header)
        {
            self.buffer.drain(..start);
        } else if self.buffer.last() == Some(&self.header[0]) {
            let last = self.buffer.len() - 1;
            self.buffer.drain(..last);
        } else {
            self.buffer.clear();
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Monitor {
    pub id: u8,
    pub voltage_mv: u16,
    pub current_ma: u16,
    pub power_mw: u16,
    pub temperature_raw: u16,
    pub status: u8,
    pub position_tenths_degree: i32,
    pub turns: i16,
}

impl Monitor {
    pub fn from_params(params: &[u8]) -> Result<Self, Error> {
        if params.len() != 16 {
            return Err(Error::Protocol(format!(
                "Monitor 数据长度应为 16，实际为 {}",
                params.len()
            )));
        }
        let position = i32::from_le_bytes(params[10..14].try_into().expect("four bytes"));
        let turns = i16::from_le_bytes(params[14..16].try_into().expect("two bytes"));
        if position == INVALID_MONITOR_POSITION && turns == 0 {
            return Err(Error::Protocol(format!(
                "Monitor ID {} 返回了厂家无效位置标记",
                params[0]
            )));
        }
        Ok(Self {
            id: params[0],
            voltage_mv: u16::from_le_bytes(params[1..3].try_into().expect("two bytes")),
            current_ma: u16::from_le_bytes(params[3..5].try_into().expect("two bytes")),
            power_mw: u16::from_le_bytes(params[5..7].try_into().expect("two bytes")),
            temperature_raw: u16::from_le_bytes(params[7..9].try_into().expect("two bytes")),
            status: params[9],
            position_tenths_degree: position,
            turns,
        })
    }
    pub fn position_degrees(self) -> f64 {
        f64::from(self.position_tenths_degree) / 10.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PositionCommand {
    pub id: u8,
    pub position_tenths_degree: i32,
    pub motion_time_ms: u32,
    pub acceleration_time_ms: u16,
    pub deceleration_time_ms: u16,
    pub power_mw: u16,
}

impl PositionCommand {
    fn append_to(self, params: &mut Vec<u8>) {
        params.push(self.id);
        params.extend_from_slice(&self.position_tenths_degree.to_le_bytes());
        params.extend_from_slice(&self.motion_time_ms.to_le_bytes());
        params.extend_from_slice(&self.acceleration_time_ms.to_le_bytes());
        params.extend_from_slice(&self.deceleration_time_ms.to_le_bytes());
        params.extend_from_slice(&self.power_mw.to_le_bytes());
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InternalParameters {
    pub id: u8,
    pub kp: u16,
    pub kd: u16,
    pub ki: u16,
    pub bias: u16,
    pub hold_kp: u16,
    pub hold_kd: u16,
    pub hold_bias: u16,
    pub full_deg: u16,
    pub reserved: u16,
    pub pwm_limit: u16,
    pub direction: u8,
    pub pwm_frequency: u8,
    pub dead_zone: u8,
    pub motor_direction: u8,
    pub version_info: u8,
}

impl InternalParameters {
    pub fn from_response(expected_id: u8, response: &[u8]) -> Result<Self, Error> {
        if response.len() != INTERNAL_PARAMETERS_RESPONSE_SIZE {
            return Err(Error::Protocol(format!(
                "内部参数响应长度应为 {INTERNAL_PARAMETERS_RESPONSE_SIZE}，实际为 {}",
                response.len()
            )));
        }
        if checksum(&response[..response.len() - 1]) != response[response.len() - 1] {
            return Err(Error::Protocol(format!(
                "舵机 {expected_id} 内部参数响应校验失败"
            )));
        }
        if response[..4] != INTERNAL_PARAMETERS_RESPONSE_HEADER {
            return Err(Error::Protocol(format!(
                "舵机 {expected_id} 内部参数响应头无效：{:02x?}",
                &response[..4]
            )));
        }
        if response[4] != expected_id {
            return Err(Error::Protocol(format!(
                "内部参数请求 ID {expected_id}，响应 ID {}",
                response[4]
            )));
        }
        let value = |offset| u16::from_le_bytes([response[offset], response[offset + 1]]);
        Ok(Self {
            id: response[4],
            kp: value(5),
            kd: value(7),
            ki: value(9),
            bias: value(11),
            hold_kp: value(13),
            hold_kd: value(15),
            hold_bias: value(17),
            full_deg: value(19),
            reserved: value(21),
            pwm_limit: value(23),
            direction: response[25],
            pwm_frequency: response[26],
            dead_zone: response[27],
            motor_direction: response[28],
            version_info: response[29],
        })
    }

    fn write_request(self) -> [u8; INTERNAL_PARAMETERS_RESPONSE_SIZE] {
        let mut request = Vec::with_capacity(INTERNAL_PARAMETERS_RESPONSE_SIZE);
        request.extend_from_slice(&INTERNAL_PARAMETERS_WRITE_HEADER);
        request.push(self.id);
        for value in [
            self.kp,
            self.kd,
            self.ki,
            self.bias,
            self.hold_kp,
            self.hold_kd,
            self.hold_bias,
            self.full_deg,
            self.reserved,
            self.pwm_limit,
        ] {
            request.extend_from_slice(&value.to_le_bytes());
        }
        request.extend_from_slice(&[
            self.direction,
            self.pwm_frequency,
            self.dead_zone,
            self.motor_direction,
            self.version_info,
        ]);
        request.push(checksum(&request));
        request
            .try_into()
            .expect("internal parameter request has a fixed size")
    }
}

pub fn degrees_to_tenths(degrees: f64) -> Result<i32, Error> {
    let rounded = (degrees * 10.0).round_ties_even();
    if !rounded.is_finite() || rounded < f64::from(i32::MIN) || rounded > f64::from(i32::MAX) {
        return Err(Error::Protocol(format!(
            "位置 {degrees}° 无法编码为协议 i32 角度"
        )));
    }
    Ok(rounded as i32)
}

pub struct FashionStarBus {
    port: Box<dyn SerialPort>,
    decoder: PacketDecoder,
}

impl FashionStarBus {
    pub fn open(path: &str) -> Result<Self, Error> {
        let port = serialport::new(path, DEFAULT_BAUD_RATE)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(DEFAULT_TIMEOUT)
            .open()?;
        Ok(Self {
            port,
            decoder: PacketDecoder::responses(),
        })
    }
    pub fn ping(&mut self, id: u8) -> Result<(), Error> {
        self.send(CODE_PING, &[id])?;
        let response = self.receive()?;
        if response.code != CODE_PING || response.params.as_slice() != [id] {
            return Err(Error::Protocol(format!("舵机 ID {id} 的 Ping 响应无效")));
        }
        Ok(())
    }
    pub fn read_monitors(&mut self, ids: &[u8]) -> Result<Vec<Monitor>, Error> {
        match self.read_monitors_once(ids) {
            Ok(monitors) => Ok(monitors),
            Err(first_error) => {
                self.clear_input().map_err(|clear_error| {
                    Error::Protocol(format!(
                        "Monitor 首次读取失败：{first_error}；清理串口输入失败：{clear_error}"
                    ))
                })?;
                self.read_monitors_once(ids).map_err(|retry_error| {
                    Error::Protocol(format!(
                        "Monitor 首次读取失败：{first_error}；重试仍失败：{retry_error}"
                    ))
                })
            }
        }
    }
    fn read_monitors_once(&mut self, ids: &[u8]) -> Result<Vec<Monitor>, Error> {
        let count = u8::try_from(ids.len())
            .map_err(|_| Error::Protocol("Monitor 舵机数量超过一个字节".to_owned()))?;
        let mut params = vec![CODE_QUERY_MONITOR, 1, count];
        params.extend_from_slice(ids);
        self.send(CODE_SYNC_COMMAND, &params)?;
        (0..ids.len())
            .map(|_| {
                let response = loop {
                    let response = self.receive()?;
                    if response.code == CODE_QUERY_MONITOR {
                        break response;
                    }
                };
                Monitor::from_params(&response.params)
            })
            .collect()
    }
    pub fn write_positions(&mut self, commands: &[PositionCommand]) -> Result<(), Error> {
        let count = u8::try_from(commands.len())
            .map_err(|_| Error::Protocol("同步位置命令数量超过一个字节".to_owned()))?;
        let mut params = vec![CODE_SET_MTURN_BY_INTERVAL, 15, count];
        for command in commands {
            command.append_to(&mut params);
        }
        self.send(CODE_SYNC_COMMAND, &params)
    }
    pub fn read_internal_parameters(&mut self, id: u8) -> Result<InternalParameters, Error> {
        self.port.clear(ClearBuffer::Input)?;
        self.decoder = PacketDecoder::responses();
        let result = (|| {
            let mut request = INTERNAL_PARAMETERS_REQUEST_HEADER.to_vec();
            request.push(id);
            request.push(checksum(&request));
            self.port.write_all(&request)?;
            thread::sleep(INTERNAL_PARAMETERS_RESPONSE_DELAY);
            let mut response = [0_u8; INTERNAL_PARAMETERS_RESPONSE_SIZE];
            self.port.read_exact(&mut response)?;
            InternalParameters::from_response(id, &response)
        })();
        self.decoder = PacketDecoder::responses();
        if result.is_err() {
            let _ = self.port.clear(ClearBuffer::Input);
        }
        result
    }
    pub fn write_internal_parameters(
        &mut self,
        parameters: InternalParameters,
    ) -> Result<(), Error> {
        self.port.clear(ClearBuffer::Input)?;
        self.decoder = PacketDecoder::responses();
        let result = (|| {
            self.send(CODE_STOP_CONTROL, &[parameters.id, 0x10, 0, 0])?;
            thread::sleep(INTERNAL_PARAMETERS_WRITE_DELAY);
            self.port.write_all(&parameters.write_request())?;
            let response = loop {
                let response = self.receive()?;
                if response.code == 0xc4 {
                    break response;
                }
            };
            if response.params.as_slice() != [parameters.id, 1] {
                return Err(Error::Protocol(format!(
                    "舵机 {} 内部参数写入响应无效：params={:02x?}",
                    parameters.id, response.params
                )));
            }
            Ok(())
        })();
        self.decoder = PacketDecoder::responses();
        if result.is_err() {
            let _ = self.port.clear(ClearBuffer::Input);
        }
        result
    }
    fn send(&mut self, code: u8, params: &[u8]) -> Result<(), Error> {
        self.port.write_all(&request_packet(code, params)?)?;
        Ok(())
    }
    fn clear_input(&mut self) -> Result<(), Error> {
        self.port.clear(ClearBuffer::Input)?;
        self.decoder = PacketDecoder::responses();
        Ok(())
    }
    fn receive(&mut self) -> Result<Packet, Error> {
        let mut last_protocol_error = None;
        loop {
            let mut byte = [0];
            if let Err(io_error) = self.port.read_exact(&mut byte) {
                return match last_protocol_error {
                    Some(protocol_error) => Err(Error::Protocol(format!(
                        "{protocol_error}；随后读取失败：{io_error}"
                    ))),
                    None => Err(Error::Io(io_error)),
                };
            }
            if let Some(result) = self.decoder.push(byte[0]) {
                match result {
                    Ok(packet) => return Ok(packet),
                    Err(error) => last_protocol_error = Some(error),
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn ping_packet_matches_vendor_sdk() {
        assert_eq!(
            request_packet(CODE_PING, &[3]).unwrap(),
            [0x12, 0x4c, 1, 1, 3, 0x63]
        );
    }
    #[test]
    fn decoder_recovers_after_noise_and_bad_checksum() {
        let mut decoder = PacketDecoder::responses();
        let mut corrupt = response_packet(CODE_PING, &[3]).unwrap();
        *corrupt.last_mut().unwrap() ^= 1;
        let valid = response_packet(CODE_PING, &[4]).unwrap();
        let results = [0xaa, 0x05, 0xaa]
            .into_iter()
            .chain(corrupt)
            .chain(valid)
            .filter_map(|byte| decoder.push(byte))
            .collect::<Vec<_>>();
        assert!(results[0].is_err());
        assert_eq!(results[1].as_ref().unwrap().params, [4]);
    }
    #[test]
    fn rounding_matches_python() {
        for (value, expected) in [(0.05, 0), (0.15, 2), (0.25, 2), (-0.15, -2)] {
            assert_eq!(degrees_to_tenths(value).unwrap(), expected);
        }
    }
    #[test]
    fn invalid_monitor_position_is_not_feedback() {
        let mut params = vec![3];
        params.extend_from_slice(&[0; 9]);
        params.extend_from_slice(&INVALID_MONITOR_POSITION.to_le_bytes());
        params.extend_from_slice(&0_i16.to_le_bytes());
        assert!(Monitor::from_params(&params).is_err());
    }
    #[test]
    fn internal_parameter_write_packet_matches_vendor_layout() {
        let parameters = InternalParameters {
            id: 2,
            kp: 750,
            kd: 50,
            ki: 0,
            bias: 0,
            hold_kp: 750,
            hold_kd: 50,
            hold_bias: 0,
            full_deg: 3600,
            reserved: 0,
            pwm_limit: 2980,
            direction: 0,
            pwm_frequency: 5,
            dead_zone: 3,
            motor_direction: 0,
            version_info: 1,
        };
        let request = parameters.write_request();
        assert_eq!(&request[..5], &[0x13, 0x4d, 0xc4, 0x1a, 2]);
        assert_eq!(&request[5..7], &750_u16.to_le_bytes());
        assert_eq!(&request[13..15], &750_u16.to_le_bytes());
        assert_eq!(request[30], checksum(&request[..30]));
    }
    #[test]
    fn internal_parameter_read_rejects_a_different_response_header() {
        let mut response = [0_u8; INTERNAL_PARAMETERS_RESPONSE_SIZE];
        response[..5].copy_from_slice(&[0x05, 0x1c, 0xc4, 0x1a, 2]);
        response[30] = checksum(&response[..30]);
        assert!(InternalParameters::from_response(2, &response).is_err());
    }
    #[test]
    fn internal_parameter_read_accepts_the_protected_response_layout() {
        let mut response = [0_u8; INTERNAL_PARAMETERS_RESPONSE_SIZE];
        response[..5].copy_from_slice(&[0x05, 0x1c, 0xc5, 0x1a, 2]);
        for (offset, value) in [
            (5, 800_u16),
            (7, 50),
            (9, 0),
            (11, 1),
            (13, 800),
            (15, 50),
            (17, 2),
            (19, 3600),
            (21, 0),
            (23, 2980),
        ] {
            response[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
        }
        response[25..30].copy_from_slice(&[0, 5, 3, 0, 1]);
        response[30] = checksum(&response[..30]);
        let parameters = InternalParameters::from_response(2, &response).unwrap();
        assert_eq!(parameters.kp, 800);
        assert_eq!(parameters.hold_kp, 800);
        assert_eq!(parameters.full_deg, 3600);
        assert_eq!(parameters.pwm_limit, 2980);
        assert_eq!(parameters.dead_zone, 3);
    }
    #[test]
    fn parameter_write_releases_torque_with_vendor_packet() {
        assert_eq!(
            request_packet(CODE_STOP_CONTROL, &[2, 0x10, 0, 0]).unwrap(),
            [0x12, 0x4c, 0x18, 4, 2, 0x10, 0, 0, 0x8c]
        );
    }
}
