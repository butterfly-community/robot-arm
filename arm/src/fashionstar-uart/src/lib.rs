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

const REQUEST_HEADER: [u8; 2] = [0x12, 0x4c];
const RESPONSE_HEADER: [u8; 2] = [0x05, 0x1c];
const FRAME_OVERHEAD: usize = 5;
const INVALID_MONITOR_POSITION: i32 = -235_929_599;
const INTERNAL_PARAMETERS_REQUEST_HEADER: [u8; 4] = [0x13, 0x4d, 0xc5, 0x01];
const INTERNAL_PARAMETERS_RESPONSE_SIZE: usize = 31;
const INTERNAL_PARAMETERS_RESPONSE_DELAY: Duration = Duration::from_millis(200);

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
    fn from(error: serialport::Error) -> Self {
        Self::Serial(error)
    }
}

impl From<io::Error> for Error {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
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
            buffer: Vec::with_capacity(usize::from(u8::MAX) + FRAME_OVERHEAD),
        }
    }

    pub fn push(&mut self, byte: u8) -> Option<Result<Packet, Error>> {
        self.buffer.push(byte);
        self.discard_before_header();
        if self.buffer.len() < 4 {
            return None;
        }
        let frame_len = usize::from(self.buffer[3]) + FRAME_OVERHEAD;
        if self.buffer.len() < frame_len {
            return None;
        }
        let frame = self.buffer.drain(..frame_len).collect::<Vec<_>>();
        if checksum(&frame[..frame_len - 1]) != frame[frame_len - 1] {
            self.discard_before_header();
            return Some(Err(Error::Protocol("数据帧校验和错误".to_owned())));
        }
        Some(Ok(Packet {
            code: frame[2],
            params: frame[4..frame_len - 1].to_vec(),
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
        let position_tenths_degree = i32::from_le_bytes(params[10..14].try_into().unwrap());
        let turns = i16::from_le_bytes(params[14..16].try_into().unwrap());
        if position_tenths_degree == INVALID_MONITOR_POSITION && turns == 0 {
            return Err(Error::Protocol(format!(
                "Monitor ID {} 返回了厂家无效位置标记",
                params[0]
            )));
        }
        Ok(Self {
            id: params[0],
            voltage_mv: u16::from_le_bytes(params[1..3].try_into().unwrap()),
            current_ma: u16::from_le_bytes(params[3..5].try_into().unwrap()),
            power_mw: u16::from_le_bytes(params[5..7].try_into().unwrap()),
            temperature_raw: u16::from_le_bytes(params[7..9].try_into().unwrap()),
            status: params[9],
            position_tenths_degree,
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
    pub direction: u8,
    pub dead_zone: u8,
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
            direction: response[25],
            dead_zone: response[27],
        })
    }
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

pub fn degrees_to_tenths(position_degrees: f64) -> Result<i32, Error> {
    let rounded = (position_degrees * 10.0).round_ties_even();
    if !rounded.is_finite() || rounded < f64::from(i32::MIN) || rounded > f64::from(i32::MAX) {
        return Err(Error::Protocol(format!(
            "位置 {position_degrees}° 无法编码为协议 i32 角度"
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
        let count = u8::try_from(ids.len())
            .map_err(|_| Error::Protocol("Monitor 舵机数量超过一个字节".to_owned()))?;
        let mut params = Vec::with_capacity(ids.len() + 3);
        params.extend_from_slice(&[CODE_QUERY_MONITOR, 1, count]);
        params.extend_from_slice(ids);
        self.send(CODE_SYNC_COMMAND, &params)?;
        (0..ids.len())
            .map(|_| {
                let response = self.receive()?;
                if response.code != CODE_QUERY_MONITOR {
                    return Err(Error::Protocol(format!(
                        "Monitor 返回了功能码 {}",
                        response.code
                    )));
                }
                Monitor::from_params(&response.params)
            })
            .collect()
    }

    pub fn write_positions(&mut self, commands: &[PositionCommand]) -> Result<(), Error> {
        let count = u8::try_from(commands.len())
            .map_err(|_| Error::Protocol("同步位置命令数量超过一个字节".to_owned()))?;
        let mut params = Vec::with_capacity(commands.len() * 15 + 3);
        params.extend_from_slice(&[CODE_SET_MTURN_BY_INTERVAL, 15, count]);
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

    fn send(&mut self, code: u8, params: &[u8]) -> Result<(), Error> {
        self.port.write_all(&request_packet(code, params)?)?;
        Ok(())
    }

    fn receive(&mut self) -> Result<Packet, Error> {
        loop {
            let mut byte = [0];
            self.port.read_exact(&mut byte)?;
            if let Some(result) = self.decoder.push(byte[0]) {
                match result {
                    Ok(packet) => return Ok(packet),
                    Err(Error::Protocol(_)) => {}
                    Err(error) => return Err(error),
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packet_matches_python_sdk_example() {
        assert_eq!(
            request_packet(CODE_PING, &[3]).unwrap(),
            [0x12, 0x4c, CODE_PING, 1, 3, 0x63]
        );
    }

    #[test]
    fn decoder_handles_noise_fragmentation_and_corrupt_frame() {
        let valid = response_packet(CODE_PING, &[4]).unwrap();
        let mut corrupt = response_packet(CODE_PING, &[3]).unwrap();
        *corrupt.last_mut().unwrap() ^= 1;
        let mut decoder = PacketDecoder::responses();
        let mut results = Vec::new();
        for byte in [0xaa, 0x05, 0xaa].into_iter().chain(corrupt).chain(valid) {
            if let Some(result) = decoder.push(byte) {
                results.push(result);
            }
        }
        assert!(results[0].is_err());
        assert_eq!(
            results[1].as_ref().unwrap(),
            &Packet {
                code: CODE_PING,
                params: vec![4]
            }
        );
    }

    #[test]
    fn decoder_accepts_every_protocol_parameter_length() {
        for size in 0..=usize::from(u8::MAX) {
            let params = (0..size).map(|index| index as u8).collect::<Vec<_>>();
            let frame = response_packet(size as u8, &params).unwrap();
            let mut decoder = PacketDecoder::responses();
            let mut decoded = None;
            for byte in [0xaa, 0x05, 0xaa].into_iter().chain(frame) {
                if let Some(result) = decoder.push(byte) {
                    decoded = Some(result.unwrap());
                }
            }
            assert_eq!(
                decoded,
                Some(Packet {
                    code: size as u8,
                    params
                })
            );
        }
    }

    #[test]
    fn monitor_layout_matches_python_struct() {
        let mut params = vec![6];
        params.extend_from_slice(&7400_u16.to_le_bytes());
        params.extend_from_slice(&500_u16.to_le_bytes());
        params.extend_from_slice(&2000_u16.to_le_bytes());
        params.extend_from_slice(&2048_u16.to_le_bytes());
        params.push(4);
        params.extend_from_slice(&(-900_i32).to_le_bytes());
        params.extend_from_slice(&(-2_i16).to_le_bytes());
        let value = Monitor::from_params(&params).unwrap();
        assert_eq!(value.id, 6);
        assert_eq!(value.voltage_mv, 7400);
        assert_eq!(value.current_ma, 500);
        assert_eq!(value.power_mw, 2000);
        assert_eq!(value.temperature_raw, 2048);
        assert_eq!(value.status, 4);
        assert_eq!(value.position_degrees(), -90.0);
        assert_eq!(value.turns, -2);
    }

    #[test]
    fn degree_rounding_matches_python_round() {
        for (degrees, expected) in [
            (0.05, 0),
            (0.15, 2),
            (0.25, 2),
            (-0.05, 0),
            (-0.15, -2),
            (-0.25, -2),
        ] {
            assert_eq!(degrees_to_tenths(degrees).unwrap(), expected);
        }
    }

    #[test]
    fn internal_parameters_use_the_vendor_read_response_layout() {
        let mut response = [0_u8; INTERNAL_PARAMETERS_RESPONSE_SIZE];
        response[..5].copy_from_slice(&[0x05, 0x1c, 0xc5, 0x1a, 3]);
        for (offset, value) in [
            (5, 200_u16),
            (7, 1100),
            (9, 7),
            (11, 8),
            (13, 201),
            (15, 1101),
            (17, 9),
        ] {
            response[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
        }
        response[25] = 1;
        response[27] = 3;
        response[30] = checksum(&response[..30]);

        assert_eq!(
            InternalParameters::from_response(3, &response).unwrap(),
            InternalParameters {
                id: 3,
                kp: 200,
                kd: 1100,
                ki: 7,
                bias: 8,
                hold_kp: 201,
                hold_kd: 1101,
                hold_bias: 9,
                direction: 1,
                dead_zone: 3,
            }
        );
        assert!(InternalParameters::from_response(2, &response).is_err());
        response[30] ^= 1;
        assert!(InternalParameters::from_response(3, &response).is_err());
    }

    #[test]
    fn protocol_length_and_integer_boundaries_are_explicit() {
        assert!(request_packet(1, &[0; 255]).is_ok());
        assert!(request_packet(1, &[0; 256]).is_err());
        assert_eq!(
            degrees_to_tenths(f64::from(i32::MIN) / 10.0).unwrap(),
            i32::MIN
        );
        assert_eq!(
            degrees_to_tenths(f64::from(i32::MAX) / 10.0).unwrap(),
            i32::MAX
        );
        assert!(degrees_to_tenths(f64::NAN).is_err());
        assert!(degrees_to_tenths(f64::INFINITY).is_err());
    }

    #[test]
    fn manufacturer_invalid_monitor_marker_is_not_reported_as_a_position() {
        let mut params = vec![3];
        params.extend_from_slice(&[0; 9]);
        params.extend_from_slice(&INVALID_MONITOR_POSITION.to_le_bytes());
        params.extend_from_slice(&0_i16.to_le_bytes());
        assert!(Monitor::from_params(&params).is_err());
    }
}
