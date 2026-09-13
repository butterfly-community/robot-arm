//! Vendor READ_DATA table. Addresses are parameter IDs, not byte offsets.
//! Sources: servo-uart-rs485-sdk/STM32F103 README appendices 1/2;
//! vendor web-controller parameterBasicDefs (address 45 power hysteresis).
use crate::{
    DEFAULT_TIMEOUT, Error, FashionStarBus, INTERNAL_PARAMETERS_REQUEST_HEADER,
    INTERNAL_PARAMETERS_RESPONSE_DELAY, PacketDecoder, checksum,
};
use std::{
    io::{Read, Write},
    time::Instant,
};

#[derive(Clone, Copy, Debug)]
pub enum RegisterType {
    U8,
    U16,
    I16,
    U32,
}
#[derive(Clone, Copy, Debug)]
pub struct Register {
    pub address: u8,
    pub key: &'static str,
    pub label: &'static str,
    pub unit: &'static str,
    pub kind: RegisterType,
}
impl Register {
    pub fn writable(self) -> bool {
        self.address >= 33
    }
    /// Validate only wire representation / documented read-only status.
    pub fn encode(self, value: i64) -> Result<Vec<u8>, Error> {
        if !self.writable() {
            return Err(Error::Protocol(format!("{} 是只读字段", self.key)));
        }
        let invalid = || {
            Error::Protocol(format!(
                "{} 的值 {value} 超出 {:?} 协议类型",
                self.key, self.kind
            ))
        };
        Ok(match self.kind {
            RegisterType::U8 => vec![u8::try_from(value).map_err(|_| invalid())?],
            RegisterType::U16 => u16::try_from(value)
                .map_err(|_| invalid())?
                .to_le_bytes()
                .to_vec(),
            RegisterType::I16 => i16::try_from(value)
                .map_err(|_| invalid())?
                .to_le_bytes()
                .to_vec(),
            RegisterType::U32 => u32::try_from(value)
                .map_err(|_| invalid())?
                .to_le_bytes()
                .to_vec(),
        })
    }
    pub fn decode(self, bytes: &[u8]) -> Result<f64, Error> {
        let invalid = || {
            Error::Protocol(format!(
                "{} (地址 {}) 响应长度错误：{}",
                self.key,
                self.address,
                bytes.len()
            ))
        };
        Ok(match self.kind {
            RegisterType::U8 => f64::from(
                *bytes
                    .first()
                    .filter(|_| bytes.len() == 1)
                    .ok_or_else(invalid)?,
            ),
            RegisterType::U16 => {
                f64::from(u16::from_le_bytes(bytes.try_into().map_err(|_| invalid())?))
            }
            RegisterType::I16 => {
                f64::from(i16::from_le_bytes(bytes.try_into().map_err(|_| invalid())?))
            }
            RegisterType::U32 => {
                f64::from(u32::from_le_bytes(bytes.try_into().map_err(|_| invalid())?))
            }
        })
    }
}
macro_rules! registers {
    ($(($address:literal, $key:literal, $label:literal, $unit:literal, $kind:ident)),* $(,)?) => {
        pub const REGISTERS: &[Register] = &[$(Register {address:$address,key:$key,label:$label,unit:$unit,kind:RegisterType::$kind}),*];
    }
}
registers![
    (1, "voltage", "电压", "mV", U16),
    (2, "current", "电流", "mA", U16),
    (3, "power", "功率", "mW", U16),
    (4, "temperature", "温度原始值", "ADC", U16),
    (5, "servo_status", "状态位", "", U8),
    (6, "servo_type", "型号编码", "", U16),
    (7, "firmware_version", "固件版本编码", "", U16),
    (8, "serial_number", "序列号", "", U32),
    (33, "response_switch", "控制响应模式", "", U8),
    (34, "servo_id", "总线 ID", "", U8),
    (36, "baudrate", "波特率选项", "", U8),
    (37, "stall_protect_mode", "堵转保护模式", "", U8),
    (38, "stall_power_limit", "堵转功率上限", "mW", U16),
    (39, "over_volt_low", "电压下限", "mV", U16),
    (40, "over_volt_high", "电压上限", "mV", U16),
    (41, "over_temperature", "温度上限原始值", "ADC", U16),
    (42, "over_power", "功率上限", "mW", U16),
    (43, "over_current", "电流上限", "mA", U16),
    (44, "accel_switch", "加速度处理开关", "", U8),
    (45, "power_hysteresis", "功率保护迟滞", "%", U8),
    (46, "po_lock_switch", "上电锁力开关", "", U8),
    (48, "angle_limit_switch", "角度限制开关", "", U8),
    (49, "soft_start_switch", "上电首次缓慢执行", "", U8),
    (50, "soft_start_time", "首次执行时间", "ms", U16),
    (51, "angle_limit_high", "角度上限", "0.1°", I16),
    (52, "angle_limit_low", "角度下限", "0.1°", I16),
    (53, "angle_mid_offset", "中位角度偏移", "0.1°", I16),
];

#[derive(Clone, Copy, Debug)]
pub enum DataRequest {
    Register { id: u8, address: u8 },
    Internal { id: u8 },
}
pub(crate) struct PendingData {
    request: DataRequest,
    started: Instant,
    retried: bool,
}
impl DataRequest {
    fn id(self) -> u8 {
        match self {
            Self::Register { id, .. } | Self::Internal { id } => id,
        }
    }
}
impl FashionStarBus {
    pub fn data_read_pending(&self) -> bool {
        self.data_read.is_some()
    }
    fn send_data_request(&mut self, request: DataRequest) -> Result<(), Error> {
        self.clear_input()?;
        match request {
            DataRequest::Register { id, address } => self.send(3, &[id, address]),
            DataRequest::Internal { id } => {
                let mut bytes = INTERNAL_PARAMETERS_REQUEST_HEADER.to_vec();
                bytes.push(id);
                bytes.push(checksum(&bytes));
                self.port.write_all(&bytes)?;
                Ok(())
            }
        }
    }
    pub fn begin_data_read(&mut self, request: DataRequest) -> Result<(), Error> {
        if self.monitor_read_pending() || self.data_read_pending() || self.command_pending() {
            return Err(Error::Protocol("串口已有读取事务".into()));
        }
        self.send_data_request(request)?;
        self.data_read = Some(PendingData {
            request,
            started: Instant::now(),
            retried: false,
        });
        Ok(())
    }
    /// Poll available bytes only; trajectory writes keep running while a register
    /// reply is pending. A missing register fails independently after one retry.
    pub fn poll_data_read(&mut self) -> Result<Option<Vec<u8>>, Error> {
        let Some(mut pending) = self.data_read.take() else {
            return Ok(None);
        };
        let mut bytes = [0u8; 256];
        let available = (self.port.bytes_to_read()? as usize).min(bytes.len());
        if available > 0 {
            let n = self.port.read(&mut bytes[..available])?;
            for byte in &bytes[..n] {
                if let Some(Ok(packet)) = self.decoder.push(*byte) {
                    let matches = match pending.request {
                        DataRequest::Register { id, address } => {
                            packet.code == 3 && packet.params.starts_with(&[id, address])
                        }
                        DataRequest::Internal { id } => {
                            packet.code == 0xc5 && packet.params.first() == Some(&id)
                        }
                    };
                    if matches {
                        return Ok(Some(match pending.request {
                            DataRequest::Register { .. } => packet.params[2..].to_vec(),
                            DataRequest::Internal { .. } => {
                                crate::response_packet(packet.code, &packet.params)?
                            }
                        }));
                    }
                }
            }
        }
        let timeout = DEFAULT_TIMEOUT
            + match pending.request {
                DataRequest::Internal { .. } => INTERNAL_PARAMETERS_RESPONSE_DELAY,
                _ => std::time::Duration::ZERO,
            };
        if pending.started.elapsed() >= timeout {
            if pending.retried {
                self.decoder = PacketDecoder::responses();
                return Err(Error::Protocol(format!(
                    "ID {} {:?} 读取超时（已重试一次）",
                    pending.request.id(),
                    pending.request
                )));
            }
            self.send_data_request(pending.request)?;
            pending.retried = true;
            pending.started = Instant::now();
        }
        self.data_read = Some(pending);
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn signed_limits_and_unsigned_identity_keep_full_precision() {
        assert_eq!(
            REGISTERS
                .iter()
                .find(|r| r.address == 52)
                .unwrap()
                .decode(&(-650i16).to_le_bytes())
                .unwrap(),
            -650.0
        );
        assert_eq!(
            REGISTERS
                .iter()
                .find(|r| r.address == 8)
                .unwrap()
                .decode(&u32::MAX.to_le_bytes())
                .unwrap(),
            f64::from(u32::MAX)
        );
        assert_eq!(
            REGISTERS
                .iter()
                .find(|r| r.address == 7)
                .unwrap()
                .decode(&[0x30, 0x03])
                .unwrap(),
            816.0
        );
        assert!(REGISTERS[0].decode(&[1]).is_err());
    }
    #[test]
    fn documented_addresses_are_unique_and_no_reserved_memory_is_scanned() {
        let ids: std::collections::BTreeSet<_> = REGISTERS.iter().map(|r| r.address).collect();
        assert_eq!(ids.len(), REGISTERS.len());
        for address in [0, 9, 32, 35, 54, 255] {
            assert!(!ids.contains(&address));
        }
    }
}
