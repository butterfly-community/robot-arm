//! NOLO CV1 production-firmware HID report decoding.

use anyhow::{Result, bail};

pub const SUPPORTED_DEVICES: [(u16, u16); 2] = [
    (0x0483, 0x5750), // Early/Kickstarter USB identity; used by the local CV1.
    (0x28e9, 0x028a), // Production USB identity.
];
pub const REPORT_SIZE: usize = 64;
pub const CONTROLLER_0_REPORT: u8 = 16;
pub const CONTROLLER_1_REPORT: u8 = 17;

const KEY: [u32; 4] = [0x875b_cc51, 0xa763_7a66, 0x5096_0967, 0xf853_6c51];
const DELTA: u32 = 0x9e37_79b9;
const POSITION_SCALE: f32 = 0.0001;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RawFrame {
    pub controller_id: u8,
    pub position: [f32; 3],
    pub accelerometer: [i16; 3],
    pub gyroscope: [i16; 3],
    pub buttons: u8,
    pub touchpad: Option<[u8; 2]>,
    /// Decrypted byte 22; semantic is not yet identified.
    pub unknown_22: u8,
    /// Decrypted byte 23; observed as a status/parity bit field.
    pub unknown_23: u8,
    /// Controller sample sequence at decrypted zero-based byte 24.
    pub controller_sequence: u8,
    pub hmd_position: [f32; 3],
    pub unknown_31_36: [u8; 6],
    pub hmd_gyroscope: [i16; 3],
    pub unknown_43_48: [u8; 6],
    pub hmd_accelerometer: [i16; 3],
    pub unknown_55_58: [u8; 4],
    /// HMD/relay sample sequence at decrypted zero-based byte 59.
    pub hmd_sequence: u8,
    pub unknown_60: u8,
    pub unknown_61_63: [u8; 3],
}

impl RawFrame {
    pub fn touchpad_pressed(self) -> bool {
        self.buttons & (1 << 0) != 0
    }

    pub fn trigger_pressed(self) -> bool {
        self.buttons & (1 << 1) != 0
    }

    pub fn menu_pressed(self) -> bool {
        self.buttons & (1 << 2) != 0
    }

    pub fn home_pressed(self) -> bool {
        self.buttons & (1 << 3) != 0
    }

    pub fn squeeze_pressed(self) -> bool {
        self.buttons & (1 << 4) != 0
    }

    pub fn touchpad_touched(self) -> bool {
        self.buttons & (1 << 5) != 0
    }
}

pub fn decode_report(report: &[u8]) -> Result<Option<RawFrame>> {
    if report.len() != REPORT_SIZE {
        bail!(
            "expected a {REPORT_SIZE}-byte HID report, got {} bytes",
            report.len()
        );
    }
    if !matches!(report[0], CONTROLLER_0_REPORT | CONTROLLER_1_REPORT) {
        return Ok(None);
    }

    let mut decrypted = [0_u8; REPORT_SIZE];
    decrypted.copy_from_slice(report);
    decrypt(&mut decrypted);
    Ok(Some(parse_decrypted(&decrypted)))
}

fn parse_decrypted(report: &[u8; REPORT_SIZE]) -> RawFrame {
    let pad = [report[20], report[21]];
    RawFrame {
        controller_id: report[0] - CONTROLLER_0_REPORT,
        position: position(report, 1),
        accelerometer: vector_i16(report, 7),
        gyroscope: vector_i16(report, 13),
        buttons: report[19],
        touchpad: (pad != [255, 255]).then_some(pad),
        unknown_22: report[22],
        unknown_23: report[23],
        controller_sequence: report[24],
        hmd_position: position(report, 25),
        unknown_31_36: report[31..37].try_into().unwrap(),
        hmd_gyroscope: vector_i16(report, 37),
        unknown_43_48: report[43..49].try_into().unwrap(),
        hmd_accelerometer: vector_i16(report, 49),
        unknown_55_58: report[55..59].try_into().unwrap(),
        hmd_sequence: report[59],
        unknown_60: report[60],
        unknown_61_63: report[61..64].try_into().unwrap(),
    }
}

fn position(report: &[u8], offset: usize) -> [f32; 3] {
    vector_i16(report, offset).map(|value| f32::from(value) * POSITION_SCALE)
}

fn vector_i16(report: &[u8], offset: usize) -> [i16; 3] {
    std::array::from_fn(|index| {
        let start = offset + index * 2;
        i16::from_le_bytes([report[start], report[start + 1]])
    })
}

fn decrypt(report: &mut [u8; REPORT_SIZE]) {
    let mut words = [0_u32; 15];
    for (index, word) in words.iter_mut().enumerate() {
        let start = 1 + index * 4;
        *word = u32::from_le_bytes(report[start..start + 4].try_into().unwrap());
    }
    btea_decrypt(&mut words);
    for (index, word) in words.iter().enumerate() {
        let start = 1 + index * 4;
        report[start..start + 4].copy_from_slice(&word.to_le_bytes());
    }
}

fn btea_decrypt(words: &mut [u32; 15]) {
    let rounds = 1 + 52 / words.len();
    let mut sum = (rounds as u32).wrapping_mul(DELTA);

    for _ in 0..rounds {
        let e = ((sum >> 2) & 3) as usize;
        let mut y = words[0];
        for index in (0..words.len()).rev() {
            let z = if index == 0 {
                words[words.len() - 1]
            } else {
                words[index - 1]
            };
            let mix = ((z >> 5) ^ y.wrapping_shl(2)).wrapping_add((y >> 3) ^ z.wrapping_shl(4))
                ^ ((sum ^ y).wrapping_add(KEY[(index & 3) ^ e] ^ z));
            words[index] = words[index].wrapping_sub(mix);
            y = words[index];
        }
        sum = sum.wrapping_sub(DELTA);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn btea_encrypt(words: &mut [u32; 15]) {
        let rounds = 1 + 52 / words.len();
        let mut sum = 0_u32;
        let mut z = words[words.len() - 1];
        for _ in 0..rounds {
            sum = sum.wrapping_add(DELTA);
            let e = ((sum >> 2) & 3) as usize;
            for index in 0..words.len() {
                let y = words[(index + 1) % words.len()];
                let mix = ((z >> 5) ^ y.wrapping_shl(2)).wrapping_add((y >> 3) ^ z.wrapping_shl(4))
                    ^ ((sum ^ y).wrapping_add(KEY[(index & 3) ^ e] ^ z));
                words[index] = words[index].wrapping_add(mix);
                z = words[index];
            }
        }
    }

    fn encrypted_sample() -> [u8; REPORT_SIZE] {
        let mut report = [0_u8; REPORT_SIZE];
        report[0] = CONTROLLER_0_REPORT;
        for (offset, value) in [1000_i16, -2000, 3000].into_iter().enumerate() {
            report[1 + offset * 2..3 + offset * 2].copy_from_slice(&value.to_le_bytes());
        }
        for (offset, value) in [1024_i16, -512, 256].into_iter().enumerate() {
            report[7 + offset * 2..9 + offset * 2].copy_from_slice(&value.to_le_bytes());
        }
        for (offset, value) in [10_i16, -20, 30].into_iter().enumerate() {
            report[13 + offset * 2..15 + offset * 2].copy_from_slice(&value.to_le_bytes());
        }
        report[19] = 0b0000_0100;
        report[20..22].copy_from_slice(&[255, 255]);
        report[24] = 0x91;
        report[22] = 0x5b;
        report[23] = 0x07;
        report[31..37].copy_from_slice(&[1, 2, 3, 4, 5, 6]);
        for (offset, value) in [101_i16, -202, 303].into_iter().enumerate() {
            report[37 + offset * 2..39 + offset * 2].copy_from_slice(&value.to_le_bytes());
        }
        report[43..49].copy_from_slice(&[7, 8, 9, 10, 11, 12]);
        for (offset, value) in [16000_i16, -8000, 4000].into_iter().enumerate() {
            report[49 + offset * 2..51 + offset * 2].copy_from_slice(&value.to_le_bytes());
        }
        report[55..59].copy_from_slice(&[13, 14, 15, 16]);
        report[59] = 0x6d;
        report[60] = 0x11;
        report[61..64].copy_from_slice(&[0x12, 0x13, 0x14]);

        let mut words = [0_u32; 15];
        for (index, word) in words.iter_mut().enumerate() {
            let start = 1 + index * 4;
            *word = u32::from_le_bytes(report[start..start + 4].try_into().unwrap());
        }
        btea_encrypt(&mut words);
        for (index, word) in words.iter().enumerate() {
            let start = 1 + index * 4;
            report[start..start + 4].copy_from_slice(&word.to_le_bytes());
        }
        report
    }

    #[test]
    fn decrypts_and_parses_production_report() {
        let frame = decode_report(&encrypted_sample()).unwrap().unwrap();
        assert_eq!(frame.controller_id, 0);
        assert_eq!(frame.position, [0.099999994, -0.19999999, 0.29999998]);
        assert_eq!(frame.accelerometer, [1024, -512, 256]);
        assert_eq!(frame.gyroscope, [10, -20, 30]);
        assert!(frame.menu_pressed());
        assert!(!frame.trigger_pressed());
        assert_eq!(frame.touchpad, None);
        assert_eq!(frame.unknown_22, 0x5b);
        assert_eq!(frame.unknown_23, 0x07);
        assert_eq!(frame.controller_sequence, 0x91);
        assert_eq!(frame.unknown_31_36, [1, 2, 3, 4, 5, 6]);
        assert_eq!(frame.hmd_gyroscope, [101, -202, 303]);
        assert_eq!(frame.unknown_43_48, [7, 8, 9, 10, 11, 12]);
        assert_eq!(frame.hmd_accelerometer, [16000, -8000, 4000]);
        assert_eq!(frame.unknown_55_58, [13, 14, 15, 16]);
        assert_eq!(frame.hmd_sequence, 0x6d);
        assert_eq!(frame.unknown_60, 0x11);
        assert_eq!(frame.unknown_61_63, [0x12, 0x13, 0x14]);
    }

    #[test]
    fn ignores_non_controller_report_types() {
        let mut report = [0_u8; REPORT_SIZE];
        report[0] = 99;
        assert_eq!(decode_report(&report).unwrap(), None);
    }

    #[test]
    fn identifies_controller_one_report() {
        let mut report = encrypted_sample();
        report[0] = CONTROLLER_1_REPORT;
        assert_eq!(decode_report(&report).unwrap().unwrap().controller_id, 1);
    }

    #[test]
    fn rejects_truncated_reports() {
        assert!(decode_report(&[0_u8; 59]).is_err());
    }
}
