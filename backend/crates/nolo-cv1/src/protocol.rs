//! NOLO CV1 production-firmware HID report decoding.

use eyre::{Result, bail};

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

#[cfg(test)]
/// Encode one decoded frame into the encrypted 64-byte report emitted by a
/// NOLO CV1 HID device.  This is used by the deterministic virtual report
/// source; production USB input continues to enter through [`decode_report`].
pub fn encode_report(frame: RawFrame) -> Result<[u8; REPORT_SIZE]> {
    if frame.controller_id > 1 {
        bail!("controller_id must be 0 or 1");
    }
    let mut report = [0_u8; REPORT_SIZE];
    report[0] = CONTROLLER_0_REPORT + frame.controller_id;
    write_position(&mut report, 1, frame.position)?;
    write_vector_i16(&mut report, 7, frame.accelerometer);
    write_vector_i16(&mut report, 13, frame.gyroscope);
    report[19] = frame.buttons;
    report[20..22].copy_from_slice(&frame.touchpad.unwrap_or([255, 255]));
    report[22] = frame.unknown_22;
    report[23] = frame.unknown_23;
    report[24] = frame.controller_sequence;
    write_position(&mut report, 25, frame.hmd_position)?;
    report[31..37].copy_from_slice(&frame.unknown_31_36);
    write_vector_i16(&mut report, 37, frame.hmd_gyroscope);
    report[43..49].copy_from_slice(&frame.unknown_43_48);
    write_vector_i16(&mut report, 49, frame.hmd_accelerometer);
    report[55..59].copy_from_slice(&frame.unknown_55_58);
    report[59] = frame.hmd_sequence;
    report[60] = frame.unknown_60;
    report[61..64].copy_from_slice(&frame.unknown_61_63);
    encrypt(&mut report);
    Ok(report)
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

#[cfg(test)]
fn write_position(report: &mut [u8], offset: usize, position: [f32; 3]) -> Result<()> {
    let mut quantized = [0_i16; 3];
    for (index, value) in position.into_iter().enumerate() {
        if !value.is_finite() {
            bail!("position axis {index} is not finite");
        }
        let counts = (value / POSITION_SCALE).round();
        if counts < f32::from(i16::MIN) || counts > f32::from(i16::MAX) {
            bail!("position axis {index} is outside the NOLO i16 range: {value} m");
        }
        quantized[index] = counts as i16;
    }
    write_vector_i16(report, offset, quantized);
    Ok(())
}

#[cfg(test)]
fn write_vector_i16(report: &mut [u8], offset: usize, vector: [i16; 3]) {
    for (index, value) in vector.into_iter().enumerate() {
        let start = offset + index * 2;
        report[start..start + 2].copy_from_slice(&value.to_le_bytes());
    }
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

#[cfg(test)]
fn encrypt(report: &mut [u8; REPORT_SIZE]) {
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

#[cfg(test)]
mod tests {
    use super::*;

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
        assert_ne!(frame.buttons & (1 << 2), 0);
        assert_eq!(frame.buttons & (1 << 1), 0);
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

    #[test]
    fn virtual_report_round_trips_through_production_decoder() {
        let expected = RawFrame {
            controller_id: 1,
            position: [0.1234, 1.0, -0.5678],
            accelerometer: [100, -200, -1024],
            gyroscope: [-10, 20, 30],
            buttons: 0b0001_0010,
            touchpad: Some([37, 201]),
            unknown_22: 35,
            unknown_23: 7,
            controller_sequence: 255,
            hmd_position: [-0.25, 1.5, -1.25],
            unknown_31_36: [1, 2, 3, 4, 5, 6],
            hmd_gyroscope: [4, 5, 6],
            unknown_43_48: [7, 8, 9, 10, 11, 12],
            hmd_accelerometer: [0, 0, -16384],
            unknown_55_58: [13, 14, 15, 16],
            hmd_sequence: 254,
            unknown_60: 17,
            unknown_61_63: [18, 19, 20],
        };
        let encoded = encode_report(expected).unwrap();
        let actual = decode_report(&encoded).unwrap().unwrap();
        for (actual, expected) in actual.position.into_iter().zip(expected.position) {
            assert!((actual - expected).abs() <= POSITION_SCALE);
        }
        for (actual, expected) in actual.hmd_position.into_iter().zip(expected.hmd_position) {
            assert!((actual - expected).abs() <= POSITION_SCALE);
        }
        let mut normalized_expected = expected;
        normalized_expected.position = actual.position;
        normalized_expected.hmd_position = actual.hmd_position;
        assert_eq!(actual, normalized_expected);
    }

    #[test]
    fn virtual_report_rejects_unrepresentable_positions() {
        let mut frame = RawFrame {
            controller_id: 0,
            position: [0.0; 3],
            accelerometer: [0; 3],
            gyroscope: [0; 3],
            buttons: 0,
            touchpad: None,
            unknown_22: 0,
            unknown_23: 0,
            controller_sequence: 0,
            hmd_position: [0.0; 3],
            unknown_31_36: [0; 6],
            hmd_gyroscope: [0; 3],
            unknown_43_48: [0; 6],
            hmd_accelerometer: [0; 3],
            unknown_55_58: [0; 4],
            hmd_sequence: 0,
            unknown_60: 0,
            unknown_61_63: [0; 3],
        };
        frame.position[0] = 4.0;
        assert!(encode_report(frame).is_err());
    }
}
