//! Device-specific output stays inside the SDL adapter. The input contract
//! remains an action and a 0–100 strength, independent of controller model.
use robot_arm_messages::InputFeedbackCapabilityInfo;
use sdl3::gamepad::{Gamepad, GamepadType};

// Streaming feedback is held until replaced or explicitly cleared. A 100 ms
// pulse expires between hardware samples (100 ms plus serial/scheduler time).
const STREAM_DURATION_MS: u32 = u32::MAX;

#[derive(Default)]
pub(super) struct FeedbackOutput {
    pub rumble: bool,
    pub trigger_rumble: bool,
    pub resistance: bool,
    triggers: [u16; 2],
}

impl FeedbackOutput {
    #[cfg(test)]
    pub fn for_test(rumble: bool, trigger_rumble: bool, resistance: bool) -> Self {
        Self {
            rumble,
            trigger_rumble,
            resistance,
            ..Self::default()
        }
    }
    pub fn discover(gamepad: &mut Gamepad) -> Self {
        let resistance = gamepad.real_type() == GamepadType::PS5
            && gamepad
                .send_effect(&resistance_packet(Some(0), Some(0)))
                .is_ok();
        Self {
            rumble: unsafe { gamepad.has_rumble() },
            trigger_rumble: unsafe { gamepad.has_rumble_triggers() },
            resistance,
            ..Self::default()
        }
    }

    pub fn capabilities(&self) -> Vec<InputFeedbackCapabilityInfo> {
        let mut result = Vec::new();
        for (supported, path, label) in [
            (self.trigger_rumble, "feedback/trigger_left", "左扳机振动"),
            (self.trigger_rumble, "feedback/trigger_right", "右扳机振动"),
            (
                self.resistance,
                "feedback/resistance_left",
                "左扳机自适应阻力",
            ),
            (
                self.resistance,
                "feedback/resistance_right",
                "右扳机自适应阻力",
            ),
            (self.rumble, "feedback/rumble", "手柄振动"),
        ] {
            if supported {
                result.push(InputFeedbackCapabilityInfo {
                    path: path.into(),
                    localized_name: Some(label.into()),
                });
            }
        }
        result
    }

    pub fn apply(
        &mut self,
        gamepad: &mut Gamepad,
        path: &str,
        intensity: u16,
    ) -> Result<(), String> {
        match path {
            "feedback/rumble" if self.rumble => gamepad
                .set_rumble(intensity, intensity, STREAM_DURATION_MS)
                .map_err(|e| e.to_string()),
            "feedback/trigger_left" | "feedback/trigger_right" if self.trigger_rumble => {
                let index = usize::from(path == "feedback/trigger_right");
                self.triggers[index] = intensity;
                gamepad
                    .set_rumble_triggers(self.triggers[0], self.triggers[1], STREAM_DURATION_MS)
                    .map_err(|e| e.to_string())
            }
            "feedback/resistance_left" if self.resistance => gamepad
                .send_effect(&resistance_packet(Some(intensity), None))
                .map_err(|e| e.to_string()),
            "feedback/resistance_right" if self.resistance => gamepad
                .send_effect(&resistance_packet(None, Some(intensity)))
                .map_err(|e| e.to_string()),
            _ => Err(format!("设备不支持反馈能力 {path}")),
        }
    }

    pub fn clear(&mut self, gamepad: &mut Gamepad) {
        if self.rumble {
            let _ = gamepad.set_rumble(0, 0, 0);
        }
        if self.trigger_rumble {
            let _ = gamepad.set_rumble_triggers(0, 0, 0);
        }
        if self.resistance {
            let _ = gamepad.send_effect(&resistance_packet(Some(0), Some(0)));
        }
        self.triggers = [0; 2];
    }
}

// SDL's DS5EffectsState_t (47 bytes), as used by testcontroller.c's
// CyclePS5TriggerEffect. SDL handles USB/Bluetooth framing and checksums.
// https://github.com/libsdl-org/SDL/blob/release-3.4.14/src/joystick/hidapi/SDL_hidapi_ps5.c
// https://github.com/libsdl-org/SDL/blob/main/test/testcontroller.c
// 0x01 = continuous resistance, start position 0; 0x05 = remove effect.
// Only the selected trigger's enable bit is set: do not reset other outputs.
fn resistance_packet(left: Option<u16>, right: Option<u16>) -> [u8; 47] {
    let mut data = [0; 47];
    for (value, flag, offset) in [(right, 0x04, 10), (left, 0x08, 21)] {
        if let Some(value) = value {
            data[0] |= flag;
            data[offset] = if value == 0 { 0x05 } else { 0x01 };
            data[offset + 2] = ((u32::from(value) * 255 + 32767) / 65535) as u8;
        }
    }
    data
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn all_feedback_types_are_independent() {
        let output = FeedbackOutput {
            rumble: true,
            trigger_rumble: true,
            resistance: true,
            ..Default::default()
        };
        assert_eq!(output.capabilities().len(), 5);
        let ps4 = FeedbackOutput {
            rumble: true,
            ..Default::default()
        };
        assert_eq!(ps4.capabilities()[0].path, "feedback/rumble");
        assert_eq!(ps4.capabilities().len(), 1);
        assert!(FeedbackOutput::default().capabilities().is_empty());
    }
    #[test]
    fn ps5_resistance_preserves_other_trigger_and_outputs() {
        let right = resistance_packet(None, Some(u16::MAX));
        assert_eq!(right[0], 0x04);
        assert_eq!(&right[10..13], &[0x01, 0, 255]);
        assert!(right[21..].iter().all(|&b| b == 0));
        assert!(right[1..10].iter().all(|&b| b == 0));
        let left = resistance_packet(Some(32768), None);
        assert_eq!(left[0], 0x08);
        assert_eq!(&left[21..24], &[0x01, 0, 128]);
        assert!(left[10..21].iter().all(|&b| b == 0));
    }
    #[test]
    fn zero_releases_resistance_and_mapping_is_monotonic() {
        let stop = resistance_packet(Some(0), Some(0));
        assert_eq!(stop[0], 0x0c);
        assert_eq!(stop[10], 0x05);
        assert_eq!(stop[21], 0x05);
        let mut previous = 0;
        for strength in 1..=u16::MAX {
            let next = resistance_packet(None, Some(strength))[12];
            assert!(next >= previous);
            previous = next;
        }
    }
}
