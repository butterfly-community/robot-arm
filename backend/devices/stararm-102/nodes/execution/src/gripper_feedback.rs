//! Continuous regulation of the existing measured-load scale.
//! Position commands express close/release intent; measured angle is never
//! latched as a holding target. Native power is adjusted from new feedback,
//! not a fixed conversion of percent to power.
use super::{GRIPPER_COMMAND_POWER_MW, GRIPPER_IDLE_POWER_MW};

#[derive(Default)]
pub struct GripperFeedbackController {
    last_requested_tenths_degree: Option<i32>,
    pub regulated_power_mw: Option<u16>,
}

impl GripperFeedbackController {
    /// Intent uses the exact native UART encoding, not floating-point noise.
    pub fn request(&mut self, angle: i32) {
        if let Some(last) = self.last_requested_tenths_degree {
            if angle > last {
                self.regulated_power_mw = None;
            } else if angle < last {
                self.regulated_power_mw
                    .get_or_insert(GRIPPER_COMMAND_POWER_MW);
            }
        }
        self.last_requested_tenths_degree = Some(angle);
    }

    /// Once per actual monitor response, including while transporting.
    /// Integrate load error in the same mW units as the existing feedback scale.
    /// No angle setpoint, first-contact latch, fixed grasp width, or new limit.
    pub fn observe(&mut self, strength: f64, target: f64) -> bool {
        let Some(previous) = self.regulated_power_mw else {
            return false;
        };
        let correction = (target - strength)
            * f64::from(GRIPPER_COMMAND_POWER_MW - GRIPPER_IDLE_POWER_MW)
            / 100.0;
        // Vendor power=0 means maximum power, NOT zero force. Keep that special
        // protocol value out of the regulator. The upper bound is unchanged.
        let next = (f64::from(previous) + correction)
            .round_ties_even()
            .clamp(1.0, f64::from(GRIPPER_COMMAND_POWER_MW)) as u16;
        self.regulated_power_mw = Some(next);
        next != previous
    }

    pub fn command_power_mw(&self) -> u16 {
        self.regulated_power_mw.unwrap_or(GRIPPER_COMMAND_POWER_MW)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracks_load_continuously_without_latching_an_angle() {
        let mut c = GripperFeedbackController::default();
        c.request(600);
        c.request(180);
        assert!(c.observe(29.1875, 20.0));
        assert_eq!(c.command_power_mw(), 1853);
        c.request(0); // Continuing close must not reset the regulator.
        assert!(c.observe(40.0, 20.0));
        assert_eq!(c.command_power_mw(), 1533);
        c.request(0); // Transport keeps the same close intent.
        assert!(c.observe(10.0, 20.0));
        assert_eq!(c.command_power_mw(), 1693);
        assert!(!c.observe(20.0, 20.0));
        assert_eq!(c.command_power_mw(), 1693);
        assert!(c.observe(0.0, 20.0)); // Lost preload calls for more effort.
        assert_eq!(c.command_power_mw(), GRIPPER_COMMAND_POWER_MW);
        c.request(10); // Explicit release only.
        assert_eq!(c.regulated_power_mw, None);
        assert!(!c.observe(70.0, 20.0));
    }

    #[test]
    fn never_encodes_zero_as_maximum_power_when_reducing_load() {
        let mut c = GripperFeedbackController::default();
        c.request(10);
        c.request(0);
        for _ in 0..10 {
            c.observe(100.0, 0.0);
        }
        assert_eq!(c.command_power_mw(), 1);
        c.observe(0.0, 100.0);
        assert_eq!(c.command_power_mw(), 1601);
        c.observe(0.0, 100.0);
        assert_eq!(c.command_power_mw(), GRIPPER_COMMAND_POWER_MW);
    }

    #[test]
    fn same_target_does_not_imply_a_fixed_power_command() {
        let mut c = GripperFeedbackController::default();
        c.request(10);
        c.request(0);
        c.observe(40.0, 20.0);
        let first = c.command_power_mw();
        c.observe(30.0, 20.0);
        assert!(c.command_power_mw() < first);
        c.observe(10.0, 20.0);
        assert_eq!(c.command_power_mw(), first);
    }
}
