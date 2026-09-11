//! Continuous regulation of the existing measured-load scale.
//! Position commands express close/release intent; measured angle is never
//! latched as a holding target. Native power is adjusted from new feedback,
//! not a fixed conversion of percent to power.
use super::{GRIPPER_COMMAND_POWER_MW, GRIPPER_IDLE_POWER_MW, MOTION_TIME_MS};

fn target_power(target: f64) -> f64 {
    f64::from(GRIPPER_IDLE_POWER_MW)
        + target * f64::from(GRIPPER_COMMAND_POWER_MW - GRIPPER_IDLE_POWER_MW) / 100.0
}

#[derive(Default)]
pub struct GripperFeedbackController {
    last_requested_tenths_degree: Option<i32>,
    // Keep sub-mW corrections; quantize only at the UART/message boundary.
    regulated_power_mw: Option<f64>,
    last_sample_time_ns: Option<i64>,
    observed_load: bool,
}

impl GripperFeedbackController {
    /// Intent uses the exact native UART encoding, not floating-point noise.
    pub fn request(&mut self, angle: i32, target: f64) {
        if let Some(last) = self.last_requested_tenths_degree {
            if angle > last {
                self.regulated_power_mw = None;
                self.observed_load = false;
            } else if angle < last && self.regulated_power_mw.is_none() {
                self.regulated_power_mw = Some(target_power(target));
                self.observed_load = false;
            }
        }
        self.last_requested_tenths_degree = Some(angle);
    }

    /// Once per actual monitor response, including while transporting.
    /// The monitor's raw power must not be clipped at the UI's 100% display.
    /// A power ceiling cannot produce load in free air: integrating that deficit
    /// wound the old controller to maximum before contact. Suppress that free
    /// closure deficit only until measurable load first appears. Once loaded,
    /// a lost preload MUST still request more effort, not freeze a small output.
    /// Response smoothing uses the existing native command horizon, not
    /// an object-specific gain, contact threshold or holding-angle setpoint.
    pub fn observe(&mut self, power_mw: u16, target: f64, sample_time_ns: i64) -> bool {
        if self
            .last_sample_time_ns
            .is_some_and(|previous| sample_time_ns <= previous)
        {
            return false;
        }
        let elapsed_ms = self
            .last_sample_time_ns
            .map(|previous| sample_time_ns.saturating_sub(previous) as f64 / 1_000_000.0)
            .unwrap_or(f64::from(MOTION_TIME_MS));
        self.last_sample_time_ns = Some(sample_time_ns);
        let Some(previous) = self.regulated_power_mw else {
            return false;
        };
        let desired = target_power(target);
        let measured = f64::from(power_mw);
        let error = desired - measured;
        self.observed_load |= power_mw > GRIPPER_IDLE_POWER_MW;
        if error > 0.0 && !self.observed_load {
            return false; // Existing zero-load baseline; no free-motion windup.
        }
        // Integrate RELATIVE error in the current command's scale. Absolute
        // mW subtraction assumed an instantaneous unit-gain actuator: recorded
        // run 028 drove the limit to 1 mW after contact, immediately before loss
        // of preload. Raw measurements can exceed the commanded ceiling.
        // The symmetric normalization is finite even for measured=0 and lies
        // within [-1, 1]; the existing command horizon smooths each correction.
        // No object gain, angle latch, new output floor or clipped measurement.
        let alpha = elapsed_ms / (elapsed_ms + f64::from(MOTION_TIME_MS));
        let correction = previous * error / (desired + measured) * alpha;
        // Vendor power=0 means maximum power, NOT zero force. Keep that special
        // protocol value out of the regulator. The upper bound is unchanged.
        let next = (previous + correction).clamp(1.0, f64::from(GRIPPER_COMMAND_POWER_MW));
        self.regulated_power_mw = Some(next);
        next.round_ties_even() != previous.round_ties_even()
    }

    pub fn regulated_power_mw(&self) -> Option<u16> {
        self.regulated_power_mw
            .map(|power| power.round_ties_even() as u16)
    }

    pub fn command_power_mw(&self) -> u16 {
        self.regulated_power_mw()
            .unwrap_or(GRIPPER_COMMAND_POWER_MW)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracks_load_continuously_without_latching_an_angle() {
        let mut c = GripperFeedbackController::default();
        c.request(600, 30.0);
        c.request(180, 30.0);
        assert_eq!(c.command_power_mw(), 880); // Initial estimate, not a fixed output.
        assert!(c.observe(1040, 30.0, 100_000_000)); // Measured load 40.
        assert!(c.command_power_mw() < 880);
        let reduced = c.command_power_mw();
        c.request(0, 30.0); // Continuing close must not reset the regulator.
        assert!(c.observe(720, 30.0, 200_000_000)); // Load dropped to 20.
        assert!(c.command_power_mw() > reduced);
        let holding = c.command_power_mw();
        c.request(0, 30.0); // Transport keeps the same close intent.
        assert!(!c.observe(880, 30.0, 300_000_000));
        assert_eq!(c.command_power_mw(), holding);
        assert!(c.observe(800, 30.0, 400_000_000));
        assert!(c.command_power_mw() > holding);
        let recovering = c.command_power_mw();
        assert!(c.observe(360, 30.0, 500_000_000)); // Lost preload still needs correction.
        assert!(c.command_power_mw() > recovering);
        c.request(10, 30.0); // Explicit release only.
        assert_eq!(c.regulated_power_mw, None);
        assert!(!c.observe(1520, 30.0, 600_000_000));
    }

    #[test]
    fn never_encodes_zero_as_maximum_power_when_reducing_load() {
        let mut c = GripperFeedbackController::default();
        c.request(10, 0.0);
        c.request(0, 0.0);
        for sample in 1..100 {
            c.observe(u16::MAX, 0.0, sample * 100_000_000);
        }
        assert_eq!(c.command_power_mw(), 1);
        for sample in 100..110 {
            c.observe(401, 100.0, sample * 100_000_000);
        }
        assert!(c.command_power_mw() > 1);
    }

    #[test]
    fn same_target_does_not_imply_a_fixed_power_command() {
        let mut c = GripperFeedbackController::default();
        c.request(10, 20.0);
        c.request(0, 20.0);
        c.observe(1040, 20.0, 100_000_000);
        let first = c.command_power_mw();
        c.observe(880, 20.0, 200_000_000);
        assert!(c.command_power_mw() < first);
        let reduced = c.command_power_mw();
        c.observe(560, 20.0, 300_000_000);
        assert!(c.command_power_mw() > reduced);
    }

    #[test]
    fn free_closure_does_not_wind_up_to_maximum() {
        let mut c = GripperFeedbackController::default();
        c.request(600, 30.0);
        c.request(0, 30.0);
        for sample in 1..100 {
            assert!(!c.observe(362, 30.0, sample * 100_000_000));
        }
        assert_eq!(c.command_power_mw(), 880);
    }

    #[test]
    fn raw_overload_is_not_clipped_to_display_full_scale() {
        let mut c = GripperFeedbackController::default();
        c.request(600, 30.0);
        c.request(0, 30.0);
        // Actual trial 007 monitor values, not a 100%-clipped substitute.
        c.observe(2613, 30.0, 100_000_000);
        let first = c.command_power_mw();
        assert!(first > 440 && first < 880);
        c.observe(7012, 30.0, 200_000_000);
        assert!(c.command_power_mw() > first / 2 && c.command_power_mw() < first);
        let mut display_clipped = GripperFeedbackController::default();
        display_clipped.request(600, 30.0);
        display_clipped.request(0, 30.0);
        display_clipped.observe(2000, 30.0, 100_000_000);
        assert!(first < display_clipped.command_power_mw());
    }

    #[test]
    fn repeated_timestamp_does_not_integrate_twice() {
        let mut c = GripperFeedbackController::default();
        c.request(600, 30.0);
        c.request(0, 30.0);
        c.observe(1040, 30.0, 100_000_000);
        let output = c.command_power_mw();
        assert!(!c.observe(1040, 30.0, 100_000_000));
        assert_eq!(c.command_power_mw(), output);
    }

    #[test]
    fn correction_uses_elapsed_time_and_native_command_horizon() {
        let mut c = GripperFeedbackController::default();
        c.observe(400, 30.0, 100_000_000);
        c.request(600, 30.0);
        c.request(0, 30.0);
        c.observe(1180, 30.0, 150_000_000);
        let expected = (880.0_f64 + 880.0 * (880.0 - 1180.0) / (880.0 + 1180.0) / 3.0)
            .round_ties_even() as u16;
        assert_eq!(c.command_power_mw(), expected);
    }

    #[test]
    fn command_quantization_does_not_discard_integral_progress() {
        let mut c = GripperFeedbackController::default();
        c.request(600, 30.0);
        c.request(0, 30.0);
        assert!(!c.observe(881, 30.0, 100_000_000));
        for sample in 2..20 {
            c.observe(881, 30.0, sample * 100_000_000);
        }
        assert!(c.command_power_mw() < 880);
    }

    #[test]
    fn delayed_linear_fixture_converges_without_unloading() {
        // Test plant only, not an identification of the physical servo. Exercise
        // delayed feedback and different gains, not just replay fixed samples.
        for gain in [0.5, 1.0, 2.0] {
            for delay in [1, 2, 3] {
                let mut c = GripperFeedbackController::default();
                c.request(600, 30.0);
                c.request(0, 30.0);
                let mut commands = std::collections::VecDeque::from(vec![880.0; delay]);
                let mut measurement = 0;
                for sample in 1..500 {
                    measurement = (commands.pop_front().unwrap() * gain) as u16;
                    c.observe(measurement, 30.0, sample * 100_000_000);
                    assert!(c.command_power_mw() > 1);
                    commands.push_back(f64::from(c.command_power_mw()));
                }
                assert!((i32::from(measurement) - 880).abs() <= 2);
            }
        }
    }
}
