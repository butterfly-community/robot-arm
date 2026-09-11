//! Encoder finite differences for ROS state feedback, never command velocities.
use robot_arm_messages::{ArmState, FeedbackSource};

#[derive(Default)]
pub struct FeedbackVelocity {
    previous: Option<(i64, FeedbackSource, Vec<f64>)>,
    velocity: Vec<f64>,
}

impl FeedbackVelocity {
    pub fn update(&mut self, state: &ArmState) -> &[f64] {
        let positions: Vec<_> = state
            .joints_rad
            .iter()
            .chain(&state.actuators_rad)
            .copied()
            .collect();
        if let Some((time, source, previous)) = &self.previous {
            if *source == state.feedback_source && previous.len() == positions.len() {
                // Repeated snapshots do not mean the motor stopped. Preserve the
                // last derivative until another encoder sample actually arrives.
                if state.sample_time_ns == *time {
                    return &self.velocity;
                }
                if state.sample_time_ns > *time {
                    let seconds = (state.sample_time_ns - time) as f64 / 1e9;
                    self.velocity = positions
                        .iter()
                        .zip(previous)
                        .map(|(now, old)| (now - old) / seconds)
                        .collect();
                    self.previous = Some((state.sample_time_ns, state.feedback_source, positions));
                    return &self.velocity;
                }
            }
        }
        // First sample / source or clock reset has no measurable derivative.
        self.velocity = vec![0.0; positions.len()];
        self.previous = Some((state.sample_time_ns, state.feedback_source, positions));
        &self.velocity
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use robot_arm_messages::SCHEMA_VERSION;

    fn sample(time: i64, finger: f64) -> ArmState {
        ArmState {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            sample_time_ns: time,
            model_revision: "test".into(),
            joints_rad: vec![0.0; 6],
            actuators_rad: vec![finger],
            feedback_source: FeedbackSource::Hardware,
        }
    }

    #[test]
    fn closing_uses_encoder_time_and_duplicates_do_not_report_a_stop() {
        let mut velocity = FeedbackVelocity::default();
        assert_eq!(velocity.update(&sample(1_000_000_000, 0.2)), &[0.0; 7]);
        assert!((velocity.update(&sample(1_100_000_000, 0.1))[6] + 1.0).abs() < 1e-12);
        assert!((velocity.update(&sample(1_100_000_000, 0.1))[6] + 1.0).abs() < 1e-12);
        assert_eq!(velocity.update(&sample(1_200_000_000, 0.1))[6], 0.0);
    }

    #[test]
    fn clock_reset_does_not_produce_a_reverse_velocity() {
        let mut velocity = FeedbackVelocity::default();
        velocity.update(&sample(2_000_000_000, 0.2));
        assert_eq!(velocity.update(&sample(1_000_000_000, 0.1))[6], 0.0);
        assert!((velocity.update(&sample(1_100_000_000, 0.2))[6] - 1.0).abs() < 1e-12);
    }
}
