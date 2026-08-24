//! Independent three-axis One Euro filtering for optical position samples.

use one_euro_filter::OneEuroFilter;

/// The parameters used by the official Rust example in casiez/OneEuroFilter.
///
/// Only `nominal_rate_hz` is device-specific: each controller produces about
/// 120 new position samples per second. The other values intentionally stay at
/// the upstream reference values instead of being tuned locally without a
/// measurement rig.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionFilterSettings {
    pub nominal_rate_hz: f64,
    pub min_cutoff_hz: f64,
    pub beta: f64,
    pub derivative_cutoff_hz: f64,
}

impl Default for PositionFilterSettings {
    fn default() -> Self {
        Self {
            nominal_rate_hz: 120.0,
            min_cutoff_hz: 1.0,
            beta: 0.1,
            derivative_cutoff_hz: 1.0,
        }
    }
}

impl PositionFilterSettings {
    pub fn validate(self) -> Result<Self, &'static str> {
        if !self.nominal_rate_hz.is_finite() || self.nominal_rate_hz <= 0.0 {
            return Err("nominal rate must be finite and greater than zero");
        }
        if !self.min_cutoff_hz.is_finite() || self.min_cutoff_hz <= 0.0 {
            return Err("minimum cutoff must be finite and greater than zero");
        }
        if !self.beta.is_finite() || self.beta < 0.0 {
            return Err("beta must be finite and non-negative");
        }
        if !self.derivative_cutoff_hz.is_finite() || self.derivative_cutoff_hz <= 0.0 {
            return Err("derivative cutoff must be finite and greater than zero");
        }
        Ok(self)
    }
}

pub struct PositionFilter {
    settings: PositionFilterSettings,
    axes: [OneEuroFilter; 3],
    last_timestamp_seconds: Option<f64>,
}

impl PositionFilter {
    pub fn new(settings: PositionFilterSettings) -> Self {
        let settings = settings
            .validate()
            .expect("position filter settings must be valid");
        Self {
            settings,
            axes: create_axes(settings),
            last_timestamp_seconds: None,
        }
    }

    /// Filters one complete XYZ sample using seconds on a monotonic clock.
    ///
    /// Invalid input clears filter history so it cannot contaminate the next
    /// valid tracking interval. The first valid sample after construction or a
    /// reset passes through unchanged, matching the upstream implementation.
    pub fn update(&mut self, position: [f32; 3], timestamp_seconds: f64) -> Option<[f32; 3]> {
        let timestamp_invalid = !timestamp_seconds.is_finite()
            || timestamp_seconds < 0.0
            || self
                .last_timestamp_seconds
                .is_some_and(|previous| timestamp_seconds <= previous);
        if timestamp_invalid || position.iter().any(|value| !value.is_finite()) {
            self.reset();
            return None;
        }

        let filtered = std::array::from_fn(|axis| {
            self.axes[axis].filter(f64::from(position[axis]), timestamp_seconds) as f32
        });
        if filtered.iter().any(|value| !value.is_finite()) {
            self.reset();
            None
        } else {
            self.last_timestamp_seconds = Some(timestamp_seconds);
            Some(filtered)
        }
    }

    pub fn reset(&mut self) {
        self.axes = create_axes(self.settings);
        self.last_timestamp_seconds = None;
    }
}

fn create_axes(settings: PositionFilterSettings) -> [OneEuroFilter; 3] {
    std::array::from_fn(|_| {
        OneEuroFilter::new(
            settings.nominal_rate_hz,
            settings.min_cutoff_hz,
            settings.beta,
            settings.derivative_cutoff_hz,
        )
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_near(left: f32, right: f32) {
        assert!((left - right).abs() < 1.0e-6, "{left} != {right}");
    }

    #[test]
    fn first_sample_passes_through() {
        let mut filter = PositionFilter::new(PositionFilterSettings::default());
        let value = filter.update([1.0, -2.0, 3.0], 10.0).unwrap();
        assert_eq!(value, [1.0, -2.0, 3.0]);
    }

    #[test]
    fn filters_each_axis_independently() {
        let mut filter = PositionFilter::new(PositionFilterSettings::default());
        filter.update([0.0, 0.0, 0.0], 0.0).unwrap();
        let value = filter.update([1.0, -2.0, 0.0], 1.0 / 120.0).unwrap();
        assert!(value[0] > 0.0 && value[0] < 1.0);
        assert!(value[1] < 0.0 && value[1] > -2.0);
        assert_near(value[2], 0.0);
    }

    #[test]
    fn reset_discards_previous_history() {
        let mut filter = PositionFilter::new(PositionFilterSettings::default());
        filter.update([0.0; 3], 0.0).unwrap();
        filter.update([1.0; 3], 1.0 / 120.0).unwrap();
        filter.reset();
        assert_eq!(filter.update([2.0; 3], 1.0), Some([2.0; 3]));
    }

    #[test]
    fn invalid_sample_resets_and_is_not_published() {
        let mut filter = PositionFilter::new(PositionFilterSettings::default());
        filter.update([0.0; 3], 0.0).unwrap();
        assert_eq!(filter.update([f32::NAN, 1.0, 2.0], 0.1), None);
        assert_eq!(filter.update([3.0, 4.0, 5.0], 0.2), Some([3.0, 4.0, 5.0]));
    }

    #[test]
    fn non_monotonic_timestamp_resets_history() {
        let mut filter = PositionFilter::new(PositionFilterSettings::default());
        filter.update([0.0; 3], 1.0).unwrap();
        assert_eq!(filter.update([1.0; 3], 0.5), None);
        assert_eq!(filter.update([2.0; 3], 2.0), Some([2.0; 3]));
    }

    #[test]
    fn rejects_invalid_settings() {
        let invalid = PositionFilterSettings {
            min_cutoff_hz: 0.0,
            ..PositionFilterSettings::default()
        };
        assert!(invalid.validate().is_err());
    }
}
