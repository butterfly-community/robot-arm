//! Sampling sequence freshness and quality statistics.

use std::time::{Duration, Instant};

const RATE_EMA_ALPHA: f32 = 0.05;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SampleObservation {
    pub changed: bool,
    pub fresh: bool,
    pub unchanged: Duration,
    pub sequence_delta: u8,
    pub samples_received: u64,
    pub samples_missed: u64,
    pub duplicate_reports: u64,
    pub measured_rate_hz: Option<f32>,
    pub jitter_ms: Option<f32>,
}

#[derive(Debug)]
pub struct SampleTracker {
    previous_sequence: Option<u8>,
    last_change: Option<Instant>,
    nominal_period: Duration,
    stale_after: Duration,
    samples_received: u64,
    samples_missed: u64,
    duplicate_reports: u64,
    period_ema_seconds: Option<f32>,
    jitter_ema_seconds: Option<f32>,
}

/// Tracks one logical source whose sequence counters are carried by multiple
/// interleaved HID report streams. NOLO's HMD fields use one sequence stream
/// per Controller report type, so comparing byte 59 across adjacent report
/// types produces false jumps.
#[derive(Debug)]
pub struct InterleavedSampleTracker<const N: usize> {
    streams: [SampleTracker; N],
}

impl<const N: usize> InterleavedSampleTracker<N> {
    pub fn new(per_stream_rate_hz: f32, stale_after: Duration) -> Self {
        assert!(N > 0);
        Self {
            streams: std::array::from_fn(|_| SampleTracker::new(per_stream_rate_hz, stale_after)),
        }
    }

    pub fn observe(&mut self, stream: usize, sequence: u8, now: Instant) -> SampleObservation {
        let current = self.streams[stream].observe(sequence, now);
        self.aggregate(now, current.changed, current.sequence_delta)
    }

    pub fn status(&self, now: Instant) -> SampleObservation {
        self.aggregate(now, false, 0)
    }

    fn aggregate(&self, now: Instant, changed: bool, sequence_delta: u8) -> SampleObservation {
        let streams: [SampleObservation; N] =
            std::array::from_fn(|index| self.streams[index].status(now));
        let (measured_rate_count, measured_rate_sum) = streams
            .iter()
            .filter_map(|stream| stream.measured_rate_hz)
            .fold((0_u8, 0.0_f32), |(count, sum), rate| {
                (count + 1, sum + rate)
            });
        SampleObservation {
            changed,
            fresh: streams.iter().any(|stream| stream.fresh),
            unchanged: streams
                .iter()
                .map(|stream| stream.unchanged)
                .min()
                .unwrap_or(Duration::MAX),
            sequence_delta,
            samples_received: streams.iter().map(|stream| stream.samples_received).sum(),
            samples_missed: streams.iter().map(|stream| stream.samples_missed).sum(),
            duplicate_reports: streams.iter().map(|stream| stream.duplicate_reports).sum(),
            measured_rate_hz: (measured_rate_count > 0).then_some(measured_rate_sum),
            jitter_ms: streams
                .iter()
                .filter_map(|stream| stream.jitter_ms)
                .reduce(f32::max),
        }
    }
}

impl SampleTracker {
    pub fn new(nominal_rate_hz: f32, stale_after: Duration) -> Self {
        assert!(nominal_rate_hz.is_finite() && nominal_rate_hz > 0.0);
        Self {
            previous_sequence: None,
            last_change: None,
            nominal_period: Duration::from_secs_f32(1.0 / nominal_rate_hz),
            stale_after,
            samples_received: 0,
            samples_missed: 0,
            duplicate_reports: 0,
            period_ema_seconds: None,
            jitter_ema_seconds: None,
        }
    }

    pub fn observe(&mut self, sequence: u8, now: Instant) -> SampleObservation {
        let delta = self
            .previous_sequence
            .map(|previous| sequence.wrapping_sub(previous))
            .unwrap_or(1);
        let changed = self.previous_sequence.is_none() || delta != 0;

        if changed {
            if let Some(previous_change) = self.last_change {
                let interval = now.duration_since(previous_change).as_secs_f32();
                let per_sample = interval / f32::from(delta);
                self.period_ema_seconds = Some(ema(self.period_ema_seconds, per_sample));
                let jitter = (per_sample - self.nominal_period.as_secs_f32()).abs();
                self.jitter_ema_seconds = Some(ema(self.jitter_ema_seconds, jitter));
            }
            self.samples_received = self.samples_received.saturating_add(1);
            self.samples_missed = self
                .samples_missed
                .saturating_add(u64::from(delta.saturating_sub(1)));
            self.previous_sequence = Some(sequence);
            self.last_change = Some(now);
        } else {
            self.duplicate_reports = self.duplicate_reports.saturating_add(1);
        }

        self.snapshot(now, changed, if changed { delta } else { 0 })
    }

    pub fn status(&self, now: Instant) -> SampleObservation {
        self.snapshot(now, false, 0)
    }

    fn snapshot(&self, now: Instant, changed: bool, sequence_delta: u8) -> SampleObservation {
        let unchanged = self
            .last_change
            .map(|last_change| now.duration_since(last_change))
            .unwrap_or(Duration::MAX);
        SampleObservation {
            changed,
            fresh: self.last_change.is_some() && unchanged <= self.stale_after,
            unchanged,
            sequence_delta,
            samples_received: self.samples_received,
            samples_missed: self.samples_missed,
            duplicate_reports: self.duplicate_reports,
            measured_rate_hz: self
                .period_ema_seconds
                .filter(|period| *period > 0.0)
                .map(|period| 1.0 / period),
            jitter_ms: self.jitter_ema_seconds.map(|jitter| jitter * 1000.0),
        }
    }
}

fn ema(previous: Option<f32>, value: f32) -> f32 {
    previous
        .map(|previous| previous + RATE_EMA_ALPHA * (value - previous))
        .unwrap_or(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    const STALE: Duration = Duration::from_millis(200);

    #[test]
    fn duplicate_does_not_become_a_new_sample_and_eventually_goes_stale() {
        let start = Instant::now();
        let mut tracker = SampleTracker::new(120.0, STALE);
        assert!(tracker.observe(10, start).changed);

        let duplicate = tracker.observe(10, start + Duration::from_millis(50));
        assert!(!duplicate.changed);
        assert!(duplicate.fresh);
        assert_eq!(duplicate.duplicate_reports, 1);

        let stale = tracker.status(start + Duration::from_millis(201));
        assert!(!stale.fresh);
        assert_eq!(stale.samples_received, 1);
    }

    #[test]
    fn wrapping_sequence_does_not_report_a_drop() {
        let start = Instant::now();
        let mut tracker = SampleTracker::new(120.0, STALE);
        tracker.observe(255, start);
        let wrapped = tracker.observe(0, start + Duration::from_millis(8));
        assert_eq!(wrapped.sequence_delta, 1);
        assert_eq!(wrapped.samples_missed, 0);
    }

    #[test]
    fn sequence_gap_counts_missing_samples_and_estimates_source_rate() {
        let start = Instant::now();
        let mut tracker = SampleTracker::new(120.0, STALE);
        tracker.observe(40, start);
        let observation = tracker.observe(43, start + Duration::from_millis(25));
        assert_eq!(observation.sequence_delta, 3);
        assert_eq!(observation.samples_missed, 2);
        assert!((observation.measured_rate_hz.unwrap() - 120.0).abs() < 0.01);
    }

    #[test]
    fn interleaved_sequence_streams_do_not_create_false_hmd_drops() {
        let start = Instant::now();
        let mut tracker = InterleavedSampleTracker::<2>::new(120.0, STALE);
        tracker.observe(0, 52, start);
        tracker.observe(1, 61, start + Duration::from_micros(4_167));
        tracker.observe(0, 53, start + Duration::from_micros(8_333));
        let observation = tracker.observe(1, 62, start + Duration::from_micros(12_500));

        assert!(observation.changed);
        assert!(observation.fresh);
        assert_eq!(observation.sequence_delta, 1);
        assert_eq!(observation.samples_missed, 0);
        assert_eq!(observation.samples_received, 4);
        assert!((observation.measured_rate_hz.unwrap() - 240.0).abs() < 0.02);
    }
}
