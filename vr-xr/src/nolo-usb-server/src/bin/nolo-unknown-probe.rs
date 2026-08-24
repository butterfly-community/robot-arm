use anyhow::{Context, Result, bail};
use hidapi::HidApi;
use nolo_usb_server::protocol::{REPORT_SIZE, SUPPORTED_DEVICES, decode_report};
use std::{
    collections::{BTreeMap, BTreeSet},
    env,
    time::{Duration, Instant},
};

#[derive(Default)]
struct ControllerStats {
    reports: usize,
    byte_22: BTreeSet<u8>,
    byte_23: BTreeMap<u8, usize>,
    imu_samples: BTreeSet<([i16; 3], [i16; 3])>,
}

fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn main() -> Result<()> {
    let seconds = env::args()
        .nth(1)
        .map(|value| value.parse::<u64>())
        .transpose()
        .context("duration must be an integer number of seconds")?
        .unwrap_or(10);
    let api = HidApi::new().context("failed to initialize HID")?;
    let (device, vid, pid) = SUPPORTED_DEVICES
        .into_iter()
        .find_map(|(vid, pid)| api.open(vid, pid).ok().map(|device| (device, vid, pid)))
        .context("NOLO USB not found or already opened by another process")?;
    println!("NOLO USB {vid:04x}:{pid:04x}; sampling for {seconds} seconds");

    let deadline = Instant::now() + Duration::from_secs(seconds);
    let mut report = [0_u8; REPORT_SIZE + 1];
    let mut controllers = [ControllerStats::default(), ControllerStats::default()];
    let mut hmd_31_36 = BTreeSet::new();
    let mut hmd_43_48 = BTreeSet::new();
    let mut hmd_55_58 = BTreeSet::new();
    let mut byte_60 = BTreeSet::new();
    let mut tail_61_63 = BTreeSet::new();
    let mut previous_63: Option<u8> = None;
    let mut byte_63_transitions = 0_usize;
    let mut byte_63_plus_one = 0_usize;

    while Instant::now() < deadline {
        let size = device.read_timeout(&mut report, 200)?;
        if size == 0 {
            continue;
        }
        let raw_report: &[u8] = match size {
            REPORT_SIZE => &report[..REPORT_SIZE],
            size if size == REPORT_SIZE + 1 && report[0] == 0 => &report[1..],
            other => bail!("unexpected HID report length: {other}"),
        };
        let Some(frame) = decode_report(raw_report)? else {
            continue;
        };
        let stats = &mut controllers[usize::from(frame.controller_id)];
        stats.reports += 1;
        stats.byte_22.insert(frame.unknown_22);
        *stats.byte_23.entry(frame.unknown_23).or_default() += 1;
        stats
            .imu_samples
            .insert((frame.accelerometer, frame.gyroscope));
        hmd_31_36.insert(frame.unknown_31_36);
        hmd_43_48.insert(frame.unknown_43_48);
        hmd_55_58.insert(frame.unknown_55_58);
        byte_60.insert(frame.unknown_60);
        tail_61_63.insert(frame.unknown_61_63);
        if let Some(previous) = previous_63 {
            byte_63_transitions += 1;
            if frame.unknown_61_63[2] == previous.wrapping_add(1) {
                byte_63_plus_one += 1;
            }
        }
        previous_63 = Some(frame.unknown_61_63[2]);
    }

    for (id, stats) in controllers.iter().enumerate() {
        println!(
            "controller {id}: reports={} imu_unique={} byte22={:?} byte23={:?}",
            stats.reports,
            stats.imu_samples.len(),
            stats.byte_22,
            stats.byte_23
        );
    }
    println!("31..36 unique={}", hmd_31_36.len());
    for value in &hmd_31_36 {
        println!("  {}", hex(value));
    }
    println!("43..48 unique={}", hmd_43_48.len());
    for value in &hmd_43_48 {
        let signed = std::array::from_fn::<_, 3, _>(|index| {
            i16::from_le_bytes([value[index * 2], value[index * 2 + 1]])
        });
        println!("  {} -> {signed:?}", hex(value));
    }
    println!("55..58 unique={}", hmd_55_58.len());
    for value in &hmd_55_58 {
        println!("  {}", hex(value));
    }
    println!("byte60={byte_60:?}");
    println!("61..63 unique={} (showing at most 12)", tail_61_63.len());
    for value in tail_61_63.iter().take(12) {
        println!("  {}", hex(value));
    }
    println!("byte63 +1 transitions={byte_63_plus_one}/{byte_63_transitions}");
    Ok(())
}
