//! Standalone generator for deterministic encrypted NOLO CV1 HID reports.

use anyhow::{Context, Result, bail};
use nolo_usb_server::simulator::{DEFAULT_REPORT_COUNT, REPORT_PERIOD_SECONDS, VirtualNolo};
use std::{
    fs::OpenOptions,
    io::{self, Write},
    path::PathBuf,
    thread,
    time::{Duration, Instant},
};

struct Config {
    output: Option<PathBuf>,
    reports: u64,
    realtime: bool,
}

fn main() -> Result<()> {
    let config = parse_config()?;
    let mut output: Box<dyn Write> = match config.output {
        Some(path) => Box::new(
            OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&path)
                .with_context(|| {
                    format!(
                        "failed creating {}; refusing to overwrite an existing capture",
                        path.display()
                    )
                })?,
        ),
        None => Box::new(io::stdout()),
    };
    let start = Instant::now();
    let period = Duration::from_secs_f64(REPORT_PERIOD_SECONDS);
    let mut simulator = VirtualNolo::new();
    for report_index in 0..config.reports {
        if config.realtime {
            let deadline = start + period.mul_f64(report_index as f64);
            thread::sleep(deadline.saturating_duration_since(Instant::now()));
        }
        output
            .write_all(&simulator.next_report()?)
            .context("failed writing virtual NOLO report")?;
    }
    output.flush().context("failed flushing virtual reports")?;
    Ok(())
}

fn parse_config() -> Result<Config> {
    let mut output = None;
    let mut reports = DEFAULT_REPORT_COUNT;
    let mut realtime = false;
    for argument in std::env::args().skip(1) {
        if let Some(value) = argument.strip_prefix("--output=") {
            if value != "-" {
                output = Some(PathBuf::from(value));
            }
        } else if let Some(value) = argument.strip_prefix("--reports=") {
            reports = value
                .parse()
                .with_context(|| format!("invalid report count: {value}"))?;
            if reports == 0 {
                bail!("report count must be greater than zero");
            }
        } else if argument == "--realtime" {
            realtime = true;
        } else if matches!(argument.as_str(), "-h" | "--help") {
            println!(
                "Usage: nolo-cv1-simulator [--output=PATH|-] [--reports=COUNT] [--realtime]\n\
                 Writes concatenated encrypted 64-byte NOLO CV1 HID reports.\n\
                 Default: one 49-second calibration/motion pass ({} reports) to stdout.",
                DEFAULT_REPORT_COUNT
            );
            std::process::exit(0);
        } else {
            bail!("unknown argument: {argument}");
        }
    }
    Ok(Config {
        output,
        reports,
        realtime,
    })
}
