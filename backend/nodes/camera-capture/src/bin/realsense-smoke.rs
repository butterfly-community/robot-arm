#![allow(dead_code)]

#[path = "../drivers/mod.rs"]
mod drivers;

use std::time::{Duration, Instant};

use eyre::{Result, eyre};
use robot_arm_messages::CameraStreamKind;

use crate::drivers::RealSenseDriver;

fn main() -> Result<()> {
    let mut driver = RealSenseDriver::new()?;
    let sources = driver.discover()?;
    println!("{}", serde_json::to_string_pretty(&sources)?);
    let source = sources
        .first()
        .ok_or_else(|| eyre!("没有发现 RealSense 相机"))?;
    let color = source
        .profiles
        .iter()
        .find(|profile| {
            profile.stream == CameraStreamKind::Color && profile.is_default && profile.available
        })
        .or_else(|| {
            source
                .profiles
                .iter()
                .find(|profile| profile.stream == CameraStreamKind::Color && profile.available)
        })
        .ok_or_else(|| eyre!("相机没有可用彩色 profile"))?;
    let depth = source
        .profiles
        .iter()
        .find(|profile| {
            profile.stream == CameraStreamKind::Depth && profile.is_default && profile.available
        })
        .or_else(|| {
            source
                .profiles
                .iter()
                .find(|profile| profile.stream == CameraStreamKind::Depth && profile.available)
        })
        .ok_or_else(|| eyre!("相机没有可用深度 profile"))?;
    let source_id = source.source_id.clone();
    let color = color.clone();
    let depth = depth.clone();
    let mut stream = driver.open(&source_id, &color, &depth, &[])?;
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline {
        if let Some(frame) = stream.next_frameset()? {
            println!(
                "frameset source={} sequence={} color={}x{} {} depth={}x{} {} scale={}",
                frame.source_id,
                frame.sequence,
                frame.color.width,
                frame.color.height,
                frame.color.pixel_format,
                frame.depth.width,
                frame.depth.height,
                frame.depth.pixel_format,
                frame.depth_scale_m
            );
            println!(
                "intrinsics color={} {:?} depth={} {:?} depth_to_color={:?}",
                frame.color_intrinsics.distortion_model,
                frame.color_intrinsics.distortion,
                frame.depth_intrinsics.distortion_model,
                frame.depth_intrinsics.distortion,
                frame.depth_to_color
            );
            drop(stream);
            let refreshed = driver.discover()?;
            if !refreshed.iter().any(|source| source.source_id == source_id) {
                return Err(eyre!("停止后重新枚举未发现同一相机"));
            }
            let mut reopened = driver.open(&source_id, &color, &depth, &[])?;
            let reopen_deadline = Instant::now() + Duration::from_secs(10);
            while Instant::now() < reopen_deadline {
                if let Some(reopened_frame) = reopened.next_frameset()? {
                    println!(
                        "reopened source={} sequence={}",
                        reopened_frame.source_id, reopened_frame.sequence
                    );
                    return Ok(());
                }
                std::thread::sleep(Duration::from_millis(5));
            }
            return Err(eyre!("重新打开后 10 秒内没有收到完整 RGB-D frameset"));
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    Err(eyre!("10 秒内没有收到完整 RGB-D frameset"))
}
