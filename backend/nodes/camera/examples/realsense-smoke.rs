use std::time::{Duration, Instant};

use eyre::{Result, eyre};
use robot_arm_messages::CameraStreamKind;

use realsense_camera::RealSenseDriver;

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
        if let realsense_camera::FramePoll::Captured {
            frame: Some(frame), ..
        } = stream.poll_frame(true)?
        {
            println!(
                "frameset source={} sequence={} color={}x{} {} depth={}x{} {} scale={}",
                frame.source_id,
                frame.sequence,
                frame.color.width,
                frame.color.height,
                frame.color.pixel_format,
                frame.aligned_depth.width,
                frame.aligned_depth.height,
                frame.aligned_depth.pixel_format,
                frame.depth_scale_m
            );
            println!(
                "aligned intrinsics={} {:?}",
                frame.intrinsics.distortion_model, frame.intrinsics.distortion
            );
            drop(stream);
            let refreshed = driver.discover()?;
            if !refreshed.iter().any(|source| source.source_id == source_id) {
                return Err(eyre!("停止后重新枚举未发现同一相机"));
            }
            let mut reopened = driver.open(&source_id, &color, &depth, &[])?;
            let reopen_deadline = Instant::now() + Duration::from_secs(10);
            while Instant::now() < reopen_deadline {
                if let realsense_camera::FramePoll::Captured {
                    frame: Some(reopened_frame),
                    ..
                } = reopened.poll_frame(true)?
                {
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
