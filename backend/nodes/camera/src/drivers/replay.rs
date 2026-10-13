//! Explicit file-backed colour source. It only supplies RGB, never depth,
//! intrinsics, robot feedback or a pre-solved calibration to the stereo pipeline.
use super::{CameraPoll, CameraStream};
use eyre::{Result, eyre};
use robot_arm_messages::{
    CameraImagePlane, CameraRawVideoFrame, CameraSourceInfo, CameraStreamKind, CameraStreamProfile,
    SCHEMA_VERSION,
};
use serde::Deserialize;
use std::{
    path::PathBuf,
    time::{Duration, Instant},
};

#[derive(Deserialize)]
struct Manifest {
    sources: Vec<Source>,
}
#[derive(Deserialize)]
struct Source {
    source_id: String,
    display_name: String,
    width: u32,
    height: u32,
    frames: Vec<Frame>,
}
#[derive(Deserialize)]
struct Frame {
    key: String,
    file: String,
}
fn root() -> PathBuf {
    std::env::var_os("CAMERA_STEREO_ASSETS")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("assets/stereo"))
}
fn manifest() -> Result<Manifest> {
    Ok(serde_json::from_slice(&std::fs::read(
        root().join("manifest.json"),
    )?)?)
}
pub fn discover() -> Result<Vec<CameraSourceInfo>> {
    Ok(manifest()?
        .sources
        .into_iter()
        .map(|s| {
            let profiles = s
                .frames
                .iter()
                .flat_map(|f| {
                    [(s.width, s.height), (1280, 720)].map(|(w, h)| CameraStreamProfile {
                        key: format!("{}:{w}x{h}:rgb8:10", f.key),
                        stream: CameraStreamKind::Color,
                        width: w,
                        height: h,
                        frames_per_second: 10,
                        pixel_format: "rgb8".into(),
                        is_default: f.key == "scene" && w == s.width,
                        available: true,
                        unavailable_reason: None,
                    })
                })
                .collect();
            CameraSourceInfo {
                source_id: s.source_id,
                display_name: s.display_name,
                driver_id: "replay".into(),
                device_model: Some("确定性双目 RGB 测试资产".into()),
                serial_number: None,
                firmware_version: None,
                connection_type: Some("测试画面，不连接设备".into()),
                physical_port: None,
                sensors: vec!["color".into()],
                profiles,
                driver_extensions: vec![],
                available: true,
            }
        })
        .collect())
}
pub fn open(source: &str, profile: &CameraStreamProfile) -> Result<Box<dyn CameraStream>> {
    let source = manifest()?
        .sources
        .into_iter()
        .find(|s| s.source_id == source)
        .ok_or_else(|| eyre!("未知测试相机"))?;
    let frame = source
        .frames
        .iter()
        .find(|f| profile.key.starts_with(&format!("{}:", f.key)))
        .ok_or_else(|| eyre!("未知测试画面"))?;
    let rgb = image::open(root().join(&frame.file))?
        .resize_exact(
            profile.width,
            profile.height,
            image::imageops::FilterType::Lanczos3,
        )
        .into_rgb8()
        .into_raw();
    Ok(Box::new(Replay {
        source: source.source_id,
        profile: profile.clone(),
        rgb,
        sequence: 0,
        next: Instant::now(),
    }))
}
struct Replay {
    source: String,
    profile: CameraStreamProfile,
    rgb: Vec<u8>,
    sequence: u64,
    next: Instant,
}
impl CameraStream for Replay {
    fn poll_frame(&mut self, _materialize: bool) -> Result<CameraPoll> {
        if Instant::now() < self.next {
            return Ok(CameraPoll::Pending);
        }
        self.next =
            Instant::now() + Duration::from_secs_f64(1.0 / self.profile.frames_per_second as f64);
        self.sequence += 1;
        let time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_nanos() as i64;
        let raw = CameraRawVideoFrame {
            schema_version: SCHEMA_VERSION,
            sequence: self.sequence,
            source_id: self.source.clone(),
            device_time_ns: time,
            received_time_ns: time,
            color: CameraImagePlane {
                width: self.profile.width,
                height: self.profile.height,
                stride_bytes: self.profile.width * 3,
                pixel_format: "rgb8".into(),
                frame_id: format!("{}_optical", self.source.replace(':', "_")),
                data: self.rgb.clone(),
            },
        };
        Ok(CameraPoll::Captured {
            video_frame: Box::new(camera_media::NativeDecoder::default().decode(raw)?),
            frame: None,
        })
    }
}
