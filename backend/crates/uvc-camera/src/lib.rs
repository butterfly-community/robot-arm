//! V4L2 camera adapter. Native formats and device indices never leave this crate.
use eyre::{Context, Result, bail, eyre};
use robot_arm_messages::{
    CameraImagePlane, CameraRawVideoFrame, CameraSourceInfo, CameraStreamKind, CameraStreamProfile,
    SCHEMA_VERSION,
};
use std::{
    fs,
    path::{Path, PathBuf},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use v4l::{Device, Format, FourCC, buffer::Type, io::traits::CaptureStream, video::Capture};

fn supported(format: &str) -> bool {
    matches!(
        format,
        "MJPG" | "JPEG" | "YUYV" | "UYVY" | "RGB3" | "BGR3" | "GREY" | "NV12"
    )
}

// Prefer a serial-based udev link; port identity is the fallback for devices
// without a serial. Never persist the enumeration index.
fn identity(path: &Path, bus: &str) -> String {
    for directory in ["/dev/v4l/by-id", "/dev/v4l/by-path"] {
        let mut links: Vec<_> = fs::read_dir(directory)
            .into_iter()
            .flatten()
            .flatten()
            .collect();
        links.sort_by_key(|entry| entry.file_name());
        for link in links {
            if fs::canonicalize(link.path()).ok().as_deref() == Some(path) {
                return format!("v4l2:{}", link.path().display());
            }
        }
    }
    let index = path
        .file_name()
        .and_then(|name| {
            fs::read_to_string(Path::new("/sys/class/video4linux").join(name).join("index")).ok()
        })
        .unwrap_or_default();
    format!("v4l2:{bus}:{}", index.trim())
}

pub fn discover() -> (Vec<CameraSourceInfo>, Vec<String>) {
    let mut sources = vec![];
    let mut errors = vec![];
    for node in v4l::context::enum_devices() {
        match inspect(node.path()) {
            Ok(Some(source)) => sources.push(source),
            Ok(None) => {}
            Err(error) => errors.push(format!("{}: {error:#}", node.path().display())),
        }
    }
    sources.sort_by(|a, b| a.source_id.cmp(&b.source_id));
    (sources, errors)
}

fn inspect(path: &Path) -> Result<Option<CameraSourceInfo>> {
    let device = Device::with_path(path)?;
    let caps = device.query_caps()?;
    if !caps
        .capabilities
        .contains(v4l::capability::Flags::VIDEO_CAPTURE)
    {
        return Ok(None);
    }
    let mut profiles = vec![];
    for format in device.enum_formats()? {
        let pixel_format = format.fourcc.to_string();
        for size in device.enum_framesizes(format.fourcc)? {
            // Continuous devices also expose a current format. Keep the range
            // endpoints rather than inventing a fixed resolution list.
            let sizes = match size.size {
                v4l::framesize::FrameSizeEnum::Discrete(s) => vec![(s.width, s.height)],
                v4l::framesize::FrameSizeEnum::Stepwise(s) => {
                    vec![(s.min_width, s.min_height), (s.max_width, s.max_height)]
                }
            };
            for (width, height) in sizes {
                for interval in device.enum_frameintervals(format.fourcc, width, height)? {
                    let intervals = match interval.interval {
                        v4l::frameinterval::FrameIntervalEnum::Discrete(i) => vec![i],
                        v4l::frameinterval::FrameIntervalEnum::Stepwise(i) => vec![i.min, i.max],
                    };
                    for i in intervals {
                        if i.numerator == 0 {
                            continue;
                        }
                        // Keep exact fraction in the key; public FPS is descriptive.
                        let available = supported(&pixel_format);
                        profiles.push(CameraStreamProfile {
                            key: format!(
                                "{pixel_format}:{width}:{height}:{}:{}",
                                i.numerator, i.denominator
                            ),
                            stream: CameraStreamKind::Color,
                            width,
                            height,
                            frames_per_second: (f64::from(i.denominator) / f64::from(i.numerator))
                                .round() as u32,
                            pixel_format: pixel_format.clone(),
                            is_default: false,
                            available,
                            unavailable_reason: (!available)
                                .then(|| "当前适配器未支持此原生像素格式".into()),
                        });
                    }
                }
            }
        }
    }
    if profiles.is_empty() {
        return Ok(None);
    }
    profiles.sort_by_key(|p| {
        (
            std::cmp::Reverse(u64::from(p.width) * u64::from(p.height)),
            std::cmp::Reverse(p.frames_per_second),
            p.key.clone(),
        )
    });
    profiles.dedup_by(|a, b| a.key == b.key);
    Ok(Some(CameraSourceInfo {
        source_id: identity(path, &caps.bus),
        driver_id: "v4l2".into(),
        display_name: format!("{} · {}", caps.card, caps.bus),
        device_model: Some(caps.card),
        connection_type: Some("V4L2".into()),
        physical_port: Some(caps.bus),
        sensors: vec!["Color".into()],
        profiles,
        available: true,
        ..Default::default()
    }))
}

pub struct Stream {
    stream: v4l::io::mmap::Stream<'static>,
    format: Format,
    source_id: String,
    last_valid_frame: Instant,
    timeout: Duration,
    decoder: camera_media::NativeDecoder,
}

impl Stream {
    pub fn open(source_id: &str, profile: &CameraStreamProfile) -> Result<Self> {
        let mut path: Option<PathBuf> = None;
        for node in v4l::context::enum_devices() {
            if let Ok(device) = Device::with_path(node.path())
                && let Ok(caps) = device.query_caps()
                && identity(node.path(), &caps.bus) == source_id
            {
                path = Some(node.path().to_owned());
                break;
            }
        }
        let path =
            path.ok_or_else(|| eyre!("相机未连接：{source_id}；绑定已保留，请重新连接并刷新"))?;
        let device = Device::with_path(&path).wrap_err("打开普通摄像头")?;
        let parts: Vec<_> = profile.key.split(':').collect();
        if parts.len() != 5 || !supported(parts[0]) {
            bail!("无效的 V4L2 流配置");
        }
        let fourcc: [u8; 4] = parts[0].as_bytes().try_into()?;
        let format = device.set_format(&Format::new(
            profile.width,
            profile.height,
            FourCC::new(&fourcc),
        ))?;
        let numerator: u32 = parts[3].parse()?;
        let denominator: u32 = parts[4].parse()?;
        let mut params = device.params()?;
        params.interval = v4l::Fraction::new(numerator, denominator);
        let actual_params = device.set_params(&params)?;
        if format.width != profile.width
            || format.height != profile.height
            || format.fourcc != FourCC::new(&fourcc)
            || u64::from(actual_params.interval.numerator) * u64::from(denominator)
                != u64::from(numerator) * u64::from(actual_params.interval.denominator)
        {
            bail!("设备未接受所选分辨率、格式或帧率，请刷新后选择驱动支持的配置");
        }
        let mut stream = v4l::io::mmap::Stream::new(&device, Type::VideoCapture)?;
        // Driver I/O deadline, not a robot motion limit. A stalled capture must
        // release its worker and explicitly clear the preview instead of hanging.
        let timeout = Duration::from_secs(1)
            + Duration::from_secs_f64(2.0 * f64::from(numerator) / f64::from(denominator));
        stream.set_timeout(timeout);
        Ok(Self {
            stream,
            format,
            source_id: source_id.into(),
            last_valid_frame: Instant::now(),
            timeout,
            decoder: camera_media::NativeDecoder::default(),
        })
    }

    pub fn next_frame(&mut self) -> Result<Option<camera_media::DecodedFrame>> {
        let (bytes, metadata) = self.stream.next().wrap_err("相机采集失败或信号中断")?;
        let received_time_ns = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos() as i64;
        let sequence = u64::from(metadata.sequence);
        let decoded = bytes
            .get(..metadata.bytesused as usize)
            .ok_or_else(|| eyre!("相机帧长度超过采集缓冲区"))
            .and_then(|bytes| {
                self.decoder.decode(CameraRawVideoFrame {
                    schema_version: SCHEMA_VERSION,
                    sequence,
                    source_id: self.source_id.clone(),
                    device_time_ns: Duration::from(metadata.timestamp).as_nanos() as i64,
                    received_time_ns,
                    color: CameraImagePlane {
                        width: self.format.width,
                        height: self.format.height,
                        stride_bytes: self.format.stride,
                        pixel_format: self.format.fourcc.to_string(),
                        frame_id: "color_optical_frame".into(),
                        data: bytes.to_vec(),
                    },
                })
            });
        let frame = match decoded {
            Ok(frame) => frame,
            Err(error) => {
                return pending_frame(self.last_valid_frame.elapsed(), self.timeout, error);
            }
        };
        self.last_valid_frame = Instant::now();
        Ok(Some(frame))
    }
}

// UVC can return incomplete JPEGs during stream transitions. Discard only the
// unusable frame, never publish stale pixels; sustained failure keeps its cause
// and uses the same existing capture deadline as missing/wrong-sized frames.
fn pending_frame<T>(
    elapsed: Duration,
    timeout: Duration,
    error: eyre::Report,
) -> Result<Option<T>> {
    if elapsed > timeout {
        Err(error.wrap_err("相机持续未输出有效图像"))
    } else {
        Ok(None)
    }
}

#[cfg(test)]
fn decode(format: &Format, bytes: &[u8]) -> Result<CameraImagePlane> {
    camera_media::native_rgb(
        format.width,
        format.height,
        format.stride,
        &format.fourcc.to_string(),
        bytes,
    )
    .wrap_err("FFmpeg 原生相机图像解码失败")
}

#[cfg(test)]
mod tests {
    use super::*;
    use opencv::{core, imgcodecs, prelude::*};
    #[test]
    fn bad_frame_is_pending_but_sustained_failure_preserves_the_cause() {
        let timeout = Duration::from_secs(1);
        let error = || decode(&Format::new(2, 1, FourCC::new(b"MJPG")), &[0, 1, 2]).unwrap_err();
        assert!(
            pending_frame::<CameraRawVideoFrame>(Duration::ZERO, timeout, error())
                .unwrap()
                .is_none()
        );
        let failure = pending_frame::<CameraRawVideoFrame>(
            timeout + Duration::from_millis(1),
            timeout,
            error(),
        )
        .unwrap_err();
        assert!(format!("{failure:#}").contains("FFmpeg 原生相机图像解码失败"));
    }

    #[test]
    fn jpeg_dimensions_are_read_from_payload_not_mislabeled_as_requested_size() {
        let pixels = core::Mat::from_slice(&[20_u8, 30, 40, 50, 60, 70]).unwrap();
        let pixels = pixels.reshape(3, 1).unwrap();
        let mut bytes = core::Vector::new();
        imgcodecs::imencode(".jpg", &pixels, &mut bytes, &core::Vector::new()).unwrap();
        let decoded = decode(
            &Format::new(1920, 1080, FourCC::new(b"MJPG")),
            bytes.as_slice(),
        )
        .unwrap();
        assert_eq!(
            (decoded.width, decoded.height, decoded.stride_bytes),
            (2, 1, 6)
        );
        for (actual, expected) in decoded.data.iter().zip([40_u8, 30, 20, 70, 60, 50]) {
            assert!(actual.abs_diff(expected) < 5, "JPEG BGR-to-RGB boundary");
        }
    }
    #[test]
    fn bgr_and_rgb_decode_to_the_same_rgb_without_padding() {
        for (name, bytes) in [
            (b"RGB3", vec![255, 0, 0, 0, 0, 255, 99, 99]),
            (b"BGR3", vec![0, 0, 255, 255, 0, 0, 99, 99]),
        ] {
            let mut format = Format::new(2, 1, FourCC::new(name));
            format.stride = 8;
            assert_eq!(
                decode(&format, &bytes).unwrap().data,
                vec![255, 0, 0, 0, 0, 255]
            );
        }
    }
}
