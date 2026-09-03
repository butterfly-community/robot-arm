#![cfg(feature = "runtime")]

use std::{collections::HashSet, convert::TryFrom, slice, task::Poll};

use eyre::{Context as _, Result, bail};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion};
use num_traits::FromPrimitive;
use realsense_rust::{
    config::Config,
    context::Context,
    device::Device,
    frame::{ColorFrame, DepthFrame, FrameEx},
    kind::{Rs2CameraInfo, Rs2DistortionModel, Rs2Format, Rs2Option, Rs2StreamKind},
    pipeline::{ActivePipeline, InactivePipeline},
    stream_profile::StreamProfile,
};
use robot_arm_messages::{
    CameraDriverExtensionInfo, CameraDriverParameterInfo, CameraDriverParameterKind,
    CameraDriverParameterValue, CameraFrameBundle, CameraImagePlane, CameraIntrinsics,
    CameraSourceInfo, CameraStreamKind, CameraStreamProfile, Pose3, SCHEMA_VERSION,
};

pub const DRIVER_NAMESPACE: &str = "librealsense2";

pub struct RealSenseDriver {
    context: Context,
}

impl RealSenseDriver {
    pub fn new() -> Result<Self> {
        Ok(Self {
            context: Context::new().context("创建 librealsense context")?,
        })
    }
}

impl RealSenseDriver {
    pub fn discover(&mut self) -> Result<Vec<CameraSourceInfo>> {
        Ok(self
            .context
            .query_devices(HashSet::new())
            .iter()
            .filter_map(source_info)
            .collect())
    }

    pub fn open(
        &mut self,
        source_id: &str,
        color_profile: &CameraStreamProfile,
        depth_profile: &CameraStreamProfile,
        driver_parameters: &[CameraDriverParameterValue],
    ) -> Result<RealSenseStream> {
        let serial = source_id
            .strip_prefix("realsense:")
            .ok_or_else(|| eyre::eyre!("RealSense source_id 无效"))?;
        let devices = self.context.query_devices(HashSet::new());
        let device = devices
            .iter()
            .find(|device| info(device, Rs2CameraInfo::SerialNumber).as_deref() == Some(serial))
            .ok_or_else(|| eyre::eyre!("RealSense {serial} 当前不可用"))?;
        apply_driver_parameters(device, driver_parameters)?;
        let serial_cstr = device
            .info(Rs2CameraInfo::SerialNumber)
            .ok_or_else(|| eyre::eyre!("RealSense 没有序列号"))?;
        let mut config = Config::new();
        config
            .enable_device_from_serial(serial_cstr)?
            .disable_all_streams()?;
        enable_profile(&mut config, color_profile)?;
        enable_profile(&mut config, depth_profile)?;
        let pipeline = InactivePipeline::try_from(&self.context)
            .map_err(|error| eyre::eyre!("创建 RealSense pipeline: {error:?}"))?
            .start(Some(config))
            .map_err(|error| eyre::eyre!("启动 RealSense pipeline: {error:?}"))?;
        Ok(RealSenseStream {
            source_id: source_id.into(),
            pipeline,
        })
    }
}

pub struct RealSenseStream {
    source_id: String,
    pipeline: ActivePipeline,
}

impl RealSenseStream {
    pub fn next_frameset(&mut self) -> Result<Option<CameraFrameBundle>> {
        let Poll::Ready(frames) = self.pipeline.poll()? else {
            return Ok(None);
        };
        let mut colors = frames.frames_of_type::<ColorFrame>();
        let mut depths = frames.frames_of_type::<DepthFrame>();
        let color = colors
            .pop()
            .ok_or_else(|| eyre::eyre!("frameset 缺少彩色帧"))?;
        let depth = depths
            .pop()
            .ok_or_else(|| eyre::eyre!("frameset 缺少深度帧"))?;
        let color_profile = color.stream_profile();
        let depth_profile = depth.stream_profile();
        let color_intrinsics = intrinsics(color_profile)?;
        let depth_intrinsics = intrinsics(depth_profile)?;
        let extrinsics = depth_profile.extrinsics(color_profile)?;
        let rotation = extrinsics.rotation();
        let matrix = Matrix3::from_column_slice(&rotation.map(f64::from));
        let orientation =
            UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(matrix));
        let q = orientation.quaternion();
        let received_time_ns = now_ns();
        Ok(Some(CameraFrameBundle {
            schema_version: SCHEMA_VERSION,
            sequence: depth.frame_number(),
            source_id: self.source_id.clone(),
            device_time_ns: (depth.timestamp() * 1_000_000.0).round() as i64,
            device_time_domain: depth.timestamp_domain().to_string(),
            received_time_ns,
            color: image_plane(&color, "color_optical_frame")?,
            depth: image_plane(&depth, "depth_optical_frame")?,
            color_intrinsics,
            depth_intrinsics,
            depth_to_color: Pose3 {
                position_m: extrinsics.translation().map(f64::from),
                orientation_xyzw: [q.i, q.j, q.k, q.w],
            },
            depth_scale_m: f64::from(
                depth
                    .depth_units()
                    .map_err(|error| eyre::eyre!("读取深度单位: {error:?}"))?,
            ),
        }))
    }
}

fn source_info(device: &Device) -> Option<CameraSourceInfo> {
    let serial = info(device, Rs2CameraInfo::SerialNumber)?;
    let model = info(device, Rs2CameraInfo::Name).unwrap_or_else(|| "RealSense".into());
    let mut sensors = Vec::new();
    let mut profiles = Vec::new();
    let mut parameters = Vec::new();
    for sensor in device.sensors() {
        let sensor_name = sensor
            .info(Rs2CameraInfo::Name)
            .and_then(|value| value.to_str().ok())
            .unwrap_or("未命名传感器")
            .to_owned();
        sensors.push(sensor_name.clone());
        profiles.extend(sensor.stream_profiles().iter().filter_map(profile_info));
        parameters.extend(sensor_parameters(&sensor_name, &sensor));
    }
    profiles.sort_by(|left, right| left.key.cmp(&right.key));
    profiles.dedup_by(|left, right| left.key == right.key);
    sensors.sort();
    sensors.dedup();
    Some(CameraSourceInfo {
        source_id: format!("realsense:{serial}"),
        driver_id: "librealsense2".into(),
        display_name: format!("{model} · {serial}"),
        device_model: Some(model),
        serial_number: Some(serial),
        firmware_version: info(device, Rs2CameraInfo::FirmwareVersion),
        connection_type: info(device, Rs2CameraInfo::UsbTypeDescriptor),
        physical_port: info(device, Rs2CameraInfo::PhysicalPort),
        sensors,
        profiles,
        driver_extensions: vec![CameraDriverExtensionInfo {
            namespace: DRIVER_NAMESPACE.into(),
            display_name: "Intel RealSense 驱动参数".into(),
            parameters,
        }],
        available: true,
    })
}

fn sensor_parameters(
    sensor_name: &str,
    sensor: &realsense_rust::sensor::Sensor,
) -> Vec<CameraDriverParameterInfo> {
    (0..=Rs2Option::Rotation as i32)
        .filter_map(Rs2Option::from_i32)
        .filter_map(|option| {
            let current = sensor.get_option(option)?;
            let range = sensor.get_option_range(option)?;
            let kind = if range.min == 0.0 && range.max == 1.0 && range.step == 1.0 {
                CameraDriverParameterKind::Boolean
            } else if range.step >= 1.0
                && [range.min, range.max, range.step, range.default]
                    .into_iter()
                    .all(|value| value.fract() == 0.0)
            {
                CameraDriverParameterKind::Integer
            } else {
                CameraDriverParameterKind::Number
            };
            Some(CameraDriverParameterInfo {
                key: parameter_key(sensor_name, option),
                display_name: option.to_string(),
                sensor_name: sensor_name.into(),
                kind,
                current_value: f64::from(current),
                default_value: f64::from(range.default),
                minimum: f64::from(range.min),
                maximum: f64::from(range.max),
                step: f64::from(range.step),
                read_only: sensor.is_option_read_only(option),
            })
        })
        .collect()
}

fn apply_driver_parameters(
    device: &Device,
    parameters: &[CameraDriverParameterValue],
) -> Result<()> {
    let mut sensors = device.sensors();
    for parameter in parameters {
        if parameter.namespace != DRIVER_NAMESPACE {
            bail!("RealSense 不支持驱动参数命名空间 {}", parameter.namespace);
        }
        let (sensor_name, option) = parse_parameter_key(&parameter.key)?;
        let matching = sensors
            .iter()
            .enumerate()
            .filter(|(_, sensor)| {
                sensor
                    .info(Rs2CameraInfo::Name)
                    .and_then(|value| value.to_str().ok())
                    == Some(sensor_name)
            })
            .map(|(index, _)| index)
            .collect::<Vec<_>>();
        let [sensor_index] = matching.as_slice() else {
            bail!("RealSense 传感器 {sensor_name} 当前不存在或名称不唯一");
        };
        sensors[*sensor_index]
            .set_option(option, parameter.value as f32)
            .map_err(|error| eyre::eyre!("设置 {} 失败: {error}", option.to_str()))?;
    }
    Ok(())
}

fn parameter_key(sensor_name: &str, option: Rs2Option) -> String {
    format!("sensor-name={sensor_name};option={}", option as i32)
}

fn parse_parameter_key(key: &str) -> Result<(&str, Rs2Option)> {
    let value = key
        .strip_prefix("sensor-name=")
        .ok_or_else(|| eyre::eyre!("RealSense 驱动参数键无效: {key}"))?;
    let (sensor_name, option_ordinal) = value
        .rsplit_once(";option=")
        .filter(|(sensor_name, _)| !sensor_name.is_empty())
        .ok_or_else(|| eyre::eyre!("RealSense 驱动参数键无效: {key}"))?;
    let option_ordinal = option_ordinal
        .parse::<i32>()
        .map_err(|_| eyre::eyre!("RealSense option 编号无效: {key}"))?;
    let option = Rs2Option::from_i32(option_ordinal)
        .ok_or_else(|| eyre::eyre!("RealSense option 不存在: {option_ordinal}"))?;
    Ok((sensor_name, option))
}

fn info(device: &Device, field: Rs2CameraInfo) -> Option<String> {
    device
        .info(field)
        .and_then(|value| value.to_str().ok())
        .map(str::to_owned)
}

fn profile_info(profile: &StreamProfile) -> Option<CameraStreamProfile> {
    let stream = match profile.kind() {
        Rs2StreamKind::Color => CameraStreamKind::Color,
        Rs2StreamKind::Depth => CameraStreamKind::Depth,
        _ => return None,
    };
    let intrinsics = profile.intrinsics();
    let (width, height) = intrinsics.as_ref().map_or((0, 0), |value| {
        (value.width() as u32, value.height() as u32)
    });
    let pixel_format = format_name(profile.format())
        .map(str::to_owned)
        .unwrap_or_else(|| format!("rs2-format-{}", profile.format() as i32));
    let unavailable_reason = match stream {
        _ if intrinsics.is_err() => Some("驱动未提供视频内参".into()),
        CameraStreamKind::Color
            if !matches!(
                pixel_format.as_str(),
                "rgb8" | "bgr8" | "rgba8" | "bgra8" | "y8"
            ) =>
        {
            Some(format!("感知链路当前不解码 {pixel_format}"))
        }
        CameraStreamKind::Depth if pixel_format != "z16le" => {
            Some(format!("感知链路当前只接受 z16le，不接受 {pixel_format}"))
        }
        _ => None,
    };
    let fps = profile.framerate() as u32;
    let prefix = match stream {
        CameraStreamKind::Color => "color",
        CameraStreamKind::Depth => "depth",
    };
    Some(CameraStreamProfile {
        key: profile_key(prefix, width, height, &pixel_format, fps),
        stream,
        width,
        height,
        frames_per_second: fps,
        pixel_format,
        is_default: profile.is_default(),
        available: unavailable_reason.is_none(),
        unavailable_reason,
    })
}

fn profile_key(prefix: &str, width: u32, height: u32, pixel_format: &str, fps: u32) -> String {
    format!("{prefix}:{width}x{height}:{pixel_format}:{fps}")
}

fn enable_profile(config: &mut Config, profile: &CameraStreamProfile) -> Result<()> {
    let kind = match profile.stream {
        CameraStreamKind::Color => Rs2StreamKind::Color,
        CameraStreamKind::Depth => Rs2StreamKind::Depth,
    };
    let format = parse_format(&profile.pixel_format)?;
    config.enable_stream(
        kind,
        None,
        profile.width as usize,
        profile.height as usize,
        format,
        profile.frames_per_second as usize,
    )?;
    Ok(())
}

fn intrinsics(profile: &StreamProfile) -> Result<CameraIntrinsics> {
    let value = profile.intrinsics()?;
    let distortion = value.distortion();
    Ok(CameraIntrinsics {
        width: value.width() as u32,
        height: value.height() as u32,
        focal_length_px: [f64::from(value.fx()), f64::from(value.fy())],
        principal_point_px: [f64::from(value.ppx()), f64::from(value.ppy())],
        distortion_model: distortion_name(distortion.model).into(),
        distortion: distortion.coeffs.into_iter().map(f64::from).collect(),
    })
}

fn distortion_name(model: Rs2DistortionModel) -> &'static str {
    match model {
        Rs2DistortionModel::None => "none",
        Rs2DistortionModel::BrownConrady => "brown_conrady",
        Rs2DistortionModel::BrownConradyModified => "modified_brown_conrady",
        Rs2DistortionModel::BrownConradyInverse => "inverse_brown_conrady",
        Rs2DistortionModel::FThetaFisheye => "ftheta",
        Rs2DistortionModel::KannalaBrandt => "kannala_brandt4",
    }
}

fn image_plane<K>(
    frame: &realsense_rust::frame::ImageFrame<K>,
    frame_id: &str,
) -> Result<CameraImagePlane> {
    let size = frame.get_data_size();
    let data = unsafe { slice::from_raw_parts(frame.get_data() as *const _ as *const u8, size) };
    let pixel_format = format_name(frame.stream_profile().format())
        .ok_or_else(|| eyre::eyre!("不支持的 RealSense 像素格式"))?;
    Ok(CameraImagePlane {
        width: frame.width() as u32,
        height: frame.height() as u32,
        stride_bytes: frame.stride() as u32,
        pixel_format: pixel_format.into(),
        frame_id: frame_id.into(),
        data: data.to_vec(),
    })
}

fn format_name(value: Rs2Format) -> Option<&'static str> {
    Some(match value {
        Rs2Format::Rgb8 => "rgb8",
        Rs2Format::Bgr8 => "bgr8",
        Rs2Format::Rgba8 => "rgba8",
        Rs2Format::Bgra8 => "bgra8",
        Rs2Format::Yuyv => "yuyv",
        Rs2Format::Uyvy => "uyvy",
        Rs2Format::Mjpeg => "mjpeg",
        Rs2Format::Y8 => "y8",
        Rs2Format::Y16 => "y16le",
        Rs2Format::Z16 => "z16le",
        _ => return None,
    })
}

fn parse_format(value: &str) -> Result<Rs2Format> {
    Ok(match value {
        "rgb8" => Rs2Format::Rgb8,
        "bgr8" => Rs2Format::Bgr8,
        "rgba8" => Rs2Format::Rgba8,
        "bgra8" => Rs2Format::Bgra8,
        "yuyv" => Rs2Format::Yuyv,
        "uyvy" => Rs2Format::Uyvy,
        "mjpeg" => Rs2Format::Mjpeg,
        "y8" => Rs2Format::Y8,
        "y16le" => Rs2Format::Y16,
        "z16le" => Rs2Format::Z16,
        other => bail!("不支持的 RealSense 像素格式 {other}"),
    })
}

fn now_ns() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as i64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pixel_formats_have_an_unambiguous_round_trip() {
        for name in ["rgb8", "bgr8", "rgba8", "bgra8", "y8", "z16le"] {
            assert_eq!(format_name(parse_format(name).unwrap()).unwrap(), name);
        }
    }

    #[test]
    fn unknown_pixel_format_is_rejected() {
        assert!(parse_format("vendor-private").is_err());
    }

    #[test]
    fn driver_parameter_keys_round_trip() {
        let (sensor, option) = parse_parameter_key("sensor-name=Stereo Module;option=3").unwrap();
        assert_eq!(sensor, "Stereo Module");
        assert_eq!(option, Rs2Option::Exposure);
    }

    #[test]
    fn persisted_profile_key_contains_only_semantic_stream_properties() {
        assert_eq!(
            profile_key("depth", 1280, 720, "z16le", 30),
            "depth:1280x720:z16le:30"
        );
    }

    #[test]
    fn malformed_driver_parameter_keys_are_rejected() {
        assert!(parse_parameter_key("sensor/exposure").is_err());
        assert!(parse_parameter_key("sensor-name=Stereo Module;option=9999").is_err());
        assert!(parse_parameter_key("sensor-name=;option=3").is_err());
    }
}
