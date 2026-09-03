mod drivers;

use std::{
    collections::BTreeMap,
    path::PathBuf,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result, eyre};
use json_config_store::{load_or_default, save};
use robot_arm_messages::{
    CameraCaptureState, CameraDriverParameterValue, CameraRequest, CameraSourceConfiguration,
    CameraSourceInfo, CameraStreamKind, MotionState, RequestAction, RequestResult, SCHEMA_VERSION,
    ServiceState, camera_frame_to_arrow, from_arrow, to_arrow,
};
use serde::{Deserialize, Serialize};

use crate::drivers::{CameraStream, Drivers};

const CONFIG_SCHEMA_VERSION: u32 = 1;

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct SavedProfiles {
    color_profile_key: String,
    depth_profile_key: String,
    #[serde(default)]
    output_frames_per_second: Option<f64>,
    #[serde(default)]
    driver_parameters: Vec<CameraDriverParameterValue>,
    #[serde(default)]
    source_snapshot: Option<CameraSourceInfo>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct CaptureConfig {
    schema_version: u32,
    config_version: u64,
    profiles: BTreeMap<String, SavedProfiles>,
}

impl Default for CaptureConfig {
    fn default() -> Self {
        Self {
            schema_version: CONFIG_SCHEMA_VERSION,
            config_version: 0,
            profiles: BTreeMap::new(),
        }
    }
}

struct CaptureNode {
    config_path: PathBuf,
    config: CaptureConfig,
    drivers: Drivers,
    sources: Vec<CameraSourceInfo>,
    selected_source_id: Option<String>,
    selected_color_profile_key: Option<String>,
    selected_depth_profile_key: Option<String>,
    selected_capture_frames_per_second: Option<f64>,
    output_frames_per_second: Option<f64>,
    selected_driver_parameters: Vec<CameraDriverParameterValue>,
    stream: Option<Box<dyn CameraStream>>,
    last_sequence: Option<u64>,
    last_frame_time_ns: Option<i64>,
    frame_window_started: Instant,
    frame_window_count: u64,
    measured_frames_per_second: Option<f64>,
    last_output_at: Option<Instant>,
    output_window_count: u64,
    measured_output_frames_per_second: Option<f64>,
    dropped_frame_count: u64,
    skipped_output_frame_count: u64,
    original_error: Option<String>,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("camera-capture-node: {error:?}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let config_path = std::env::var_os("CAMERA_CAPTURE_CONFIG")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/config/camera-capture.json"));
    let config = load_or_default(&config_path)?;
    let mut capture = CaptureNode {
        config_path,
        config,
        drivers: Drivers::new()?,
        sources: vec![],
        selected_source_id: None,
        selected_color_profile_key: None,
        selected_depth_profile_key: None,
        selected_capture_frames_per_second: None,
        output_frames_per_second: None,
        selected_driver_parameters: vec![],
        stream: None,
        last_sequence: None,
        last_frame_time_ns: None,
        frame_window_started: Instant::now(),
        frame_window_count: 0,
        measured_frames_per_second: None,
        last_output_at: None,
        output_window_count: 0,
        measured_output_frames_per_second: None,
        dropped_frame_count: 0,
        skipped_output_frame_count: 0,
        original_error: None,
    };
    // Persisted sources remain visible before the first explicit hardware
    // refresh. They are unavailable until a driver confirms them.
    merge_saved_sources(&mut capture.sources, &capture.config);
    let (mut node, mut events) = DoraNode::init_from_env()?;
    capture.publish_state(&mut node)?;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => match id.as_str() {
                "tick" => capture.tick(&mut node)?,
                "snapshot" => capture.publish_state(&mut node)?,
                "request" => {
                    let request = from_arrow(data.as_array()).context("解析相机请求")?;
                    capture.apply_request(&mut node, request)?;
                }
                "motion_state" => {
                    let state: MotionState = from_arrow(data.as_array())?;
                    if let Some(pose) = state.current_tool_pose {
                        capture.drivers.update_tool_pose(pose);
                    }
                }
                _ => {}
            },
            Event::Stop(_) => break,
            _ => {}
        }
    }
    capture.stream = None;
    Ok(())
}

impl CaptureNode {
    fn tick(&mut self, node: &mut DoraNode) -> Result<()> {
        let Some(stream) = self.stream.as_mut() else {
            return Ok(());
        };
        match stream.next_frameset() {
            Ok(Some(frame)) => {
                if let Some(previous) = self.last_sequence
                    && frame.sequence > previous + 1
                {
                    self.dropped_frame_count += frame.sequence - previous - 1;
                }
                self.last_sequence = Some(frame.sequence);
                self.last_frame_time_ns = Some(frame.received_time_ns);
                self.frame_window_count += 1;
                let should_publish = output_due(
                    self.last_output_at.map(|last| last.elapsed()),
                    self.output_frames_per_second.expect("selected output FPS"),
                    self.selected_capture_frames_per_second
                        .expect("selected capture FPS"),
                );
                if should_publish {
                    node.send_output(
                        DataId::from("frame".to_owned()),
                        MetadataParameters::default(),
                        camera_frame_to_arrow(&frame)?,
                    )?;
                    self.last_output_at = Some(Instant::now());
                    self.output_window_count += 1;
                } else {
                    self.skipped_output_frame_count += 1;
                }
                let elapsed = self.frame_window_started.elapsed().as_secs_f64();
                if elapsed >= 1.0 {
                    self.measured_frames_per_second =
                        Some(self.frame_window_count as f64 / elapsed);
                    self.measured_output_frames_per_second =
                        Some(self.output_window_count as f64 / elapsed);
                    self.frame_window_started = Instant::now();
                    self.frame_window_count = 0;
                    self.output_window_count = 0;
                    self.publish_state(node)?;
                }
            }
            Ok(None) => {}
            Err(error) => {
                self.original_error = Some(format!("{error:#}"));
                self.stream = None;
                self.publish_state(node)?;
            }
        }
        Ok(())
    }

    fn apply_request(&mut self, node: &mut DoraNode, request: CameraRequest) -> Result<()> {
        let action = request.action;
        self.original_error = None;
        let result = self
            .handle_request(&request)
            .map_err(|error| format!("{error:#}"));
        if let Err(error) = &result {
            self.original_error = Some(error.clone());
        }
        let original_error = result.err();
        let response = RequestResult {
            schema_version: SCHEMA_VERSION,
            request_id: request.request_id,
            acknowledged_action: action,
            value: Some(self.state()),
            original_error,
        };
        send(node, "request_result", &response)?;
        self.publish_state(node)
    }

    fn handle_request(&mut self, request: &CameraRequest) -> Result<()> {
        match request.action {
            RequestAction::Refresh | RequestAction::Discover => self.refresh(),
            RequestAction::Select | RequestAction::Apply => self.select(request),
            RequestAction::Connect => self.start(),
            RequestAction::Disconnect => {
                self.stream = None;
                self.reset_stream_statistics();
                Ok(())
            }
            RequestAction::Unselect | RequestAction::Cancel => {
                self.stream = None;
                self.selected_source_id = None;
                self.selected_color_profile_key = None;
                self.selected_depth_profile_key = None;
                self.selected_capture_frames_per_second = None;
                self.output_frames_per_second = None;
                self.selected_driver_parameters.clear();
                self.reset_stream_statistics();
                Ok(())
            }
            RequestAction::Reset => self.reset(request.source_id.as_deref()),
            RequestAction::Snapshot => Ok(()),
        }
    }

    fn refresh(&mut self) -> Result<()> {
        let (mut sources, errors) = self.drivers.discover();
        merge_saved_sources(&mut sources, &self.config);
        self.sources = sources;
        self.original_error = (!errors.is_empty()).then(|| errors.join("；"));
        if let Some(selected) = self.selected_source_id.as_deref()
            && !self
                .sources
                .iter()
                .any(|source| source.source_id == selected && source.available)
        {
            self.stream = None;
        }
        Ok(())
    }

    fn select(&mut self, request: &CameraRequest) -> Result<()> {
        let source_id = request
            .source_id
            .as_deref()
            .ok_or_else(|| eyre!("请选择相机来源"))?;
        let source = self
            .sources
            .iter()
            .find(|source| source.source_id == source_id && source.available)
            .ok_or_else(|| eyre!("来源 {source_id} 不在最近一次刷新结果中"))?;
        let saved = self.config.profiles.get(source_id);
        let color_key = request
            .color_profile_key
            .as_deref()
            .or_else(|| {
                saved
                    .map(|value| value.color_profile_key.as_str())
                    .filter(|key| {
                        source.profiles.iter().any(|profile| {
                            profile.stream == CameraStreamKind::Color
                                && profile.key == *key
                                && profile.available
                        })
                    })
            })
            .or_else(|| {
                first_profile(source, CameraStreamKind::Color).map(|value| value.key.as_str())
            })
            .ok_or_else(|| eyre!("来源没有彩色 profile"))?;
        let depth_key = request
            .depth_profile_key
            .as_deref()
            .or_else(|| {
                saved
                    .map(|value| value.depth_profile_key.as_str())
                    .filter(|key| {
                        source.profiles.iter().any(|profile| {
                            profile.stream == CameraStreamKind::Depth
                                && profile.key == *key
                                && profile.available
                        })
                    })
            })
            .or_else(|| {
                first_profile(source, CameraStreamKind::Depth).map(|value| value.key.as_str())
            })
            .ok_or_else(|| eyre!("来源没有深度 profile"))?;
        let color_profile = source
            .profiles
            .iter()
            .find(|profile| profile.stream == CameraStreamKind::Color && profile.key == color_key)
            .ok_or_else(|| eyre!("所选彩色 profile 不是驱动最近报告的能力"))?;
        let depth_profile = source
            .profiles
            .iter()
            .find(|profile| profile.stream == CameraStreamKind::Depth && profile.key == depth_key)
            .ok_or_else(|| eyre!("所选深度 profile 不是驱动最近报告的能力"))?;
        if !color_profile.available {
            return Err(eyre!(
                "所选彩色 profile 当前不可用：{}",
                color_profile
                    .unavailable_reason
                    .as_deref()
                    .unwrap_or("未知原因")
            ));
        }
        if !depth_profile.available {
            return Err(eyre!(
                "所选深度 profile 当前不可用：{}",
                depth_profile
                    .unavailable_reason
                    .as_deref()
                    .unwrap_or("未知原因")
            ));
        }
        let maximum_output_fps = f64::from(
            color_profile
                .frames_per_second
                .min(depth_profile.frames_per_second),
        );
        let output_frames_per_second = request
            .output_frames_per_second
            .or_else(|| saved.and_then(|value| value.output_frames_per_second))
            .unwrap_or(maximum_output_fps);
        validate_output_rate(output_frames_per_second, maximum_output_fps)?;
        let mut driver_parameters =
            saved.map_or_else(Vec::new, |value| value.driver_parameters.clone());
        if let Some(changes) = &request.driver_parameters {
            for change in changes {
                if let Some(existing) = driver_parameters
                    .iter_mut()
                    .find(|value| value.namespace == change.namespace && value.key == change.key)
                {
                    *existing = change.clone();
                } else {
                    driver_parameters.push(change.clone());
                }
            }
        }
        validate_driver_parameters(source, &driver_parameters)?;
        let source_snapshot = source.clone();
        let source_id = source_id.to_owned();
        let color_key = color_key.to_owned();
        let depth_key = depth_key.to_owned();
        let mut config = self.config.clone();
        config.profiles.insert(
            source_id.clone(),
            SavedProfiles {
                color_profile_key: color_key.clone(),
                depth_profile_key: depth_key.clone(),
                output_frames_per_second: Some(output_frames_per_second),
                driver_parameters: driver_parameters.clone(),
                source_snapshot: Some(source_snapshot),
            },
        );
        config.config_version += 1;
        self.commit_config(config)?;
        self.stream = None;
        self.selected_source_id = Some(source_id);
        self.selected_color_profile_key = Some(color_key);
        self.selected_depth_profile_key = Some(depth_key);
        self.selected_capture_frames_per_second = Some(maximum_output_fps);
        self.output_frames_per_second = Some(output_frames_per_second);
        self.selected_driver_parameters = driver_parameters;
        self.reset_stream_statistics();
        Ok(())
    }

    fn start(&mut self) -> Result<()> {
        let source_id = self
            .selected_source_id
            .as_deref()
            .ok_or_else(|| eyre!("尚未选择相机"))?;
        let source = self
            .sources
            .iter()
            .find(|source| source.source_id == source_id)
            .ok_or_else(|| eyre!("所选相机不在最近一次刷新结果中"))?;
        let color = selected_profile(
            source,
            self.selected_color_profile_key.as_deref(),
            CameraStreamKind::Color,
        )?;
        let depth = selected_profile(
            source,
            self.selected_depth_profile_key.as_deref(),
            CameraStreamKind::Depth,
        )?;
        self.stream =
            Some(
                self.drivers
                    .open(source_id, color, depth, &self.selected_driver_parameters)?,
            );
        self.reset_stream_statistics();
        Ok(())
    }

    fn reset_stream_statistics(&mut self) {
        self.last_sequence = None;
        self.last_frame_time_ns = None;
        self.frame_window_started = Instant::now();
        self.frame_window_count = 0;
        self.measured_frames_per_second = None;
        self.last_output_at = None;
        self.output_window_count = 0;
        self.measured_output_frames_per_second = None;
        self.dropped_frame_count = 0;
        self.skipped_output_frame_count = 0;
    }

    fn commit_config(&mut self, config: CaptureConfig) -> Result<()> {
        save(&self.config_path, &config)?;
        self.config = config;
        Ok(())
    }

    fn reset(&mut self, source_id: Option<&str>) -> Result<()> {
        let source_id = source_id
            .or(self.selected_source_id.as_deref())
            .ok_or_else(|| eyre!("请选择需要重置的相机"))?
            .to_owned();
        self.stream = None;
        self.drivers.reset(&source_id)?;
        let mut config = self.config.clone();
        config.profiles.remove(&source_id);
        config.config_version += 1;
        self.commit_config(config)?;
        if self.selected_source_id.as_deref() == Some(source_id.as_str()) {
            self.selected_color_profile_key = None;
            self.selected_depth_profile_key = None;
            self.selected_capture_frames_per_second = None;
            self.output_frames_per_second = None;
            self.selected_driver_parameters.clear();
        }
        self.sources
            .retain(|source| source.source_id != source_id || source.available);
        Ok(())
    }

    fn state(&self) -> CameraCaptureState {
        CameraCaptureState {
            schema_version: SCHEMA_VERSION,
            available_sources: self.sources.clone(),
            selected_source_id: self.selected_source_id.clone(),
            selected_color_profile_key: self.selected_color_profile_key.clone(),
            selected_depth_profile_key: self.selected_depth_profile_key.clone(),
            output_frames_per_second: self.output_frames_per_second,
            configurations: self
                .config
                .profiles
                .iter()
                .filter_map(|(source_id, value)| {
                    Some(CameraSourceConfiguration {
                        source_id: source_id.clone(),
                        color_profile_key: value.color_profile_key.clone(),
                        depth_profile_key: value.depth_profile_key.clone(),
                        output_frames_per_second: value.output_frames_per_second?,
                        driver_parameters: value.driver_parameters.clone(),
                    })
                })
                .collect(),
            streaming: self.stream.is_some(),
            last_sequence: self.last_sequence,
            last_frame_time_ns: self.last_frame_time_ns,
            measured_frames_per_second: self.measured_frames_per_second,
            measured_output_frames_per_second: self.measured_output_frames_per_second,
            dropped_frame_count: self.dropped_frame_count,
            skipped_output_frame_count: self.skipped_output_frame_count,
            original_error: self.original_error.clone(),
            service: ServiceState {
                schema_version: SCHEMA_VERSION,
                build_version: env!("CARGO_PKG_VERSION").into(),
                config_version: self.config.config_version,
                running: true,
                has_input: self.last_frame_time_ns.is_some(),
                has_output: self.last_frame_time_ns.is_some(),
                last_error: self.original_error.clone(),
                updated_at_ns: now_ns(),
            },
        }
    }

    fn publish_state(&self, node: &mut DoraNode) -> Result<()> {
        let state = self.state();
        send(node, "camera_state", &state)?;
        send(node, "service_state", &state.service)
    }
}

fn first_profile(
    source: &CameraSourceInfo,
    kind: CameraStreamKind,
) -> Option<&robot_arm_messages::CameraStreamProfile> {
    source
        .profiles
        .iter()
        .find(|profile| profile.stream == kind && profile.is_default && profile.available)
        .or_else(|| {
            source
                .profiles
                .iter()
                .find(|profile| profile.stream == kind && profile.available)
        })
}

fn selected_profile<'a>(
    source: &'a CameraSourceInfo,
    key: Option<&str>,
    kind: CameraStreamKind,
) -> Result<&'a robot_arm_messages::CameraStreamProfile> {
    let key = key.ok_or_else(|| eyre!("尚未选择 {:?} profile", kind))?;
    source
        .profiles
        .iter()
        .find(|profile| profile.stream == kind && profile.key == key && profile.available)
        .ok_or_else(|| eyre!("profile {key} 当前不可用"))
}

fn merge_saved_sources(sources: &mut Vec<CameraSourceInfo>, config: &CaptureConfig) {
    for (source_id, saved) in &config.profiles {
        let Some(snapshot) = &saved.source_snapshot else {
            continue;
        };
        if let Some(current) = sources
            .iter_mut()
            .find(|source| source.source_id == *source_id)
        {
            for profile in &snapshot.profiles {
                if !current
                    .profiles
                    .iter()
                    .any(|value| value.key == profile.key)
                {
                    let mut unavailable = profile.clone();
                    unavailable.available = false;
                    unavailable.unavailable_reason = Some("驱动本次未报告该配置".into());
                    current.profiles.push(unavailable);
                }
            }
            current
                .profiles
                .sort_by(|left, right| left.key.cmp(&right.key));
        } else {
            let mut unavailable = snapshot.clone();
            unavailable.available = false;
            for profile in &mut unavailable.profiles {
                profile.available = false;
                profile.unavailable_reason = Some("设备当前未连接".into());
            }
            sources.push(unavailable);
        }
    }
    sources.sort_by(|left, right| left.display_name.cmp(&right.display_name));
}

fn validate_driver_parameters(
    source: &CameraSourceInfo,
    values: &[CameraDriverParameterValue],
) -> Result<()> {
    let mut seen = std::collections::BTreeSet::new();
    for value in values {
        if !seen.insert((&value.namespace, &value.key)) {
            return Err(eyre!("驱动参数 {} / {} 重复", value.namespace, value.key));
        }
        let parameter = source
            .driver_extensions
            .iter()
            .find(|extension| extension.namespace == value.namespace)
            .and_then(|extension| {
                extension
                    .parameters
                    .iter()
                    .find(|parameter| parameter.key == value.key)
            })
            .ok_or_else(|| eyre!("驱动最近没有报告参数 {} / {}", value.namespace, value.key))?;
        if parameter.read_only {
            return Err(eyre!("驱动参数 {} 是只读信息", parameter.display_name));
        }
        if !value.value.is_finite()
            || value.value < parameter.minimum
            || value.value > parameter.maximum
        {
            return Err(eyre!(
                "驱动参数 {} 必须位于 {} 到 {}",
                parameter.display_name,
                parameter.minimum,
                parameter.maximum
            ));
        }
    }
    Ok(())
}

fn validate_output_rate(value: f64, maximum: f64) -> Result<()> {
    if !value.is_finite() || value <= 0.0 || value > maximum {
        return Err(eyre!(
            "上送频率必须大于 0 且不超过当前两路 profile 的共同采样频率 {maximum} FPS"
        ));
    }
    Ok(())
}

fn output_due(
    elapsed_since_output: Option<Duration>,
    output_frames_per_second: f64,
    capture_frames_per_second: f64,
) -> bool {
    elapsed_since_output.is_none_or(|elapsed| {
        let output_period = Duration::from_secs_f64(1.0 / output_frames_per_second);
        let sampling_tolerance = Duration::from_secs_f64(0.5 / capture_frames_per_second);
        elapsed.saturating_add(sampling_tolerance) >= output_period
    })
}

fn send<T: Serialize>(node: &mut DoraNode, output: &str, value: &T) -> Result<()> {
    node.send_output(
        DataId::from(output.to_owned()),
        MetadataParameters::default(),
        to_arrow(value)?,
    )?;
    Ok(())
}

fn now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as i64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_state_does_not_select_or_start_a_camera() {
        let state = CameraCaptureState {
            schema_version: SCHEMA_VERSION,
            available_sources: vec![],
            selected_source_id: None,
            selected_color_profile_key: None,
            selected_depth_profile_key: None,
            output_frames_per_second: None,
            configurations: vec![],
            streaming: false,
            last_sequence: None,
            last_frame_time_ns: None,
            measured_frames_per_second: None,
            measured_output_frames_per_second: None,
            dropped_frame_count: 0,
            skipped_output_frame_count: 0,
            original_error: None,
            service: ServiceState::default(),
        };
        assert!(state.selected_source_id.is_none());
        assert!(!state.streaming);
    }

    #[test]
    fn config_does_not_persist_runtime_selection() {
        let encoded = serde_json::to_string(&CaptureConfig::default()).unwrap();
        assert!(!encoded.contains("selected_source"));
    }

    #[test]
    fn driver_parameters_must_come_from_reported_capabilities() {
        let source = CameraSourceInfo::default();
        let result = validate_driver_parameters(
            &source,
            &[CameraDriverParameterValue {
                namespace: "librealsense2".into(),
                key: "not-a-reported-key".into(),
                value: 100.0,
            }],
        );
        assert!(result.is_err());
    }

    #[test]
    fn output_rate_may_be_lower_than_capture_but_not_higher() {
        assert!(validate_output_rate(1.0, 60.0).is_ok());
        assert!(validate_output_rate(60.0, 60.0).is_ok());
        assert!(validate_output_rate(0.0, 60.0).is_err());
        assert!(validate_output_rate(60.1, 60.0).is_err());
        assert!(validate_output_rate(f64::NAN, 60.0).is_err());
    }

    #[test]
    fn output_scheduler_selects_the_frame_nearest_each_requested_period() {
        assert!(output_due(None, 1.0, 60.0));
        assert!(output_due(Some(Duration::from_millis(99)), 10.0, 10.0));
        assert!(!output_due(Some(Duration::from_millis(950)), 1.0, 60.0));
        assert!(output_due(Some(Duration::from_millis(995)), 1.0, 60.0));
    }

    #[test]
    fn saved_camera_and_profile_remain_visible_when_temporarily_unavailable() {
        let profile = robot_arm_messages::CameraStreamProfile {
            key: "depth:640x480:z16le:30".into(),
            stream: CameraStreamKind::Depth,
            width: 640,
            height: 480,
            frames_per_second: 30,
            pixel_format: "z16le".into(),
            is_default: true,
            available: true,
            unavailable_reason: None,
        };
        let snapshot = CameraSourceInfo {
            source_id: "realsense:stable-serial".into(),
            display_name: "Intel RealSense D415 · stable-serial".into(),
            profiles: vec![profile],
            available: true,
            ..Default::default()
        };
        let config = CaptureConfig {
            schema_version: CONFIG_SCHEMA_VERSION,
            config_version: 1,
            profiles: BTreeMap::from([(
                snapshot.source_id.clone(),
                SavedProfiles {
                    color_profile_key: "color:640x480:rgb8:30".into(),
                    depth_profile_key: "depth:640x480:z16le:30".into(),
                    output_frames_per_second: Some(1.0),
                    driver_parameters: vec![],
                    source_snapshot: Some(snapshot),
                },
            )]),
        };
        let mut sources = vec![];
        merge_saved_sources(&mut sources, &config);
        assert_eq!(sources[0].source_id, "realsense:stable-serial");
        assert!(!sources[0].available);
        assert!(!sources[0].profiles[0].available);
        assert_eq!(
            sources[0].profiles[0].unavailable_reason.as_deref(),
            Some("设备当前未连接")
        );
    }
}
