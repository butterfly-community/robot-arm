mod capture_worker;
mod drivers;
mod video_server;

use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result, bail, eyre};
use futures::FutureExt;
#[cfg(feature = "opencv-runtime")]
use image::{DynamicImage, ImageFormat, RgbImage};
use json_config_store::{load_or_default, save};
#[cfg(feature = "opencv-runtime")]
use robot_arm_messages::Pose3;
use robot_arm_messages::{
    ArmState, CalibrationAction, CalibrationObservation, CalibrationPhase, CalibrationRequest,
    CalibrationResult, CalibrationSessionState, CameraCaptureState, CameraDriverParameterValue,
    CameraFrameBundle, CameraRequest, CameraSourceConfiguration, CameraSourceInfo,
    CameraStreamKind, ControlMode, DepthCameraCalibration, JointPosition, MotionRequest,
    MotionState, NamedMotionTarget, PerceptionAssetRequest, PerceptionAssetResponse, RequestAction,
    RequestResult, RequestState, RobotModelInfo, SCHEMA_VERSION, ServiceState,
    SetControlModeRequest, ToolPose, camera_frame_to_arrow, from_arrow, to_arrow,
};
use serde::{Deserialize, Serialize};
#[cfg(feature = "opencv-runtime")]
use std::io::Cursor;

use crate::capture_worker::Command as CaptureCommand;
use crate::capture_worker::{Completion as CaptureCompletion, Output as CaptureOutput};

const CONFIG_SCHEMA_VERSION: u32 = 2;
const CALIBRATION_CAPTURE_DELAY: Duration = Duration::from_secs(10);

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
struct CameraConfigStore {
    schema_version: u32,
    config_version: u64,
    profiles: BTreeMap<String, SavedProfiles>,
    #[serde(default)]
    cameras: BTreeMap<String, SavedCalibration>,
    #[serde(skip, default = "default_calibration_session")]
    calibration_session: CalibrationSessionState,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct SavedCalibration {
    calibration: Option<CalibrationResult>,
}

impl Default for CameraConfigStore {
    fn default() -> Self {
        Self {
            schema_version: CONFIG_SCHEMA_VERSION,
            config_version: 0,
            profiles: BTreeMap::new(),
            cameras: BTreeMap::new(),
            calibration_session: default_calibration_session(),
        }
    }
}

struct CameraNode {
    runtime: tokio::runtime::Handle,
    calibration_work: Option<tokio::task::JoinHandle<Result<CalibrationWork>>>,
    config_path: PathBuf,
    config: CameraConfigStore,
    sources: Vec<CameraSourceInfo>,
    selected_source_id: Option<String>,
    selected_color_profile_key: Option<String>,
    selected_depth_profile_key: Option<String>,
    selected_capture_frames_per_second: Option<f64>,
    output_frames_per_second: Option<f64>,
    selected_driver_parameters: Vec<CameraDriverParameterValue>,
    capture_commands: tokio::sync::mpsc::Sender<CaptureCommand>,
    capture_completions: tokio::sync::mpsc::Receiver<CaptureCompletion>,
    capture_outputs: tokio::sync::watch::Receiver<Option<CaptureOutput>>,
    desired_streaming: bool,
    pending_open_request_id: Option<String>,
    streaming: bool,
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
    worker_capture_count: u64,
    original_error: Option<String>,
    latest_frame: Option<CameraFrameBundle>,
    latest_arm_state: Option<ArmState>,
    latest_tool_pose: Option<ToolPose>,
    latest_motion_state: Option<MotionState>,
    robot_model: Option<RobotModelInfo>,
    calibration_capture_at: Option<Instant>,
    last_calibration_attempt_ns: Option<i64>,
    calibration_preview: Option<Vec<u8>>,
}

enum CalibrationWork {
    Detected(CalibrationObservation, Vec<u8>),
    Solved(CalibrationResult),
}

fn default_calibration_session() -> CalibrationSessionState {
    CalibrationSessionState {
        schema_version: SCHEMA_VERSION,
        ..Default::default()
    }
}

fn main() {
    if let Err(error) = run() {
        eprintln!("camera-node: {error:?}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .context("创建 camera-node Tokio runtime")?;
    let capture = capture_worker::spawn(&runtime);
    let (video_shutdown, video_shutdown_receiver) = tokio::sync::watch::channel(false);
    let video_listener = runtime.block_on(video_server::bind())?;
    let _video_server = runtime.spawn(video_server::serve(
        video_listener,
        capture.video,
        video_shutdown_receiver,
    ));
    let config_path = std::env::var_os("CAMERA_CONFIG")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/config/camera.json"));
    let config = load_config(&config_path)?;
    let mut camera = CameraNode {
        runtime: runtime.handle().clone(),
        calibration_work: None,
        config_path,
        config,
        sources: vec![],
        selected_source_id: None,
        selected_color_profile_key: None,
        selected_depth_profile_key: None,
        selected_capture_frames_per_second: None,
        output_frames_per_second: None,
        selected_driver_parameters: vec![],
        capture_commands: capture.commands,
        capture_completions: capture.completions,
        capture_outputs: capture.outputs,
        desired_streaming: false,
        pending_open_request_id: None,
        streaming: false,
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
        worker_capture_count: 0,
        original_error: None,
        latest_frame: None,
        latest_arm_state: None,
        latest_tool_pose: None,
        latest_motion_state: None,
        robot_model: None,
        calibration_capture_at: None,
        last_calibration_attempt_ns: None,
        calibration_preview: None,
    };
    camera.set_simulation_calibration_active(camera.config.calibration_session.active);
    // Persisted sources remain visible before the first explicit hardware
    // refresh. They are unavailable until a driver confirms them.
    merge_saved_sources(&mut camera.sources, &camera.config);
    let (mut node, mut events) = DoraNode::init_from_env()?;
    camera.publish_state(&mut node)?;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => match id.as_str() {
                "tick" => camera.tick(&mut node)?,
                "snapshot" => camera.publish_state(&mut node)?,
                "request" => {
                    let request = from_arrow(data.as_array()).context("解析相机请求")?;
                    camera.apply_request(&mut node, request)?;
                }
                "calibration_request" => {
                    let request = from_arrow(data.as_array()).context("解析相机标定请求")?;
                    camera.apply_calibration_request(&mut node, request)?;
                }
                "asset_request" => {
                    let request = from_arrow(data.as_array()).context("解析相机资源请求")?;
                    camera.send_asset(&mut node, request)?;
                }
                "arm_state" => camera.latest_arm_state = Some(from_arrow(data.as_array())?),
                "motion_state" => {
                    let state: MotionState = from_arrow(data.as_array())?;
                    if let Some(pose) = state.current_tool_pose.clone() {
                        camera.update_tool_pose(pose.clone())?;
                        camera.latest_tool_pose = Some(pose);
                    }
                    camera.latest_motion_state = Some(state);
                }
                "calibration_mode_result" => {
                    let result: RequestResult<MotionState> = from_arrow(data.as_array())?;
                    if camera.config.calibration_session.phase == CalibrationPhase::Preparing
                        && camera
                            .config
                            .calibration_session
                            .run_id
                            .as_ref()
                            .is_some_and(|run| result.request_id == format!("{run}-manual-mode"))
                    {
                        if let Some(error) = result.original_error {
                            camera.fail_automatic_calibration(error);
                        } else if let Err(error) = camera.send_current_calibration_target(&mut node)
                        {
                            camera.fail_automatic_calibration(error.to_string());
                        }
                    }
                }
                "robot_model_info" => camera.robot_model = Some(from_arrow(data.as_array())?),
                _ => {}
            },
            Event::Stop(_) => break,
            _ => {}
        }
    }
    let _ = camera.capture_commands.try_send(CaptureCommand::Shutdown);
    video_shutdown.send_replace(true);
    Ok(())
}

impl CameraNode {
    fn tick(&mut self, node: &mut DoraNode) -> Result<()> {
        while let Ok(completion) = self.capture_completions.try_recv() {
            self.finish_capture_command(node, completion)?;
        }
        if let Err(error) = self.advance_automatic_calibration(node) {
            self.fail_automatic_calibration(error.to_string());
        }
        if let Err(error) = self
            .poll_calibration_work(node)
            .and_then(|()| self.try_automatic_calibration_capture())
        {
            self.fail_automatic_calibration(error.to_string());
        }
        if self.capture_outputs.has_changed().unwrap_or(false) {
            let output = self.capture_outputs.borrow_and_update().clone();
            if let Some(output) = output {
                match output {
                    CaptureOutput::Capture {
                        sequence,
                        received_time_ns,
                        frame,
                        captured_count,
                        skipped_count,
                    } => {
                        if !self.streaming
                            || self.selected_source_id.as_deref() != Some(frame.source_id.as_str())
                        {
                            return self.publish_state(node);
                        }
                        let captured_since_update =
                            captured_count.saturating_sub(self.worker_capture_count);
                        if let Some(previous) = self.last_sequence {
                            let sequence_advance = sequence.saturating_sub(previous);
                            self.dropped_frame_count +=
                                sequence_advance.saturating_sub(captured_since_update);
                        }
                        self.last_sequence = Some(sequence);
                        self.worker_capture_count = captured_count;
                        self.last_frame_time_ns = Some(received_time_ns);
                        self.frame_window_count += captured_since_update;
                        self.skipped_output_frame_count = skipped_count;
                        let mut frame = *frame;
                        frame.calibration = self.calibration_for_frame(&frame)?;
                        self.latest_frame = Some(frame.clone());
                        node.send_output(
                            DataId::from("frame".to_owned()),
                            MetadataParameters::default(),
                            camera_frame_to_arrow(&frame)?,
                        )?;
                        self.last_output_at = Some(Instant::now());
                        self.output_window_count += 1;
                    }
                    CaptureOutput::Failed(error) => {
                        if self.pending_open_request_id.is_some() {
                            return self.publish_state(node);
                        }
                        if self.config.calibration_session.active {
                            self.fail_automatic_calibration(error.clone());
                        }
                        self.original_error = Some(error);
                        self.desired_streaming = false;
                        self.streaming = false;
                        self.latest_frame = None;
                    }
                }
            }
        }
        let elapsed = self.frame_window_started.elapsed().as_secs_f64();
        if elapsed >= 1.0 {
            self.measured_frames_per_second = Some(self.frame_window_count as f64 / elapsed);
            self.measured_output_frames_per_second =
                Some(self.output_window_count as f64 / elapsed);
            self.frame_window_started = Instant::now();
            self.frame_window_count = 0;
            self.output_window_count = 0;
        }
        self.publish_state(node)?;
        Ok(())
    }

    fn finish_capture_command(
        &mut self,
        node: &mut DoraNode,
        completion: CaptureCompletion,
    ) -> Result<()> {
        let (request_id, action, result) = match completion {
            CaptureCompletion::Discovered {
                request_id,
                action,
                mut sources,
                errors,
            } => {
                merge_saved_sources(&mut sources, &self.config);
                self.sources = sources;
                let mut result = if errors.is_empty() {
                    Ok(())
                } else {
                    Err(errors.join("；"))
                };
                if let Some(selected) = self.selected_source_id.as_deref()
                    && !self
                        .sources
                        .iter()
                        .any(|source| source.source_id == selected && source.available)
                    && let Err(error) = self.stop_capture()
                {
                    result = Err(format!("{error:#}"));
                }
                (request_id, action, result)
            }
            CaptureCompletion::Opened {
                request_id,
                action,
                source_id,
                result,
            } => {
                let current = self.pending_open_request_id.as_deref() == Some(request_id.as_str());
                let result = if !current {
                    Err("相机启动请求已被后续操作替换".into())
                } else {
                    result.and_then(|()| {
                        if self.desired_streaming
                            && self.selected_source_id.as_deref() == Some(source_id.as_str())
                        {
                            self.streaming = true;
                            self.reset_stream_statistics();
                            Ok(())
                        } else {
                            Err("相机启动期间选择或运行状态已改变".into())
                        }
                    })
                };
                if current {
                    self.pending_open_request_id = None;
                }
                if current && result.is_err() {
                    self.desired_streaming = false;
                    self.streaming = false;
                }
                (request_id, action, result)
            }
            CaptureCompletion::Reset {
                request_id,
                action,
                source_id,
                result,
            } => {
                let result = result.and_then(|()| {
                    self.apply_reset(&source_id)
                        .map_err(|error| format!("{error:#}"))
                });
                (request_id, action, result)
            }
        };
        let original_error = result.err();
        self.original_error = original_error.clone();
        send(
            node,
            "request_result",
            &RequestResult {
                schema_version: SCHEMA_VERSION,
                request_id,
                acknowledged_action: action,
                value: Some(self.state()),
                original_error,
            },
        )
    }

    fn apply_request(&mut self, node: &mut DoraNode, request: CameraRequest) -> Result<()> {
        let action = request.action;
        self.original_error = None;
        let asynchronous = match action {
            RequestAction::Refresh | RequestAction::Discover => Some(self.refresh(&request)),
            RequestAction::Connect => Some(self.start(&request)),
            RequestAction::Reset => Some(self.reset(&request)),
            _ => None,
        };
        if let Some(result) = asynchronous {
            if let Err(error) = result {
                let error = format!("{error:#}");
                self.original_error = Some(error.clone());
                send(
                    node,
                    "request_result",
                    &RequestResult {
                        schema_version: SCHEMA_VERSION,
                        request_id: request.request_id,
                        acknowledged_action: action,
                        value: Some(self.state()),
                        original_error: Some(error),
                    },
                )?;
            }
            return self.publish_state(node);
        }
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
            RequestAction::Refresh
            | RequestAction::Discover
            | RequestAction::Connect
            | RequestAction::Reset => unreachable!("asynchronous request handled before dispatch"),
            RequestAction::Select | RequestAction::Apply => self.select(request),
            RequestAction::Disconnect => {
                self.stop_capture()?;
                self.reset_stream_statistics();
                Ok(())
            }
            RequestAction::Unselect | RequestAction::Cancel => {
                self.stop_capture()?;
                self.selected_source_id = None;
                self.selected_color_profile_key = None;
                self.selected_depth_profile_key = None;
                self.selected_capture_frames_per_second = None;
                self.output_frames_per_second = None;
                self.selected_driver_parameters.clear();
                self.reset_stream_statistics();
                Ok(())
            }
            RequestAction::Snapshot => Ok(()),
        }
    }

    fn refresh(&mut self, request: &CameraRequest) -> Result<()> {
        self.capture_commands
            .try_send(CaptureCommand::Discover {
                request_id: request.request_id.clone(),
                action: request.action,
            })
            .map_err(capture_command_error)
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
        self.stop_capture()?;
        self.selected_source_id = Some(source_id);
        self.selected_color_profile_key = Some(color_key);
        self.selected_depth_profile_key = Some(depth_key);
        self.selected_capture_frames_per_second = Some(maximum_output_fps);
        self.output_frames_per_second = Some(output_frames_per_second);
        self.selected_driver_parameters = driver_parameters;
        self.reset_stream_statistics();
        Ok(())
    }

    fn start(&mut self, request: &CameraRequest) -> Result<()> {
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
        self.capture_commands
            .try_send(CaptureCommand::Open {
                request_id: request.request_id.clone(),
                action: request.action,
                source_id: source_id.to_owned(),
                color: color.clone(),
                depth: depth.clone(),
                driver_parameters: self.selected_driver_parameters.clone(),
                output_frames_per_second: self
                    .output_frames_per_second
                    .expect("selected output FPS"),
                capture_frames_per_second: self
                    .selected_capture_frames_per_second
                    .expect("selected capture FPS"),
            })
            .map_err(capture_command_error)?;
        self.desired_streaming = true;
        self.streaming = false;
        self.latest_frame = None;
        self.pending_open_request_id = Some(request.request_id.clone());
        Ok(())
    }

    fn stop_capture(&mut self) -> Result<()> {
        self.latest_frame = None;
        self.pending_open_request_id = None;
        if !self.streaming && !self.desired_streaming {
            return Ok(());
        }
        self.capture_commands
            .try_send(CaptureCommand::Stop)
            .map_err(capture_command_error)?;
        self.desired_streaming = false;
        self.streaming = false;
        if self.config.calibration_session.active {
            self.fail_automatic_calibration("相机采集已停止，自动标定已结束".into());
        }
        Ok(())
    }

    fn update_tool_pose(&self, pose: ToolPose) -> Result<()> {
        match self
            .capture_commands
            .try_send(CaptureCommand::UpdateToolPose(pose))
        {
            Ok(()) | Err(tokio::sync::mpsc::error::TrySendError::Full(_)) => Ok(()),
            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => {
                Err(eyre!("相机采集任务已经停止"))
            }
        }
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
        self.worker_capture_count = 0;
    }

    fn commit_config(&mut self, config: CameraConfigStore) -> Result<()> {
        save(&self.config_path, &config)?;
        self.config = config;
        Ok(())
    }

    fn reset(&mut self, request: &CameraRequest) -> Result<()> {
        let source_id = request
            .source_id
            .as_deref()
            .or(self.selected_source_id.as_deref())
            .ok_or_else(|| eyre!("请选择需要重置的相机"))?
            .to_owned();
        self.stop_capture()?;
        self.capture_commands
            .try_send(CaptureCommand::Reset {
                request_id: request.request_id.clone(),
                action: request.action,
                source_id,
            })
            .map_err(capture_command_error)
    }

    fn apply_reset(&mut self, source_id: &str) -> Result<()> {
        let mut config = self.config.clone();
        config.profiles.remove(source_id);
        config.cameras.remove(source_id);
        if config.calibration_session.camera_source_id.as_deref() == Some(source_id) {
            config.calibration_session = default_calibration_session();
        }
        config.config_version += 1;
        self.commit_config(config)?;
        if self.selected_source_id.as_deref() == Some(source_id) {
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

    fn calibration_for_frame(
        &self,
        frame: &CameraFrameBundle,
    ) -> Result<Option<DepthCameraCalibration>> {
        let saved = self
            .config
            .cameras
            .get(&frame.source_id)
            .and_then(|camera| camera.calibration.as_ref());
        let Some(camera_in_base) = saved.map(|result| result.camera_in_base.clone()) else {
            return Ok(frame.calibration.clone());
        };
        let camera_matrix = intrinsics_matrix(&frame.intrinsics);
        Ok(Some(DepthCameraCalibration {
            schema_version: SCHEMA_VERSION,
            sequence: frame.sequence,
            source_time_ns: frame.device_time_ns,
            source_id: frame.source_id.clone(),
            parent_frame_id: "base_link".into(),
            frame_id: frame.color.frame_id.clone(),
            translation_m: camera_in_base.position_m,
            orientation_xyzw: camera_in_base.orientation_xyzw,
            width: frame.intrinsics.width,
            height: frame.intrinsics.height,
            distortion_model: frame.intrinsics.distortion_model.clone(),
            distortion: frame.intrinsics.distortion.clone(),
            camera_matrix,
            projection_matrix: [
                camera_matrix[0],
                camera_matrix[1],
                camera_matrix[2],
                0.0,
                camera_matrix[3],
                camera_matrix[4],
                camera_matrix[5],
                0.0,
                camera_matrix[6],
                camera_matrix[7],
                camera_matrix[8],
                0.0,
            ],
        }))
    }

    fn apply_calibration_request(
        &mut self,
        node: &mut DoraNode,
        request: CalibrationRequest,
    ) -> Result<()> {
        let result = match request.action {
            CalibrationAction::Start => self.start_automatic_calibration(node, &request),
            CalibrationAction::Apply => self.apply_solved_calibration(),
            CalibrationAction::Cancel => {
                self.calibration_work = None;
                self.config.calibration_session = default_calibration_session();
                self.calibration_capture_at = None;
                Ok(())
            }
        };
        if let Err(error) = result {
            let message = error.to_string();
            if request.action == CalibrationAction::Start {
                self.fail_automatic_calibration(message);
            } else {
                self.config.calibration_session.original_error = Some(message);
            }
        } else {
            self.config.calibration_session.original_error = None;
        }
        self.set_simulation_calibration_active(self.config.calibration_session.active);
        send(
            node,
            "calibration_request_result",
            &RequestResult {
                schema_version: SCHEMA_VERSION,
                request_id: request.request_id,
                acknowledged_action: RequestAction::Apply,
                value: Some(self.config.calibration_session.clone()),
                original_error: self.config.calibration_session.original_error.clone(),
            },
        )?;
        self.publish_state(node)
    }

    fn start_automatic_calibration(
        &mut self,
        node: &mut DoraNode,
        request: &CalibrationRequest,
    ) -> Result<()> {
        let camera_source_id = request
            .camera_source_id
            .clone()
            .or_else(|| self.selected_source_id.clone())
            .ok_or_else(|| eyre!("开始标定需要相机来源"))?;
        if self.selected_source_id.as_deref() != Some(camera_source_id.as_str()) || !self.streaming
        {
            bail!("标定相机必须是当前正在采集的相机来源");
        }
        let model = self
            .robot_model
            .as_ref()
            .ok_or_else(|| eyre!("开始标定需要机械臂型号信息"))?;
        let model_revision = request
            .robot_model_revision
            .clone()
            .unwrap_or_else(|| model.model_revision.clone());
        if model_revision != model.model_revision {
            bail!("标定请求与当前机械臂型号不一致");
        }
        if model.calibration_targets.is_empty() {
            bail!("当前机械臂型号没有声明自动标定姿态");
        }
        let board = request
            .board
            .clone()
            .ok_or_else(|| eyre!("开始标定需要标定板实测参数"))?;
        let calibration_tool_id = request
            .calibration_tool_id
            .clone()
            .ok_or_else(|| eyre!("开始标定需要测试爪标识"))?;
        let run_id = request.request_id.clone();
        self.config.calibration_session = CalibrationSessionState {
            schema_version: SCHEMA_VERSION,
            active: true,
            phase: CalibrationPhase::Preparing,
            run_id: Some(run_id.clone()),
            current_target_index: None,
            target_count: model.calibration_targets.len() as u32,
            current_target_key: None,
            motion_request_id: None,
            stage_message: Some("正在切换到手动关节控制".into()),
            board: Some(board),
            camera_source_id: Some(camera_source_id),
            robot_model_revision: Some(model_revision),
            calibration_tool_id: Some(calibration_tool_id),
            observations: vec![],
            solved_result: None,
            original_error: None,
        };
        self.last_calibration_attempt_ns = None;
        self.calibration_work = None;
        self.calibration_capture_at = None;
        send(
            node,
            "calibration_control_mode",
            &SetControlModeRequest {
                schema_version: SCHEMA_VERSION,
                request_id: format!("{run_id}-manual-mode"),
                mode: ControlMode::Manual,
            },
        )
    }

    fn advance_automatic_calibration(&mut self, node: &mut DoraNode) -> Result<()> {
        if !self.config.calibration_session.active {
            return Ok(());
        }
        if self.config.calibration_session.camera_source_id != self.selected_source_id {
            bail!("当前相机来源已改变，自动标定已停止");
        }
        if self.config.calibration_session.phase == CalibrationPhase::Moving {
            let expected = self.config.calibration_session.motion_request_id.as_deref();
            let latest = self
                .latest_motion_state
                .as_ref()
                .and_then(|state| state.latest_motion.as_ref());
            if latest.map(|status| status.request_id.as_str()) != expected {
                return Ok(());
            }
            match latest.map(|status| status.state) {
                Some(RequestState::Succeeded) => {
                    if self
                        .config
                        .calibration_session
                        .current_target_index
                        .is_none()
                    {
                        self.config.calibration_session.current_target_index = Some(0);
                        self.send_current_calibration_target(node)?;
                        return Ok(());
                    }
                    let capture_at = *self
                        .calibration_capture_at
                        .get_or_insert_with(|| Instant::now() + CALIBRATION_CAPTURE_DELAY);
                    let remaining = capture_at.saturating_duration_since(Instant::now());
                    if !remaining.is_zero() {
                        self.config.calibration_session.stage_message = Some(format!(
                            "运动完成，等待 {:.1} 秒后采样",
                            remaining.as_secs_f64()
                        ));
                        return Ok(());
                    }
                    self.calibration_capture_at = None;
                    // The first sample must come from a frame received
                    // after the requested settling wait, not its old cache.
                    self.latest_frame = None;
                    self.config.calibration_session.phase = CalibrationPhase::Detecting;
                    self.config.calibration_session.stage_message =
                        Some("运动完成，正在识别 ChArUco".into());
                }
                Some(RequestState::Failed | RequestState::Cancelled) => {
                    let message = latest
                        .and_then(|status| status.result_message.clone())
                        .unwrap_or_else(|| "标定姿态运动失败".into());
                    self.fail_automatic_calibration(message);
                }
                Some(RequestState::Planning) => {
                    self.config.calibration_session.stage_message =
                        Some("正在规划当前标定姿态".into());
                }
                Some(RequestState::Executing) => {
                    self.config.calibration_session.stage_message =
                        Some("正在执行当前标定姿态".into());
                }
                _ => {}
            }
        }
        Ok(())
    }

    fn send_current_calibration_target(&mut self, node: &mut DoraNode) -> Result<()> {
        self.calibration_capture_at = None;
        let session = &self.config.calibration_session;
        let model = self
            .robot_model
            .as_ref()
            .ok_or_else(|| eyre!("机械臂型号信息缺失"))?;
        let target = match session.current_target_index {
            Some(index) => model.calibration_targets.get(index as usize),
            None => model
                .named_targets
                .iter()
                .find(|target| target.key == "work"),
        }
        .ok_or_else(|| eyre!("当前型号缺少工作位或标定姿态"))?;
        let run_id = session
            .run_id
            .as_deref()
            .ok_or_else(|| eyre!("标定运行标识缺失"))?;
        let request = calibration_motion_request(model, target, run_id)?;
        let request_id = request.request_id.clone();
        send(node, "calibration_motion_request", &request)?;
        let session = &mut self.config.calibration_session;
        session.phase = CalibrationPhase::Moving;
        session.current_target_key = Some(target.key.clone());
        session.motion_request_id = Some(request_id);
        session.stage_message = Some(format!("正在前往{}", target.label));
        Ok(())
    }

    fn try_automatic_calibration_capture(&mut self) -> Result<()> {
        if self.calibration_work.is_some()
            || self.config.calibration_session.phase != CalibrationPhase::Detecting
        {
            return Ok(());
        }
        let Some(frame_time_ns) = self.latest_frame.as_ref().map(|frame| frame.device_time_ns)
        else {
            self.config.calibration_session.stage_message = Some("等待彩色相机帧".into());
            return Ok(());
        };
        if self.last_calibration_attempt_ns == Some(frame_time_ns) {
            return Ok(());
        }
        self.last_calibration_attempt_ns = Some(frame_time_ns);
        let index = self
            .config
            .calibration_session
            .current_target_index
            .unwrap_or_default();
        let sample_id = format!(
            "{}-sample-{}",
            self.config
                .calibration_session
                .run_id
                .as_deref()
                .unwrap_or("calibration"),
            index + 1
        );
        let session = self.config.calibration_session.clone();
        let frame = self.latest_frame.clone().expect("frame checked above");
        let arm = self
            .latest_arm_state
            .clone()
            .ok_or_else(|| eyre!("尚未收到关节反馈"))?;
        let tool = self
            .latest_tool_pose
            .clone()
            .ok_or_else(|| eyre!("尚未收到当前 TCP 位姿"))?;
        self.calibration_work = Some(self.runtime.spawn_blocking(move || {
            Self::capture_calibration_observation(&sample_id, &session, &frame, &arm, &tool)
        }));
        Ok(())
    }

    fn poll_calibration_work(&mut self, node: &mut DoraNode) -> Result<()> {
        let Some(result) = self
            .calibration_work
            .as_mut()
            .and_then(|work| work.now_or_never())
        else {
            return Ok(());
        };
        self.calibration_work = None;
        let result = result.context("标定计算任务异常结束")?;
        let observation = match result {
            Ok(CalibrationWork::Detected(observation, preview)) => {
                self.calibration_preview = Some(preview);
                observation
            }
            Ok(CalibrationWork::Solved(solved)) => {
                self.config.calibration_session.solved_result = Some(solved);
                self.config.calibration_session.phase = CalibrationPhase::AwaitingConfirmation;
                self.config.calibration_session.stage_message =
                    Some("自动采样和求解完成，等待确认应用".into());
                return Ok(());
            }
            Err(error) if self.config.calibration_session.phase == CalibrationPhase::Detecting => {
                self.config.calibration_session.stage_message =
                    Some(format!("等待识别 ChArUco：{error}"));
                return Ok(());
            }
            Err(error) => return Err(error),
        };
        self.config
            .calibration_session
            .observations
            .push(observation);
        self.config.calibration_session.solved_result = None;
        let next = self
            .config
            .calibration_session
            .current_target_index
            .unwrap_or_default()
            + 1;
        if next < self.config.calibration_session.target_count {
            self.config.calibration_session.current_target_index = Some(next);
            self.send_current_calibration_target(node)?;
        } else {
            self.config.calibration_session.phase = CalibrationPhase::Solving;
            self.config.calibration_session.stage_message = Some("正在求解相机外参".into());
            let session = self.config.calibration_session.clone();
            self.calibration_work = Some(self.runtime.spawn_blocking(move || {
                Self::solve_calibration(&session).map(CalibrationWork::Solved)
            }));
        }
        Ok(())
    }

    fn apply_solved_calibration(&mut self) -> Result<()> {
        let solved = self
            .config
            .calibration_session
            .solved_result
            .clone()
            .ok_or_else(|| eyre!("尚无可应用的标定结果"))?;
        let source_id = solved.camera_source_id.clone();
        let mut next = self.config.clone();
        next.cameras.entry(source_id).or_default().calibration = Some(solved);
        next.config_version += 1;
        save(&self.config_path, &next)?;
        self.config = next;
        let session = &mut self.config.calibration_session;
        session.active = false;
        session.phase = CalibrationPhase::Applied;
        session.stage_message = Some("标定结果已应用并保存".into());
        session.original_error = None;
        Ok(())
    }

    fn fail_automatic_calibration(&mut self, message: String) {
        self.calibration_work = None;
        let session = &mut self.config.calibration_session;
        session.active = false;
        session.phase = CalibrationPhase::Failed;
        session.stage_message = Some(message.clone());
        session.original_error = Some(message);
        self.set_simulation_calibration_active(false);
    }

    fn set_simulation_calibration_active(&self, active: bool) {
        let _ = self
            .capture_commands
            .blocking_send(CaptureCommand::SetCalibrationActive(active));
    }

    #[cfg(feature = "opencv-runtime")]
    fn capture_calibration_observation(
        sample_id: &str,
        session: &CalibrationSessionState,
        frame: &CameraFrameBundle,
        arm: &ArmState,
        tool: &ToolPose,
    ) -> Result<CalibrationWork> {
        let board = session
            .board
            .as_ref()
            .ok_or_else(|| eyre!("标定板配置缺失"))?;
        let detection = camera_calibration::detect(
            &color_png(&frame.color)?,
            &intrinsics_matrix(&frame.intrinsics),
            &frame.intrinsics.distortion,
            &frame.intrinsics.distortion_model,
            board,
        )?;
        Ok(CalibrationWork::Detected(
            CalibrationObservation {
                sample_id: sample_id.into(),
                sample_time_ns: arm.sample_time_ns,
                camera_frame_id: frame.color.frame_id.clone(),
                board_in_camera: detection.board_in_camera,
                tcp_in_base: Pose3 {
                    position_m: tool.position_m,
                    orientation_xyzw: tool.orientation_xyzw,
                },
                joint_feedback_rad: arm.joints_rad.clone(),
            },
            detection.visualization_png,
        ))
    }

    #[cfg(not(feature = "opencv-runtime"))]
    fn capture_calibration_observation(
        _sample_id: &str,
        _session: &CalibrationSessionState,
        _frame: &CameraFrameBundle,
        _arm: &ArmState,
        _tool: &ToolPose,
    ) -> Result<CalibrationWork> {
        bail!("camera-node 构建时未启用 OpenCV 标定支持")
    }

    #[cfg(feature = "opencv-runtime")]
    fn solve_calibration(session: &CalibrationSessionState) -> Result<CalibrationResult> {
        let tcp = session
            .observations
            .iter()
            .map(|observation| observation.tcp_in_base.clone())
            .collect::<Vec<_>>();
        let board = session
            .observations
            .iter()
            .map(|observation| observation.board_in_camera.clone())
            .collect::<Vec<_>>();
        let solved = camera_calibration::solve(&tcp, &board)?;
        Ok(CalibrationResult {
            schema_version: SCHEMA_VERSION,
            camera_source_id: session
                .camera_source_id
                .clone()
                .ok_or_else(|| eyre!("标定相机来源缺失"))?,
            robot_model_revision: session
                .robot_model_revision
                .clone()
                .ok_or_else(|| eyre!("标定机械臂型号缺失"))?,
            calibration_tool_id: session
                .calibration_tool_id
                .clone()
                .ok_or_else(|| eyre!("标定测试爪标识缺失"))?,
            board: session
                .board
                .clone()
                .ok_or_else(|| eyre!("标定板配置缺失"))?,
            camera_in_base: solved.camera_in_base,
            board_in_calibration_tool: solved.board_in_calibration_tool,
            solver: solved.solver,
            solved_at_ns: now_ns(),
            sample_count: session.observations.len() as u32,
            translation_residuals_m: solved.translation_residuals_m,
            rotation_residuals_rad: solved.rotation_residuals_rad,
        })
    }

    #[cfg(not(feature = "opencv-runtime"))]
    fn solve_calibration(_session: &CalibrationSessionState) -> Result<CalibrationResult> {
        bail!("camera-node 构建时未启用 OpenCV 标定支持")
    }

    fn send_asset(&self, node: &mut DoraNode, request: PerceptionAssetRequest) -> Result<()> {
        let content = (request.asset_key == "calibration.png")
            .then(|| self.calibration_preview.clone())
            .flatten();
        send(
            node,
            "asset_response",
            &PerceptionAssetResponse {
                schema_version: SCHEMA_VERSION,
                request_id: request.request_id,
                asset_key: request.asset_key.clone(),
                mime_type: content.as_ref().map(|_| "image/png".into()),
                content,
                original_error: (request.asset_key != "calibration.png")
                    .then(|| format!("camera-node 不提供资源 {}", request.asset_key))
                    .or_else(|| {
                        self.calibration_preview
                            .is_none()
                            .then(|| "尚未生成标定预览".into())
                    }),
            },
        )
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
            streaming: self.streaming,
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
        send(node, "calibration_state", &self.config.calibration_session)?;
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

fn load_config(path: &Path) -> Result<CameraConfigStore> {
    let config: CameraConfigStore = load_or_default(path)?;
    if config.schema_version != CONFIG_SCHEMA_VERSION {
        bail!("不支持的相机配置版本 {}", config.schema_version);
    }
    Ok(config)
}

fn intrinsics_matrix(value: &robot_arm_messages::CameraIntrinsics) -> [f64; 9] {
    [
        value.focal_length_px[0],
        0.0,
        value.principal_point_px[0],
        0.0,
        value.focal_length_px[1],
        value.principal_point_px[1],
        0.0,
        0.0,
        1.0,
    ]
}

#[cfg(feature = "opencv-runtime")]
fn color_png(message: &robot_arm_messages::CameraImagePlane) -> Result<Vec<u8>> {
    let rgb = RgbImage::from_raw(message.width, message.height, message.packed_rgb()?)
        .expect("validated packed RGB dimensions");
    let mut output = Cursor::new(Vec::new());
    DynamicImage::ImageRgb8(rgb).write_to(&mut output, ImageFormat::Png)?;
    Ok(output.into_inner())
}

fn calibration_motion_request(
    model: &RobotModelInfo,
    target: &NamedMotionTarget,
    run_id: &str,
) -> Result<MotionRequest> {
    let joints = model
        .joints
        .iter()
        .map(|joint| {
            target
                .joint_positions_rad
                .get(&joint.key)
                .copied()
                .map(|position_rad| JointPosition {
                    joint_key: joint.key.clone(),
                    position_rad,
                })
                .ok_or_else(|| eyre!("标定姿态缺少关节 {}", joint.key))
        })
        .collect::<Result<Vec<_>>>()?;
    Ok(MotionRequest {
        schema_version: SCHEMA_VERSION,
        request_id: format!("{run_id}-{}", target.key),
        model_revision: model.model_revision.clone(),
        joints,
        actuators: vec![],
        options: BTreeMap::new(),
        action: RequestAction::Apply,
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

fn merge_saved_sources(sources: &mut Vec<CameraSourceInfo>, config: &CameraConfigStore) {
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

fn capture_command_error(
    error: tokio::sync::mpsc::error::TrySendError<CaptureCommand>,
) -> eyre::Report {
    match error {
        tokio::sync::mpsc::error::TrySendError::Full(_) => eyre!("相机操作正在处理中"),
        tokio::sync::mpsc::error::TrySendError::Closed(_) => eyre!("相机采集任务已经停止"),
    }
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
        let mut config = CameraConfigStore::default();
        config.calibration_session.active = true;
        config.calibration_session.phase = CalibrationPhase::Moving;
        let encoded = serde_json::to_string(&config).unwrap();
        assert!(!encoded.contains("selected_source"));
        assert!(!encoded.contains("calibration_session"));
    }

    #[test]
    fn calibration_motion_uses_the_model_declared_joint_set() {
        let target = NamedMotionTarget {
            key: "view-a".into(),
            label: "视角 A".into(),
            joint_positions_rad: BTreeMap::from([("axis-a".into(), 0.25), ("axis-b".into(), -0.5)]),
            actuator_positions_rad: BTreeMap::new(),
        };
        let model: RobotModelInfo = serde_json::from_value(serde_json::json!({
            "schema_version": SCHEMA_VERSION,
            "model_id": "fixture",
            "model_revision": "fixture-v1",
            "display_name": "Fixture",
            "base_frame": "base",
            "tcp_frame": "tcp",
            "joints": [
                {"key":"axis-a", "label":"A", "unit":"rad", "minimum":-1.0, "maximum":1.0},
                {"key":"axis-b", "label":"B", "unit":"rad", "minimum":-1.0, "maximum":1.0}
            ],
            "tool_actuators": [],
            "named_targets": [],
            "calibration_targets": [target.clone()],
            "motion_options": [],
            "diagnostics": [],
            "visualization": {"manifest_hash":"none", "root_path":"model.urdf", "files":[], "link_materials":{}}
        }))
        .unwrap();
        let request = calibration_motion_request(&model, &target, "run").unwrap();
        assert_eq!(request.request_id, "run-view-a");
        assert_eq!(request.model_revision, "fixture-v1");
        assert_eq!(request.joints[0].joint_key, "axis-a");
        assert_eq!(request.joints[1].position_rad, -0.5);
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
        let config = CameraConfigStore {
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
            cameras: BTreeMap::new(),
            calibration_session: default_calibration_session(),
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
