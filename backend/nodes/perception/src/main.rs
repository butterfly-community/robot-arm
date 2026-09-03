use std::{
    collections::BTreeMap,
    io::Cursor,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use base64::{Engine, engine::general_purpose::STANDARD as BASE64};
use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result, bail, eyre};
use image::{DynamicImage, ImageFormat, Rgb, RgbImage};
use json_config_store::{load_or_default, save};
use nalgebra::{Isometry3, Matrix3, Quaternion, Rotation3, Translation3, UnitQuaternion};
use perception_core::{InstancePointCloud, world_scene_and_instance_clouds_from_aligned_depth};
use robot_arm_messages::{
    AlignedDepthFrame, ArmState, CalibrationAction, CalibrationObservation, CalibrationPhase,
    CalibrationRequest, CalibrationResult, CalibrationSessionState, CameraCaptureState,
    CameraFrameBundle, CameraImagePlane, CameraIntrinsics, ControlMode, DepthCameraCalibration,
    DetectedInstance2D, ImageFrameInfo, JointPosition, MotionRequest, MotionState,
    NamedMotionTarget, PerceptionAssetRequest, PerceptionAssetResponse, PerceptionInstanceSummary,
    PerceptionRequest, PerceptionState, Pose3, RequestAction, RequestResult, RequestState,
    RobotModelInfo, SCHEMA_VERSION, ServiceState, SetControlModeRequest, ToolPose, WorldScene,
    camera_frame_from_arrow, from_arrow, to_arrow,
};
use serde::{Deserialize, Serialize};

const CONFIG_SCHEMA_VERSION: u32 = 1;
const CALIBRATION_CAPTURE_DELAY: Duration = Duration::from_secs(10);
const CALIBRATION_JOINT_SETTLE_TOLERANCE_RAD: f64 = 2.0_f64.to_radians();
const PRESET_CALIBRATIONS: &str = include_str!("../assets/preset-calibrations.json");
#[derive(Clone, Debug, Serialize, Deserialize)]
struct PerceptionConfig {
    schema_version: u32,
    config_version: u64,
    enabled: bool,
    #[serde(skip)]
    source_id: Option<String>,
    compute_service_url: String,
    classes: Vec<String>,
    #[serde(default)]
    placement_labels: Vec<String>,
    #[serde(default)]
    cameras: BTreeMap<String, CameraConfig>,
    // An in-flight calibration is runtime workflow state, not retained configuration.
    #[serde(skip, default = "default_calibration_session")]
    calibration_session: CalibrationSessionState,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct CameraConfig {
    calibration: Option<CalibrationResult>,
}

#[derive(Deserialize)]
struct PresetCalibration {
    source_id: String,
    camera_in_base: Pose3,
}

impl Default for PerceptionConfig {
    fn default() -> Self {
        Self {
            schema_version: CONFIG_SCHEMA_VERSION,
            config_version: 0,
            enabled: false,
            source_id: None,
            compute_service_url: "http://perception-compute:8000".into(),
            classes: Vec::new(),
            placement_labels: Vec::new(),
            cameras: BTreeMap::new(),
            calibration_session: default_calibration_session(),
        }
    }
}

struct PerceptionNode {
    config_path: PathBuf,
    config: PerceptionConfig,
    http: reqwest::blocking::Client,
    model: String,
    latest_bundle: Option<CameraFrameBundle>,
    camera_state: Option<CameraCaptureState>,
    sequence: u64,
    last_frame_time_ns: Option<i64>,
    last_scene: Option<WorldScene>,
    color_frame: Option<ImageFrameInfo>,
    depth_frame: Option<ImageFrameInfo>,
    camera_calibration: Option<DepthCameraCalibration>,
    instances: Vec<PerceptionInstanceSummary>,
    point_count: Option<u64>,
    assets: BTreeMap<String, (String, Vec<u8>)>,
    last_error: Option<String>,
    calibration_color: Option<CameraImagePlane>,
    calibration_intrinsics: Option<CameraIntrinsics>,
    last_calibration_attempt_ns: Option<i64>,
    calibration_capture_at: Option<Instant>,
    preview_depth: Option<CameraImagePlane>,
    latest_arm_state: Option<ArmState>,
    latest_tool_pose: Option<ToolPose>,
    latest_motion_state: Option<MotionState>,
    robot_model: Option<RobotModelInfo>,
}

fn default_calibration_session() -> CalibrationSessionState {
    CalibrationSessionState {
        schema_version: SCHEMA_VERSION,
        ..Default::default()
    }
}

#[derive(Serialize)]
struct SegmentRequest<'a> {
    image_base64: String,
    classes: &'a [String],
}

#[derive(Deserialize)]
struct SegmentResponse {
    instances: Vec<SegmentInstance>,
}

#[derive(Deserialize)]
struct ModelResponse {
    model: String,
}

#[derive(Deserialize)]
struct SegmentInstance {
    instance_id: String,
    label: String,
    confidence: f64,
    bounding_box_xyxy: [f64; 4],
    mask_width: u32,
    mask_height: u32,
    mask_png_base64: String,
}

#[derive(Serialize)]
struct GraspRequest<'a> {
    points_xyz_m: &'a [[f32; 3]],
    gripper_asset_id: &'a str,
}

#[derive(Deserialize)]
struct GraspResponse {
    candidates: Vec<GraspCandidateResponse>,
}

#[derive(Deserialize)]
struct GraspCandidateResponse {
    transform: [[f64; 4]; 4],
    confidence: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct MatrixPose {
    rotation_matrix: [f64; 9],
    translation_m: [f64; 3],
}

#[derive(Deserialize)]
struct CalibrationSolveResponse {
    camera_in_base: MatrixPose,
    board_in_calibration_tool: MatrixPose,
    translation_residuals_m: Vec<f64>,
    rotation_residuals_rad: Vec<f64>,
    solver: String,
}

#[derive(Deserialize)]
struct CalibrationDetection {
    #[serde(flatten)]
    pose: MatrixPose,
    visualization_png_base64: String,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("perception-node: {error:?}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let config_path = std::env::var_os("PERCEPTION_CONFIG")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/config/perception.json"));
    let config = load_config(&config_path)?;
    let http = reqwest::blocking::Client::new();
    let model = http
        .get(format!(
            "{}/v1/model",
            config.compute_service_url.trim_end_matches('/')
        ))
        .send()
        .context("读取 perception-compute-service 模型")?
        .error_for_status()
        .context("perception-compute-service 模型接口返回错误")?
        .json::<ModelResponse>()?
        .model;
    let mut perception = PerceptionNode {
        config_path,
        config,
        http,
        model,
        latest_bundle: None,
        camera_state: None,
        sequence: 0,
        last_frame_time_ns: None,
        last_scene: None,
        color_frame: None,
        depth_frame: None,
        camera_calibration: None,
        instances: vec![],
        point_count: None,
        assets: BTreeMap::new(),
        last_error: None,
        calibration_color: None,
        calibration_intrinsics: None,
        last_calibration_attempt_ns: None,
        calibration_capture_at: None,
        preview_depth: None,
        latest_arm_state: None,
        latest_tool_pose: None,
        latest_motion_state: None,
        robot_model: None,
    };
    let (mut node, mut events) = DoraNode::init_from_env()?;
    perception.publish_snapshot(&mut node)?;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => match id.as_str() {
                "request" => perception.apply_request(&mut node, from_arrow(data.as_array())?)?,
                "calibration_request" => {
                    perception.apply_calibration_request(&mut node, from_arrow(data.as_array())?)?
                }
                "asset_request" => {
                    perception.send_asset(&mut node, from_arrow(data.as_array())?)?
                }
                "camera_frame" => {
                    let frame = camera_frame_from_arrow(data.as_array())?;
                    if let Err(error) = perception.handle_camera_frame(frame) {
                        perception.last_error = Some(format!("{error:#}"));
                    }
                }
                "camera_state" => {
                    let state: CameraCaptureState = from_arrow(data.as_array())?;
                    let active_source = state
                        .streaming
                        .then(|| state.selected_source_id.clone())
                        .flatten();
                    let changed = perception.config.source_id != active_source;
                    perception.config.source_id = active_source;
                    perception.camera_state = Some(state);
                    if changed {
                        perception.clear_output(&mut node)?;
                    }
                }
                "arm_state" => {
                    perception.latest_arm_state = Some(from_arrow(data.as_array())?);
                }
                "motion_state" => {
                    let state: MotionState = from_arrow(data.as_array())?;
                    perception.latest_tool_pose = state.current_tool_pose.clone();
                    perception.latest_motion_state = Some(state);
                }
                "robot_model_info" => perception.robot_model = Some(from_arrow(data.as_array())?),
                "snapshot" => perception.publish_snapshot(&mut node)?,
                "tick" => perception.tick(&mut node)?,
                _ => {}
            },
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}

impl PerceptionNode {
    fn selected_camera(&self, source_id: &str) -> bool {
        self.config.source_id.as_deref() == Some(source_id)
    }

    fn has_calibration(&self) -> bool {
        let Some(source_id) = self.config.source_id.as_deref() else {
            return false;
        };
        self.config
            .cameras
            .get(source_id)
            .and_then(|camera| camera.calibration.as_ref())
            .is_some()
            || preset_camera_in_base(source_id).is_some()
    }

    fn clear_output(&mut self, node: &mut DoraNode) -> Result<()> {
        let frame_id = self
            .last_scene
            .as_ref()
            .map(|scene| scene.frame_id.clone())
            .unwrap_or_default();
        self.latest_bundle = None;
        self.last_scene = None;
        self.color_frame = None;
        self.depth_frame = None;
        self.camera_calibration = None;
        self.instances.clear();
        self.point_count = None;
        self.last_frame_time_ns = None;
        self.assets.clear();
        self.calibration_color = None;
        self.calibration_intrinsics = None;
        self.last_calibration_attempt_ns = None;
        self.preview_depth = None;
        self.sequence += 1;
        let sample_time_ns = now_ns();
        send(
            node,
            "world_scene",
            &WorldScene {
                schema_version: SCHEMA_VERSION,
                sequence: self.sequence,
                sample_time_ns,
                frame_id,
                objects: vec![],
                placement_regions: vec![],
                obstacles: vec![],
            },
        )
    }

    fn tick(&mut self, node: &mut DoraNode) -> Result<()> {
        if let Err(error) = self.advance_automatic_calibration(node) {
            self.fail_automatic_calibration(error.to_string());
            self.config.config_version += 1;
            save(&self.config_path, &self.config)?;
        }
        if let Err(error) = self.try_automatic_calibration_capture(node) {
            self.fail_automatic_calibration(error.to_string());
            self.config.config_version += 1;
            save(&self.config_path, &self.config)?;
        }
        self.publish_state(node)
    }

    fn handle_camera_frame(&mut self, frame: CameraFrameBundle) -> Result<()> {
        if !self.selected_camera(&frame.source_id) {
            return Ok(());
        }
        if self
            .latest_bundle
            .as_ref()
            .is_some_and(|previous| previous.sequence >= frame.sequence)
        {
            return Ok(());
        }
        self.calibration_color = Some(frame.color.clone());
        self.calibration_intrinsics = Some(frame.color_intrinsics.clone());
        self.preview_depth = Some(frame.depth.clone());
        self.color_frame = Some(image_frame_info(&frame.color));
        self.depth_frame = Some(image_frame_info(&frame.depth));
        self.last_frame_time_ns = Some(frame.received_time_ns);
        self.latest_bundle = Some(frame.clone());
        if self.has_calibration() {
            self.camera_calibration = Some(camera_calibration(
                &frame,
                self.config
                    .cameras
                    .get(&frame.source_id)
                    .and_then(|camera| camera.calibration.as_ref()),
            )?);
        }
        self.assets.insert(
            "color.png".into(),
            ("image/png".into(), color_png(&frame.color)?),
        );
        self.assets.insert(
            "depth.png".into(),
            (
                "image/png".into(),
                depth_preview_png(&decode_depth(&frame)?)?,
            ),
        );
        Ok(())
    }

    fn process_latest_scene(&mut self, node: &mut DoraNode) -> Result<()> {
        if self.config.calibration_session.active {
            bail!("相机外参标定进行中，请在标定结束后运行感知");
        }
        if !self.has_calibration() {
            bail!("当前相机尚未配置外参");
        }
        let frame = self
            .latest_bundle
            .clone()
            .ok_or_else(|| eyre!("尚未收到相机帧"))?;
        let color = frame.color.clone();
        let mut aligned_depth = align_depth_to_color(&frame)?;
        let mut calibration = camera_calibration(
            &frame,
            self.config
                .cameras
                .get(&frame.source_id)
                .and_then(|camera| camera.calibration.as_ref()),
        )?;
        let sequence = self.sequence + 1;
        aligned_depth.sequence = sequence;
        calibration.sequence = sequence;
        let instances = segment(
            &self.http,
            &self.config.compute_service_url,
            &self.config.classes,
            &color,
        )?;
        let overlay = segmentation_debug_image(&color, &instances)?;
        let (mut scene, instance_clouds) = world_scene_and_instance_clouds_from_aligned_depth(
            sequence,
            &aligned_depth,
            &calibration,
            &instances,
            &self.config.placement_labels,
        )?;
        self.point_count = Some(
            instance_clouds
                .iter()
                .map(|cloud| cloud.points_xyz_m.len() as u64)
                .sum(),
        );
        attach_grasp_candidates(
            &self.http,
            &self.config.compute_service_url,
            self.robot_model
                .as_ref()
                .and_then(|model| model.gripper_asset_id.as_deref()),
            &mut scene,
            &instance_clouds,
        )?;
        self.sequence = sequence;
        self.store_frame_details(
            &color,
            &aligned_depth,
            &overlay,
            &calibration,
            &instances,
            &scene,
        )?;
        self.last_frame_time_ns = Some(frame.received_time_ns);
        self.publish_scene(node, scene)
    }

    fn publish_scene(&mut self, node: &mut DoraNode, scene: WorldScene) -> Result<()> {
        send(node, "world_scene", &scene)?;
        self.last_scene = Some(scene);
        Ok(())
    }

    fn store_frame_details(
        &mut self,
        color: &CameraImagePlane,
        depth: &AlignedDepthFrame,
        overlay: &CameraImagePlane,
        calibration: &DepthCameraCalibration,
        instances: &[DetectedInstance2D],
        scene: &WorldScene,
    ) -> Result<()> {
        self.color_frame = Some(image_frame_info(color));
        self.depth_frame = Some(ImageFrameInfo {
            width: depth.width,
            height: depth.height,
            encoding: "16UC1".into(),
            frame_id: depth.frame_id.clone(),
        });
        self.camera_calibration = Some(calibration.clone());
        self.instances = instances
            .iter()
            .map(|instance| {
                let object = scene
                    .objects
                    .iter()
                    .find(|object| object.object_id == instance.instance_id);
                PerceptionInstanceSummary {
                    instance_id: instance.instance_id.clone(),
                    label: instance.label.clone(),
                    confidence: instance.confidence,
                    bounding_box_xyxy: instance.bounding_box_xyxy,
                    position_m: object.map(|object| object.pose.position_m),
                    size_m: object.map(|object| object.size_m),
                    grasp_candidate_count: object
                        .map(|object| object.grasp_candidates.len() as u32)
                        .unwrap_or_default(),
                }
            })
            .collect();
        self.assets
            .insert("color.png".into(), ("image/png".into(), color_png(color)?));
        self.assets.insert(
            "overlay.png".into(),
            ("image/png".into(), color_png(overlay)?),
        );
        self.assets.insert(
            "depth.png".into(),
            ("image/png".into(), depth_preview_png(depth)?),
        );
        Ok(())
    }

    fn send_asset(&self, node: &mut DoraNode, request: PerceptionAssetRequest) -> Result<()> {
        let asset = self.assets.get(&request.asset_key);
        send(
            node,
            "asset_response",
            &PerceptionAssetResponse {
                schema_version: SCHEMA_VERSION,
                request_id: request.request_id,
                asset_key: request.asset_key.clone(),
                mime_type: asset.map(|(mime, _)| mime.clone()),
                content: asset.map(|(_, content)| content.clone()),
                original_error: asset
                    .is_none()
                    .then(|| format!("感知资源 {} 尚未生成", request.asset_key)),
            },
        )
    }

    fn apply_request(&mut self, node: &mut DoraNode, request: PerceptionRequest) -> Result<()> {
        let mut next = self.config.clone();
        let result = match request.action {
            RequestAction::Apply => {
                if let Some(classes) = request.classes {
                    next.classes = classes;
                }
                if let Some(labels) = request.placement_labels {
                    next.placement_labels = labels;
                }
                next.enabled = true;
                Ok(())
            }
            RequestAction::Refresh => self.process_latest_scene(node),
            RequestAction::Snapshot => self.snapshot_previews(),
            RequestAction::Reset => {
                let source_id = request
                    .source_id
                    .as_deref()
                    .or(next.source_id.as_deref())
                    .ok_or_else(|| eyre!("请选择要重置的深度相机"))?
                    .to_owned();
                next.cameras.remove(&source_id);
                if next.calibration_session.camera_source_id.as_deref() == Some(source_id.as_str())
                {
                    next.calibration_session = default_calibration_session();
                }
                Ok(())
            }
            RequestAction::Cancel | RequestAction::Disconnect => {
                next.enabled = false;
                Ok(())
            }
            action => Err(eyre!(
                "perception request action {action:?} is not supported"
            )),
        };
        let persist = !matches!(
            request.action,
            RequestAction::Refresh | RequestAction::Snapshot
        );
        let error = result
            .and_then(|()| {
                if !persist {
                    return Ok(());
                }
                next.config_version += 1;
                save(&self.config_path, &next)?;
                Ok(())
            })
            .err()
            .map(|error: eyre::Report| error.to_string());
        if error.is_none() && persist {
            self.config = next;
            self.clear_output(node)?;
        }
        self.last_error = error.clone();
        send(
            node,
            "request_result",
            &RequestResult {
                schema_version: SCHEMA_VERSION,
                request_id: request.request_id,
                acknowledged_action: request.action,
                value: Some(self.state()),
                original_error: error,
            },
        )?;
        self.publish_state(node)
    }

    fn snapshot_previews(&mut self) -> Result<()> {
        let color = self
            .calibration_color
            .as_ref()
            .ok_or_else(|| eyre!("尚未收到彩色图"))?;
        let depth_message = self
            .preview_depth
            .as_ref()
            .ok_or_else(|| eyre!("尚未收到深度图"))?;
        let bundle = self
            .latest_bundle
            .as_ref()
            .ok_or_else(|| eyre!("尚未收到相机帧"))?;
        let depth = decode_depth(bundle)?;
        let color_png = color_png(color)?;
        let depth_png = depth_preview_png(&depth)?;
        self.color_frame = Some(image_frame_info(color));
        self.depth_frame = Some(image_frame_info(depth_message));
        self.assets
            .insert("color.png".into(), ("image/png".into(), color_png));
        self.assets
            .insert("depth.png".into(), ("image/png".into(), depth_png));
        self.last_frame_time_ns = Some(depth.source_time_ns);
        Ok(())
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
                self.config.calibration_session = default_calibration_session();
                self.calibration_capture_at = None;
                Ok(())
            }
        };
        let error = result.err().map(|error| error.to_string());
        if let Some(message) = error.as_ref() {
            if request.action == CalibrationAction::Start {
                self.fail_automatic_calibration(message.clone());
            } else {
                self.config.calibration_session.original_error = Some(message.clone());
            }
        } else {
            self.config.calibration_session.original_error = None;
        }
        self.config.config_version += 1;
        if let Err(save_error) = save(&self.config_path, &self.config) {
            self.config.calibration_session.original_error = Some(save_error.to_string());
        }
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
            .or_else(|| self.config.source_id.clone())
            .ok_or_else(|| eyre!("开始标定需要相机来源"))?;
        if self.config.source_id.as_deref() != Some(camera_source_id.as_str()) {
            bail!("标定相机必须是当前启用的相机来源");
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
        let run_id = request.request_id.clone();
        self.config.calibration_session = CalibrationSessionState {
            schema_version: SCHEMA_VERSION,
            active: true,
            phase: CalibrationPhase::Preparing,
            run_id: Some(run_id.clone()),
            current_target_index: Some(0),
            target_count: model.calibration_targets.len() as u32,
            current_target_key: None,
            motion_request_id: None,
            stage_message: Some("正在切换到手动关节控制".into()),
            board: Some(
                request
                    .board
                    .clone()
                    .ok_or_else(|| eyre!("开始标定需要标定板实测参数"))?,
            ),
            camera_source_id: Some(camera_source_id),
            robot_model_revision: Some(model_revision),
            calibration_tool_id: Some(
                request
                    .calibration_tool_id
                    .clone()
                    .ok_or_else(|| eyre!("开始标定需要测试爪标识"))?,
            ),
            observations: vec![],
            solved_result: None,
            original_error: None,
        };
        self.last_calibration_attempt_ns = None;
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
        if self.config.calibration_session.camera_source_id != self.config.source_id {
            bail!("当前相机来源已改变，自动标定已停止");
        }
        match self.config.calibration_session.phase {
            CalibrationPhase::Preparing => {
                if self
                    .latest_motion_state
                    .as_ref()
                    .is_some_and(|state| state.control_mode == ControlMode::Manual)
                {
                    self.send_current_calibration_target(node)?;
                }
            }
            CalibrationPhase::Moving => {
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
                        if !self.calibration_target_reached()? {
                            self.calibration_capture_at = None;
                            self.config.calibration_session.stage_message =
                                Some("轨迹已结束，等待关节反馈稳定在标定姿态".into());
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
            _ => {}
        }
        Ok(())
    }

    fn calibration_target_reached(&self) -> Result<bool> {
        let session = &self.config.calibration_session;
        let index = session
            .current_target_index
            .ok_or_else(|| eyre!("标定姿态序号缺失"))? as usize;
        let model = self
            .robot_model
            .as_ref()
            .ok_or_else(|| eyre!("机械臂型号信息缺失"))?;
        let target = model
            .calibration_targets
            .get(index)
            .ok_or_else(|| eyre!("标定姿态 {index} 不存在"))?;
        let Some(feedback) = self.latest_arm_state.as_ref() else {
            return Ok(false);
        };
        Ok(model
            .joints
            .iter()
            .zip(&feedback.joints_rad)
            .all(|(joint, actual)| {
                target
                    .joint_positions_rad
                    .get(&joint.key)
                    .is_some_and(|expected| {
                        (actual - expected).abs() <= CALIBRATION_JOINT_SETTLE_TOLERANCE_RAD
                    })
            }))
    }

    fn send_current_calibration_target(&mut self, node: &mut DoraNode) -> Result<()> {
        self.calibration_capture_at = None;
        let session = &self.config.calibration_session;
        let index = session
            .current_target_index
            .ok_or_else(|| eyre!("标定姿态序号缺失"))? as usize;
        let model = self
            .robot_model
            .as_ref()
            .ok_or_else(|| eyre!("机械臂型号信息缺失"))?;
        let target = model
            .calibration_targets
            .get(index)
            .ok_or_else(|| eyre!("标定姿态 {index} 不存在"))?;
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

    fn try_automatic_calibration_capture(&mut self, node: &mut DoraNode) -> Result<()> {
        if self.config.calibration_session.phase != CalibrationPhase::Detecting {
            return Ok(());
        }
        let Some(frame_time_ns) = self
            .latest_bundle
            .as_ref()
            .map(|frame| frame.device_time_ns)
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
        let observation = match self.capture_calibration_observation(&sample_id) {
            Ok(observation) => observation,
            Err(error) => {
                self.config.calibration_session.stage_message =
                    Some(format!("等待识别 ChArUco：{error}"));
                return Ok(());
            }
        };
        self.config
            .calibration_session
            .observations
            .push(observation);
        self.config.calibration_session.solved_result = None;
        let next = index + 1;
        if next < self.config.calibration_session.target_count {
            self.config.calibration_session.current_target_index = Some(next);
            self.send_current_calibration_target(node)?;
        } else {
            self.config.calibration_session.phase = CalibrationPhase::Solving;
            self.config.calibration_session.stage_message = Some("正在求解相机外参".into());
            match self.solve_calibration() {
                Ok(solved) => {
                    self.config.calibration_session.solved_result = Some(solved);
                    self.config.calibration_session.phase = CalibrationPhase::AwaitingConfirmation;
                    self.config.calibration_session.stage_message =
                        Some("自动采样和求解完成，等待确认应用".into());
                }
                Err(error) => self.fail_automatic_calibration(error.to_string()),
            }
        }
        self.config.config_version += 1;
        save(&self.config_path, &self.config)?;
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
        self.config
            .cameras
            .entry(source_id.clone())
            .or_default()
            .calibration = Some(solved.clone());
        let session = &mut self.config.calibration_session;
        session.active = false;
        session.phase = CalibrationPhase::Applied;
        session.stage_message = Some("标定结果已应用并保存".into());
        session.original_error = None;
        Ok(())
    }

    fn fail_automatic_calibration(&mut self, message: String) {
        let session = &mut self.config.calibration_session;
        session.active = false;
        session.phase = CalibrationPhase::Failed;
        session.stage_message = Some(message.clone());
        session.original_error = Some(message);
    }

    fn capture_calibration_observation(
        &mut self,
        sample_id: &str,
    ) -> Result<CalibrationObservation> {
        if !self.config.calibration_session.active {
            bail!("标定会话尚未开始");
        }
        let board = self
            .config
            .calibration_session
            .board
            .as_ref()
            .ok_or_else(|| eyre!("标定板配置缺失"))?;
        let image = self
            .calibration_color
            .as_ref()
            .ok_or_else(|| eyre!("尚未收到彩色相机帧"))?;
        let info = self
            .calibration_intrinsics
            .as_ref()
            .ok_or_else(|| eyre!("尚未收到相机内参"))?;
        let arm = self
            .latest_arm_state
            .as_ref()
            .ok_or_else(|| eyre!("尚未收到真实关节反馈"))?;
        let tool = self
            .latest_tool_pose
            .as_ref()
            .ok_or_else(|| eyre!("尚未收到当前 TCP 位姿"))?;
        let detected: CalibrationDetection = invoke_calibration_tool(&serde_json::json!({
            "operation": "detect",
            "image_png_base64": BASE64.encode(color_png(image)?),
            "camera_matrix": intrinsics_matrix(info),
            "distortion": info.distortion,
            "board": board,
        }))?;
        self.assets.insert(
            "calibration.png".into(),
            (
                "image/png".into(),
                BASE64.decode(&detected.visualization_png_base64)?,
            ),
        );
        Ok(CalibrationObservation {
            sample_id: sample_id.into(),
            sample_time_ns: arm.sample_time_ns,
            camera_frame_id: image.frame_id.clone(),
            board_in_camera: matrix_pose_to_pose(&detected.pose)?,
            tcp_in_base: Pose3 {
                position_m: tool.position_m,
                orientation_xyzw: tool.orientation_xyzw,
            },
            joint_feedback_rad: arm.joints_rad.clone(),
        })
    }

    fn solve_calibration(&self) -> Result<CalibrationResult> {
        let session = &self.config.calibration_session;
        let solved: CalibrationSolveResponse = invoke_calibration_tool(&serde_json::json!({
            "operation": "solve",
            "tcp_in_base": session.observations.iter().map(|value| pose_to_matrix(&value.tcp_in_base)).collect::<Result<Vec<_>>>()?,
            "board_in_camera": session.observations.iter().map(|value| pose_to_matrix(&value.board_in_camera)).collect::<Result<Vec<_>>>()?,
        }))?;
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
            camera_in_base: matrix_pose_to_pose(&solved.camera_in_base)?,
            board_in_calibration_tool: matrix_pose_to_pose(&solved.board_in_calibration_tool)?,
            solver: solved.solver,
            solved_at_ns: now_ns(),
            sample_count: session.observations.len() as u32,
            translation_residuals_m: solved.translation_residuals_m,
            rotation_residuals_rad: solved.rotation_residuals_rad,
        })
    }

    fn state(&self) -> PerceptionState {
        PerceptionState {
            schema_version: SCHEMA_VERSION,
            enabled: self.config.enabled,
            source_id: self.config.source_id.clone(),
            compute_service_url: self.config.compute_service_url.clone(),
            model: self.model.clone(),
            classes: self.config.classes.clone(),
            placement_labels: self.config.placement_labels.clone(),
            color_frame: self.color_frame.clone(),
            depth_frame: self.depth_frame.clone(),
            camera_calibration: self.camera_calibration.clone(),
            depth_scale_m: self.latest_bundle.as_ref().map(|frame| frame.depth_scale_m),
            instances: self.instances.clone(),
            point_count: self.point_count,
            last_frame_time_ns: self.last_frame_time_ns,
            last_scene_sequence: self.last_scene.as_ref().map(|scene| scene.sequence),
            calibrated: self.has_calibration(),
            original_error: self.last_error.clone(),
            service: self.service_state(),
        }
    }

    fn service_state(&self) -> ServiceState {
        ServiceState {
            schema_version: SCHEMA_VERSION,
            build_version: env!("CARGO_PKG_VERSION").into(),
            config_version: self.config.config_version,
            running: true,
            has_input: self.last_frame_time_ns.is_some(),
            has_output: self.last_scene.is_some(),
            last_error: self.last_error.clone(),
            updated_at_ns: now_ns(),
        }
    }

    fn publish_state(&self, node: &mut DoraNode) -> Result<()> {
        send(node, "perception_state", &self.state())?;
        send(node, "calibration_state", &self.config.calibration_session)?;
        send(node, "service_state", &self.service_state())
    }

    fn publish_snapshot(&self, node: &mut DoraNode) -> Result<()> {
        self.publish_state(node)?;
        if let Some(scene) = &self.last_scene {
            send(node, "world_scene", scene)?;
        }
        Ok(())
    }
}

fn segment(
    http: &reqwest::blocking::Client,
    compute_service_url: &str,
    classes: &[String],
    color: &CameraImagePlane,
) -> Result<Vec<DetectedInstance2D>> {
    let response = http
        .post(format!(
            "{}/v1/segment",
            compute_service_url.trim_end_matches('/')
        ))
        .json(&SegmentRequest {
            image_base64: BASE64.encode(color_png(color)?),
            classes,
        })
        .send()
        .context("调用 perception-compute-service")?
        .error_for_status()
        .context("perception-compute-service 返回错误")?
        .json::<SegmentResponse>()?;
    response
        .instances
        .into_iter()
        .map(|instance| {
            Ok(DetectedInstance2D {
                instance_id: instance.instance_id,
                label: instance.label,
                confidence: instance.confidence,
                bounding_box_xyxy: instance.bounding_box_xyxy,
                mask_width: instance.mask_width,
                mask_height: instance.mask_height,
                mask_png: BASE64.decode(instance.mask_png_base64)?,
            })
        })
        .collect()
}

fn attach_grasp_candidates(
    http: &reqwest::blocking::Client,
    compute_service_url: &str,
    gripper_asset_id: Option<&str>,
    scene: &mut WorldScene,
    instance_clouds: &[InstancePointCloud],
) -> Result<()> {
    let Some(gripper_asset_id) = gripper_asset_id else {
        return Ok(());
    };
    let placement_sources = scene
        .placement_regions
        .iter()
        .filter_map(|region| region.source_object_id.as_deref())
        .collect::<std::collections::BTreeSet<_>>();
    for object in &mut scene.objects {
        if placement_sources.contains(object.object_id.as_str()) {
            continue;
        }
        let cloud = instance_clouds
            .iter()
            .find(|cloud| cloud.instance_id == object.object_id)
            .ok_or_else(|| eyre!("实例 {} 缺少点云", object.object_id))?;
        let mut candidates = http
            .post(format!(
                "{}/v1/grasps",
                compute_service_url.trim_end_matches('/')
            ))
            .json(&GraspRequest {
                points_xyz_m: &cloud.points_xyz_m,
                gripper_asset_id,
            })
            .send()
            .context("调用 GraspGenX")?
            .error_for_status()
            .context("GraspGenX 返回错误")?
            .json::<GraspResponse>()?
            .candidates;
        candidates.sort_by(|left, right| right.confidence.total_cmp(&left.confidence));
        object.grasp_candidates = candidates
            .iter()
            .map(|candidate| candidate_tcp_pose_in_base(candidate.transform))
            .collect::<Result<Vec<_>>>()?;
    }
    Ok(())
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

fn candidate_tcp_pose_in_base(base_tcp: [[f64; 4]; 4]) -> Result<Pose3> {
    let base_tcp = Isometry3::from_parts(
        Translation3::new(base_tcp[0][3], base_tcp[1][3], base_tcp[2][3]),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(Matrix3::new(
            base_tcp[0][0],
            base_tcp[0][1],
            base_tcp[0][2],
            base_tcp[1][0],
            base_tcp[1][1],
            base_tcp[1][2],
            base_tcp[2][0],
            base_tcp[2][1],
            base_tcp[2][2],
        ))),
    );
    let quaternion = base_tcp.rotation.quaternion();
    Ok(Pose3 {
        position_m: base_tcp.translation.vector.into(),
        orientation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
    })
}

fn pose_to_matrix(pose: &Pose3) -> Result<MatrixPose> {
    let [x, y, z, w] = pose.orientation_xyzw;
    let quaternion = Quaternion::new(w, x, y, z);
    if quaternion.norm_squared() == 0.0 {
        bail!("位姿四元数无效");
    }
    let rotation = UnitQuaternion::new_normalize(quaternion).to_rotation_matrix();
    let matrix = rotation.matrix();
    Ok(MatrixPose {
        rotation_matrix: [
            matrix[(0, 0)],
            matrix[(0, 1)],
            matrix[(0, 2)],
            matrix[(1, 0)],
            matrix[(1, 1)],
            matrix[(1, 2)],
            matrix[(2, 0)],
            matrix[(2, 1)],
            matrix[(2, 2)],
        ],
        translation_m: pose.position_m,
    })
}

fn matrix_pose_to_pose(value: &MatrixPose) -> Result<Pose3> {
    if !value
        .rotation_matrix
        .iter()
        .chain(&value.translation_m)
        .all(|number| number.is_finite())
    {
        bail!("标定求解器返回了非有限数值");
    }
    let rotation = Matrix3::from_row_slice(&value.rotation_matrix);
    let quaternion = UnitQuaternion::from_matrix(&rotation);
    let quaternion = quaternion.quaternion();
    Ok(Pose3 {
        position_m: value.translation_m,
        orientation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
    })
}

fn invoke_calibration_tool<T: for<'de> Deserialize<'de>>(payload: &serde_json::Value) -> Result<T> {
    let helper = std::env::var_os("PERCEPTION_CALIBRATION_HELPER")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/src/target/release/perception-calibration"));
    let mut child = Command::new(helper)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .context("启动 OpenCV Rust 标定程序")?;
    serde_json::to_writer(
        child
            .stdin
            .take()
            .expect("calibration helper stdin is piped"),
        payload,
    )?;
    let output = child.wait_with_output()?;
    if !output.status.success() {
        bail!(
            "OpenCV 标定失败：{}",
            String::from_utf8_lossy(&output.stderr).trim()
        );
    }
    serde_json::from_slice(&output.stdout).context("解析 OpenCV 标定结果")
}

fn load_config(path: &Path) -> Result<PerceptionConfig> {
    let config: PerceptionConfig = load_or_default(path)?;
    if config.schema_version != CONFIG_SCHEMA_VERSION {
        bail!("不支持的 perception 配置版本 {}", config.schema_version);
    }
    Ok(config)
}

fn camera_calibration(
    frame: &CameraFrameBundle,
    result: Option<&CalibrationResult>,
) -> Result<DepthCameraCalibration> {
    let camera_in_base = result
        .map(|result| result.camera_in_base.clone())
        .or_else(|| preset_camera_in_base(&frame.source_id))
        .ok_or_else(|| eyre!("相机尚未完成外参标定"))?;
    let camera_matrix = intrinsics_matrix(&frame.color_intrinsics);
    Ok(DepthCameraCalibration {
        schema_version: SCHEMA_VERSION,
        sequence: 0,
        source_time_ns: frame.device_time_ns,
        source_id: frame.source_id.clone(),
        parent_frame_id: "base_link".into(),
        frame_id: frame.color.frame_id.clone(),
        translation_m: camera_in_base.position_m,
        orientation_xyzw: camera_in_base.orientation_xyzw,
        width: frame.color_intrinsics.width,
        height: frame.color_intrinsics.height,
        distortion_model: frame.color_intrinsics.distortion_model.clone(),
        distortion: frame.color_intrinsics.distortion.clone(),
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
    })
}

fn preset_camera_in_base(source_id: &str) -> Option<Pose3> {
    serde_json::from_str::<Vec<PresetCalibration>>(PRESET_CALIBRATIONS)
        .ok()?
        .into_iter()
        .find(|value| value.source_id == source_id)
        .map(|value| value.camera_in_base)
}

fn decode_depth(frame: &CameraFrameBundle) -> Result<AlignedDepthFrame> {
    let message = &frame.depth;
    if message.pixel_format != "z16le" && message.pixel_format != "z16be" {
        bail!("不支持的深度编码 {}", message.pixel_format);
    }
    let mut depth = Vec::with_capacity((message.width * message.height) as usize);
    for row in message.data.chunks_exact(message.stride_bytes as usize) {
        for bytes in row[..message.width as usize * 2].chunks_exact(2) {
            depth.push(if message.pixel_format == "z16le" {
                u16::from_le_bytes([bytes[0], bytes[1]])
            } else {
                u16::from_be_bytes([bytes[0], bytes[1]])
            });
        }
    }
    Ok(AlignedDepthFrame {
        schema_version: SCHEMA_VERSION,
        sequence: frame.sequence,
        source_time_ns: frame.device_time_ns,
        source_id: frame.source_id.clone(),
        frame_id: frame.color.frame_id.clone(),
        width: message.width,
        height: message.height,
        depth_scale_m: frame.depth_scale_m,
        depth,
    })
}

fn align_depth_to_color(frame: &CameraFrameBundle) -> Result<AlignedDepthFrame> {
    let input = decode_depth(frame)?;
    let output_width = frame.color_intrinsics.width;
    let output_height = frame.color_intrinsics.height;
    let pose = &frame.depth_to_color;
    if input.width == output_width
        && input.height == output_height
        && pose.position_m == [0.0; 3]
        && pose.orientation_xyzw == [0.0, 0.0, 0.0, 1.0]
    {
        return Ok(AlignedDepthFrame {
            frame_id: frame.color.frame_id.clone(),
            ..input
        });
    }
    let q = UnitQuaternion::from_quaternion(Quaternion::new(
        pose.orientation_xyzw[3],
        pose.orientation_xyzw[0],
        pose.orientation_xyzw[1],
        pose.orientation_xyzw[2],
    ));
    let mut output = vec![0_u16; (output_width * output_height) as usize];
    for (index, raw) in input.depth.iter().copied().enumerate() {
        if raw == 0 {
            continue;
        }
        let z = raw as f64 * input.depth_scale_m;
        let point = q * deproject_pixel(
            &frame.depth_intrinsics,
            [
                (index as u32 % input.width) as f64,
                (index as u32 / input.width) as f64,
            ],
            z,
        )? + nalgebra::Vector3::from(pose.position_m);
        if point.z <= 0.0 {
            continue;
        }
        let [cu, cv] =
            project_point(&frame.color_intrinsics, point)?.map(|value| value.round() as i64);
        if cu < 0 || cv < 0 || cu >= i64::from(output_width) || cv >= i64::from(output_height) {
            continue;
        }
        let aligned_raw = (point.z / input.depth_scale_m)
            .round()
            .clamp(1.0, f64::from(u16::MAX)) as u16;
        let target = &mut output[(cv as u32 * output_width + cu as u32) as usize];
        if *target == 0 || aligned_raw < *target {
            *target = aligned_raw;
        }
    }
    Ok(AlignedDepthFrame {
        width: output_width,
        height: output_height,
        frame_id: frame.color.frame_id.clone(),
        depth: output,
        ..input
    })
}

fn distortion_coefficients(intrinsics: &CameraIntrinsics) -> [f64; 5] {
    std::array::from_fn(|index| {
        intrinsics
            .distortion
            .get(index)
            .copied()
            .unwrap_or_default()
    })
}

fn deproject_pixel(
    intrinsics: &CameraIntrinsics,
    pixel: [f64; 2],
    depth: f64,
) -> Result<nalgebra::Vector3<f64>> {
    let [fx, fy] = intrinsics.focal_length_px;
    let [cx, cy] = intrinsics.principal_point_px;
    if fx == 0.0 || fy == 0.0 {
        bail!("相机焦距不能为 0");
    }
    let coefficients = distortion_coefficients(intrinsics);
    let mut x = (pixel[0] - cx) / fx;
    let mut y = (pixel[1] - cy) / fy;
    let [xo, yo] = [x, y];
    let has_distortion = coefficients.iter().any(|value| *value != 0.0);
    if has_distortion {
        match intrinsics.distortion_model.as_str() {
            "none" => {}
            "brown_conrady" | "plumb_bob" => {
                for _ in 0..10 {
                    let r2 = x * x + y * y;
                    let inverse = 1.0
                        / (1.0
                            + ((coefficients[4] * r2 + coefficients[1]) * r2 + coefficients[0])
                                * r2);
                    let delta_x =
                        2.0 * coefficients[2] * x * y + coefficients[3] * (r2 + 2.0 * x * x);
                    let delta_y =
                        2.0 * coefficients[3] * x * y + coefficients[2] * (r2 + 2.0 * y * y);
                    x = (xo - delta_x) * inverse;
                    y = (yo - delta_y) * inverse;
                }
            }
            "inverse_brown_conrady" => {
                for _ in 0..10 {
                    let r2 = x * x + y * y;
                    let inverse = 1.0
                        / (1.0
                            + ((coefficients[4] * r2 + coefficients[1]) * r2 + coefficients[0])
                                * r2);
                    let xq = x / inverse;
                    let yq = y / inverse;
                    let delta_x =
                        2.0 * coefficients[2] * xq * yq + coefficients[3] * (r2 + 2.0 * xq * xq);
                    let delta_y =
                        2.0 * coefficients[3] * xq * yq + coefficients[2] * (r2 + 2.0 * yq * yq);
                    x = (xo - delta_x) * inverse;
                    y = (yo - delta_y) * inverse;
                }
            }
            "kannala_brandt4" => {
                let distorted_radius = x.hypot(y).max(f64::EPSILON);
                let mut theta = distorted_radius;
                for _ in 0..4 {
                    let theta2 = theta * theta;
                    let residual = theta
                        * (1.0
                            + theta2
                                * (coefficients[0]
                                    + theta2
                                        * (coefficients[1]
                                            + theta2
                                                * (coefficients[2] + theta2 * coefficients[3]))))
                        - distorted_radius;
                    if residual.abs() < f64::EPSILON {
                        break;
                    }
                    let derivative = 1.0
                        + theta2
                            * (3.0 * coefficients[0]
                                + theta2
                                    * (5.0 * coefficients[1]
                                        + theta2
                                            * (7.0 * coefficients[2]
                                                + 9.0 * theta2 * coefficients[3])));
                    theta -= residual / derivative;
                }
                let radius = theta.tan();
                x *= radius / distorted_radius;
                y *= radius / distorted_radius;
            }
            "ftheta" => {
                let distorted_radius = x.hypot(y).max(f64::EPSILON);
                let radius = (coefficients[0] * distorted_radius).tan()
                    / (2.0 * (coefficients[0] / 2.0).tan()).atan();
                x *= radius / distorted_radius;
                y *= radius / distorted_radius;
            }
            "modified_brown_conrady" => {
                bail!("modified_brown_conrady 深度流不能直接反投影")
            }
            other => bail!("不支持的相机畸变模型 {other}"),
        }
    }
    Ok(nalgebra::Vector3::new(depth * x, depth * y, depth))
}

fn project_point(intrinsics: &CameraIntrinsics, point: nalgebra::Vector3<f64>) -> Result<[f64; 2]> {
    if point.z == 0.0 {
        bail!("不能投影 Z=0 的相机坐标点");
    }
    let [fx, fy] = intrinsics.focal_length_px;
    let [cx, cy] = intrinsics.principal_point_px;
    let coefficients = distortion_coefficients(intrinsics);
    let mut x = point.x / point.z;
    let mut y = point.y / point.z;
    if coefficients.iter().any(|value| *value != 0.0) {
        match intrinsics.distortion_model.as_str() {
            "none" => {}
            "brown_conrady" | "plumb_bob" | "modified_brown_conrady" | "inverse_brown_conrady" => {
                let r2 = x * x + y * y;
                let factor = 1.0
                    + coefficients[0] * r2
                    + coefficients[1] * r2 * r2
                    + coefficients[4] * r2 * r2 * r2;
                let radial_x = x * factor;
                let radial_y = y * factor;
                let delta_x = 2.0 * coefficients[2] * x * y + coefficients[3] * (r2 + 2.0 * x * x);
                let delta_y = 2.0 * coefficients[3] * x * y + coefficients[2] * (r2 + 2.0 * y * y);
                x = radial_x + delta_x;
                y = radial_y + delta_y;
            }
            "kannala_brandt4" => {
                let radius = x.hypot(y).max(f64::EPSILON);
                let theta = radius.atan();
                let theta2 = theta * theta;
                let distorted_radius = theta
                    * (1.0
                        + theta2
                            * (coefficients[0]
                                + theta2
                                    * (coefficients[1]
                                        + theta2 * (coefficients[2] + theta2 * coefficients[3]))));
                x *= distorted_radius / radius;
                y *= distorted_radius / radius;
            }
            "ftheta" => {
                let radius = x.hypot(y).max(f64::EPSILON);
                let distorted_radius =
                    (2.0 * radius * (coefficients[0] / 2.0).tan()).atan() / coefficients[0];
                x *= distorted_radius / radius;
                y *= distorted_radius / radius;
            }
            other => bail!("不支持的相机畸变模型 {other}"),
        }
    }
    Ok([x * fx + cx, y * fy + cy])
}

fn intrinsics_matrix(value: &CameraIntrinsics) -> [f64; 9] {
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

fn color_rgb(message: &CameraImagePlane) -> Result<RgbImage> {
    let mut rgb = RgbImage::new(message.width, message.height);
    match message.pixel_format.as_str() {
        "rgb8" | "bgr8" => {
            for (pixel, source) in rgb
                .pixels_mut()
                .zip(packed_rows(message, 3)?.flat_map(|row| row.chunks_exact(3)))
            {
                *pixel = if message.pixel_format == "rgb8" {
                    Rgb([source[0], source[1], source[2]])
                } else {
                    Rgb([source[2], source[1], source[0]])
                };
            }
        }
        "rgba8" | "bgra8" => {
            for (pixel, source) in rgb
                .pixels_mut()
                .zip(packed_rows(message, 4)?.flat_map(|row| row.chunks_exact(4)))
            {
                *pixel = if message.pixel_format == "rgba8" {
                    Rgb([source[0], source[1], source[2]])
                } else {
                    Rgb([source[2], source[1], source[0]])
                };
            }
        }
        "y8" => {
            for (pixel, value) in rgb.pixels_mut().zip(packed_rows(message, 1)?.flatten()) {
                *pixel = Rgb([*value; 3]);
            }
        }
        other => bail!("不支持的彩色图编码 {other}"),
    }
    Ok(rgb)
}

fn packed_rows(
    message: &CameraImagePlane,
    bytes_per_pixel: usize,
) -> Result<impl Iterator<Item = &[u8]>> {
    let packed = message.width as usize * bytes_per_pixel;
    if (message.stride_bytes as usize) < packed {
        bail!("图像 stride 小于像素行宽");
    }
    Ok(message
        .data
        .chunks_exact(message.stride_bytes as usize)
        .map(move |row| &row[..packed]))
}

fn color_png(message: &CameraImagePlane) -> Result<Vec<u8>> {
    let rgb = color_rgb(message)?;
    let mut output = Cursor::new(Vec::new());
    DynamicImage::ImageRgb8(rgb).write_to(&mut output, ImageFormat::Png)?;
    Ok(output.into_inner())
}

fn image_frame_info(message: &CameraImagePlane) -> ImageFrameInfo {
    ImageFrameInfo {
        width: message.width,
        height: message.height,
        encoding: message.pixel_format.clone(),
        frame_id: message.frame_id.clone(),
    }
}

fn depth_preview_png(depth: &AlignedDepthFrame) -> Result<Vec<u8>> {
    let maximum = depth.depth.iter().copied().max().unwrap_or_default();
    let pixels = depth
        .depth
        .iter()
        .map(|value| {
            if maximum == 0 {
                0
            } else {
                (u32::from(*value) * 255 / u32::from(maximum)) as u8
            }
        })
        .collect::<Vec<_>>();
    let image = image::GrayImage::from_raw(depth.width, depth.height, pixels)
        .ok_or_else(|| eyre!("深度图尺寸与数据长度不一致"))?;
    let mut output = Cursor::new(Vec::new());
    DynamicImage::ImageLuma8(image).write_to(&mut output, ImageFormat::Png)?;
    Ok(output.into_inner())
}

fn segmentation_debug_image(
    source: &CameraImagePlane,
    instances: &[DetectedInstance2D],
) -> Result<CameraImagePlane> {
    const COLORS: [[u8; 3]; 4] = [
        [255, 72, 72],
        [64, 196, 255],
        [255, 196, 64],
        [150, 92, 255],
    ];
    let mut image = color_rgb(source)?;
    for (index, instance) in instances.iter().enumerate() {
        let color = COLORS[index % COLORS.len()];
        let mask =
            image::load_from_memory_with_format(&instance.mask_png, ImageFormat::Png)?.into_luma8();
        if mask.dimensions() != image.dimensions() {
            bail!("实例分割掩码尺寸与彩色图不一致");
        }
        for (pixel, mask_value) in image.pixels_mut().zip(mask.pixels()) {
            if mask_value[0] != 0 {
                for channel in 0..3 {
                    pixel[channel] =
                        ((u16::from(pixel[channel]) * 2 + u16::from(color[channel])) / 3) as u8;
                }
            }
        }
        let [left, top, right, bottom] = instance.bounding_box_xyxy;
        let left = left.round().clamp(0.0, f64::from(image.width() - 1)) as u32;
        let right = right.round().clamp(0.0, f64::from(image.width() - 1)) as u32;
        let top = top.round().clamp(0.0, f64::from(image.height() - 1)) as u32;
        let bottom = bottom.round().clamp(0.0, f64::from(image.height() - 1)) as u32;
        for x in left..=right {
            image.put_pixel(x, top, Rgb(color));
            image.put_pixel(x, bottom, Rgb(color));
        }
        for y in top..=bottom {
            image.put_pixel(left, y, Rgb(color));
            image.put_pixel(right, y, Rgb(color));
        }
    }
    Ok(CameraImagePlane {
        height: image.height(),
        width: image.width(),
        pixel_format: "rgb8".into(),
        frame_id: source.frame_id.clone(),
        stride_bytes: image.width() * 3,
        data: image.into_raw(),
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
        .map(|duration| duration.as_nanos() as i64)
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame_fixture(width: u32, height: u32, depth: Vec<u16>) -> CameraFrameBundle {
        let intrinsics = CameraIntrinsics {
            width,
            height,
            focal_length_px: [1.0, 1.0],
            principal_point_px: [0.0, 0.0],
            distortion_model: "none".into(),
            distortion: vec![],
        };
        CameraFrameBundle {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            source_id: "fixture:camera".into(),
            device_time_ns: 1,
            device_time_domain: "fixture".into(),
            received_time_ns: 2,
            color: CameraImagePlane {
                width,
                height,
                stride_bytes: width * 3,
                pixel_format: "rgb8".into(),
                frame_id: "color".into(),
                data: vec![0; (width * height * 3) as usize],
            },
            depth: CameraImagePlane {
                width,
                height,
                stride_bytes: width * 2,
                pixel_format: "z16le".into(),
                frame_id: "depth".into(),
                data: depth.into_iter().flat_map(u16::to_le_bytes).collect(),
            },
            color_intrinsics: intrinsics.clone(),
            depth_intrinsics: intrinsics,
            depth_to_color: Pose3 {
                position_m: [0.0; 3],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            depth_scale_m: 0.001,
        }
    }

    #[test]
    fn default_configuration_has_no_selected_camera() {
        let config = PerceptionConfig::default();
        assert!(!config.enabled);
        assert_eq!(config.source_id, None);
    }

    #[test]
    fn graspgenx_tcp_pose_is_already_in_base_frame() {
        let pose = candidate_tcp_pose_in_base([
            [1.0, 0.0, 0.0, 0.1],
            [0.0, 1.0, 0.0, 0.2],
            [0.0, 0.0, 1.0, 0.3],
            [0.0, 0.0, 0.0, 1.0],
        ])
        .unwrap();
        assert_eq!(pose.position_m, [0.1, 0.2, 0.3]);
        assert_eq!(pose.orientation_xyzw, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn identity_alignment_preserves_the_atomic_depth_frame() {
        let frame = frame_fixture(2, 2, vec![10, 20, 30, 40]);
        let aligned = align_depth_to_color(&frame).unwrap();
        assert_eq!(aligned.depth, vec![10, 20, 30, 40]);
        assert_eq!(aligned.frame_id, "color");
    }

    #[test]
    fn alignment_applies_depth_to_color_extrinsics() {
        let mut frame = frame_fixture(3, 1, vec![1_000, 0, 0]);
        frame.depth_to_color.position_m = [1.0, 0.0, 0.0];
        let aligned = align_depth_to_color(&frame).unwrap();
        assert_eq!(aligned.depth, vec![0, 1_000, 0]);
        assert_eq!(aligned.frame_id, "color");
    }

    #[test]
    fn brown_conrady_projection_round_trips_without_vendor_types() {
        let intrinsics = CameraIntrinsics {
            width: 640,
            height: 480,
            focal_length_px: [610.0, 608.0],
            principal_point_px: [319.5, 239.5],
            distortion_model: "brown_conrady".into(),
            distortion: vec![0.08, -0.03, 0.001, -0.002, 0.004],
        };
        let point = nalgebra::Vector3::new(0.12, -0.07, 0.8);
        let pixel = project_point(&intrinsics, point).unwrap();
        let recovered = deproject_pixel(&intrinsics, pixel, point.z).unwrap();
        assert!((recovered - point).norm() < 1e-9);
    }

    #[test]
    fn unknown_nonzero_distortion_is_not_silently_ignored() {
        let intrinsics = CameraIntrinsics {
            width: 1,
            height: 1,
            focal_length_px: [1.0, 1.0],
            principal_point_px: [0.0, 0.0],
            distortion_model: "unknown".into(),
            distortion: vec![0.1],
        };
        assert!(deproject_pixel(&intrinsics, [0.0, 0.0], 1.0).is_err());
        assert!(project_point(&intrinsics, nalgebra::Vector3::new(0.0, 0.0, 1.0)).is_err());
    }

    #[test]
    fn calibration_motion_uses_the_model_declared_joint_set() {
        let target = NamedMotionTarget {
            key: "view-a".into(),
            label: "View A".into(),
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
        assert_eq!(request.actuators, vec![]);
        assert_eq!(request.joints[0].joint_key, "axis-a");
        assert_eq!(request.joints[1].position_rad, -0.5);
    }

    #[test]
    fn configuration_round_trips_without_runtime_state() {
        let directory = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../temp/tests");
        std::fs::create_dir_all(&directory).unwrap();
        let path = directory.join(format!(
            "perception-config-{}-{}.json",
            std::process::id(),
            now_ns()
        ));
        let config = PerceptionConfig {
            enabled: true,
            source_id: Some("simulation:pick-place-scene".into()),
            cameras: BTreeMap::from([(
                "simulation:pick-place-scene".into(),
                CameraConfig { calibration: None },
            )]),
            ..Default::default()
        };
        let mut config = config;
        config.calibration_session.active = true;
        config.calibration_session.phase = CalibrationPhase::Moving;
        assert!(config.classes.is_empty());
        assert!(config.placement_labels.is_empty());
        save(&path, &config).unwrap();
        let serialized = std::fs::read_to_string(&path).unwrap();
        assert!(!serialized.contains("calibration_session"));
        let loaded = load_config(&path).unwrap();
        assert_eq!(loaded.source_id, None);
        assert!(
            loaded.cameras["simulation:pick-place-scene"]
                .calibration
                .is_none()
        );
        assert_eq!(loaded.calibration_session, default_calibration_session());
        std::fs::remove_file(path).unwrap();
    }
}
