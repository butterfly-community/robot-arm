mod ros;
mod test_source;

use std::{
    collections::BTreeMap,
    io::Cursor,
    path::{Path, PathBuf},
    process::{Command, Stdio},
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc::{Receiver, channel},
    },
    time::{SystemTime, UNIX_EPOCH},
};

use base64::{Engine, engine::general_purpose::STANDARD as BASE64};
use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result, bail, eyre};
use image::{DynamicImage, ImageFormat, Rgb, RgbImage};
use json_config_store::{load_or_default, save};
use nalgebra::{Isometry3, Matrix3, Quaternion, Rotation3, Translation3, UnitQuaternion};
use perception_core::{
    InstancePointCloud, aligned_obstacle_point_cloud,
    world_scene_and_instance_clouds_from_aligned_depth,
};
use robot_arm_messages::{
    AlignedDepthFrame, ArmState, CalibrationAction, CalibrationObservation, CalibrationRequest,
    CalibrationResult, CalibrationSessionState, DepthCameraCalibration, DepthCameraSourceInfo,
    DetectedInstance2D, ImageFrameInfo, MotionState, PerceptionAssetRequest,
    PerceptionAssetResponse, PerceptionInstanceSummary, PerceptionRequest, PerceptionState, Pose3,
    RequestAction, RequestResult, RobotModelInfo, SCHEMA_VERSION, ServiceState, ToolPose,
    WorldScene, from_arrow, to_arrow,
};
use serde::{Deserialize, Serialize};

use crate::{
    ros::{RosEvent, RosInterface, time_ns},
    test_source::{simulation_frame, simulation_source, simulation_sources},
};

const CONFIG_SCHEMA_VERSION: u32 = 1;
#[derive(Clone, Debug, Serialize, Deserialize)]
struct PerceptionConfig {
    schema_version: u32,
    config_version: u64,
    enabled: bool,
    source_id: Option<String>,
    compute_service_url: String,
    model: String,
    classes: Vec<String>,
    #[serde(default = "default_placement_labels")]
    placement_labels: Vec<String>,
    #[serde(default)]
    cameras: BTreeMap<String, CameraConfig>,
    #[serde(default = "default_calibration_session")]
    calibration_session: CalibrationSessionState,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct CameraConfig {
    #[serde(default = "default_depth_scale_m")]
    depth_scale_m: f64,
    calibration: Option<CalibrationResult>,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            depth_scale_m: default_depth_scale_m(),
            calibration: None,
        }
    }
}

impl Default for PerceptionConfig {
    fn default() -> Self {
        Self {
            schema_version: CONFIG_SCHEMA_VERSION,
            config_version: 0,
            enabled: false,
            source_id: None,
            compute_service_url: "http://perception-compute:8000".into(),
            model: "yoloe-26s-seg.pt".into(),
            classes: vec!["red cube".into(), "gray storage bin".into()],
            placement_labels: default_placement_labels(),
            cameras: BTreeMap::new(),
            calibration_session: default_calibration_session(),
        }
    }
}

fn default_depth_scale_m() -> f64 {
    0.001
}

fn default_placement_labels() -> Vec<String> {
    vec!["gray storage bin".into()]
}

#[derive(Default)]
struct CameraFrames {
    color: Option<r2r::sensor_msgs::msg::Image>,
    depth: Option<r2r::sensor_msgs::msg::Image>,
    depth_info: Option<r2r::sensor_msgs::msg::CameraInfo>,
    driver_calibration: Option<DepthCameraCalibration>,
}

struct PerceptionNode {
    config_path: PathBuf,
    config: PerceptionConfig,
    ros: RosInterface,
    ros_events: Receiver<RosEvent>,
    http: reqwest::blocking::Client,
    frames: CameraFrames,
    sequence: u64,
    last_frame_time_ns: Option<i64>,
    last_scene: Option<WorldScene>,
    available_sources: Vec<DepthCameraSourceInfo>,
    color_frame: Option<ImageFrameInfo>,
    depth_frame: Option<ImageFrameInfo>,
    camera_calibration: Option<DepthCameraCalibration>,
    instances: Vec<PerceptionInstanceSummary>,
    point_count: Option<u64>,
    assets: BTreeMap<String, (String, Vec<u8>)>,
    last_error: Option<String>,
    simulation_published: bool,
    calibration_color: Option<r2r::sensor_msgs::msg::Image>,
    latest_arm_state: Option<ArmState>,
    latest_tool_pose: Option<ToolPose>,
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
    let stop = Arc::new(AtomicBool::new(false));
    let (ros_sender, ros_receiver) = channel();
    let ros = RosInterface::start(ros_sender, Arc::clone(&stop))?;
    let config_path = std::env::var_os("PERCEPTION_CONFIG")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/config/perception.json"));
    let config = load_config(&config_path)?;
    let mut available_sources = simulation_sources()?;
    if let Some(source_id) = config.source_id.as_deref()
        && simulation_source(source_id)?.is_none()
    {
        available_sources.push(RosInterface::camera_source_info(source_id));
        if config.enabled {
            ros.select_camera_id(source_id)?;
        }
    }
    let mut perception = PerceptionNode {
        config_path,
        config,
        ros,
        ros_events: ros_receiver,
        http: reqwest::blocking::Client::new(),
        frames: CameraFrames::default(),
        sequence: 0,
        last_frame_time_ns: None,
        last_scene: None,
        available_sources,
        color_frame: None,
        depth_frame: None,
        camera_calibration: None,
        instances: vec![],
        point_count: None,
        assets: BTreeMap::new(),
        last_error: None,
        simulation_published: false,
        calibration_color: None,
        latest_arm_state: None,
        latest_tool_pose: None,
        robot_model: None,
    };
    perception.refresh_source_config();
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
                "arm_state" => {
                    perception.latest_arm_state = Some(from_arrow(data.as_array())?);
                }
                "motion_state" => {
                    let state: MotionState = from_arrow(data.as_array())?;
                    perception.latest_tool_pose = state.current_tool_pose;
                }
                "robot_model_info" => {
                    let refresh_simulation = if let Some(source_id) =
                        perception.config.source_id.as_deref()
                    {
                        perception.last_scene.is_some() && simulation_source(source_id)?.is_some()
                    } else {
                        false
                    };
                    perception.robot_model = Some(from_arrow(data.as_array())?);
                    if refresh_simulation {
                        perception.simulation_published = false;
                    }
                }
                "snapshot" => perception.publish_snapshot(&mut node)?,
                "tick" => perception.tick(&mut node)?,
                _ => {}
            },
            Event::Stop(_) => break,
            _ => {}
        }
    }
    stop.store(true, Ordering::Relaxed);
    Ok(())
}

impl PerceptionNode {
    fn selected_camera(&self, source_id: &str) -> bool {
        self.config.source_id.as_deref() == Some(source_id)
    }

    fn depth_scale_m(&self, source_id: &str) -> f64 {
        self.config
            .cameras
            .get(source_id)
            .map(|camera| camera.depth_scale_m)
            .or_else(|| {
                self.available_sources
                    .iter()
                    .find(|source| source.source_id == source_id)
                    .map(|source| source.depth_scale_m)
            })
            .unwrap_or_else(default_depth_scale_m)
    }

    fn has_calibration(&self) -> bool {
        let Some(source_id) = self.config.source_id.as_deref() else {
            return false;
        };
        self.frames.driver_calibration.is_some()
            || self
                .config
                .cameras
                .get(source_id)
                .and_then(|camera| camera.calibration.as_ref())
                .is_some()
    }

    fn refresh_source_config(&mut self) {
        for source in &mut self.available_sources {
            if let Some(camera) = self.config.cameras.get(&source.source_id) {
                source.depth_scale_m = camera.depth_scale_m;
                source.calibrated |= camera.calibration.is_some();
            }
        }
    }

    fn discover_sources(&self) -> Result<Vec<DepthCameraSourceInfo>> {
        let mut sources = simulation_sources()?;
        sources.extend(self.ros.discover_cameras()?);
        let mut unique = BTreeMap::new();
        for mut source in sources {
            if let Some(camera) = self.config.cameras.get(&source.source_id) {
                source.depth_scale_m = camera.depth_scale_m;
                source.calibrated |= camera.calibration.is_some();
            }
            unique.insert(source.source_id.clone(), source);
        }
        Ok(unique.into_values().collect())
    }

    fn clear_output(&mut self) -> Result<()> {
        self.simulation_published = false;
        self.frames = CameraFrames::default();
        self.last_scene = None;
        self.color_frame = None;
        self.depth_frame = None;
        self.camera_calibration = None;
        self.instances.clear();
        self.point_count = None;
        self.last_frame_time_ns = None;
        self.assets.clear();
        self.calibration_color = None;
        self.ros.clear_markers()
    }

    fn tick(&mut self, node: &mut DoraNode) -> Result<()> {
        while let Ok(event) = self.ros_events.try_recv() {
            if let Err(error) = self.handle_ros_event(node, event) {
                self.last_error = Some(error.to_string());
            }
        }
        if self.config.enabled && !self.simulation_published {
            let source_id = self.config.source_id.clone().unwrap_or_default();
            match simulation_frame(
                &source_id,
                self.sequence + 1,
                now_ns(),
                self.depth_scale_m(&source_id),
            ) {
                Ok(Some(frame)) => {
                    self.frames.driver_calibration = Some(frame.calibration);
                    let result = self
                        .handle_ros_event(node, RosEvent::Color(source_id.clone(), frame.color))
                        .and_then(|()| {
                            self.handle_ros_event(
                                node,
                                RosEvent::Depth(source_id.clone(), frame.depth),
                            )
                        })
                        .and_then(|()| {
                            self.handle_ros_event(
                                node,
                                RosEvent::DepthInfo(source_id, frame.camera_info),
                            )
                        });
                    match result {
                        Ok(()) => {
                            self.simulation_published = true;
                            self.last_error = None;
                        }
                        Err(error) => self.last_error = Some(error.to_string()),
                    }
                }
                Ok(None) => {}
                Err(error) => self.last_error = Some(error.to_string()),
            }
        }
        self.publish_state(node)
    }

    fn handle_ros_event(&mut self, node: &mut DoraNode, event: RosEvent) -> Result<()> {
        if !self.config.enabled {
            return Ok(());
        }
        match event {
            RosEvent::Color(source_id, message) if self.selected_camera(&source_id) => {
                self.ros.publish_color(message.clone())?;
                self.calibration_color = Some(message.clone());
                self.frames.color = Some(message);
            }
            RosEvent::Depth(source_id, message) if self.selected_camera(&source_id) => {
                self.ros.publish_depth(message.clone())?;
                self.frames.depth = Some(message);
            }
            RosEvent::DepthInfo(source_id, message) if self.selected_camera(&source_id) => {
                self.frames.depth_info = Some(message);
            }
            _ => return Ok(()),
        }
        if self.frames.color.is_some()
            && self.frames.depth.is_some()
            && self.frames.depth_info.is_some()
            && self.has_calibration()
        {
            self.process_camera_scene(node)?;
        }
        Ok(())
    }

    fn process_camera_scene(&mut self, node: &mut DoraNode) -> Result<()> {
        let color = self.frames.color.take().expect("checked color frame");
        let depth = self.frames.depth.take().expect("checked depth frame");
        let info = self.frames.depth_info.as_ref().expect("checked depth info");
        let source_id = self
            .config
            .source_id
            .as_deref()
            .ok_or_else(|| eyre!("尚未选择深度相机"))?;
        let mut aligned_depth = decode_depth(&depth, source_id, self.depth_scale_m(source_id))?;
        let mut calibration = self
            .frames
            .driver_calibration
            .clone()
            .map(Ok)
            .unwrap_or_else(|| {
                camera_calibration(
                    info,
                    source_id,
                    self.config
                        .cameras
                        .get(source_id)
                        .and_then(|camera| camera.calibration.as_ref()),
                )
            })?;
        self.sequence += 1;
        aligned_depth.sequence = self.sequence;
        calibration.sequence = self.sequence;
        self.ros.publish_calibration(&calibration)?;
        let instances = self.segment(&color)?;
        let overlay = segmentation_debug_image(&color, &instances)?;
        self.ros.publish_segmentation(overlay.clone())?;
        let cloud =
            aligned_obstacle_point_cloud(self.sequence, &aligned_depth, &calibration, &instances)?;
        self.point_count = Some(cloud.points_xyz_m.len() as u64);
        self.ros.publish_cloud(cloud)?;
        let (mut scene, instance_clouds) = world_scene_and_instance_clouds_from_aligned_depth(
            self.sequence,
            &aligned_depth,
            &calibration,
            &instances,
            &self.config.placement_labels,
        )?;
        self.attach_grasp_candidates(&mut scene, &instance_clouds)?;
        self.store_frame_details(
            &color,
            &aligned_depth,
            &overlay,
            &calibration,
            &instances,
            &scene,
        )?;
        self.last_frame_time_ns = Some(aligned_depth.source_time_ns);
        self.publish_scene(node, scene)
    }

    fn segment(&self, color: &r2r::sensor_msgs::msg::Image) -> Result<Vec<DetectedInstance2D>> {
        let png = color_png(color)?;
        let response = self
            .http
            .post(format!(
                "{}/v1/segment",
                self.config.compute_service_url.trim_end_matches('/')
            ))
            .json(&SegmentRequest {
                image_base64: BASE64.encode(png),
                classes: &self.config.classes,
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
        &self,
        scene: &mut WorldScene,
        instance_clouds: &[InstancePointCloud],
    ) -> Result<()> {
        let Some(descriptor_id) = self
            .robot_model
            .as_ref()
            .and_then(|model| model.gripper_asset_id.as_deref())
        else {
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
            let mut candidates = self
                .http
                .post(format!(
                    "{}/v1/grasps",
                    self.config.compute_service_url.trim_end_matches('/')
                ))
                .json(&GraspRequest {
                    points_xyz_m: &cloud.points_xyz_m,
                    gripper_asset_id: descriptor_id,
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

    fn publish_scene(&mut self, node: &mut DoraNode, scene: WorldScene) -> Result<()> {
        self.ros.publish_markers(&scene)?;
        send(node, "world_scene", &scene)?;
        self.last_scene = Some(scene);
        Ok(())
    }

    fn store_frame_details(
        &mut self,
        color: &r2r::sensor_msgs::msg::Image,
        depth: &AlignedDepthFrame,
        overlay: &r2r::sensor_msgs::msg::Image,
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
                next.enabled = true;
                if let Some(source_id) = request.source_id.clone() {
                    next.source_id = Some(source_id);
                }
                if let Some(classes) = request.classes {
                    next.classes = classes;
                }
                if let Some(labels) = request.placement_labels {
                    next.placement_labels = labels;
                }
                let source_id = next
                    .source_id
                    .as_deref()
                    .ok_or_else(|| eyre!("请选择深度相机来源"))?;
                if let Some(depth_scale_m) = request.depth_scale_m {
                    next.cameras
                        .entry(source_id.into())
                        .or_default()
                        .depth_scale_m = depth_scale_m;
                }
                if simulation_source(source_id)?.is_none() {
                    self.ros.select_camera_id(source_id)?;
                }
                Ok(())
            }
            RequestAction::Refresh => {
                self.available_sources = self.discover_sources()?;
                Ok(())
            }
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
                next.source_id = None;
                Ok(())
            }
            action => Err(eyre!(
                "perception request action {action:?} is not supported"
            )),
        };
        let persist = request.action != RequestAction::Refresh;
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
            self.available_sources = self.discover_sources()?;
            self.clear_output()?;
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

    fn apply_calibration_request(
        &mut self,
        node: &mut DoraNode,
        request: CalibrationRequest,
    ) -> Result<()> {
        let result = self.update_calibration(&request);
        let error = result.err().map(|error| error.to_string());
        self.config.calibration_session.original_error = error.clone();
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

    fn update_calibration(&mut self, request: &CalibrationRequest) -> Result<()> {
        match request.action {
            CalibrationAction::Start => {
                self.config.calibration_session = CalibrationSessionState {
                    schema_version: SCHEMA_VERSION,
                    active: true,
                    board: Some(
                        request
                            .board
                            .clone()
                            .ok_or_else(|| eyre!("开始标定需要标定板实测参数"))?,
                    ),
                    camera_source_id: Some(
                        request
                            .camera_source_id
                            .clone()
                            .or_else(|| self.config.source_id.clone())
                            .ok_or_else(|| eyre!("开始标定需要相机来源"))?,
                    ),
                    robot_model_revision: Some(
                        request
                            .robot_model_revision
                            .clone()
                            .or_else(|| {
                                self.latest_arm_state
                                    .as_ref()
                                    .map(|state| state.model_revision.clone())
                            })
                            .ok_or_else(|| eyre!("开始标定需要机械臂型号反馈"))?,
                    ),
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
            }
            CalibrationAction::Capture => {
                let observation = self.capture_calibration_observation(&request.request_id)?;
                self.config
                    .calibration_session
                    .observations
                    .push(observation);
                self.config.calibration_session.solved_result = None;
            }
            CalibrationAction::Solve => {
                let solved = self.solve_calibration()?;
                self.config.calibration_session.solved_result = Some(solved);
            }
            CalibrationAction::Apply => {
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
                if let Some(info) = self.frames.depth_info.as_ref() {
                    self.ros.publish_calibration(&camera_calibration(
                        info,
                        &source_id,
                        Some(&solved),
                    )?)?;
                }
                self.refresh_source_config();
            }
            CalibrationAction::Cancel => {
                self.config.calibration_session = default_calibration_session();
            }
        }
        Ok(())
    }

    fn capture_calibration_observation(&self, sample_id: &str) -> Result<CalibrationObservation> {
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
            .frames
            .depth_info
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
            "camera_matrix": info.k,
            "distortion": info.d,
            "board": board,
        }))?;
        self.ros.publish_calibration_debug(png_image(
            &BASE64.decode(&detected.visualization_png_base64)?,
            image.header.clone(),
        )?)?;
        Ok(CalibrationObservation {
            sample_id: sample_id.into(),
            sample_time_ns: arm.sample_time_ns,
            camera_frame_id: info.header.frame_id.clone(),
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
            model: self.config.model.clone(),
            classes: self.config.classes.clone(),
            placement_labels: self.config.placement_labels.clone(),
            available_sources: self.available_sources.clone(),
            color_frame: self.color_frame.clone(),
            depth_frame: self.depth_frame.clone(),
            camera_calibration: self.camera_calibration.clone(),
            depth_scale_m: self
                .config
                .source_id
                .as_deref()
                .map(|source_id| self.depth_scale_m(source_id))
                .unwrap_or_else(default_depth_scale_m),
            instances: self.instances.clone(),
            point_count: self.point_count,
            last_frame_time_ns: self.last_frame_time_ns,
            last_scene_sequence: self.last_scene.as_ref().map(|scene| scene.sequence),
            calibrated: self
                .config
                .source_id
                .as_deref()
                .and_then(|source_id| {
                    self.available_sources
                        .iter()
                        .find(|source| source.source_id == source_id)
                })
                .is_some_and(|source| source.calibrated),
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
    info: &r2r::sensor_msgs::msg::CameraInfo,
    source_id: &str,
    result: Option<&CalibrationResult>,
) -> Result<DepthCameraCalibration> {
    let result = result.ok_or_else(|| eyre!("相机尚未完成外参标定"))?;
    Ok(DepthCameraCalibration {
        schema_version: SCHEMA_VERSION,
        sequence: 0,
        source_time_ns: time_ns(&info.header.stamp),
        source_id: source_id.into(),
        parent_frame_id: "base_link".into(),
        frame_id: info.header.frame_id.clone(),
        translation_m: result.camera_in_base.position_m,
        orientation_xyzw: result.camera_in_base.orientation_xyzw,
        width: info.width,
        height: info.height,
        distortion_model: info.distortion_model.clone(),
        distortion: info.d.clone(),
        camera_matrix: info
            .k
            .clone()
            .try_into()
            .map_err(|_| eyre!("CameraInfo K 长度错误"))?,
        projection_matrix: info
            .p
            .clone()
            .try_into()
            .map_err(|_| eyre!("CameraInfo P 长度错误"))?,
    })
}

fn decode_depth(
    message: &r2r::sensor_msgs::msg::Image,
    source_id: &str,
    depth_scale_m: f64,
) -> Result<AlignedDepthFrame> {
    if message.encoding != "16UC1" && message.encoding != "mono16" {
        bail!("不支持的对齐深度编码 {}", message.encoding);
    }
    let depth = message
        .data
        .chunks_exact(2)
        .map(|bytes| {
            if message.is_bigendian == 0 {
                u16::from_le_bytes([bytes[0], bytes[1]])
            } else {
                u16::from_be_bytes([bytes[0], bytes[1]])
            }
        })
        .collect::<Vec<_>>();
    Ok(AlignedDepthFrame {
        schema_version: SCHEMA_VERSION,
        sequence: 0,
        source_time_ns: time_ns(&message.header.stamp),
        source_id: source_id.into(),
        frame_id: message.header.frame_id.clone(),
        width: message.width,
        height: message.height,
        depth_scale_m,
        depth,
    })
}

fn color_rgb(message: &r2r::sensor_msgs::msg::Image) -> Result<RgbImage> {
    let mut rgb = RgbImage::new(message.width, message.height);
    match message.encoding.as_str() {
        "rgb8" | "bgr8" => {
            for (pixel, source) in rgb.pixels_mut().zip(message.data.chunks_exact(3)) {
                *pixel = if message.encoding == "rgb8" {
                    Rgb([source[0], source[1], source[2]])
                } else {
                    Rgb([source[2], source[1], source[0]])
                };
            }
        }
        other => bail!("不支持的彩色图编码 {other}"),
    }
    Ok(rgb)
}

fn color_png(message: &r2r::sensor_msgs::msg::Image) -> Result<Vec<u8>> {
    let rgb = color_rgb(message)?;
    let mut output = Cursor::new(Vec::new());
    DynamicImage::ImageRgb8(rgb).write_to(&mut output, ImageFormat::Png)?;
    Ok(output.into_inner())
}

fn image_frame_info(message: &r2r::sensor_msgs::msg::Image) -> ImageFrameInfo {
    ImageFrameInfo {
        width: message.width,
        height: message.height,
        encoding: message.encoding.clone(),
        frame_id: message.header.frame_id.clone(),
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
    source: &r2r::sensor_msgs::msg::Image,
    instances: &[DetectedInstance2D],
) -> Result<r2r::sensor_msgs::msg::Image> {
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
    Ok(r2r::sensor_msgs::msg::Image {
        header: source.header.clone(),
        height: image.height(),
        width: image.width(),
        encoding: "rgb8".into(),
        is_bigendian: 0,
        step: image.width() * 3,
        data: image.into_raw(),
    })
}

fn png_image(
    encoded: &[u8],
    header: r2r::std_msgs::msg::Header,
) -> Result<r2r::sensor_msgs::msg::Image> {
    let image = image::load_from_memory_with_format(encoded, ImageFormat::Png)?.into_rgb8();
    Ok(r2r::sensor_msgs::msg::Image {
        header,
        height: image.height(),
        width: image.width(),
        encoding: "rgb8".into(),
        is_bigendian: 0,
        step: image.width() * 3,
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
    fn configuration_round_trips_without_runtime_state() {
        let path = std::env::temp_dir().join(format!(
            "perception-config-{}-{}.json",
            std::process::id(),
            now_ns()
        ));
        let config = PerceptionConfig {
            enabled: true,
            source_id: Some("simulation:pick-place-scene".into()),
            cameras: BTreeMap::from([(
                "simulation:pick-place-scene".into(),
                CameraConfig {
                    depth_scale_m: 0.002,
                    calibration: None,
                },
            )]),
            ..Default::default()
        };
        assert_eq!(config.classes, ["red cube", "gray storage bin"]);
        assert_eq!(config.placement_labels, ["gray storage bin"]);
        save(&path, &config).unwrap();
        let loaded = load_config(&path).unwrap();
        assert_eq!(loaded.source_id, config.source_id);
        assert_eq!(
            loaded.cameras["simulation:pick-place-scene"].depth_scale_m,
            0.002
        );
        std::fs::remove_file(path).unwrap();
    }
}
