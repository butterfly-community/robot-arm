use std::{
    collections::BTreeMap,
    io::Cursor,
    path::{Path, PathBuf},
    sync::Arc,
    time::{SystemTime, UNIX_EPOCH},
};

use base64::{Engine, engine::general_purpose::STANDARD as BASE64};
use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result, bail, eyre};
use image::{DynamicImage, ImageFormat, Rgb, RgbImage};
use json_config_store::{load_or_default, save};
use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion};
use robot_arm_messages::{
    AlignedDepthFrame, CameraCaptureState, CameraFrameBundle, CameraImagePlane,
    DepthCameraCalibration, DetectedInstance2D, GraspCandidate, ImageFrameInfo, MotionState,
    PerceptionAssetRequest, PerceptionAssetResponse, PerceptionInstanceSummary,
    PerceptionModelInfo, PerceptionRequest, PerceptionState, Pose3, RequestAction, RequestResult,
    RequestState, RobotModelInfo, SCHEMA_VERSION, SegmentationPrompt, ServiceState,
    ToolPoseFeedback, WorldScene, camera_frame_from_arrow, from_arrow, to_arrow,
};
use scene_core::{InstancePointCloud, world_scene_and_instance_clouds_from_aligned_depth};
use serde::{Deserialize, Serialize};

#[cfg(test)]
mod stage_tests;

const CONFIG_SCHEMA_VERSION: u32 = 2;
fn default_grasp_collision_distance_m() -> f64 {
    0.02 // GraspGenX demo_scene_pc default; a proximity distance, not mesh inflation.
}
#[derive(Clone, Debug, Serialize, Deserialize)]
struct SceneConfig {
    schema_version: u32,
    config_version: u64,
    enabled: bool,
    #[serde(skip)]
    source_id: Option<String>,
    compute_service_url: String,
    #[serde(default)]
    model: Option<String>,
    classes: Vec<String>,
    #[serde(default)]
    prompt: SegmentationPrompt,
    #[serde(default)]
    placement_labels: Vec<String>,
    #[serde(default = "default_grasp_collision_distance_m")]
    grasp_collision_distance_m: f64,
}

impl Default for SceneConfig {
    fn default() -> Self {
        Self {
            schema_version: CONFIG_SCHEMA_VERSION,
            config_version: 0,
            enabled: false,
            source_id: None,
            compute_service_url: "http://perception-compute:8000".into(),
            model: None,
            classes: Vec::new(),
            prompt: SegmentationPrompt::Text,
            placement_labels: Vec::new(),
            grasp_collision_distance_m: default_grasp_collision_distance_m(),
        }
    }
}

struct SceneNode {
    config_path: PathBuf,
    config: SceneConfig,
    http: reqwest::Client,
    model: String,
    models: Vec<PerceptionModelInfo>,
    catalog_updates: tokio::sync::watch::Receiver<Option<Result<ModelResponse, String>>>,
    latest_bundle: Option<CameraFrameBundle>,
    frame_updates: tokio::sync::watch::Sender<Option<CameraFrameBundle>>,
    tool_updates: tokio::sync::watch::Sender<Option<ToolPoseFeedback>>,
    camera_state: Option<CameraCaptureState>,
    sequence: u64,
    last_frame_time_ns: Option<i64>,
    last_scene: Option<WorldScene>,
    segmented: Option<Arc<SegmentedFrame>>,
    instance_clouds: Arc<Vec<InstancePointCloud>>,
    color_frame: Option<ImageFrameInfo>,
    depth_frame: Option<ImageFrameInfo>,
    camera_calibration: Option<DepthCameraCalibration>,
    instances: Vec<PerceptionInstanceSummary>,
    point_count: Option<u64>,
    assets: BTreeMap<String, (String, Vec<u8>)>,
    last_error: Option<String>,
    robot_model: Option<RobotModelInfo>,
    task_request_id: Option<String>,
    task_state: RequestState,
    task_action: Option<RequestAction>,
    task_results: tokio::sync::mpsc::Receiver<SceneTaskResult>,
    task_sender: tokio::sync::mpsc::Sender<SceneTaskResult>,
}

struct SegmentedFrame {
    sequence: u64,
    model: String,
    models: Vec<PerceptionModelInfo>,
    frame: CameraFrameBundle,
    tool: Option<ToolPoseFeedback>,
    instances: Vec<DetectedInstance2D>,
    placement_labels: Vec<String>,
    assets: BTreeMap<String, (String, Vec<u8>)>,
}

enum ProcessedStage {
    Segmentation(Arc<SegmentedFrame>),
    Reconstruction(WorldScene, Arc<Vec<InstancePointCloud>>),
    Grasps(WorldScene),
}

struct SceneTaskResult {
    request_id: String,
    input_sequence: u64,
    action: RequestAction,
    result: Result<ProcessedStage, String>,
}

async fn next_observation_frame(
    frames: &mut tokio::sync::watch::Receiver<Option<CameraFrameBundle>>,
    requested_at: i64,
) -> Result<CameraFrameBundle> {
    loop {
        frames.changed().await.context("相机帧通道已关闭")?;
        let frame = frames
            .borrow_and_update()
            .clone()
            .ok_or_else(|| eyre!("等待新帧期间相机输入已清除"))?;
        if frame.received_time_ns > requested_at {
            return Ok(frame);
        }
    }
}

async fn feedback_after_frame(
    tools: &mut tokio::sync::watch::Receiver<Option<ToolPoseFeedback>>,
    received_time_ns: i64,
) -> Result<ToolPoseFeedback> {
    loop {
        if let Some(tool) = tools.borrow_and_update().clone()
            && tool.arm_state.sample_time_ns >= received_time_ns
        {
            return Ok(tool);
        }
        tools.changed().await.context("实际 TCP 反馈通道已关闭")?;
    }
}

#[derive(Serialize)]
struct ObservedGripper {
    tcp_pose: Pose3,
    joint_positions_rad: BTreeMap<String, f64>,
    feedback_time_ns: i64,
}

fn observed_gripper(
    model: &RobotModelInfo,
    tool: &ToolPoseFeedback,
    frame: &str,
) -> Result<ObservedGripper> {
    if tool.pose.frame != frame || tool.arm_state.model_revision != model.model_revision {
        bail!("实际夹爪反馈坐标系或模型版本与场景不一致");
    }
    let mut joints = BTreeMap::new();
    for (index, actuator) in model.tool_actuators.iter().enumerate() {
        if let Some(key) = &actuator.visualization_joint_key {
            let value = tool
                .arm_state
                .actuators_rad
                .get(index)
                .ok_or_else(|| eyre!("夹爪实际关节反馈缺失"))?;
            joints.insert(key.clone(), *value);
        }
    }
    Ok(ObservedGripper {
        tcp_pose: Pose3 {
            position_m: tool.pose.position_m,
            orientation_xyzw: tool.pose.orientation_xyzw,
        },
        joint_positions_rad: joints,
        feedback_time_ns: tool.arm_state.sample_time_ns,
    })
}

#[derive(Serialize)]
struct SegmentRequest<'a> {
    model: &'a str,
    image_base64: String,
    classes: &'a [String],
    prompt: &'a SegmentationPrompt,
}

#[derive(Deserialize)]
struct SegmentResponse {
    instances: Vec<SegmentInstance>,
}

#[derive(Clone, Deserialize)]
struct ModelResponse {
    model: String,
    models: Vec<PerceptionModelInfo>,
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
    scene_points_xyz_m: &'a [[f32; 3]],
    gripper_asset_id: &'a str,
    collision_threshold_m: f64,
    observed_gripper: Option<&'a ObservedGripper>,
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

fn main() {
    if let Err(error) = run() {
        eprintln!("scene-node: {error:?}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let config_path = std::env::var_os("SCENE_CONFIG")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/config/scene.json"));
    let config = load_config(&config_path)?;
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .context("创建 scene-node Tokio runtime")?;
    let http = reqwest::Client::new();
    let (catalog_sender, catalog_updates) = tokio::sync::watch::channel(None);
    let catalog_http = http.clone();
    let catalog_url = config.compute_service_url.clone();
    runtime.spawn(async move {
        let result = model_catalog(&catalog_http, &catalog_url)
            .await
            .map_err(|error| format!("{error:#}"));
        catalog_sender.send_replace(Some(result));
    });
    let (task_sender, task_results) = tokio::sync::mpsc::channel(1);
    let mut scene_node = SceneNode {
        config_path,
        config,
        http,
        model: "尚未查询".into(),
        models: vec![],
        catalog_updates,
        latest_bundle: None,
        frame_updates: tokio::sync::watch::channel(None).0,
        tool_updates: tokio::sync::watch::channel(None).0,
        camera_state: None,
        sequence: 0,
        last_frame_time_ns: None,
        last_scene: None,
        segmented: None,
        instance_clouds: Arc::new(vec![]),
        color_frame: None,
        depth_frame: None,
        camera_calibration: None,
        instances: vec![],
        point_count: None,
        assets: BTreeMap::new(),
        last_error: None,
        robot_model: None,
        task_request_id: None,
        task_state: RequestState::Idle,
        task_action: None,
        task_results,
        task_sender,
    };
    let (mut node, mut events) = DoraNode::init_from_env()?;
    scene_node.publish_snapshot(&mut node)?;
    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => match id.as_str() {
                "request" | "manipulation_observation" => {
                    scene_node.apply_request(&runtime, &mut node, from_arrow(data.as_array())?)?
                }
                "asset_request" => {
                    scene_node.send_asset(&mut node, from_arrow(data.as_array())?)?
                }
                "camera_frame" => {
                    let frame = camera_frame_from_arrow(data.as_array())?;
                    if let Err(error) = scene_node.handle_camera_frame(frame) {
                        scene_node.last_error = Some(format!("{error:#}"));
                    }
                }
                "camera_state" => {
                    let state: CameraCaptureState = from_arrow(data.as_array())?;
                    let active_source = state
                        .streaming
                        .then(|| state.selected_source_id.clone())
                        .flatten();
                    let changed = scene_node.config.source_id != active_source
                        || scene_node.camera_state.as_ref().is_some_and(|previous| {
                            previous.service.config_version != state.service.config_version
                        });
                    scene_node.config.source_id = active_source;
                    scene_node.camera_state = Some(state);
                    if changed {
                        scene_node.clear_output(&mut node)?;
                    }
                }
                "robot_model_info" => {
                    let model: RobotModelInfo = from_arrow(data.as_array())?;
                    if scene_node.robot_model.as_ref().is_some_and(|previous| {
                        previous.gripper_asset_id != model.gripper_asset_id
                            || previous.model_revision != model.model_revision
                    }) {
                        scene_node.clear_scene(&mut node)?;
                    }
                    scene_node.robot_model = Some(model);
                }
                "motion_state" => {
                    let state: MotionState = from_arrow(data.as_array())?;
                    scene_node
                        .tool_updates
                        .send_replace(state.current_tool_pose);
                }
                "snapshot" => scene_node.publish_snapshot(&mut node)?,
                "tick" => scene_node.tick(&mut node)?,
                _ => {}
            },
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}

impl SceneNode {
    fn selected_camera(&self, source_id: &str) -> bool {
        self.config.source_id.as_deref() == Some(source_id)
    }

    fn clear_output(&mut self, node: &mut DoraNode) -> Result<()> {
        let frame_id = self
            .last_scene
            .as_ref()
            .map(|scene| scene.frame_id.clone())
            .unwrap_or_default();
        self.latest_bundle = None;
        self.frame_updates.send_replace(None);
        self.last_scene = None;
        self.segmented = None;
        self.instance_clouds = Arc::new(vec![]);
        self.color_frame = None;
        self.depth_frame = None;
        self.camera_calibration = None;
        self.instances.clear();
        self.point_count = None;
        self.last_frame_time_ns = None;
        self.assets.clear();
        if self.task_state != RequestState::Executing {
            self.task_request_id = None;
            self.task_state = RequestState::Idle;
        }
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
                point_cloud: None,
            },
        )
    }

    fn clear_scene(&mut self, node: &mut DoraNode) -> Result<()> {
        let frame_id = self
            .last_scene
            .as_ref()
            .map(|scene| scene.frame_id.clone())
            .unwrap_or_else(|| "base_link".into());
        self.last_scene = None;
        self.segmented = None;
        self.instance_clouds = Arc::new(vec![]);
        self.instances.clear();
        self.point_count = None;
        self.assets
            .retain(|key, _| key != "overlay.png" && !key.starts_with("mask-"));
        self.sequence += 1;
        send(
            node,
            "world_scene",
            &WorldScene {
                schema_version: SCHEMA_VERSION,
                sequence: self.sequence,
                sample_time_ns: now_ns(),
                frame_id,
                objects: vec![],
                placement_regions: vec![],
                obstacles: vec![],
                point_cloud: None,
            },
        )
    }

    fn tick(&mut self, node: &mut DoraNode) -> Result<()> {
        let catalog = self.catalog_updates.borrow_and_update().clone();
        if let Some(Ok(catalog)) = catalog {
            self.models = catalog.models;
            if self.config.model.is_none() {
                self.model = catalog.model;
            }
        }
        while let Ok(completed) = self.task_results.try_recv() {
            self.finish_scene_task(node, completed)?;
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
        self.color_frame = Some(image_frame_info(&frame.color));
        self.depth_frame = Some(image_frame_info(&frame.aligned_depth));
        self.last_frame_time_ns = Some(frame.received_time_ns);
        self.latest_bundle = Some(frame.clone());
        self.camera_calibration = frame.calibration.clone();
        self.frame_updates.send_replace(Some(frame));
        Ok(())
    }

    fn start_scene_task(
        &mut self,
        runtime: &tokio::runtime::Runtime,
        request: &PerceptionRequest,
    ) -> Result<()> {
        if self.task_state == RequestState::Executing {
            bail!("感知任务正在执行");
        }
        validate_stage_input(request, self.segmented.as_deref(), self.last_scene.as_ref())?;
        if request.action == RequestAction::Refresh && self.latest_bundle.is_none() {
            bail!("尚未收到相机帧");
        }
        let mut frames = self.frame_updates.subscribe();
        let mut tools = self.tool_updates.subscribe();
        let requested_at = now_ns();
        if !self.config.enabled {
            let mut next = self.config.clone();
            next.enabled = true;
            next.config_version += 1;
            save(&self.config_path, &next)?;
            self.config = next;
        }
        let http = self.http.clone();
        let config = self.config.clone();
        let robot_model = self.robot_model.clone();
        let sequence = self.sequence + 1;
        let sender = self.task_sender.clone();
        let completed_request_id = request.request_id.clone();
        let input_sequence = self.sequence;
        let action = request.action;
        let segmented = self.segmented.clone();
        let scene = self.last_scene.clone();
        let clouds = self.instance_clouds.clone();
        let object_id = request.object_id.clone();
        runtime.spawn(async move {
            let result = async {
                match action {
                    RequestAction::Refresh => {
                        let frame = next_observation_frame(&mut frames, requested_at).await?;
                        // Pair feedback concurrently for later grasp self filtering.
                        // The segmentation result is never delayed by missing feedback.
                        let received_time = frame.received_time_ns;
                        let mut work = Box::pin(segment_frame(http, config, frame, None, sequence));
                        let segmented = tokio::select! {
                            result = &mut work => result?,
                            tool = feedback_after_frame(&mut tools, received_time) => {
                                let mut result = work.await?;
                                result.tool = tool.ok();
                                result
                            }
                        };
                        Ok(ProcessedStage::Segmentation(Arc::new(segmented)))
                    }
                    RequestAction::Reconstruct => {
                        let segmented = segmented.expect("validated segmented input");
                        tokio::task::spawn_blocking(move || reconstruct_frame(&segmented, sequence))
                            .await
                            .context("三维定位任务异常结束")?
                    }
                    RequestAction::GenerateGrasps => {
                        let segmented = segmented.expect("validated segmented input");
                        let mut scene = scene.expect("validated reconstructed input");
                        let model = robot_model.ok_or_else(|| eyre!("尚未收到机械臂模型"))?;
                        let asset = model
                            .gripper_asset_id
                            .as_deref()
                            .ok_or_else(|| eyre!("机械臂未配置抓取模型夹爪资产"))?;
                        let tool = segmented.tool.as_ref().ok_or_else(|| {
                            eyre!("该分割帧缺少同步夹爪反馈，请连接机械臂后重新分割")
                        })?;
                        let gripper = observed_gripper(&model, tool, &scene.frame_id)?;
                        attach_grasp_candidates(
                            &http,
                            &config,
                            asset,
                            Some(&gripper),
                            object_id.as_deref().expect("validated selected object"),
                            &mut scene,
                            &clouds,
                        )
                        .await?;
                        scene.sequence = sequence;
                        Ok(ProcessedStage::Grasps(scene))
                    }
                    _ => unreachable!("validated stage"),
                }
            }
            .await
            .map_err(|error: eyre::Report| format!("{error:#}"));
            let _ = sender
                .send(SceneTaskResult {
                    request_id: completed_request_id,
                    input_sequence,
                    action,
                    result,
                })
                .await;
        });
        self.task_request_id = Some(request.request_id.clone());
        self.task_action = Some(action);
        self.task_state = RequestState::Executing;
        self.last_error = None;
        Ok(())
    }

    fn finish_scene_task(&mut self, node: &mut DoraNode, completed: SceneTaskResult) -> Result<()> {
        // Clearing inputs increments the scene sequence, including A → B → A
        // source switches and calibration changes on the same physical camera.
        let result = if self.sequence == completed.input_sequence {
            completed.result
        } else {
            Err("感知任务运行期间相机、标定、模型或配置已改变，已丢弃旧结果".into())
        };
        let error = match result {
            Ok(processed) => {
                match processed {
                    ProcessedStage::Segmentation(segmented) => {
                        self.clear_scene(node)?;
                        self.sequence = segmented.sequence;
                        self.model = segmented.model.clone();
                        self.models = segmented.models.clone();
                        self.assets = segmented.assets.clone();
                        self.segmented = Some(segmented);
                    }
                    ProcessedStage::Reconstruction(scene, clouds) => {
                        self.point_count = Some(
                            clouds
                                .iter()
                                .map(|cloud| cloud.points_xyz_m.len() as u64)
                                .sum(),
                        );
                        self.instance_clouds = clouds;
                        self.sequence = scene.sequence;
                        self.publish_scene(node, scene)?;
                    }
                    ProcessedStage::Grasps(scene) => {
                        self.sequence = scene.sequence;
                        self.publish_scene(node, scene)?;
                    }
                }
                self.update_instance_summaries();
                self.task_state = RequestState::Succeeded;
                None
            }
            Err(error) => {
                self.task_state = RequestState::Failed;
                Some(error)
            }
        };
        self.last_error = error.clone();
        self.task_request_id = None;
        send(
            node,
            "request_result",
            &RequestResult {
                schema_version: SCHEMA_VERSION,
                request_id: completed.request_id,
                acknowledged_action: completed.action,
                value: Some(self.state()),
                original_error: error,
            },
        )?;
        Ok(())
    }

    fn publish_scene(&mut self, node: &mut DoraNode, scene: WorldScene) -> Result<()> {
        send_scene(node, &scene)?;
        self.last_scene = Some(scene);
        Ok(())
    }

    fn update_instance_summaries(&mut self) {
        let Some(segmented) = &self.segmented else {
            return;
        };
        self.instances = segmented
            .instances
            .iter()
            .enumerate()
            .map(|(index, instance)| {
                let object = self
                    .last_scene
                    .as_ref()
                    .into_iter()
                    .flat_map(|scene| &scene.objects)
                    .find(|object| object.object_id == instance.instance_id);
                PerceptionInstanceSummary {
                    instance_id: instance.instance_id.clone(),
                    label: instance.label.clone(),
                    confidence: instance.confidence,
                    bounding_box_xyxy: instance.bounding_box_xyxy,
                    mask_asset_key: format!("mask-{index}.png"),
                    position_m: object.map(|object| object.pose.position_m),
                    size_m: object.map(|object| object.size_m),
                    grasp_candidate_count: object
                        .map(|object| object.grasp_candidates.len() as u32)
                        .unwrap_or_default(),
                }
            })
            .collect();
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

    fn apply_request(
        &mut self,
        runtime: &tokio::runtime::Runtime,
        node: &mut DoraNode,
        request: PerceptionRequest,
    ) -> Result<()> {
        if matches!(
            request.action,
            RequestAction::Refresh | RequestAction::Reconstruct | RequestAction::GenerateGrasps
        ) {
            if let Err(error) = self.start_scene_task(runtime, &request) {
                let error = error.to_string();
                if self.task_state != RequestState::Executing {
                    self.task_state = RequestState::Failed;
                    self.last_error = Some(error.clone());
                }
                send(
                    node,
                    "request_result",
                    &RequestResult {
                        schema_version: SCHEMA_VERSION,
                        request_id: request.request_id,
                        acknowledged_action: request.action,
                        value: Some(self.state()),
                        original_error: Some(error),
                    },
                )?;
            }
            return self.publish_state(node);
        }
        let mut next = self.config.clone();
        let result = (|| match request.action {
            RequestAction::Apply => {
                if let Some(model) = request.model {
                    if !self.models.iter().any(|item| item.id == model) {
                        return Err(eyre!("未知分割模型 {model}"));
                    }
                    next.model = Some(model);
                }
                if let Some(classes) = request.classes {
                    next.classes = classes;
                    // Editing text classes explicitly restores text prompting.
                    next.prompt = SegmentationPrompt::Text;
                }
                if let Some(prompt) = request.prompt {
                    next.prompt = prompt;
                }
                if let Some(labels) = request.placement_labels {
                    next.placement_labels = labels;
                }
                if let Some(distance) = request.grasp_collision_distance_m {
                    if !distance.is_finite() || distance < 0.0 {
                        return Err(eyre!("点云碰撞邻近距离必须是非负有限数值"));
                    }
                    next.grasp_collision_distance_m = distance;
                }
                next.enabled = true;
                Ok(())
            }
            RequestAction::Snapshot => self.snapshot_previews(),
            RequestAction::Cancel | RequestAction::Disconnect => {
                next.enabled = false;
                Ok(())
            }
            action => Err(eyre!("感知请求不支持动作 {action:?}")),
        })();
        let persist = request.action != RequestAction::Snapshot;
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
            self.clear_scene(node)?;
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
        let color = &self
            .latest_bundle
            .as_ref()
            .ok_or_else(|| eyre!("尚未收到彩色图"))?
            .color;
        let bundle = self
            .latest_bundle
            .as_ref()
            .ok_or_else(|| eyre!("尚未收到相机帧"))?;
        let depth = decode_depth(bundle)?;
        let color_png = color_png(color)?;
        let depth_png = depth_preview_png(&depth)?;
        self.color_frame = Some(image_frame_info(color));
        self.depth_frame = Some(image_frame_info(&bundle.aligned_depth));
        self.assets
            .insert("color.png".into(), ("image/png".into(), color_png));
        self.assets
            .insert("depth.png".into(), ("image/png".into(), depth_png));
        self.last_frame_time_ns = Some(bundle.received_time_ns);
        Ok(())
    }

    fn state(&self) -> PerceptionState {
        PerceptionState {
            schema_version: SCHEMA_VERSION,
            enabled: self.config.enabled,
            source_id: self.config.source_id.clone(),
            compute_service_url: self.config.compute_service_url.clone(),
            model: self
                .config
                .model
                .clone()
                .unwrap_or_else(|| self.model.clone()),
            available_models: self.models.clone(),
            classes: self.config.classes.clone(),
            visual_prompt_active: !self
                .models
                .iter()
                .any(|model| Some(&model.id) == self.config.model.as_ref() && model.prompt_free)
                && matches!(self.config.prompt, SegmentationPrompt::Visual { .. }),
            placement_labels: self.config.placement_labels.clone(),
            grasp_collision_distance_m: self.config.grasp_collision_distance_m,
            color_frame: self.color_frame.clone(),
            depth_frame: self.depth_frame.clone(),
            camera_calibration: self.camera_calibration.clone(),
            depth_scale_m: self.latest_bundle.as_ref().map(|frame| frame.depth_scale_m),
            instances: self.instances.clone(),
            point_count: self.point_count,
            last_frame_time_ns: self.last_frame_time_ns,
            last_scene_sequence: self.last_scene.as_ref().map(|scene| scene.sequence),
            last_segmentation_sequence: self.segmented.as_ref().map(|value| value.sequence),
            task_action: self.task_action,
            task_request_id: self.task_request_id.clone(),
            task_state: self.task_state,
            calibrated: self.camera_calibration.is_some(),
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
            has_output: self.segmented.is_some(),
            last_error: self.last_error.clone(),
            updated_at_ns: now_ns(),
        }
    }

    fn publish_state(&self, node: &mut DoraNode) -> Result<()> {
        send(node, "perception_state", &self.state())?;
        send(node, "service_state", &self.service_state())
    }

    fn publish_snapshot(&self, node: &mut DoraNode) -> Result<()> {
        self.publish_state(node)?;
        if let Some(scene) = &self.last_scene {
            send_scene(node, scene)?;
        }
        Ok(())
    }
}

fn validate_stage_input(
    request: &PerceptionRequest,
    segmented: Option<&SegmentedFrame>,
    scene: Option<&WorldScene>,
) -> Result<()> {
    match request.action {
        RequestAction::Refresh => Ok(()),
        RequestAction::Reconstruct => {
            let input = segmented.ok_or_else(|| eyre!("请先运行分割"))?;
            if request.input_sequence != Some(input.sequence) {
                bail!("分割结果已改变，请使用当前分割序号");
            }
            Ok(())
        }
        RequestAction::GenerateGrasps => {
            let scene = scene.ok_or_else(|| eyre!("请先进行三维定位"))?;
            if request.input_sequence != Some(scene.sequence) {
                bail!("三维场景已改变，请使用当前场景序号");
            }
            if !scene
                .objects
                .iter()
                .any(|object| Some(&object.object_id) == request.object_id.as_ref())
            {
                bail!("请选择当前场景中的一个抓取目标");
            }
            Ok(())
        }
        _ => bail!("不是感知计算动作"),
    }
}

async fn segment_frame(
    http: reqwest::Client,
    config: SceneConfig,
    frame: CameraFrameBundle,
    tool: Option<ToolPoseFeedback>,
    sequence: u64,
) -> Result<SegmentedFrame> {
    let catalog = model_catalog(&http, &config.compute_service_url).await?;
    let model = config.model.as_deref().unwrap_or(&catalog.model).to_owned();
    let prompt_free = catalog
        .models
        .iter()
        .find(|item| item.id == model)
        .ok_or_else(|| eyre!("未知分割模型 {model}"))?
        .prompt_free;
    let color = frame.color.clone();
    let instances = segment(
        &http,
        &config.compute_service_url,
        &model,
        if prompt_free { &[] } else { &config.classes },
        if prompt_free {
            &SegmentationPrompt::Text
        } else {
            &config.prompt
        },
        &color,
    )
    .await?;
    // An automatic model names the instances itself. Any observed instance can
    // be selected as a destination; no text aliases are fed back into inference.
    let placement_labels = if prompt_free {
        instances
            .iter()
            .map(|instance| instance.label.clone())
            .collect()
    } else {
        config.placement_labels.clone()
    };
    tokio::task::spawn_blocking(move || {
        let overlay = segmentation_debug_image(&color, &instances)?;
        let mut assets = BTreeMap::from([
            ("color.png".into(), ("image/png".into(), color_png(&color)?)),
            (
                "overlay.png".into(),
                ("image/png".into(), color_png(&overlay)?),
            ),
        ]);
        for (index, instance) in instances.iter().enumerate() {
            assets.insert(
                format!("mask-{index}.png"),
                ("image/png".into(), instance.mask_png.clone()),
            );
        }
        Ok(SegmentedFrame {
            sequence,
            model,
            models: catalog.models,
            frame,
            tool,
            instances,
            placement_labels,
            assets,
        })
    })
    .await
    .context("分割图像编码任务异常结束")?
}

fn reconstruct_frame(input: &SegmentedFrame, sequence: u64) -> Result<ProcessedStage> {
    // Never read latest_bundle here: masks, depth and calibration share one capture.
    let depth = decode_depth(&input.frame)?;
    let calibration = input
        .frame
        .calibration
        .as_ref()
        .ok_or_else(|| eyre!("分割时的相机帧未标定，请标定后重新分割"))?;
    let (scene, clouds) = world_scene_and_instance_clouds_from_aligned_depth(
        sequence,
        &depth,
        calibration,
        &input.instances,
        &input.placement_labels,
    )?;
    Ok(ProcessedStage::Reconstruction(scene, Arc::new(clouds)))
}

async fn model_catalog(http: &reqwest::Client, url: &str) -> Result<ModelResponse> {
    http.get(format!("{}/v1/model", url.trim_end_matches('/')))
        .send()
        .await
        .context("读取分割模型目录")?
        .error_for_status()
        .context("分割模型目录接口返回错误")?
        .json()
        .await
        .context("解析分割模型目录")
}

async fn segment(
    http: &reqwest::Client,
    compute_service_url: &str,
    model: &str,
    classes: &[String],
    prompt: &SegmentationPrompt,
    color: &CameraImagePlane,
) -> Result<Vec<DetectedInstance2D>> {
    let color = color.clone();
    let image_base64 =
        tokio::task::spawn_blocking(move || color_png(&color).map(|png| BASE64.encode(png)))
            .await
            .context("彩色图编码任务异常结束")??;
    let response = http
        .post(format!(
            "{}/v1/segment",
            compute_service_url.trim_end_matches('/')
        ))
        .json(&SegmentRequest {
            model,
            image_base64,
            classes,
            prompt,
        })
        .send()
        .await
        .context("调用 perception-compute-service")?
        .error_for_status()
        .context("perception-compute-service 返回错误")?
        .json::<SegmentResponse>()
        .await?;
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

async fn attach_grasp_candidates(
    http: &reqwest::Client,
    config: &SceneConfig,
    gripper_asset_id: &str,
    observed_gripper: Option<&ObservedGripper>,
    object_id: &str,
    scene: &mut WorldScene,
    instance_clouds: &[InstancePointCloud],
) -> Result<()> {
    let object = scene
        .objects
        .iter_mut()
        .find(|object| object.object_id == object_id)
        .ok_or_else(|| eyre!("场景中没有选定目标 {object_id}"))?;
    let cloud = instance_clouds
        .iter()
        .find(|cloud| cloud.instance_id == object.object_id)
        .ok_or_else(|| eyre!("实例 {} 缺少点云", object.object_id))?;
    let mut candidates = http
        .post(format!(
            "{}/v1/grasps",
            config.compute_service_url.trim_end_matches('/')
        ))
        .json(&GraspRequest {
            points_xyz_m: &cloud.points_xyz_m,
            scene_points_xyz_m: &cloud.scene_points_xyz_m,
            gripper_asset_id,
            collision_threshold_m: config.grasp_collision_distance_m,
            observed_gripper,
        })
        .send()
        .await
        .context("调用 GraspGenX")?
        .error_for_status()
        .context("GraspGenX 返回错误")?
        .json::<GraspResponse>()
        .await?
        .candidates;
    candidates.sort_by(|left, right| right.confidence.total_cmp(&left.confidence));
    object.grasp_candidates = candidates
        .iter()
        .map(|candidate| GraspCandidate {
            pose: candidate_tcp_pose_in_base(candidate.transform),
            confidence: candidate.confidence,
        })
        .collect();
    Ok(())
}

fn candidate_tcp_pose_in_base(base_tcp: [[f64; 4]; 4]) -> Pose3 {
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
    Pose3 {
        position_m: base_tcp.translation.vector.into(),
        orientation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
    }
}

fn load_config(path: &Path) -> Result<SceneConfig> {
    let config: SceneConfig = load_or_default(path)?;
    if config.schema_version != CONFIG_SCHEMA_VERSION {
        bail!("不支持的 scene 配置版本 {}", config.schema_version);
    }
    Ok(config)
}

fn decode_depth(frame: &CameraFrameBundle) -> Result<AlignedDepthFrame> {
    let message = &frame.aligned_depth;
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

fn color_rgb(message: &CameraImagePlane) -> Result<RgbImage> {
    Ok(
        RgbImage::from_raw(message.width, message.height, message.packed_rgb()?)
            .expect("validated packed RGB dimensions"),
    )
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

fn send_scene(node: &mut DoraNode, scene: &WorldScene) -> Result<()> {
    node.send_output(
        "world_scene".into(),
        MetadataParameters::default(),
        robot_arm_messages::world_scene_to_arrow(scene)?,
    )?;
    Ok(())
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

    #[tokio::test]
    async fn self_filter_waits_for_real_feedback_after_the_image() {
        let tool = ToolPoseFeedback {
            pose: robot_arm_messages::ToolPose {
                frame: "base".into(),
                position_m: [1., 2., 3.],
                orientation_xyzw: [0., 0., 0., 1.],
            },
            arm_state: robot_arm_messages::ArmState {
                schema_version: SCHEMA_VERSION,
                sequence: 1,
                sample_time_ns: 100,
                model_revision: "fixture".into(),
                joints_rad: vec![0.1234],
                actuators_rad: vec![0.4321],
                feedback_source: robot_arm_messages::FeedbackSource::Hardware,
            },
        };
        let (sender, mut receiver) = tokio::sync::watch::channel(Some(tool.clone()));
        let pending = tokio::spawn(async move { feedback_after_frame(&mut receiver, 101).await });
        tokio::task::yield_now().await;
        assert!(!pending.is_finished());
        let mut next = tool;
        next.arm_state.sample_time_ns = 102;
        next.arm_state.sequence = 2;
        sender.send_replace(Some(next.clone()));
        let paired = pending.await.unwrap().unwrap();
        assert_eq!(paired, next);
    }

    #[tokio::test]
    async fn refresh_waits_for_a_new_frame_without_blocking_the_producer() {
        let image = CameraImagePlane {
            width: 1,
            height: 1,
            stride_bytes: 3,
            pixel_format: "rgb8".into(),
            frame_id: "camera".into(),
            data: vec![1, 2, 3],
        };
        let frame = CameraFrameBundle {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            source_id: "test".into(),
            device_time_ns: 1,
            device_time_domain: "test".into(),
            received_time_ns: 100,
            color: image.clone(),
            aligned_depth: CameraImagePlane {
                pixel_format: "z16le".into(),
                stride_bytes: 2,
                data: vec![1, 0],
                ..image
            },
            intrinsics: robot_arm_messages::CameraIntrinsics {
                width: 1,
                height: 1,
                focal_length_px: [1.0; 2],
                principal_point_px: [0.0; 2],
                distortion_model: "none".into(),
                distortion: vec![],
            },
            depth_scale_m: 0.001,
            calibration: None,
        };
        let (sender, mut receiver) = tokio::sync::watch::channel(Some(frame.clone()));
        let waiting = tokio::spawn(async move { next_observation_frame(&mut receiver, 200).await });
        tokio::task::yield_now().await;
        assert!(!waiting.is_finished());
        sender.send_replace(Some(frame.clone()));
        tokio::task::yield_now().await;
        assert!(
            !waiting.is_finished(),
            "queued pre-motion frame was accepted"
        );
        let mut fresh = frame;
        fresh.sequence = 2;
        fresh.received_time_ns = 201;
        sender.send_replace(Some(fresh));
        assert_eq!(waiting.await.unwrap().unwrap().sequence, 2);

        let mut receiver = sender.subscribe();
        sender.send_replace(None);
        assert!(next_observation_frame(&mut receiver, 200).await.is_err());
    }

    #[test]
    fn default_configuration_has_no_selected_camera() {
        let config = SceneConfig::default();
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
        ]);
        assert_eq!(pose.position_m, [0.1, 0.2, 0.3]);
        assert_eq!(pose.orientation_xyzw, [0.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn configuration_round_trips_without_runtime_state() {
        let directory = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../temp/tests");
        std::fs::create_dir_all(&directory).unwrap();
        let path = directory.join(format!(
            "scene-config-{}-{}.json",
            std::process::id(),
            now_ns()
        ));
        let config = SceneConfig {
            enabled: true,
            model: Some("automatic-fixture".into()),
            classes: vec!["saved text".into()],
            source_id: Some("simulation:pick-place-scene".into()),
            ..Default::default()
        };
        assert!(config.placement_labels.is_empty());
        save(&path, &config).unwrap();
        let serialized = std::fs::read_to_string(&path).unwrap();
        assert!(!serialized.contains("camera_source_id"));
        let loaded = load_config(&path).unwrap();
        assert_eq!(loaded.source_id, None);
        assert_eq!(loaded.model, config.model);
        assert_eq!(loaded.classes, config.classes);
        std::fs::remove_file(path).unwrap();
    }
}
