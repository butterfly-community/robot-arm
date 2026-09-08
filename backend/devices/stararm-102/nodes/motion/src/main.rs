mod core;
mod ros;

use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc::{Receiver, channel},
    },
    time::{SystemTime, UNIX_EPOCH},
};

use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::Result;
use robot_arm_messages::{
    ArmCommand, ArmState, ControlMode, DiagnosticValue, FeedbackSource, ManipulationTaskState,
    MotionRequest, MotionState, MotionStatus, PerceptionState, PickPlaceRequest, RequestAction,
    RequestResult, RequestState, RobotModelInfo, SCHEMA_VERSION, ServiceState,
    SetControlModeRequest, ToolActuatorRequest, ToolActuatorStatus, ToolPose, ToolPoseFeedback,
    TransformedControlFrame, WorldScene, from_arrow, to_arrow,
};
use serde::Serialize;
use serde_json::{Value, json};
use stararm_102_model::{
    DEFAULT_JOINTS_RAD, GRIPPER_DRIVE_JOINT_CLOSED_RAD, GRIPPER_DRIVE_JOINT_MECHANICAL_LIMIT_RAD,
    GRIPPER_JOINT, GRIPPER_KEY, JOINTS, MODEL_REVISION, joint_limits_rad,
};

use crate::{
    core::{
        MotionConfig, Pose, merge_controller_command, target_pose, tool_action_transition,
        tool_position_rad,
    },
    ros::{ManipulationJob, MotionJob, MotionResult, RosEvent, RosInterface},
};

struct ActiveMotion {
    request_id: String,
    complete_in_relative_mode: bool,
    cancel: Arc<AtomicBool>,
}

struct PendingMotion {
    job: MotionJob,
    complete_in_relative_mode: bool,
}

struct PendingManipulation {
    job: ManipulationJob,
    state: ManipulationTaskState,
}

enum WorkItem {
    Motion(PendingMotion),
    Manipulation(Box<PendingManipulation>),
}

struct MotionNode {
    config_path: PathBuf,
    config: MotionConfig,
    ros: RosInterface,
    ros_events: Receiver<RosEvent>,
    model_info: Option<RobotModelInfo>,
    latest_arm_state: Option<ArmState>,
    feedback_source: Option<FeedbackSource>,
    controller_sync_required: bool,
    controller_sync_running: bool,
    controller_output_armed: bool,
    last_controller_command: Option<(Vec<f64>, f64)>,
    current_tcp: Option<Pose>,
    current_tcp_feedback: Option<ArmState>,
    anchor_tcp: Option<Pose>,
    target_tcp: Option<Pose>,
    control_session_id: Option<u64>,
    primary_tool_value: Option<f64>,
    sequence: u64,
    servo_code: Option<i64>,
    servo_message: Option<String>,
    pose_mode_ready: bool,
    fk_pending: bool,
    active_motion: Option<ActiveMotion>,
    work_queue: VecDeque<WorkItem>,
    motion_status: MotionStatus,
    actuator_status: Option<ToolActuatorStatus>,
    last_error: Option<String>,
    latest_scene: Option<WorldScene>,
    active_manipulation: Option<String>,
    manipulation_state: ManipulationTaskState,
}

fn planning_state(state: &ArmState) -> (Vec<f64>, f64) {
    let joints = state
        .joints_rad
        .iter()
        .zip(joint_limits_rad())
        .map(|(value, (minimum, maximum))| value.clamp(minimum, maximum))
        .collect();
    let actuator = state.actuators_rad[0].clamp(
        GRIPPER_DRIVE_JOINT_CLOSED_RAD,
        GRIPPER_DRIVE_JOINT_MECHANICAL_LIMIT_RAD,
    );
    (joints, actuator)
}

fn controller_sync_needed(previous: Option<FeedbackSource>, next: FeedbackSource) -> bool {
    next == FeedbackSource::Hardware && previous != Some(FeedbackSource::Hardware)
}

fn main() {
    if let Err(error) = run() {
        eprintln!("stararm-102-motion-node: {error:?}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let stop = Arc::new(AtomicBool::new(false));
    let (sender, receiver) = channel();
    let ros = RosInterface::start(sender, Arc::clone(&stop))?;
    let config_path = std::env::var_os("STARARM_MOTION_CONFIG")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/config/stararm-102-motion.json"));
    let mut motion = MotionNode {
        config: MotionConfig::load(&config_path)?,
        config_path,
        ros,
        ros_events: receiver,
        model_info: None,
        latest_arm_state: None,
        feedback_source: None,
        controller_sync_required: false,
        controller_sync_running: false,
        controller_output_armed: false,
        last_controller_command: None,
        current_tcp: None,
        current_tcp_feedback: None,
        anchor_tcp: None,
        target_tcp: None,
        control_session_id: None,
        primary_tool_value: None,
        sequence: 0,
        servo_code: None,
        servo_message: None,
        pose_mode_ready: false,
        fk_pending: false,
        active_motion: None,
        work_queue: VecDeque::new(),
        motion_status: idle_status(),
        actuator_status: None,
        last_error: None,
        latest_scene: None,
        active_manipulation: None,
        manipulation_state: idle_manipulation_state(),
    };
    let (mut node, mut events) = DoraNode::init_from_env()?;
    motion.publish_state(&mut node)?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => {
                match id.as_str() {
                    "arm_state" => motion.apply_arm_state(from_arrow(data.as_array())?)?,
                    "robot_model_info" => {
                        motion.model_info = Some(from_arrow(data.as_array())?);
                    }
                    "transformed_control" => {
                        motion.apply_transformed_control(from_arrow(data.as_array())?)?
                    }
                    "set_control_mode" | "calibration_control_mode" => {
                        motion.set_control_mode(&mut node, from_arrow(data.as_array())?)?
                    }
                    "prepare_relative" => {
                        let request: Value = from_arrow(data.as_array())?;
                        motion.prepare_relative(
                            &mut node,
                            request["request_id"]
                                .as_str()
                                .unwrap_or_default()
                                .to_owned(),
                        )?;
                    }
                    "motion_request" | "calibration_motion_request" => {
                        let request: Value = from_arrow(data.as_array())?;
                        if request["action"] == "cancel" {
                            motion.cancel_motion(
                                &mut node,
                                request["request_id"]
                                    .as_str()
                                    .unwrap_or_default()
                                    .to_owned(),
                            )?
                        } else {
                            motion.handle_motion_request(
                                &mut node,
                                serde_json::from_value(request)?,
                            )?
                        }
                    }
                    "tool_actuator_request" => {
                        motion.handle_actuator_request(&mut node, from_arrow(data.as_array())?)?
                    }
                    "world_scene" => {
                        let scene = robot_arm_messages::world_scene_from_arrow(data.as_array())?;
                        motion.latest_scene = Some(scene);
                    }
                    "perception_state" => {
                        let state: PerceptionState = from_arrow(data.as_array())?;
                        if !state.enabled {
                            motion.latest_scene = None;
                        }
                    }
                    "pick_place_request" => {
                        motion.handle_pick_place(&mut node, from_arrow(data.as_array())?)?;
                    }
                    "snapshot" => motion.publish_state(&mut node)?,
                    _ => {}
                }
                motion.drain_ros_events(&mut node)?;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    stop.store(true, Ordering::Relaxed);
    Ok(())
}

impl MotionNode {
    fn apply_arm_state(&mut self, state: ArmState) -> Result<()> {
        if state.model_revision != MODEL_REVISION
            || state.joints_rad.len() != JOINTS.len()
            || state.actuators_rad.len() != 1
            || !state
                .joints_rad
                .iter()
                .chain(&state.actuators_rad)
                .all(|value| value.is_finite())
        {
            eyre::bail!("ArmState does not match StarArm-102 model revision and shape");
        }
        let requires_sync = controller_sync_needed(self.feedback_source, state.feedback_source);
        self.feedback_source = Some(state.feedback_source);
        if requires_sync {
            self.controller_sync_required = true;
            self.controller_output_armed = false;
        }
        if !self.controller_output_armed {
            self.last_controller_command = Some((state.joints_rad.clone(), state.actuators_rad[0]));
        }
        let (planning_joints, planning_actuator) = planning_state(&state);
        self.ros
            .publish_state(&planning_joints, planning_actuator)?;
        if !self.fk_pending {
            self.fk_pending = true;
            // FK measures reality; planning-only limit projection must not alter it.
            self.ros.request_current_pose(state.clone());
        }
        self.latest_arm_state = Some(state);
        self.start_controller_sync();
        Ok(())
    }

    fn apply_transformed_control(&mut self, frame: TransformedControlFrame) -> Result<()> {
        if !frame.active {
            self.reset_relative_baseline();
            return Ok(());
        }
        if self.config.control_mode != ControlMode::Relative
            || self.active_motion.is_some()
            || self.active_manipulation.is_some()
            || !self.work_queue.is_empty()
        {
            return Ok(());
        }
        let open = frame.actuator_actions.primary_tool_open;
        let tool = frame.actuator_actions.primary_tool;
        if open.is_active && open.changed_since_last_sync && open.value {
            self.primary_tool_value = Some(0.0);
            self.publish_actuator("input-action", tool_position_rad(0.0))?;
        } else if tool.is_active {
            let transition = tool_action_transition(self.primary_tool_value, tool.value);
            self.primary_tool_value = Some(tool.value);
            if let Some(value) = transition {
                self.publish_actuator("input-action", tool_position_rad(value))?;
            }
        }
        if frame.control_session_id != self.control_session_id {
            self.control_session_id = frame.control_session_id;
            self.anchor_tcp = self.current_tcp;
        }
        let Some(anchor) = self.anchor_tcp else {
            return Ok(());
        };
        let target = target_pose(anchor, &frame);
        self.target_tcp = Some(target);
        self.controller_output_armed = true;
        self.ros.publish_pose(target)?;
        Ok(())
    }

    fn set_control_mode(
        &mut self,
        node: &mut DoraNode,
        request: SetControlModeRequest,
    ) -> Result<()> {
        let error = self
            .apply_control_mode(request.mode)
            .err()
            .map(|error| error.to_string());
        let result = RequestResult {
            schema_version: SCHEMA_VERSION,
            request_id: request.request_id,
            acknowledged_action: RequestAction::Apply,
            value: Some(self.motion_state()),
            original_error: error,
        };
        send(node, "mode_request_result", &result)?;
        self.publish_state(node)
    }

    fn apply_control_mode(&mut self, mode: ControlMode) -> Result<()> {
        self.config.set_mode(&self.config_path, mode)?;
        self.reset_relative_baseline();
        Ok(())
    }

    fn prepare_relative(&mut self, node: &mut DoraNode, request_id: String) -> Result<()> {
        if let Err(error) = self.apply_control_mode(ControlMode::Manual) {
            return self.fail_motion(node, request_id, error.to_string(), RequestAction::Apply);
        }
        let Some(state) = self.latest_arm_state.as_ref() else {
            return self.fail_motion(
                node,
                request_id,
                "缺少当前关节反馈".into(),
                RequestAction::Apply,
            );
        };
        let (current, _) = planning_state(state);
        let job = MotionJob {
            cancel: Arc::default(),
            request_id,
            current,
            target: DEFAULT_JOINTS_RAD.to_vec(),
            actuator: GRIPPER_DRIVE_JOINT_CLOSED_RAD,
            options: BTreeMap::new(),
        };
        self.queue_or_start_motion(node, job, true)
    }

    fn handle_motion_request(&mut self, node: &mut DoraNode, request: MotionRequest) -> Result<()> {
        if self.config.control_mode != ControlMode::Manual {
            return self.fail_motion(
                node,
                request.request_id,
                "普通关节运动只在手动控制模式接受".into(),
                RequestAction::Apply,
            );
        }
        if request.action != RequestAction::Apply {
            return self.fail_motion(
                node,
                request.request_id,
                format!("unsupported motion action {:?}", request.action),
                RequestAction::Apply,
            );
        }
        let Some(state) = self.latest_arm_state.as_ref() else {
            return self.fail_motion(
                node,
                request.request_id,
                "model revision mismatch or no ArmState".into(),
                RequestAction::Apply,
            );
        };
        if request.model_revision != MODEL_REVISION {
            return self.fail_motion(
                node,
                request.request_id,
                "model revision mismatch or no ArmState".into(),
                RequestAction::Apply,
            );
        }
        let unknown = request
            .options
            .keys()
            .filter(|key| !matches!(key.as_str(), "velocity_scaling" | "acceleration_scaling"))
            .cloned()
            .collect::<Vec<_>>();
        if !unknown.is_empty() || request.options.values().any(|value| !value.is_finite()) {
            return self.fail_motion(
                node,
                request.request_id,
                if unknown.is_empty() {
                    "motion option values must be finite numbers".into()
                } else {
                    format!("unknown motion options {unknown:?}")
                },
                RequestAction::Apply,
            );
        }
        let (current, mut actuator) = planning_state(state);
        let mut target = JOINTS
            .iter()
            .copied()
            .zip(&current)
            .map(|(key, value)| (key, *value))
            .collect::<BTreeMap<_, _>>();
        let mut seen = BTreeSet::new();
        for joint in request.joints {
            if !target.contains_key(joint.joint_key.as_str())
                || !joint.position_rad.is_finite()
                || !seen.insert(joint.joint_key.clone())
            {
                return self.fail_motion(
                    node,
                    request.request_id,
                    format!("invalid joint target {}", joint.joint_key),
                    RequestAction::Apply,
                );
            }
            *target
                .get_mut(joint.joint_key.as_str())
                .expect("joint checked above") = joint.position_rad;
        }
        seen.clear();
        for item in request.actuators {
            if item.actuator_key != GRIPPER_KEY
                || !item.position_rad.is_finite()
                || !seen.insert(item.actuator_key.clone())
            {
                return self.fail_motion(
                    node,
                    request.request_id,
                    format!("invalid actuator target {}", item.actuator_key),
                    RequestAction::Apply,
                );
            }
            actuator = item.position_rad;
        }
        let job = MotionJob {
            cancel: Arc::default(),
            request_id: request.request_id,
            current,
            target: JOINTS.map(|name| target[name]).to_vec(),
            actuator,
            options: request.options,
        };
        self.queue_or_start_motion(node, job, false)
    }

    fn queue_or_start_motion(
        &mut self,
        node: &mut DoraNode,
        job: MotionJob,
        complete_in_relative_mode: bool,
    ) -> Result<()> {
        self.work_queue.push_back(WorkItem::Motion(PendingMotion {
            job,
            complete_in_relative_mode,
        }));
        self.start_next_work(node)
    }

    fn start_next_work(&mut self, node: &mut DoraNode) -> Result<()> {
        if self.active_motion.is_some() || self.active_manipulation.is_some() {
            return Ok(());
        }
        let Some(next) = self.work_queue.front_mut() else {
            self.start_controller_sync();
            return Ok(());
        };
        if let (WorkItem::Motion(pending), Some(state)) = (next, &self.latest_arm_state) {
            pending.job.current = planning_state(state).0;
        }
        if self.controller_sync_required || self.controller_sync_running {
            self.start_controller_sync();
            return Ok(());
        }
        match self.work_queue.pop_front().expect("queue checked above") {
            WorkItem::Motion(pending) => {
                self.motion_status = MotionStatus {
                    request_id: pending.job.request_id.clone(),
                    state: RequestState::Planning,
                    result_message: Some("MoveIt 正在规划普通关节目标".into()),
                    ..idle_status()
                };
                send(node, "motion_status", &self.motion_status)?;
                self.start_motion(pending.job, pending.complete_in_relative_mode);
            }
            WorkItem::Manipulation(pending) => {
                self.manipulation_state = pending.state;
                self.active_manipulation = Some(pending.job.request_id.clone());
                send(node, "manipulation_state", &self.manipulation_state)?;
                self.controller_output_armed = true;
                self.ros.run_manipulation(pending.job);
            }
        }
        Ok(())
    }

    fn start_motion(&mut self, job: MotionJob, complete_in_relative_mode: bool) {
        self.active_motion = Some(ActiveMotion {
            request_id: job.request_id.clone(),
            complete_in_relative_mode,
            cancel: job.cancel.clone(),
        });
        self.ros.run_motion(job);
    }

    fn handle_actuator_request(
        &mut self,
        node: &mut DoraNode,
        request: ToolActuatorRequest,
    ) -> Result<()> {
        let error = if self.config.control_mode != ControlMode::Manual {
            Some("夹爪手动命令只在手动控制模式接受".into())
        } else if request.model_revision != MODEL_REVISION
            || request.actuator_key != GRIPPER_KEY
            || !request.position_rad.is_finite()
        {
            Some("request does not match StarArm-102 gripper".into())
        } else {
            self.publish_actuator(&request.request_id, request.position_rad)
                .err()
                .map(|error| error.to_string())
        };
        if let Some(message) = &error {
            self.actuator_status = Some(ToolActuatorStatus {
                schema_version: SCHEMA_VERSION,
                request_id: request.request_id.clone(),
                actuator_key: request.actuator_key,
                state: RequestState::Failed,
                result_code: None,
                result_message: Some(message.clone()),
            });
        }
        send(
            node,
            "actuator_status",
            self.actuator_status
                .as_ref()
                .expect("status assigned above"),
        )?;
        send(
            node,
            "actuator_request_result",
            &RequestResult {
                schema_version: SCHEMA_VERSION,
                request_id: request.request_id,
                acknowledged_action: RequestAction::Apply,
                value: self.actuator_status.clone(),
                original_error: error,
            },
        )?;
        self.publish_state(node)
    }

    fn publish_actuator(&mut self, request_id: &str, position_rad: f64) -> Result<()> {
        self.controller_output_armed = true;
        self.ros.publish_actuator(position_rad)?;
        self.actuator_status = Some(ToolActuatorStatus {
            schema_version: SCHEMA_VERSION,
            request_id: request_id.into(),
            actuator_key: GRIPPER_KEY.into(),
            state: RequestState::Succeeded,
            result_code: None,
            result_message: None,
        });
        Ok(())
    }

    fn handle_pick_place(&mut self, node: &mut DoraNode, request: PickPlaceRequest) -> Result<()> {
        let request_id = request.request_id.clone();
        if self.config.control_mode != ControlMode::Perception {
            return self.fail_manipulation(node, request_id, "抓放任务只在感知控制模式接受".into());
        }
        let result = self
            .latest_scene
            .as_ref()
            .ok_or_else(|| eyre::eyre!("尚未收到结构化感知场景"))
            .and_then(|scene| manipulation_job(scene, request));
        match result {
            Ok(pending) => {
                // Acknowledge validation/queueing before returning HTTP 202.
                // Completion is still reported through manipulation_state.
                Self::send_manipulation_result(node, &pending.state)?;
                self.work_queue
                    .push_back(WorkItem::Manipulation(Box::new(pending)));
                self.start_next_work(node)
            }
            Err(error) => self.fail_manipulation(node, request_id, error.to_string()),
        }
    }

    fn fail_manipulation(
        &mut self,
        node: &mut DoraNode,
        request_id: String,
        message: String,
    ) -> Result<()> {
        let failed = ManipulationTaskState {
            request_id,
            state: RequestState::Failed,
            original_error: Some(message),
            ..idle_manipulation_state()
        };
        Self::send_manipulation_result(node, &failed)?;
        if self.active_manipulation.is_none() {
            self.manipulation_state = failed;
            send(node, "manipulation_state", &self.manipulation_state)?;
        }
        Ok(())
    }

    fn send_manipulation_result(node: &mut DoraNode, status: &ManipulationTaskState) -> Result<()> {
        send(
            node,
            "manipulation_request_result",
            &RequestResult {
                schema_version: SCHEMA_VERSION,
                request_id: status.request_id.clone(),
                acknowledged_action: RequestAction::Apply,
                value: Some(status.clone()),
                original_error: status.original_error.clone(),
            },
        )
    }

    fn start_controller_sync(&mut self) {
        if self.controller_sync_required
            && !self.controller_sync_running
            && self.active_motion.is_none()
            && self.active_manipulation.is_none()
        {
            self.controller_sync_running = true;
            self.controller_sync_required = false;
            self.ros.synchronize_controllers();
        }
    }

    fn drain_ros_events(&mut self, node: &mut DoraNode) -> Result<()> {
        while let Ok(event) = self.ros_events.try_recv() {
            match event {
                RosEvent::ControllerCommand(value) => {
                    self.apply_controller_command(node, &value)?
                }
                RosEvent::ServoStatus(value) => {
                    self.servo_code = value["code"].as_i64();
                    self.servo_message = value["message"].as_str().map(str::to_owned);
                    self.publish_state(node)?;
                }
                RosEvent::CurrentPose(feedback, result) => {
                    self.fk_pending = false;
                    match result {
                        Ok(pose) => {
                            self.current_tcp = Some(pose);
                            self.current_tcp_feedback = Some(feedback);
                        }
                        Err(error) => self.last_error = Some(error),
                    }
                    self.publish_state(node)?;
                }
                RosEvent::PoseMode(result) => {
                    match result {
                        Ok(()) => self.pose_mode_ready = true,
                        Err(error) => self.last_error = Some(error),
                    }
                    self.publish_state(node)?;
                }
                RosEvent::SyncFinished(result) => {
                    self.controller_sync_running = false;
                    match result {
                        Ok(()) => {
                            self.last_error = None;
                            self.start_next_work(node)?;
                        }
                        Err(error) => {
                            self.controller_sync_required = true;
                            self.last_error = Some(error.clone());
                            if let Some(pending) = self.work_queue.pop_front() {
                                match pending {
                                    WorkItem::Motion(pending) => self.fail_motion(
                                        node,
                                        pending.job.request_id,
                                        format!("同步 ros2_control 控制器失败：{error}"),
                                        RequestAction::Apply,
                                    )?,
                                    WorkItem::Manipulation(pending) => {
                                        self.manipulation_state = pending.state;
                                        self.manipulation_state.state = RequestState::Failed;
                                        self.manipulation_state.original_error =
                                            Some(format!("同步 ros2_control 控制器失败：{error}"));
                                        send(node, "manipulation_state", &self.manipulation_state)?;
                                        Self::send_manipulation_result(
                                            node,
                                            &self.manipulation_state,
                                        )?;
                                    }
                                }
                            }
                            self.start_next_work(node)?;
                        }
                    }
                    self.publish_state(node)?;
                }
                RosEvent::MotionExecuting {
                    request_id,
                    points,
                    duration_s,
                    collision_pairs,
                } => {
                    if self.active_request_is(&request_id) {
                        self.controller_output_armed = true;
                        self.motion_status.state = RequestState::Executing;
                        self.motion_status.trajectory_points = Some(points);
                        self.motion_status.planned_duration_s = Some(duration_s);
                        self.motion_status.result_message = Some(if collision_pairs.is_empty() {
                            "普通运动规划通过".into()
                        } else {
                            format!(
                                "普通运动规划通过；本次临时放行 {}",
                                format_pairs(&collision_pairs)
                            )
                        });
                        send(node, "motion_status", &self.motion_status)?;
                    }
                }
                RosEvent::MotionFinished { request_id, result } => {
                    if !self.active_request_is(&request_id) {
                        continue;
                    }
                    let active = self.active_motion.take().expect("active request checked");
                    self.reset_relative_baseline();
                    match result {
                        Ok(MotionResult::Cancelled) => {
                            self.motion_status.state = RequestState::Cancelled;
                            self.motion_status.result_message = Some("普通运动已取消".into());
                            send(node, "motion_status", &self.motion_status)?;
                            self.send_motion_result(node, &self.motion_status)?;
                        }
                        Ok(MotionResult::Succeeded(code)) => {
                            if active.complete_in_relative_mode
                                && let Err(error) = self.apply_control_mode(ControlMode::Relative)
                            {
                                self.fail_motion(
                                    node,
                                    request_id,
                                    error.to_string(),
                                    RequestAction::Apply,
                                )?;
                                self.start_next_work(node)?;
                                self.publish_state(node)?;
                                continue;
                            }
                            self.motion_status.state = RequestState::Succeeded;
                            self.motion_status.result_code = Some(code.to_string());
                            self.motion_status.result_message = Some("普通运动执行完成".into());
                            send(node, "motion_status", &self.motion_status)?;
                            self.send_motion_result(node, &self.motion_status)?;
                        }
                        Err(error) => {
                            self.fail_motion(node, request_id, error, RequestAction::Apply)?;
                        }
                    }
                    self.start_next_work(node)?;
                    self.publish_state(node)?;
                }
                RosEvent::ManipulationFeedback {
                    request_id,
                    state,
                    stage,
                    solution_count,
                    selected_cost,
                } => {
                    if self.active_manipulation.as_deref() == Some(request_id.as_str()) {
                        self.manipulation_state.state = if state == "executing" {
                            RequestState::Executing
                        } else {
                            RequestState::Planning
                        };
                        self.manipulation_state.stage = Some(stage);
                        self.manipulation_state.solution_count =
                            (solution_count > 0).then_some(solution_count);
                        self.manipulation_state.selected_cost =
                            (solution_count > 0).then_some(selected_cost);
                        send(node, "manipulation_state", &self.manipulation_state)?;
                    }
                }
                RosEvent::ManipulationFinished { request_id, result } => {
                    if self.active_manipulation.as_deref() != Some(request_id.as_str()) {
                        continue;
                    }
                    self.active_manipulation = None;
                    match result {
                        Ok(result) => {
                            self.manipulation_state.state = RequestState::Succeeded;
                            self.manipulation_state.stage = Some(result.message);
                            self.manipulation_state.solution_count = Some(result.solution_count);
                            self.manipulation_state.selected_cost = Some(result.selected_cost);
                            self.manipulation_state.original_error = None;
                        }
                        Err(error) => {
                            self.manipulation_state.state = RequestState::Failed;
                            self.manipulation_state.original_error = Some(error);
                        }
                    }
                    send(node, "manipulation_state", &self.manipulation_state)?;
                    Self::send_manipulation_result(node, &self.manipulation_state)?;
                    self.start_next_work(node)?;
                    self.publish_state(node)?;
                }
            }
        }
        Ok(())
    }

    fn apply_controller_command(&mut self, node: &mut DoraNode, value: &Value) -> Result<()> {
        if self.latest_arm_state.is_none()
            || self.controller_sync_required
            || self.controller_sync_running
            || !self.controller_output_armed
        {
            return Ok(());
        }
        let names = value["name"]
            .as_array()
            .map(|values| {
                values
                    .iter()
                    .filter_map(|value| value.as_str().map(str::to_owned))
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        let positions = value["position"]
            .as_array()
            .map(|values| {
                values
                    .iter()
                    .map(|value| value.as_f64().unwrap_or(f64::NAN))
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        let state = self.latest_arm_state.as_ref().expect("checked above");
        let (current_joints, current_actuator) = self
            .last_controller_command
            .as_ref()
            .map(|(joints, actuator)| (joints.as_slice(), *actuator))
            .unwrap_or((&state.joints_rad, state.actuators_rad[0]));
        let Some((joints, actuator)) = merge_controller_command(
            &names,
            &positions,
            &JOINTS,
            GRIPPER_JOINT,
            current_joints,
            current_actuator,
        ) else {
            return Ok(());
        };
        if self.last_controller_command.as_ref() == Some(&(joints.clone(), actuator)) {
            return Ok(());
        }
        self.last_controller_command = Some((joints.clone(), actuator));
        self.sequence += 1;
        send(
            node,
            "arm_command",
            &ArmCommand {
                schema_version: SCHEMA_VERSION,
                sequence: self.sequence,
                controller_time_ns: now_ns(),
                model_revision: MODEL_REVISION.into(),
                joints_rad: joints,
                actuators_rad: vec![actuator],
            },
        )
    }

    fn cancel_motion(&mut self, node: &mut DoraNode, request_id: String) -> Result<()> {
        let cancelling_active = self.active_motion.is_some();
        if let Some(active) = &self.active_motion {
            active.cancel.store(true, Ordering::Relaxed);
        } else if let Some(WorkItem::Motion(_)) = self.work_queue.front() {
            let WorkItem::Motion(pending) = self.work_queue.pop_front().expect("front checked")
            else {
                unreachable!()
            };
            let cancelled = MotionStatus {
                request_id: pending.job.request_id,
                state: RequestState::Cancelled,
                result_message: Some("普通运动请求已取消".into()),
                ..idle_status()
            };
            self.send_motion_result(node, &cancelled)?;
            self.start_next_work(node)?;
        }
        let acknowledgement = MotionStatus {
            request_id,
            acknowledged_action: "cancel".into(),
            state: RequestState::Succeeded,
            result_message: Some(
                if cancelling_active {
                    "已请求取消，原运动请求将返回最终结果"
                } else {
                    "当前没有正在执行的普通运动"
                }
                .into(),
            ),
            ..idle_status()
        };
        self.send_motion_result(node, &acknowledgement)
    }

    fn fail_motion(
        &mut self,
        node: &mut DoraNode,
        request_id: String,
        message: String,
        action: RequestAction,
    ) -> Result<()> {
        let failed = MotionStatus {
            request_id,
            acknowledged_action: action_name(action).into(),
            state: RequestState::Failed,
            result_message: Some(message),
            ..idle_status()
        };
        self.send_motion_result(node, &failed)?;
        if self.active_motion.is_none() || self.active_request_is(&failed.request_id) {
            self.motion_status = failed;
            send(node, "motion_status", &self.motion_status)?;
        }
        Ok(())
    }

    fn send_motion_result(&self, node: &mut DoraNode, status: &MotionStatus) -> Result<()> {
        send(
            node,
            "motion_request_result",
            &RequestResult {
                schema_version: SCHEMA_VERSION,
                request_id: status.request_id.clone(),
                acknowledged_action: if status.acknowledged_action == "cancel" {
                    RequestAction::Cancel
                } else {
                    RequestAction::Apply
                },
                value: Some(status.clone()),
                original_error: (status.state == RequestState::Failed)
                    .then(|| status.result_message.clone())
                    .flatten(),
            },
        )
    }

    fn active_request_is(&self, request_id: &str) -> bool {
        self.active_motion
            .as_ref()
            .is_some_and(|active| active.request_id == request_id)
    }

    fn reset_relative_baseline(&mut self) {
        self.control_session_id = None;
        self.anchor_tcp = None;
        self.target_tcp = None;
    }

    fn motion_state(&self) -> MotionState {
        MotionState {
            schema_version: SCHEMA_VERSION,
            control_mode: self.config.control_mode,
            current_tool_pose: self
                .current_tcp
                .zip(self.current_tcp_feedback.as_ref())
                .map(|(pose, feedback)| ToolPoseFeedback {
                    pose: tool_pose(pose),
                    arm_state: feedback.clone(),
                }),
            target_tool_pose: self.target_tcp.map(tool_pose),
            control_session_id: self.control_session_id,
            latest_motion: Some(self.motion_status.clone()),
            latest_actuator: self.actuator_status.clone(),
            diagnostics: vec![
                DiagnosticValue {
                    key: "servo_status_code".into(),
                    value: self.servo_code.map_or(Value::Null, Value::from),
                },
                DiagnosticValue {
                    key: "servo_status_message".into(),
                    value: self
                        .servo_message
                        .as_ref()
                        .map_or(Value::Null, |value| json!(value)),
                },
            ],
            service: self.service_state(),
        }
    }

    fn service_state(&self) -> ServiceState {
        ServiceState {
            schema_version: SCHEMA_VERSION,
            build_version: env!("CARGO_PKG_VERSION").into(),
            config_version: self.config.config_version,
            running: true,
            has_input: self.latest_arm_state.is_some() && self.model_info.is_some(),
            has_output: self.pose_mode_ready
                && !self.controller_sync_required
                && !self.controller_sync_running
                && self.ros.outputs_ready(),
            last_error: self.last_error.clone(),
            updated_at_ns: now_ns(),
        }
    }

    fn publish_state(&self, node: &mut DoraNode) -> Result<()> {
        let state = self.motion_state();
        send(node, "motion_state", &state)?;
        send(node, "manipulation_state", &self.manipulation_state)?;
        send(node, "service_state", &state.service)
    }
}

fn idle_status() -> MotionStatus {
    MotionStatus {
        schema_version: SCHEMA_VERSION,
        request_id: String::new(),
        acknowledged_action: "apply".into(),
        state: RequestState::Idle,
        backend_name: "moveit".into(),
        result_code: None,
        result_message: Some("尚未请求普通运动".into()),
        trajectory_points: None,
        planned_duration_s: None,
    }
}

fn idle_manipulation_state() -> ManipulationTaskState {
    ManipulationTaskState {
        schema_version: SCHEMA_VERSION,
        request_id: String::new(),
        object_id: None,
        placement_region_id: None,
        pick_position_m: None,
        place_position_m: None,
        state: RequestState::Idle,
        stage: None,
        solution_count: None,
        selected_cost: None,
        original_error: None,
    }
}

fn manipulation_job(scene: &WorldScene, request: PickPlaceRequest) -> Result<PendingManipulation> {
    eyre::ensure!(
        scene.sequence == request.scene_sequence,
        "感知场景已改变，请从当前场景重新选择目标"
    );
    let object = scene
        .objects
        .iter()
        .find(|object| object.object_id == request.object_id)
        .ok_or_else(|| eyre::eyre!("场景中没有抓取对象 {}", request.object_id))?;
    let placement = scene
        .placement_regions
        .iter()
        .find(|region| region.region_id == request.placement_region_id)
        .ok_or_else(|| eyre::eyre!("场景中没有放置区 {}", request.placement_region_id))?;
    if object.grasp_candidates.is_empty() {
        return Err(eyre::eyre!("抓取对象 {} 没有抓取候选", object.object_id));
    }
    let pose = |pose: &robot_arm_messages::Pose3| {
        let [x, y, z] = pose.position_m;
        let [qx, qy, qz, qw] = pose.orientation_xyzw;
        json!({
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        })
    };
    let size = |[x, y, z]: [f64; 3]| json!({"x": x, "y": y, "z": z});
    let mut placement_pose = placement.pose.clone();
    // User-defined minimum TCP release height above the Z=0 ground, not an
    // offset from the destination. MTC ignores the destination orientation.
    placement_pose.position_m[2] = placement_pose.position_m[2].max(0.10);
    let goal = json!({
        "request_id": request.request_id,
        "frame_id": scene.frame_id,
        "object_id": object.object_id,
        "object_pose": pose(&object.pose),
        "object_size": size(object.size_m),
        "grasp_poses": object.grasp_candidates.iter().map(|candidate| pose(&candidate.pose)).collect::<Vec<_>>(),
        "grasp_confidences": object.grasp_candidates.iter().map(|candidate| candidate.confidence).collect::<Vec<_>>(),
        "placement_region_id": placement.region_id,
        "placement_pose": pose(&placement_pose),
        "placement_size": size(placement.size_m),
    });
    Ok(PendingManipulation {
        job: ManipulationJob {
            request_id: request.request_id.clone(),
            goal,
            point_cloud: scene
                .point_cloud
                .clone()
                .ok_or_else(|| eyre::eyre!("当前场景没有同帧点云，请重新运行感知"))?,
        },
        state: ManipulationTaskState {
            schema_version: SCHEMA_VERSION,
            request_id: request.request_id,
            object_id: Some(object.object_id.clone()),
            placement_region_id: Some(placement.region_id.clone()),
            pick_position_m: Some(object.pose.position_m),
            place_position_m: Some(placement_pose.position_m),
            state: RequestState::Planning,
            stage: Some("等待 MTC 规划".into()),
            solution_count: None,
            selected_cost: None,
            original_error: None,
        },
    })
}

fn tool_pose(pose: Pose) -> ToolPose {
    ToolPose {
        frame: stararm_102_model::BASE_FRAME.into(),
        position_m: pose.position_m,
        orientation_xyzw: pose.orientation_xyzw,
    }
}

fn action_name(action: RequestAction) -> &'static str {
    match action {
        RequestAction::Cancel => "cancel",
        _ => "apply",
    }
}

fn format_pairs(pairs: &[(String, String)]) -> String {
    pairs
        .iter()
        .map(|(first, second)| format!("{first}↔{second}"))
        .collect::<Vec<_>>()
        .join("、")
}

fn send<T: Serialize>(node: &mut DoraNode, id: &str, value: &T) -> Result<()> {
    node.send_output(
        DataId::from(id.to_owned()),
        MetadataParameters::default(),
        to_arrow(value)?,
    )?;
    Ok(())
}

fn now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|value| value.as_nanos() as i64)
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use robot_arm_messages::{PlacementRegion, Pose3, SceneObject};

    #[test]
    fn controller_sync_is_only_needed_when_entering_hardware_feedback() {
        assert!(!controller_sync_needed(None, FeedbackSource::Software));
        assert!(!controller_sync_needed(
            Some(FeedbackSource::Software),
            FeedbackSource::Software,
        ));
        assert!(controller_sync_needed(None, FeedbackSource::Hardware));
        assert!(controller_sync_needed(
            Some(FeedbackSource::Software),
            FeedbackSource::Hardware,
        ));
        assert!(!controller_sync_needed(
            Some(FeedbackSource::Hardware),
            FeedbackSource::Hardware,
        ));
    }

    #[test]
    fn planning_state_projects_feedback_to_the_model_without_changing_raw_feedback() {
        let state = ArmState {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            sample_time_ns: 2,
            model_revision: MODEL_REVISION.into(),
            joints_rad: vec![2.0, -0.005, 0.005, 2.0, 2.0, 3.0],
            actuators_rad: vec![-0.003],
            feedback_source: FeedbackSource::Hardware,
        };
        let raw_joints = state.joints_rad.clone();
        let limits = joint_limits_rad();

        let (joints, actuator) = planning_state(&state);

        assert_eq!(
            joints,
            vec![
                limits[0].1,
                limits[1].0,
                limits[2].1,
                limits[3].1,
                limits[4].1,
                limits[5].1,
            ]
        );
        assert_eq!(actuator, GRIPPER_DRIVE_JOINT_CLOSED_RAD);
        assert_eq!(state.joints_rad, raw_joints);
        assert_eq!(state.actuators_rad, vec![-0.003]);
    }

    #[test]
    fn collision_pairs_have_a_stable_human_readable_form() {
        assert_eq!(
            format_pairs(&[("link1".into(), "link4".into())]),
            "link1↔link4"
        );
    }

    #[test]
    fn mtc_goal_releases_tcp_above_10cm_without_overriding_orientation() {
        let pose = Pose3 {
            position_m: [0.1, 0.2, 0.02],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        };
        let mut scene = WorldScene {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            sample_time_ns: 2,
            frame_id: "base_link".into(),
            objects: vec![
                SceneObject {
                    object_id: "selected".into(),
                    label: "arbitrary target".into(),
                    pose: pose.clone(),
                    size_m: [0.04; 3],
                    confidence: 1.0,
                    grasp_candidates: vec![robot_arm_messages::GraspCandidate {
                        pose: pose.clone(),
                        confidence: 0.87,
                    }],
                },
                SceneObject {
                    object_id: "support".into(),
                    label: "arbitrary support".into(),
                    pose: Pose3 {
                        position_m: [-0.1, 0.2, 0.04],
                        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                    },
                    size_m: [0.08, 0.08, 0.08],
                    confidence: 1.0,
                    grasp_candidates: vec![],
                },
            ],
            placement_regions: vec![PlacementRegion {
                region_id: "destination".into(),
                label: "arbitrary destination".into(),
                pose: Pose3 {
                    position_m: [-0.1, 0.2, 0.04],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                },
                size_m: [0.08, 0.08, 0.08],
                source_object_id: Some("support".into()),
            }],
            obstacles: vec![],
            point_cloud: Some(robot_arm_messages::ScenePointCloud {
                frame_id: "camera".into(),
                sensor_in_scene: pose.clone(),
                width: 1,
                height: 1,
                xyz_le: vec![0; 12],
            }),
        };
        let pending = manipulation_job(
            &scene,
            PickPlaceRequest {
                schema_version: SCHEMA_VERSION,
                request_id: "request".into(),
                scene_sequence: scene.sequence,
                object_id: "selected".into(),
                placement_region_id: "destination".into(),
            },
        )
        .unwrap();
        assert!(
            manipulation_job(
                &scene,
                PickPlaceRequest {
                    schema_version: SCHEMA_VERSION,
                    request_id: "stale-selection".into(),
                    scene_sequence: scene.sequence - 1,
                    object_id: "selected".into(),
                    placement_region_id: "destination".into(),
                },
            )
            .err()
            .unwrap()
            .to_string()
            .contains("场景")
        );
        assert_eq!(pending.job.goal["object_pose"]["position"]["z"], 0.02);
        assert_eq!(pending.job.goal["grasp_confidences"], json!([0.87]));
        #[cfg(feature = "ros-runtime")]
        {
            let native_goal: r2r::stararm_102_mtc::action::PickPlace::Goal =
                serde_json::from_value(pending.job.goal.clone()).unwrap();
            assert_eq!(native_goal.grasp_confidences, vec![0.87]);
        }
        assert_eq!(pending.job.goal["grasp_poses"][0]["position"]["z"], 0.02);
        assert_eq!(pending.job.goal["object_size"]["z"], 0.04);
        assert!(pending.job.goal.get("obstacle_ids").is_none());
        assert_eq!(pending.state.pick_position_m, Some([0.1, 0.2, 0.02]));
        let release = pending.state.place_position_m.unwrap();
        assert_eq!(&release[..2], &[-0.1, 0.2]);
        assert_eq!(release[2], 0.10);
        assert_eq!(
            pending.job.goal["placement_pose"]["position"]["z"],
            release[2]
        );
        assert_eq!(pending.job.goal["placement_pose"]["orientation"]["x"], 0.0);
        assert_eq!(pending.job.goal["placement_pose"]["orientation"]["w"], 1.0);
        scene.placement_regions[0].pose.position_m[2] = 0.25;
        let elevated = manipulation_job(
            &scene,
            PickPlaceRequest {
                schema_version: SCHEMA_VERSION,
                request_id: "elevated".into(),
                scene_sequence: scene.sequence,
                object_id: "selected".into(),
                placement_region_id: "destination".into(),
            },
        )
        .unwrap();
        assert_eq!(elevated.state.place_position_m, Some([-0.1, 0.2, 0.25]));
    }
}
