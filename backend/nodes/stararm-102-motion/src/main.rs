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
    ArmCommand, ArmState, ControlMode, DiagnosticValue, FeedbackSource, MotionRequest, MotionState,
    MotionStatus, RequestAction, RequestResult, RequestState, RobotModelInfo, SCHEMA_VERSION,
    ServiceState, SetControlModeRequest, ToolActuatorRequest, ToolActuatorStatus, ToolPose,
    TransformedControlFrame, from_arrow, to_arrow,
};
use serde::Serialize;
use serde_json::{Value, json};
use stararm_102_model::{
    CLOSED_GRIPPER_RAD, GRIPPER_JOINT, GRIPPER_KEY, JOINTS, MODEL_REVISION, START_JOINTS_RAD,
};

use crate::{
    core::{
        MotionConfig, Pose, merge_controller_command, target_pose, tool_action_transition,
        tool_position_rad,
    },
    ros::{MotionJob, RosEvent, RosInterface},
};

struct ActiveMotion {
    request_id: String,
    complete_in_relative_mode: bool,
    cancelled: bool,
}

struct PendingMotion {
    job: MotionJob,
    complete_in_relative_mode: bool,
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
    motion_queue: VecDeque<PendingMotion>,
    motion_status: MotionStatus,
    actuator_status: Option<ToolActuatorStatus>,
    last_error: Option<String>,
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
        motion_queue: VecDeque::new(),
        motion_status: idle_status(),
        actuator_status: None,
        last_error: None,
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
                    "set_control_mode" => {
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
                    "motion_request" => {
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
                    "snapshot" => motion.publish_state(&mut node)?,
                    "tick" => {}
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
        let requires_sync = self.feedback_source.is_none()
            || (self.feedback_source != Some(state.feedback_source)
                && state.feedback_source == FeedbackSource::Hardware);
        self.feedback_source = Some(state.feedback_source);
        if requires_sync {
            self.controller_sync_required = true;
            self.controller_output_armed = false;
        }
        if !self.controller_output_armed {
            self.last_controller_command = Some((state.joints_rad.clone(), state.actuators_rad[0]));
        }
        self.ros
            .publish_state(&state.joints_rad, state.actuators_rad[0])?;
        if !self.fk_pending {
            self.fk_pending = true;
            self.ros.request_current_pose(state.joints_rad.clone());
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
        if self.config.control_mode != ControlMode::Relative
            || self.active_motion.is_some()
            || !self.motion_queue.is_empty()
        {
            return Ok(());
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
        let job = MotionJob {
            request_id,
            current: state.joints_rad.clone(),
            target: START_JOINTS_RAD.to_vec(),
            actuator: CLOSED_GRIPPER_RAD,
            options: BTreeMap::new(),
        };
        self.queue_or_start_motion(node, job, true)
    }

    fn handle_motion_request(&mut self, node: &mut DoraNode, request: MotionRequest) -> Result<()> {
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
        let mut target = JOINTS
            .iter()
            .copied()
            .zip(&state.joints_rad)
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
        let mut actuator = state.actuators_rad[0];
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
            request_id: request.request_id,
            current: state.joints_rad.clone(),
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
        self.motion_queue.push_back(PendingMotion {
            job,
            complete_in_relative_mode,
        });
        self.start_next_motion(node)
    }

    fn start_next_motion(&mut self, node: &mut DoraNode) -> Result<()> {
        if self.active_motion.is_some() {
            return Ok(());
        }
        let Some(pending) = self.motion_queue.front_mut() else {
            self.start_controller_sync();
            return Ok(());
        };
        if let Some(state) = &self.latest_arm_state {
            pending.job.current.clone_from(&state.joints_rad);
        }
        self.motion_status = MotionStatus {
            request_id: pending.job.request_id.clone(),
            state: RequestState::Planning,
            result_message: Some(
                if self.controller_sync_required || self.controller_sync_running {
                    "等待 ros2_control 同步当前反馈".into()
                } else {
                    "MoveIt 正在规划普通关节目标".into()
                },
            ),
            ..idle_status()
        };
        send(node, "motion_status", &self.motion_status)?;
        if self.controller_sync_required || self.controller_sync_running {
            self.start_controller_sync();
        } else {
            let pending = self.motion_queue.pop_front().expect("queue checked above");
            self.start_motion(pending.job, pending.complete_in_relative_mode);
        }
        Ok(())
    }

    fn start_motion(&mut self, job: MotionJob, complete_in_relative_mode: bool) {
        self.active_motion = Some(ActiveMotion {
            request_id: job.request_id.clone(),
            complete_in_relative_mode,
            cancelled: false,
        });
        self.ros.run_motion(job);
    }

    fn handle_actuator_request(
        &mut self,
        node: &mut DoraNode,
        request: ToolActuatorRequest,
    ) -> Result<()> {
        if request.model_revision != MODEL_REVISION
            || request.actuator_key != GRIPPER_KEY
            || !request.position_rad.is_finite()
        {
            self.actuator_status = Some(ToolActuatorStatus {
                schema_version: SCHEMA_VERSION,
                request_id: request.request_id,
                actuator_key: request.actuator_key,
                state: RequestState::Failed,
                result_code: Some("invalid_request".into()),
                result_message: Some("request does not match StarArm-102 gripper".into()),
            });
        } else {
            self.publish_actuator(&request.request_id, request.position_rad)?;
        }
        send(
            node,
            "actuator_status",
            self.actuator_status
                .as_ref()
                .expect("status assigned above"),
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

    fn start_controller_sync(&mut self) {
        if self.controller_sync_required
            && !self.controller_sync_running
            && self.active_motion.is_none()
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
                RosEvent::CurrentPose(result) => {
                    self.fk_pending = false;
                    match result {
                        Ok(pose) => self.current_tcp = Some(pose),
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
                            self.start_next_motion(node)?;
                        }
                        Err(error) => {
                            self.controller_sync_required = true;
                            self.last_error = Some(error.clone());
                            if let Some(pending) = self.motion_queue.pop_front() {
                                self.fail_motion(
                                    node,
                                    pending.job.request_id,
                                    format!("同步 ros2_control 控制器失败：{error}"),
                                    RequestAction::Apply,
                                )?;
                            }
                            self.start_next_motion(node)?;
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
                    let Some(active) = self
                        .active_motion
                        .take()
                        .filter(|active| active.request_id == request_id)
                    else {
                        continue;
                    };
                    self.reset_relative_baseline();
                    if active.cancelled {
                        self.start_next_motion(node)?;
                        self.publish_state(node)?;
                        continue;
                    }
                    match result {
                        Ok(result) => {
                            if active.complete_in_relative_mode
                                && let Err(error) = self.apply_control_mode(ControlMode::Relative)
                            {
                                self.fail_motion(
                                    node,
                                    request_id,
                                    error.to_string(),
                                    RequestAction::Apply,
                                )?;
                                continue;
                            }
                            self.motion_status.state = RequestState::Succeeded;
                            self.motion_status.result_code = Some(result.code.to_string());
                            self.motion_status.result_message = Some("普通运动执行完成".into());
                            send(node, "motion_status", &self.motion_status)?;
                            self.send_motion_result(node, &self.motion_status)?;
                        }
                        Err(error) => {
                            self.fail_motion(node, request_id, error, RequestAction::Apply)?;
                        }
                    }
                    self.start_next_motion(node)?;
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
        self.reset_relative_baseline();
        if self.active_motion.is_none()
            && let Some(pending) = self.motion_queue.pop_front()
        {
            let cancelled = MotionStatus {
                request_id: pending.job.request_id,
                state: RequestState::Cancelled,
                result_message: Some("普通运动请求已取消".into()),
                ..idle_status()
            };
            self.send_motion_result(node, &cancelled)?;
            self.start_next_motion(node)?;
        }
        if let Some(active) = self.active_motion.as_mut() {
            active.cancelled = true;
            let cancelled = MotionStatus {
                request_id: active.request_id.clone(),
                state: RequestState::Cancelled,
                result_message: Some("普通运动请求已取消；当前 MoveIt 动作结束后可再次运动".into()),
                ..idle_status()
            };
            self.send_motion_result(node, &cancelled)?;
        }
        self.motion_status = MotionStatus {
            request_id,
            acknowledged_action: "cancel".into(),
            state: RequestState::Cancelled,
            result_message: Some("普通运动请求已取消".into()),
            ..idle_status()
        };
        send(node, "motion_status", &self.motion_status)?;
        self.send_motion_result(node, &self.motion_status)
    }

    fn fail_motion(
        &mut self,
        node: &mut DoraNode,
        request_id: String,
        message: String,
        action: RequestAction,
    ) -> Result<()> {
        self.reset_relative_baseline();
        self.motion_status = MotionStatus {
            request_id,
            acknowledged_action: action_name(action).into(),
            state: RequestState::Failed,
            result_message: Some(message),
            ..idle_status()
        };
        send(node, "motion_status", &self.motion_status)?;
        self.send_motion_result(node, &self.motion_status)
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
            .is_some_and(|active| active.request_id == request_id && !active.cancelled)
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
            current_tool_pose: self.current_tcp.map(tool_pose),
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

    #[test]
    fn collision_pairs_have_a_stable_human_readable_form() {
        assert_eq!(
            format_pairs(&[("link1".into(), "link4".into())]),
            "link1↔link4"
        );
    }
}
