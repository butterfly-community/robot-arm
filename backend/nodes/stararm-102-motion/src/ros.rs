use std::{
    collections::{BTreeMap, BTreeSet},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc::{Receiver, Sender, channel},
    },
    thread,
    time::Duration,
};

use eyre::{Context, Result as EyreResult, bail, eyre};
use futures::{Future, FutureExt, StreamExt, executor::block_on};
use r2r::{
    ActionClientUntyped, ClientUntyped, Context as RosContext, Node, PublisherUntyped, QosProfile,
};
use robot_arm_messages::{Pose3, WorldScene};
use serde_json::{Value, json};
use stararm_102_model::{BASE_FRAME, GRIPPER_JOINT, JOINTS, TCP_FRAME};

use crate::core::Pose;

const MOVEIT_SUCCESS: i64 = 1;
const MOVE_GROUP_DEFAULT_PLANNING_TIME_S: f64 = 5.0;
const MOVE_GROUP_DEFAULT_SCALING_FACTOR: f64 = 1.0;
const ROBOT_LINK: u64 = 0;
const ALLOWED_COLLISION_MATRIX: u64 = 128;

#[derive(Debug)]
pub enum RosEvent {
    ControllerCommand(Value),
    ServoStatus(Value),
    CurrentPose(Result<Pose, String>),
    PoseMode(Result<(), String>),
    SyncFinished(Result<(), String>),
    MotionExecuting {
        request_id: String,
        points: u64,
        duration_s: f64,
        collision_pairs: Vec<(String, String)>,
    },
    MotionFinished {
        request_id: String,
        result: Result<MotionResult, String>,
    },
}

#[derive(Debug)]
pub struct MotionResult {
    pub code: i64,
}

#[derive(Clone)]
pub struct MotionJob {
    pub request_id: String,
    pub current: Vec<f64>,
    pub target: Vec<f64>,
    pub actuator: f64,
    pub options: BTreeMap<String, f64>,
}

#[derive(Clone)]
pub struct RosInterface {
    context: RosContext,
    pose_publisher: PublisherUntyped,
    hand_publisher: PublisherUntyped,
    state_publisher: PublisherUntyped,
    collision_ids: Arc<Mutex<BTreeSet<String>>>,
    command_type: Arc<ClientUntyped>,
    switch_controller: Arc<ClientUntyped>,
    pause_servo: Arc<ClientUntyped>,
    forward_kinematics: Arc<ClientUntyped>,
    inverse_kinematics: Arc<ClientUntyped>,
    state_validity: Arc<ClientUntyped>,
    planning_scene: Arc<ClientUntyped>,
    apply_planning_scene: Arc<ClientUntyped>,
    motion_sender: Sender<MotionJob>,
    event_sender: Sender<RosEvent>,
}

impl RosInterface {
    pub fn start(event_sender: Sender<RosEvent>, stop: Arc<AtomicBool>) -> EyreResult<Self> {
        let context = r2r::Context::create().context("初始化 ROS 2 context")?;
        let mut node = Node::create(context.clone(), "stararm_102_motion_node", "")
            .context("创建 ROS 2 motion 节点")?;
        let pose_publisher = node.create_publisher_untyped(
            "/servo_node/pose_target_cmds",
            "geometry_msgs/msg/PoseStamped",
            QosProfile::default(),
        )?;
        let hand_publisher = node.create_publisher_untyped(
            "/hand_controller/joint_trajectory",
            "trajectory_msgs/msg/JointTrajectory",
            QosProfile::default(),
        )?;
        let state_publisher = node.create_publisher_untyped(
            "/stararm102/joint_states",
            "sensor_msgs/msg/JointState",
            QosProfile::default(),
        )?;
        let controller_commands = node.subscribe_untyped(
            "/stararm102/joint_commands",
            "sensor_msgs/msg/JointState",
            QosProfile::default(),
        )?;
        let servo_status = node.subscribe_untyped(
            "/servo_node/status",
            "moveit_msgs/msg/ServoStatus",
            QosProfile::default(),
        )?;
        let command_type = node.create_client_untyped(
            "/servo_node/switch_command_type",
            "moveit_msgs/srv/ServoCommandType",
            QosProfile::default(),
        )?;
        let switch_controller = node.create_client_untyped(
            "/controller_manager/switch_controller",
            "controller_manager_msgs/srv/SwitchController",
            QosProfile::default(),
        )?;
        let pause_servo = node.create_client_untyped(
            "/servo_node/pause_servo",
            "std_srvs/srv/SetBool",
            QosProfile::default(),
        )?;
        let forward_kinematics = node.create_client_untyped(
            "/compute_fk",
            "moveit_msgs/srv/GetPositionFK",
            QosProfile::default(),
        )?;
        let inverse_kinematics = node.create_client_untyped(
            "/compute_ik",
            "moveit_msgs/srv/GetPositionIK",
            QosProfile::default(),
        )?;
        let state_validity = node.create_client_untyped(
            "/check_state_validity",
            "moveit_msgs/srv/GetStateValidity",
            QosProfile::default(),
        )?;
        let planning_scene = node.create_client_untyped(
            "/get_planning_scene",
            "moveit_msgs/srv/GetPlanningScene",
            QosProfile::default(),
        )?;
        let apply_planning_scene = node.create_client_untyped(
            "/apply_planning_scene",
            "moveit_msgs/srv/ApplyPlanningScene",
            QosProfile::default(),
        )?;
        let (motion_sender, motion_receiver) = channel();
        let node = Arc::new(Mutex::new(node));
        let spin_node = Arc::clone(&node);
        thread::spawn(move || {
            while !stop.load(Ordering::Relaxed) {
                spin_node
                    .lock()
                    .expect("ROS node mutex poisoned")
                    .spin_once(Duration::from_millis(10));
            }
        });
        forward_stream(
            controller_commands,
            event_sender.clone(),
            RosEvent::ControllerCommand,
        );
        forward_stream(servo_status, event_sender.clone(), RosEvent::ServoStatus);

        let interface = Self {
            context,
            pose_publisher,
            hand_publisher,
            state_publisher,
            collision_ids: Arc::new(Mutex::new(BTreeSet::new())),
            command_type: Arc::new(command_type),
            switch_controller: Arc::new(switch_controller),
            pause_servo: Arc::new(pause_servo),
            forward_kinematics: Arc::new(forward_kinematics),
            inverse_kinematics: Arc::new(inverse_kinematics),
            state_validity: Arc::new(state_validity),
            planning_scene: Arc::new(planning_scene),
            apply_planning_scene: Arc::new(apply_planning_scene),
            motion_sender,
            event_sender,
        };
        let worker = interface.clone();
        thread::spawn(move || worker.motion_loop(motion_receiver));
        interface.select_pose_mode();
        Ok(interface)
    }

    pub fn publish_state(&self, joints: &[f64], actuator: f64) -> EyreResult<()> {
        let names = [JOINTS.as_slice(), &[GRIPPER_JOINT]].concat();
        let positions = [joints, &[actuator]].concat();
        self.state_publisher.publish(json!({
            "name": names,
            "position": positions,
        }))?;
        Ok(())
    }

    pub fn publish_world_scene(&self, scene: &WorldScene) -> EyreResult<()> {
        let placement_sources = scene
            .placement_regions
            .iter()
            .filter_map(|region| region.source_object_id.as_deref())
            .collect::<BTreeSet<_>>();
        let next_ids = collision_object_ids(scene);
        let mut previous_ids = self
            .collision_ids
            .lock()
            .map_err(|_| eyre!("collision id lock poisoned"))?;
        let mut collision_objects = previous_ids
            .difference(&next_ids)
            .map(|id| {
                json!({
                "header": {"frame_id": scene.frame_id},
                "id": id,
                "operation": 1,
                })
            })
            .collect::<Vec<_>>();
        collision_objects.extend(
            scene
                .objects
                .iter()
                .filter(|object| !placement_sources.contains(object.object_id.as_str()))
                .map(|object| {
                    collision_box(
                        &scene.frame_id,
                        &object.object_id,
                        &object.pose,
                        object.size_m,
                    )
                }),
        );
        collision_objects.extend(scene.obstacles.iter().map(|obstacle| {
            collision_box(
                &scene.frame_id,
                &obstacle.obstacle_id,
                &obstacle.pose,
                obstacle.size_m,
            )
        }));
        if !collision_objects.is_empty() {
            self.apply_scene(json!({"world": {"collision_objects": collision_objects}}))?;
        }
        *previous_ids = next_ids;
        Ok(())
    }

    pub fn clear_world_scene(&self) -> EyreResult<()> {
        let mut ids = self
            .collision_ids
            .lock()
            .map_err(|_| eyre!("collision id lock poisoned"))?;
        if !ids.is_empty() {
            let collision_objects = ids
                .iter()
                .map(|id| {
                    json!({
                        "header": {"frame_id": BASE_FRAME},
                        "id": id,
                        "operation": 1,
                    })
                })
                .collect::<Vec<_>>();
            self.apply_scene(json!({"world": {"collision_objects": collision_objects}}))?;
        }
        ids.clear();
        Ok(())
    }

    pub fn remove_world_object(&self, object_id: &str) -> EyreResult<()> {
        self.apply_scene(json!({
            "world": {"collision_objects": [{
                "header": {"frame_id": BASE_FRAME},
                "id": object_id,
                "operation": 1,
            }]},
        }))?;
        self.collision_ids
            .lock()
            .map_err(|_| eyre!("collision id lock poisoned"))?
            .remove(object_id);
        Ok(())
    }

    pub fn attach_object(&self, object_id: &str, size_m: [f64; 3]) -> EyreResult<()> {
        self.apply_scene(json!({
            "robot_state": {
                "is_diff": true,
                "attached_collision_objects": [{
                    "link_name": TCP_FRAME,
                    "object": {
                        "header": {"frame_id": TCP_FRAME},
                        "id": object_id,
                        "primitives": [{"type": 1, "dimensions": size_m}],
                        "primitive_poses": [{
                            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                        }],
                        "operation": 0,
                    },
                    "touch_links": [TCP_FRAME, "link6", "link7_left", "link7_right"],
                }],
            },
        }))
    }

    pub fn detach_object(&self, object_id: &str) -> EyreResult<()> {
        self.apply_scene(json!({
            "robot_state": {
                "is_diff": true,
                "attached_collision_objects": [{
                    "link_name": TCP_FRAME,
                    "object": {"id": object_id, "operation": 1},
                }],
            },
        }))
    }

    fn apply_scene(&self, scene: Value) -> EyreResult<()> {
        let mut scene = scene;
        scene["is_diff"] = json!(true);
        let response = block_on(call(&self.apply_planning_scene, json!({"scene": scene})))?;
        if response["success"].as_bool() == Some(true) {
            Ok(())
        } else {
            bail!("MoveIt 拒绝更新规划场景")
        }
    }

    pub fn publish_pose(&self, pose: Pose) -> EyreResult<()> {
        let [x, y, z] = pose.position_m;
        let [qx, qy, qz, qw] = pose.orientation_xyzw;
        self.pose_publisher.publish(json!({
            "header": {"frame_id": BASE_FRAME},
            "pose": {
                "position": {"x": x, "y": y, "z": z},
                "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
            },
        }))?;
        Ok(())
    }

    pub fn publish_actuator(&self, position_rad: f64) -> EyreResult<()> {
        self.hand_publisher.publish(json!({
            "joint_names": [GRIPPER_JOINT],
            "points": [{
                "positions": [position_rad],
                "time_from_start": {"nanosec": 10_000_000},
            }],
        }))?;
        Ok(())
    }

    pub fn outputs_ready(&self) -> bool {
        [
            &self.pose_publisher,
            &self.hand_publisher,
            &self.state_publisher,
        ]
        .iter()
        .all(|publisher| {
            publisher
                .get_inter_process_subscription_count()
                .is_ok_and(|count| count > 0)
        })
    }

    pub fn request_current_pose(&self, joints: Vec<f64>) {
        let client = self.forward_kinematics.clone();
        let sender = self.event_sender.clone();
        thread::spawn(move || {
            let result = block_on(call(
                &client,
                json!({
                    "header": {"frame_id": BASE_FRAME},
                    "fk_link_names": [TCP_FRAME],
                    "robot_state": {
                        "joint_state": {"name": JOINTS, "position": joints},
                        "is_diff": false,
                    },
                }),
            ))
            .and_then(parse_fk)
            .map_err(|error| error.to_string());
            let _ = sender.send(RosEvent::CurrentPose(result));
        });
    }

    pub fn solve_ik(&self, pose: Pose, current: &[f64]) -> EyreResult<Vec<f64>> {
        let [x, y, z] = pose.position_m;
        let [qx, qy, qz, qw] = pose.orientation_xyzw;
        let response = block_on(call(
            &self.inverse_kinematics,
            json!({
                "ik_request": {
                    "group_name": "arm",
                    "robot_state": {
                        "joint_state": {"name": JOINTS, "position": current},
                        "is_diff": false,
                    },
                    "avoid_collisions": false,
                    "ik_link_name": TCP_FRAME,
                    "pose_stamped": {
                        "header": {"frame_id": BASE_FRAME},
                        "pose": {
                            "position": {"x": x, "y": y, "z": z},
                            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
                        },
                    },
                },
            }),
        ))?;
        let code = response["error_code"]["val"].as_i64().unwrap_or_default();
        if code != MOVEIT_SUCCESS {
            bail!("MoveIt IK 失败，错误码 {code}");
        }
        let names = response["solution"]["joint_state"]["name"]
            .as_array()
            .ok_or_else(|| eyre!("MoveIt IK 未返回关节名称"))?;
        let positions = response["solution"]["joint_state"]["position"]
            .as_array()
            .ok_or_else(|| eyre!("MoveIt IK 未返回关节位置"))?;
        JOINTS
            .iter()
            .map(|joint| {
                names
                    .iter()
                    .position(|name| name.as_str() == Some(joint))
                    .and_then(|index| positions.get(index))
                    .and_then(Value::as_f64)
                    .ok_or_else(|| eyre!("MoveIt IK 缺少关节 {joint}"))
            })
            .collect()
    }

    pub fn synchronize_controllers(&self) {
        let interface = self.clone();
        thread::spawn(move || {
            let result = block_on(interface.synchronize()).map_err(|error| error.to_string());
            let _ = interface.event_sender.send(RosEvent::SyncFinished(result));
        });
    }

    pub fn run_motion(&self, job: MotionJob) {
        if let Err(error) = self.motion_sender.send(job) {
            let job = error.0;
            let request_id = job.request_id.clone();
            let _ = self.event_sender.send(RosEvent::MotionFinished {
                request_id,
                result: Err("MoveIt action worker 已结束".into()),
            });
        }
    }

    fn motion_loop(self, receiver: Receiver<MotionJob>) {
        let mut actions = match MotionActions::new(self.context.clone()) {
            Ok(actions) => actions,
            Err(error) => {
                eprintln!("创建 MoveIt action worker 失败：{error:?}");
                return;
            }
        };
        for job in receiver {
            let request_id = job.request_id.clone();
            let result =
                block_on(self.motion(job, &mut actions)).map_err(|error| error.to_string());
            let _ = self
                .event_sender
                .send(RosEvent::MotionFinished { request_id, result });
        }
    }

    fn select_pose_mode(&self) {
        let client = self.command_type.clone();
        let sender = self.event_sender.clone();
        thread::spawn(move || {
            let result = block_on(call(&client, json!({"command_type": 2})))
                .and_then(|response| {
                    response["success"]
                        .as_bool()
                        .filter(|success| *success)
                        .map(|_| ())
                        .ok_or_else(|| eyre!("MoveIt Servo 拒绝 Pose 命令模式"))
                })
                .map_err(|error| error.to_string());
            let _ = sender.send(RosEvent::PoseMode(result));
        });
    }

    async fn synchronize(&self) -> EyreResult<()> {
        self.switch_controllers(vec![], vec!["arm_controller", "hand_controller"])
            .await?;
        self.switch_controllers(vec!["arm_controller", "hand_controller"], vec![])
            .await
    }

    async fn switch_controllers(
        &self,
        activate: Vec<&str>,
        deactivate: Vec<&str>,
    ) -> EyreResult<()> {
        let response = call(
            &self.switch_controller,
            json!({
                "activate_controllers": activate,
                "deactivate_controllers": deactivate,
                "strictness": 1,
            }),
        )
        .await?;
        if response["ok"].as_bool() == Some(true) {
            Ok(())
        } else {
            bail!(
                "controller manager rejected switch: {}",
                response["message"].as_str().unwrap_or_default()
            )
        }
    }

    async fn motion(
        &self,
        job: MotionJob,
        actions: &mut MotionActions,
    ) -> EyreResult<MotionResult> {
        self.pause(true).await?;
        let result = self.plan_and_execute(&job, actions).await;
        let resume = self.pause(false).await;
        match (result, resume) {
            (Ok(result), Ok(())) => Ok(result),
            (Err(error), _) => Err(error),
            (Ok(_), Err(error)) => Err(error),
        }
    }

    async fn pause(&self, paused: bool) -> EyreResult<()> {
        let response = call(&self.pause_servo, json!({"data": paused})).await?;
        if response["success"].as_bool() == Some(true) {
            Ok(())
        } else {
            bail!(
                "MoveIt Servo 拒绝{}：{}",
                if paused { "暂停" } else { "恢复" },
                response["message"].as_str().unwrap_or_default()
            )
        }
    }

    async fn plan_and_execute(
        &self,
        job: &MotionJob,
        actions: &mut MotionActions,
    ) -> EyreResult<MotionResult> {
        let mut collision_pairs = vec![];
        let mut result = self.request_plan(job, None, actions)?;
        let mut code = result["error_code"]["val"].as_i64().unwrap_or_default();
        if code != MOVEIT_SUCCESS {
            collision_pairs = self.collision_pairs(&job.current).await?;
            if collision_pairs.is_empty() {
                bail!("MoveIt 规划失败，错误码 {code}");
            }
            let matrix = self.allowed_collision_matrix(&collision_pairs).await?;
            result = self.request_plan(job, Some(matrix), actions)?;
            code = result["error_code"]["val"].as_i64().unwrap_or_default();
            if code != MOVEIT_SUCCESS {
                bail!("MoveIt 规划失败，错误码 {code}");
            }
        }
        let trajectory = result["planned_trajectory"].clone();
        let points = trajectory["joint_trajectory"]["points"]
            .as_array()
            .ok_or_else(|| eyre!("MoveIt 返回空轨迹"))?;
        let end = points.last().ok_or_else(|| eyre!("MoveIt 返回空轨迹"))?;
        let duration_s = end["time_from_start"]["sec"].as_i64().unwrap_or_default() as f64
            + end["time_from_start"]["nanosec"]
                .as_u64()
                .unwrap_or_default() as f64
                * 1e-9;
        let _ = self.event_sender.send(RosEvent::MotionExecuting {
            request_id: job.request_id.clone(),
            points: points.len() as u64,
            duration_s,
            collision_pairs: collision_pairs.clone(),
        });
        self.publish_actuator(job.actuator)?;
        let result = actions.execute(json!({
            "trajectory": trajectory,
            "controller_names": ["arm_controller"],
        }))?;
        let code = result["error_code"]["val"].as_i64().unwrap_or_default();
        if code != MOVEIT_SUCCESS {
            bail!("普通运动执行失败，MoveIt 错误码 {code}");
        }
        Ok(MotionResult { code })
    }

    fn request_plan(
        &self,
        job: &MotionJob,
        matrix: Option<Value>,
        actions: &mut MotionActions,
    ) -> EyreResult<Value> {
        let constraints = JOINTS
            .iter()
            .zip(&job.target)
            .map(
                |(name, position)| json!({"joint_name": name, "position": position, "weight": 1.0}),
            )
            .collect::<Vec<_>>();
        let mut request = json!({
            "group_name": "arm",
            "pipeline_id": "ompl",
            "allowed_planning_time": MOVE_GROUP_DEFAULT_PLANNING_TIME_S,
            "max_velocity_scaling_factor": MOVE_GROUP_DEFAULT_SCALING_FACTOR,
            "max_acceleration_scaling_factor": MOVE_GROUP_DEFAULT_SCALING_FACTOR,
            "start_state": {
                "joint_state": {"name": JOINTS, "position": job.current},
                "is_diff": false,
            },
            "goal_constraints": [{
                "name": "joint_target",
                "joint_constraints": constraints,
            }],
        });
        if let Some(value) = job.options.get("velocity_scaling") {
            request["max_velocity_scaling_factor"] = json!(value);
        }
        if let Some(value) = job.options.get("acceleration_scaling") {
            request["max_acceleration_scaling_factor"] = json!(value);
        }
        let mut planning_options = json!({"plan_only": true});
        if let Some(matrix) = matrix {
            planning_options["planning_scene_diff"] = json!({
                "is_diff": true,
                "allowed_collision_matrix": matrix,
            });
        }
        actions.plan(json!({"request": request, "planning_options": planning_options}))
    }

    async fn collision_pairs(&self, current: &[f64]) -> EyreResult<Vec<(String, String)>> {
        let response = call(
            &self.state_validity,
            json!({
                "robot_state": {
                    "joint_state": {"name": JOINTS, "position": current},
                    "is_diff": false,
                },
                "group_name": "arm",
            }),
        )
        .await?;
        if response["valid"].as_bool() == Some(true) {
            return Ok(vec![]);
        }
        let mut pairs = BTreeSet::new();
        for contact in response["contacts"].as_array().into_iter().flatten() {
            if contact["body_type_1"].as_u64() != Some(ROBOT_LINK)
                || contact["body_type_2"].as_u64() != Some(ROBOT_LINK)
            {
                continue;
            }
            let first = contact["contact_body_1"].as_str().unwrap_or_default();
            let second = contact["contact_body_2"].as_str().unwrap_or_default();
            if first != second {
                let mut pair = [first.to_owned(), second.to_owned()];
                pair.sort();
                pairs.insert((pair[0].clone(), pair[1].clone()));
            }
        }
        Ok(pairs.into_iter().collect())
    }

    async fn allowed_collision_matrix(&self, pairs: &[(String, String)]) -> EyreResult<Value> {
        let mut response = call(
            &self.planning_scene,
            json!({"components": {"components": ALLOWED_COLLISION_MATRIX}}),
        )
        .await?;
        let matrix = &mut response["scene"]["allowed_collision_matrix"];
        allow_pairs(matrix, pairs)?;
        Ok(matrix.take())
    }
}

fn collision_box(frame_id: &str, id: &str, pose: &Pose3, size_m: [f64; 3]) -> Value {
    let [x, y, z] = pose.position_m;
    let [qx, qy, qz, qw] = pose.orientation_xyzw;
    json!({
        "header": {"frame_id": frame_id},
        "id": id,
        "primitives": [{"type": 1, "dimensions": size_m}],
        "primitive_poses": [{
            "position": {"x": x, "y": y, "z": z},
            "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
        }],
        "operation": 0,
    })
}

fn collision_object_ids(scene: &WorldScene) -> BTreeSet<String> {
    let placement_sources = scene
        .placement_regions
        .iter()
        .filter_map(|region| region.source_object_id.as_deref())
        .collect::<BTreeSet<_>>();
    scene
        .objects
        .iter()
        .filter(|object| !placement_sources.contains(object.object_id.as_str()))
        .map(|object| object.object_id.clone())
        .chain(
            scene
                .obstacles
                .iter()
                .map(|obstacle| obstacle.obstacle_id.clone()),
        )
        .collect()
}

struct MotionActions {
    node: Node,
    move_group: ActionClientUntyped,
    execute_trajectory: ActionClientUntyped,
}

impl MotionActions {
    fn new(context: RosContext) -> EyreResult<Self> {
        let mut node = Node::create(context, "stararm_102_motion_action_worker", "")?;
        let move_group =
            node.create_action_client_untyped("/move_action", "moveit_msgs/action/MoveGroup")?;
        let execute_trajectory = node.create_action_client_untyped(
            "/execute_trajectory",
            "moveit_msgs/action/ExecuteTrajectory",
        )?;
        Ok(Self {
            node,
            move_group,
            execute_trajectory,
        })
    }

    fn plan(&mut self, goal: Value) -> EyreResult<Value> {
        action(&mut self.node, &self.move_group, goal)
    }

    fn execute(&mut self, goal: Value) -> EyreResult<Value> {
        action(&mut self.node, &self.execute_trajectory, goal)
    }
}

fn action(node: &mut Node, client: &ActionClientUntyped, goal: Value) -> EyreResult<Value> {
    spin_until(node, Node::is_available(client)?)?;
    let (_handle, result, _feedback) = spin_until(node, client.send_goal_request(goal)?)?;
    let (status, value) = spin_until(node, result)?;
    value
        .map_err(|error| eyre!(error))
        .and_then(|value| match status {
            r2r::GoalStatus::Succeeded => Ok(value),
            other => bail!("MoveIt action ended with {other}"),
        })
}

fn spin_until<F: Future>(node: &mut Node, future: F) -> F::Output {
    futures::pin_mut!(future);
    loop {
        if let Some(output) = future.as_mut().now_or_never() {
            return output;
        }
        node.spin_once(Duration::from_millis(10));
    }
}

async fn call(client: &ClientUntyped, request: Value) -> EyreResult<Value> {
    Node::is_available(client)?.await?;
    client
        .request(request)?
        .await?
        .map_err(|error| eyre!(error))
}

fn parse_fk(response: Value) -> EyreResult<Pose> {
    let code = response["error_code"]["val"].as_i64().unwrap_or_default();
    if code != MOVEIT_SUCCESS {
        bail!("MoveIt FK 失败，错误码 {code}");
    }
    let pose = response["pose_stamped"]
        .as_array()
        .and_then(|poses| poses.first())
        .and_then(|value| value.get("pose"))
        .ok_or_else(|| eyre!("MoveIt FK 没有返回 TCP"))?;
    Ok(Pose {
        position_m: [
            number(&pose["position"]["x"]),
            number(&pose["position"]["y"]),
            number(&pose["position"]["z"]),
        ],
        orientation_xyzw: [
            number(&pose["orientation"]["x"]),
            number(&pose["orientation"]["y"]),
            number(&pose["orientation"]["z"]),
            number(&pose["orientation"]["w"]),
        ],
    })
}

fn number(value: &Value) -> f64 {
    value.as_f64().unwrap_or_default()
}

fn allow_pairs(matrix: &mut Value, pairs: &[(String, String)]) -> EyreResult<()> {
    let mut names = matrix["entry_names"]
        .as_array()
        .cloned()
        .ok_or_else(|| eyre!("MoveIt AllowedCollisionMatrix 缺少 entry_names"))?;
    let mut values = matrix["entry_values"]
        .as_array()
        .cloned()
        .ok_or_else(|| eyre!("MoveIt AllowedCollisionMatrix 缺少 entry_values"))?;
    if values.len() != names.len()
        || values.iter().any(|row| {
            row["enabled"]
                .as_array()
                .is_none_or(|enabled| enabled.len() != names.len())
        })
    {
        bail!("MoveIt AllowedCollisionMatrix 不是方阵");
    }
    for name in pairs.iter().flat_map(|(first, second)| [first, second]) {
        if names.iter().any(|entry| entry.as_str() == Some(name)) {
            continue;
        }
        names.push(json!(name));
        for row in &mut values {
            row["enabled"]
                .as_array_mut()
                .expect("matrix row checked above")
                .push(json!(false));
        }
        values.push(json!({"enabled": vec![false; names.len()]}));
    }
    let indices = names
        .iter()
        .enumerate()
        .filter_map(|(index, name)| name.as_str().map(|name| (name.to_owned(), index)))
        .collect::<BTreeMap<_, _>>();
    for (first, second) in pairs {
        let first = indices[first];
        let second = indices[second];
        values[first]["enabled"][second] = json!(true);
        values[second]["enabled"][first] = json!(true);
    }
    matrix["entry_names"] = Value::Array(names);
    matrix["entry_values"] = Value::Array(values);
    Ok(())
}

fn forward_stream(
    mut stream: impl futures::Stream<Item = r2r::Result<Value>> + Unpin + Send + 'static,
    sender: Sender<RosEvent>,
    wrap: fn(Value) -> RosEvent,
) {
    thread::spawn(move || {
        block_on(async move {
            while let Some(message) = stream.next().await {
                if let Ok(message) = message {
                    let _ = sender.send(wrap(message));
                }
            }
        });
    });
}

#[cfg(test)]
mod tests {
    use robot_arm_messages::{PlacementRegion, Pose3, SceneObject, WorldScene};

    use super::*;

    #[test]
    fn placement_container_bounds_are_not_published_as_solid_obstacles() {
        let pose = Pose3 {
            position_m: [0.0; 3],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        };
        let scene = WorldScene {
            schema_version: 3,
            sequence: 1,
            sample_time_ns: 2,
            frame_id: "base_link".into(),
            objects: ["cube", "bin"]
                .map(|id| SceneObject {
                    object_id: id.into(),
                    label: id.into(),
                    pose: pose.clone(),
                    size_m: [0.1; 3],
                    confidence: 1.0,
                    graspable: id == "cube",
                })
                .to_vec(),
            placement_regions: vec![PlacementRegion {
                region_id: "bin-interior".into(),
                label: "interior".into(),
                pose,
                size_m: [0.1; 3],
                source_object_id: Some("bin".into()),
            }],
            obstacles: vec![],
        };
        assert_eq!(
            collision_object_ids(&scene),
            BTreeSet::from(["cube".into()])
        );
    }

    #[test]
    fn collision_matrix_expands_symmetrically() {
        let mut matrix = json!({
            "entry_names": ["link1"],
            "entry_values": [{"enabled": [false]}],
        });
        allow_pairs(&mut matrix, &[("link1".into(), "link4".into())]).unwrap();
        assert_eq!(matrix["entry_names"], json!(["link1", "link4"]));
        assert_eq!(matrix["entry_values"][0]["enabled"], json!([false, true]));
        assert_eq!(matrix["entry_values"][1]["enabled"], json!([true, false]));
    }

    #[test]
    fn invalid_collision_matrix_is_rejected() {
        let mut matrix = json!({
            "entry_names": ["link1", "link2"],
            "entry_values": [{"enabled": [false]}],
        });
        assert!(allow_pairs(&mut matrix, &[]).is_err());
    }
}
