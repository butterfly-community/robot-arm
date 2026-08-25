use anyhow::{Context, Result, bail};
use async_stream::stream;
use axum::{
    Json, Router,
    extract::{Path, State},
    http::StatusCode,
    response::sse::{Event, KeepAlive, Sse},
    routing::{get, post},
};
use hidapi::HidApi;
use nolo_usb_server::{
    fusion::{ControllerFusion, FusionDiagnostics, HmdFusion},
    gyro_bias::{GyroBiasStore, SOURCE_COUNT},
    position_filter::PositionFilter,
    protocol::{REPORT_SIZE, SUPPORTED_DEVICES, decode_report},
    sample::{InterleavedSampleTracker, SampleObservation, SampleTracker},
    simulator::{REPORT_PERIOD_SECONDS, VirtualNolo},
    teleop::{TeleopIntent, TeleopIntentMachine, TeleopSample, TeleopStopReason},
};
use serde::Serialize;
use serde_json::json;
use stararm102_control::{
    ArmModel, HomeRequestAction, HomeRequestFrame, HomeState, HomeStatusFrame, MoveItController,
    RelativeIntent, RelativeIntentState, ServoFeedbackFrame, SimulationSnapshot,
};
use std::{
    convert::Infallible,
    net::{IpAddr, SocketAddr},
    path::{Path as FilePath, PathBuf},
    sync::{
        Arc, RwLock,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    thread,
    time::{Duration, Instant},
};
use tokio::sync::{broadcast, watch};
use tokio::{
    io::{AsyncBufReadExt, AsyncWriteExt, BufReader},
    net::{UnixListener, UnixStream},
};
use tower_http::services::ServeDir;

const DEVICE_NAMES: [&str; 3] = [
    "NOLO CV1: Controller 0 (USB)",
    "NOLO CV1: Controller 1 (USB)",
    "NOLO CV1: Head Marker (USB)",
];
const VIRTUAL_DEVICE_NAMES: [&str; 3] = [
    "NOLO CV1: Controller 0 (Virtual USB)",
    "NOLO CV1: Controller 1 (Virtual USB)",
    "NOLO CV1: Head Marker (Virtual USB)",
];
const ARM_SIM_INTERVAL: Duration = Duration::from_millis(10);

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
enum ArmOutputBackend {
    #[default]
    Simulation,
    Hardware,
}

impl ArmOutputBackend {
    fn index(self) -> usize {
        self as usize
    }

    fn parse(value: &str) -> Option<Self> {
        match value {
            "simulation" => Some(Self::Simulation),
            "hardware" => Some(Self::Hardware),
            _ => None,
        }
    }

    fn simulation_only(self) -> bool {
        self == Self::Simulation
    }

    fn controller_backend(self) -> &'static str {
        match self {
            Self::Simulation => "moveit_servo_simulation",
            Self::Hardware => "moveit_servo_hardware",
        }
    }
}

#[derive(Clone, Debug, Serialize)]
struct PoseFrame {
    time_ns: u64,
    source_id: u8,
    sample_rate_hz: u16,
    device: &'static str,
    simulated: bool,
    position: [f32; 3],
    filtered_position: Option<[f32; 3]>,
    orientation: [f32; 4],
    communication_fresh: bool,
    sample_changed: bool,
    unchanged_ms: u64,
    hmd_unchanged_ms: u64,
    hmd_relay_online: bool,
    sample_sequence: u8,
    hmd_sequence: u8,
    samples_received: u64,
    samples_missed: u64,
    duplicate_reports: u64,
    measured_rate_hz: Option<f32>,
    sample_jitter_ms: Option<f32>,
    acceleration_error_degrees: f32,
    accelerometer_ignored: bool,
    acceleration_recovery: bool,
    fusion_initialising: bool,
    gyro_bias_dps: [f32; 3],
    gyro_calibration_active: bool,
    gyro_calibration_complete: bool,
    gyro_calibration_progress: f32,
    menu_pressed: bool,
    trigger_pressed: bool,
    squeeze_pressed: bool,
}

#[derive(Clone, Debug, Serialize)]
struct StatusPayload {
    status: String,
    #[serde(rename = "latestFrames")]
    latest_frames: [Option<PoseFrame>; SOURCE_COUNT],
    #[serde(rename = "latestTeleopIntent")]
    latest_teleop_intent: TeleopIntent,
    #[serde(rename = "latestArmSimulation")]
    latest_arm_simulation: SimulationSnapshot,
    #[serde(rename = "latestArmHardware")]
    latest_arm_hardware: SimulationSnapshot,
    #[serde(rename = "armOutputBackend")]
    arm_output_backend: ArmOutputBackend,
    #[serde(rename = "armHomeSimulation")]
    arm_home_simulation: HomeStatusFrame,
    #[serde(rename = "armHomeHardware")]
    arm_home_hardware: HomeStatusFrame,
    #[serde(rename = "simulationRequested")]
    simulation_requested: bool,
    #[serde(rename = "simulationActive")]
    simulation_active: bool,
}

#[derive(Clone, Debug)]
enum ServerEvent {
    Status(String),
    Pose(Box<PoseFrame>),
}

#[derive(Debug)]
struct Snapshot {
    status: String,
    latest_frames: [Option<PoseFrame>; SOURCE_COUNT],
}

#[derive(Clone)]
struct AppState {
    snapshot: Arc<RwLock<Snapshot>>,
    events: broadcast::Sender<ServerEvent>,
    calibration_requests: Arc<[AtomicBool; SOURCE_COUNT]>,
    pose_calibration_requests: Arc<[AtomicBool; SOURCE_COUNT]>,
    teleop_intent: watch::Sender<TeleopIntent>,
    arm_snapshots: [watch::Sender<SimulationSnapshot>; 2],
    arm_output_backend: Arc<RwLock<ArmOutputBackend>>,
    arm_output_generation: Arc<AtomicU64>,
    arm_home_statuses: [watch::Sender<HomeStatusFrame>; 2],
    arm_home_requests: [watch::Sender<Option<HomeRequestFrame>>; 2],
    arm_home_request_id: Arc<AtomicU64>,
    shutdown: watch::Sender<bool>,
    simulation_requested: Arc<AtomicBool>,
    simulation_active: Arc<AtomicBool>,
}

impl AppState {
    fn new() -> Self {
        let (events, _) = broadcast::channel(256);
        let initial_teleop_intent = TeleopIntentMachine::new().latest().clone();
        let (teleop_intent, _) = watch::channel(initial_teleop_intent);
        let initial_arm_simulation =
            MoveItController::new(ArmModel::embedded().expect("invalid embedded FL profile"))
                .latest()
                .clone();
        let (arm_simulation, _) = watch::channel(initial_arm_simulation);
        let initial_arm_hardware = MoveItController::new_for_backend(
            ArmModel::embedded().expect("invalid embedded FL profile"),
            false,
            ArmOutputBackend::Hardware.controller_backend(),
        )
        .latest()
        .clone();
        let (arm_hardware, _) = watch::channel(initial_arm_hardware);
        let arm_home_statuses = std::array::from_fn(|_| watch::channel(Default::default()).0);
        let arm_home_requests = std::array::from_fn(|_| watch::channel(None).0);
        let (shutdown, _) = watch::channel(false);
        Self {
            snapshot: Arc::new(RwLock::new(Snapshot {
                status: "正在连接 NOLO USB…".to_owned(),
                latest_frames: std::array::from_fn(|_| None),
            })),
            events,
            calibration_requests: Arc::new(std::array::from_fn(|_| AtomicBool::new(false))),
            pose_calibration_requests: Arc::new(std::array::from_fn(|_| AtomicBool::new(false))),
            teleop_intent,
            arm_snapshots: [arm_simulation, arm_hardware],
            arm_output_backend: Arc::new(RwLock::new(ArmOutputBackend::default())),
            arm_output_generation: Arc::new(AtomicU64::new(0)),
            arm_home_statuses,
            arm_home_requests,
            arm_home_request_id: Arc::new(AtomicU64::new(0)),
            shutdown,
            simulation_requested: Arc::new(AtomicBool::new(false)),
            simulation_active: Arc::new(AtomicBool::new(false)),
        }
    }

    fn set_status(&self, status: impl Into<String>) {
        let status = status.into();
        let changed = {
            let mut snapshot = self.snapshot.write().unwrap();
            if snapshot.status == status {
                false
            } else {
                snapshot.status.clone_from(&status);
                true
            }
        };
        if changed {
            let _ = self.events.send(ServerEvent::Status(status));
        }
    }

    fn set_pose(&self, pose: PoseFrame) {
        let source_id = usize::from(pose.source_id);
        let mut snapshot = self.snapshot.write().unwrap();
        snapshot.latest_frames[source_id] = Some(pose.clone());
        drop(snapshot);
        let _ = self.events.send(ServerEvent::Pose(Box::new(pose)));
    }

    fn payload(&self) -> StatusPayload {
        let snapshot = self.snapshot.read().unwrap();
        StatusPayload {
            status: snapshot.status.clone(),
            latest_frames: snapshot.latest_frames.clone(),
            latest_teleop_intent: self.teleop_intent.borrow().clone(),
            latest_arm_simulation: self.arm_snapshot(ArmOutputBackend::Simulation),
            latest_arm_hardware: self.arm_snapshot(ArmOutputBackend::Hardware),
            arm_output_backend: self.arm_output_backend(),
            arm_home_simulation: self.arm_home_status(ArmOutputBackend::Simulation),
            arm_home_hardware: self.arm_home_status(ArmOutputBackend::Hardware),
            simulation_requested: self.simulation_requested.load(Ordering::Acquire),
            simulation_active: self.simulation_active.load(Ordering::Acquire),
        }
    }

    fn set_teleop_intent(&self, intent: TeleopIntent) {
        self.teleop_intent.send_replace(intent);
    }

    fn set_arm_snapshot(&self, backend: ArmOutputBackend, snapshot: SimulationSnapshot) {
        self.arm_snapshots[backend.index()].send_replace(snapshot);
    }

    fn arm_snapshot(&self, backend: ArmOutputBackend) -> SimulationSnapshot {
        self.arm_snapshots[backend.index()].borrow().clone()
    }

    fn arm_output_backend(&self) -> ArmOutputBackend {
        *self.arm_output_backend.read().unwrap()
    }

    fn arm_home_status(&self, backend: ArmOutputBackend) -> HomeStatusFrame {
        self.arm_home_statuses[backend.index()].borrow().clone()
    }

    fn set_arm_home_status(&self, backend: ArmOutputBackend, status: HomeStatusFrame) {
        self.arm_home_statuses[backend.index()].send_replace(status);
    }

    fn arm_home_request(&self, backend: ArmOutputBackend) -> Option<HomeRequestFrame> {
        self.arm_home_requests[backend.index()].borrow().clone()
    }

    fn send_arm_home_request(&self, backend: ArmOutputBackend, request: HomeRequestFrame) {
        self.arm_home_requests[backend.index()].send_replace(Some(request));
    }

    fn acknowledge_arm_home_request(
        &self,
        backend: ArmOutputBackend,
        request_id: u64,
        action: Option<HomeRequestAction>,
    ) {
        let sender = &self.arm_home_requests[backend.index()];
        let acknowledged = sender.borrow().as_ref().is_some_and(|request| {
            request.request_id == request_id && Some(request.action) == action
        });
        if acknowledged {
            sender.send_replace(None);
        }
    }

    fn reset_arm_home_session(&self, backend: ArmOutputBackend) {
        self.set_arm_home_status(backend, HomeStatusFrame::default());
        self.arm_home_requests[backend.index()].send_replace(None);
    }

    fn next_arm_home_request_id(&self) -> u64 {
        self.arm_home_request_id.fetch_add(1, Ordering::AcqRel) + 1
    }

    fn arm_home_blocks_teleop(&self, backend: ArmOutputBackend) -> bool {
        self.arm_home_status(backend).state.blocks_teleop()
    }

    fn select_arm_output(&self, backend: ArmOutputBackend) {
        let changed = {
            let mut selected = self.arm_output_backend.write().unwrap();
            if *selected == backend {
                false
            } else {
                *selected = backend;
                true
            }
        };
        if changed {
            self.arm_output_generation.fetch_add(1, Ordering::AcqRel);
        }
    }

    fn request_gyro_calibration(&self, source_id: usize) {
        self.calibration_requests[source_id].store(true, Ordering::Release);
    }

    fn take_gyro_calibration_request(&self, source_id: usize) -> bool {
        self.calibration_requests[source_id].swap(false, Ordering::AcqRel)
    }

    fn request_pose_calibration(&self, source_id: usize) {
        self.calibration_requests[source_id].store(false, Ordering::Release);
        self.pose_calibration_requests[source_id].store(true, Ordering::Release);
    }

    fn take_pose_calibration_request(&self, source_id: usize) -> bool {
        self.pose_calibration_requests[source_id].swap(false, Ordering::AcqRel)
    }

    fn notify_shutdown(&self) {
        self.shutdown.send_replace(true);
    }

    fn request_simulation(&self, enabled: bool) {
        self.simulation_requested.store(enabled, Ordering::Release);
        self.set_status(if enabled {
            "正在启动 NOLO 虚拟 USB…"
        } else {
            "正在停止 NOLO 虚拟 USB…"
        });
    }

    fn simulation_requested(&self) -> bool {
        self.simulation_requested.load(Ordering::Acquire)
    }

    fn set_simulation_active(&self, active: bool) {
        self.simulation_active.store(active, Ordering::Release);
    }

    fn clear_calibration_requests(&self) {
        for source_id in 0..SOURCE_COUNT {
            self.calibration_requests[source_id].store(false, Ordering::Release);
            self.pose_calibration_requests[source_id].store(false, Ordering::Release);
        }
    }

    fn mark_all_offline(&self, time_ns: u64) {
        let changed_poses = {
            let mut snapshot = self.snapshot.write().unwrap();
            let mut changed = Vec::new();
            for frame in snapshot.latest_frames.iter_mut().flatten() {
                if !frame.communication_fresh && !frame.hmd_relay_online {
                    continue;
                }
                frame.time_ns = time_ns;
                frame.communication_fresh = false;
                frame.sample_changed = false;
                frame.filtered_position = None;
                frame.hmd_relay_online = false;
                frame.menu_pressed = false;
                changed.push(frame.clone());
            }
            changed
        };
        for pose in changed_poses {
            let _ = self.events.send(ServerEvent::Pose(Box::new(pose)));
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let config = Config::parse()?;
    let address = SocketAddr::new(config.host, config.port);
    let listener = tokio::net::TcpListener::bind(address)
        .await
        .with_context(|| format!("failed to listen on http://{address}/"))?;
    // Reserve both externally visible endpoints before touching USB.  A
    // second instance must fail without disrupting the running instance.
    let servo_listener = bind_servo_ipc(&config.servo_ipc_path)?;
    let hardware_servo_listener = match bind_servo_ipc(&config.hardware_servo_ipc_path) {
        Ok(listener) => listener,
        Err(error) => {
            drop(servo_listener);
            remove_servo_socket(&config.servo_ipc_path)?;
            return Err(error);
        }
    };
    let state = AppState::new();
    spawn_usb_reader(state.clone(), config.gyro_calibration_file);
    let servo_task = tokio::spawn(servo_ipc_accept_loop(
        servo_listener,
        state.clone(),
        ArmOutputBackend::Simulation,
    ));
    let hardware_servo_task = tokio::spawn(servo_ipc_accept_loop(
        hardware_servo_listener,
        state.clone(),
        ArmOutputBackend::Hardware,
    ));

    let static_files = ServeDir::new(&config.static_dir).append_index_html_on_directories(true);
    let app = Router::new()
        .route("/events", get(events))
        .route("/api/status", get(status))
        .route(
            "/api/gyro-calibration/{source_id}",
            post(start_gyro_calibration),
        )
        .route(
            "/api/pose-calibration/{source_id}",
            post(start_pose_calibration),
        )
        .route("/api/simulation/start", post(start_simulation))
        .route("/api/simulation/stop", post(stop_simulation))
        .route("/api/arm-output/{backend}", post(select_arm_output))
        .route("/api/arm-home/{backend}/{action}", post(arm_home))
        .fallback_service(static_files)
        .with_state(state.clone());

    println!("NOLO direct USB viewer: http://{address}/");
    println!("Static files: {}", config.static_dir.display());
    let server_result = axum::serve(listener, app)
        .with_graceful_shutdown(shutdown_signal(state))
        .await
        .context("web server failed");
    let _ = servo_task.await;
    let _ = hardware_servo_task.await;
    remove_servo_socket(&config.servo_ipc_path)?;
    remove_servo_socket(&config.hardware_servo_ipc_path)?;
    server_result
}

async fn status(State(state): State<AppState>) -> Json<StatusPayload> {
    Json(state.payload())
}

async fn start_simulation(State(state): State<AppState>) -> (StatusCode, Json<serde_json::Value>) {
    state.request_simulation(true);
    (
        StatusCode::ACCEPTED,
        Json(json!({
            "status": "queued",
            "simulation_requested": true,
            "instruction": "controller 0 will calibrate automatically before looping"
        })),
    )
}

async fn stop_simulation(State(state): State<AppState>) -> (StatusCode, Json<serde_json::Value>) {
    state.request_simulation(false);
    (
        StatusCode::ACCEPTED,
        Json(json!({
            "status": "stopped",
            "simulation_requested": false
        })),
    )
}

async fn select_arm_output(
    Path(backend): Path<String>,
    State(state): State<AppState>,
) -> (StatusCode, Json<serde_json::Value>) {
    let Some(backend) = ArmOutputBackend::parse(&backend) else {
        return (
            StatusCode::BAD_REQUEST,
            Json(json!({ "error": "backend must be simulation or hardware" })),
        );
    };
    state.select_arm_output(backend);
    (
        StatusCode::OK,
        Json(json!({
            "arm_output_backend": backend
        })),
    )
}

async fn arm_home(
    Path((backend, action)): Path<(String, String)>,
    State(state): State<AppState>,
) -> (StatusCode, Json<serde_json::Value>) {
    let Some(backend) = ArmOutputBackend::parse(&backend) else {
        return (
            StatusCode::BAD_REQUEST,
            Json(json!({ "error": "backend must be simulation or hardware" })),
        );
    };
    let current = state.arm_home_status(backend);
    match action.as_str() {
        "plan" => {
            if current.state.blocks_teleop() {
                return (
                    StatusCode::CONFLICT,
                    Json(json!({ "error": "已有回零操作正在进行" })),
                );
            }
            let request_id = state.next_arm_home_request_id();
            let status = HomeStatusFrame {
                request_id,
                state: HomeState::Planning,
                message: "正在规划回零轨迹".to_owned(),
                ..HomeStatusFrame::default()
            };
            state.set_arm_home_status(backend, status);
            state.send_arm_home_request(
                backend,
                HomeRequestFrame {
                    request_id,
                    action: HomeRequestAction::Plan,
                },
            );
            (
                StatusCode::ACCEPTED,
                Json(json!({
                    "request_id": request_id,
                    "state": "planning",
                    "auto_execute": true
                })),
            )
        }
        "cancel" => {
            if current.request_id == 0 || !current.state.blocks_teleop() {
                return (
                    StatusCode::OK,
                    Json(json!({ "state": "idle", "message": "没有活动的回零操作" })),
                );
            }
            state.send_arm_home_request(
                backend,
                HomeRequestFrame {
                    request_id: current.request_id,
                    action: HomeRequestAction::Cancel,
                },
            );
            state.set_arm_home_status(
                backend,
                HomeStatusFrame {
                    state: HomeState::Cancelled,
                    message: "正在取消回零操作".to_owned(),
                    ..current.clone()
                },
            );
            (
                StatusCode::ACCEPTED,
                Json(json!({ "request_id": current.request_id, "state": "cancelled" })),
            )
        }
        _ => (
            StatusCode::BAD_REQUEST,
            Json(json!({ "error": "action must be plan or cancel" })),
        ),
    }
}

async fn start_gyro_calibration(
    Path(source_id): Path<usize>,
    State(state): State<AppState>,
) -> (StatusCode, Json<serde_json::Value>) {
    if source_id >= SOURCE_COUNT {
        return (
            StatusCode::BAD_REQUEST,
            Json(json!({ "error": "source_id must be 0, 1, or 2" })),
        );
    }
    state.request_gyro_calibration(source_id);
    (
        StatusCode::ACCEPTED,
        Json(json!({
            "status": "queued",
            "source_id": source_id,
            "instruction": "keep the selected device motionless for 3 seconds"
        })),
    )
}

async fn start_pose_calibration(
    Path(source_id): Path<usize>,
    State(state): State<AppState>,
) -> (StatusCode, Json<serde_json::Value>) {
    if source_id >= SOURCE_COUNT {
        return (
            StatusCode::BAD_REQUEST,
            Json(json!({ "error": "source_id must be 0, 1, or 2" })),
        );
    }
    state.request_pose_calibration(source_id);
    (
        StatusCode::ACCEPTED,
        Json(json!({
            "status": "queued",
            "source_id": source_id,
            "instruction": "keep the selected device motionless for 6 seconds"
        })),
    )
}

async fn events(
    State(state): State<AppState>,
) -> Sse<impl futures_core::Stream<Item = Result<Event, Infallible>>> {
    let initial = state.payload();
    let mut receiver = state.events.subscribe();
    let mut shutdown = state.shutdown.subscribe();
    let output = stream! {
        if *shutdown.borrow() {
            return;
        }
        yield Ok(sse_event("status", &json!({ "status": initial.status })));
        for pose in initial.latest_frames.into_iter().flatten() {
            yield Ok(sse_event("pose", &pose));
        }
        loop {
            tokio::select! {
                result = shutdown.changed() => {
                    if result.is_err() || *shutdown.borrow() {
                        break;
                    }
                }
                event = receiver.recv() => match event {
                    Ok(ServerEvent::Status(status)) => {
                        yield Ok(sse_event("status", &json!({ "status": status })));
                    }
                    Ok(ServerEvent::Pose(pose)) => yield Ok(sse_event("pose", pose.as_ref())),
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
        }
    };
    Sse::new(output).keep_alive(KeepAlive::default())
}

fn sse_event(name: &'static str, value: &impl Serialize) -> Event {
    Event::default()
        .event(name)
        .data(serde_json::to_string(value).expect("serializable event"))
}

fn spawn_usb_reader(state: AppState, gyro_calibration_file: PathBuf) {
    thread::Builder::new()
        .name("nolo-usb-reader".to_owned())
        .spawn(move || usb_reader_loop(state, gyro_calibration_file))
        .expect("failed to start USB reader thread");
}

async fn servo_ipc_accept_loop(listener: UnixListener, state: AppState, backend: ArmOutputBackend) {
    let mut shutdown = state.shutdown.subscribe();
    loop {
        tokio::select! {
            result = shutdown.changed() => {
                if result.is_err() || *shutdown.borrow() {
                    return;
                }
            }
            result = listener.accept() => match result {
                Ok((stream, _)) => {
                    state.reset_arm_home_session(backend);
                    if let Err(error) = servo_ipc_connection(stream, state.clone(), backend).await {
                        eprintln!("MoveIt Servo {backend:?} IPC disconnected: {error:#}");
                    }
                    state.reset_arm_home_session(backend);
                    let controller = MoveItController::new_for_backend(
                        ArmModel::embedded().expect("invalid embedded Star Arm 102-FL profile"),
                        backend.simulation_only(),
                        backend.controller_backend(),
                    );
                    state.set_arm_snapshot(backend, controller.latest().clone());
                }
                Err(error) => {
                    eprintln!("MoveIt Servo IPC accept failed: {error}");
                    return;
                }
            },
        }
    }
}

async fn servo_ipc_connection(
    stream: UnixStream,
    state: AppState,
    backend: ArmOutputBackend,
) -> Result<()> {
    let model = ArmModel::embedded()
        .map_err(anyhow::Error::msg)
        .context("invalid embedded Star Arm 102-FL profile")?;
    let mut controller = MoveItController::new_for_backend(
        model,
        backend.simulation_only(),
        backend.controller_backend(),
    );
    let teleop = state.teleop_intent.subscribe();
    let mut shutdown = state.shutdown.subscribe();
    let (reader, mut writer) = stream.into_split();
    let mut lines = BufReader::new(reader).lines();
    let mut ticker = tokio::time::interval(ARM_SIM_INTERVAL);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let start = Instant::now();
    let mut latest_feedback: Option<(ServoFeedbackFrame, Instant)> = None;
    let mut output_generation = state.arm_output_generation.load(Ordering::Acquire);

    loop {
        tokio::select! {
            result = shutdown.changed() => {
                if result.is_err() || *shutdown.borrow() {
                    return Ok(());
                }
            }
            line = lines.next_line() => {
                let Some(line) = line.context("failed reading Servo feedback")? else {
                    bail!("peer closed the socket");
                };
                let feedback: ServoFeedbackFrame = serde_json::from_str(&line)
                    .context("invalid Servo feedback frame")?;
                if let Some(home_status) = feedback.home_status.clone() {
                    let current = state.arm_home_status(backend);
                    if home_status.request_id >= current.request_id {
                        state.acknowledge_arm_home_request(
                            backend,
                            home_status.request_id,
                            home_status.acknowledged_action,
                        );
                        state.set_arm_home_status(backend, home_status);
                    }
                }
                latest_feedback = Some((feedback, Instant::now()));
            }
            _ = ticker.tick() => {
                let now = Instant::now();
                let generation = state.arm_output_generation.load(Ordering::Acquire);
                if generation != output_generation {
                    output_generation = generation;
                    controller.require_reanchor();
                }
                let mut intent = relative_intent(&teleop.borrow());
                if state.arm_output_backend() != backend || state.arm_home_blocks_teleop(backend) {
                    intent.state = RelativeIntentState::Idle;
                }
                let feedback_age_ms = latest_feedback.as_ref().map(|(_, received)| {
                    duration_ms(now.saturating_duration_since(*received))
                });
                let (mut command, snapshot) = controller.step(
                    elapsed_ns(start),
                    intent,
                    latest_feedback.as_ref().map(|(feedback, _)| feedback),
                    feedback_age_ms,
                );
                command.home_request = state.arm_home_request(backend);
                state.set_arm_snapshot(backend, snapshot);
                let mut encoded = serde_json::to_vec(&command)
                    .context("failed serializing Servo command")?;
                encoded.push(b'\n');
                writer.write_all(&encoded).await.context("failed writing Servo command")?;
            }
        }
    }
}

fn bind_servo_ipc(path: &FilePath) -> Result<UnixListener> {
    use std::os::unix::fs::FileTypeExt;

    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).with_context(|| {
            format!("failed creating Servo IPC directory: {}", parent.display())
        })?;
    }
    if let Ok(metadata) = std::fs::symlink_metadata(path) {
        if !metadata.file_type().is_socket() {
            bail!(
                "refusing to replace non-socket Servo IPC path: {}",
                path.display()
            );
        }
        bail!(
            "refusing to replace existing Servo IPC socket: {}; stop the owning server, or verify it is stale and remove it explicitly",
            path.display()
        );
    }
    UnixListener::bind(path)
        .with_context(|| format!("failed binding Servo IPC socket: {}", path.display()))
}

fn remove_servo_socket(path: &FilePath) -> Result<()> {
    match std::fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error)
            .with_context(|| format!("failed removing Servo IPC socket: {}", path.display())),
    }
}

fn relative_intent(intent: &TeleopIntent) -> RelativeIntent {
    RelativeIntent {
        state: match intent.state {
            nolo_usb_server::teleop::TeleopIntentState::Idle => RelativeIntentState::Idle,
            nolo_usb_server::teleop::TeleopIntentState::Active => RelativeIntentState::Active,
            nolo_usb_server::teleop::TeleopIntentState::Faulted => RelativeIntentState::Faulted,
        },
        relative_position_m: intent
            .relative_position
            .map(|position| position.map(f64::from)),
        relative_orientation_xyzw: intent
            .relative_orientation
            .map(|orientation| orientation.map(f64::from)),
        gripper_closed: intent.gripper_closed,
    }
}

struct TrackingSession {
    controllers: [ControllerState; 2],
    hmd: HmdState,
    teleop: TeleopIntentMachine,
    gyro_bias_store: GyroBiasStore,
    gyro_bias_persisted_or_attempted: [bool; SOURCE_COUNT],
    gyro_calibration_file: Option<PathBuf>,
    simulated: bool,
}

impl TrackingSession {
    fn physical(state: &AppState, gyro_calibration_file: PathBuf) -> Self {
        let gyro_bias_store = match GyroBiasStore::load(&gyro_calibration_file) {
            Ok(store) => store,
            Err(error) => {
                eprintln!("陀螺仪零偏文件加载失败，将重新标定：{error:#}");
                GyroBiasStore::default()
            }
        };
        Self::new(state, gyro_bias_store, Some(gyro_calibration_file), false)
    }

    fn virtual_usb(state: &AppState) -> Self {
        // Never mix physical-device biases or persistence with deterministic
        // virtual IMU samples.  The six-second virtual Menu hold exercises the
        // ordinary calibration path from a clean Fusion state.
        Self::new(state, GyroBiasStore::default(), None, true)
    }

    fn new(
        state: &AppState,
        gyro_bias_store: GyroBiasStore,
        gyro_calibration_file: Option<PathBuf>,
        simulated: bool,
    ) -> Self {
        let controllers =
            std::array::from_fn(|source_id| ControllerState::new(gyro_bias_store.bias(source_id)));
        let hmd = HmdState::new(gyro_bias_store.bias(2));
        let gyro_bias_persisted_or_attempted =
            std::array::from_fn(|source_id| gyro_bias_store.bias(source_id).is_some());
        let teleop = TeleopIntentMachine::new();
        state.set_teleop_intent(teleop.latest().clone());
        Self {
            controllers,
            hmd,
            teleop,
            gyro_bias_store,
            gyro_bias_persisted_or_attempted,
            gyro_calibration_file,
            simulated,
        }
    }

    fn process_raw(
        &mut self,
        state: &AppState,
        raw: nolo_usb_server::protocol::RawFrame,
        now: Instant,
        time_ns: u64,
    ) {
        let hmd_sample =
            self.hmd
                .samples
                .observe(usize::from(raw.controller_id), raw.hmd_sequence, now);
        if state.take_pose_calibration_request(2) {
            self.hmd.fusion.start_pose_calibration();
        } else if state.take_gyro_calibration_request(2) {
            self.hmd.fusion.start_gyro_calibration();
            self.gyro_bias_persisted_or_attempted[2] = false;
        }
        if hmd_sample.changed {
            self.hmd.orientation = self.hmd.fusion.update(raw, now);
            self.persist_bias(2, self.hmd.fusion.completed_gyro_bias());
        }
        if hmd_sample.changed {
            state.set_pose(hmd_pose(
                raw,
                self.hmd.orientation,
                self.hmd.fusion.diagnostics(),
                hmd_sample,
                time_ns,
                self.simulated,
            ));
        }

        let controller_id = usize::from(raw.controller_id);
        let mut completed_bias = None;
        let controller_sample = {
            let controller = &mut self.controllers[controller_id];
            let controller_sample = controller.samples.observe(raw.controller_sequence, now);
            if state.take_pose_calibration_request(controller_id) {
                controller.reset_position_filter();
                controller.fusion.start_pose_calibration();
            } else if state.take_gyro_calibration_request(controller_id) {
                controller.fusion.start_gyro_calibration();
                self.gyro_bias_persisted_or_attempted[controller_id] = false;
            }
            if controller_sample.changed {
                controller.orientation = controller.fusion.update(raw, now);
                completed_bias = controller.fusion.completed_gyro_bias();
                controller.filtered_position = controller
                    .position_filter
                    .update(raw.position, time_ns as f64 * 1.0e-9);
            }
            controller_sample
        };
        if controller_sample.changed {
            self.persist_bias(controller_id, completed_bias);
        }
        if controller_sample.changed {
            let controller = &self.controllers[controller_id];
            let pose = controller_pose(
                raw,
                controller,
                controller_sample,
                hmd_sample,
                time_ns,
                self.simulated,
            );
            if controller_id == 0 {
                state.set_teleop_intent(self.teleop.update(teleop_sample(&pose)));
            }
            state.set_pose(pose);
        }
        if !self.simulated {
            state.set_status("原始 USB 数据已连接");
        }
    }

    fn persist_bias(&mut self, source_id: usize, bias: Option<[f32; 3]>) {
        let Some(path) = self.gyro_calibration_file.as_deref() else {
            return;
        };
        persist_completed_gyro_bias(
            source_id,
            bias,
            &mut self.gyro_bias_store,
            &mut self.gyro_bias_persisted_or_attempted,
            path,
        );
    }

    fn force_fault(&mut self, state: &AppState, time_ns: u64, reason: TeleopStopReason) {
        state.set_teleop_intent(self.teleop.force_fault(time_ns, reason));
    }

    fn stop(&mut self, state: &AppState, time_ns: u64) {
        state.set_teleop_intent(self.teleop.stop(time_ns));
    }
}

fn usb_reader_loop(state: AppState, gyro_calibration_file: PathBuf) {
    let process_start = Instant::now();
    loop {
        if state.simulation_requested() {
            virtual_usb_loop(&state, process_start);
            continue;
        }
        state.set_simulation_active(false);
        state.set_status("正在连接 NOLO USB…");
        let api = match HidApi::new() {
            Ok(api) => api,
            Err(error) => {
                state.set_status(format!("初始化 HID 失败：{error}"));
                wait_for_simulation_request(&state, Duration::from_secs(2));
                continue;
            }
        };
        let opened = SUPPORTED_DEVICES
            .into_iter()
            .find_map(|(vid, pid)| api.open(vid, pid).ok().map(|device| (device, vid, pid)));
        let (device, vid, pid) = match opened {
            Some(opened) => opened,
            None => {
                state.mark_all_offline(elapsed_ns(process_start));
                state.set_status(
                    "未能打开 NOLO USB (0483:5750 或 28e9:028a)；请检查连接、权限以及是否有其他程序独占设备",
                );
                wait_for_simulation_request(&state, Duration::from_secs(2));
                continue;
            }
        };

        state.set_status(format!(
            "NOLO USB ({vid:04x}:{pid:04x}) 已连接，等待位姿数据"
        ));
        let mut session = TrackingSession::physical(&state, gyro_calibration_file.clone());
        let mut report = [0_u8; REPORT_SIZE + 1];
        loop {
            if state.simulation_requested() {
                let time_ns = elapsed_ns(process_start);
                session.force_fault(&state, time_ns, TeleopStopReason::UsbDisconnected);
                break;
            }
            let size = match device.read_timeout(&mut report, 500) {
                Ok(0) => continue,
                Ok(size) => size,
                Err(error) => {
                    let time_ns = elapsed_ns(process_start);
                    session.force_fault(&state, time_ns, TeleopStopReason::UsbDisconnected);
                    state.mark_all_offline(time_ns);
                    state.set_status(format!("NOLO USB 读取中断：{error}；2 秒后重连"));
                    break;
                }
            };
            let raw_report: &[u8] = match size {
                REPORT_SIZE => &report[..REPORT_SIZE],
                size if size == REPORT_SIZE + 1 && report[0] == 0 => &report[1..],
                other => {
                    let time_ns = elapsed_ns(process_start);
                    session.force_fault(&state, time_ns, TeleopStopReason::InvalidUsbReport);
                    state.set_status(format!("收到长度异常的 HID 报告：{other} 字节"));
                    continue;
                }
            };
            let raw = match decode_report(raw_report) {
                Ok(Some(frame)) => frame,
                // Other HID report types are not controller pose samples. They
                // neither refresh nor invalidate Controller 0; its own sequence
                // freshness remains available as a diagnostic signal.
                Ok(None) => continue,
                Err(error) => {
                    let time_ns = elapsed_ns(process_start);
                    session.force_fault(&state, time_ns, TeleopStopReason::InvalidUsbReport);
                    state.set_status(format!("NOLO 报告解析失败：{error}"));
                    continue;
                }
            };

            session.process_raw(&state, raw, Instant::now(), elapsed_ns(process_start));
        }
        if !state.simulation_requested() {
            wait_for_simulation_request(&state, Duration::from_secs(2));
        }
    }
}

fn virtual_usb_loop(state: &AppState, process_start: Instant) {
    state.clear_calibration_requests();
    state.set_simulation_active(true);
    state.set_status("NOLO 虚拟 USB：自动标定（保持静止）");
    let mut session = TrackingSession::virtual_usb(state);
    let mut simulator = VirtualNolo::new();
    let period = Duration::from_secs_f64(REPORT_PERIOD_SECONDS);
    let started = Instant::now();
    let mut report_index = 0_u64;
    let mut previous_phase = "";
    let mut failed = false;
    while state.simulation_requested() {
        let deadline = started + period.mul_f64(report_index as f64);
        thread::sleep(deadline.saturating_duration_since(Instant::now()));
        let sample = simulator.next_sample();
        let report = match nolo_usb_server::protocol::encode_report(sample.frame) {
            Ok(report) => report,
            Err(error) => {
                state.set_status(format!("NOLO 虚拟报告编码失败：{error:#}"));
                failed = true;
                state.request_simulation(false);
                break;
            }
        };
        let raw = match decode_report(&report) {
            Ok(Some(raw)) => raw,
            Ok(None) => unreachable!("virtual generator emitted an unsupported report type"),
            Err(error) => {
                state.set_status(format!("NOLO 虚拟报告解码失败：{error:#}"));
                failed = true;
                state.request_simulation(false);
                break;
            }
        };
        session.process_raw(state, raw, Instant::now(), elapsed_ns(process_start));
        if sample.frame.controller_id == 0 && sample.phase != previous_phase {
            previous_phase = sample.phase;
            state.set_status(format!("NOLO 虚拟 USB：{}", sample.phase));
        }
        report_index = report_index.saturating_add(1);
    }
    let time_ns = elapsed_ns(process_start);
    if failed {
        session.force_fault(state, time_ns, TeleopStopReason::InvalidUsbReport);
    } else {
        session.stop(state, time_ns);
    }
    state.mark_all_offline(time_ns);
    state.clear_calibration_requests();
    state.set_simulation_active(false);
}

fn wait_for_simulation_request(state: &AppState, duration: Duration) {
    let deadline = Instant::now() + duration;
    while !state.simulation_requested() && Instant::now() < deadline {
        thread::sleep(Duration::from_millis(50));
    }
}

struct ControllerState {
    fusion: ControllerFusion,
    samples: SampleTracker,
    orientation: [f32; 4],
    position_filter: PositionFilter,
    filtered_position: Option<[f32; 3]>,
}

impl ControllerState {
    fn new(saved_gyro_bias: Option<[f32; 3]>) -> Self {
        let mut fusion = ControllerFusion::new();
        if let Some(bias) = saved_gyro_bias {
            let loaded = fusion.load_gyro_bias(bias);
            debug_assert!(loaded);
        }
        Self {
            fusion,
            samples: SampleTracker::new(120.0),
            orientation: [0.0, 0.0, 0.0, 1.0],
            position_filter: PositionFilter::default(),
            filtered_position: None,
        }
    }

    fn reset_position_filter(&mut self) {
        self.position_filter.reset();
        self.filtered_position = None;
    }
}

struct HmdState {
    fusion: HmdFusion,
    samples: InterleavedSampleTracker<2>,
    orientation: [f32; 4],
}

impl HmdState {
    fn new(saved_gyro_bias: Option<[f32; 3]>) -> Self {
        let mut fusion = HmdFusion::new();
        if let Some(bias) = saved_gyro_bias {
            let loaded = fusion.load_gyro_bias(bias);
            debug_assert!(loaded);
        }
        Self {
            fusion,
            samples: InterleavedSampleTracker::new(120.0),
            orientation: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

fn persist_completed_gyro_bias(
    source_id: usize,
    bias: Option<[f32; 3]>,
    store: &mut GyroBiasStore,
    persisted_or_attempted: &mut [bool; SOURCE_COUNT],
    path: &FilePath,
) {
    if persisted_or_attempted[source_id] {
        return;
    }
    let Some(bias) = bias else {
        return;
    };
    persisted_or_attempted[source_id] = true;
    if let Err(error) = store
        .set_bias(source_id, bias)
        .and_then(|()| store.save(path))
    {
        eprintln!(
            "设备 {source_id} 的陀螺仪零偏写入 {} 失败：{error:#}",
            path.display()
        );
    } else {
        eprintln!("设备 {source_id} 的陀螺仪零偏已保存到 {}", path.display());
    }
}

fn hmd_pose(
    raw: nolo_usb_server::protocol::RawFrame,
    orientation: [f32; 4],
    fusion: FusionDiagnostics,
    sample: SampleObservation,
    time_ns: u64,
    simulated: bool,
) -> PoseFrame {
    PoseFrame {
        time_ns,
        source_id: 2,
        sample_rate_hz: 240,
        device: if simulated {
            VIRTUAL_DEVICE_NAMES[2]
        } else {
            DEVICE_NAMES[2]
        },
        simulated,
        position: raw.hmd_position,
        filtered_position: None,
        orientation,
        communication_fresh: sample.fresh,
        sample_changed: sample.changed,
        unchanged_ms: duration_ms(sample.unchanged),
        hmd_unchanged_ms: duration_ms(sample.unchanged),
        hmd_relay_online: sample.fresh,
        sample_sequence: raw.hmd_sequence,
        hmd_sequence: raw.hmd_sequence,
        samples_received: sample.samples_received,
        samples_missed: sample.samples_missed,
        duplicate_reports: sample.duplicate_reports,
        measured_rate_hz: sample.measured_rate_hz,
        sample_jitter_ms: sample.jitter_ms,
        acceleration_error_degrees: fusion.acceleration_error_degrees,
        accelerometer_ignored: fusion.accelerometer_ignored,
        acceleration_recovery: fusion.acceleration_recovery,
        fusion_initialising: fusion.initialising,
        gyro_bias_dps: fusion.gyro_bias_dps,
        gyro_calibration_active: fusion.gyro_calibration_active,
        gyro_calibration_complete: fusion.gyro_calibration_complete,
        gyro_calibration_progress: fusion.gyro_calibration_progress,
        menu_pressed: false,
        trigger_pressed: false,
        squeeze_pressed: false,
    }
}

fn controller_pose(
    raw: nolo_usb_server::protocol::RawFrame,
    controller: &ControllerState,
    sample: SampleObservation,
    hmd_sample: SampleObservation,
    time_ns: u64,
    simulated: bool,
) -> PoseFrame {
    let orientation = controller.orientation;
    let fusion = controller.fusion.diagnostics();
    let online = sample.fresh;
    PoseFrame {
        time_ns,
        source_id: raw.controller_id,
        sample_rate_hz: 120,
        device: if simulated {
            VIRTUAL_DEVICE_NAMES[usize::from(raw.controller_id)]
        } else {
            DEVICE_NAMES[usize::from(raw.controller_id)]
        },
        simulated,
        position: raw.position,
        filtered_position: controller.filtered_position,
        orientation,
        communication_fresh: online,
        sample_changed: sample.changed,
        unchanged_ms: duration_ms(sample.unchanged),
        hmd_unchanged_ms: duration_ms(hmd_sample.unchanged),
        hmd_relay_online: hmd_sample.fresh,
        sample_sequence: raw.controller_sequence,
        hmd_sequence: raw.hmd_sequence,
        samples_received: sample.samples_received,
        samples_missed: sample.samples_missed,
        duplicate_reports: sample.duplicate_reports,
        measured_rate_hz: sample.measured_rate_hz,
        sample_jitter_ms: sample.jitter_ms,
        acceleration_error_degrees: fusion.acceleration_error_degrees,
        accelerometer_ignored: fusion.accelerometer_ignored,
        acceleration_recovery: fusion.acceleration_recovery,
        fusion_initialising: fusion.initialising,
        gyro_bias_dps: fusion.gyro_bias_dps,
        gyro_calibration_active: fusion.gyro_calibration_active,
        gyro_calibration_complete: fusion.gyro_calibration_complete,
        gyro_calibration_progress: fusion.gyro_calibration_progress,
        menu_pressed: online && raw.menu_pressed(),
        trigger_pressed: raw.trigger_pressed(),
        squeeze_pressed: raw.squeeze_pressed(),
    }
}

fn teleop_sample(frame: &PoseFrame) -> TeleopSample {
    debug_assert_eq!(frame.source_id, 0);
    TeleopSample {
        receive_time_ns: frame.time_ns,
        sample_sequence: frame.sample_sequence,
        raw_position: frame.position,
        filtered_position: frame.filtered_position,
        orientation: frame.orientation,
        communication_fresh: frame.communication_fresh,
        fusion_initialising: frame.fusion_initialising,
        gyro_calibration_active: frame.gyro_calibration_active,
        trigger_pressed: frame.trigger_pressed,
        squeeze_pressed: frame.squeeze_pressed,
    }
}

fn elapsed_ns(start: Instant) -> u64 {
    start.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64
}

fn duration_ms(duration: Duration) -> u64 {
    duration.as_millis().min(u128::from(u64::MAX)) as u64
}

async fn shutdown_signal(state: AppState) {
    let ctrl_c = async {
        tokio::signal::ctrl_c()
            .await
            .expect("failed to install Ctrl-C handler");
    };
    #[cfg(unix)]
    let terminate = async {
        tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())
            .expect("failed to install SIGTERM handler")
            .recv()
            .await;
    };
    #[cfg(not(unix))]
    let terminate = std::future::pending::<()>();
    tokio::select! {
        _ = ctrl_c => {},
        _ = terminate => {},
    }
    state.notify_shutdown();
}

struct Config {
    host: IpAddr,
    port: u16,
    static_dir: PathBuf,
    gyro_calibration_file: PathBuf,
    servo_ipc_path: PathBuf,
    hardware_servo_ipc_path: PathBuf,
}

impl Config {
    fn parse() -> Result<Self> {
        let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let vr_xr_root = manifest_dir
            .parent()
            .and_then(FilePath::parent)
            .expect("nolo-usb-server manifest must be under vr-xr/src")
            .to_path_buf();
        let default_static = vr_xr_root.join("src/controller-viewer/public");
        let default_gyro_calibration_file = vr_xr_root.join("state/gyro-bias-v1.json");
        let default_servo_ipc_path = vr_xr_root.join("state/moveit-servo.sock");
        let default_hardware_servo_ipc_path = vr_xr_root.join("state/moveit-servo-hardware.sock");
        let mut host = "127.0.0.1".parse().unwrap();
        let mut port = 8765;
        let mut static_dir = default_static;
        let mut gyro_calibration_file = default_gyro_calibration_file;
        let mut servo_ipc_path = default_servo_ipc_path;
        let mut hardware_servo_ipc_path = default_hardware_servo_ipc_path;
        for argument in std::env::args().skip(1) {
            if let Some(value) = argument.strip_prefix("--host=") {
                host = value
                    .parse()
                    .with_context(|| format!("invalid host: {value}"))?;
            } else if let Some(value) = argument.strip_prefix("--port=") {
                port = value
                    .parse()
                    .with_context(|| format!("invalid port: {value}"))?;
            } else if let Some(value) = argument.strip_prefix("--static-dir=") {
                static_dir = PathBuf::from(value);
            } else if let Some(value) = argument.strip_prefix("--gyro-calibration-file=") {
                gyro_calibration_file = PathBuf::from(value);
            } else if let Some(value) = argument.strip_prefix("--servo-ipc=") {
                servo_ipc_path = PathBuf::from(value);
            } else if let Some(value) = argument.strip_prefix("--hardware-servo-ipc=") {
                hardware_servo_ipc_path = PathBuf::from(value);
            } else if matches!(argument.as_str(), "-h" | "--help") {
                println!(
                    "Usage: nolo-usb-server [--host=IP] [--port=PORT] [--static-dir=PATH] [--gyro-calibration-file=PATH] [--servo-ipc=PATH] [--hardware-servo-ipc=PATH]"
                );
                std::process::exit(0);
            } else {
                bail!("unknown argument: {argument}");
            }
        }
        if !static_dir.join("index.html").is_file() {
            bail!(
                "static directory has no index.html: {}",
                static_dir.display()
            );
        }
        if servo_ipc_path == hardware_servo_ipc_path {
            bail!("simulation and hardware Servo IPC paths must differ");
        }
        Ok(Self {
            host,
            port,
            static_dir,
            gyro_calibration_file,
            servo_ipc_path,
            hardware_servo_ipc_path,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_payload_exposes_initial_idle_teleop_intent() {
        let payload = AppState::new().payload();
        assert_eq!(
            payload.latest_teleop_intent.state,
            nolo_usb_server::teleop::TeleopIntentState::Idle
        );
        assert_eq!(
            payload.latest_teleop_intent.stop_reason,
            Some(TeleopStopReason::NoSample)
        );

        let json = serde_json::to_value(payload).unwrap();
        assert_eq!(json["latestTeleopIntent"]["state"], "idle");
        assert_eq!(json["latestTeleopIntent"]["stop_reason"], "no_sample");
        assert_eq!(json["latestTeleopIntent"]["squeeze_pressed"], false);
        assert_eq!(
            json["latestTeleopIntent"]["gripper_closed"],
            serde_json::Value::Null
        );
        assert_eq!(json["latestArmSimulation"]["state"], "faulted");
        assert_eq!(
            json["latestArmSimulation"]["stop_reason"],
            "servo_unavailable"
        );
        assert_eq!(
            json["latestArmSimulation"]["backend"],
            "moveit_servo_simulation"
        );
        assert_eq!(json["latestArmSimulation"]["simulation_only"], true);
        assert_eq!(
            json["latestArmHardware"]["backend"],
            "moveit_servo_hardware"
        );
        assert_eq!(json["latestArmHardware"]["simulation_only"], false);
        assert_eq!(json["armOutputBackend"], "simulation");
        assert_eq!(json["armHomeSimulation"]["state"], "idle");
        assert_eq!(json["armHomeHardware"]["state"], "idle");
        assert_eq!(json["simulationRequested"], false);
        assert_eq!(json["simulationActive"], false);
        assert_eq!(json["latestArmSimulation"]["gripper_closed"], true);
        assert_eq!(
            json["latestArmSimulation"]["tcp_pose"],
            serde_json::Value::Null
        );
        assert!(json["latestArmSimulation"]["gripper_rad"].is_number());
        let model_joints = json["latestArmSimulation"]["model_joints_rad"]
            .as_array()
            .unwrap();
        for (actual, expected) in model_joints.iter().zip([0.0; 6]) {
            assert!((actual.as_f64().unwrap() - expected).abs() < 1.0e-9);
        }
        assert_eq!(json["latestArmSimulation"]["model_id"], "stararm102-fl-v1");
        assert!(json.get("latest_teleop_intent").is_none());
        assert!(json.get("latest_arm_simulation").is_none());
        assert!(json.get("simulation_requested").is_none());
        assert!(json.get("simulation_active").is_none());
    }

    #[test]
    fn virtual_usb_request_is_visible_without_claiming_it_is_active() {
        let state = AppState::new();
        state.request_simulation(true);
        let payload = state.payload();
        assert!(payload.simulation_requested);
        assert!(!payload.simulation_active);
        state.set_simulation_active(true);
        assert!(state.payload().simulation_active);
        state.request_simulation(false);
        assert!(!state.payload().simulation_requested);
    }

    #[tokio::test]
    async fn stop_simulation_only_stops_virtual_input() {
        let state = AppState::new();
        state.request_simulation(true);
        let (status, Json(response)) = stop_simulation(State(state.clone())).await;
        assert_eq!(status, StatusCode::ACCEPTED);
        assert!(!state.simulation_requested());
        assert_eq!(response["simulation_requested"], false);
        assert_eq!(response["status"], "stopped");
        assert!(response.get("home_request_id").is_none());
        assert_eq!(
            state.arm_home_status(ArmOutputBackend::Simulation).state,
            HomeState::Idle
        );
    }

    #[tokio::test]
    async fn arm_output_defaults_to_simulation_and_switches_explicitly() {
        let state = AppState::new();
        assert_eq!(state.arm_output_backend(), ArmOutputBackend::Simulation);
        let generation = state.arm_output_generation.load(Ordering::Acquire);

        let (status, Json(response)) =
            select_arm_output(Path("hardware".to_owned()), State(state.clone())).await;
        assert_eq!(status, StatusCode::OK);
        assert_eq!(response["arm_output_backend"], "hardware");
        assert_eq!(state.arm_output_backend(), ArmOutputBackend::Hardware);
        assert_eq!(
            state.arm_output_generation.load(Ordering::Acquire),
            generation + 1
        );

        let (status, _) = select_arm_output(Path("invalid".to_owned()), State(state.clone())).await;
        assert_eq!(status, StatusCode::BAD_REQUEST);
        assert_eq!(state.arm_output_backend(), ArmOutputBackend::Hardware);
    }

    #[tokio::test]
    async fn home_plan_is_one_step_for_simulation_and_hardware() {
        for (backend_name, backend) in [
            ("simulation", ArmOutputBackend::Simulation),
            ("hardware", ArmOutputBackend::Hardware),
        ] {
            let state = AppState::new();
            state.select_arm_output(backend);
            let (status, Json(response)) = arm_home(
                Path((backend_name.to_owned(), "plan".to_owned())),
                State(state.clone()),
            )
            .await;
            assert_eq!(status, StatusCode::ACCEPTED);
            assert_eq!(response["auto_execute"], true);
            assert_eq!(state.arm_home_status(backend).state, HomeState::Planning);
            assert_eq!(
                state.arm_home_request(backend).unwrap().action,
                HomeRequestAction::Plan
            );
        }
    }

    #[test]
    fn reconnect_clears_only_the_selected_backend_home_session() {
        let state = AppState::new();
        let simulation_status = HomeStatusFrame {
            request_id: 7,
            state: HomeState::Succeeded,
            message: "simulation succeeded".to_owned(),
            ..HomeStatusFrame::default()
        };
        let hardware_status = HomeStatusFrame {
            request_id: 8,
            state: HomeState::Executing,
            message: "hardware executing".to_owned(),
            ..HomeStatusFrame::default()
        };
        state.set_arm_home_status(ArmOutputBackend::Simulation, simulation_status.clone());
        state.set_arm_home_status(ArmOutputBackend::Hardware, hardware_status);
        state.send_arm_home_request(
            ArmOutputBackend::Hardware,
            HomeRequestFrame {
                request_id: 8,
                action: HomeRequestAction::Plan,
            },
        );

        state.reset_arm_home_session(ArmOutputBackend::Hardware);

        assert_eq!(
            state.arm_home_status(ArmOutputBackend::Simulation),
            simulation_status
        );
        assert_eq!(
            state.arm_home_status(ArmOutputBackend::Hardware),
            HomeStatusFrame::default()
        );
        assert!(state.arm_home_request(ArmOutputBackend::Hardware).is_none());
    }

    #[tokio::test]
    async fn shutdown_notification_reaches_existing_subscribers() {
        let state = AppState::new();
        let mut shutdown = state.shutdown.subscribe();
        assert!(!*shutdown.borrow());

        state.notify_shutdown();
        shutdown.changed().await.unwrap();
        assert!(*shutdown.borrow());
    }

    #[tokio::test]
    async fn servo_ipc_bind_never_replaces_an_existing_socket() {
        let path = std::env::temp_dir().join(format!(
            "nolo-usb-server-bind-test-{}-{}.sock",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        let listener = match bind_servo_ipc(&path) {
            Ok(listener) => listener,
            Err(error)
                if error
                    .root_cause()
                    .downcast_ref::<std::io::Error>()
                    .is_some_and(|error| error.kind() == std::io::ErrorKind::PermissionDenied) =>
            {
                // Some CI/sandbox seccomp profiles prohibit AF_UNIX bind.
                // The same test is exercised in the host-side verification.
                return;
            }
            Err(error) => panic!("first Servo IPC bind failed: {error:#}"),
        };
        let error = bind_servo_ipc(&path).unwrap_err();
        assert!(error.to_string().contains("refusing to replace existing"));
        drop(listener);
        remove_servo_socket(&path).unwrap();
    }
}
