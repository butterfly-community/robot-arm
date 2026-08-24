use anyhow::{Context, Result, bail};
use async_stream::stream;
use axum::{
    Json, Router,
    extract::{Path, State},
    http::{HeaderValue, Request, StatusCode},
    middleware::{self, Next},
    response::{
        Response,
        sse::{Event, KeepAlive, Sse},
    },
    routing::{get, post},
};
use hidapi::HidApi;
use nolo_usb_server::{
    fusion::{ControllerFusion, FusionDiagnostics, HmdFusion},
    gyro_bias::GyroBiasStore,
    pose::estimated_grip_position,
    position_filter::{PositionFilter, PositionFilterSettings},
    protocol::{REPORT_SIZE, SUPPORTED_DEVICES, decode_report},
    sample::{InterleavedSampleTracker, SampleObservation, SampleTracker},
    teleop::{TeleopIntent, TeleopIntentMachine, TeleopSample, TeleopStopReason},
};
use serde::Serialize;
use serde_json::json;
use stararm102_control::{
    ArmModel, MoveItSimulationController, RelativeIntent, RelativeIntentState, ServoFeedbackFrame,
    SimulationSnapshot,
};
use std::{
    convert::Infallible,
    net::{IpAddr, SocketAddr},
    path::{Path as FilePath, PathBuf},
    sync::{
        Arc, Mutex, RwLock,
        atomic::{AtomicBool, Ordering},
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

const CONTENT_SECURITY_POLICY: &str = "default-src 'self'; script-src 'self' https://cdn.jsdelivr.net; style-src 'self'; connect-src 'self'; frame-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'self'";

const DEVICE_NAMES: [&str; 3] = [
    "NOLO CV1: Controller 0 (USB)",
    "NOLO CV1: Controller 1 (USB)",
    "NOLO CV1: Head Marker (USB)",
];
const SEQUENCE_STALE_AFTER: Duration = Duration::from_millis(200);
const SSE_MIN_INTERVAL: Duration = Duration::from_micros(16_667);
const ARM_SIM_INTERVAL: Duration = Duration::from_millis(10);
const SOURCE_COUNT: usize = 3;

#[derive(Clone, Debug, Serialize)]
struct PoseFrame {
    time_ns: u64,
    source_id: u8,
    source_kind: &'static str,
    sample_rate_hz: u16,
    controller_id: Option<u8>,
    device: &'static str,
    /// Deprecated compatibility field. This is not an OpenXR tracking flag.
    flags: u32,
    position: [f32; 3],
    filtered_position: Option<[f32; 3]>,
    marker_position: [f32; 3],
    grip_position: Option<[f32; 3]>,
    position_mode: &'static str,
    orientation: [f32; 4],
    communication_fresh: bool,
    sample_changed: bool,
    optical_tracking_valid: Option<bool>,
    pose_usable: bool,
    usb_silent_ms: u64,
    unchanged_ms: u64,
    hmd_unchanged_ms: u64,
    source_online: bool,
    hmd_relay_online: bool,
    sample_sequence: u8,
    controller_sequence: Option<u8>,
    hmd_sequence: u8,
    sequence_delta: u8,
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
    menu_active: bool,
    menu_pressed: bool,
    buttons_raw: u8,
    touchpad_pressed: bool,
    trigger_pressed: bool,
    home_pressed: bool,
    squeeze_pressed: bool,
    touchpad_touched: bool,
    touchpad: Option<[u8; 2]>,
    accelerometer_raw: [i16; 3],
    gyroscope_raw: [i16; 3],
}

#[derive(Clone, Debug, Serialize)]
struct StatusPayload {
    status: String,
    #[serde(rename = "latestFrame")]
    latest_frame: Option<PoseFrame>,
    #[serde(rename = "latestFrames")]
    latest_frames: [Option<PoseFrame>; SOURCE_COUNT],
    #[serde(rename = "latestTeleopIntent")]
    latest_teleop_intent: TeleopIntent,
    #[serde(rename = "latestArmSimulation")]
    latest_arm_simulation: SimulationSnapshot,
}

#[derive(Clone, Debug)]
enum ServerEvent {
    Status(String),
    Pose(Box<PoseFrame>),
}

#[derive(Debug)]
struct Snapshot {
    status: String,
    latest_frame: Option<PoseFrame>,
    latest_frames: [Option<PoseFrame>; SOURCE_COUNT],
}

#[derive(Clone)]
struct AppState {
    snapshot: Arc<RwLock<Snapshot>>,
    events: broadcast::Sender<ServerEvent>,
    calibration_requests: Arc<[AtomicBool; SOURCE_COUNT]>,
    pose_calibration_requests: Arc<[AtomicBool; SOURCE_COUNT]>,
    last_sse_pose: Arc<Mutex<[Option<Instant>; SOURCE_COUNT]>>,
    teleop_intent: watch::Sender<TeleopIntent>,
    arm_simulation: watch::Sender<SimulationSnapshot>,
    shutdown: watch::Sender<bool>,
}

impl AppState {
    fn new() -> Self {
        let (events, _) = broadcast::channel(256);
        let initial_teleop_intent = TeleopIntentMachine::new().latest().clone();
        let (teleop_intent, _) = watch::channel(initial_teleop_intent);
        let initial_arm_simulation = MoveItSimulationController::new(
            ArmModel::embedded().expect("invalid embedded FL profile"),
        )
        .latest()
        .clone();
        let (arm_simulation, _) = watch::channel(initial_arm_simulation);
        let (shutdown, _) = watch::channel(false);
        Self {
            snapshot: Arc::new(RwLock::new(Snapshot {
                status: "正在连接 NOLO USB…".to_owned(),
                latest_frame: None,
                latest_frames: std::array::from_fn(|_| None),
            })),
            events,
            calibration_requests: Arc::new(std::array::from_fn(|_| AtomicBool::new(false))),
            pose_calibration_requests: Arc::new(std::array::from_fn(|_| AtomicBool::new(false))),
            last_sse_pose: Arc::new(Mutex::new(std::array::from_fn(|_| None))),
            teleop_intent,
            arm_simulation,
            shutdown,
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
        let freshness_changed = snapshot.latest_frames[source_id]
            .as_ref()
            .map(|previous| previous.communication_fresh != pose.communication_fresh)
            .unwrap_or(true);
        snapshot.latest_frame = Some(pose.clone());
        snapshot.latest_frames[source_id] = Some(pose.clone());
        drop(snapshot);

        let now = Instant::now();
        let mut last_sse_pose = self.last_sse_pose.lock().unwrap();
        let interval_elapsed = last_sse_pose[source_id]
            .map(|previous| now.duration_since(previous) >= SSE_MIN_INTERVAL)
            .unwrap_or(true);
        if freshness_changed || !pose.pose_usable || interval_elapsed {
            last_sse_pose[source_id] = Some(now);
            drop(last_sse_pose);
            let _ = self.events.send(ServerEvent::Pose(Box::new(pose)));
        }
    }

    fn payload(&self) -> StatusPayload {
        let snapshot = self.snapshot.read().unwrap();
        StatusPayload {
            status: snapshot.status.clone(),
            latest_frame: snapshot.latest_frame.clone(),
            latest_frames: snapshot.latest_frames.clone(),
            latest_teleop_intent: self.teleop_intent.borrow().clone(),
            latest_arm_simulation: self.arm_simulation.borrow().clone(),
        }
    }

    fn set_teleop_intent(&self, intent: TeleopIntent) {
        self.teleop_intent.send_replace(intent);
    }

    fn set_arm_simulation(&self, snapshot: SimulationSnapshot) {
        self.arm_simulation.send_replace(snapshot);
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

    fn mark_all_offline(&self, time_ns: u64, usb_silent: Duration) {
        let usb_silent_ms = duration_ms(usb_silent);
        let changed_poses = {
            let mut snapshot = self.snapshot.write().unwrap();
            let mut changed = Vec::new();
            for frame in snapshot.latest_frames.iter_mut().flatten() {
                if !frame.communication_fresh && !frame.hmd_relay_online {
                    continue;
                }
                frame.time_ns = time_ns;
                frame.flags = 0;
                frame.communication_fresh = false;
                frame.sample_changed = false;
                frame.pose_usable = false;
                frame.filtered_position = None;
                frame.source_online = false;
                frame.hmd_relay_online = false;
                frame.menu_active = false;
                frame.menu_pressed = false;
                frame.usb_silent_ms = usb_silent_ms;
                frame.unchanged_ms = frame.unchanged_ms.max(usb_silent_ms);
                frame.hmd_unchanged_ms = frame.hmd_unchanged_ms.max(usb_silent_ms);
                changed.push(frame.clone());
            }
            if let Some(last) = changed.last() {
                snapshot.latest_frame = Some(last.clone());
            }
            changed
        };
        for pose in changed_poses {
            let _ = self.events.send(ServerEvent::Pose(Box::new(pose)));
        }
    }

    fn mark_source_offline(
        &self,
        source_id: usize,
        time_ns: u64,
        sample: SampleObservation,
        hmd_sample: SampleObservation,
    ) {
        let pose = {
            let mut snapshot = self.snapshot.write().unwrap();
            let Some(frame) = snapshot.latest_frames[source_id].as_mut() else {
                return;
            };
            if !frame.communication_fresh {
                return;
            }
            frame.time_ns = time_ns;
            frame.flags = 0;
            frame.communication_fresh = false;
            frame.sample_changed = false;
            frame.pose_usable = false;
            frame.filtered_position = None;
            frame.source_online = false;
            frame.menu_active = false;
            frame.menu_pressed = false;
            frame.unchanged_ms = duration_ms(sample.unchanged);
            frame.sequence_delta = 0;
            frame.samples_received = sample.samples_received;
            frame.samples_missed = sample.samples_missed;
            frame.duplicate_reports = sample.duplicate_reports;
            frame.measured_rate_hz = sample.measured_rate_hz;
            frame.sample_jitter_ms = sample.jitter_ms;
            frame.hmd_relay_online = hmd_sample.fresh;
            frame.hmd_unchanged_ms = duration_ms(hmd_sample.unchanged);
            let pose = frame.clone();
            snapshot.latest_frame = Some(pose.clone());
            pose
        };
        let _ = self.events.send(ServerEvent::Pose(Box::new(pose)));
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
    let state = AppState::new();
    spawn_usb_reader(
        state.clone(),
        config.controller_position_mode,
        config.position_filter_settings,
        config.gyro_calibration_file,
    );
    let servo_task = tokio::spawn(servo_ipc_accept_loop(servo_listener, state.clone()));

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
        .fallback_service(static_files)
        .layer(middleware::from_fn(security_headers))
        .with_state(state.clone());

    println!("NOLO direct USB viewer: http://{address}/");
    println!("Static files: {}", config.static_dir.display());
    let server_result = axum::serve(listener, app)
        .with_graceful_shutdown(shutdown_signal(state))
        .await
        .context("web server failed");
    let _ = servo_task.await;
    remove_servo_socket(&config.servo_ipc_path)?;
    server_result
}

async fn status(State(state): State<AppState>) -> Json<StatusPayload> {
    Json(state.payload())
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

async fn security_headers(request: Request<axum::body::Body>, next: Next) -> Response {
    let mut response = next.run(request).await;
    let headers = response.headers_mut();
    headers.insert(
        "x-content-type-options",
        HeaderValue::from_static("nosniff"),
    );
    headers.insert("referrer-policy", HeaderValue::from_static("no-referrer"));
    headers.insert(
        "content-security-policy",
        HeaderValue::from_static(CONTENT_SECURITY_POLICY),
    );
    response
}

fn spawn_usb_reader(
    state: AppState,
    controller_position_mode: ControllerPositionMode,
    position_filter_settings: PositionFilterSettings,
    gyro_calibration_file: PathBuf,
) {
    thread::Builder::new()
        .name("nolo-usb-reader".to_owned())
        .spawn(move || {
            usb_reader_loop(
                state,
                controller_position_mode,
                position_filter_settings,
                gyro_calibration_file,
            )
        })
        .expect("failed to start USB reader thread");
}

async fn servo_ipc_accept_loop(listener: UnixListener, state: AppState) {
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
                    if let Err(error) = servo_ipc_connection(stream, state.clone()).await {
                        eprintln!("MoveIt Servo IPC disconnected: {error:#}");
                    }
                    let controller = MoveItSimulationController::new(
                        ArmModel::embedded().expect("invalid embedded Star Arm 102-FL profile"),
                    );
                    state.set_arm_simulation(controller.latest().clone());
                }
                Err(error) => {
                    eprintln!("MoveIt Servo IPC accept failed: {error}");
                    return;
                }
            },
        }
    }
}

async fn servo_ipc_connection(stream: UnixStream, state: AppState) -> Result<()> {
    let model = ArmModel::embedded()
        .map_err(anyhow::Error::msg)
        .context("invalid embedded Star Arm 102-FL profile")?;
    let mut controller = MoveItSimulationController::new(model);
    let teleop = state.teleop_intent.subscribe();
    let mut shutdown = state.shutdown.subscribe();
    let (reader, mut writer) = stream.into_split();
    let mut lines = BufReader::new(reader).lines();
    let mut ticker = tokio::time::interval(ARM_SIM_INTERVAL);
    ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let start = Instant::now();
    let mut previous_tick = start;
    let mut latest_feedback: Option<(ServoFeedbackFrame, Instant)> = None;
    let mut last_feedback_sequence = None;

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
                accept_feedback_sequence(&mut last_feedback_sequence, feedback.sequence)?;
                latest_feedback = Some((feedback, Instant::now()));
            }
            _ = ticker.tick() => {
                let now = Instant::now();
                let dt = now.duration_since(previous_tick).as_secs_f64().clamp(0.000_001, 0.1);
                previous_tick = now;
                let intent = relative_intent(&teleop.borrow());
                let feedback_age_ms = latest_feedback.as_ref().map(|(_, received)| {
                    duration_ms(now.saturating_duration_since(*received))
                });
                let (command, snapshot) = controller.step(
                    elapsed_ns(start),
                    dt,
                    intent,
                    latest_feedback.as_ref().map(|(feedback, _)| feedback),
                    feedback_age_ms,
                );
                state.set_arm_simulation(snapshot);
                let mut encoded = serde_json::to_vec(&command)
                    .context("failed serializing Servo command")?;
                encoded.push(b'\n');
                writer.write_all(&encoded).await.context("failed writing Servo command")?;
            }
        }
    }
}

fn accept_feedback_sequence(last: &mut Option<u64>, sequence: u64) -> Result<()> {
    if last.is_some_and(|previous| sequence <= previous) {
        bail!("Servo feedback sequence must increase: previous={last:?}, current={sequence}");
    }
    *last = Some(sequence);
    Ok(())
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
        receive_time_ns: intent.receive_time_ns,
        sample_sequence: intent.sample_sequence,
        relative_position_m: intent
            .relative_position
            .map(|position| position.map(f64::from)),
        relative_orientation_xyzw: intent
            .relative_orientation
            .map(|orientation| orientation.map(f64::from)),
        gripper_closed: intent.gripper_closed,
    }
}

fn usb_reader_loop(
    state: AppState,
    controller_position_mode: ControllerPositionMode,
    position_filter_settings: PositionFilterSettings,
    gyro_calibration_file: PathBuf,
) {
    let process_start = Instant::now();
    let mut teleop = TeleopIntentMachine::new();
    let mut gyro_bias_store = match GyroBiasStore::load(&gyro_calibration_file) {
        Ok(store) => store,
        Err(error) => {
            eprintln!("陀螺仪零偏文件加载失败，将重新标定：{error:#}");
            GyroBiasStore::default()
        }
    };
    let mut gyro_bias_persisted_or_attempted =
        std::array::from_fn(|source_id| gyro_bias_store.bias(source_id).is_some());
    state.set_teleop_intent(teleop.latest().clone());
    loop {
        state.set_status("正在连接 NOLO USB…");
        let api = match HidApi::new() {
            Ok(api) => api,
            Err(error) => {
                state.set_status(format!("初始化 HID 失败：{error}"));
                thread::sleep(Duration::from_secs(2));
                continue;
            }
        };
        let opened = SUPPORTED_DEVICES
            .into_iter()
            .find_map(|(vid, pid)| api.open(vid, pid).ok().map(|device| (device, vid, pid)));
        let (device, vid, pid) = match opened {
            Some(opened) => opened,
            None => {
                state.set_status(
                    "未能打开 NOLO USB (0483:5750 或 28e9:028a)；请检查连接、权限以及是否有其他程序独占设备",
                );
                thread::sleep(Duration::from_secs(2));
                continue;
            }
        };

        state.set_status(format!(
            "NOLO USB ({vid:04x}:{pid:04x}) 已连接，等待位姿数据"
        ));
        let mut controllers: [ControllerState; 2] = std::array::from_fn(|source_id| {
            ControllerState::new(position_filter_settings, gyro_bias_store.bias(source_id))
        });
        let mut hmd = HmdState::new(gyro_bias_store.bias(2));
        let mut report = [0_u8; REPORT_SIZE + 1];
        let mut last_usb_report = Instant::now();

        loop {
            let size = match device.read_timeout(&mut report, 500) {
                Ok(0) => {
                    let now = Instant::now();
                    let silent = now.duration_since(last_usb_report);
                    if silent > SEQUENCE_STALE_AFTER {
                        let time_ns = elapsed_ns(process_start);
                        state.mark_all_offline(time_ns, silent);
                        state.set_teleop_intent(
                            teleop.force_fault(time_ns, TeleopStopReason::CommunicationStale),
                        );
                        hmd.published_online = false;
                        for controller in &mut controllers {
                            controller.published_online = false;
                            controller.reset_position_filter();
                        }
                        state.set_status("NOLO USB 报告已超时，所有位姿均不可用");
                    }
                    continue;
                }
                Ok(size) => size,
                Err(error) => {
                    let time_ns = elapsed_ns(process_start);
                    state.set_teleop_intent(
                        teleop.force_fault(time_ns, TeleopStopReason::UsbDisconnected),
                    );
                    state.set_status(format!("NOLO USB 读取中断：{error}；2 秒后重连"));
                    break;
                }
            };
            let raw_report: &[u8] = match size {
                REPORT_SIZE => &report[..REPORT_SIZE],
                size if size == REPORT_SIZE + 1 && report[0] == 0 => &report[1..],
                other => {
                    let time_ns = elapsed_ns(process_start);
                    state.set_teleop_intent(
                        teleop.force_fault(time_ns, TeleopStopReason::InvalidUsbReport),
                    );
                    state.set_status(format!("收到长度异常的 HID 报告：{other} 字节"));
                    continue;
                }
            };
            last_usb_report = Instant::now();
            let raw = match decode_report(raw_report) {
                Ok(Some(frame)) => frame,
                // Other HID report types are not controller pose samples. They
                // neither refresh nor invalidate Controller 0; its own sequence
                // freshness remains the authoritative safety signal.
                Ok(None) => continue,
                Err(error) => {
                    let time_ns = elapsed_ns(process_start);
                    state.set_teleop_intent(
                        teleop.force_fault(time_ns, TeleopStopReason::InvalidUsbReport),
                    );
                    state.set_status(format!("NOLO 报告解析失败：{error}"));
                    continue;
                }
            };

            let now = Instant::now();
            let time_ns = elapsed_ns(process_start);
            let hmd_sample =
                hmd.samples
                    .observe(usize::from(raw.controller_id), raw.hmd_sequence, now);
            if state.take_pose_calibration_request(2) {
                hmd.fusion.start_pose_calibration();
            } else if state.take_gyro_calibration_request(2) {
                hmd.fusion.start_gyro_calibration();
                gyro_bias_persisted_or_attempted[2] = false;
            }
            if hmd_sample.changed {
                hmd.orientation = hmd.fusion.update(raw, now);
                persist_completed_gyro_bias(
                    2,
                    hmd.fusion.completed_gyro_bias(),
                    &mut gyro_bias_store,
                    &mut gyro_bias_persisted_or_attempted,
                    &gyro_calibration_file,
                );
            }
            if hmd_sample.changed || (!hmd_sample.fresh && hmd.published_online) {
                state.set_pose(hmd_pose(
                    raw,
                    hmd.orientation,
                    hmd.fusion.diagnostics(),
                    hmd_sample,
                    time_ns,
                ));
                hmd.published_online = hmd_sample.fresh;
            }

            let controller_id = usize::from(raw.controller_id);
            let controller = &mut controllers[controller_id];
            let controller_sample = controller.samples.observe(raw.controller_sequence, now);
            if state.take_pose_calibration_request(controller_id) {
                if !controller.fusion.diagnostics().gyro_calibration_complete {
                    controller.reset_position_filter();
                }
                controller.fusion.start_pose_calibration();
            } else if state.take_gyro_calibration_request(controller_id) {
                controller.fusion.start_gyro_calibration();
                gyro_bias_persisted_or_attempted[controller_id] = false;
            }
            if controller_sample.changed {
                controller.orientation = controller.fusion.update(raw, now);
                persist_completed_gyro_bias(
                    controller_id,
                    controller.fusion.completed_gyro_bias(),
                    &mut gyro_bias_store,
                    &mut gyro_bias_persisted_or_attempted,
                    &gyro_calibration_file,
                );
                let marker_position = raw.position;
                let grip_position =
                    estimated_grip_position(marker_position, controller.orientation);
                let selected_position =
                    controller_position_mode.select(marker_position, grip_position);
                controller.filtered_position = controller
                    .position_filter
                    .update(selected_position, time_ns as f64 * 1.0e-9);
            } else if !controller_sample.fresh && controller.published_online {
                controller.reset_position_filter();
            }
            if controller_sample.changed
                || (!controller_sample.fresh && controller.published_online)
            {
                let pose = controller_pose(
                    raw,
                    controller,
                    controller_sample,
                    hmd_sample,
                    time_ns,
                    controller_position_mode,
                );
                if controller_id == 0 {
                    state.set_teleop_intent(teleop.update(teleop_sample(&pose)));
                }
                state.set_pose(pose);
                controller.published_online = controller_sample.fresh;
            }
            for (other_id, other) in controllers.iter_mut().enumerate() {
                if other_id == controller_id {
                    continue;
                }
                let other_sample = other.samples.status(now);
                if !other_sample.fresh && other.published_online {
                    other.reset_position_filter();
                    if other_id == 0 {
                        state.set_teleop_intent(
                            teleop.force_fault(time_ns, TeleopStopReason::CommunicationStale),
                        );
                    }
                    state.mark_source_offline(other_id, time_ns, other_sample, hmd_sample);
                    other.published_online = false;
                }
            }
            state.set_status(if !hmd_sample.fresh {
                "HMD/中继采样序号已停止"
            } else {
                "原始 USB 数据已连接"
            });
        }
        thread::sleep(Duration::from_secs(2));
    }
}

struct ControllerState {
    fusion: ControllerFusion,
    samples: SampleTracker,
    orientation: [f32; 4],
    position_filter: PositionFilter,
    filtered_position: Option<[f32; 3]>,
    published_online: bool,
}

impl ControllerState {
    fn new(
        position_filter_settings: PositionFilterSettings,
        saved_gyro_bias: Option<[f32; 3]>,
    ) -> Self {
        let mut fusion = ControllerFusion::new();
        if let Some(bias) = saved_gyro_bias {
            let loaded = fusion.load_gyro_bias(bias);
            debug_assert!(loaded);
        }
        Self {
            fusion,
            samples: SampleTracker::new(120.0, SEQUENCE_STALE_AFTER),
            orientation: [0.0, 0.0, 0.0, 1.0],
            position_filter: PositionFilter::new(position_filter_settings),
            filtered_position: None,
            published_online: false,
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
    published_online: bool,
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
            samples: InterleavedSampleTracker::new(120.0, SEQUENCE_STALE_AFTER),
            orientation: [0.0, 0.0, 0.0, 1.0],
            published_online: false,
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
) -> PoseFrame {
    PoseFrame {
        time_ns,
        source_id: 2,
        source_kind: "head",
        sample_rate_hz: 240,
        controller_id: None,
        device: DEVICE_NAMES[2],
        flags: 0,
        position: raw.hmd_position,
        filtered_position: None,
        marker_position: raw.hmd_position,
        grip_position: None,
        position_mode: "marker",
        orientation,
        communication_fresh: sample.fresh,
        sample_changed: sample.changed,
        optical_tracking_valid: None,
        pose_usable: sample.fresh && sample.changed,
        usb_silent_ms: 0,
        unchanged_ms: duration_ms(sample.unchanged),
        hmd_unchanged_ms: duration_ms(sample.unchanged),
        source_online: sample.fresh,
        hmd_relay_online: sample.fresh,
        sample_sequence: raw.hmd_sequence,
        controller_sequence: None,
        hmd_sequence: raw.hmd_sequence,
        sequence_delta: sample.sequence_delta,
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
        menu_active: false,
        menu_pressed: false,
        buttons_raw: 0,
        touchpad_pressed: false,
        trigger_pressed: false,
        home_pressed: false,
        squeeze_pressed: false,
        touchpad_touched: false,
        touchpad: None,
        accelerometer_raw: raw.hmd_accelerometer,
        gyroscope_raw: raw.hmd_gyroscope,
    }
}

fn controller_pose(
    raw: nolo_usb_server::protocol::RawFrame,
    controller: &ControllerState,
    sample: SampleObservation,
    hmd_sample: SampleObservation,
    time_ns: u64,
    position_mode: ControllerPositionMode,
) -> PoseFrame {
    let orientation = controller.orientation;
    let fusion = controller.fusion.diagnostics();
    let marker_position = raw.position;
    let grip_position = estimated_grip_position(marker_position, orientation);
    let position = position_mode.select(marker_position, grip_position);
    let online = sample.fresh;
    PoseFrame {
        time_ns,
        source_id: raw.controller_id,
        source_kind: "controller",
        sample_rate_hz: 120,
        controller_id: Some(raw.controller_id),
        device: DEVICE_NAMES[usize::from(raw.controller_id)],
        flags: 0,
        position,
        filtered_position: controller.filtered_position,
        marker_position,
        grip_position: Some(grip_position),
        position_mode: position_mode.name(),
        orientation,
        communication_fresh: online,
        sample_changed: sample.changed,
        optical_tracking_valid: None,
        pose_usable: online && sample.changed,
        usb_silent_ms: 0,
        unchanged_ms: duration_ms(sample.unchanged),
        hmd_unchanged_ms: duration_ms(hmd_sample.unchanged),
        source_online: online,
        hmd_relay_online: hmd_sample.fresh,
        sample_sequence: raw.controller_sequence,
        controller_sequence: Some(raw.controller_sequence),
        hmd_sequence: raw.hmd_sequence,
        sequence_delta: sample.sequence_delta,
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
        menu_active: online,
        menu_pressed: online && raw.menu_pressed(),
        buttons_raw: raw.buttons,
        touchpad_pressed: raw.touchpad_pressed(),
        trigger_pressed: raw.trigger_pressed(),
        home_pressed: raw.home_pressed(),
        squeeze_pressed: raw.squeeze_pressed(),
        touchpad_touched: raw.touchpad_touched(),
        touchpad: raw.touchpad,
        accelerometer_raw: raw.accelerometer,
        gyroscope_raw: raw.gyroscope,
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
        sample_changed: frame.sample_changed,
        pose_usable: frame.pose_usable,
        optical_tracking_valid: frame.optical_tracking_valid,
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
    controller_position_mode: ControllerPositionMode,
    position_filter_settings: PositionFilterSettings,
    gyro_calibration_file: PathBuf,
    servo_ipc_path: PathBuf,
}

#[derive(Clone, Copy, Debug, Default)]
enum ControllerPositionMode {
    #[default]
    Marker,
    Grip,
}

impl ControllerPositionMode {
    fn name(self) -> &'static str {
        match self {
            Self::Marker => "marker",
            Self::Grip => "grip",
        }
    }

    fn select(self, marker: [f32; 3], grip: [f32; 3]) -> [f32; 3] {
        match self {
            Self::Marker => marker,
            Self::Grip => grip,
        }
    }
}

impl Config {
    fn parse() -> Result<Self> {
        let default_static =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../controller-viewer/public");
        let default_gyro_calibration_file =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../state/gyro-bias-v1.json");
        let default_servo_ipc_path =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../state/moveit-servo.sock");
        let mut host = "127.0.0.1".parse().unwrap();
        let mut port = 8765;
        let mut static_dir = default_static;
        let mut controller_position_mode = ControllerPositionMode::default();
        let mut position_filter_settings = PositionFilterSettings::default();
        let mut gyro_calibration_file = default_gyro_calibration_file;
        let mut servo_ipc_path = default_servo_ipc_path;
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
            } else if let Some(value) = argument.strip_prefix("--controller-position=") {
                controller_position_mode = match value {
                    "marker" => ControllerPositionMode::Marker,
                    "grip" => ControllerPositionMode::Grip,
                    _ => bail!("invalid controller position mode: {value}; use marker or grip"),
                };
            } else if let Some(value) = argument.strip_prefix("--gyro-calibration-file=") {
                gyro_calibration_file = PathBuf::from(value);
            } else if let Some(value) = argument.strip_prefix("--servo-ipc=") {
                servo_ipc_path = PathBuf::from(value);
            } else if let Some(value) = argument.strip_prefix("--position-filter-min-cutoff=") {
                position_filter_settings.min_cutoff_hz =
                    parse_finite_f64(value, "position filter minimum cutoff")?;
            } else if let Some(value) = argument.strip_prefix("--position-filter-beta=") {
                position_filter_settings.beta = parse_finite_f64(value, "position filter beta")?;
            } else if let Some(value) =
                argument.strip_prefix("--position-filter-derivative-cutoff=")
            {
                position_filter_settings.derivative_cutoff_hz =
                    parse_finite_f64(value, "position filter derivative cutoff")?;
            } else if matches!(argument.as_str(), "-h" | "--help") {
                println!(
                    "Usage: nolo-usb-server [--host=IP] [--port=PORT] [--static-dir=PATH] [--controller-position=marker|grip] [--gyro-calibration-file=PATH] [--servo-ipc=PATH] [--position-filter-min-cutoff=HZ] [--position-filter-beta=VALUE] [--position-filter-derivative-cutoff=HZ]"
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
        position_filter_settings = position_filter_settings
            .validate()
            .map_err(|error| anyhow::anyhow!("invalid position filter settings: {error}"))?;
        Ok(Self {
            host,
            port,
            static_dir,
            controller_position_mode,
            position_filter_settings,
            gyro_calibration_file,
            servo_ipc_path,
        })
    }
}

fn parse_finite_f64(value: &str, name: &str) -> Result<f64> {
    let parsed: f64 = value
        .parse()
        .with_context(|| format!("invalid {name}: {value}"))?;
    if !parsed.is_finite() {
        bail!("invalid {name}: {value}");
    }
    Ok(parsed)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn security_policy_allows_only_same_origin_frames() {
        assert!(CONTENT_SECURITY_POLICY.contains("frame-src 'self'"));
        assert!(CONTENT_SECURITY_POLICY.contains("frame-ancestors 'self'"));
        assert!(!CONTENT_SECURITY_POLICY.contains("frame-ancestors 'none'"));
    }

    #[test]
    fn status_payload_exposes_safe_initial_teleop_intent() {
        let payload = AppState::new().payload();
        assert_eq!(
            payload.latest_teleop_intent.state,
            nolo_usb_server::teleop::TeleopIntentState::Idle
        );
        assert!(!payload.latest_teleop_intent.enabled);
        assert!(!payload.latest_teleop_intent.sample_valid);
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
        assert_eq!(json["latestArmSimulation"]["backend"], "moveit_servo");
        assert_eq!(json["latestArmSimulation"]["simulation_only"], true);
        assert_eq!(json["latestArmSimulation"]["gripper_closed"], true);
        assert_eq!(
            json["latestArmSimulation"]["tcp_pose"],
            serde_json::Value::Null
        );
        assert!(json["latestArmSimulation"]["gripper_rad"].is_number());
        let model_joints = json["latestArmSimulation"]["model_joints_deg"]
            .as_array()
            .unwrap();
        for (actual, expected) in model_joints.iter().zip([0.0; 6]) {
            assert!((actual.as_f64().unwrap() - expected).abs() < 1.0e-9);
        }
        assert_eq!(
            json["latestArmSimulation"]["model_id"],
            "stararm102-fl-sim-v1"
        );
        assert!(json.get("latest_teleop_intent").is_none());
        assert!(json.get("latest_arm_simulation").is_none());
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

    #[test]
    fn servo_feedback_sequence_must_strictly_increase() {
        let mut last = None;
        assert!(accept_feedback_sequence(&mut last, 1).is_ok());
        assert!(accept_feedback_sequence(&mut last, 2).is_ok());
        assert!(accept_feedback_sequence(&mut last, 2).is_err());
        assert!(accept_feedback_sequence(&mut last, 1).is_err());
        assert_eq!(last, Some(2));
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
