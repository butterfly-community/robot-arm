//! Two stereo rigs, one camera adapter and one calibration implementation.
//! Images come only from the already-owned role streams; no second device opens.
mod capture;
mod pairing;
use crate::video_server::VideoStreams;
use axum::{
    Json, Router,
    extract::{Query, State},
    http::StatusCode,
    response::{IntoResponse, Response},
    routing::get,
};
use camera_calibration::stereo::{self as cv, Calibration, Sample};
pub use capture::{Stream, sources};
use eyre::{Result, ensure, eyre};
use pairing::{fresh_pair, pair};
use robot_arm_messages::{
    CalibrationBoard, CameraFrameBundle, CameraImagePlane, CameraIntrinsics, CameraRole,
    DepthCameraCalibration, Pose3, SCHEMA_VERSION, ToolPoseFeedback,
};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeMap,
    path::PathBuf,
    sync::{Arc, OnceLock, RwLock},
};

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, PartialOrd, Ord)]
#[serde(rename_all = "snake_case")]
pub enum Rig {
    External,
    Wrist,
}
impl Rig {
    pub fn roles(self) -> [CameraRole; 2] {
        match self {
            Self::External => [CameraRole::External, CameraRole::ExternalRight],
            Self::Wrist => [CameraRole::Wrist, CameraRole::WristRight],
        }
    }
    pub fn source(self) -> &'static str {
        match self {
            Self::External => "stereo:external",
            Self::Wrist => "stereo:wrist",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Saved {
    pub sources: [String; 2],
    pub capture_fps: u32,
    pub board: CalibrationBoard,
    pub optics: Calibration,
    /// Original left camera in base (external) or tool (wrist).
    pub camera_pose: Option<Pose3>,
    pub translation_residuals_m: Vec<f64>,
    pub rotation_residuals_rad: Vec<f64>,
}
#[derive(Clone, Default)]
struct Session {
    busy: bool,
    message: String,
    original_error: Option<String>,
    pair_delta_ms: Option<f64>,
    candidate: Option<Saved>,
    applied: Option<Saved>,
    samples: Vec<Sample>,
    identity: Option<([String; 2], u32, u32)>,
    board: Option<CalibrationBoard>,
    poses: Vec<(Pose3, Pose3)>,
    last_pair: Option<[i64; 2]>,
    capture_fps: u32,
}

#[derive(Serialize)]
struct SessionStatus<'a> {
    busy: bool,
    message: &'a str,
    original_error: &'a Option<String>,
    sample_count: usize,
    pose_sample_count: usize,
    pair_delta_ms: Option<f64>,
    candidate: &'a Option<Saved>,
    applied: &'a Option<Saved>,
}
impl Session {
    fn status(&self) -> SessionStatus<'_> {
        SessionStatus {
            busy: self.busy,
            message: &self.message,
            original_error: &self.original_error,
            sample_count: self.samples.len(),
            pose_sample_count: self.poses.len(),
            pair_delta_ms: self.pair_delta_ms,
            candidate: &self.candidate,
            applied: &self.applied,
        }
    }
}

struct Matcher {
    optics: Calibration,
    matcher: cv::Matcher,
}

pub struct Shared {
    videos: VideoStreams,
    feedback: tokio::sync::watch::Sender<Option<ToolPoseFeedback>>,
    feedback_history: std::sync::Mutex<std::collections::VecDeque<ToolPoseFeedback>>,
    sessions: RwLock<BTreeMap<Rig, Session>>,
    saved: RwLock<BTreeMap<Rig, Saved>>,
    config_path: PathBuf,
    operation: tokio::sync::Mutex<()>,
    jobs: std::sync::Mutex<BTreeMap<Rig, tokio::task::AbortHandle>>,
    matchers: BTreeMap<Rig, std::sync::Mutex<Option<Matcher>>>,
}
impl Shared {
    // One OpenCV matcher per rig, shared by capture and on-demand observations.
    // Only optical changes rebuild rectification maps; hand-eye changes do not.
    fn with_matcher<T>(
        &self,
        rig: Rig,
        optics: &Calibration,
        f: impl FnOnce(&mut cv::Matcher) -> Result<T>,
    ) -> Result<T> {
        let mut slot = self.matchers[&rig].lock().unwrap();
        if slot.as_ref().is_none_or(|m| m.optics != *optics) {
            *slot = Some(Matcher {
                optics: optics.clone(),
                matcher: cv::Matcher::new(optics)?,
            });
        }
        f(&mut slot.as_mut().unwrap().matcher)
    }
}
static SHARED: OnceLock<Arc<Shared>> = OnceLock::new();
pub fn shared() -> Result<Arc<Shared>> {
    SHARED
        .get()
        .cloned()
        .ok_or_else(|| eyre!("双目服务尚未初始化"))
}

/// The existing camera reset command explicitly deletes this source's saved calibration.
/// Starting a new calibration from the calibration card intentionally preserves it.
pub fn forget(source_id: &str) -> Result<()> {
    let s = shared()?;
    let rig = [Rig::External, Rig::Wrist]
        .into_iter()
        .find(|rig| rig.source() == source_id)
        .ok_or_else(|| eyre!("未知双目来源"))?;
    let _guard = s
        .operation
        .try_lock()
        .map_err(|_| eyre!("请等待当前标定操作完成后重置相机"))?;
    let mut saved = s.saved.read().unwrap().clone();
    saved.remove(&rig);
    json_config_store::save(&s.config_path, &saved)?;
    *s.saved.write().unwrap() = saved;
    s.sessions.write().unwrap().insert(rig, Session::default());
    Ok(())
}
pub fn initialize(videos: VideoStreams, config_path: PathBuf) -> Result<()> {
    let saved: BTreeMap<Rig, Saved> = json_config_store::load_or_default(&config_path)?;
    let sessions = [Rig::External, Rig::Wrist]
        .into_iter()
        .map(|r| {
            (
                r,
                Session {
                    applied: saved.get(&r).cloned(),
                    message: if saved.contains_key(&r) {
                        "已恢复保存的标定，可直接使用".into()
                    } else {
                        "尚未开始".into()
                    },
                    ..Default::default()
                },
            )
        })
        .collect();
    let (feedback, _) = tokio::sync::watch::channel(None);
    SHARED
        .set(Arc::new(Shared {
            videos,
            feedback,
            feedback_history: Default::default(),
            sessions: RwLock::new(sessions),
            saved: RwLock::new(saved),
            config_path,
            operation: tokio::sync::Mutex::new(()),
            jobs: std::sync::Mutex::new(BTreeMap::new()),
            matchers: [Rig::External, Rig::Wrist]
                .into_iter()
                .map(|rig| (rig, Default::default()))
                .collect(),
        }))
        .map_err(|_| eyre!("双目服务重复初始化"))
}
pub fn update_feedback(feedback: ToolPoseFeedback) {
    if let Ok(shared) = shared() {
        let mut history = shared.feedback_history.lock().unwrap();
        if history.back().is_none_or(|previous| {
            previous.arm_state.sample_time_ns < feedback.arm_state.sample_time_ns
        }) {
            history.push_back(feedback.clone());
            // Bounded feedback cache only, not a motion or calibration threshold.
            while history.len() > 256 {
                history.pop_front();
            }
        }
        shared.feedback.send_replace(Some(feedback));
    }
}

fn feedback_at(s: &Shared, time: i64) -> Result<nalgebra::Isometry3<f64>> {
    let history = s.feedback_history.lock().unwrap();
    let before = history
        .iter()
        .rev()
        .find(|v| v.arm_state.sample_time_ns <= time)
        .ok_or_else(|| eyre!("图像时间之前没有实际关节反馈，等待新的同步采样"))?;
    let after = history
        .iter()
        .find(|v| v.arm_state.sample_time_ns >= time)
        .ok_or_else(|| eyre!("等待图像时间之后的实际关节反馈"))?;
    let pose = |v: &ToolPoseFeedback| {
        pose_iso(&Pose3 {
            position_m: v.pose.position_m,
            orientation_xyzw: v.pose.orientation_xyzw,
        })
    };
    let a = pose(before);
    let b = pose(after);
    let span = after.arm_state.sample_time_ns - before.arm_state.sample_time_ns;
    let t = if span == 0 {
        0.0
    } else {
        (time - before.arm_state.sample_time_ns) as f64 / span as f64
    };
    Ok(nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::from(a.translation.vector.lerp(&b.translation.vector, t)),
        a.rotation.slerp(&b.rotation, t),
    ))
}

#[derive(Deserialize)]
struct View {
    rig: Rig,
    u: Option<f64>,
    v: Option<f64>,
    #[serde(default)]
    image: bool,
}
#[derive(Deserialize)]
struct Request {
    rig: Rig,
    action: Action,
    board: Option<CalibrationBoard>,
}
#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Action {
    Sample,
    Solve,
    Apply,
    Reset,
    SamplePose,
    SolvePose,
}

pub fn router() -> Router {
    Router::new()
        .route("/stereo", get(status).post(request))
        .route("/stereo/depth", get(depth))
        .with_state(shared().expect("initialized before HTTP serving"))
}
async fn status(State(s): State<Arc<Shared>>) -> Response {
    let sessions = s.sessions.read().unwrap();
    Json(
        sessions
            .iter()
            .map(|(rig, session)| (*rig, session.status()))
            .collect::<BTreeMap<_, _>>(),
    )
    .into_response()
}
async fn request(State(s): State<Arc<Shared>>, Json(req): Json<Request>) -> Response {
    if matches!(req.action, Action::Reset) {
        if let Some(job) = s.jobs.lock().unwrap().remove(&req.rig) {
            job.abort();
        }
        s.sessions.write().unwrap().get_mut(&req.rig).unwrap().busy = false;
    }
    {
        let mut sessions = s.sessions.write().unwrap();
        let session = sessions.get_mut(&req.rig).unwrap();
        if session.busy {
            return (
                StatusCode::CONFLICT,
                Json(serde_json::json!({"original_error":"当前标定操作尚未完成"})),
            )
                .into_response();
        }
        session.busy = true;
        session.original_error = None;
        session.message = match req.action {
            Action::Sample | Action::SamplePose => "正在等待左右相机的新图像",
            Action::Solve => "OpenCV 正在求解内参与双目关系",
            Action::SolvePose => "OpenCV 正在求解机械臂外参",
            Action::Apply => "正在保存并应用标定",
            Action::Reset => "正在开始新一轮标定",
        }
        .into();
    }
    let registry = s.clone();
    let rig = req.rig;
    let job = tokio::spawn(async move {
        let _guard = s.operation.lock().await;
        let rig = req.rig;
        let result = operate(s.clone(), req).await;
        let mut sessions = s.sessions.write().unwrap();
        let session = sessions.get_mut(&rig).unwrap();
        session.busy = false;
        if let Err(e) = result {
            session.original_error = Some(format!("{e:#}"));
            session.message = "操作失败，已应用结果未被覆盖".into();
        }
    });
    registry
        .jobs
        .lock()
        .unwrap()
        .insert(rig, job.abort_handle());
    (
        StatusCode::ACCEPTED,
        Json(serde_json::json!({"accepted":true})),
    )
        .into_response()
}

async fn operate(s: Arc<Shared>, req: Request) -> Result<()> {
    let rig = req.rig;
    match req.action {
        Action::Reset => {
            let applied = s.saved.read().unwrap().get(&rig).cloned();
            s.sessions.write().unwrap().insert(
                rig,
                Session {
                    applied,
                    message: "新一轮采样，已保存结果继续保留".into(),
                    ..Default::default()
                },
            );
        }
        Action::Sample | Action::SamplePose => {
            let p = fresh_pair(s.clone(), rig).await?;
            let existing = s.sessions.read().unwrap()[&rig].clone();
            let board = req
                .board
                .or(existing.board.clone())
                .or_else(|| existing.applied.as_ref().map(|a| a.board.clone()))
                .unwrap_or(serde_json::from_str(include_str!(
                    "../assets/charuco-board.json"
                ))?);
            if let Some(old) = &existing.board {
                ensure!(*old == board, "标定板参数已改变，请重新开始采样");
            }
            let identity = (p.sources.clone(), p.width, p.height);
            if let Some(old) = &existing.identity {
                ensure!(
                    *old == identity,
                    "采样期间相机或图像尺寸发生变化，请重新开始标定"
                );
            }
            ensure!(
                existing.last_pair != Some(p.times),
                "这对图像已采集，请改变姿态后重新采样"
            );
            let delta = (p.times[0] - p.times[1]).unsigned_abs() as f64 / 1e6;
            let times = p.times;
            let capture_fps = p.fps;
            let pose_sample = matches!(req.action, Action::SamplePose);
            let feedback = if pose_sample {
                let mut feedback = s.feedback.subscribe();
                ensure!(
                    feedback.borrow().is_some(),
                    "没有机械臂实际反馈，请先连接执行节点或启用虚拟反馈"
                );
                s.sessions.write().unwrap().get_mut(&rig).unwrap().message =
                    "已采集图像，等待同次采样的新电机反馈及 TCP；请保持姿态不动".into();
                loop {
                    let value = feedback.borrow_and_update().clone();
                    if let Some(value) = value
                        && value.arm_state.sample_time_ns >= *times.iter().max().unwrap()
                    {
                        break Some(iso_pose(&feedback_at(&s, times[0])?));
                    }
                    feedback.changed().await?;
                }
            } else {
                None
            };
            let b = board.clone();
            let (sample, pose) = tokio::task::spawn_blocking(move || -> Result<_> {
                let left = p.left()?;
                let sample = if pose_sample {
                    None
                } else {
                    Some(Sample {
                        left: cv::detect(&left, p.width, p.height, &b)?,
                        right: cv::detect(&p.right()?, p.width, p.height, &b)?,
                    })
                };
                let pose = if let Some(feedback) = feedback {
                    let saved = existing
                        .candidate
                        .as_ref()
                        .or(existing.applied.as_ref())
                        .ok_or_else(|| eyre!("先求解双目内参，再采集机械臂外参"))?;
                    ensure!(
                        saved.sources == p.sources
                            && saved.optics.width == p.width
                            && saved.optics.height == p.height,
                        "图像来源与双目标定不匹配"
                    );
                    let detected = camera_calibration::detect(
                        &left,
                        p.width,
                        p.height,
                        &saved.optics.left_k,
                        &saved.optics.left_d,
                        "plumb_bob",
                        &b,
                    )?;
                    Some((feedback, detected.board_in_camera))
                } else {
                    None
                };
                Ok((sample, pose))
            })
            .await??;
            let mut sessions = s.sessions.write().unwrap();
            let session = sessions.get_mut(&rig).unwrap();
            session.identity = Some(identity);
            session.board = Some(board);
            session.last_pair = Some(times);
            session.pair_delta_ms = Some(delta);
            session.capture_fps = capture_fps;
            if let Some(p) = pose {
                session.poses.push(p);
            } else if let Some(sample) = sample {
                session.samples.push(sample);
            }
            session.message = "采样完成，请改变位置及倾角继续采样".into();
        }
        Action::Solve => {
            let session = s.sessions.read().unwrap()[&rig].clone();
            let (sources, w, h) = session.identity.ok_or_else(|| eyre!("请先采样"))?;
            let board = session.board.ok_or_else(|| eyre!("缺少标定板参数"))?;
            let capture_fps = session.capture_fps;
            let b = board.clone();
            let optics =
                tokio::task::spawn_blocking(move || cv::calibrate(&session.samples, w, h, &b))
                    .await??;
            let mut sessions = s.sessions.write().unwrap();
            let session = sessions.get_mut(&rig).unwrap();
            session.candidate = Some(Saved {
                sources,
                capture_fps,
                board,
                optics,
                camera_pose: None,
                translation_residuals_m: vec![],
                rotation_residuals_rad: vec![],
            });
            session.poses.clear();
            session.message = "双目参数已求解；可保存生成深度，机械臂定位还需外参".into();
        }
        Action::SolvePose => {
            let session = s.sessions.read().unwrap()[&rig].clone();
            let mut candidate = session
                .candidate
                .or(session.applied)
                .ok_or_else(|| eyre!("先求解双目参数"))?;
            let (tools, boards): (Vec<_>, Vec<_>) = session.poses.into_iter().unzip();
            ensure!(tools.len() >= 3, "请采集至少三个不同的机械臂姿态");
            let solved = tokio::task::spawn_blocking(move || match rig {
                Rig::External => camera_calibration::solve(&tools, &boards),
                Rig::Wrist => cv::solve_wrist(&tools, &boards),
            })
            .await??;
            ensure!(
                solved
                    .camera_in_base
                    .position_m
                    .iter()
                    .chain(&solved.camera_in_base.orientation_xyzw)
                    .chain(&solved.translation_residuals_m)
                    .chain(&solved.rotation_residuals_rad)
                    .all(|v| v.is_finite()),
                "外参求解退化，请增加不同位置和不同旋转轴的姿态"
            );
            candidate.camera_pose = Some(solved.camera_in_base);
            candidate.translation_residuals_m = solved.translation_residuals_m;
            candidate.rotation_residuals_rad = solved.rotation_residuals_rad;
            let mut sessions = s.sessions.write().unwrap();
            let session = sessions.get_mut(&rig).unwrap();
            session.candidate = Some(candidate);
            session.message = "机械臂外参已求解，核对误差后保存应用".into();
        }
        Action::Apply => {
            let candidate = s.sessions.read().unwrap()[&rig]
                .candidate
                .clone()
                .ok_or_else(|| eyre!("没有待应用结果"))?;
            let mut saved = s.saved.read().unwrap().clone();
            saved.insert(rig, candidate.clone());
            json_config_store::save(&s.config_path, &saved)?;
            *s.saved.write().unwrap() = saved;
            let mut sessions = s.sessions.write().unwrap();
            let session = sessions.get_mut(&rig).unwrap();
            session.applied = Some(candidate);
            session.candidate = None;
            session.message = "标定已保存并应用；在相机来源中刷新并选择双目深度".into();
        }
    }
    Ok(())
}

fn now_ns() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as i64
}

async fn depth(State(s): State<Arc<Shared>>, Query(view): Query<View>) -> Response {
    let result = async {
        let saved = s.saved.read().unwrap().get(&view.rig).cloned()
            .ok_or_else(|| eyre!("请先完成并应用双目标定"))?;
        let pair = fresh_pair(s.clone(), view.rig).await?;
        tokio::task::spawn_blocking(move || -> Result<_> {
            pair.validate(&saved)?;
            let u = view.u.unwrap_or(0.5);
            let v = view.v.unwrap_or(0.5);
            ensure!(
                u.is_finite() && v.is_finite() && (0.0..=1.0).contains(&u) && (0.0..=1.0).contains(&v),
                "像素位置使用 0 到 1 的归一化坐标"
            );
            let left = pair.left()?;
            let right = pair.right()?;
            let depth = s.with_matcher(view.rig, &saved.optics, |m| m.compute(&left, &right))?;
            let (count, min, max) = depth.z_m.iter().copied().filter(|z| z.is_finite()).fold(
                (0usize, f32::INFINITY, f32::NEG_INFINITY),
                |(count, min, max), z| (count + 1, min.min(z), max.max(z)),
            );
            let x = (u * (pair.width - 1) as f64).round() as usize;
            let y = (v * (pair.height - 1) as f64).round() as usize;
            let z = depth.z_m[y * pair.width as usize + x] as f64;
            let k = depth.k;
            let point = z.is_finite().then(|| [(x as f64 - k[2]) * z / k[0], (y as f64 - k[5]) * z / k[4], z]);
            let in_base = if let Some(point) = point {
                camera_in_base(&s, view.rig, &saved, pair.times[0], &depth.rectified_in_left)?
                    .map(|camera| <[f64; 3]>::from((camera * nalgebra::Point3::from(point)).coords))
            } else {
                None
            };
            let report = serde_json::json!({
                "source_id": view.rig.source(), "received_time_ns": pair.times,
                "width": pair.width, "height": pair.height,
                "pair_delta_ms": (pair.times[0] - pair.times[1]).unsigned_abs() as f64 / 1e6,
                "valid_pixels": count, "total_pixels": depth.z_m.len(),
                "min_depth_m": (count > 0).then_some(min),
                "max_depth_m": (count > 0).then_some(max),
                "camera_matrix": depth.k, "robot_localization_ready": saved.camera_pose.is_some(),
                "pixel_xy": [x, y], "point_in_rectified_camera_m": point, "point_in_base_m": in_base,
                "frame": "rectified_left_optical",
                "synchronization": "host receipt timestamp pairing; not hardware synchronized"
            });
            let png = if view.image {
                let rgb = image::RgbImage::from_raw(pair.width, pair.height, depth.rgb)
                    .ok_or_else(|| eyre!("图像尺寸无效"))?;
                let mut out = std::io::Cursor::new(Vec::new());
                rgb.write_to(&mut out, image::ImageFormat::Png)?;
                Some(out.into_inner())
            } else {
                None
            };
            Ok((report, png))
        }).await?
    }.await;
    match result {
        Ok((v, Some(png))) => (
            [
                ("content-type", "image/png"),
                ("cache-control", "no-store"),
                ("x-camera-frame", &v.to_string()),
            ],
            png,
        )
            .into_response(),
        Ok((v, None)) => Json(v).into_response(),
        Err(e) => (
            StatusCode::BAD_REQUEST,
            Json(serde_json::json!({"original_error":format!("{e:#}")})),
        )
            .into_response(),
    }
}

fn camera_in_base(
    s: &Shared,
    rig: Rig,
    saved: &Saved,
    time: i64,
    rectified_in_left: &Pose3,
) -> Result<Option<nalgebra::Isometry3<f64>>> {
    saved
        .camera_pose
        .as_ref()
        .map(|pose| {
            let camera = pose_iso(pose) * pose_iso(rectified_in_left);
            Ok(if rig == Rig::Wrist {
                feedback_at(s, time)? * camera
            } else {
                camera
            })
        })
        .transpose()
}

fn pose_iso(p: &Pose3) -> nalgebra::Isometry3<f64> {
    nalgebra::Isometry3::from_parts(
        nalgebra::Translation3::from(p.position_m),
        nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            p.orientation_xyzw[3],
            p.orientation_xyzw[0],
            p.orientation_xyzw[1],
            p.orientation_xyzw[2],
        )),
    )
}
fn iso_pose(p: &nalgebra::Isometry3<f64>) -> Pose3 {
    let q = p.rotation.quaternion();
    Pose3 {
        position_m: p.translation.vector.into(),
        orientation_xyzw: [q.i, q.j, q.k, q.w],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use robot_arm_messages::{ArmState, FeedbackSource, ToolPose};

    fn shared_fixture() -> Shared {
        Shared {
            videos: Default::default(),
            feedback: tokio::sync::watch::channel(None).0,
            feedback_history: Default::default(),
            sessions: Default::default(),
            saved: Default::default(),
            config_path: PathBuf::new(),
            operation: tokio::sync::Mutex::new(()),
            jobs: Default::default(),
            matchers: [Rig::External, Rig::Wrist]
                .into_iter()
                .map(|rig| (rig, Default::default()))
                .collect(),
        }
    }

    fn calibration_fixture() -> Saved {
        Saved {
            sources: ["test:left".into(), "test:right".into()],
            capture_fps: 10,
            board: serde_json::from_str(include_str!("../assets/charuco-board.json")).unwrap(),
            optics: Calibration {
                width: 64,
                height: 48,
                left_k: [60., 0., 32., 0., 60., 24., 0., 0., 1.],
                right_k: [60., 0., 32., 0., 60., 24., 0., 0., 1.],
                left_d: vec![0.; 5],
                right_d: vec![0.; 5],
                left_to_right: Pose3 {
                    position_m: [-0.06, 0., 0.],
                    orientation_xyzw: [0., 0., 0., 1.],
                },
                left_rms_px: 0.,
                right_rms_px: 0.,
                stereo_rms_px: 0.,
                sample_count: 3,
            },
            camera_pose: None,
            translation_residuals_m: vec![],
            rotation_residuals_rad: vec![],
        }
    }

    #[test]
    fn status_derives_counts_without_serializing_samples() {
        let corners = cv::Corners {
            ids: vec![1],
            pixels: vec![[2., 3.]],
        };
        let mut session = Session::default();
        session.samples.push(Sample {
            left: corners.clone(),
            right: corners,
        });
        let pose = iso_pose(&nalgebra::Isometry3::identity());
        session.poses.push((pose.clone(), pose));
        let value = serde_json::to_value(session.status()).unwrap();
        assert_eq!(value["sample_count"], 1);
        assert_eq!(value["pose_sample_count"], 1);
        assert!(value.get("samples").is_none());
        assert!(value.get("poses").is_none());
        session.samples.clear();
        session.poses.clear();
        let value = serde_json::to_value(session.status()).unwrap();
        assert_eq!(value["sample_count"], 0);
        assert_eq!(value["pose_sample_count"], 0);
    }

    #[test]
    fn matcher_is_reused_until_optics_change() {
        let s = shared_fixture();
        let mut saved = calibration_fixture();
        let rgb = vec![128; 64 * 48 * 3];
        let first = s
            .with_matcher(Rig::External, &saved.optics, |m| m.rectify_color(&rgb))
            .unwrap();
        let allocation = || {
            s.matchers[&Rig::External]
                .lock()
                .unwrap()
                .as_ref()
                .unwrap()
                .optics
                .left_d
                .as_ptr() as usize
        };
        let original = allocation();
        saved.camera_pose = Some(iso_pose(&nalgebra::Isometry3::identity()));
        let next = s
            .with_matcher(Rig::External, &saved.optics, |m| m.rectify_color(&rgb))
            .unwrap();
        assert_eq!(first, next);
        assert_eq!(original, allocation());
        assert!(s.matchers[&Rig::Wrist].lock().unwrap().is_none());
        saved.optics.left_k[0] = 61.;
        s.with_matcher(Rig::External, &saved.optics, |_| Ok(()))
            .unwrap();
        assert_ne!(original, allocation());
    }

    #[test]
    fn pairing_needs_no_decode_and_preview_does_not_decode_right() {
        use crate::drivers::CameraStream;
        use robot_arm_messages::CameraRawVideoFrame;
        let mut s = shared_fixture();
        let saved = calibration_fixture();
        let raw = |source: &str, format: &str, data: Vec<u8>| CameraRawVideoFrame {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            source_id: source.into(),
            device_time_ns: 100,
            received_time_ns: 100,
            color: CameraImagePlane {
                width: 64,
                height: 48,
                stride_bytes: 64 * 3,
                pixel_format: format.into(),
                frame_id: "test".into(),
                data,
            },
        };
        let left = camera_media::Encoder::new(64, 48, 10.)
            .unwrap()
            .encode(raw("test:left", "rgb8", vec![128; 64 * 48 * 3]), true)
            .unwrap();
        let invalid_right = Arc::new(raw("test:right", "h264", vec![0]));
        let right = camera_media::VideoFrame {
            latest: invalid_right.clone(),
            gop: vec![invalid_right],
            previous_gop: vec![],
            stream_id: 2,
            frames_per_second: 10.,
        };
        for (role, frame) in [
            (CameraRole::External, left),
            (CameraRole::ExternalRight, right),
        ] {
            s.videos
                .insert(role, tokio::sync::watch::channel(Some(Arc::new(frame))).1);
        }
        let p = pair(&s, Rig::External).unwrap();
        assert_eq!(p.times, [100, 100]);
        p.validate(&saved).unwrap();
        s.saved
            .write()
            .unwrap()
            .insert(Rig::External, saved.clone());
        let mut stream = capture::Stream::test_stream(Arc::new(s), Rig::External, saved);
        assert!(matches!(
            stream.poll_frame(false).unwrap(),
            crate::drivers::CameraPoll::Captured { frame: None, .. }
        ));
        stream.test_next_poll();
        // Same pair: reject before attempting to decode even in depth mode.
        assert!(matches!(
            stream.poll_frame(true).unwrap(),
            crate::drivers::CameraPoll::Pending
        ));
    }

    #[test]
    fn image_time_uses_bracketing_actual_feedback_not_latest_pose() {
        let feedback = |time, x, angle| ToolPoseFeedback {
            pose: ToolPose {
                frame: "base_link".into(),
                position_m: [x, 0.0, 0.0],
                orientation_xyzw: iso_pose(&nalgebra::Isometry3::new(
                    nalgebra::Vector3::zeros(),
                    nalgebra::Vector3::new(0.0, 0.0, angle),
                ))
                .orientation_xyzw,
            },
            arm_state: ArmState {
                schema_version: SCHEMA_VERSION,
                sequence: time as u64,
                sample_time_ns: time,
                model_revision: "timestamp-test".into(),
                joints_rad: vec![],
                actuators_rad: vec![],
                feedback_source: FeedbackSource::Software,
            },
        };
        let s = shared_fixture();
        s.feedback_history.lock().unwrap().extend([
            feedback(100, 0.0, 0.0),
            feedback(200, 0.02, std::f64::consts::FRAC_PI_2),
        ]);
        let middle = feedback_at(&s, 150).unwrap();
        assert!((middle.translation.x - 0.01).abs() < 1e-12);
        assert!((middle.rotation.angle() - std::f64::consts::FRAC_PI_4).abs() < 1e-12);
        assert!(feedback_at(&s, 99).is_err());
        assert!(feedback_at(&s, 201).is_err());
        assert_eq!(feedback_at(&s, 100).unwrap().translation.x, 0.0);
        let mut saved = calibration_fixture();
        saved.camera_pose = Some(iso_pose(&nalgebra::Isometry3::translation(0., 0.03, 0.)));
        let rectified = iso_pose(&nalgebra::Isometry3::translation(0., 0., 0.04));
        let external = camera_in_base(&s, Rig::External, &saved, 150, &rectified)
            .unwrap()
            .unwrap();
        let wrist = camera_in_base(&s, Rig::Wrist, &saved, 150, &rectified)
            .unwrap()
            .unwrap();
        assert!(
            (external.translation.vector - nalgebra::Vector3::new(0., 0.03, 0.04)).norm() < 1e-12
        );
        assert!((wrist.to_homogeneous() - (middle * external).to_homogeneous()).norm() < 1e-12);
    }

    #[test]
    fn wrist_capture_waits_for_feedback_without_disconnect_or_losing_the_pair() {
        use crate::drivers::{CameraPoll, CameraStream};
        let mut s = shared_fixture();
        let mut saved = calibration_fixture();
        saved.camera_pose = Some(iso_pose(&nalgebra::Isometry3::identity()));
        for (role, source) in [
            (CameraRole::Wrist, "test:left"),
            (CameraRole::WristRight, "test:right"),
        ] {
            let frame = camera_media::Encoder::new(64, 48, 10.)
                .unwrap()
                .encode(
                    robot_arm_messages::CameraRawVideoFrame {
                        schema_version: SCHEMA_VERSION,
                        sequence: 1,
                        source_id: source.into(),
                        device_time_ns: 100,
                        received_time_ns: 100,
                        color: CameraImagePlane {
                            width: 64,
                            height: 48,
                            stride_bytes: 64 * 3,
                            pixel_format: "rgb8".into(),
                            frame_id: "test".into(),
                            data: vec![128; 64 * 48 * 3],
                        },
                    },
                    true,
                )
                .unwrap();
            s.videos
                .insert(role, tokio::sync::watch::channel(Some(Arc::new(frame))).1);
        }
        let feedback = |time, x| ToolPoseFeedback {
            pose: ToolPose {
                frame: "base_link".into(),
                position_m: [x, 0., 0.],
                orientation_xyzw: [0., 0., 0., 1.],
            },
            arm_state: ArmState {
                schema_version: SCHEMA_VERSION,
                sequence: time as u64,
                sample_time_ns: time,
                model_revision: "timestamp-test".into(),
                joints_rad: vec![],
                actuators_rad: vec![],
                feedback_source: FeedbackSource::Software,
            },
        };
        s.feedback_history
            .lock()
            .unwrap()
            .push_back(feedback(50, 0.));
        s.saved.write().unwrap().insert(Rig::Wrist, saved.clone());
        let s = Arc::new(s);
        let mut stream = capture::Stream::test_stream(s.clone(), Rig::Wrist, saved);
        assert!(matches!(
            stream.poll_frame(true).unwrap(),
            CameraPoll::Pending
        ));
        // Delayed feedback arrives without a new image. Consume the retained pair
        // once, using its interpolated actual pose, not the newer pose directly.
        s.feedback_history
            .lock()
            .unwrap()
            .push_back(feedback(150, 0.02));
        stream.test_next_poll();
        let CameraPoll::Captured {
            frame: Some(frame), ..
        } = stream.poll_frame(true).unwrap()
        else {
            panic!("paired feedback must resume capture")
        };
        assert!((frame.calibration.unwrap().translation_m[0] - 0.01).abs() < 1e-12);
        stream.test_next_poll();
        assert!(matches!(
            stream.poll_frame(true).unwrap(),
            CameraPoll::Pending
        ));
    }
}
