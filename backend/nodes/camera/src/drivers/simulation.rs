use std::{
    sync::{
        Arc, LazyLock, RwLock,
        atomic::{AtomicBool, Ordering},
    },
    time::{Duration, Instant},
};

use eyre::{Context, Result, bail};
use image::imageops::FilterType;
use image::{GrayImage, Rgb, RgbImage, imageops};
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};
use opencv::{core, geometry, imgproc, prelude::*};
use robot_arm_messages::{
    CameraDriverParameterValue, CameraFrameBundle, CameraImagePlane, CameraIntrinsics,
    CameraRawVideoFrame, CameraSourceInfo, CameraStreamKind, CameraStreamProfile,
    DepthCameraCalibration, Pose3, SCHEMA_VERSION, ToolPose,
};
use serde::Deserialize;

use super::{CameraDriver, CameraPoll, CameraStream};

const CONFIG: &str = include_str!("../../assets/simulation-cameras.json");
const PICK_PLACE_RGB: &[u8] = include_bytes!("../../assets/pick-place-scene.png");
const PICK_PLACE_DEPTH: &[u8] = include_bytes!("../../assets/pick-place-scene-depth.png");
const CHARUCO_BOARD: &[u8] = include_bytes!("../../assets/charuco-5x5-sq15-mk11.png");
const CHARUCO_CONFIG: &str = include_str!("../../assets/charuco-board.json");
const SIMULATION_PROFILES: [(u32, u32); 2] = [(1_920, 1_080), (1_280, 720)];

static CHARUCO: LazyLock<CharucoFixture> = LazyLock::new(|| {
    serde_json::from_str(CHARUCO_CONFIG).expect("embedded ChArUco fixture configuration")
});

#[derive(Deserialize)]
struct CharucoFixture {
    pattern: String,
    dictionary: String,
    squares_x: u32,
    squares_y: u32,
    square_size_m: f64,
    marker_size_m: f64,
    measured_width_m: f64,
    measured_height_m: f64,
    image_width_px: u32,
    image_height_px: u32,
    print_margin_px: u32,
    marker_border_bits: u32,
    asset: String,
}

#[derive(Clone, Deserialize)]
struct SimulationConfig {
    source_id: String,
    display_name: String,
    simulation: SimulationKind,
    depth_scale_m: f64,
    calibration: Calibration,
    camera_in_base: Pose3,
    board_in_tool: Pose3,
}

#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum SimulationKind {
    PickPlace,
    DepthGrid,
}

#[derive(Clone, Deserialize)]
struct Calibration {
    frame_id: String,
    width: u32,
    height: u32,
    distortion_model: String,
    distortion: Vec<f64>,
    camera_matrix: [f64; 9],
}

pub(crate) struct SimulationDriver {
    configs: Vec<SimulationConfig>,
    tool_pose: Arc<RwLock<Option<ToolPose>>>,
    calibration_active: Arc<AtomicBool>,
}

impl SimulationDriver {
    pub(crate) fn new() -> Result<Self> {
        validate_charuco_fixture()?;
        Ok(Self {
            configs: serde_json::from_str(CONFIG).context("解析模拟相机配置")?,
            tool_pose: Arc::new(RwLock::new(None)),
            calibration_active: Arc::new(AtomicBool::new(false)),
        })
    }

    pub(crate) fn update_tool_pose(&mut self, pose: ToolPose) {
        *self.tool_pose.write().expect("simulation tool pose lock") = Some(pose);
    }

    pub(crate) fn set_calibration_active(&mut self, active: bool) {
        self.calibration_active.store(active, Ordering::Release);
    }
}

fn validate_charuco_fixture() -> Result<()> {
    if CHARUCO.pattern != "charuco" {
        bail!("模拟标定板 pattern 必须为 charuco");
    }
    if CHARUCO.dictionary != "DICT_4X4_50" {
        bail!("模拟标定板字典与正式打印资产不一致");
    }
    if CHARUCO.asset != "charuco-5x5-sq15-mk11.png" {
        bail!("模拟标定板清单没有指向嵌入的正式资产");
    }
    if CHARUCO.squares_x < 2
        || CHARUCO.squares_y < 2
        || CHARUCO.marker_size_m <= 0.0
        || CHARUCO.marker_size_m >= CHARUCO.square_size_m
    {
        bail!("模拟标定板方格或标记尺寸无效");
    }
    let width_error =
        (CHARUCO.measured_width_m - f64::from(CHARUCO.squares_x) * CHARUCO.square_size_m).abs();
    let height_error =
        (CHARUCO.measured_height_m - f64::from(CHARUCO.squares_y) * CHARUCO.square_size_m).abs();
    if width_error > f64::EPSILON || height_error > f64::EPSILON {
        bail!("模拟标定板实测尺寸与方格定义不一致");
    }
    if CHARUCO.print_margin_px * 2 >= CHARUCO.image_width_px
        || CHARUCO.print_margin_px * 2 >= CHARUCO.image_height_px
        || CHARUCO.marker_border_bits == 0
    {
        bail!("模拟标定板打印边距或标记边框无效");
    }
    let image = image::load_from_memory(CHARUCO_BOARD)
        .context("读取嵌入的模拟标定板")?
        .into_luma8();
    if image.dimensions() != (CHARUCO.image_width_px, CHARUCO.image_height_px) {
        bail!("模拟标定板 PNG 尺寸与清单不一致");
    }
    Ok(())
}

impl CameraDriver for SimulationDriver {
    fn discover(&mut self) -> Result<Vec<CameraSourceInfo>> {
        Ok(self.configs.iter().map(source_info).collect())
    }

    fn open(
        &mut self,
        source_id: &str,
        color_profile: &CameraStreamProfile,
        depth_profile: &CameraStreamProfile,
        driver_parameters: &[CameraDriverParameterValue],
    ) -> Result<Box<dyn CameraStream>> {
        if !driver_parameters.is_empty() {
            bail!("模拟相机没有设备专属驱动参数");
        }
        let Some(mut config) = self
            .configs
            .iter()
            .find(|config| config.source_id == source_id)
            .cloned()
        else {
            bail!("模拟相机 {source_id} 不存在");
        };
        let source = source_info(&config);
        if !source.profiles.iter().any(|value| value == color_profile)
            || !source.profiles.iter().any(|value| value == depth_profile)
        {
            bail!("所选 profile 不是模拟相机实际声明的能力");
        }
        if color_profile.width != depth_profile.width
            || color_profile.height != depth_profile.height
            || color_profile.frames_per_second != depth_profile.frames_per_second
        {
            bail!("模拟相机的彩色流和对齐深度流必须使用相同的尺寸与帧率");
        }
        config.calibration = scaled_calibration(
            &config.calibration,
            color_profile.width,
            color_profile.height,
        );
        let static_color = match config.simulation {
            SimulationKind::PickPlace => image::load_from_memory(PICK_PLACE_RGB)?
                .resize_exact(
                    config.calibration.width,
                    config.calibration.height,
                    FilterType::Lanczos3,
                )
                .to_rgb8()
                .into_raw(),
            SimulationKind::DepthGrid => {
                RgbImage::new(config.calibration.width, config.calibration.height).into_raw()
            }
        };
        let static_depth = simulation_depth(&config)?;
        Ok(Box::new(SimulationStream {
            config,
            sequence: 0,
            frame_period: Duration::from_secs_f64(1.0 / f64::from(color_profile.frames_per_second)),
            next_frame_at: None,
            static_color,
            static_depth,
            tool_pose: Arc::clone(&self.tool_pose),
            calibration_active: Arc::clone(&self.calibration_active),
        }))
    }
}

struct SimulationStream {
    config: SimulationConfig,
    sequence: u64,
    frame_period: Duration,
    next_frame_at: Option<Instant>,
    static_color: Vec<u8>,
    static_depth: Vec<u16>,
    tool_pose: Arc<RwLock<Option<ToolPose>>>,
    calibration_active: Arc<AtomicBool>,
}

impl CameraStream for SimulationStream {
    fn poll_frame(&mut self, materialize: bool) -> Result<CameraPoll> {
        let now = Instant::now();
        if self.next_frame_at.is_some_and(|deadline| now < deadline) {
            return Ok(CameraPoll::Pending);
        }
        let deadline = now + self.frame_period;
        self.next_frame_at = Some(deadline);
        self.sequence += 1;
        let now_ns = now_ns();
        let width = self.config.calibration.width;
        let height = self.config.calibration.height;
        let mut color = self.static_color.clone();
        let mut depth = self.static_depth.clone();
        if self.calibration_active.load(Ordering::Acquire)
            && let Some(tool_pose) = self
                .tool_pose
                .read()
                .expect("simulation tool pose lock")
                .as_ref()
        {
            render_charuco(&self.config, tool_pose, &mut color, &mut depth)?;
        }
        if Instant::now() >= deadline {
            // A slow renderer must not create a permanently overdue timer that
            // starves camera control requests while it tries to catch up.
            self.next_frame_at = Some(Instant::now() + self.frame_period);
        }
        let intrinsics = intrinsics(&self.config.calibration);
        let color = CameraImagePlane {
            width,
            height,
            stride_bytes: width * 3,
            pixel_format: "rgb8".into(),
            frame_id: self.config.calibration.frame_id.clone(),
            data: color,
        };
        let video_frame = Box::new(CameraRawVideoFrame {
            schema_version: SCHEMA_VERSION,
            sequence: self.sequence,
            source_id: self.config.source_id.clone(),
            received_time_ns: now_ns,
            color: color.clone(),
        });
        if !materialize {
            return Ok(CameraPoll::Captured {
                sequence: self.sequence,
                received_time_ns: now_ns,
                video_frame,
                frame: None,
            });
        }
        Ok(CameraPoll::Captured {
            sequence: self.sequence,
            received_time_ns: now_ns,
            video_frame,
            frame: Some(Box::new(CameraFrameBundle {
                schema_version: SCHEMA_VERSION,
                sequence: self.sequence,
                source_id: self.config.source_id.clone(),
                device_time_ns: now_ns,
                device_time_domain: "simulation_clock".into(),
                received_time_ns: now_ns,
                color,
                aligned_depth: CameraImagePlane {
                    width,
                    height,
                    stride_bytes: width * 2,
                    pixel_format: "z16le".into(),
                    frame_id: self.config.calibration.frame_id.clone(),
                    data: depth.iter().flat_map(|value| value.to_le_bytes()).collect(),
                },
                intrinsics,
                depth_scale_m: self.config.depth_scale_m,
                calibration: Some(depth_camera_calibration(
                    &self.config,
                    self.sequence,
                    now_ns,
                )),
            })),
        })
    }
}

fn depth_camera_calibration(
    config: &SimulationConfig,
    sequence: u64,
    source_time_ns: i64,
) -> DepthCameraCalibration {
    let camera_matrix = config.calibration.camera_matrix;
    DepthCameraCalibration {
        schema_version: SCHEMA_VERSION,
        sequence,
        source_time_ns,
        source_id: config.source_id.clone(),
        parent_frame_id: "base_link".into(),
        frame_id: config.calibration.frame_id.clone(),
        translation_m: config.camera_in_base.position_m,
        orientation_xyzw: config.camera_in_base.orientation_xyzw,
        width: config.calibration.width,
        height: config.calibration.height,
        distortion_model: config.calibration.distortion_model.clone(),
        distortion: config.calibration.distortion.clone(),
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
    }
}

fn source_info(config: &SimulationConfig) -> CameraSourceInfo {
    let profiles = SIMULATION_PROFILES
        .into_iter()
        .flat_map(|(width, height)| {
            [
                CameraStreamProfile {
                    key: format!("color:{width}x{height}:rgb8:10"),
                    stream: CameraStreamKind::Color,
                    width,
                    height,
                    frames_per_second: 10,
                    pixel_format: "rgb8".into(),
                    is_default: width == SIMULATION_PROFILES[0].0,
                    available: true,
                    unavailable_reason: None,
                },
                CameraStreamProfile {
                    key: format!("depth:{width}x{height}:z16le:10"),
                    stream: CameraStreamKind::Depth,
                    width,
                    height,
                    frames_per_second: 10,
                    pixel_format: "z16le".into(),
                    is_default: width == SIMULATION_PROFILES[0].0,
                    available: true,
                    unavailable_reason: None,
                },
            ]
        })
        .collect();
    CameraSourceInfo {
        source_id: config.source_id.clone(),
        driver_id: "simulation".into(),
        display_name: config.display_name.clone(),
        device_model: Some("确定性 RGB-D 测试源".into()),
        serial_number: None,
        firmware_version: None,
        connection_type: Some("内置资产".into()),
        physical_port: None,
        sensors: vec!["color".into(), "depth".into()],
        profiles,
        driver_extensions: vec![],
        available: true,
    }
}

fn scaled_calibration(value: &Calibration, width: u32, height: u32) -> Calibration {
    let x_scale = f64::from(width) / f64::from(value.width);
    let y_scale = f64::from(height) / f64::from(value.height);
    let mut scaled = value.clone();
    scaled.width = width;
    scaled.height = height;
    scaled.camera_matrix[0] *= x_scale;
    scaled.camera_matrix[2] = (scaled.camera_matrix[2] + 0.5) * x_scale - 0.5;
    scaled.camera_matrix[4] *= y_scale;
    scaled.camera_matrix[5] = (scaled.camera_matrix[5] + 0.5) * y_scale - 0.5;
    scaled
}

fn intrinsics(value: &Calibration) -> CameraIntrinsics {
    CameraIntrinsics {
        width: value.width,
        height: value.height,
        focal_length_px: [value.camera_matrix[0], value.camera_matrix[4]],
        principal_point_px: [value.camera_matrix[2], value.camera_matrix[5]],
        distortion_model: value.distortion_model.clone(),
        distortion: value.distortion.clone(),
    }
}

fn simulation_depth(config: &SimulationConfig) -> Result<Vec<u16>> {
    let width = config.calibration.width;
    let height = config.calibration.height;
    let mut values = vec![0; (width * height) as usize];
    match config.simulation {
        SimulationKind::PickPlace => {
            let rendered = image::load_from_memory(PICK_PLACE_DEPTH)?
                .resize_exact(width, height, FilterType::Nearest)
                .into_luma16();
            return Ok(rendered.into_raw());
        }
        SimulationKind::DepthGrid => {
            fill_relative_rect(&mut values, width, height, [310, 210, 330, 230], 400);
            fill_relative_rect(&mut values, width, height, [350, 210, 356, 217], 380);
        }
    }
    Ok(values)
}

fn fill_relative_rect(
    values: &mut [u16],
    width: u32,
    height: u32,
    reference_xyxy: [u32; 4],
    depth: u16,
) {
    let [left, top, right, bottom] = reference_xyxy;
    let scale = |coordinate: u32, dimension: u32, reference: u32| {
        ((u64::from(coordinate) * u64::from(dimension.saturating_sub(1)))
            / u64::from(reference - 1)) as u32
    };
    fill_rect(
        values,
        width,
        scale(left, width, 640),
        scale(top, height, 480),
        scale(right, width, 640),
        scale(bottom, height, 480),
        depth,
    );
}

fn fill_rect(
    values: &mut [u16],
    width: u32,
    left: u32,
    top: u32,
    right: u32,
    bottom: u32,
    depth: u16,
) {
    for y in top..=bottom {
        for x in left..=right {
            values[(y * width + x) as usize] = depth;
        }
    }
}

fn render_charuco(
    config: &SimulationConfig,
    tool: &ToolPose,
    color: &mut Vec<u8>,
    depth: &mut [u16],
) -> Result<()> {
    static BOARD: LazyLock<GrayImage> = LazyLock::new(|| {
        let image = image::load_from_memory(CHARUCO_BOARD)
            .expect("embedded ChArUco board")
            .into_luma8();
        crop_print_margin(&image)
    });
    let camera_in_base = isometry(&config.camera_in_base);
    let tool_in_base = Isometry3::from_parts(
        Translation3::from(Vector3::from(tool.position_m)),
        UnitQuaternion::from_quaternion(Quaternion::new(
            tool.orientation_xyzw[3],
            tool.orientation_xyzw[0],
            tool.orientation_xyzw[1],
            tool.orientation_xyzw[2],
        )),
    );
    let board_in_tool = isometry(&config.board_in_tool);
    let board_in_camera = camera_in_base.inverse() * tool_in_base * board_in_tool;
    // OpenCV's ChArUco object frame starts at the board's top-left outer
    // corner. Keep the simulation asset in that same frame so the solved
    // board-in-tool transform can be compared directly with its oracle.
    let width = CHARUCO.measured_width_m;
    let height = CHARUCO.measured_height_m;
    let corners = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(width, 0.0, 0.0),
        Point3::new(width, height, 0.0),
        Point3::new(0.0, height, 0.0),
    ];
    let mut projected = [[0.0; 2]; 4];
    for (index, corner) in corners.into_iter().enumerate() {
        let point = board_in_camera * corner;
        if point.z <= 0.0 {
            return Ok(());
        }
        projected[index] = [
            config.calibration.camera_matrix[0] * point.x / point.z
                + config.calibration.camera_matrix[2],
            config.calibration.camera_matrix[4] * point.y / point.z
                + config.calibration.camera_matrix[5],
        ];
    }
    let left = projected
        .iter()
        .map(|p| p[0])
        .fold(f64::INFINITY, f64::min)
        .floor()
        .max(0.0) as u32;
    let top = projected
        .iter()
        .map(|p| p[1])
        .fold(f64::INFINITY, f64::min)
        .floor()
        .max(0.0) as u32;
    let right = projected
        .iter()
        .map(|p| p[0])
        .fold(f64::NEG_INFINITY, f64::max)
        .ceil()
        .min(f64::from(config.calibration.width - 1)) as u32;
    let bottom = projected
        .iter()
        .map(|p| p[1])
        .fold(f64::NEG_INFINITY, f64::max)
        .ceil()
        .min(f64::from(config.calibration.height - 1)) as u32;
    if left >= right || top >= bottom {
        return Ok(());
    }
    let texture = warp_board_texture(&BOARD, projected, [left, top, right, bottom])?;
    let mut image = RgbImage::from_raw(
        config.calibration.width,
        config.calibration.height,
        std::mem::take(color),
    )
    .ok_or_else(|| eyre::eyre!("模拟彩色图尺寸错误"))?;
    let board_origin = board_in_camera * Point3::origin();
    let board_normal = board_in_camera.rotation * Vector3::z();
    let plane_distance = board_normal.dot(&board_origin.coords);
    let camera_in_board = board_in_camera.inverse();
    for y in top..=bottom {
        for x in left..=right {
            let start = (((y - top) * (right - left + 1) + x - left) * 4) as usize;
            let rgba = &texture[start..start + 4];
            if rgba[3] == 0 {
                continue;
            }
            // The RGB warp and metric depth must describe the same
            // tilted physical plane. Intersect this optical pixel ray with
            // the board instead of assigning one average depth to the
            // complete quadrilateral.
            let ray = Vector3::new(
                (f64::from(x) - config.calibration.camera_matrix[2])
                    / config.calibration.camera_matrix[0],
                (f64::from(y) - config.calibration.camera_matrix[5])
                    / config.calibration.camera_matrix[4],
                1.0,
            );
            let denominator = board_normal.dot(&ray);
            if denominator.abs() < f64::EPSILON {
                continue;
            }
            let metric_depth = plane_distance / denominator;
            if metric_depth <= 0.0 {
                continue;
            }
            let raw_depth = (metric_depth / config.depth_scale_m)
                .round()
                .clamp(1.0, f64::from(u16::MAX)) as u16;
            let offset = (y * config.calibration.width + x) as usize;
            if depth[offset] != 0 && depth[offset] <= raw_depth {
                continue;
            }
            // Warping transparent black outside the board produces
            // premultiplied colour: do not multiply it by alpha twice.
            let background = image.get_pixel(x, y);
            let background_weight = 1.0 - f64::from(rgba[3]) / 255.0;
            let blended = std::array::from_fn(|channel| {
                (f64::from(rgba[channel]) + f64::from(background[channel]) * background_weight)
                    .round()
                    .min(255.0) as u8
            });
            image.put_pixel(x, y, Rgb(blended));
            let board_point = camera_in_board * Point3::from(ray * metric_depth);
            // RGB integrates coverage; Z16, like the scene Z-buffer,
            // describes the centre ray, not a fractional edge sample.
            if (0.0..=width).contains(&board_point.x) && (0.0..=height).contains(&board_point.y) {
                depth[offset] = raw_depth;
            }
        }
    }
    *color = image.into_raw();
    Ok(())
}

fn crop_print_margin(image: &GrayImage) -> GrayImage {
    let mut bounds: Option<(u32, u32, u32, u32)> = None;
    for (x, y, pixel) in image.enumerate_pixels() {
        if pixel[0] >= 128 {
            continue;
        }
        bounds = Some(match bounds {
            None => (x, y, x, y),
            Some((left, top, right, bottom)) => {
                (left.min(x), top.min(y), right.max(x), bottom.max(y))
            }
        });
    }
    let Some((left, top, right, bottom)) = bounds else {
        return image.clone();
    };
    imageops::crop_imm(image, left, top, right - left + 1, bottom - top + 1).to_image()
}

fn warp_board_texture(
    board: &GrayImage,
    projected: [[f64; 2]; 4],
    [left, top, right, bottom]: [u32; 4],
) -> Result<Vec<u8>> {
    // OpenCV performs the perspective warp and pixel-area reduction. Only the
    // board ROI is supersampled, not the entire camera frame. Eight samples per
    // axis are rendering quality, not a limit on camera/robot operation.
    const SCALE: i32 = 8;
    let width = (right - left + 1) as i32;
    let height = (bottom - top + 1) as i32;
    let source = [
        [-0.5, -0.5],
        [f64::from(board.width()) - 0.5, -0.5],
        [
            f64::from(board.width()) - 0.5,
            f64::from(board.height()) - 0.5,
        ],
        [-0.5, f64::from(board.height()) - 0.5],
    ]
    .map(|[x, y]| core::Point2f::new(x as f32, y as f32));
    // Physical board edges map to texture pixel edges, not first/last centres.
    let target = projected.map(|[x, y]| {
        core::Point2f::new(
            ((x - f64::from(left) + 0.5) * f64::from(SCALE) - 0.5) as f32,
            ((y - f64::from(top) + 0.5) * f64::from(SCALE) - 0.5) as f32,
        )
    });
    let transform = geometry::get_perspective_transform_slice_def(source, target)?;
    let bytes = core::Mat::from_slice(board.as_raw())?;
    let gray = bytes.reshape(1, board.height() as i32)?;
    let mut rgba = core::Mat::default();
    imgproc::cvt_color_def(&gray, &mut rgba, imgproc::COLOR_GRAY2RGBA)?;
    let mut warped = core::Mat::default();
    imgproc::warp_perspective(
        &rgba,
        &mut warped,
        &transform,
        core::Size::new(width * SCALE, height * SCALE),
        imgproc::INTER_LINEAR,
        core::BORDER_CONSTANT,
        core::Scalar::all(0.0),
        core::AlgorithmHint::ALGO_HINT_DEFAULT,
    )?;
    let mut pixels = core::Mat::default();
    imgproc::resize(
        &warped,
        &mut pixels,
        core::Size::new(width, height),
        0.0,
        0.0,
        imgproc::INTER_AREA,
    )?;
    Ok(pixels.data_bytes()?.to_vec())
}

fn isometry(pose: &Pose3) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::from(Vector3::from(pose.position_m)),
        UnitQuaternion::from_quaternion(Quaternion::new(
            pose.orientation_xyzw[3],
            pose.orientation_xyzw[0],
            pose.orientation_xyzw[1],
            pose.orientation_xyzw[2],
        )),
    )
}

fn now_ns() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as i64
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tool_pose_for_board(config: &SimulationConfig, board_in_camera: Isometry3<f64>) -> ToolPose {
        let tool = isometry(&config.camera_in_base)
            * board_in_camera
            * isometry(&config.board_in_tool).inverse();
        let q = tool.rotation.quaternion();
        ToolPose {
            frame: "base_link".into(),
            position_m: tool.translation.vector.into(),
            orientation_xyzw: [q.i, q.j, q.k, q.w],
        }
    }

    fn materialized(stream: &mut dyn CameraStream) -> CameraFrameBundle {
        match stream.poll_frame(true).unwrap() {
            CameraPoll::Captured {
                frame: Some(frame), ..
            } => *frame,
            _ => panic!("expected a materialized simulation frame"),
        }
    }

    #[test]
    fn replay_is_declared_and_deterministic() {
        let mut driver = SimulationDriver::new().unwrap();
        let sources = driver.discover().unwrap();
        assert_eq!(sources.len(), 2);
        let source = &sources[0];
        let color = source
            .profiles
            .iter()
            .find(|profile| profile.stream == CameraStreamKind::Color)
            .unwrap();
        let depth = source
            .profiles
            .iter()
            .find(|profile| profile.stream == CameraStreamKind::Depth)
            .unwrap();
        let mut stream = driver.open(&source.source_id, color, depth, &[]).unwrap();
        let first = materialized(stream.as_mut());
        std::thread::sleep(Duration::from_millis(110));
        let second = materialized(stream.as_mut());
        assert_eq!(second.sequence, first.sequence + 1);
        assert_eq!(first.color.data, second.color.data);
        assert_eq!(first.aligned_depth.data, second.aligned_depth.data);
        assert_eq!(
            first.calibration.as_ref().unwrap().source_id,
            source.source_id
        );
        assert_eq!((first.color.width, first.color.height), (1_920, 1_080));
        assert_eq!(
            first.calibration.as_ref().unwrap().camera_matrix[0],
            driver.configs[0].calibration.camera_matrix[0] * 1920.0
                / f64::from(driver.configs[0].calibration.width)
        );
    }

    #[test]
    fn simulation_declares_both_common_rgbd_resolutions() {
        let mut driver = SimulationDriver::new().unwrap();
        let source = driver.discover().unwrap().remove(0);
        for (width, height) in SIMULATION_PROFILES {
            for stream in [CameraStreamKind::Color, CameraStreamKind::Depth] {
                assert!(source.profiles.iter().any(|profile| {
                    profile.stream == stream
                        && profile.width == width
                        && profile.height == height
                        && profile.available
                }));
            }
        }
        assert_eq!(
            source
                .profiles
                .iter()
                .filter(|profile| profile.is_default)
                .count(),
            2
        );
    }

    #[test]
    fn skipped_frame_is_not_materialized() {
        let mut driver = SimulationDriver::new().unwrap();
        let source = driver.discover().unwrap().remove(0);
        let color = source
            .profiles
            .iter()
            .find(|profile| profile.stream == CameraStreamKind::Color)
            .unwrap();
        let depth = source
            .profiles
            .iter()
            .find(|profile| profile.stream == CameraStreamKind::Depth)
            .unwrap();
        let mut stream = driver.open(&source.source_id, color, depth, &[]).unwrap();
        assert!(matches!(
            stream.poll_frame(false).unwrap(),
            CameraPoll::Captured { frame: None, .. }
        ));
    }

    #[test]
    fn pick_place_uses_the_rendered_metric_depth_asset() {
        let driver = SimulationDriver::new().unwrap();
        let config = &driver.configs[0];
        let depth = simulation_depth(config).unwrap();
        assert_eq!(
            depth.len(),
            (config.calibration.width * config.calibration.height) as usize
        );
        let nonzero: Vec<_> = depth.iter().copied().filter(|value| *value > 0).collect();
        assert_eq!(nonzero.len(), depth.len());
        assert_ne!(nonzero.iter().min(), nonzero.iter().max());
    }

    #[test]
    fn depth_grid_scales_with_the_declared_profile() {
        let driver = SimulationDriver::new().unwrap();
        let template = driver
            .configs
            .iter()
            .find(|config| matches!(config.simulation, SimulationKind::DepthGrid))
            .unwrap();
        for (width, height) in [(1_280, 720), (1_920, 1_080)] {
            let mut config = template.clone();
            config.calibration.width = width;
            config.calibration.height = height;
            let depth = simulation_depth(&config).unwrap();
            assert_eq!(depth.len(), (width * height) as usize);
            assert!(depth.contains(&400));
            assert!(depth.contains(&380));
        }
    }

    #[test]
    fn charuco_is_only_rendered_during_calibration() {
        let mut driver = SimulationDriver::new().unwrap();
        let source = driver.discover().unwrap().remove(0);
        let color = source
            .profiles
            .iter()
            .find(|profile| profile.stream == CameraStreamKind::Color)
            .unwrap()
            .clone();
        let depth = source
            .profiles
            .iter()
            .find(|profile| profile.stream == CameraStreamKind::Depth)
            .unwrap()
            .clone();
        let mut baseline = driver.open(&source.source_id, &color, &depth, &[]).unwrap();
        let without_tool = materialized(baseline.as_mut());
        drop(baseline);

        // Put the board 50 cm along the selected camera's optical axis. This is
        // deliberately expressed as a normal ToolPose, exactly as at runtime.
        let camera_in_base = isometry(&driver.configs[0].camera_in_base);
        let board_in_base = camera_in_base * Isometry3::translation(0.0, 0.0, 0.5);
        let tool_in_base = board_in_base * isometry(&driver.configs[0].board_in_tool).inverse();
        let q = tool_in_base.rotation.quaternion();
        driver.update_tool_pose(ToolPose {
            frame: "base_link".into(),
            position_m: tool_in_base.translation.vector.into(),
            orientation_xyzw: [q.i, q.j, q.k, q.w],
        });
        let mut inactive = driver.open(&source.source_id, &color, &depth, &[]).unwrap();
        let without_calibration = materialized(inactive.as_mut());
        assert_eq!(without_tool.color.data, without_calibration.color.data);
        assert_eq!(
            without_tool.aligned_depth.data,
            without_calibration.aligned_depth.data
        );
        drop(inactive);

        driver.set_calibration_active(true);
        let mut rendered = driver.open(&source.source_id, &color, &depth, &[]).unwrap();
        let with_tool = materialized(rendered.as_mut());

        assert_ne!(without_tool.color.data, with_tool.color.data);
        assert_ne!(
            without_tool.aligned_depth.data,
            with_tool.aligned_depth.data
        );
    }

    #[test]
    fn printable_margin_is_not_part_of_the_physical_board() {
        let board = image::load_from_memory(CHARUCO_BOARD).unwrap().into_luma8();
        assert_eq!(
            board.dimensions(),
            (CHARUCO.image_width_px, CHARUCO.image_height_px)
        );
        assert_eq!(CHARUCO.asset, "charuco-5x5-sq15-mk11.png");
        assert_eq!(CHARUCO.print_margin_px, 40);
        assert_eq!(CHARUCO.marker_border_bits, 1);
        assert_eq!(
            CHARUCO.measured_width_m,
            f64::from(CHARUCO.squares_x) * CHARUCO.square_size_m
        );
        assert_eq!(
            CHARUCO.measured_height_m,
            f64::from(CHARUCO.squares_y) * CHARUCO.square_size_m
        );
        let cropped = crop_print_margin(&board);
        assert!(cropped.width() < board.width());
        assert!(cropped.height() < board.height());
        assert_eq!(cropped.get_pixel(0, 0)[0], 0);
    }

    #[test]
    fn tilted_charuco_rgb_and_depth_describe_the_same_plane() {
        let driver = SimulationDriver::new().unwrap();
        let config = &driver.configs[0];
        let board_in_camera = Isometry3::from_parts(
            Translation3::new(-0.035, -0.035, 0.52),
            UnitQuaternion::from_euler_angles(0.24, -0.19, 0.08),
        );
        let tool = tool_pose_for_board(config, board_in_camera);
        let mut color =
            vec![127; (config.calibration.width * config.calibration.height * 3) as usize];
        let mut depth = vec![0; (config.calibration.width * config.calibration.height) as usize];
        render_charuco(config, &tool, &mut color, &mut depth).unwrap();

        let changed = color
            .chunks_exact(3)
            .enumerate()
            .filter_map(|(index, pixel)| (pixel != [127, 127, 127]).then_some(index))
            .collect::<Vec<_>>();
        assert!(changed.len() > 4_000);
        // Antialiased RGB boundary pixels can partly cover the board while
        // their centre rays miss it. Z16 represents the centre ray only.
        let plane_pixels = depth
            .iter()
            .enumerate()
            .filter_map(|(index, raw)| (*raw > 0).then_some(index))
            .collect::<Vec<_>>();
        let minimum_depth = plane_pixels
            .iter()
            .map(|index| depth[*index])
            .min()
            .unwrap();
        let maximum_depth = plane_pixels
            .iter()
            .map(|index| depth[*index])
            .max()
            .unwrap();
        assert!(maximum_depth - minimum_depth > 15);

        let inverse = board_in_camera.inverse();
        let mut maximum_plane_error = 0.0_f64;
        for index in plane_pixels.iter().step_by(17) {
            let x = (*index as u32) % config.calibration.width;
            let y = (*index as u32) / config.calibration.width;
            let z = f64::from(depth[*index]) * config.depth_scale_m;
            let point = Point3::new(
                (f64::from(x) - config.calibration.camera_matrix[2]) * z
                    / config.calibration.camera_matrix[0],
                (f64::from(y) - config.calibration.camera_matrix[5]) * z
                    / config.calibration.camera_matrix[4],
                z,
            );
            let local = inverse * point;
            maximum_plane_error = maximum_plane_error.max(local.z.abs());
            assert!((-0.002..=0.077).contains(&local.x));
            assert!((-0.002..=0.077).contains(&local.y));
        }
        assert!(maximum_plane_error <= 0.0006);
    }

    #[test]
    fn charuco_respects_existing_nearer_depth() {
        let driver = SimulationDriver::new().unwrap();
        let config = &driver.configs[0];
        let board_in_camera = Isometry3::translation(-0.0375, -0.0375, 0.5);
        let tool = tool_pose_for_board(config, board_in_camera);
        let original =
            vec![127; (config.calibration.width * config.calibration.height * 3) as usize];
        let mut color = original.clone();
        let mut depth = vec![400; (config.calibration.width * config.calibration.height) as usize];
        render_charuco(config, &tool, &mut color, &mut depth).unwrap();
        assert_eq!(color, original);
        assert!(depth.iter().all(|value| *value == 400));
    }

    #[cfg(feature = "opencv-runtime")]
    #[test]
    fn charuco_render_detection_and_hand_eye_solution_match_oracle() {
        use std::{fs, io::Cursor, path::PathBuf};

        use image::{DynamicImage, ImageFormat};
        use robot_arm_messages::CalibrationBoard;
        use serde_json::json;

        let driver = SimulationDriver::new().unwrap();
        let assets = std::env::var_os("ROBOT_ARM_TEST_SIMULATION_ASSETS").map(PathBuf::from);
        let read_asset = |name: &str, embedded: &[u8]| {
            assets.as_ref().map_or_else(
                || embedded.to_vec(),
                |dir| fs::read(dir.join(name)).unwrap(),
            )
        };
        let mut config = if let Some(dir) = &assets {
            serde_json::from_slice::<Vec<SimulationConfig>>(
                &fs::read(dir.join("simulation-cameras.json")).unwrap(),
            )
            .unwrap()
            .remove(0)
        } else {
            driver.configs[0].clone()
        };
        if let Ok(width) = std::env::var("ROBOT_ARM_TEST_IMAGE_WIDTH") {
            let width: u32 = width.parse().unwrap();
            let height = config.calibration.height * width / config.calibration.width;
            config.calibration = scaled_calibration(&config.calibration, width, height);
        }
        let config = &config;
        let background =
            image::load_from_memory(&read_asset("pick-place-scene.png", PICK_PLACE_RGB))
                .unwrap()
                .resize_exact(
                    config.calibration.width,
                    config.calibration.height,
                    FilterType::Lanczos3,
                )
                .to_rgb8()
                .into_raw();
        let background_depth =
            image::load_from_memory(&read_asset("pick-place-scene-depth.png", PICK_PLACE_DEPTH))
                .unwrap()
                .resize_exact(
                    config.calibration.width,
                    config.calibration.height,
                    FilterType::Nearest,
                )
                .into_luma16()
                .into_raw();
        // Replay recorded TCP feedback through the same renderer/detector. The
        // close-up synthetic poses below alone do not exercise the distances
        // and viewing angles used by automatic arm calibration.
        let recorded_boards = std::env::var_os("ROBOT_ARM_TEST_CALIBRATION_SESSION").map(|path| {
            let session: serde_json::Value =
                serde_json::from_slice(&fs::read(path).unwrap()).unwrap();
            session
                .get("observations")
                .unwrap_or(&session["rounds"][0]["samples"])
                .as_array()
                .unwrap()
                .iter()
                .map(|observation| {
                    let tool: Pose3 =
                        serde_json::from_value(observation["tcp_in_base"].clone()).unwrap();
                    isometry(&config.camera_in_base).inverse()
                        * isometry(&tool)
                        * isometry(&config.board_in_tool)
                })
                .collect::<Vec<_>>()
        });
        let calibration_board = CalibrationBoard {
            pattern: CHARUCO.pattern.clone(),
            dictionary: CHARUCO.dictionary.clone(),
            squares_x: CHARUCO.squares_x,
            squares_y: CHARUCO.squares_y,
            square_size_m: CHARUCO.square_size_m,
            marker_size_m: CHARUCO.marker_size_m,
            measured_width_m: CHARUCO.measured_width_m,
            measured_height_m: CHARUCO.measured_height_m,
        };
        let samples = [
            ([-0.010, -0.010, 0.17], [0.5, 0.0, 0.0]),
            (
                [0.00, -0.010, 0.19],
                [0.5, 0.0, std::f64::consts::TAU / 3.0],
            ),
            (
                [0.010, -0.010, 0.18],
                [0.5, 0.0, -std::f64::consts::TAU / 3.0],
            ),
            ([-0.010, 0.00, 0.19], [0.0, 0.5, 0.0]),
            ([0.00, 0.00, 0.17], [0.0, 0.5, std::f64::consts::TAU / 3.0]),
            (
                [0.010, 0.00, 0.18],
                [0.0, 0.5, -std::f64::consts::TAU / 3.0],
            ),
            ([-0.010, 0.010, 0.18], [-0.5, 0.0, 0.0]),
            (
                [0.00, 0.010, 0.19],
                [-0.5, 0.0, std::f64::consts::TAU / 3.0],
            ),
            (
                [0.010, 0.010, 0.17],
                [-0.5, 0.0, -std::f64::consts::TAU / 3.0],
            ),
        ];
        let expected_camera = isometry(&config.camera_in_base);
        let expected_board = isometry(&config.board_in_tool);
        let round_offsets = [
            ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            ([0.004, -0.003, 0.008], [0.012, -0.008, 0.006]),
            ([-0.003, 0.004, -0.006], [-0.009, 0.011, -0.005]),
        ];
        let artifact_root = std::env::var_os("ROBOT_ARM_TEST_ARTIFACTS").map(PathBuf::from);
        if let Some(root) = &artifact_root {
            fs::create_dir_all(root).unwrap();
        }
        let mut detection_sheet = RgbImage::new(426 * 3, 240 * 3);
        let mut round_reports = Vec::new();
        let mut checks_passed = true;
        for (round, (center_offset, angle_offset)) in round_offsets.into_iter().enumerate() {
            if recorded_boards.is_some() && round != 0 {
                break;
            }
            let mut tools = Vec::new();
            let mut detections = Vec::new();
            let mut sample_reports = Vec::new();
            let mut maximum_translation_error = 0.0_f64;
            let mut maximum_rotation_error = 0.0_f64;
            let mut maximum_reprojection_rmse = 0.0_f64;
            let mut minimum_corner_count = usize::MAX;
            for (sample, (center, angles)) in samples.into_iter().enumerate() {
                let center = Vector3::from(center) + Vector3::from(center_offset);
                let angles = Vector3::from(angles) + Vector3::from(angle_offset);
                let rotation = UnitQuaternion::from_euler_angles(angles.x, angles.y, angles.z);
                let origin = center - rotation * Vector3::new(0.0375, 0.0375, 0.0);
                let board_in_camera = recorded_boards.as_ref().map_or_else(
                    || Isometry3::from_parts(Translation3::from(origin), rotation),
                    |boards| boards[sample],
                );
                let tool = tool_pose_for_board(config, board_in_camera);
                let mut color = background.clone();
                let mut depth = background_depth.clone();
                render_charuco(config, &tool, &mut color, &mut depth).unwrap();
                let image =
                    RgbImage::from_raw(config.calibration.width, config.calibration.height, color)
                        .unwrap();
                let mut png = Cursor::new(Vec::new());
                DynamicImage::ImageRgb8(image)
                    .write_to(&mut png, ImageFormat::Png)
                    .unwrap();
                let detection = camera_calibration::detect(
                    png.get_ref(),
                    &config.calibration.camera_matrix,
                    &config.calibration.distortion,
                    &calibration_board,
                )
                .unwrap();
                if let Some(root) = &artifact_root {
                    let pose_dir = root.join(format!("round-{}-pose-{}", round + 1, sample + 1));
                    fs::create_dir_all(&pose_dir).unwrap();
                    fs::write(pose_dir.join("input.png"), png.get_ref()).unwrap();
                    fs::write(pose_dir.join("detected.png"), &detection.visualization_png).unwrap();
                    image::ImageBuffer::<image::Luma<u16>, _>::from_raw(
                        config.calibration.width,
                        config.calibration.height,
                        depth.clone(),
                    )
                    .unwrap()
                    .save(pose_dir.join("depth.png"))
                    .unwrap();
                }
                if artifact_root.is_some() && round == 0 {
                    let thumbnail = image::load_from_memory(&detection.visualization_png)
                        .unwrap()
                        .resize_exact(426, 240, FilterType::Lanczos3)
                        .to_rgb8();
                    imageops::replace(
                        &mut detection_sheet,
                        &thumbnail,
                        i64::try_from(sample % 3).unwrap() * 426,
                        i64::try_from(sample / 3).unwrap() * 240,
                    );
                }
                let observed = isometry(&detection.board_in_camera);
                let translation_error =
                    (observed.translation.vector - board_in_camera.translation.vector).norm();
                let rotation_error =
                    (observed.rotation.inverse() * board_in_camera.rotation).angle();
                eprintln!(
                    "round {} sample: {} corners / {:.3} px; oracle error {:.3} mm / {:.3} deg",
                    round + 1,
                    detection.matched_corner_count,
                    detection.reprojection_rmse_px,
                    translation_error * 1000.0,
                    rotation_error.to_degrees()
                );
                minimum_corner_count = minimum_corner_count.min(detection.matched_corner_count);
                maximum_reprojection_rmse =
                    maximum_reprojection_rmse.max(detection.reprojection_rmse_px);
                maximum_translation_error = maximum_translation_error.max(translation_error);
                maximum_rotation_error = maximum_rotation_error.max(rotation_error);
                sample_reports.push(json!({
                    "pose": sample + 1,
                    "board_distance_m": board_in_camera.translation.vector.z,
                    "board_tilt_deg": (board_in_camera.rotation * Vector3::z()).z.clamp(-1.0, 1.0).acos().to_degrees(),
                    "board_in_camera": detection.board_in_camera,
                    "tcp_in_base": {
                        "position_m": tool.position_m,
                        "orientation_xyzw": tool.orientation_xyzw,
                    },
                    "matched_corners": detection.matched_corner_count,
                    "reprojection_rmse_px": detection.reprojection_rmse_px,
                    "translation_error_mm": translation_error * 1000.0,
                    "rotation_error_deg": rotation_error.to_degrees(),
                }));
                checks_passed &= translation_error <= 0.0001;
                checks_passed &= rotation_error <= 3.0_f64.to_radians();
                tools.push(Pose3 {
                    position_m: tool.position_m,
                    orientation_xyzw: tool.orientation_xyzw,
                });
                detections.push(detection.board_in_camera);
            }
            checks_passed &= minimum_corner_count
                == ((CHARUCO.squares_x - 1) * (CHARUCO.squares_y - 1)) as usize;
            checks_passed &= maximum_reprojection_rmse <= 0.5;
            let solution = camera_calibration::solve(&tools, &detections).unwrap();
            let solved_camera = isometry(&solution.camera_in_base);
            let solved_board = isometry(&solution.board_in_calibration_tool);
            let camera_translation_error =
                (solved_camera.translation.vector - expected_camera.translation.vector).norm();
            let camera_rotation_error =
                (solved_camera.rotation.inverse() * expected_camera.rotation).angle();
            let board_translation_error =
                (solved_board.translation.vector - expected_board.translation.vector).norm();
            let board_rotation_error =
                (solved_board.rotation.inverse() * expected_board.rotation).angle();
            // Isolate the effect of extrinsic error over every valid scene
            // depth pixel, not just the camera translation or board centre.
            // Both transforms consume the same Z16 sample, so depth
            // quantization is not mislabeled as calibration error.
            let rotation_error_matrix = solved_camera.rotation.to_rotation_matrix().matrix()
                - expected_camera.rotation.to_rotation_matrix().matrix();
            let translation_error_vector =
                solved_camera.translation.vector - expected_camera.translation.vector;
            let [fx, _, cx, _, fy, cy, _, _, _] = config.calibration.camera_matrix;
            let mut scene_error_squared = 0.0_f64;
            for (index, raw) in background_depth.iter().copied().enumerate() {
                if raw == 0 {
                    continue;
                }
                let z = f64::from(raw) * config.depth_scale_m;
                let point = Vector3::new(
                    ((index as u32 % config.calibration.width) as f64 - cx) * z / fx,
                    ((index as u32 / config.calibration.width) as f64 - cy) * z / fy,
                    z,
                );
                scene_error_squared = scene_error_squared
                    .max((rotation_error_matrix * point + translation_error_vector).norm_squared());
            }
            let maximum_scene_extrinsic_error = scene_error_squared.sqrt();
            let maximum_translation_residual = solution
                .translation_residuals_m
                .iter()
                .copied()
                .fold(0.0_f64, f64::max);
            let maximum_rotation_residual = solution
                .rotation_residuals_rad
                .iter()
                .copied()
                .fold(0.0_f64, f64::max);
            eprintln!(
                "round {} maximum detection error: {:.3} mm / {:.3} deg / {:.3} px; camera error: {:.3} mm / {:.3} deg; board error: {:.3} mm / {:.3} deg",
                round + 1,
                maximum_translation_error * 1000.0,
                maximum_rotation_error.to_degrees(),
                maximum_reprojection_rmse,
                camera_translation_error * 1000.0,
                camera_rotation_error.to_degrees(),
                board_translation_error * 1000.0,
                board_rotation_error.to_degrees()
            );
            round_reports.push(json!({
                "round": round + 1,
                "samples": sample_reports,
                "maximum_detection_translation_error_mm": maximum_translation_error * 1000.0,
                "maximum_detection_rotation_error_deg": maximum_rotation_error.to_degrees(),
                "maximum_reprojection_rmse_px": maximum_reprojection_rmse,
                "camera_translation_error_mm": camera_translation_error * 1000.0,
                "camera_in_base": solution.camera_in_base,
                "camera_rotation_error_deg": camera_rotation_error.to_degrees(),
                "maximum_scene_extrinsic_error_mm": maximum_scene_extrinsic_error * 1000.0,
                "board_translation_error_mm": board_translation_error * 1000.0,
                "board_rotation_error_deg": board_rotation_error.to_degrees(),
                "maximum_translation_residual_mm": maximum_translation_residual * 1000.0,
                "maximum_rotation_residual_deg": maximum_rotation_residual.to_degrees(),
                "solver": solution.solver,
            }));
            checks_passed &= camera_translation_error <= 0.0001;
            checks_passed &= maximum_scene_extrinsic_error <= 0.0001;
            checks_passed &= camera_rotation_error <= 0.035;
            checks_passed &= board_translation_error <= 0.0001;
            checks_passed &= board_rotation_error <= 0.035;
            checks_passed &= solution
                .translation_residuals_m
                .iter()
                .all(|value| *value <= 0.006);
            checks_passed &= solution
                .rotation_residuals_rad
                .iter()
                .all(|value| *value <= 0.035);
        }
        if let Some(root) = &artifact_root {
            detection_sheet
                .save(root.join("charuco-detection-poses.png"))
                .unwrap();
            let report = json!({
                "image": {
                    "width": config.calibration.width,
                    "height": config.calibration.height,
                    "camera_matrix": config.calibration.camera_matrix,
                    "distortion": config.calibration.distortion,
                },
                "board": calibration_board,
                "expected_camera_in_base": config.camera_in_base,
                "expected_board_in_tool": config.board_in_tool,
                "depth_scale_m": config.depth_scale_m,
                "translation_acceptance_mm": 0.1,
                "pose_source": if recorded_boards.is_some() { "TCP pose replay (input file determines provenance)" } else { "close-up synthetic poses" },
                "pose_file": std::env::var_os("ROBOT_ARM_TEST_CALIBRATION_SESSION").map(PathBuf::from),
                "asset_directory": assets,
                "checks_passed": checks_passed,
                "rounds": round_reports,
            });
            fs::write(
                root.join("charuco-validation.json"),
                serde_json::to_vec_pretty(&report).unwrap(),
            )
            .unwrap();
        }
        assert!(
            checks_passed,
            "ChArUco oracle checks failed; inspect the saved per-pose report"
        );
    }
}
