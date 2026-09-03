use std::{
    sync::{Arc, LazyLock, RwLock},
    time::{Duration, Instant},
};

use eyre::{Context, Result, bail};
use image::imageops::FilterType;
use image::{GrayImage, Rgb, RgbImage, imageops};
use nalgebra::{
    Isometry3, Matrix3, Point3, Quaternion, SMatrix, SVector, Translation3, UnitQuaternion, Vector3,
};
use robot_arm_messages::{
    CameraDriverParameterValue, CameraFrameBundle, CameraImagePlane, CameraIntrinsics,
    CameraSourceInfo, CameraStreamKind, CameraStreamProfile, Pose3, SCHEMA_VERSION, ToolPose,
};
use serde::Deserialize;

use super::{CameraDriver, CameraStream};

const CONFIG: &str = include_str!("../../assets/simulation-cameras.json");
const PICK_PLACE_RGB: &[u8] = include_bytes!("../../assets/pick-place-scene.png");
const CHARUCO_BOARD: &[u8] = include_bytes!("../../assets/charuco-5x5-sq15-mk11.png");

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
}

impl SimulationDriver {
    pub(crate) fn new() -> Result<Self> {
        Ok(Self {
            configs: serde_json::from_str(CONFIG).context("解析模拟相机配置")?,
            tool_pose: Arc::new(RwLock::new(None)),
        })
    }

    pub(crate) fn update_tool_pose(&mut self, pose: ToolPose) {
        *self.tool_pose.write().expect("simulation tool pose lock") = Some(pose);
    }
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
        let Some(config) = self
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
        let static_depth = simulation_depth(&config);
        Ok(Box::new(SimulationStream {
            config,
            sequence: 0,
            frame_period: Duration::from_secs_f64(1.0 / f64::from(color_profile.frames_per_second)),
            next_frame_at: None,
            static_color,
            static_depth,
            tool_pose: Arc::clone(&self.tool_pose),
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
}

impl CameraStream for SimulationStream {
    fn next_frameset(&mut self) -> Result<Option<CameraFrameBundle>> {
        let now = Instant::now();
        if self.next_frame_at.is_some_and(|deadline| now < deadline) {
            return Ok(None);
        }
        let deadline = now + self.frame_period;
        self.next_frame_at = Some(deadline);
        self.sequence += 1;
        let now_ns = now_ns();
        let width = self.config.calibration.width;
        let height = self.config.calibration.height;
        let mut color = self.static_color.clone();
        let mut depth = self.static_depth.clone();
        if let Some(tool_pose) = self
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
        Ok(Some(CameraFrameBundle {
            schema_version: SCHEMA_VERSION,
            sequence: self.sequence,
            source_id: self.config.source_id.clone(),
            device_time_ns: now_ns,
            device_time_domain: "simulation_clock".into(),
            received_time_ns: now_ns,
            color: CameraImagePlane {
                width,
                height,
                stride_bytes: width * 3,
                pixel_format: "rgb8".into(),
                frame_id: self.config.calibration.frame_id.clone(),
                data: color,
            },
            depth: CameraImagePlane {
                width,
                height,
                stride_bytes: width * 2,
                pixel_format: "z16le".into(),
                frame_id: self.config.calibration.frame_id.clone(),
                data: depth.iter().flat_map(|value| value.to_le_bytes()).collect(),
            },
            color_intrinsics: intrinsics.clone(),
            depth_intrinsics: intrinsics,
            depth_to_color: Pose3 {
                position_m: [0.0; 3],
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            depth_scale_m: self.config.depth_scale_m,
        }))
    }
}

fn source_info(config: &SimulationConfig) -> CameraSourceInfo {
    let width = config.calibration.width;
    let height = config.calibration.height;
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
        profiles: vec![
            CameraStreamProfile {
                key: format!("color:{width}x{height}:rgb8:10"),
                stream: CameraStreamKind::Color,
                width,
                height,
                frames_per_second: 10,
                pixel_format: "rgb8".into(),
                is_default: true,
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
                is_default: true,
                available: true,
                unavailable_reason: None,
            },
        ],
        driver_extensions: vec![],
        available: true,
    }
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

fn simulation_depth(config: &SimulationConfig) -> Vec<u16> {
    let width = config.calibration.width;
    let height = config.calibration.height;
    let mut values = vec![0; (width * height) as usize];
    match config.simulation {
        SimulationKind::PickPlace => {
            fill_rect(&mut values, width, 80, 217, 171, 318, 630);
            fill_rect(&mut values, width, 263, 119, 580, 359, 630);
        }
        SimulationKind::DepthGrid => {
            fill_rect(&mut values, width, 310, 210, 330, 230, 400);
            fill_rect(&mut values, width, 350, 210, 356, 217, 380);
        }
    }
    values
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
    let size = 0.075;
    let corners = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(size, 0.0, 0.0),
        Point3::new(size, size, 0.0),
        Point3::new(0.0, size, 0.0),
    ];
    let mut projected = [[0.0; 2]; 4];
    let mut depths = [0.0; 4];
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
        depths[index] = point.z;
    }
    let source = [
        [0.0, 0.0],
        [f64::from(BOARD.width() - 1), 0.0],
        [f64::from(BOARD.width() - 1), f64::from(BOARD.height() - 1)],
        [0.0, f64::from(BOARD.height() - 1)],
    ];
    let Some(homography) = homography(source, projected) else {
        return Ok(());
    };
    let Some(inverse) = homography.try_inverse() else {
        return Ok(());
    };
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
    let mut image = RgbImage::from_raw(
        config.calibration.width,
        config.calibration.height,
        std::mem::take(color),
    )
    .ok_or_else(|| eyre::eyre!("模拟彩色图尺寸错误"))?;
    let board_depth = depths.iter().sum::<f64>() / depths.len() as f64;
    let raw_depth = (board_depth / config.depth_scale_m)
        .round()
        .clamp(1.0, f64::from(u16::MAX)) as u16;
    for y in top..=bottom {
        for x in left..=right {
            let mapped = inverse * Vector3::new(f64::from(x), f64::from(y), 1.0);
            if mapped.z.abs() < f64::EPSILON {
                continue;
            }
            let u = mapped.x / mapped.z;
            let v = mapped.y / mapped.z;
            if u >= 0.0 && v >= 0.0 && u < f64::from(BOARD.width()) && v < f64::from(BOARD.height())
            {
                let value = BOARD.get_pixel(u as u32, v as u32)[0];
                image.put_pixel(x, y, Rgb([value; 3]));
                depth[(y * config.calibration.width + x) as usize] = raw_depth;
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

fn homography(source: [[f64; 2]; 4], target: [[f64; 2]; 4]) -> Option<Matrix3<f64>> {
    let mut a = SMatrix::<f64, 8, 8>::zeros();
    let mut b = SVector::<f64, 8>::zeros();
    for index in 0..4 {
        let [x, y] = source[index];
        let [u, v] = target[index];
        a[(index * 2, 0)] = x;
        a[(index * 2, 1)] = y;
        a[(index * 2, 2)] = 1.0;
        a[(index * 2, 6)] = -u * x;
        a[(index * 2, 7)] = -u * y;
        b[index * 2] = u;
        a[(index * 2 + 1, 3)] = x;
        a[(index * 2 + 1, 4)] = y;
        a[(index * 2 + 1, 5)] = 1.0;
        a[(index * 2 + 1, 6)] = -v * x;
        a[(index * 2 + 1, 7)] = -v * y;
        b[index * 2 + 1] = v;
    }
    let solution = a.lu().solve(&b)?;
    Some(Matrix3::new(
        solution[0],
        solution[1],
        solution[2],
        solution[3],
        solution[4],
        solution[5],
        solution[6],
        solution[7],
        1.0,
    ))
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
        let first = stream.next_frameset().unwrap().unwrap();
        std::thread::sleep(Duration::from_millis(110));
        let second = stream.next_frameset().unwrap().unwrap();
        assert_eq!(second.sequence, first.sequence + 1);
        assert_eq!(first.color.data, second.color.data);
        assert_eq!(first.depth.data, second.depth.data);
    }

    #[test]
    fn tool_pose_changes_the_same_rgbd_stream() {
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
        let without_tool = baseline.next_frameset().unwrap().unwrap();
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
        let mut rendered = driver.open(&source.source_id, &color, &depth, &[]).unwrap();
        let with_tool = rendered.next_frameset().unwrap().unwrap();

        assert_ne!(without_tool.color.data, with_tool.color.data);
        assert_ne!(without_tool.depth.data, with_tool.depth.data);
    }

    #[test]
    fn printable_margin_is_not_part_of_the_physical_board() {
        let board = image::load_from_memory(CHARUCO_BOARD).unwrap().into_luma8();
        let cropped = crop_print_margin(&board);
        assert!(cropped.width() < board.width());
        assert!(cropped.height() < board.height());
        assert_eq!(cropped.get_pixel(0, 0)[0], 0);
    }
}
