//! OpenCV owns detection, intrinsic/stereo calibration, rectification and matching.
//! Input/output pixels are RGB; disparity and all transforms have explicit units.
use eyre::{Result, bail, ensure};
use opencv::{
    calib,
    core::{self, Mat, Point2f, Point3f, Size, Vector},
    imgproc, objdetect,
    prelude::*,
    stereo,
};
use robot_arm_messages::{CalibrationBoard, Pose3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Corners {
    pub ids: Vec<i32>,
    pub pixels: Vec<[f32; 2]>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sample {
    pub left: Corners,
    pub right: Corners,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Calibration {
    pub width: u32,
    pub height: u32,
    pub left_k: [f64; 9],
    pub right_k: [f64; 9],
    pub left_d: Vec<f64>,
    pub right_d: Vec<f64>,
    /// Maps original left-camera coordinates into right-camera coordinates.
    pub left_to_right: Pose3,
    pub left_rms_px: f64,
    pub right_rms_px: f64,
    pub stereo_rms_px: f64,
    pub sample_count: usize,
}

fn board(config: &CalibrationBoard) -> Result<objdetect::CharucoBoard> {
    let dictionary =
        objdetect::get_predefined_dictionary_i32(super::dictionary_id(&config.dictionary)?)?;
    Ok(objdetect::CharucoBoard::new_def(
        Size::new(config.squares_x as i32, config.squares_y as i32),
        config.square_size_m as f32,
        config.marker_size_m as f32,
        &dictionary,
    )?)
}

pub fn detect(rgb: &[u8], width: u32, height: u32, config: &CalibrationBoard) -> Result<Corners> {
    ensure!(
        rgb.len() == width as usize * height as usize * 3,
        "无效 RGB 图像尺寸"
    );
    let bytes = Mat::from_slice(rgb)?;
    let image = bytes.reshape(3, height as i32)?;
    let mut gray = Mat::default();
    imgproc::cvt_color_def(&image, &mut gray, imgproc::COLOR_RGB2GRAY)?;
    // Intrinsics are deliberately not required to bootstrap calibration.
    let detector = objdetect::CharucoDetector::new_def(&board(config)?)?;
    let mut corners = Mat::default();
    let mut ids = Mat::default();
    detector.detect_board_def(&gray, &mut corners, &mut ids)?;
    ensure!(!ids.empty(), "未识别到 ChArUco 标定板");
    Ok(Corners {
        ids: ids.data_typed::<i32>()?.to_vec(),
        pixels: corners
            .data_typed::<Point2f>()?
            .iter()
            .map(|p| [p.x, p.y])
            .collect(),
    })
}

pub fn calibrate(
    samples: &[Sample],
    width: u32,
    height: u32,
    config: &CalibrationBoard,
) -> Result<Calibration> {
    // Three distinct plane orientations are needed to solve unconstrained intrinsics.
    ensure!(
        samples.len() >= 3,
        "至少采集三个不同倾角的标定板姿态，再求解内参"
    );
    let points = board(config)?.get_chessboard_corners()?;
    let mut objects = Vector::<Vector<Point3f>>::new();
    let mut left = Vector::<Vector<Point2f>>::new();
    let mut right = Vector::<Vector<Point2f>>::new();
    for sample in samples {
        let mut op = Vector::new();
        let mut lp = Vector::new();
        let mut rp = Vector::new();
        for (i, id) in sample.left.ids.iter().enumerate() {
            if let Some(j) = sample.right.ids.iter().position(|r| r == id) {
                op.push(points.get(*id as usize)?);
                lp.push(Point2f::new(
                    sample.left.pixels[i][0],
                    sample.left.pixels[i][1],
                ));
                rp.push(Point2f::new(
                    sample.right.pixels[j][0],
                    sample.right.pixels[j][1],
                ));
            }
        }
        ensure!(op.len() >= 4, "左右图至少需要四个共同 ChArUco 角点");
        objects.push(op);
        left.push(lp);
        right.push(rp);
    }
    let size = Size::new(width as i32, height as i32);
    let (mut kl, mut kr, mut dl, mut dr) = (
        Mat::default(),
        Mat::default(),
        Mat::default(),
        Mat::default(),
    );
    let (mut rv, mut tv) = (Vector::<Mat>::new(), Vector::<Mat>::new());
    let criteria = core::TermCriteria::new(
        core::TermCriteria_Type::COUNT as i32 + core::TermCriteria_Type::EPS as i32,
        30,
        f64::EPSILON,
    )?;
    // OpenCV's QR solver avoids the much slower default SVD normal-system solve.
    // Keep its default convergence criteria and full five-coefficient lens model.
    let left_rms_px = calib::calibrate_camera(
        &objects,
        &left,
        size,
        &mut kl,
        &mut dl,
        &mut rv,
        &mut tv,
        calib::CALIB_USE_QR,
        criteria,
    )?;
    let right_rms_px = calib::calibrate_camera(
        &objects,
        &right,
        size,
        &mut kr,
        &mut dr,
        &mut rv,
        &mut tv,
        calib::CALIB_USE_QR,
        criteria,
    )?;
    let (mut r, mut t, mut e, mut f) = (
        Mat::default(),
        Mat::default(),
        Mat::default(),
        Mat::default(),
    );
    // Jointly refine both cameras from the independent intrinsic initializations.
    // FIX_INTRINSIC would freeze each monocular fit's sampling error into depth.
    let stereo_rms_px = calib::stereo_calibrate(
        &objects,
        &left,
        &right,
        &mut kl,
        &mut dl,
        &mut kr,
        &mut dr,
        size,
        &mut r,
        &mut t,
        &mut e,
        &mut f,
        calib::CALIB_USE_INTRINSIC_GUESS | calib::CALIB_USE_QR,
        criteria,
    )?;
    let calibration = Calibration {
        width,
        height,
        left_k: kl.data_typed::<f64>()?.try_into()?,
        right_k: kr.data_typed::<f64>()?.try_into()?,
        left_d: dl.data_typed::<f64>()?.to_vec(),
        right_d: dr.data_typed::<f64>()?.to_vec(),
        left_to_right: super::matrix_pose(&r, &t)?,
        left_rms_px,
        right_rms_px,
        stereo_rms_px,
        sample_count: samples.len(),
    };
    ensure!(
        calibration
            .left_k
            .iter()
            .chain(&calibration.right_k)
            .chain(&calibration.left_d)
            .chain(&calibration.right_d)
            .chain(&calibration.left_to_right.position_m)
            .all(|v| v.is_finite())
            && stereo_rms_px.is_finite(),
        "标定退化：请改变标定板位置和倾角重新采样"
    );
    Ok(calibration)
}

pub struct Depth {
    pub rgb: Vec<u8>,
    /// Camera optical Z, metres. NaN means no valid match.
    pub z_m: Vec<f32>,
    pub k: [f64; 9],
    /// Rectified optical camera expressed in original left camera.
    pub rectified_in_left: Pose3,
}

pub struct Matcher {
    calibration: Calibration,
    maps: [(Mat, Mat); 2],
    q: Mat,
    p: Mat,
    rectified_in_left: Pose3,
    vertical: bool,
    min_disparity: i32,
    matcher: core::Ptr<stereo::StereoSGBM>,
}

impl Matcher {
    pub fn new(c: &Calibration) -> Result<Self> {
        let kl = super::matrix3(&c.left_k)?;
        let kr = super::matrix3(&c.right_k)?;
        let dl = Mat::from_slice(&c.left_d)?;
        let dr = Mat::from_slice(&c.right_d)?;
        let pose = super::transform(&c.left_to_right);
        let (rs, ts) = super::opencv_poses(&[pose])?;
        let (mut r1, mut r2, mut p1, mut p2, mut q) = (
            Mat::default(),
            Mat::default(),
            Mat::default(),
            Mat::default(),
            Mat::default(),
        );
        let size = Size::new(c.width as i32, c.height as i32);
        stereo::stereo_rectify_def(
            &kl,
            &dl,
            &kr,
            &dr,
            size,
            &rs.get(0)?,
            &ts.get(0)?,
            &mut r1,
            &mut r2,
            &mut p1,
            &mut p2,
            &mut q,
        )?;
        let mut maps = [
            (Mat::default(), Mat::default()),
            (Mat::default(), Mat::default()),
        ];
        let (x, y) = &mut maps[0];
        imgproc::init_undistort_rectify_map(&kl, &dl, &r1, &p1, size, core::CV_32FC1, x, y)?;
        let (x, y) = &mut maps[1];
        imgproc::init_undistort_rectify_map(&kr, &dr, &r2, &p2, size, core::CV_32FC1, x, y)?;
        let rectified_in_left = super::matrix_pose(
            &r1.t()?.to_mat()?,
            &Mat::zeros(3, 1, core::CV_64F)?.to_mat()?,
        )?;
        // OpenCV's documented SGBM smoothness terms for one channel and 3x3 blocks.
        // Search range scales with resolution; it is not an arm/workspace constraint.
        let vertical = p2.at_2d::<f64>(1, 3)?.abs() > p2.at_2d::<f64>(0, 3)?.abs();
        let extent = if vertical { c.height } else { c.width };
        let disparities = ((extent as i32 / 4 + 15) / 16 * 16).max(16);
        let axis = if vertical { 1 } else { 0 };
        let min_disparity = if *p2.at_2d::<f64>(axis, 3)? > 0.0 {
            -disparities
        } else {
            0
        };
        let matcher = stereo::StereoSGBM::create(
            min_disparity,
            disparities,
            3,
            8 * 9,
            32 * 9,
            1,
            0,
            // OpenCV's documented uniqueness and speckle filtering, not a
            // metric-depth cutoff. Fixture comparison removes false far points
            // while preserving the two measured surface depths.
            10,
            100,
            2,
            stereo::StereoSGBM_MODE_SGBM_3WAY,
        )?;
        Ok(Self {
            calibration: c.clone(),
            maps,
            q,
            p: p1,
            rectified_in_left,
            vertical,
            min_disparity,
            matcher,
        })
    }

    fn rectify(&self, rgb: &[u8], side: usize) -> Result<Mat> {
        let c = &self.calibration;
        ensure!(
            rgb.len() == c.width as usize * c.height as usize * 3,
            "双目图像尺寸与标定配置不一致，请重新标定或恢复原配置"
        );
        let bytes = Mat::from_slice(rgb)?;
        let input = bytes.reshape(3, c.height as i32)?;
        let mut output = Mat::default();
        imgproc::remap_def(
            &input,
            &mut output,
            &self.maps[side].0,
            &self.maps[side].1,
            imgproc::INTER_LINEAR,
        )?;
        Ok(output)
    }

    pub fn rectify_color(&self, left: &[u8]) -> Result<Vec<u8>> {
        Ok(self.rectify(left, 0)?.data_bytes()?.to_vec())
    }

    pub fn compute(&mut self, left: &[u8], right: &[u8]) -> Result<Depth> {
        let mut rectified = [Mat::default(), Mat::default()];
        let mut gray = [Mat::default(), Mat::default()];
        for (i, rgb) in [left, right].iter().enumerate() {
            rectified[i] = self.rectify(rgb, i)?;
            imgproc::cvt_color_def(&rectified[i], &mut gray[i], imgproc::COLOR_RGB2GRAY)?;
        }
        let mut disparity = Mat::default();
        if self.vertical {
            let mut transposed = [Mat::default(), Mat::default()];
            core::transpose(&gray[0], &mut transposed[0])?;
            core::transpose(&gray[1], &mut transposed[1])?;
            let mut matched = Mat::default();
            self.matcher
                .compute(&transposed[0], &transposed[1], &mut matched)?;
            core::transpose(&matched, &mut disparity)?;
        } else {
            self.matcher.compute(&gray[0], &gray[1], &mut disparity)?;
        }
        let mut float = Mat::default();
        disparity.convert_to(&mut float, core::CV_32F, 1.0 / 16.0, 0.0)?;
        let mut xyz = Mat::default();
        stereo::reproject_image_to_3d_def(&float, &mut xyz, &self.q)?;
        let d = float.data_typed::<f32>()?;
        let z_m = xyz
            .data_typed::<core::Vec3f>()?
            .iter()
            .zip(d)
            .map(|(p, d)| {
                if *d > (self.min_disparity - 1) as f32 && p[2].is_finite() && p[2] > 0.0 {
                    p[2]
                } else {
                    f32::NAN
                }
            })
            .collect();
        let p = self.p.data_typed::<f64>()?;
        Ok(Depth {
            rgb: rectified[0].data_bytes()?.to_vec(),
            z_m,
            k: [p[0], p[1], p[2], p[4], p[5], p[6], p[8], p[9], p[10]],
            rectified_in_left: self.rectified_in_left.clone(),
        })
    }
}

pub fn solve_wrist(tools: &[Pose3], boards: &[Pose3]) -> Result<super::Solution> {
    ensure!(
        tools.len() == boards.len() && tools.len() >= 3,
        "手眼标定需要至少三个不同姿态及对应反馈"
    );
    let tools = tools.iter().map(super::transform).collect::<Vec<_>>();
    let boards = boards.iter().map(super::transform).collect::<Vec<_>>();
    let (tr, tt) = super::opencv_poses(&tools)?;
    let (br, bt) = super::opencv_poses(&boards)?;
    let (mut r, mut t) = (Mat::default(), Mat::default());
    calib::calibrate_hand_eye(
        &tr,
        &tt,
        &br,
        &bt,
        &mut r,
        &mut t,
        calib::HandEyeCalibrationMethod::CALIB_HAND_EYE_PARK,
    )?;
    let camera = super::matrix_pose(&r, &t)?;
    if !camera
        .position_m
        .iter()
        .chain(&camera.orientation_xyzw)
        .all(|v| v.is_finite())
    {
        bail!("腕部手眼标定退化，请增加不同轴的旋转姿态");
    }
    let transformed = tools
        .iter()
        .zip(&boards)
        .map(|(tool, board)| tool * super::transform(&camera) * board)
        .collect::<Vec<_>>();
    let origin = transformed[0];
    let translation_residuals_m = transformed
        .iter()
        .map(|v| (v.fixed_view::<3, 1>(0, 3) - origin.fixed_view::<3, 1>(0, 3)).norm())
        .collect();
    let rotation_residuals_rad = transformed
        .iter()
        .map(|v| {
            let d = v.fixed_view::<3, 3>(0, 0) * origin.fixed_view::<3, 3>(0, 0).transpose();
            ((d.trace() - 1.0) / 2.0).clamp(-1.0, 1.0).acos()
        })
        .collect();
    let (rr, rt) = super::opencv_poses(&[origin])?;
    Ok(super::Solution {
        camera_in_base: camera,
        board_in_calibration_tool: super::matrix_pose(&rr.get(0)?, &rt.get(0)?)?,
        translation_residuals_m,
        rotation_residuals_rad,
        solver: "OpenCV calibrateHandEye/PARK (camera in tool, fixed board in base)".into(),
    })
}
