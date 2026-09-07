use eyre::{Result, bail, eyre};
use nalgebra::{
    Isometry3, Matrix3, Matrix4, Quaternion, Rotation3, Translation3, UnitQuaternion, Vector3,
};
use opencv::{
    calib,
    core::{self, Mat, Point2f, Scalar, Size, Vector},
    geometry, imgcodecs, imgproc, objdetect,
    prelude::*,
};
use robot_arm_messages::{CalibrationBoard, Pose3};

#[derive(Clone, Debug)]
pub struct Detection {
    pub board_in_camera: Pose3,
    pub visualization_png: Vec<u8>,
    pub matched_corner_count: usize,
    pub reprojection_rmse_px: f64,
}

#[derive(Clone, Debug)]
pub struct Solution {
    pub camera_in_base: Pose3,
    pub board_in_calibration_tool: Pose3,
    pub translation_residuals_m: Vec<f64>,
    pub rotation_residuals_rad: Vec<f64>,
    pub solver: String,
}

pub fn detect(
    image_png: &[u8],
    camera_matrix_values: &[f64; 9],
    distortion_values: &[f64],
    board_config: &CalibrationBoard,
) -> Result<Detection> {
    // The public boundary is a standard PNG (RGB), not a raw BGR buffer.
    // ChArUco treats three-channel Mats as BGR, and imencode expects BGR too.
    // Let OpenCV's codecs own both boundary conversions; never swap manually.
    let mut image =
        imgcodecs::imdecode(&Vector::from_slice(image_png), imgcodecs::IMREAD_COLOR_BGR)?;
    if image.empty() {
        bail!("cannot decode calibration image");
    }
    let dictionary =
        objdetect::get_predefined_dictionary_i32(dictionary_id(&board_config.dictionary)?)?;
    let board = objdetect::CharucoBoard::new_def(
        Size::new(board_config.squares_x as i32, board_config.squares_y as i32),
        board_config.square_size_m as f32,
        board_config.marker_size_m as f32,
        &dictionary,
    )?;
    let camera_matrix = matrix3(camera_matrix_values)?;
    let distortion = Mat::from_slice(distortion_values)?;
    let mut charuco_parameters = objdetect::CharucoParameters::default()?;
    charuco_parameters.set_camera_matrix(camera_matrix.clone());
    charuco_parameters.set_dist_coeffs(distortion.try_clone()?);
    // ChArUco refines chessboard corners itself. Keep marker refinement at the
    // official default: nearby chess squares can bias marker subpixel windows.
    let detector_parameters = objdetect::DetectorParameters::default()?;
    let detector = objdetect::CharucoDetector::new(
        &board,
        &charuco_parameters,
        &detector_parameters,
        objdetect::RefineParameters::new_def()?,
    )?;
    let mut corners = Mat::default();
    let mut ids = Mat::default();
    // Mild prefiltering reduces raster phase bias in subpixel corner gradients.
    // The 0.8 px sigma is measured against projected corner/pose truth, not a
    // detection gate. Keep the original image for the diagnostic overlay.
    let mut detection_image = Mat::default();
    imgproc::gaussian_blur_def(&image, &mut detection_image, Size::new(0, 0), 0.8)?;
    detector.detect_board_def(&detection_image, &mut corners, &mut ids)?;
    if ids.empty() {
        bail!("ChArUco board was not detected");
    }

    // Refit the detected chess corners with OpenCV's local subpixel solver.
    // ChArUco uses an adaptive window and 0.1 px default termination. This local
    // refit was compared on identical PNGs against pose truth at 1280 and 1920;
    // its window/iteration parameters are not observation rejection gates.
    let mut gray = Mat::default();
    imgproc::cvt_color_def(&detection_image, &mut gray, imgproc::COLOR_BGR2GRAY)?;
    imgproc::corner_sub_pix(
        &gray,
        &mut corners,
        Size::new(7, 7),
        Size::new(-1, -1),
        core::TermCriteria::new(
            core::TermCriteria_Type::COUNT as i32 | core::TermCriteria_Type::EPS as i32,
            100,
            0.0001,
        )?,
    )?;

    let mut object_points = Mat::default();
    let mut image_points = Mat::default();
    board.match_image_points(&corners, &ids, &mut object_points, &mut image_points)?;
    let matched_corner_count = image_points.total();
    if matched_corner_count < 4 {
        bail!("ChArUco 只匹配到 {matched_corner_count} 个角点，求解位姿至少需要 4 个");
    }
    let mut rotation_vector = Mat::default();
    let mut translation = Mat::default();
    if !geometry::solve_pnp(
        &object_points,
        &image_points,
        &camera_matrix,
        &distortion,
        &mut rotation_vector,
        &mut translation,
        false,
        geometry::SOLVEPNP_ITERATIVE,
    )? {
        bail!("OpenCV solvePnP did not return a board pose");
    }
    let mut reprojected_points = Mat::default();
    geometry::project_points_def(
        &object_points,
        &rotation_vector,
        &translation,
        &camera_matrix,
        &distortion,
        &mut reprojected_points,
    )?;
    let observed_points = image_points.data_typed::<Point2f>()?;
    let reprojected_points = reprojected_points.data_typed::<Point2f>()?;
    if observed_points.len() != reprojected_points.len() || observed_points.is_empty() {
        bail!("OpenCV returned an inconsistent set of reprojected ChArUco corners");
    }
    let squared_error_sum = observed_points
        .iter()
        .zip(reprojected_points)
        .map(|(observed, reprojected)| {
            let dx = f64::from(observed.x - reprojected.x);
            let dy = f64::from(observed.y - reprojected.y);
            dx * dx + dy * dy
        })
        .sum::<f64>();
    let reprojection_rmse_px = (squared_error_sum / observed_points.len() as f64).sqrt();
    let mut rotation = Mat::default();
    geometry::rodrigues_def(&rotation_vector, &mut rotation)?;
    objdetect::draw_detected_corners_charuco(
        &mut image,
        &corners,
        &ids,
        Scalar::new(255.0, 0.0, 0.0, 0.0),
    )?;
    imgproc::draw_frame_axes_def(
        &mut image,
        &camera_matrix,
        &distortion,
        &rotation_vector,
        &translation,
        board_config.square_size_m as f32 * 2.0,
    )?;
    let mut visualization = Vector::<u8>::new();
    if !imgcodecs::imencode_def(".png", &image, &mut visualization)? {
        bail!("cannot encode calibration visualization");
    }
    Ok(Detection {
        board_in_camera: matrix_pose(&rotation, &translation)?,
        visualization_png: visualization.to_vec(),
        matched_corner_count,
        reprojection_rmse_px,
    })
}

pub fn solve(tcp_in_base_poses: &[Pose3], board_in_camera_poses: &[Pose3]) -> Result<Solution> {
    if tcp_in_base_poses.len() != board_in_camera_poses.len() {
        bail!("calibration pose arrays have different lengths");
    }
    let tcp_in_base = tcp_in_base_poses.iter().map(transform).collect::<Vec<_>>();
    let board_in_camera = board_in_camera_poses
        .iter()
        .map(transform)
        .collect::<Vec<_>>();

    // OpenCV solves ^cT_w * ^wT_b = ^cT_g * ^gT_b. Swapping the semantic
    // roles gives the fixed-camera transforms used by this project:
    // world2cam := ^toolT_base, base2gripper := ^boardT_camera.
    let world_to_camera = tcp_in_base
        .iter()
        .map(inverse)
        .collect::<Result<Vec<_>>>()?;
    let base_to_gripper = board_in_camera
        .iter()
        .map(inverse)
        .collect::<Result<Vec<_>>>()?;
    let (world_r, world_t) = opencv_poses(&world_to_camera)?;
    let (gripper_r, gripper_t) = opencv_poses(&base_to_gripper)?;
    let mut camera_r = Mat::default();
    let mut camera_t = Mat::default();
    let mut board_r = Mat::default();
    let mut board_t = Mat::default();
    calib::calibrate_robot_world_hand_eye(
        &world_r,
        &world_t,
        &gripper_r,
        &gripper_t,
        &mut camera_r,
        &mut camera_t,
        &mut board_r,
        &mut board_t,
        calib::RobotWorldHandEyeCalibrationMethod::CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
    )?;
    let camera_in_base_pose = matrix_pose(&camera_r, &camera_t)?;
    let board_in_tool_pose = matrix_pose(&board_r, &board_t)?;
    let camera_in_base = transform(&camera_in_base_pose);
    let board_in_tool = transform(&board_in_tool_pose);
    let camera_to_base = inverse(&camera_in_base)?;
    let mut translation_residuals_m = Vec::with_capacity(tcp_in_base.len());
    let mut rotation_residuals_rad = Vec::with_capacity(tcp_in_base.len());
    for (observed, tool) in board_in_camera.iter().zip(&tcp_in_base) {
        let predicted = camera_to_base * tool * board_in_tool;
        translation_residuals_m
            .push((predicted.fixed_view::<3, 1>(0, 3) - observed.fixed_view::<3, 1>(0, 3)).norm());
        let delta =
            predicted.fixed_view::<3, 3>(0, 0) * observed.fixed_view::<3, 3>(0, 0).transpose();
        rotation_residuals_rad.push(((delta.trace() - 1.0) / 2.0).clamp(-1.0, 1.0).acos());
    }
    Ok(Solution {
        camera_in_base: camera_in_base_pose,
        board_in_calibration_tool: board_in_tool_pose,
        translation_residuals_m,
        rotation_residuals_rad,
        solver: format!(
            "OpenCV {} via opencv-rust calibrateRobotWorldHandEye/SHAH",
            core::get_version_string()?
        ),
    })
}

fn dictionary_id(name: &str) -> Result<i32> {
    Ok(match name {
        "DICT_4X4_50" => objdetect::DICT_4X4_50,
        "DICT_4X4_100" => objdetect::DICT_4X4_100,
        "DICT_4X4_250" => objdetect::DICT_4X4_250,
        "DICT_4X4_1000" => objdetect::DICT_4X4_1000,
        "DICT_5X5_50" => objdetect::DICT_5X5_50,
        "DICT_5X5_100" => objdetect::DICT_5X5_100,
        "DICT_5X5_250" => objdetect::DICT_5X5_250,
        "DICT_5X5_1000" => objdetect::DICT_5X5_1000,
        "DICT_6X6_50" => objdetect::DICT_6X6_50,
        "DICT_6X6_100" => objdetect::DICT_6X6_100,
        "DICT_6X6_250" => objdetect::DICT_6X6_250,
        "DICT_6X6_1000" => objdetect::DICT_6X6_1000,
        "DICT_7X7_50" => objdetect::DICT_7X7_50,
        "DICT_7X7_100" => objdetect::DICT_7X7_100,
        "DICT_7X7_250" => objdetect::DICT_7X7_250,
        "DICT_7X7_1000" => objdetect::DICT_7X7_1000,
        "DICT_ARUCO_ORIGINAL" => objdetect::DICT_ARUCO_ORIGINAL,
        "DICT_APRILTAG_16h5" => objdetect::DICT_APRILTAG_16h5,
        "DICT_APRILTAG_25h9" => objdetect::DICT_APRILTAG_25h9,
        "DICT_APRILTAG_36h10" => objdetect::DICT_APRILTAG_36h10,
        "DICT_APRILTAG_36h11" => objdetect::DICT_APRILTAG_36h11,
        "DICT_ARUCO_MIP_36h12" => objdetect::DICT_ARUCO_MIP_36h12,
        _ => bail!("unsupported OpenCV dictionary {name}"),
    })
}

fn matrix3(values: &[f64; 9]) -> Result<Mat> {
    Mat::from_slice_2d(&[
        [values[0], values[1], values[2]],
        [values[3], values[4], values[5]],
        [values[6], values[7], values[8]],
    ])
    .map_err(Into::into)
}

fn transform(value: &Pose3) -> Matrix4<f64> {
    let [x, y, z, w] = value.orientation_xyzw;
    Isometry3::from_parts(
        Translation3::from(Vector3::from(value.position_m)),
        UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z)),
    )
    .to_homogeneous()
}

fn inverse(value: &Matrix4<f64>) -> Result<Matrix4<f64>> {
    value
        .try_inverse()
        .ok_or_else(|| eyre!("calibration transform is not invertible"))
}

fn opencv_poses(values: &[Matrix4<f64>]) -> Result<(Vector<Mat>, Vector<Mat>)> {
    let mut rotations = Vector::new();
    let mut translations = Vector::new();
    for value in values {
        rotations.push(Mat::from_slice_2d(&[
            [value[(0, 0)], value[(0, 1)], value[(0, 2)]],
            [value[(1, 0)], value[(1, 1)], value[(1, 2)]],
            [value[(2, 0)], value[(2, 1)], value[(2, 2)]],
        ])?);
        translations.push(Mat::from_slice_2d(&[
            [value[(0, 3)]],
            [value[(1, 3)]],
            [value[(2, 3)]],
        ])?);
    }
    Ok((rotations, translations))
}

fn matrix_pose(rotation: &Mat, translation: &Mat) -> Result<Pose3> {
    let rotation = Matrix3::from_row_slice(&[
        *rotation.at_2d::<f64>(0, 0)?,
        *rotation.at_2d::<f64>(0, 1)?,
        *rotation.at_2d::<f64>(0, 2)?,
        *rotation.at_2d::<f64>(1, 0)?,
        *rotation.at_2d::<f64>(1, 1)?,
        *rotation.at_2d::<f64>(1, 2)?,
        *rotation.at_2d::<f64>(2, 0)?,
        *rotation.at_2d::<f64>(2, 1)?,
        *rotation.at_2d::<f64>(2, 2)?,
    ]);
    let orientation =
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation));
    let quaternion = orientation.quaternion();
    Ok(Pose3 {
        position_m: [
            *translation.at_2d::<f64>(0, 0)?,
            *translation.at_2d::<f64>(1, 0)?,
            *translation.at_2d::<f64>(2, 0)?,
        ],
        orientation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
    })
}

fn synthetic_pose(rotation_vector: [f64; 3], translation: [f64; 3]) -> Matrix4<f64> {
    let rotation = UnitQuaternion::from_scaled_axis(Vector3::from(rotation_vector));
    let mut value = Matrix4::identity();
    value
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(rotation.to_rotation_matrix().matrix());
    value
        .fixed_view_mut::<3, 1>(0, 3)
        .copy_from(&Vector3::from(translation));
    value
}

fn pose_from_transform(value: &Matrix4<f64>) -> Pose3 {
    let rotation = Matrix3::new(
        value[(0, 0)],
        value[(0, 1)],
        value[(0, 2)],
        value[(1, 0)],
        value[(1, 1)],
        value[(1, 2)],
        value[(2, 0)],
        value[(2, 1)],
        value[(2, 2)],
    );
    let orientation =
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation));
    let quaternion = orientation.quaternion();
    Pose3 {
        position_m: [value[(0, 3)], value[(1, 3)], value[(2, 3)]],
        orientation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
    }
}

pub fn self_test() -> Result<()> {
    let camera = synthetic_pose([0.08, -0.12, 0.05], [0.24, -0.18, 0.42]);
    let board = synthetic_pose([-0.04, 0.03, 0.09], [0.02, 0.01, 0.06]);
    let tools = [
        synthetic_pose([0.10, -0.05, 0.02], [0.11, -0.08, 0.19]),
        synthetic_pose([-0.08, 0.14, -0.04], [0.16, -0.03, 0.22]),
        synthetic_pose([0.03, 0.06, 0.15], [0.09, 0.04, 0.17]),
        synthetic_pose([-0.12, -0.02, 0.09], [0.19, 0.06, 0.25]),
        synthetic_pose([0.17, 0.04, -0.11], [0.13, -0.11, 0.23]),
        synthetic_pose([-0.05, -0.16, -0.07], [0.21, 0.01, 0.20]),
        synthetic_pose([0.09, 0.18, 0.05], [0.07, 0.09, 0.21]),
        synthetic_pose([-0.15, 0.08, 0.13], [0.17, -0.06, 0.16]),
        synthetic_pose([0.06, -0.11, 0.18], [0.12, 0.07, 0.24]),
    ];
    let camera_inverse = inverse(&camera)?;
    let observations = tools
        .iter()
        .map(|tool| camera_inverse * tool * board)
        .collect::<Vec<_>>();
    let tool_poses = tools.iter().map(pose_from_transform).collect::<Vec<_>>();
    let observation_poses = observations
        .iter()
        .map(pose_from_transform)
        .collect::<Vec<_>>();
    let solved = solve(&tool_poses, &observation_poses)?;
    if (transform(&solved.camera_in_base) - camera).amax() > 1e-7
        || (transform(&solved.board_in_calibration_tool) - board).amax() > 1e-7
    {
        bail!("synthetic calibration transforms were not recovered");
    }

    let dictionary = objdetect::get_predefined_dictionary_i32(objdetect::DICT_4X4_50)?;
    let calibration_board =
        objdetect::CharucoBoard::new_def(Size::new(5, 5), 0.015, 0.011, &dictionary)?;
    let mut board_image = Mat::default();
    calibration_board.generate_image(Size::new(600, 600), &mut board_image, 40, 1)?;
    let mut encoded = Vector::<u8>::new();
    if !imgcodecs::imencode_def(".png", &board_image, &mut encoded)? {
        bail!("synthetic ChArUco board was not encoded");
    }
    let detection = detect(
        encoded.as_slice(),
        &[800.0, 0.0, 300.0, 0.0, 800.0, 300.0, 0.0, 0.0, 1.0],
        &[0.0; 5],
        &CalibrationBoard {
            pattern: "charuco".into(),
            dictionary: "DICT_4X4_50".into(),
            squares_x: 5,
            squares_y: 5,
            square_size_m: 0.015,
            marker_size_m: 0.011,
            measured_width_m: 0.075,
            measured_height_m: 0.075,
        },
    )?;
    if detection.visualization_png.is_empty() {
        bail!("calibration visualization was not produced");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    #[test]
    fn opencv_rust_uses_opencv_5() {
        assert_eq!(opencv::core::CV_VERSION_MAJOR, 5);
        assert_eq!(opencv::core::get_version_major(), 5);
    }

    #[test]
    fn opencv_recovers_synthetic_hand_eye_transforms() {
        super::self_test().unwrap();
    }
}
