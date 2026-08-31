use std::io;

use base64::{Engine, engine::general_purpose::STANDARD as BASE64};
use eyre::{Result, bail, eyre};
use nalgebra::{Matrix4, UnitQuaternion, Vector3};
use opencv::{
    calib,
    core::{self, Mat, Scalar, Size, Vector},
    geometry, imgcodecs, imgproc, objdetect,
    prelude::*,
};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

#[derive(Clone, Debug, Serialize, Deserialize)]
struct MatrixPose {
    rotation_matrix: [f64; 9],
    translation_m: [f64; 3],
}

#[derive(Clone, Debug, Deserialize)]
struct BoardConfig {
    dictionary: String,
    squares_x: i32,
    squares_y: i32,
    square_size_m: f32,
    marker_size_m: f32,
}

#[derive(Deserialize)]
struct DetectRequest {
    image_png_base64: String,
    camera_matrix: [f64; 9],
    distortion: Vec<f64>,
    board: BoardConfig,
}

#[derive(Serialize)]
struct DetectResponse {
    #[serde(flatten)]
    pose: MatrixPose,
    visualization_png_base64: String,
}

#[derive(Deserialize)]
struct SolveRequest {
    tcp_in_base: Vec<MatrixPose>,
    board_in_camera: Vec<MatrixPose>,
}

#[derive(Serialize)]
struct SolveResponse {
    camera_in_base: MatrixPose,
    board_in_calibration_tool: MatrixPose,
    translation_residuals_m: Vec<f64>,
    rotation_residuals_rad: Vec<f64>,
    solver: String,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error:?}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    if std::env::args().nth(1).as_deref() == Some("--self-test") {
        return self_test();
    }
    let mut request: Value = serde_json::from_reader(io::stdin())?;
    let operation = request
        .get("operation")
        .and_then(Value::as_str)
        .ok_or_else(|| eyre!("calibration operation is required"))?
        .to_owned();
    request
        .as_object_mut()
        .expect("calibration request is an object")
        .remove("operation");
    let response = match operation.as_str() {
        "detect" => serde_json::to_value(detect(serde_json::from_value(request)?)?)?,
        "solve" => serde_json::to_value(solve(serde_json::from_value(request)?)?)?,
        _ => bail!("unsupported calibration operation {operation}"),
    };
    serde_json::to_writer(io::stdout(), &response)?;
    Ok(())
}

fn detect(request: DetectRequest) -> Result<DetectResponse> {
    let encoded = BASE64.decode(request.image_png_base64)?;
    let mut image = imgcodecs::imdecode(&Vector::from_slice(&encoded), imgcodecs::IMREAD_COLOR)?;
    if image.empty() {
        bail!("cannot decode calibration image");
    }
    let dictionary =
        objdetect::get_predefined_dictionary_i32(dictionary_id(&request.board.dictionary)?)?;
    let board = objdetect::CharucoBoard::new_def(
        Size::new(request.board.squares_x, request.board.squares_y),
        request.board.square_size_m,
        request.board.marker_size_m,
        &dictionary,
    )?;
    let detector = objdetect::CharucoDetector::new_def(&board)?;
    let mut corners = Mat::default();
    let mut ids = Mat::default();
    detector.detect_board_def(&image, &mut corners, &mut ids)?;
    if ids.empty() {
        bail!("ChArUco board was not detected");
    }

    let mut object_points = Mat::default();
    let mut image_points = Mat::default();
    board.match_image_points(&corners, &ids, &mut object_points, &mut image_points)?;
    let camera_matrix = matrix3(&request.camera_matrix)?;
    let distortion = Mat::from_slice(&request.distortion)?;
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
        request.board.square_size_m * 2.0,
    )?;
    let mut visualization = Vector::<u8>::new();
    if !imgcodecs::imencode_def(".png", &image, &mut visualization)? {
        bail!("cannot encode calibration visualization");
    }
    Ok(DetectResponse {
        pose: matrix_pose(&rotation, &translation)?,
        visualization_png_base64: BASE64.encode(visualization.as_slice()),
    })
}

fn solve(request: SolveRequest) -> Result<SolveResponse> {
    if request.tcp_in_base.len() != request.board_in_camera.len() {
        bail!("calibration pose arrays have different lengths");
    }
    let tcp_in_base = request
        .tcp_in_base
        .iter()
        .map(transform)
        .collect::<Vec<_>>();
    let board_in_camera = request
        .board_in_camera
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
    Ok(SolveResponse {
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

fn transform(value: &MatrixPose) -> Matrix4<f64> {
    Matrix4::new(
        value.rotation_matrix[0],
        value.rotation_matrix[1],
        value.rotation_matrix[2],
        value.translation_m[0],
        value.rotation_matrix[3],
        value.rotation_matrix[4],
        value.rotation_matrix[5],
        value.translation_m[1],
        value.rotation_matrix[6],
        value.rotation_matrix[7],
        value.rotation_matrix[8],
        value.translation_m[2],
        0.0,
        0.0,
        0.0,
        1.0,
    )
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

fn matrix_pose(rotation: &Mat, translation: &Mat) -> Result<MatrixPose> {
    Ok(MatrixPose {
        rotation_matrix: [
            *rotation.at_2d::<f64>(0, 0)?,
            *rotation.at_2d::<f64>(0, 1)?,
            *rotation.at_2d::<f64>(0, 2)?,
            *rotation.at_2d::<f64>(1, 0)?,
            *rotation.at_2d::<f64>(1, 1)?,
            *rotation.at_2d::<f64>(1, 2)?,
            *rotation.at_2d::<f64>(2, 0)?,
            *rotation.at_2d::<f64>(2, 1)?,
            *rotation.at_2d::<f64>(2, 2)?,
        ],
        translation_m: [
            *translation.at_2d::<f64>(0, 0)?,
            *translation.at_2d::<f64>(1, 0)?,
            *translation.at_2d::<f64>(2, 0)?,
        ],
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

fn matrix_pose_from_transform(value: &Matrix4<f64>) -> MatrixPose {
    MatrixPose {
        rotation_matrix: [
            value[(0, 0)],
            value[(0, 1)],
            value[(0, 2)],
            value[(1, 0)],
            value[(1, 1)],
            value[(1, 2)],
            value[(2, 0)],
            value[(2, 1)],
            value[(2, 2)],
        ],
        translation_m: [value[(0, 3)], value[(1, 3)], value[(2, 3)]],
    }
}

fn self_test() -> Result<()> {
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
    ];
    let camera_inverse = inverse(&camera)?;
    let observations = tools
        .iter()
        .map(|tool| camera_inverse * tool * board)
        .collect::<Vec<_>>();
    let solved = solve(SolveRequest {
        tcp_in_base: tools.iter().map(matrix_pose_from_transform).collect(),
        board_in_camera: observations
            .iter()
            .map(matrix_pose_from_transform)
            .collect(),
    })?;
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
    let detection = detect(DetectRequest {
        image_png_base64: BASE64.encode(encoded.as_slice()),
        camera_matrix: [800.0, 0.0, 300.0, 0.0, 800.0, 300.0, 0.0, 0.0, 1.0],
        distortion: vec![0.0; 5],
        board: BoardConfig {
            dictionary: "DICT_4X4_50".into(),
            squares_x: 5,
            squares_y: 5,
            square_size_m: 0.015,
            marker_size_m: 0.011,
        },
    })?;
    if detection.visualization_png_base64.is_empty() {
        bail!("calibration visualization was not produced");
    }
    println!(
        "{}",
        json!({"opencv": core::get_version_string()?, "self_test": "passed"})
    );
    Ok(())
}
