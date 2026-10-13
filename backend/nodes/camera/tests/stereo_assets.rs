//! Source: tools/perception/generate-stereo-assets.py. Truth is an assertion
//! oracle only; calibrate() consumes detected RGB corners, never fixture K/R/T.
#![cfg(feature = "opencv-runtime")]
use camera_calibration::stereo::{self, Sample};
use robot_arm_messages::CalibrationBoard;
use std::path::Path;

#[test]
fn both_rigs_calibrate_from_images_and_recover_metric_depth() {
    let assets = Path::new(env!("CARGO_MANIFEST_DIR")).join("assets");
    let board: CalibrationBoard =
        serde_json::from_slice(&std::fs::read(assets.join("charuco-board.json")).unwrap()).unwrap();
    for (rig, baseline) in [("external", 0.08), ("wrist", 0.06)] {
        for (w, h) in [(1280, 720), (1920, 1080)] {
            let read = |side: &str, suffix: &str| {
                image::open(assets.join(format!("stereo/{rig}-{side}-{suffix}.png")))
                    .unwrap()
                    .resize_exact(w, h, image::imageops::FilterType::Lanczos3)
                    .into_rgb8()
                    .into_raw()
            };
            let samples = (1..=24)
                .map(|i| Sample {
                    left: stereo::detect(&read("left", &format!("board-{i:02}")), w, h, &board)
                        .unwrap(),
                    right: stereo::detect(&read("right", &format!("board-{i:02}")), w, h, &board)
                        .unwrap(),
                })
                .collect::<Vec<_>>();
            let calibration = stereo::calibrate(&samples, w, h, &board).unwrap();
            let b = calibration
                .left_to_right
                .position_m
                .iter()
                .map(|v| v * v)
                .sum::<f64>()
                .sqrt();
            eprintln!(
                "{rig}: K {:?}, D {:?}",
                calibration.left_k, calibration.left_d
            );
            assert!((b - baseline).abs() < 0.001, "{rig} baseline={b}");
            let mut matcher = stereo::Matcher::new(&calibration).unwrap();
            let depth = matcher
                .compute(&read("left", "scene"), &read("right", "scene"))
                .unwrap();
            let mut cube = vec![];
            let mut table = vec![];
            // Interior optical rays, away from surface edges and matcher borders.
            for y in h / 2 - h / 40..h / 2 + h / 40 {
                for x in w / 2 - w / 40..w / 2 + w / 40 {
                    let z = depth.z_m[(y * w + x) as usize];
                    if z.is_finite() {
                        cube.push(z);
                    }
                }
            }
            for y in h / 4..h / 3 {
                for x in w / 2..w * 2 / 3 {
                    let z = depth.z_m[(y * w + x) as usize];
                    if z.is_finite() {
                        table.push(z);
                    }
                }
            }
            cube.sort_by(f32::total_cmp);
            table.sort_by(f32::total_cmp);
            assert!(!cube.is_empty() && !table.is_empty(), "depth missing");
            let cz = cube[cube.len() / 2];
            let tz = table[table.len() / 2];
            eprintln!(
                "{rig} {w}x{h}: RMS {:.4}px baseline {:.3}mm cube {cz:.6}m table {tz:.6}m",
                calibration.stereo_rms_px,
                b * 1000.
            );
            assert!((cz - 0.50).abs() < 0.001, "cube depth {cz}");
            assert!((tz - 0.55).abs() < 0.001, "table depth {tz}");
            // The entire fixture ends at 0.55 m. Far matches in uniform or
            // occluded regions must be rejected by the matcher, not clipped.
            assert!(
                depth.z_m.iter().all(|z| !z.is_finite() || *z < 1.0),
                "spurious far-depth match"
            );
            let truth: serde_json::Value =
                serde_json::from_slice(&std::fs::read(assets.join("stereo/truth.json")).unwrap())
                    .unwrap();
            let as_pose = |value: &serde_json::Value| {
                let rows = value.as_array().unwrap();
                let m = nalgebra::Matrix3::from_fn(|r, c| rows[r][c].as_f64().unwrap());
                let q = nalgebra::UnitQuaternion::from_rotation_matrix(
                    &nalgebra::Rotation3::from_matrix_unchecked(m),
                );
                let q = q.quaternion();
                robot_arm_messages::Pose3 {
                    position_m: std::array::from_fn(|r| rows[r][3].as_f64().unwrap()),
                    orientation_xyzw: [q.i, q.j, q.k, q.w],
                }
            };
            let mut poses = Vec::new();
            let mut boards = Vec::new();
            for (i, view) in truth["handeye"]["views"]
                .as_array()
                .unwrap()
                .iter()
                .enumerate()
            {
                poses.push(as_pose(&view["tcp_in_base"]));
                boards.push(
                    camera_calibration::detect(
                        &read("left", &format!("handeye-{:02}", i + 1)),
                        w,
                        h,
                        &calibration.left_k,
                        &calibration.left_d,
                        "plumb_bob",
                        &board,
                    )
                    .unwrap()
                    .board_in_camera,
                );
            }
            let solved = if rig == "external" {
                camera_calibration::solve(&poses, &boards).unwrap()
            } else {
                stereo::solve_wrist(&poses, &boards).unwrap()
            };
            let target = as_pose(
                &truth["handeye"][if rig == "external" {
                    "camera_in_base"
                } else {
                    "camera_in_tool"
                }],
            );
            let error = solved
                .camera_in_base
                .position_m
                .iter()
                .zip(target.position_m)
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>()
                .sqrt();
            eprintln!("{rig} {w} hand-eye position error {:.4} mm", error * 1000.);
            assert!(error < 0.001, "hand-eye error {error}");
        }
    }
}

#[test]
fn wrist_hand_eye_recovers_camera_in_tool_not_camera_in_base() {
    use nalgebra::{Isometry3, Translation3, UnitQuaternion};
    use robot_arm_messages::Pose3;
    let pose = |v: Isometry3<f64>| {
        let q = v.rotation.quaternion();
        Pose3 {
            position_m: v.translation.vector.into(),
            orientation_xyzw: [q.i, q.j, q.k, q.w],
        }
    };
    let camera = Isometry3::from_parts(
        Translation3::new(0.02, 0.01, 0.04),
        UnitQuaternion::from_euler_angles(0.1, -0.2, 0.03),
    );
    let board = Isometry3::translation(0.3, 0.0, 0.1);
    let mut tools = vec![];
    let mut boards = vec![];
    for i in 0..12 {
        let a = i as f64;
        let tool = Isometry3::from_parts(
            Translation3::new(0.1 + a * 0.005, 0.02, 0.2),
            UnitQuaternion::from_euler_angles(
                a * 0.03,
                (a * 0.7).sin() * 0.3,
                (a * 0.4).cos() * 0.2,
            ),
        );
        tools.push(pose(tool));
        boards.push(pose((tool * camera).inverse() * board));
    }
    let result = stereo::solve_wrist(&tools, &boards).unwrap();
    for (got, want) in result
        .camera_in_base
        .position_m
        .iter()
        .zip(camera.translation.vector.iter())
    {
        assert!((got - want).abs() < 1e-6);
    }
    assert!(result.translation_residuals_m.iter().all(|v| *v < 1e-6));
}

#[test]
fn matching_handles_reversed_and_vertical_camera_mounts() {
    use robot_arm_messages::Pose3;
    let assets = Path::new(env!("CARGO_MANIFEST_DIR")).join("assets/stereo");
    let read = |side| {
        image::open(assets.join(format!("external-{side}-scene.png")))
            .unwrap()
            .resize_exact(1280, 720, image::imageops::FilterType::Lanczos3)
            .into_rgb8()
    };
    let left = read("left");
    let right = read("right");
    let original = stereo::Calibration {
        width: 1280,
        height: 720,
        left_k: [1200., 0., 639.5, 0., 1200., 359.5, 0., 0., 1.],
        right_k: [1200., 0., 639.5, 0., 1200., 359.5, 0., 0., 1.],
        left_d: vec![0.; 5],
        right_d: vec![0.; 5],
        left_to_right: Pose3 {
            position_m: [-0.08, 0., 0.],
            orientation_xyzw: [0., 0., 0., 1.],
        },
        left_rms_px: 0.,
        right_rms_px: 0.,
        stereo_rms_px: 0.,
        sample_count: 0,
    };
    for vertical in [false, true] {
        let mut c = original.clone();
        let (a, b, x, y) = if vertical {
            c.width = 720;
            c.height = 1280;
            c.left_k = [1200., 0., 359.5, 0., 1200., 639.5, 0., 0., 1.];
            c.right_k = c.left_k;
            c.left_to_right.position_m = [0., -0.08, 0.];
            (
                image::imageops::rotate90(&left),
                image::imageops::rotate90(&right),
                360,
                640,
            )
        } else {
            c.left_to_right.position_m = [0.08, 0., 0.];
            (right.clone(), left.clone(), 448, 360)
        };
        let depth = stereo::Matcher::new(&c)
            .unwrap()
            .compute(a.as_raw(), b.as_raw())
            .unwrap();
        let z = depth.z_m[y * c.width as usize + x];
        assert!((z - 0.5).abs() < 0.001, "vertical={vertical}, depth={z}");
    }
}
