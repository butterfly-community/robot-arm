//! Stage boundaries use a local compute fixture, never robot motion or real model weights.
use super::*;
use axum::{
    Json, Router,
    routing::{get, post},
};
use serde_json::{Value, json};
use std::sync::Mutex;

fn frame() -> CameraFrameBundle {
    CameraFrameBundle {
        schema_version: SCHEMA_VERSION,
        sequence: 7,
        source_id: "fixture".into(),
        device_time_ns: 100,
        received_time_ns: 100,
        device_time_domain: "fixture".into(),
        color: CameraImagePlane {
            width: 16,
            height: 16,
            stride_bytes: 48,
            pixel_format: "rgb8".into(),
            frame_id: "optical".into(),
            data: vec![127; 16 * 16 * 3],
        },
        // Segmentation must succeed even without usable depth / extrinsics.
        aligned_depth: CameraImagePlane {
            width: 0,
            height: 0,
            stride_bytes: 0,
            pixel_format: "not-depth".into(),
            frame_id: "optical".into(),
            data: vec![],
        },
        intrinsics: robot_arm_messages::CameraIntrinsics {
            width: 16,
            height: 16,
            focal_length_px: [20.; 2],
            principal_point_px: [8.; 2],
            distortion_model: "none".into(),
            distortion: vec![],
        },
        depth_scale_m: 0.001,
        calibration: None,
    }
}

async fn compute_fixture() -> (
    String,
    Arc<Mutex<Vec<(String, Value)>>>,
    tokio::task::JoinHandle<()>,
) {
    let calls = Arc::new(Mutex::new(vec![]));
    let segment_calls = calls.clone();
    let grasp_calls = calls.clone();
    let mut png = Cursor::new(Vec::new());
    DynamicImage::ImageLuma8(image::GrayImage::from_pixel(16, 16, image::Luma([255])))
        .write_to(&mut png, ImageFormat::Png)
        .unwrap();
    let mask = BASE64.encode(png.into_inner());
    let app = Router::new()
        .route(
            "/v1/model",
            get(|| async {
                Json(json!({ "model": "auto", "models": [
            { "id": "auto", "label": "Automatic", "prompt_free": true },
            { "id": "text", "label": "Prompted", "prompt_free": false }
        ] }))
            }),
        )
        .route(
            "/v1/segment",
            post(move |Json(request): Json<Value>| async move {
                segment_calls
                    .lock()
                    .unwrap()
                    .push(("segment".into(), request));
                Json(
                    json!({ "instances": [{ "instance_id": "frame-0", "label": "frame",
                "confidence": 0.9, "bounding_box_xyxy": [0.,0.,16.,16.], "mask_width": 16,
                "mask_height": 16, "mask_png_base64": mask }] }),
                )
            }),
        )
        .route(
            "/v1/grasps",
            post(move |Json(request): Json<Value>| async move {
                grasp_calls.lock().unwrap().push(("grasps".into(), request));
                Json(
                    json!({ "candidates": [{ "transform": [[1.,0.,0.,0.1],[0.,1.,0.,0.2],
                [0.,0.,1.,0.3],[0.,0.,0.,1.]], "confidence": 0.8 }] }),
                )
            }),
        );
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("http://{}", listener.local_addr().unwrap());
    let task = tokio::spawn(async move { axum::serve(listener, app).await.unwrap() });
    (url, calls, task)
}

#[tokio::test]
async fn segmentation_only_needs_rgb_and_does_not_call_grasps() {
    let (url, calls, server) = compute_fixture().await;
    for model in ["auto", "text"] {
        let config = SceneConfig {
            compute_service_url: url.clone(),
            model: Some(model.into()),
            classes: vec!["frame".into()],
            ..Default::default()
        };
        let output = segment_frame(reqwest::Client::new(), config, frame(), None, 10)
            .await
            .unwrap();
        assert_eq!(output.instances.len(), 1);
        assert!(output.assets.contains_key("mask-0.png"));
        assert!(output.assets.contains_key("overlay.png"));
        assert!(!output.assets.contains_key("depth.png"));
        assert!(output.frame.calibration.is_none());
        assert!(reconstruct_frame(&output, 11).is_err());
    }
    let calls = calls.lock().unwrap();
    assert_eq!(calls.len(), 2);
    assert!(calls.iter().all(|(kind, _)| kind == "segment"));
    assert_eq!(calls[0].1["classes"], json!([]));
    assert_eq!(calls[1].1["classes"], json!(["frame"]));
    server.abort();
}

#[tokio::test]
async fn reconstruction_has_no_grasps_and_generation_targets_only_one_instance() {
    let (url, calls, server) = compute_fixture().await;
    let config = SceneConfig {
        compute_service_url: url.clone(),
        ..Default::default()
    };
    let mut input = segment_frame(reqwest::Client::new(), config.clone(), frame(), None, 10)
        .await
        .unwrap();
    input.frame.aligned_depth = CameraImagePlane {
        width: 16,
        height: 16,
        stride_bytes: 32,
        pixel_format: "z16le".into(),
        frame_id: "optical".into(),
        data: [232, 3].repeat(16 * 16),
    };
    input.frame.calibration = Some(DepthCameraCalibration {
        schema_version: SCHEMA_VERSION,
        sequence: 7,
        source_time_ns: 100,
        source_id: "fixture".into(),
        parent_frame_id: "base".into(),
        frame_id: "optical".into(),
        translation_m: [0.; 3],
        orientation_xyzw: [0., 0., 0., 1.],
        width: 16,
        height: 16,
        distortion_model: "none".into(),
        distortion: vec![],
        camera_matrix: [20., 0., 8., 0., 20., 8., 0., 0., 1.],
        projection_matrix: [20., 0., 8., 0., 0., 20., 8., 0., 0., 0., 1., 0.],
    });
    let ProcessedStage::Reconstruction(mut scene, clouds) = reconstruct_frame(&input, 11).unwrap()
    else {
        panic!()
    };
    assert_eq!(scene.sample_time_ns, 100);
    assert!(scene.objects.iter().all(|o| o.grasp_candidates.is_empty()));
    assert_eq!(calls.lock().unwrap().len(), 1);
    let mut other = scene.objects[0].clone();
    other.object_id = "unselected".into();
    scene.objects.push(other);
    attach_grasp_candidates(
        &reqwest::Client::new(),
        &config,
        "fixture-gripper",
        None,
        "frame-0",
        &mut scene,
        &clouds,
    )
    .await
    .unwrap();
    assert_eq!(scene.objects[0].grasp_candidates.len(), 1);
    assert!(scene.objects[1].grasp_candidates.is_empty());
    let calls = calls.lock().unwrap();
    assert_eq!(calls.len(), 2);
    assert_eq!(calls[1].0, "grasps");
    assert_eq!(
        calls[1].1["points_xyz_m"].as_array().unwrap().len(),
        clouds[0].points_xyz_m.len()
    );
    server.abort();
}

#[tokio::test]
async fn stages_reject_stale_or_unselected_inputs_without_recomputing_upstream() {
    let (url, calls, server) = compute_fixture().await;
    let config = SceneConfig {
        compute_service_url: url,
        ..Default::default()
    };
    let input = segment_frame(reqwest::Client::new(), config, frame(), None, 10)
        .await
        .unwrap();
    let mut request: PerceptionRequest = serde_json::from_value(json!({
        "schema_version": SCHEMA_VERSION, "request_id": "test", "action": "reconstruct",
        "input_sequence": 9 }))
    .unwrap();
    assert!(validate_stage_input(&request, Some(&input), None).is_err());
    request.input_sequence = Some(10);
    assert!(validate_stage_input(&request, Some(&input), None).is_ok());
    request.action = RequestAction::GenerateGrasps;
    assert!(validate_stage_input(&request, Some(&input), None).is_err());
    assert_eq!(calls.lock().unwrap().len(), 1);
    server.abort();
}
