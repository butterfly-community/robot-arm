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
        "model:auto:frame-0",
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

#[tokio::test]
async fn manual_and_both_models_compose_on_one_frame_without_duplicate_layers() {
    let (url, calls, server) = compute_fixture().await;
    let config = SceneConfig {
        compute_service_url: url,
        model: Some("auto".into()),
        ..Default::default()
    };
    let empty = rebuild_segmented(empty_segmented(frame(), 20, "auto".into(), vec![])).unwrap();
    assert!(empty.instances.is_empty());
    assert!(calls.lock().unwrap().is_empty());
    let manual = ManualRegion {
        id: "region-1".into(),
        label: "自定义放置区".into(),
        bounding_box_xyxy: [2., 3., 6., 8.],
    };
    let edit = SegmentationEdit::Manual {
        regions: vec![manual.clone()],
    };
    let mut result = edit_segmented(reqwest::Client::new(), config.clone(), empty, edit, 21)
        .await
        .unwrap();
    assert!(
        calls.lock().unwrap().is_empty(),
        "manual rectangles must not call compute"
    );
    assert_eq!(result.instances[0].instance_id, "manual:region-1");
    assert_eq!(result.instances[0].label, "自定义放置区");
    let mask = image::load_from_memory(&result.instances[0].mask_png)
        .unwrap()
        .into_luma8();
    assert_eq!(mask.pixels().filter(|p| p[0] == 255).count(), 20);
    assert_eq!(mask.get_pixel(2, 3)[0], 255);
    assert_eq!(mask.get_pixel(6, 3)[0], 0);
    let frozen_color = result.assets["segmentation-color.png"].1.clone();
    for (sequence, model) in [(22, "auto"), (23, "text"), (24, "auto")] {
        let config = SceneConfig {
            model: Some(model.into()),
            classes: vec!["frame".into()],
            ..config.clone()
        };
        result = edit_segmented(
            reqwest::Client::new(),
            config,
            result,
            SegmentationEdit::Model,
            sequence,
        )
        .await
        .unwrap();
        assert_eq!(result.frame.sequence, 7);
        assert_eq!(result.frame.received_time_ns, 100);
        assert_eq!(result.assets["segmentation-color.png"].1, frozen_color);
        assert_eq!(result.manual_regions, vec![manual.clone()]);
    }
    assert_eq!(result.instances.len(), 3);
    assert_eq!(result.layers.len(), 2);
    // The combined masks use the normal geometry path, not a manual-only scene.
    let mut calibrated = result.clone();
    calibrated.frame.aligned_depth = CameraImagePlane {
        width: 16,
        height: 16,
        stride_bytes: 32,
        pixel_format: "z16le".into(),
        frame_id: "optical".into(),
        data: [232, 3].repeat(16 * 16),
    };
    calibrated.frame.calibration = Some(DepthCameraCalibration {
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
    let ProcessedStage::Reconstruction(scene, clouds) = reconstruct_frame(&calibrated, 25).unwrap()
    else {
        panic!()
    };
    assert_eq!(scene.objects.len(), 3);
    assert_eq!(clouds.len(), 3);
    assert_eq!(scene.sample_time_ns, 100);
    assert!(
        scene
            .objects
            .iter()
            .all(|object| object.grasp_candidates.is_empty())
    );
    assert!(
        scene
            .placement_regions
            .iter()
            .any(|region| region.source_object_id.as_deref() == Some("manual:region-1"))
    );
    let calls = calls.lock().unwrap().clone();
    assert_eq!(calls.len(), 3);
    assert!(
        calls
            .iter()
            .all(|(_, r)| r["image_base64"] == calls[0].1["image_base64"])
    );
    assert_eq!(calls[0].1["classes"], json!([]));
    assert_eq!(calls[1].1["classes"], json!(["frame"]));
    assert_eq!(
        result
            .instances
            .iter()
            .map(|i| &i.instance_id)
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        3
    );
    let request: PerceptionRequest = serde_json::from_value(json!({ "schema_version": 3,
        "request_id": "stale", "action": "refresh", "input_sequence": 23,
        "segmentation_edit": { "kind": "manual", "regions": [] } }))
    .unwrap();
    assert!(validate_stage_input(&request, Some(&result), None).is_err());
    let result = edit_segmented(
        reqwest::Client::new(),
        config.clone(),
        result,
        SegmentationEdit::Remove {
            instance_ids: vec!["model:text:frame-0".into()],
        },
        25,
    )
    .await
    .unwrap();
    assert_eq!(result.instances.len(), 2);
    let result = edit_segmented(
        reqwest::Client::new(),
        config,
        result,
        SegmentationEdit::Manual { regions: vec![] },
        26,
    )
    .await
    .unwrap();
    assert_eq!(result.instances.len(), 1);
    assert!(!result.assets.contains_key("mask-1.png"));
    assert_eq!(result.instances[0].instance_id, "model:auto:frame-0");
    server.abort();
}

#[test]
fn manual_region_validation_is_transactional_and_resolution_independent() {
    for [width, height] in [[1280, 720], [1920, 1080]] {
        let mut source = frame();
        source.color.width = width;
        source.color.height = height;
        source.color.stride_bytes = width * 3;
        source.color.data = vec![0; (width * height * 3) as usize];
        let mut input = empty_segmented(source, 1, String::new(), vec![]);
        input.manual_regions = vec![ManualRegion {
            id: "edge".into(),
            label: "边缘".into(),
            bounding_box_xyxy: [
                (width - 2) as f64,
                (height - 3) as f64,
                width as f64,
                height as f64,
            ],
        }];
        let result = rebuild_segmented(input.clone()).unwrap();
        let mask = image::load_from_memory(&result.instances[0].mask_png)
            .unwrap()
            .into_luma8();
        assert_eq!(mask.dimensions(), (width, height));
        assert_eq!(mask.pixels().filter(|p| p[0] == 255).count(), 6);
        for bounds in [
            [-1., 0., 2., 2.],
            [0., 0., width as f64 + 1., 1.],
            [2., 2., 1., 3.],
            [f64::NAN, 0., 2., 2.],
        ] {
            let mut invalid = input.clone();
            invalid.manual_regions[0].bounding_box_xyxy = bounds;
            assert!(rebuild_segmented(invalid).is_err());
        }
        let mut invalid = input.clone();
        invalid.manual_regions[0].label = " ".into();
        assert!(rebuild_segmented(invalid).is_err());
        input.manual_regions.push(input.manual_regions[0].clone());
        assert!(rebuild_segmented(input).is_err());
    }
}
