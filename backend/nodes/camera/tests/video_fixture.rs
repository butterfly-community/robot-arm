//! Isolated browser fixture, no Dora connection, device access or robot actions.
#[path = "../src/video_server.rs"]
mod video_server;

/// Replays a captured image to distinguish codec time from USB/frame delivery.
/// This is a codec benchmark, not a camera or browser acceptance test.
#[test]
#[ignore = "set CAMERA_BENCHMARK_IMAGE to a captured PNG/JPEG"]
fn benchmark_real_image_codec() {
    use robot_arm_messages::{CameraImagePlane, CameraRawVideoFrame, SCHEMA_VERSION};
    use std::time::Instant;
    let path = std::env::var("CAMERA_BENCHMARK_IMAGE").unwrap();
    let rgb = image::open(path).unwrap().to_rgb8();
    let (width, height) = rgb.dimensions();
    let pixels = rgb.into_raw();
    let mut encoder = camera_media::Encoder::new(width, height, 60.0).unwrap();
    let mut encode_ms = vec![];
    let mut encoded_bytes = 0;
    for sequence in 0..120 {
        let start = Instant::now();
        let frame = encoder
            .encode(
                CameraRawVideoFrame {
                    schema_version: SCHEMA_VERSION,
                    sequence,
                    source_id: "benchmark".into(),
                    device_time_ns: sequence as i64 * 16_666_667,
                    received_time_ns: sequence as i64 * 16_666_667,
                    color: CameraImagePlane {
                        width,
                        height,
                        stride_bytes: width * 3,
                        pixel_format: "rgb8".into(),
                        frame_id: "optical".into(),
                        data: pixels.clone(),
                    },
                },
                false,
            )
            .unwrap();
        encode_ms.push(start.elapsed().as_secs_f64() * 1000.0);
        encoded_bytes += frame.latest.color.data.len();
    }
    println!(
        "CODEC_BENCHMARK {}",
        serde_json::json!({
            "width": width, "height": height, "frames":120,
            "h264_encode_mean_ms":encode_ms.iter().sum::<f64>() / 120.0,
            "h264_bytes":encoded_bytes
        })
    );
}

#[tokio::test(flavor = "multi_thread")]
#[ignore = "persistent server for frontend/tests/media; stop after browser tests"]
async fn serve_browser_fixture() {
    use robot_arm_messages::{CameraImagePlane, CameraRawVideoFrame, CameraRole, SCHEMA_VERSION};
    use std::{
        collections::BTreeMap,
        sync::Arc,
        time::{Duration, Instant, SystemTime, UNIX_EPOCH},
    };
    let mut streams = BTreeMap::new();
    for (role, width, height, rgb) in [
        (CameraRole::Depth, 1280, 720, [210_u8, 30, 20]),
        (CameraRole::External, 1920, 1080, [20_u8, 210, 30]),
        (CameraRole::Wrist, 1280, 720, [20_u8, 30, 210]),
    ] {
        let (sender, receiver) = tokio::sync::watch::channel(None);
        streams.insert(role, receiver);
        tokio::task::spawn_blocking(move || {
            let mut encoder = camera_media::Encoder::new(width, height, 30.0).unwrap();
            for sequence in 1.. {
                let start = Instant::now();
                let mut data = rgb.repeat((width * height) as usize);
                let x = (sequence as usize * 8) % (width as usize - 100);
                for y in 100..200 {
                    let offset = (y * width as usize + x) * 3;
                    data[offset..offset + 300].fill(240);
                }
                let frame = CameraRawVideoFrame {
                    schema_version: SCHEMA_VERSION,
                    sequence,
                    source_id: format!("fixture:{role:?}"),
                    device_time_ns: sequence as i64 * 33_333_333,
                    received_time_ns: SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_nanos() as i64,
                    color: CameraImagePlane {
                        width,
                        height,
                        stride_bytes: width * 3,
                        pixel_format: "rgb8".into(),
                        frame_id: "optical".into(),
                        data,
                    },
                };
                // Periodically restart one encoder, covering capture reopen and
                // sequence reset without making a test-only production route.
                if role == CameraRole::Wrist && sequence % 120 == 0 {
                    encoder = camera_media::Encoder::new(width, height, 30.0).unwrap();
                }
                sender.send_replace(Some(Arc::new(
                    encoder.encode(frame, sequence % 30 == 0).unwrap(),
                )));
                std::thread::sleep(
                    Duration::from_secs_f64(1.0 / 30.0).saturating_sub(start.elapsed()),
                );
            }
        });
    }
    let (_stop, shutdown) = tokio::sync::watch::channel(false);
    let listener = video_server::bind().await.unwrap();
    video_server::serve(listener, streams, shutdown)
        .await
        .unwrap();
}
