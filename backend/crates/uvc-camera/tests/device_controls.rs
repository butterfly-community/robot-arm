//! Explicit hardware diagnostics using the same V4L2 library as the driver.
//! The query is read-only; the separately selected FPS trial restores its change.
#[test]
#[ignore = "set UVC_DIAGNOSTIC_DEVICE; requires explicit hardware access"]
fn report_device_controls_without_starting_capture() {
    use v4l::video::Capture;
    let path = std::env::var("UVC_DIAGNOSTIC_DEVICE").unwrap();
    let device = v4l::Device::with_path(path).unwrap();
    println!("CAPABILITIES {:?}", device.query_caps().unwrap());
    println!("CURRENT_FORMAT {:?}", device.format().unwrap());
    println!("CURRENT_PARAMETERS {:?}", device.params().unwrap());
    for description in device.query_controls().unwrap() {
        if description.typ == v4l::control::Type::CtrlClass {
            continue;
        }
        println!(
            "CONTROL {:?} VALUE {:?}",
            description,
            device.control(description.id)
        );
    }
}

/// Temporary A/B test only. Always restore the original device value on exit.
#[test]
#[ignore = "explicit hardware trial; disables dynamic FPS for 90 seconds then restores it"]
fn fixed_rate_trial() {
    let device = v4l::Device::with_path(std::env::var("UVC_DIAGNOSTIC_DEVICE").unwrap()).unwrap();
    let description = device
        .query_controls()
        .unwrap()
        .into_iter()
        .find(|c| c.name == "Exposure, Dynamic Framerate")
        .expect("device has dynamic FPS control");
    let original = device.control(description.id).unwrap();
    struct Restore(v4l::Device, Option<v4l::control::Control>);
    impl Drop for Restore {
        fn drop(&mut self) {
            self.0
                .set_control(self.1.take().unwrap())
                .expect("restore original exposure policy");
        }
    }
    let guard = Restore(device, Some(original));
    guard
        .0
        .set_control(v4l::control::Control {
            id: description.id,
            value: v4l::control::Value::Boolean(false),
        })
        .unwrap();
    println!("FIXED_RATE_TRIAL_ACTIVE");
    std::thread::sleep(std::time::Duration::from_secs(90));
    drop(guard);
    println!("ORIGINAL_EXPOSURE_POLICY_RESTORED");
}

/// Supplementary stage timing, not a substitute for real page playback tests.
/// Stop this role with the page's capture switch first; never open it twice.
#[test]
#[ignore = "exclusive camera access required; stop page capture before running"]
fn capture_stage_benchmark() {
    use std::time::{Duration, Instant};
    use v4l::{buffer::Type, io::traits::CaptureStream, video::Capture};
    let device = v4l::Device::with_path(std::env::var("UVC_DIAGNOSTIC_DEVICE").unwrap()).unwrap();
    let format = device.format().unwrap();
    let params = device.params().unwrap();
    println!("BENCHMARK_FORMAT {format:?} {params:?}");
    let fps = f64::from(params.interval.denominator) / f64::from(params.interval.numerator);
    let mut stream = v4l::io::mmap::Stream::new(&device, Type::VideoCapture).unwrap();
    stream.set_timeout(Duration::from_secs(3));
    for _ in 0..30 {
        stream.next().unwrap();
    }
    for stage in ["capture", "decode", "encode"] {
        let mut encoder = camera_media::Encoder::new(format.width, format.height, fps).unwrap();
        let mut frames = 0_u64;
        let mut bytes = 0_u64;
        let mut output_bytes = 0_u64;
        let mut wait = Duration::ZERO;
        let mut decode = Duration::ZERO;
        let mut encode = Duration::ZERO;
        let mut first_sequence = None;
        let mut last_sequence = 0;
        let mut first_timestamp = None;
        let mut last_timestamp = Duration::ZERO;
        let mut decoder = camera_media::NativeDecoder::default();
        let started = Instant::now();
        while started.elapsed() < Duration::from_secs(20) {
            let before = Instant::now();
            let (data, metadata) = stream.next().unwrap();
            wait += before.elapsed();
            let timestamp = Duration::from(metadata.timestamp);
            first_timestamp.get_or_insert(timestamp);
            last_timestamp = timestamp;
            first_sequence.get_or_insert(metadata.sequence);
            last_sequence = metadata.sequence;
            let data = &data[..metadata.bytesused as usize];
            bytes += data.len() as u64;
            frames += 1;
            if stage == "capture" {
                continue;
            }
            let before = Instant::now();
            let decoded = decoder
                .decode(robot_arm_messages::CameraRawVideoFrame {
                    schema_version: robot_arm_messages::SCHEMA_VERSION,
                    sequence: u64::from(metadata.sequence),
                    source_id: "diagnostic".into(),
                    device_time_ns: timestamp.as_nanos() as i64,
                    received_time_ns: timestamp.as_nanos() as i64,
                    color: robot_arm_messages::CameraImagePlane {
                        width: format.width,
                        height: format.height,
                        stride_bytes: format.stride,
                        pixel_format: format.fourcc.to_string(),
                        frame_id: "optical".into(),
                        data: data.to_vec(),
                    },
                })
                .unwrap();
            decode += before.elapsed();
            if stage == "decode" {
                continue;
            }
            let before = Instant::now();
            let encoded = encoder.encode_decoded(decoded, false).unwrap();
            encode += before.elapsed();
            output_bytes += encoded.latest.color.data.len() as u64;
        }
        let seconds = started.elapsed().as_secs_f64();
        println!(
            "CAPTURE_STAGE stage={stage} frames={frames} seconds={seconds:.3} delivered_fps={:.3} device_fps={:.3} input_MBps={:.3} h264_MBps={:.3} wait_ms={:.3} decode_ms={:.3} encode_ms={:.3}",
            frames as f64 / seconds,
            f64::from(last_sequence - first_sequence.unwrap())
                / (last_timestamp - first_timestamp.unwrap()).as_secs_f64(),
            bytes as f64 / seconds / 1e6,
            output_bytes as f64 / seconds / 1e6,
            wait.as_secs_f64() * 1000.0 / frames as f64,
            decode.as_secs_f64() * 1000.0 / frames as f64,
            encode.as_secs_f64() * 1000.0 / frames as f64,
        );
    }
}
