use camera_media::{Encoder, decode, decode_image, native_rgb};
use robot_arm_messages::{CameraImagePlane, CameraRawVideoFrame, SCHEMA_VERSION};

fn source(width: u32, height: u32, sequence: u64) -> CameraRawVideoFrame {
    let mut data = vec![0; (width * height * 3) as usize];
    for y in 0..height {
        for x in 0..width {
            let color = match x * 3 / width {
                0 => [220, 20, 30],
                1 => [20, 220, 30],
                _ => [20, 30, 220],
            };
            let i = ((y * width + x) * 3) as usize;
            data[i..i + 3].copy_from_slice(&color);
        }
    }
    CameraRawVideoFrame {
        schema_version: SCHEMA_VERSION,
        sequence,
        source_id: "fixture".into(),
        device_time_ns: sequence as i64 * 16_666_667,
        received_time_ns: sequence as i64 * 16_666_667,
        color: CameraImagePlane {
            width,
            height,
            stride_bytes: width * 3,
            pixel_format: "rgb8".into(),
            frame_id: "optical".into(),
            data,
        },
    }
}

#[test]
fn full_resolution_h264_roundtrip_preserves_geometry_and_color_order() {
    for (width, height) in [(1280, 720), (1920, 1080)] {
        let input = source(width, height, 1);
        let mut encoder = Encoder::new(width, height, 60.0).unwrap();
        let output = encoder.encode(input.clone(), true).unwrap();
        assert_eq!(output.latest.sequence, input.sequence);
        assert_eq!(output.latest.received_time_ns, input.received_time_ns);
        assert_eq!(output.latest.color.pixel_format, "h264");
        assert_eq!(output.latest.color.stride_bytes, 0);
        let decoded = decode_image(&output.latest.color).unwrap();
        let mut decoder = ffmpeg_next::codec::context::Context::new_with_codec(
            ffmpeg_next::decoder::find(ffmpeg_next::codec::Id::H264).unwrap(),
        )
        .decoder()
        .video()
        .unwrap();
        decoder
            .send_packet(&ffmpeg_next::Packet::copy(&output.latest.color.data))
            .unwrap();
        let mut picture = ffmpeg_next::frame::Video::empty();
        decoder.receive_frame(&mut picture).unwrap();
        assert_eq!(picture.color_space(), ffmpeg_next::color::Space::BT709);
        assert_eq!(
            picture.color_primaries(),
            ffmpeg_next::color::Primaries::BT709
        );
        assert_eq!(
            picture.color_transfer_characteristic(),
            ffmpeg_next::color::TransferCharacteristic::IEC61966_2_1
        );
        assert_eq!(decoded.len(), input.color.data.len());
        for x in [width / 6, width / 2, width * 5 / 6] {
            let i = ((height / 2 * width + x) * 3) as usize;
            for channel in 0..3 {
                assert!(
                    decoded[i + channel].abs_diff(input.color.data[i + channel]) < 8,
                    "codec color mismatch: {:?} vs {:?}",
                    &decoded[i..i + 3],
                    &input.color.data[i..i + 3]
                );
            }
        }
        assert!(output.latest.color.data.len() < input.color.data.len() / 10);
    }
}

#[test]
fn native_mjpeg_transcodes_without_rgb_intermediate_and_keeps_colour() {
    use ffmpeg_next::{codec, format::Pixel, frame, software::scaling};
    let input = source(1280, 720, 1);
    let mut rgb = frame::Video::new(Pixel::RGB24, 1280, 720);
    let stride = rgb.stride(0);
    for (y, row) in input.color.data.chunks_exact(1280 * 3).enumerate() {
        rgb.data_mut(0)[y * stride..y * stride + row.len()].copy_from_slice(row);
    }
    let mut scaler = scaling::Context::get(
        Pixel::RGB24,
        1280,
        720,
        Pixel::YUVJ420P,
        1280,
        720,
        scaling::Flags::BILINEAR,
    )
    .unwrap();
    let mut yuv = frame::Video::empty();
    scaler.run(&rgb, &mut yuv).unwrap();
    yuv.set_pts(Some(0));
    let mut jpeg = codec::context::Context::new_with_codec(
        ffmpeg_next::encoder::find(codec::Id::MJPEG).unwrap(),
    )
    .encoder()
    .video()
    .unwrap();
    jpeg.set_width(1280);
    jpeg.set_height(720);
    jpeg.set_format(Pixel::YUVJ420P);
    jpeg.set_time_base((1, 60));
    let mut jpeg = jpeg.open().unwrap();
    jpeg.send_frame(&yuv).unwrap();
    let mut packet = ffmpeg_next::Packet::empty();
    jpeg.receive_packet(&mut packet).unwrap();
    let mut native = input.clone();
    native.color.pixel_format = "MJPG".into();
    native.color.stride_bytes = 0;
    native.color.data = packet.data().unwrap().to_vec();
    let mut decoder = camera_media::NativeDecoder::default();
    let mut h264 = Encoder::new(1280, 720, 60.0).unwrap();
    for sequence in 1..=3 {
        native.sequence = sequence;
        native.device_time_ns = sequence as i64 * 16_666_667;
        let decoded = decoder.decode(native.clone()).unwrap();
        assert_eq!(decoded.source.color.pixel_format, "MJPG");
        assert!(
            decoded.source.color.data.is_empty(),
            "decoded pixels stay in AVFrame, not a packed RGB vector"
        );
        let video = h264.encode_decoded(decoded, true).unwrap();
        let rgb = decode_image(&video.latest.color).unwrap();
        for x in [1280 / 6, 1280 / 2, 1280 * 5 / 6] {
            let i = (360 * 1280 + x) * 3;
            for c in 0..3 {
                assert!(
                    rgb[i + c].abs_diff(input.color.data[i + c]) < 10,
                    "MJPEG->H264 color mismatch: {:?} vs {:?}",
                    &rgb[i..i + 3],
                    &input.color.data[i..i + 3]
                );
            }
        }
    }
}

#[test]
fn sparse_sampling_and_late_join_decode_from_same_encoder() {
    let mut encoder = Encoder::new(64, 48, 30.0).unwrap();
    let first = encoder.encode(source(64, 48, 1), true).unwrap();
    let second = encoder.encode(source(64, 48, 2), false).unwrap();
    assert_eq!(second.gop.len(), 2);
    assert_eq!(first.stream_id, second.stream_id);
    assert_eq!(
        decode(second.gop.iter().map(|f| &f.color)).unwrap().len(),
        64 * 48 * 3
    );
    let sampled = encoder.encode(source(64, 48, 3), true).unwrap();
    assert_eq!(sampled.gop.len(), 1);
    assert_eq!(
        decode_image(&sampled.latest.color).unwrap().len(),
        64 * 48 * 3
    );
    // Old snapshot owns its references even after the live stream advances.
    assert_eq!(second.gop.len(), 2);
    assert!(decode(second.gop.iter().map(|f| &f.color)).is_ok());
    let next_stream = Encoder::new(64, 48, 30.0)
        .unwrap()
        .encode(source(64, 48, 1), true)
        .unwrap();
    assert_ne!(sampled.stream_id, next_stream.stream_id);

    let mut slow = source(64, 48, 4);
    slow.received_time_ns = 2_000_000_000;
    slow.device_time_ns = 2_000_000_000;
    let fresh_keyframe = encoder.encode(slow, false).unwrap();
    assert_eq!(fresh_keyframe.gop.len(), 1);
    assert!(decode_image(&fresh_keyframe.latest.color).is_ok());
}

#[test]
fn ffmpeg_converts_native_formats_and_padding() {
    for (format, bytes) in [
        ("RGB3", vec![255, 0, 0, 0, 0, 255, 99, 99]),
        ("BGR3", vec![0, 0, 255, 255, 0, 0, 99, 99]),
        ("rgba8", vec![255, 0, 0, 255, 0, 0, 255, 255]),
        ("bgra8", vec![0, 0, 255, 255, 255, 0, 0, 255]),
    ] {
        let rgb = native_rgb(2, 1, 8, format, &bytes).unwrap();
        assert_eq!(rgb.data, [255, 0, 0, 0, 0, 255], "{format}");
    }
    for (format, bytes, stride) in [
        ("YUYV", vec![16, 128, 16, 128, 235, 128, 235, 128], 4),
        ("UYVY", vec![128, 16, 128, 16, 128, 235, 128, 235], 4),
        ("NV12", vec![16, 16, 235, 235, 128, 128], 2),
    ] {
        let rgb = native_rgb(2, 2, stride, format, &bytes).unwrap();
        assert!(rgb.data[..6].iter().all(|v| *v < 3), "{format}");
        assert!(rgb.data[6..].iter().all(|v| *v > 250), "{format}");
    }
    // Chroma-bearing fixtures catch U/V swaps that black/white frames cannot.
    for (format, bytes, stride) in [
        ("YUYV", vec![81, 90, 81, 240, 81, 90, 81, 240], 4),
        ("UYVY", vec![90, 81, 240, 81, 90, 81, 240, 81], 4),
        ("NV12", vec![81, 81, 81, 81, 90, 240], 2),
    ] {
        let rgb = native_rgb(2, 2, stride, format, &bytes).unwrap();
        for pixel in rgb.data.chunks_exact(3) {
            assert!(
                pixel[0] > 245 && pixel[1] < 5 && pixel[2] < 5,
                "{format}: {pixel:?}"
            );
        }
    }
    assert!(native_rgb(2, 2, 6, "RGB3", &[0]).is_err());
    assert!(native_rgb(2, 2, 6, "unknown", &[0; 12]).is_err());
}
