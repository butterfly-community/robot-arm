//! One FFmpeg H.264 encoder per capture stream. Consumers decode only when
//! pixels are needed; no device-specific video transport or handwritten codec.
use std::sync::{
    Arc,
    atomic::{AtomicU64, Ordering},
};

use eyre::{Result, bail, eyre};
use ffmpeg::{codec, color, format::Pixel, frame, software::scaling};
use ffmpeg_next as ffmpeg;
use robot_arm_messages::{CameraImagePlane, CameraRawVideoFrame};

static NEXT_STREAM: AtomicU64 = AtomicU64::new(1);

/// Match the HD matrix used by browser video decoders. ffmpeg-next exposes
/// scaling but not this libswscale setting, so keep the FFI boundary here.
fn colorspace(
    scaler: &mut scaling::Context,
    input_matrix: i32,
    output_matrix: i32,
    input_full: bool,
    output_full: bool,
) -> Result<()> {
    // SAFETY: scaler owns a live SwsContext; FFmpeg returns a static coefficient
    // table and copies its values. No pointers escape this call.
    let result = unsafe {
        ffmpeg::ffi::sws_setColorspaceDetails(
            scaler.as_mut_ptr(),
            ffmpeg::ffi::sws_getCoefficients(input_matrix),
            i32::from(input_full),
            ffmpeg::ffi::sws_getCoefficients(output_matrix),
            i32::from(output_full),
            0,
            1 << 16,
            1 << 16,
        )
    };
    if result < 0 {
        bail!("FFmpeg 色彩范围配置失败：{}", ffmpeg::Error::from(result));
    }
    Ok(())
}

#[derive(Clone)]
pub struct VideoFrame {
    pub latest: Arc<CameraRawVideoFrame>,
    /// Complete dependency chain from the latest IDR. Watch receivers may skip
    /// updates, but must never send a P-frame without its references.
    pub gop: Vec<Arc<CameraRawVideoFrame>>,
    /// Preserve the just-completed GOP across an IDR publication. A viewer
    /// waking after that publication can still drain its unseen final frames.
    pub previous_gop: Vec<Arc<CameraRawVideoFrame>>,
    pub stream_id: u64,
    pub frames_per_second: f64,
}

pub struct Encoder {
    codec: ffmpeg::encoder::Video,
    scaler: Option<(Pixel, color::Space, color::Range, scaling::Context)>,
    fps: f64,
    stream_id: u64,
    origin_ns: Option<i64>,
    gop: Vec<Arc<CameraRawVideoFrame>>,
    previous_gop: Vec<Arc<CameraRawVideoFrame>>,
}

impl Encoder {
    pub fn new(width: u32, height: u32, fps: f64) -> Result<Self> {
        ffmpeg::init()?;
        let codec = ffmpeg::encoder::find_by_name("libx264")
            .ok_or_else(|| eyre!("FFmpeg 未提供 libx264 编码器"))?;
        let mut encoder = codec::context::Context::new_with_codec(codec)
            .encoder()
            .video()?;
        encoder.set_width(width);
        encoder.set_height(height);
        encoder.set_format(Pixel::YUV420P);
        encoder.set_time_base((1, 1_000_000));
        encoder.set_frame_rate(Some(((fps * 1000.0).round() as i32, 1000)));
        // One-second random access interval bounds reference storage and join
        // latency. This does not cap capture or perception sampling frequency.
        encoder.set_gop(fps.ceil() as u32);
        encoder.set_max_b_frames(0);
        encoder.set_colorspace(color::Space::BT709);
        encoder.set_color_range(color::Range::MPEG);
        let mut options = ffmpeg::Dictionary::new();
        options.set("preset", "veryfast");
        options.set("tune", "zerolatency");
        options.set("crf", "18");
        options.set("forced-idr", "1");
        // RGB image pixels use sRGB transfer; leaving this unspecified lets
        // browser color management choose a different display transfer curve.
        options.set("color_primaries", "bt709");
        options.set("color_trc", "iec61966-2-1");
        options.set("x264-params", "repeat-headers=1:annexb=1");
        let codec = encoder.open_with(options)?;
        Ok(Self {
            codec,
            scaler: None,
            fps,
            stream_id: NEXT_STREAM.fetch_add(1, Ordering::Relaxed),
            origin_ns: None,
            gop: vec![],
            previous_gop: vec![],
        })
    }

    pub fn matches(&self, image: &CameraImagePlane, fps: f64) -> bool {
        self.codec.width() == image.width && self.codec.height() == image.height && self.fps == fps
    }

    pub fn encode(&mut self, source: CameraRawVideoFrame, standalone: bool) -> Result<VideoFrame> {
        self.encode_decoded(NativeDecoder::default().decode(source)?, standalone)
    }

    pub fn encode_decoded(
        &mut self,
        decoded: DecodedFrame,
        standalone: bool,
    ) -> Result<VideoFrame> {
        let DecodedFrame {
            mut source,
            pixels: input,
        } = decoded;
        let width = source.color.width;
        let height = source.color.height;
        if width != self.codec.width()
            || height != self.codec.height()
            || (input.width(), input.height()) != (width, height)
        {
            bail!("相机帧尺寸与编码器不一致");
        }
        let format = input.format();
        let space = input.color_space();
        let range = input.color_range();
        if self
            .scaler
            .as_ref()
            .is_none_or(|(f, s, r, _)| (*f, *s, *r) != (format, space, range))
        {
            let mut scaler = scaling::Context::get(
                format,
                width,
                height,
                Pixel::YUV420P,
                width,
                height,
                scaling::Flags::BILINEAR,
            )?;
            colorspace(
                &mut scaler,
                matrix(space),
                ffmpeg::ffi::SWS_CS_ITU709,
                range == color::Range::JPEG,
                false,
            )?;
            self.scaler = Some((format, space, range, scaler));
        }
        let mut yuv = frame::Video::empty();
        self.scaler.as_mut().unwrap().3.run(&input, &mut yuv)?;
        let origin = *self.origin_ns.get_or_insert(source.device_time_ns);
        yuv.set_pts(Some((source.device_time_ns - origin) / 1000));
        // A device may negotiate 60 FPS but expose at 14 FPS. Bound random
        // access by capture time as well, not sixty frames (four seconds).
        let keyframe_due = self.gop.first().is_some_and(|first| {
            source.device_time_ns.saturating_sub(first.device_time_ns) >= 1_000_000_000
        });
        yuv.set_kind(if standalone || keyframe_due {
            ffmpeg::picture::Type::I
        } else {
            ffmpeg::picture::Type::None
        });
        self.codec.send_frame(&yuv)?;
        let mut packet = ffmpeg::Packet::empty();
        self.codec.receive_packet(&mut packet)?;
        if standalone && !packet.is_key() {
            bail!("H.264 采样帧缺少独立解码关键帧");
        }
        if packet.is_key() {
            self.previous_gop = std::mem::take(&mut self.gop);
        }
        source.color.stride_bytes = 0;
        source.color.pixel_format = "h264".into();
        source.color.data = packet
            .data()
            .ok_or_else(|| eyre!("H.264 编码器返回空帧"))?
            .to_vec();
        let latest = Arc::new(source);
        self.gop.push(latest.clone());
        Ok(VideoFrame {
            latest,
            gop: self.gop.clone(),
            previous_gop: self.previous_gop.clone(),
            stream_id: self.stream_id,
            frames_per_second: self.fps,
        })
    }
}

/// Decode a complete IDR-to-target packet chain, returning exactly its last
/// frame. FFmpeg owns reference pictures, chroma conversion and row alignment.
pub fn decode<'a>(images: impl IntoIterator<Item = &'a CameraImagePlane>) -> Result<Vec<u8>> {
    ffmpeg::init()?;
    let codec =
        ffmpeg::decoder::find(codec::Id::H264).ok_or_else(|| eyre!("FFmpeg 缺少 H.264 解码器"))?;
    let mut decoder = codec::context::Context::new_with_codec(codec)
        .decoder()
        .video()?;
    let mut last = frame::Video::empty();
    let mut dimensions = None;
    for image in images {
        if image.pixel_format != "h264" {
            bail!("彩色传输需要 H.264，收到 {}", image.pixel_format);
        }
        dimensions = Some((image.width, image.height));
        decoder.send_packet(&ffmpeg::Packet::copy(&image.data))?;
        // zerolatency / no B-frames: each complete access unit produces its own
        // picture, never return a previous image for an undecodable packet.
        decoder.receive_frame(&mut last)?;
    }
    let (width, height) = dimensions.ok_or_else(|| eyre!("H.264 帧链为空"))?;
    if last.width() != width || last.height() != height {
        bail!("H.264 解码尺寸与帧元数据不一致");
    }
    let mut scaler = scaling::Context::get(
        last.format(),
        width,
        height,
        Pixel::RGB24,
        width,
        height,
        scaling::Flags::BILINEAR,
    )?;
    let mut rgb = frame::Video::empty();
    colorspace(
        &mut scaler,
        ffmpeg::ffi::SWS_CS_ITU709,
        ffmpeg::ffi::SWS_CS_ITU709,
        false,
        true,
    )?;
    scaler.run(&last, &mut rgb)?;
    Ok(pack_rgb(&rgb))
}

fn pack_rgb(frame: &frame::Video) -> Vec<u8> {
    let row_bytes = frame.width() as usize * 3;
    let mut pixels = Vec::with_capacity(row_bytes * frame.height() as usize);
    for row in frame
        .data(0)
        .chunks(frame.stride(0))
        .take(frame.height() as usize)
    {
        pixels.extend_from_slice(&row[..row_bytes]);
    }
    pixels
}

pub fn decode_image(image: &CameraImagePlane) -> Result<Vec<u8>> {
    decode(std::iter::once(image))
}

/// FFmpeg's decoded frame stays in its native pixel format (usually YUV for
/// MJPEG). No RGB roundtrip or packed RGB copy on the video transcoding path.
pub struct DecodedFrame {
    pub source: CameraRawVideoFrame,
    pixels: frame::Video,
}

#[derive(Default)]
pub struct NativeDecoder {
    jpeg: Option<ffmpeg::decoder::Video>,
}

impl NativeDecoder {
    pub fn decode(&mut self, mut source: CameraRawVideoFrame) -> Result<DecodedFrame> {
        let image = &source.color;
        let pixels = self.pixels(
            image.width,
            image.height,
            image.stride_bytes,
            &image.pixel_format,
            &image.data,
        )?;
        if (pixels.width(), pixels.height()) != (image.width, image.height) {
            bail!("相机原生帧尺寸与所选配置不一致");
        }
        // The AVFrame owns the decoded planes; the original packet is no longer
        // needed. Only small metadata travels alongside it to the encoder.
        source.color.data = Vec::new();
        Ok(DecodedFrame { source, pixels })
    }

    fn pixels(
        &mut self,
        width: u32,
        height: u32,
        stride: u32,
        format: &str,
        bytes: &[u8],
    ) -> Result<frame::Video> {
        ffmpeg::init()?;
        let mut input = if matches!(format, "MJPG" | "JPEG") {
            if self.jpeg.is_none() {
                let codec = ffmpeg::decoder::find(codec::Id::MJPEG)
                    .ok_or_else(|| eyre!("FFmpeg 缺少 MJPEG 解码器"))?;
                self.jpeg = Some(
                    codec::context::Context::new_with_codec(codec)
                        .decoder()
                        .video()?,
                );
            }
            let decoder = self.jpeg.as_mut().unwrap();
            decoder.send_packet(&ffmpeg::Packet::copy(bytes))?;
            let mut input = frame::Video::empty();
            decoder.receive_frame(&mut input)?;
            input
        } else {
            let (pixel, channels) = match format {
                "RGB3" | "rgb8" => (Pixel::RGB24, 3),
                "BGR3" | "bgr8" => (Pixel::BGR24, 3),
                "rgba8" => (Pixel::RGBA, 4),
                "bgra8" => (Pixel::BGRA, 4),
                "GREY" | "y8" => (Pixel::GRAY8, 1),
                "YUYV" | "yuyv" => (Pixel::YUYV422, 2),
                "UYVY" | "uyvy" => (Pixel::UYVY422, 2),
                "NV12" => (Pixel::NV12, 1),
                _ => bail!("未支持的原生相机格式 {format}"),
            };
            let row_bytes = width as usize * channels;
            let source_stride = if stride == 0 {
                row_bytes
            } else {
                stride as usize
            };
            if width == 0 || height == 0 || source_stride < row_bytes {
                bail!("无效相机尺寸或行步长");
            }
            let mut input = frame::Video::new(pixel, width, height);
            // Raw RGB values are full-range sRGB. Native YUV defaults to the V4L2
            // limited-range SD matrix unless the decoded frame declares otherwise.
            if matches!(
                pixel,
                Pixel::RGB24 | Pixel::BGR24 | Pixel::RGBA | Pixel::BGRA | Pixel::GRAY8
            ) {
                input.set_color_range(color::Range::JPEG);
                input.set_color_space(color::Space::BT709);
            }
            let mut offset = 0;
            for plane in 0..input.planes() {
                let rows = input.plane_height(plane) as usize;
                let target_stride = input.stride(plane);
                for y in 0..rows {
                    let start = offset + y * source_stride;
                    let source = bytes
                        .get(start..start + row_bytes)
                        .ok_or_else(|| eyre!("相机图像载荷不足"))?;
                    input.data_mut(plane)[y * target_stride..y * target_stride + row_bytes]
                        .copy_from_slice(source);
                }
                offset += source_stride * rows;
            }
            input
        };
        // YUVJ means full-range JPEG, not a different plane layout. Use FFmpeg's
        // modern format + explicit range instead of generating a deprecation log
        // for every camera frame. Never treat JPEG's full range as limited video.
        let jpeg_format = match input.format() {
            Pixel::YUVJ420P => Some(Pixel::YUV420P),
            Pixel::YUVJ422P => Some(Pixel::YUV422P),
            Pixel::YUVJ444P => Some(Pixel::YUV444P),
            Pixel::YUVJ440P => Some(Pixel::YUV440P),
            _ => None,
        };
        if let Some(format) = jpeg_format {
            input.set_format(format);
            input.set_color_range(color::Range::JPEG);
        }
        Ok(input)
    }
}

fn matrix(space: color::Space) -> i32 {
    match space {
        color::Space::BT709 => ffmpeg::ffi::SWS_CS_ITU709,
        color::Space::BT2020NCL | color::Space::BT2020CL => ffmpeg::ffi::SWS_CS_BT2020,
        _ => ffmpeg::ffi::SWS_CS_DEFAULT,
    }
}

/// Explicit pixel access, used by image consumers and diagnostics, never the
/// video transcode worker. All pixel formulas remain inside FFmpeg.
pub fn native_rgb(
    width: u32,
    height: u32,
    stride: u32,
    format: &str,
    bytes: &[u8],
) -> Result<CameraImagePlane> {
    let input = NativeDecoder::default().pixels(width, height, stride, format, bytes)?;
    let (width, height) = (input.width(), input.height());
    let mut scaler = scaling::Context::get(
        input.format(),
        width,
        height,
        Pixel::RGB24,
        width,
        height,
        scaling::Flags::BILINEAR,
    )?;
    colorspace(
        &mut scaler,
        matrix(input.color_space()),
        matrix(input.color_space()),
        input.color_range() == color::Range::JPEG,
        true,
    )?;
    let mut rgb = frame::Video::empty();
    scaler.run(&input, &mut rgb)?;
    Ok(CameraImagePlane {
        width,
        height,
        stride_bytes: width * 3,
        pixel_format: "rgb8".into(),
        frame_id: "color_optical_frame".into(),
        data: pack_rgb(&rgb),
    })
}
