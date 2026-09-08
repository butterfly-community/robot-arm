mod simulation;

#[cfg(feature = "realsense-runtime")]
mod realsense;

use eyre::{Result, bail};
use opencv::{core, imgproc, prelude::*};
use robot_arm_messages::{
    CameraDriverParameterValue, CameraFrameBundle, CameraImagePlane, CameraRawVideoFrame,
    CameraSourceInfo, CameraStreamProfile, ToolPose,
};

pub(crate) use simulation::SimulationDriver;

#[cfg(feature = "realsense-runtime")]
pub(crate) use realsense::RealSenseDriver;

pub(crate) trait CameraStream {
    fn poll_frame(&mut self, materialize: bool) -> Result<CameraPoll>;
}

#[derive(Clone)]
pub(crate) enum CameraPoll {
    Pending,
    Captured {
        sequence: u64,
        received_time_ns: i64,
        video_frame: Box<CameraRawVideoFrame>,
        frame: Option<Box<CameraFrameBundle>>,
    },
}

impl CameraPoll {
    /// Normalize once at the acquisition boundary, before either publication.
    pub(crate) fn into_rgb(mut self) -> Result<Self> {
        if let Self::Captured {
            video_frame, frame, ..
        } = &mut self
        {
            normalize_rgb(&mut video_frame.color)?;
            if let Some(frame) = frame {
                normalize_rgb(&mut frame.color)?;
            }
        }
        Ok(self)
    }
}

fn normalize_rgb(image: &mut CameraImagePlane) -> Result<()> {
    image.validate_layout("driver color")?;
    let (channels, conversion) = match image.pixel_format.as_str() {
        "rgb8" => (3, None),
        "bgr8" => (3, Some(imgproc::COLOR_BGR2RGB)),
        "rgba8" => (4, Some(imgproc::COLOR_RGBA2RGB)),
        "bgra8" => (4, Some(imgproc::COLOR_BGRA2RGB)),
        "y8" => (1, Some(imgproc::COLOR_GRAY2RGB)),
        format => bail!("不支持的驱动彩色格式 {format}"),
    };
    let packed = image.width as usize * channels as usize;
    if (image.stride_bytes as usize) < packed {
        bail!("驱动彩色图 stride 小于像素行宽");
    }
    let Some(conversion) = conversion else {
        return Ok(());
    };
    // The byte-row ROI retains arbitrary native row padding. Reshape changes
    // only channel/column interpretation; cvtColor owns the channel conversion.
    let data = {
        let bytes = core::Mat::from_slice(&image.data)?;
        let rows = bytes.reshape(1, i32::try_from(image.height)?)?;
        let pixels = core::Mat::roi(
            &rows,
            core::Rect::new(0, 0, i32::try_from(packed)?, i32::try_from(image.height)?),
        )?;
        let native = pixels.reshape(channels, 0)?;
        let mut rgb = core::Mat::default();
        imgproc::cvt_color_def(&native, &mut rgb, conversion)?;
        rgb.data_bytes()?.to_vec()
    };
    image.data = data;
    image.stride_bytes = image.width * 3;
    image.pixel_format = "rgb8".into();
    Ok(())
}

pub(crate) trait CameraDriver {
    fn discover(&mut self) -> Result<Vec<CameraSourceInfo>>;

    fn open(
        &mut self,
        source_id: &str,
        color_profile: &CameraStreamProfile,
        depth_profile: &CameraStreamProfile,
        driver_parameters: &[CameraDriverParameterValue],
    ) -> Result<Box<dyn CameraStream>>;

    fn reset(&mut self, _source_id: &str) -> Result<()> {
        Ok(())
    }
}

pub(crate) struct Drivers {
    simulation: SimulationDriver,
    #[cfg(feature = "realsense-runtime")]
    realsense: RealSenseDriver,
}

impl Drivers {
    pub(crate) fn new() -> Result<Self> {
        Ok(Self {
            simulation: SimulationDriver::new()?,
            #[cfg(feature = "realsense-runtime")]
            realsense: RealSenseDriver::new()?,
        })
    }

    pub(crate) fn discover(&mut self) -> (Vec<CameraSourceInfo>, Vec<String>) {
        let mut sources = Vec::new();
        let mut errors = Vec::new();
        collect_discovery(
            &mut sources,
            &mut errors,
            self.simulation.discover(),
            "simulation",
        );
        #[cfg(feature = "realsense-runtime")]
        collect_discovery(
            &mut sources,
            &mut errors,
            self.realsense.discover(),
            "librealsense2",
        );
        (sources, errors)
    }

    pub(crate) fn open(
        &mut self,
        source_id: &str,
        color_profile: &CameraStreamProfile,
        depth_profile: &CameraStreamProfile,
        driver_parameters: &[CameraDriverParameterValue],
    ) -> Result<Box<dyn CameraStream>> {
        match source_id.split_once(':').map(|value| value.0) {
            Some("simulation") => {
                self.simulation
                    .open(source_id, color_profile, depth_profile, driver_parameters)
            }
            #[cfg(feature = "realsense-runtime")]
            Some("realsense") => CameraDriver::open(
                &mut self.realsense,
                source_id,
                color_profile,
                depth_profile,
                driver_parameters,
            ),
            _ => eyre::bail!("未知相机来源 {source_id}"),
        }
    }

    pub(crate) fn reset(&mut self, source_id: &str) -> Result<()> {
        match source_id.split_once(':').map(|value| value.0) {
            Some("simulation") => self.simulation.reset(source_id),
            #[cfg(feature = "realsense-runtime")]
            Some("realsense") => self.realsense.reset(source_id),
            _ => eyre::bail!("未知相机来源 {source_id}"),
        }
    }

    pub(crate) fn update_tool_pose(&mut self, pose: ToolPose) {
        self.simulation.update_tool_pose(pose);
    }

    pub(crate) fn set_calibration_active(&mut self, active: bool) {
        self.simulation.set_calibration_active(active);
    }
}

fn collect_discovery(
    output: &mut Vec<CameraSourceInfo>,
    errors: &mut Vec<String>,
    result: Result<Vec<CameraSourceInfo>>,
    driver: &str,
) {
    match result {
        Ok(values) => output.extend(values),
        Err(error) => errors.push(format!("{driver}: {error:#}")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn native_colour_normalizes_once_and_opencv_codecs_preserve_rgb() {
        let colours = [
            [255, 0, 0],
            [0, 255, 0],
            [0, 0, 255],
            [17, 83, 201],
            [255, 255, 255],
            [0, 0, 0],
        ];
        for (format, channels) in [
            ("rgb8", 3),
            ("bgr8", 3),
            ("rgba8", 4),
            ("bgra8", 4),
            ("y8", 1),
        ] {
            let mut data = Vec::new();
            let mut expected = Vec::new();
            for row in colours.chunks_exact(3) {
                for &[r, g, b] in row {
                    let rgb = if format == "y8" { [r, r, r] } else { [r, g, b] };
                    expected.extend(rgb);
                    let native = match format {
                        "bgr8" | "bgra8" => [b, g, r, 255],
                        _ => [r, g, b, 255],
                    };
                    data.extend_from_slice(&native[..channels]);
                }
                data.extend([91; 5]); // padding need not be divisible by channels
            }
            let mut frame = CameraImagePlane {
                width: 3,
                height: 2,
                stride_bytes: (3 * channels + 5) as u32,
                pixel_format: format.into(),
                frame_id: "optical".into(),
                data,
            };
            normalize_rgb(&mut frame).unwrap();
            assert_eq!(frame.pixel_format, "rgb8");
            assert_eq!(frame.frame_id, "optical");
            assert_eq!(frame.packed_rgb().unwrap(), expected, "{format}");
            let once = frame.clone();
            normalize_rgb(&mut frame).unwrap();
            assert_eq!(frame, once, "normalizing RGB must not swap it again");

            #[cfg(feature = "opencv-runtime")]
            {
                use opencv::imgcodecs;
                // Exercise the codec boundary independently of snapshot helpers.
                let image = image::RgbImage::from_raw(
                    frame.width,
                    frame.height,
                    frame.packed_rgb().unwrap(),
                )
                .unwrap();
                let mut png = std::io::Cursor::new(Vec::new());
                image.write_to(&mut png, image::ImageFormat::Png).unwrap();
                let encoded = core::Vector::from_slice(png.get_ref());
                let rgb = imgcodecs::imdecode(&encoded, imgcodecs::IMREAD_COLOR_RGB).unwrap();
                assert_eq!(rgb.data_bytes().unwrap(), expected);
                let bgr = imgcodecs::imdecode(&encoded, imgcodecs::IMREAD_COLOR_BGR).unwrap();
                let expected_bgr = expected
                    .chunks_exact(3)
                    .flat_map(|p| [p[2], p[1], p[0]])
                    .collect::<Vec<_>>();
                assert_eq!(bgr.data_bytes().unwrap(), expected_bgr);
                let mut output = core::Vector::<u8>::new();
                imgcodecs::imencode_def(".png", &bgr, &mut output).unwrap();
                let decoded = image::load_from_memory(output.as_slice())
                    .unwrap()
                    .into_rgb8();
                assert_eq!(decoded.into_raw(), expected, "OpenCV BGR -> PNG -> RGB");
            }
        }
    }
}
