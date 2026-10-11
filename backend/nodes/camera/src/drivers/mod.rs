mod simulation;

#[cfg(feature = "realsense-runtime")]
mod realsense;

use eyre::Result;
use robot_arm_messages::{
    CameraDriverParameterValue, CameraFrameBundle, CameraSourceInfo, CameraStreamProfile, ToolPose,
};

pub(crate) use simulation::SimulationDriver;

#[cfg(feature = "realsense-runtime")]
pub(crate) use realsense::RealSenseDriver;

pub(crate) trait CameraStream {
    fn poll_frame(&mut self, materialize: bool) -> Result<CameraPoll>;
}

pub(crate) enum CameraPoll {
    Pending,
    Captured {
        video_frame: Box<camera_media::DecodedFrame>,
        frame: Option<Box<CameraFrameBundle>>,
    },
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
    pub(crate) fn open_color(
        &mut self,
        source_id: &str,
        color: &CameraStreamProfile,
    ) -> Result<Box<dyn CameraStream>> {
        if source_id.starts_with("v4l2:") {
            return Ok(Box::new(ColorStream(uvc_camera::Stream::open(
                source_id, color,
            )?)));
        }
        // RGB-D drivers can supply their existing color plane too. They remain
        // the only owner of native capture; no fabricated depth or intrinsics.
        let (sources, _) = self.discover();
        let source = sources
            .iter()
            .find(|s| s.source_id == source_id)
            .ok_or_else(|| eyre::eyre!("相机未连接"))?;
        let depth = source
            .profiles
            .iter()
            .find(|p| {
                p.stream == robot_arm_messages::CameraStreamKind::Depth
                    && p.available
                    && p.width == color.width
                    && p.height == color.height
                    && p.frames_per_second == color.frames_per_second
            })
            .or_else(|| {
                source.profiles.iter().find(|p| {
                    p.stream == robot_arm_messages::CameraStreamKind::Depth && p.available
                })
            })
            .ok_or_else(|| eyre::eyre!("驱动没有可用采集配置"))?;
        self.open(source_id, color, depth, &[])
    }
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
        let (ordinary, ordinary_errors) = uvc_camera::discover();
        sources.extend(ordinary);
        errors.extend(ordinary_errors);
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

struct ColorStream(uvc_camera::Stream);
impl CameraStream for ColorStream {
    fn poll_frame(&mut self, _materialize: bool) -> Result<CameraPoll> {
        let Some(frame) = self.0.next_frame()? else {
            return Ok(CameraPoll::Pending);
        };
        Ok(CameraPoll::Captured {
            video_frame: Box::new(frame),
            frame: None,
        })
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
            let frame =
                camera_media::native_rgb(3, 2, (3 * channels + 5) as u32, format, &data).unwrap();
            assert_eq!(frame.pixel_format, "rgb8");
            assert_eq!(frame.stride_bytes, frame.width * 3);
            assert_eq!(frame.data, expected, "{format}");

            #[cfg(feature = "opencv-runtime")]
            {
                use opencv::{core, imgcodecs, prelude::*};
                // Exercise the codec boundary independently of snapshot helpers.
                let image =
                    image::RgbImage::from_raw(frame.width, frame.height, frame.data).unwrap();
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
