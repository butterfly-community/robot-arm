use eyre::Result;
use realsense_camera::RealSenseStream;
use robot_arm_messages::{CameraDriverParameterValue, CameraSourceInfo, CameraStreamProfile};

use super::{CameraDriver, CameraPoll, CameraStream};

pub(crate) use realsense_camera::RealSenseDriver;

impl CameraDriver for RealSenseDriver {
    fn discover(&mut self) -> Result<Vec<CameraSourceInfo>> {
        RealSenseDriver::discover(self)
    }

    fn open(
        &mut self,
        source_id: &str,
        color_profile: &CameraStreamProfile,
        depth_profile: &CameraStreamProfile,
        driver_parameters: &[CameraDriverParameterValue],
    ) -> Result<Box<dyn CameraStream>> {
        Ok(Box::new(RealSenseDriver::open(
            self,
            source_id,
            color_profile,
            depth_profile,
            driver_parameters,
        )?))
    }
}

impl CameraStream for RealSenseStream {
    fn poll_frame(&mut self, materialize: bool) -> Result<CameraPoll> {
        RealSenseStream::poll_frame(self, materialize).map(|poll| match poll {
            realsense_camera::FramePoll::Pending => CameraPoll::Pending,
            realsense_camera::FramePoll::Captured {
                sequence,
                received_time_ns,
                video_frame,
                frame,
            } => CameraPoll::Captured {
                sequence,
                received_time_ns,
                video_frame,
                frame,
            },
        })
    }
}
