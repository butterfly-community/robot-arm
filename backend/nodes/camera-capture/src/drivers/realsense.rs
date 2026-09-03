use eyre::Result;
use realsense_camera::RealSenseStream;
use robot_arm_messages::{
    CameraDriverParameterValue, CameraFrameBundle, CameraSourceInfo, CameraStreamProfile,
};

use super::{CameraDriver, CameraStream};

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
    fn next_frameset(&mut self) -> Result<Option<CameraFrameBundle>> {
        RealSenseStream::next_frameset(self)
    }
}
