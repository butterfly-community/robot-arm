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
    fn next_frameset(&mut self) -> Result<Option<CameraFrameBundle>>;
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
