use std::io::Cursor;

use eyre::{Result, eyre};
use image::{DynamicImage, GrayImage, ImageFormat, Luma, RgbImage, imageops::FilterType};
use robot_arm_messages::{
    AlignedDepthFrame, DepthCameraCalibration, DepthCameraSourceInfo, DetectedInstance2D,
    SCHEMA_VERSION,
};
use serde::Deserialize;

use crate::ros::ros_time;

const SIMULATION_CONFIG: &str = include_str!("../test-assets/simulation-cameras.json");
const PICK_PLACE_RGB: &[u8] = include_bytes!("../test-assets/pick-place-scene.png");

#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum SimulationKind {
    PickPlace,
    DepthGrid,
}

#[derive(Clone, Deserialize)]
struct SimulationCameraConfig {
    source_id: String,
    display_name: String,
    simulation: SimulationKind,
    color_stream: String,
    depth_stream: String,
    camera_info_stream: String,
    depth_scale_m: f64,
    calibration: SimulationCalibration,
}

#[derive(Clone, Deserialize)]
struct SimulationCalibration {
    parent_frame_id: String,
    frame_id: String,
    translation_m: [f64; 3],
    orientation_xyzw: [f64; 4],
    width: u32,
    height: u32,
    distortion_model: String,
    distortion: Vec<f64>,
    camera_matrix: [f64; 9],
    projection_matrix: [f64; 12],
}

pub(super) struct SimulationFrame {
    pub color: r2r::sensor_msgs::msg::Image,
    pub depth: r2r::sensor_msgs::msg::Image,
    pub camera_info: r2r::sensor_msgs::msg::CameraInfo,
    pub calibration: DepthCameraCalibration,
}

pub(super) fn simulation_sources() -> Result<Vec<DepthCameraSourceInfo>> {
    Ok(configs()?
        .into_iter()
        .map(|config| DepthCameraSourceInfo {
            source_id: config.source_id,
            driver_id: "simulation".into(),
            display_name: config.display_name,
            color_stream: config.color_stream,
            depth_stream: config.depth_stream,
            camera_info_stream: config.camera_info_stream,
            depth_scale_m: config.depth_scale_m,
            calibrated: true,
        })
        .collect())
}

pub(super) fn simulation_frame(
    source_id: &str,
    sequence: u64,
    now_ns: i64,
    depth_scale_m: f64,
) -> Result<Option<SimulationFrame>> {
    let Some(config) = configs()?
        .into_iter()
        .find(|config| config.source_id == source_id)
    else {
        return Ok(None);
    };
    let calibration = calibration(&config, sequence, now_ns);
    let (color, depth, _) = match config.simulation {
        SimulationKind::PickPlace => pick_place_rgbd(&config, sequence, now_ns, depth_scale_m)?,
        SimulationKind::DepthGrid => depth_grid_rgbd(&config, sequence, now_ns, depth_scale_m),
    };
    Ok(Some(SimulationFrame {
        color,
        depth: depth_image(&depth),
        camera_info: camera_info(&calibration),
        calibration,
    }))
}

pub(super) fn simulation_source(source_id: &str) -> Result<Option<DepthCameraSourceInfo>> {
    Ok(simulation_sources()?
        .into_iter()
        .find(|source| source.source_id == source_id))
}

fn configs() -> Result<Vec<SimulationCameraConfig>> {
    Ok(serde_json::from_str(SIMULATION_CONFIG)?)
}

fn calibration(
    config: &SimulationCameraConfig,
    sequence: u64,
    now_ns: i64,
) -> DepthCameraCalibration {
    let value = &config.calibration;
    DepthCameraCalibration {
        schema_version: SCHEMA_VERSION,
        sequence,
        source_time_ns: now_ns,
        source_id: config.source_id.clone(),
        parent_frame_id: value.parent_frame_id.clone(),
        frame_id: value.frame_id.clone(),
        translation_m: value.translation_m,
        orientation_xyzw: value.orientation_xyzw,
        width: value.width,
        height: value.height,
        distortion_model: value.distortion_model.clone(),
        distortion: value.distortion.clone(),
        camera_matrix: value.camera_matrix,
        projection_matrix: value.projection_matrix,
    }
}

fn camera_info(calibration: &DepthCameraCalibration) -> r2r::sensor_msgs::msg::CameraInfo {
    r2r::sensor_msgs::msg::CameraInfo {
        header: r2r::std_msgs::msg::Header {
            stamp: ros_time(calibration.source_time_ns),
            frame_id: calibration.frame_id.clone(),
        },
        height: calibration.height,
        width: calibration.width,
        distortion_model: calibration.distortion_model.clone(),
        d: calibration.distortion.clone(),
        k: calibration.camera_matrix.to_vec(),
        r: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p: calibration.projection_matrix.to_vec(),
        binning_x: 0,
        binning_y: 0,
        roi: r2r::sensor_msgs::msg::RegionOfInterest {
            x_offset: 0,
            y_offset: 0,
            height: 0,
            width: 0,
            do_rectify: false,
        },
    }
}

fn pick_place_rgbd(
    config: &SimulationCameraConfig,
    sequence: u64,
    now_ns: i64,
    depth_scale_m: f64,
) -> Result<(
    r2r::sensor_msgs::msg::Image,
    AlignedDepthFrame,
    Vec<DetectedInstance2D>,
)> {
    let width = config.calibration.width;
    let height = config.calibration.height;
    let rgb = image::load_from_memory(PICK_PLACE_RGB)?
        .resize_exact(width, height, FilterType::Lanczos3)
        .to_rgb8();
    let cube_mask = polygon_mask(
        width,
        height,
        &[
            (80, 265),
            (101, 230),
            (151, 217),
            (171, 239),
            (170, 297),
            (136, 318),
            (87, 307),
        ],
    );
    let bin_mask = polygon_mask(
        width,
        height,
        &[
            (264, 219),
            (312, 127),
            (377, 119),
            (567, 165),
            (580, 221),
            (564, 309),
            (504, 359),
            (293, 315),
            (263, 271),
        ],
    );
    let color = color_image(rgb, &config.calibration.frame_id, now_ns);
    let mut values = vec![0_u16; (width * height) as usize];
    fill_masked_depth(&mut values, &cube_mask, 590, 670);
    fill_masked_depth(&mut values, &bin_mask, 590, 670);
    let depth = aligned_depth(config, sequence, now_ns, depth_scale_m, values);
    let instances = vec![
        simulated_instance("red-cube-0", "red cube", cube_mask)?,
        simulated_instance("gray-storage-bin-0", "gray storage bin", bin_mask)?,
    ];
    Ok((color, depth, instances))
}

fn depth_grid_rgbd(
    config: &SimulationCameraConfig,
    sequence: u64,
    now_ns: i64,
    depth_scale_m: f64,
) -> (
    r2r::sensor_msgs::msg::Image,
    AlignedDepthFrame,
    Vec<DetectedInstance2D>,
) {
    let width = config.calibration.width;
    let height = config.calibration.height;
    let color = color_image(
        RgbImage::new(width, height),
        &config.calibration.frame_id,
        now_ns,
    );
    let mut values = vec![0_u16; (width * height) as usize];
    for y in 210..=230 {
        for x in 310..=330 {
            values[(y * width + x) as usize] = 400;
        }
    }
    for y in 210..=217 {
        for x in 350..=356 {
            values[(y * width + x) as usize] = 350 + (y - 210) as u16 * 10;
        }
    }
    (
        color,
        aligned_depth(config, sequence, now_ns, depth_scale_m, values),
        vec![],
    )
}

fn color_image(rgb: RgbImage, frame_id: &str, now_ns: i64) -> r2r::sensor_msgs::msg::Image {
    let width = rgb.width();
    let height = rgb.height();
    r2r::sensor_msgs::msg::Image {
        header: r2r::std_msgs::msg::Header {
            stamp: ros_time(now_ns),
            frame_id: frame_id.into(),
        },
        height,
        width,
        encoding: "rgb8".into(),
        is_bigendian: 0,
        step: width * 3,
        data: rgb.into_raw(),
    }
}

fn aligned_depth(
    config: &SimulationCameraConfig,
    sequence: u64,
    now_ns: i64,
    depth_scale_m: f64,
    depth: Vec<u16>,
) -> AlignedDepthFrame {
    AlignedDepthFrame {
        schema_version: SCHEMA_VERSION,
        sequence,
        source_time_ns: now_ns,
        source_id: config.source_id.clone(),
        frame_id: config.calibration.frame_id.clone(),
        width: config.calibration.width,
        height: config.calibration.height,
        depth_scale_m,
        depth,
    }
}

fn depth_image(depth: &AlignedDepthFrame) -> r2r::sensor_msgs::msg::Image {
    r2r::sensor_msgs::msg::Image {
        header: r2r::std_msgs::msg::Header {
            stamp: ros_time(depth.source_time_ns),
            frame_id: depth.frame_id.clone(),
        },
        height: depth.height,
        width: depth.width,
        encoding: "16UC1".into(),
        is_bigendian: 0,
        step: depth.width * 2,
        data: depth
            .depth
            .iter()
            .flat_map(|value| value.to_le_bytes())
            .collect(),
    }
}

fn simulated_instance(id: &str, label: &str, mask: GrayImage) -> Result<DetectedInstance2D> {
    let bounds = mask_bounds(&mask).ok_or_else(|| eyre!("simulation mask is empty"))?;
    let width = mask.width();
    let height = mask.height();
    let mut output = Cursor::new(Vec::new());
    DynamicImage::ImageLuma8(mask).write_to(&mut output, ImageFormat::Png)?;
    Ok(DetectedInstance2D {
        instance_id: id.into(),
        label: label.into(),
        confidence: 1.0,
        bounding_box_xyxy: bounds.map(f64::from),
        mask_width: width,
        mask_height: height,
        mask_png: output.into_inner(),
    })
}

fn polygon_mask(width: u32, height: u32, polygon: &[(u32, u32)]) -> GrayImage {
    let mut mask = GrayImage::new(width, height);
    for y in 0..height {
        for x in 0..width {
            let mut inside = false;
            let mut previous = polygon.len() - 1;
            for current in 0..polygon.len() {
                let (x1, y1) = polygon[current];
                let (x2, y2) = polygon[previous];
                if (y1 > y) != (y2 > y)
                    && f64::from(x)
                        < (f64::from(x2) - f64::from(x1)) * (f64::from(y) - f64::from(y1))
                            / (f64::from(y2) - f64::from(y1))
                            + f64::from(x1)
                {
                    inside = !inside;
                }
                previous = current;
            }
            if inside {
                mask.put_pixel(x, y, Luma([255]));
            }
        }
    }
    mask
}

fn mask_bounds(mask: &GrayImage) -> Option<[u32; 4]> {
    let mut bounds = [u32::MAX, u32::MAX, 0, 0];
    let mut found = false;
    for (x, y, value) in mask.enumerate_pixels() {
        if value[0] != 0 {
            bounds = [
                bounds[0].min(x),
                bounds[1].min(y),
                bounds[2].max(x),
                bounds[3].max(y),
            ];
            found = true;
        }
    }
    found.then_some(bounds)
}

fn fill_masked_depth(depth: &mut [u16], mask: &GrayImage, near: u16, far: u16) {
    let [_, minimum_y, _, maximum_y] = mask_bounds(mask).expect("simulation mask has bounds");
    let span = maximum_y - minimum_y;
    for (x, y, mask_value) in mask.enumerate_pixels() {
        if mask_value[0] != 0 {
            let offset = (u32::from(far - near) * (y - minimum_y))
                .checked_div(span)
                .unwrap_or_default() as u16;
            depth[(y * mask.width() + x) as usize] = near + offset;
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;
    use perception_core::world_scene_and_instance_clouds_from_aligned_depth;

    use super::*;

    #[test]
    fn simulation_cameras_are_declared_by_the_prebuilt_config() {
        let sources = simulation_sources().unwrap();
        assert_eq!(sources.len(), 2);
        assert!(
            sources
                .iter()
                .all(|source| source.driver_id == "simulation")
        );
        assert!(sources.iter().all(|source| source.calibrated));
    }

    #[test]
    fn depth_grid_is_a_standard_rgbd_camera_frame() {
        let frame = simulation_frame("simulation:depth-grid", 1, 2, 0.001)
            .unwrap()
            .unwrap();
        assert_eq!(frame.depth.encoding, "16UC1");
        assert_eq!(
            frame
                .depth
                .data
                .chunks_exact(2)
                .filter(|bytes| u16::from_le_bytes([bytes[0], bytes[1]]) != 0)
                .count(),
            497
        );
        assert_eq!(
            frame.camera_info.k,
            frame.calibration.camera_matrix.to_vec()
        );
    }

    #[test]
    fn simulated_pick_and_place_points_share_one_support_plane() {
        let mut configs = configs().unwrap();
        let config = configs.remove(0);
        let (_, depth, instances) = pick_place_rgbd(&config, 1, 2, 0.001).unwrap();
        let calibration = calibration(&config, 1, 2);
        let (scene, _) = world_scene_and_instance_clouds_from_aligned_depth(
            1,
            &depth,
            &calibration,
            &instances,
            &["gray storage bin".into()],
        )
        .unwrap();
        let cube = scene
            .objects
            .iter()
            .find(|object| object.label == "red cube")
            .unwrap();
        let bin = scene
            .objects
            .iter()
            .find(|object| object.label == "gray storage bin")
            .unwrap();
        let placement = &scene.placement_regions[0];
        let cube_bottom = cube.pose.position_m[2] - cube.size_m[2] / 2.0;
        let bin_bottom = bin.pose.position_m[2] - bin.size_m[2] / 2.0;
        assert_abs_diff_eq!(cube_bottom, bin_bottom);
        assert_abs_diff_eq!(cube_bottom, 0.0);
        assert_abs_diff_eq!(placement.pose.position_m[2], bin.pose.position_m[2]);
        assert_abs_diff_eq!(cube.pose.position_m[2], 0.04);
        assert_abs_diff_eq!(bin.pose.position_m[2], 0.04);
        assert_abs_diff_eq!(bin.size_m[2], 0.08);
    }
}
