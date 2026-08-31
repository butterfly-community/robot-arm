use std::io::Cursor;

use eyre::{Result, eyre};
use image::{DynamicImage, GrayImage, ImageFormat, Luma, imageops::FilterType};
use robot_arm_messages::{
    AlignedDepthFrame, DepthCameraCalibration, DepthPointCloudFrame, DetectedInstance2D,
    SCHEMA_VERSION,
};

use crate::ros::ros_time;

const GENERATED_RGB_ASSET: &[u8] =
    include_bytes!("../../../../tools/perception/assets/pick-place-scene.png");
const GENERATED_DEPTH_SOURCE_ID: &str = "generated-test-depth-scene";
const GENERATED_DEPTH_FRAME_ID: &str = "depth_sim_frame";

pub(super) fn generated_depth_test_cloud(sequence: u64, now_ns: i64) -> DepthPointCloudFrame {
    let mut points = Vec::with_capacity(497);
    for x in -10..=10 {
        for y in 30..=50 {
            points.push([x as f32 * 0.01, y as f32 * 0.01, 0.0]);
        }
    }
    for x in -3..=3 {
        for z in 1..=8 {
            points.push([x as f32 * 0.01, 0.4, z as f32 * 0.01]);
        }
    }
    DepthPointCloudFrame {
        schema_version: SCHEMA_VERSION,
        sequence,
        source_time_ns: now_ns,
        source_id: GENERATED_DEPTH_SOURCE_ID.into(),
        frame_id: GENERATED_DEPTH_FRAME_ID.into(),
        width: points.len() as u32,
        height: 1,
        points_xyz_m: points,
    }
}

pub(super) fn generated_depth_test_calibration(
    sequence: u64,
    now_ns: i64,
) -> DepthCameraCalibration {
    DepthCameraCalibration {
        schema_version: SCHEMA_VERSION,
        sequence,
        source_time_ns: now_ns,
        source_id: GENERATED_DEPTH_SOURCE_ID.into(),
        parent_frame_id: "base_link".into(),
        frame_id: GENERATED_DEPTH_FRAME_ID.into(),
        translation_m: [0.0; 3],
        orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        width: 640,
        height: 480,
        distortion_model: "plumb_bob".into(),
        distortion: vec![0.0; 5],
        camera_matrix: [500.0, 0.0, 319.5, 0.0, 500.0, 239.5, 0.0, 0.0, 1.0],
        projection_matrix: [
            500.0, 0.0, 319.5, 0.0, 0.0, 500.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0,
        ],
    }
}

pub(super) fn generated_pick_place_calibration(
    sequence: u64,
    now_ns: i64,
) -> DepthCameraCalibration {
    let half_sqrt_two = std::f64::consts::FRAC_1_SQRT_2;
    DepthCameraCalibration {
        schema_version: SCHEMA_VERSION,
        sequence,
        source_time_ns: now_ns,
        source_id: GENERATED_DEPTH_SOURCE_ID.into(),
        parent_frame_id: "base_link".into(),
        frame_id: GENERATED_DEPTH_FRAME_ID.into(),
        translation_m: [0.20, 0.06, 0.67],
        orientation_xyzw: [half_sqrt_two, half_sqrt_two, 0.0, 0.0],
        width: 640,
        height: 480,
        distortion_model: "plumb_bob".into(),
        distortion: vec![0.0; 5],
        camera_matrix: [850.0, 0.0, 319.5, 0.0, 850.0, 239.5, 0.0, 0.0, 1.0],
        projection_matrix: [
            850.0, 0.0, 319.5, 0.0, 0.0, 850.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0,
        ],
    }
}

pub(super) fn generated_rgbd(
    sequence: u64,
    now_ns: i64,
) -> Result<(
    r2r::sensor_msgs::msg::Image,
    AlignedDepthFrame,
    Vec<DetectedInstance2D>,
)> {
    const WIDTH: u32 = 640;
    const HEIGHT: u32 = 480;
    let rgb = image::load_from_memory(GENERATED_RGB_ASSET)?
        .resize_exact(WIDTH, HEIGHT, FilterType::Lanczos3)
        .to_rgb8();
    let cube_mask = polygon_mask(
        WIDTH,
        HEIGHT,
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
        WIDTH,
        HEIGHT,
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
    let bin_interior_mask = polygon_mask(
        WIDTH,
        HEIGHT,
        &[
            (303, 224),
            (334, 146),
            (381, 138),
            (548, 177),
            (562, 219),
            (549, 278),
            (519, 301),
            (309, 259),
        ],
    );
    let color = r2r::sensor_msgs::msg::Image {
        header: r2r::std_msgs::msg::Header {
            stamp: ros_time(now_ns),
            frame_id: GENERATED_DEPTH_FRAME_ID.into(),
        },
        height: HEIGHT,
        width: WIDTH,
        encoding: "rgb8".into(),
        is_bigendian: 0,
        step: WIDTH * 3,
        data: rgb.into_raw(),
    };
    let mut values = vec![700_u16; (WIDTH * HEIGHT) as usize];
    fill_masked_depth(&mut values, &cube_mask, 590, 620);
    fill_masked_depth(&mut values, &bin_mask, 620, 660);
    fill_masked_depth(&mut values, &bin_interior_mask, 680, 690);
    let depth = AlignedDepthFrame {
        schema_version: SCHEMA_VERSION,
        sequence,
        source_time_ns: now_ns,
        source_id: GENERATED_DEPTH_SOURCE_ID.into(),
        frame_id: GENERATED_DEPTH_FRAME_ID.into(),
        width: WIDTH,
        height: HEIGHT,
        depth_scale_m: 0.001,
        depth: values,
    };
    let instances = vec![
        generated_instance("red-cube-0", "red cube", cube_mask)?,
        generated_instance("gray-storage-bin-0", "gray storage bin", bin_mask)?,
    ];
    Ok((color, depth, instances))
}

pub(super) fn fill_generated_depth_from_instances(
    depth: &mut AlignedDepthFrame,
    instances: &[DetectedInstance2D],
) -> Result<()> {
    depth.depth.fill(0);
    for instance in instances {
        let mask =
            image::load_from_memory_with_format(&instance.mask_png, ImageFormat::Png)?.into_luma8();
        match instance.label.as_str() {
            "red cube" => fill_masked_depth(&mut depth.depth, &mask, 630, 670),
            "gray storage bin" => fill_masked_depth(&mut depth.depth, &mask, 590, 670),
            _ => {}
        }
    }
    Ok(())
}

pub(super) fn depth_image(depth: &AlignedDepthFrame) -> r2r::sensor_msgs::msg::Image {
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

fn generated_instance(id: &str, label: &str, mask: GrayImage) -> Result<DetectedInstance2D> {
    let bounds = mask_bounds(&mask).ok_or_else(|| eyre!("generated mask is empty"))?;
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
    let [_, minimum_y, _, maximum_y] = mask_bounds(mask).expect("generated mask has bounds");
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
    use perception_core::world_scene_from_aligned_depth;

    use super::*;

    #[test]
    fn migrated_depth_fixture_is_unchanged() {
        let cloud = generated_depth_test_cloud(1, 2);
        assert_eq!(cloud.points_xyz_m.len(), 497);
        assert_abs_diff_eq!(cloud.points_xyz_m[0][0], -0.1);
        assert_abs_diff_eq!(cloud.points_xyz_m[0][1], 0.3);
        assert_abs_diff_eq!(cloud.points_xyz_m[440][0], 0.1);
        assert_abs_diff_eq!(cloud.points_xyz_m[440][1], 0.5);
        assert_abs_diff_eq!(cloud.points_xyz_m[496][0], 0.03);
        assert_abs_diff_eq!(cloud.points_xyz_m[496][1], 0.4);
        assert_abs_diff_eq!(cloud.points_xyz_m[496][2], 0.08);
    }

    #[test]
    fn generated_pick_and_place_points_share_one_support_plane() {
        let (_, mut depth, instances) = generated_rgbd(1, 2).unwrap();
        fill_generated_depth_from_instances(&mut depth, &instances).unwrap();
        let scene = world_scene_from_aligned_depth(
            1,
            &depth,
            &generated_pick_place_calibration(1, 2),
            &instances,
        )
        .unwrap();
        let cube = scene
            .objects
            .iter()
            .find(|object| object.graspable)
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
        assert_abs_diff_eq!(
            cube.pose.position_m[2],
            placement.pose.position_m[2] + cube.size_m[2] / 2.0
        );
        assert_abs_diff_eq!(cube.pose.position_m[2], 0.02);
        assert_abs_diff_eq!(bin.pose.position_m[2], 0.04);
        assert_abs_diff_eq!(bin.size_m[2], 0.08);
    }
}
