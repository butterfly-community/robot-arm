use std::collections::VecDeque;

use image::ImageFormat;
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};
use opencv::{
    core::{Mat, Point2f, Vector},
    geometry, imgproc,
};
use robot_arm_messages::{
    AlignedDepthFrame, DepthCameraCalibration, DetectedInstance2D, PlacementRegion, Pose3,
    SCHEMA_VERSION, SceneObject, WorldScene,
};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum SceneError {
    #[error("depth dimensions do not match payload")]
    InvalidDepthDimensions,
    #[error("camera matrix does not contain usable focal lengths")]
    InvalidIntrinsics,
    #[error("invalid camera transform quaternion")]
    InvalidTransform,
    #[error("cannot decode instance mask: {0}")]
    InvalidMask(#[from] image::ImageError),
    #[error("mask dimensions do not match aligned depth")]
    MaskDimensionsMismatch,
    #[error("instance {0} has no valid aligned depth")]
    MissingInstanceDepth(String),
    #[error("cannot derive instance geometry from depth: {0}")]
    InstanceGeometry(#[from] opencv::Error),
}

#[derive(Clone, Debug, PartialEq)]
pub struct InstancePointCloud {
    pub instance_id: String,
    pub points_xyz_m: Vec<[f32; 3]>,
    /// Same RGB-D frame in base coordinates, excluding this target.
    pub scene_points_xyz_m: Vec<[f32; 3]>,
}

pub fn world_scene_and_instance_clouds_from_aligned_depth(
    sequence: u64,
    depth: &AlignedDepthFrame,
    calibration: &DepthCameraCalibration,
    instances: &[DetectedInstance2D],
    placement_labels: &[String],
) -> Result<(WorldScene, Vec<InstancePointCloud>), SceneError> {
    if depth.depth.len() != (depth.width as usize) * (depth.height as usize) {
        return Err(SceneError::InvalidDepthDimensions);
    }
    let [fx, _, cx, _, fy, cy, _, _, _] = calibration.camera_matrix;
    if fx == 0.0 || fy == 0.0 {
        return Err(SceneError::InvalidIntrinsics);
    }
    let transform = isometry(calibration.translation_m, calibration.orientation_xyzw)?;
    let mut objects = Vec::new();
    let mut placement_regions = Vec::new();
    let mut instance_clouds = Vec::new();
    for instance in instances {
        let mask =
            image::load_from_memory_with_format(&instance.mask_png, ImageFormat::Png)?.into_luma8();
        if mask.width() != depth.width || mask.height() != depth.height {
            return Err(SceneError::MaskDimensionsMismatch);
        }
        let selected = depth_continuous_component(&mask, depth)?;
        let mut points_xyz_m = Vec::new();
        let mut scene_points_xyz_m = Vec::new();
        for (x, y, mask_value) in mask.enumerate_pixels() {
            let index = (y * depth.width + x) as usize;
            let raw_depth = depth.depth[index];
            if raw_depth == 0 {
                continue;
            }
            let z = f64::from(raw_depth) * depth.depth_scale_m;
            let camera_point = Vector3::new(
                (f64::from(x) - cx) * z / fx,
                (f64::from(y) - cy) * z / fy,
                z,
            );
            let point = (transform * Point3::from(camera_point)).coords;
            let point = [point.x as f32, point.y as f32, point.z as f32];
            if mask_value[0] != 0 && selected[index] {
                points_xyz_m.push(point);
            } else {
                scene_points_xyz_m.push(point);
            }
        }
        if points_xyz_m.is_empty() {
            return Err(SceneError::MissingInstanceDepth(
                instance.instance_id.clone(),
            ));
        }
        let bounds = gravity_aligned_bounding_box(&points_xyz_m)?;
        let object = SceneObject {
            object_id: instance.instance_id.clone(),
            label: instance.label.clone(),
            pose: Pose3 {
                position_m: bounds.center_m.into(),
                orientation_xyzw: bounds.orientation_xyzw,
            },
            size_m: bounds.size_m.into(),
            confidence: instance.confidence,
            grasp_candidates: vec![],
        };
        if placement_labels.contains(&instance.label) {
            placement_regions.push(PlacementRegion {
                region_id: format!("{}-interior", instance.instance_id),
                label: format!("{} interior", instance.label),
                pose: object.pose.clone(),
                size_m: object.size_m,
                source_object_id: Some(instance.instance_id.clone()),
            });
        }
        instance_clouds.push(InstancePointCloud {
            instance_id: instance.instance_id.clone(),
            points_xyz_m,
            scene_points_xyz_m,
        });
        objects.push(object);
    }
    Ok((
        WorldScene {
            schema_version: SCHEMA_VERSION,
            sequence,
            sample_time_ns: depth.source_time_ns,
            frame_id: calibration.parent_frame_id.clone(),
            objects,
            placement_regions,
            obstacles: vec![],
        },
        instance_clouds,
    ))
}

/// Approximate a segmented instance with the smallest rectangle in the base
struct GravityAlignedBoundingBox {
    center_m: Vector3<f64>,
    size_m: Vector3<f64>,
    orientation_xyzw: [f64; 4],
}

/// frame's gravity plane and the observed vertical extent. OpenCV's rotating
/// calipers preserve the footprint orientation that an axis-aligned box loses;
/// this keeps the generic scene contract useful without assuming an object
/// class or maintaining a second collision-geometry path.
fn gravity_aligned_bounding_box(
    points_xyz_m: &[[f32; 3]],
) -> Result<GravityAlignedBoundingBox, SceneError> {
    let footprint = Vector::<Point2f>::from_iter(
        points_xyz_m
            .iter()
            .map(|point| Point2f::new(point[0], point[1])),
    );
    let rectangle = geometry::min_area_rect(&footprint)?;
    let (minimum_z, maximum_z) = points_xyz_m.iter().fold(
        (f32::INFINITY, f32::NEG_INFINITY),
        |(minimum, maximum), point| (minimum.min(point[2]), maximum.max(point[2])),
    );
    let (width, height, angle_degrees) = if rectangle.size.width >= rectangle.size.height {
        (rectangle.size.width, rectangle.size.height, rectangle.angle)
    } else {
        (
            rectangle.size.height,
            rectangle.size.width,
            rectangle.angle + 90.0,
        )
    };
    let rotation =
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), f64::from(angle_degrees).to_radians());
    let quaternion = rotation.quaternion();
    Ok(GravityAlignedBoundingBox {
        center_m: Vector3::new(
            f64::from(rectangle.center.x),
            f64::from(rectangle.center.y),
            f64::from((minimum_z + maximum_z) / 2.0),
        ),
        size_m: Vector3::new(
            f64::from(width),
            f64::from(height),
            f64::from(maximum_z - minimum_z),
        ),
        orientation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
    })
}

/// Keep the largest 4-connected surface whose adjacent Z16 samples belong to
/// the same Otsu depth-difference class. Instance masks commonly cover a few
/// pixels across an object silhouette; without this standard edge-aware split,
/// those background pixels dominate an extrema-based 3D box and grasp cloud.
fn depth_continuous_component(
    mask: &image::GrayImage,
    depth: &AlignedDepthFrame,
) -> Result<Vec<bool>, SceneError> {
    let width = depth.width as usize;
    let height = depth.height as usize;
    let valid = mask
        .as_raw()
        .iter()
        .zip(&depth.depth)
        .map(|(mask, depth)| *mask != 0 && *depth != 0)
        .collect::<Vec<_>>();
    let mut differences = Vec::new();
    for y in 0..height {
        for x in 0..width {
            let index = y * width + x;
            if !valid[index] {
                continue;
            }
            if x + 1 < width && valid[index + 1] {
                differences.push(depth.depth[index].abs_diff(depth.depth[index + 1]));
            }
            if y + 1 < height && valid[index + width] {
                differences.push(depth.depth[index].abs_diff(depth.depth[index + width]));
            }
        }
    }
    if differences.is_empty() {
        return Ok(valid);
    }
    let samples = Mat::from_slice(&differences)?;
    let mut classified = Mat::default();
    let max_value = f64::from(u16::MAX);
    let threshold = imgproc::threshold(
        &samples,
        &mut classified,
        0.0,
        max_value,
        imgproc::THRESH_BINARY | imgproc::THRESH_OTSU,
    )?
    .round() as u16;

    let mut visited = vec![false; valid.len()];
    let mut largest = Vec::new();
    for start in 0..valid.len() {
        if !valid[start] || visited[start] {
            continue;
        }
        let mut queue = VecDeque::from([start]);
        let mut component = Vec::new();
        visited[start] = true;
        while let Some(index) = queue.pop_front() {
            component.push(index);
            let x = index % width;
            let y = index / width;
            let mut visit = |next: usize| {
                if valid[next]
                    && !visited[next]
                    && depth.depth[index].abs_diff(depth.depth[next]) <= threshold
                {
                    visited[next] = true;
                    queue.push_back(next);
                }
            };
            if x > 0 {
                visit(index - 1);
            }
            if x + 1 < width {
                visit(index + 1);
            }
            if y > 0 {
                visit(index - width);
            }
            if y + 1 < height {
                visit(index + width);
            }
        }
        if component.len() > largest.len() {
            largest = component;
        }
    }
    let mut selected = vec![false; valid.len()];
    for index in largest {
        selected[index] = true;
    }
    Ok(selected)
}

fn isometry(
    translation: [f64; 3],
    orientation_xyzw: [f64; 4],
) -> Result<Isometry3<f64>, SceneError> {
    let [x, y, z, w] = orientation_xyzw;
    let quaternion = Quaternion::new(w, x, y, z);
    if quaternion.norm_squared() == 0.0 {
        return Err(SceneError::InvalidTransform);
    }
    Ok(Isometry3::from_parts(
        Translation3::from(translation),
        UnitQuaternion::new_normalize(quaternion),
    ))
}

#[cfg(test)]
mod tests {
    use std::io::Cursor;

    use approx::assert_abs_diff_eq;
    use image::{DynamicImage, GrayImage, ImageFormat, Luma};

    use super::*;

    const TEST_SOURCE_ID: &str = "test-depth-source";
    const TEST_FRAME_ID: &str = "test-depth-frame";

    fn calibration(width: u32, height: u32) -> DepthCameraCalibration {
        DepthCameraCalibration {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            source_time_ns: 2,
            source_id: TEST_SOURCE_ID.into(),
            parent_frame_id: "base_link".into(),
            frame_id: TEST_FRAME_ID.into(),
            translation_m: [0.0; 3],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            width,
            height,
            distortion_model: "plumb_bob".into(),
            distortion: vec![0.0; 5],
            camera_matrix: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            projection_matrix: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        }
    }

    fn mask_png() -> Vec<u8> {
        let mut mask = GrayImage::new(4, 3);
        mask.put_pixel(1, 1, Luma([255]));
        mask.put_pixel(2, 1, Luma([255]));
        let mut output = Cursor::new(Vec::new());
        DynamicImage::ImageLuma8(mask)
            .write_to(&mut output, ImageFormat::Png)
            .unwrap();
        output.into_inner()
    }

    fn encoded_mask(mask: GrayImage) -> Vec<u8> {
        let mut output = Cursor::new(Vec::new());
        DynamicImage::ImageLuma8(mask)
            .write_to(&mut output, ImageFormat::Png)
            .unwrap();
        output.into_inner()
    }

    #[test]
    fn gravity_aligned_box_preserves_rotated_footprint() {
        let yaw = -25_f32.to_radians();
        let (sin, cos) = yaw.sin_cos();
        let points = [-0.045_f32, 0.045]
            .into_iter()
            .flat_map(|x| {
                [-0.03_f32, 0.03].into_iter().flat_map(move |y| {
                    [0.0_f32, 0.08]
                        .into_iter()
                        .map(move |z| [0.1 + cos * x - sin * y, -0.15 + sin * x + cos * y, z])
                })
            })
            .collect::<Vec<_>>();
        let bounds = gravity_aligned_bounding_box(&points).unwrap();

        assert_abs_diff_eq!(bounds.center_m.x, 0.1, epsilon = 1e-6);
        assert_abs_diff_eq!(bounds.center_m.y, -0.15, epsilon = 1e-6);
        assert_abs_diff_eq!(bounds.center_m.z, 0.04, epsilon = 1e-6);
        let mut footprint_size = [bounds.size_m.x, bounds.size_m.y];
        footprint_size.sort_by(f64::total_cmp);
        assert_abs_diff_eq!(footprint_size[0], 0.06, epsilon = 1e-6);
        assert_abs_diff_eq!(footprint_size[1], 0.09, epsilon = 1e-6);
        assert_abs_diff_eq!(bounds.size_m.z, 0.08, epsilon = 1e-6);
        assert!(bounds.orientation_xyzw[2].abs() > 0.1);
    }

    #[test]
    fn aligned_depth_and_mask_become_base_frame_scene() {
        let mut calibration = calibration(4, 3);
        calibration.translation_m = [1.0, 0.0, 0.0];
        let depth = AlignedDepthFrame {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            source_time_ns: 2,
            source_id: TEST_SOURCE_ID.into(),
            frame_id: TEST_FRAME_ID.into(),
            width: 4,
            height: 3,
            depth_scale_m: 0.001,
            depth: vec![1000; 12],
        };
        let (scene, clouds) = world_scene_and_instance_clouds_from_aligned_depth(
            3,
            &depth,
            &calibration,
            &[DetectedInstance2D {
                instance_id: "red-cube-0".into(),
                label: "red cube".into(),
                confidence: 1.0,
                bounding_box_xyxy: [1.0, 1.0, 2.0, 1.0],
                mask_width: 4,
                mask_height: 3,
                mask_png: mask_png(),
            }],
            &[],
        )
        .unwrap();
        assert_eq!(scene.frame_id, "base_link");
        assert_eq!(scene.objects.len(), 1);
        assert_abs_diff_eq!(scene.objects[0].pose.position_m[0], 2.5);
        assert_abs_diff_eq!(scene.objects[0].pose.position_m[1], 1.0);
        assert_abs_diff_eq!(scene.objects[0].pose.position_m[2], 1.0);
        assert_abs_diff_eq!(scene.objects[0].size_m[0], 1.0);
        assert!(scene.objects[0].grasp_candidates.is_empty());
        assert_eq!(clouds[0].points_xyz_m.len(), 2);
        assert_eq!(clouds[0].points_xyz_m[0], [2.0, 1.0, 1.0]);
        assert_eq!(clouds[0].scene_points_xyz_m.len(), 10);
        assert_eq!(clouds[0].scene_points_xyz_m[0], [1.0, 0.0, 1.0]);
        assert!(
            clouds[0]
                .points_xyz_m
                .iter()
                .all(|point| !clouds[0].scene_points_xyz_m.contains(point))
        );
    }

    #[test]
    fn scene_environment_retains_ground_and_excludes_only_target_points() {
        let mut calibration = calibration(4, 3);
        calibration.translation_m = [0.0, 0.0, 0.8];
        calibration.orientation_xyzw = [1.0, 0.0, 0.0, 0.0];
        let mut samples = vec![800; 12];
        samples[5] = 700;
        samples[6] = 700;
        samples[0] = 0; // Missing depth is unknown, not synthetic ground.
        let depth = AlignedDepthFrame {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            source_time_ns: 2,
            source_id: TEST_SOURCE_ID.into(),
            frame_id: TEST_FRAME_ID.into(),
            width: 4,
            height: 3,
            depth_scale_m: 0.001,
            depth: samples,
        };
        let (_, clouds) = world_scene_and_instance_clouds_from_aligned_depth(
            3,
            &depth,
            &calibration,
            &[DetectedInstance2D {
                instance_id: "target".into(),
                label: "object".into(),
                confidence: 1.0,
                bounding_box_xyxy: [1.0, 1.0, 2.0, 1.0],
                mask_width: 4,
                mask_height: 3,
                mask_png: mask_png(),
            }],
            &[],
        )
        .unwrap();
        assert_eq!(clouds[0].points_xyz_m.len(), 2);
        assert_eq!(clouds[0].scene_points_xyz_m.len(), 9);
        for point in &clouds[0].points_xyz_m {
            assert_abs_diff_eq!(point[2], 0.1, epsilon = 1e-6);
        }
        for point in &clouds[0].scene_points_xyz_m {
            assert_abs_diff_eq!(point[2], 0.0, epsilon = 1e-6);
        }
    }

    #[test]
    fn generated_pick_place_scene_is_in_the_small_arm_workspace() {
        let half_sqrt_two = std::f64::consts::FRAC_1_SQRT_2;
        let mut calibration = calibration(640, 480);
        calibration.translation_m = [0.23, 0.06, 0.81];
        calibration.orientation_xyzw = [half_sqrt_two, half_sqrt_two, 0.0, 0.0];
        calibration.camera_matrix = [850.0, 0.0, 319.5, 0.0, 850.0, 239.5, 0.0, 0.0, 1.0];
        let depth = AlignedDepthFrame {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            source_time_ns: 2,
            source_id: TEST_SOURCE_ID.into(),
            frame_id: TEST_FRAME_ID.into(),
            width: 640,
            height: 480,
            depth_scale_m: 0.001,
            depth: vec![600; 640 * 480],
        };
        let mask = image::GrayImage::from_pixel(640, 480, image::Luma([255]));
        let mut encoded = std::io::Cursor::new(Vec::new());
        image::DynamicImage::ImageLuma8(mask)
            .write_to(&mut encoded, image::ImageFormat::Png)
            .unwrap();
        let instance = DetectedInstance2D {
            instance_id: "red-cube-0".into(),
            label: "red cube".into(),
            confidence: 1.0,
            bounding_box_xyxy: [0.0, 0.0, 639.0, 479.0],
            mask_width: 640,
            mask_height: 480,
            mask_png: encoded.into_inner(),
        };
        let (scene, _) = world_scene_and_instance_clouds_from_aligned_depth(
            1,
            &depth,
            &calibration,
            &[instance],
            &[],
        )
        .unwrap();
        let center = scene.objects[0].pose.position_m;
        assert!((center[0] - 0.23).abs() < 0.001);
        assert!((center[1] - 0.06).abs() < 0.001);
        assert!((center[2] - 0.21).abs() < 0.001);
    }

    #[test]
    fn silhouette_depth_spill_does_not_expand_grasp_geometry() {
        let mut depth_values = vec![0; 15];
        for index in [6, 7, 11, 12] {
            depth_values[index] = 1000;
        }
        depth_values[8] = 2000;
        let depth = AlignedDepthFrame {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            source_time_ns: 2,
            source_id: TEST_SOURCE_ID.into(),
            frame_id: TEST_FRAME_ID.into(),
            width: 5,
            height: 3,
            depth_scale_m: 0.001,
            depth: depth_values,
        };
        let mut mask = GrayImage::new(5, 3);
        for (x, y) in [(1, 1), (2, 1), (3, 1), (1, 2), (2, 2)] {
            mask.put_pixel(x, y, Luma([255]));
        }
        let instance = DetectedInstance2D {
            instance_id: "object-0".into(),
            label: "object".into(),
            confidence: 1.0,
            bounding_box_xyxy: [1.0, 1.0, 3.0, 2.0],
            mask_width: 5,
            mask_height: 3,
            mask_png: encoded_mask(mask),
        };
        let (scene, clouds) = world_scene_and_instance_clouds_from_aligned_depth(
            1,
            &depth,
            &calibration(5, 3),
            &[instance],
            &[],
        )
        .unwrap();
        assert_eq!(clouds[0].points_xyz_m.len(), 4);
        assert_abs_diff_eq!(scene.objects[0].size_m[0], 1.0);
        assert_abs_diff_eq!(scene.objects[0].size_m[1], 1.0);
        assert_abs_diff_eq!(scene.objects[0].size_m[2], 0.0);
    }
}
