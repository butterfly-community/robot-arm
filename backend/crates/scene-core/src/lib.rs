use image::ImageFormat;
use nalgebra::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};
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
}

#[derive(Clone, Debug, PartialEq)]
pub struct InstancePointCloud {
    pub instance_id: String,
    pub points_xyz_m: Vec<[f32; 3]>,
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
        let mut minimum = Vector3::repeat(f64::INFINITY);
        let mut maximum = Vector3::repeat(f64::NEG_INFINITY);
        let mut count = 0_u64;
        let mut points_xyz_m = Vec::new();
        for (x, y, mask_value) in mask.enumerate_pixels() {
            if mask_value[0] == 0 {
                continue;
            }
            let raw_depth = depth.depth[(y * depth.width + x) as usize];
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
            points_xyz_m.push([point.x as f32, point.y as f32, point.z as f32]);
            minimum = minimum.inf(&point);
            maximum = maximum.sup(&point);
            count += 1;
        }
        if count == 0 {
            return Err(SceneError::MissingInstanceDepth(
                instance.instance_id.clone(),
            ));
        }
        let center = (minimum + maximum) / 2.0;
        let size = maximum - minimum;
        let object = SceneObject {
            object_id: instance.instance_id.clone(),
            label: instance.label.clone(),
            pose: Pose3 {
                position_m: center.into(),
                // The dimensions above are an axis-aligned box in the base frame,
                // so its pose must use that same frame rather than the camera
                // calibration rotation.
                orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            },
            size_m: size.into(),
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
}
