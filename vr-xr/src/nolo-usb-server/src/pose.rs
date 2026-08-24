//! Position variants derived from the optical marker and fused orientation.

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

/// Rotation-center offset recovered from the preserved NOLO CV1 SDK.
///
/// The exact sign/convention still needs physical validation, so callers must
/// keep the optical marker position available and opt in before using this as
/// the primary position.
pub const CONTROLLER_ROTATION_CENTER_OFFSET_METERS: [f32; 3] = [0.0, -0.0045, 0.0755];

pub fn estimated_grip_position(marker_position: [f32; 3], orientation_xyzw: [f32; 4]) -> [f32; 3] {
    let [x, y, z, w] = orientation_xyzw;
    let orientation = UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z));
    let offset =
        orientation.transform_vector(&Vector3::from(CONTROLLER_ROTATION_CENTER_OFFSET_METERS));
    let marker = Vector3::from(marker_position);
    (marker - offset).into()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_vector_near(actual: [f32; 3], expected: [f32; 3]) {
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-6, "{actual} != {expected}");
        }
    }

    #[test]
    fn identity_orientation_subtracts_body_offset() {
        assert_vector_near(
            estimated_grip_position([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]),
            [1.0, 2.0045, 2.9245],
        );
    }

    #[test]
    fn offset_rotates_with_controller_orientation() {
        let half = std::f32::consts::FRAC_PI_4;
        assert_vector_near(
            estimated_grip_position([0.0; 3], [0.0, 0.0, half.sin(), half.cos()]),
            [-0.0045, 0.0, -0.0755],
        );
    }
}
