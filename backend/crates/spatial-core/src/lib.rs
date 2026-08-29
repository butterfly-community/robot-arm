use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};
use robot_arm_messages::{
    AbsolutePoseFrame, ControlInputFrame, RelativeToolMotion, SCHEMA_VERSION, SpatialConfigPatch,
    SpatialConfigState,
};

#[derive(Clone, Debug)]
struct PoseSample {
    position_m: Option<[f64; 3]>,
    orientation: Option<UnitQuaternion<f64>>,
    position_source_capable: bool,
    orientation_source_capable: bool,
}

#[derive(Clone, Debug)]
struct SessionAnchor {
    id: u64,
    pose: Option<PoseSample>,
    action_translation_m: [f64; 3],
    action_front_pitch_rad: f64,
    action_horizontal_arc_rad: f64,
    last_action_time_ns: i64,
}

#[derive(Clone, Debug)]
pub struct SpatialTransform {
    config: SpatialConfigState,
    current_pose: Option<PoseSample>,
    current_input: ControlInputFrame,
    previous_confirm_origin: bool,
    session: Option<SessionAnchor>,
    next_session_id: u64,
    output_sequence: u64,
}

impl SpatialTransform {
    pub fn new(config: SpatialConfigState) -> Self {
        Self {
            config,
            current_pose: None,
            current_input: ControlInputFrame::default(),
            previous_confirm_origin: false,
            session: None,
            next_session_id: 1,
            output_sequence: 0,
        }
    }

    pub fn config(&self) -> &SpatialConfigState {
        &self.config
    }

    pub fn apply_config(&mut self, patch: SpatialConfigPatch) {
        if let Some(value) = patch.position_source_id {
            self.config.position_source_id = value;
        }
        if let Some(value) = patch.orientation_source_id {
            self.config.orientation_source_id = value;
        }
        if let Some(value) = patch.base_from_tracking_axes {
            self.config.base_from_tracking_axes = value;
        }
        if let Some(value) = patch.translation_scale {
            self.config.translation_scale = value;
        }
        if let Some(value) = patch.action_translation_m_per_s {
            self.config.action_translation_m_per_s = value;
        }
        if let Some(value) = patch.action_arc_rad_per_s {
            self.config.action_arc_rad_per_s = value;
        }
        if let Some(value) = patch.origin_position_m {
            self.config.origin_position_m = value;
        }
        if let Some(value) = patch.switches {
            self.config.switches = value;
        }
        self.config.config_version += 1;
    }

    pub fn confirm_origin(&mut self) -> bool {
        let Some(pose) = &self.current_pose else {
            return false;
        };
        let Some(position) = pose.position_m else {
            return false;
        };
        self.config.origin_position_m = Some(position);
        self.config.config_version += 1;
        true
    }

    pub fn update_pose(&mut self, frame: AbsolutePoseFrame) {
        let source_changed = self.config.position_source_id != frame.position_source_id
            || self.config.orientation_source_id != frame.orientation_source_id;
        if source_changed {
            self.end_session();
            self.current_pose = None;
        }
        self.config.position_source_id = frame.position_source_id.clone();
        self.config.orientation_source_id = frame.orientation_source_id.clone();
        self.current_pose = Some(PoseSample {
            position_m: frame.flags.position_valid.then_some(frame.position_m),
            orientation: frame
                .flags
                .orientation_valid
                .then(|| unit_quaternion(frame.orientation_xyzw)),
            position_source_capable: frame.position_source_capable,
            orientation_source_capable: frame.orientation_source_capable,
        });
    }

    pub fn handle_control(
        &mut self,
        frame: ControlInputFrame,
        transformed_time_ns: i64,
    ) -> RelativeToolMotion {
        let confirm_now = frame.confirm_origin.is_active && frame.confirm_origin.value;
        if confirm_now && !self.previous_confirm_origin {
            self.confirm_origin();
        }
        self.previous_confirm_origin = confirm_now;

        if !frame.control_active.is_active || !frame.control_active.value {
            self.current_input = frame;
            self.end_session();
            return self.inactive_output(self.current_input.source_time_ns, transformed_time_ns);
        }

        let source_time_ns = frame.source_time_ns;
        self.current_input = frame;
        self.ensure_session(source_time_ns);
        self.integrate_action_only(source_time_ns);
        self.current_output(source_time_ns, transformed_time_ns)
    }

    fn ensure_session(&mut self, source_time_ns: i64) {
        if self.session.is_some() {
            return;
        }
        let id = self.next_session_id;
        self.next_session_id += 1;
        self.session = Some(SessionAnchor {
            id,
            pose: self.current_pose.clone(),
            action_translation_m: [0.0; 3],
            action_front_pitch_rad: 0.0,
            action_horizontal_arc_rad: 0.0,
            last_action_time_ns: source_time_ns,
        });
        self.config.control_session_id = Some(id);
    }

    fn end_session(&mut self) {
        self.session = None;
        self.config.control_session_id = None;
    }

    fn integrate_action_only(&mut self, source_time_ns: i64) {
        let Some(session) = &mut self.session else {
            return;
        };
        let elapsed_ns = source_time_ns - session.last_action_time_ns;
        session.last_action_time_ns = source_time_ns;
        if elapsed_ns <= 0 {
            return;
        }
        let elapsed_s = elapsed_ns as f64 / 1_000_000_000.0;

        let has_absolute_position = self
            .current_pose
            .as_ref()
            .is_some_and(|value| value.position_source_capable);
        let has_absolute_orientation = self
            .current_pose
            .as_ref()
            .is_some_and(|value| value.orientation_source_capable);
        if !has_absolute_position && let Some(rate) = self.config.action_translation_m_per_s {
            session.action_translation_m[0] +=
                active_value(self.current_input.move_forward_back) * rate * elapsed_s;
            session.action_translation_m[1] +=
                active_value(self.current_input.move_left_right) * rate * elapsed_s;
            session.action_translation_m[2] +=
                active_value(self.current_input.move_up_down) * rate * elapsed_s;
        }
        if !has_absolute_orientation && let Some(rate) = self.config.action_arc_rad_per_s {
            session.action_front_pitch_rad +=
                active_value(self.current_input.front_pitch) * rate * elapsed_s;
            session.action_horizontal_arc_rad +=
                active_value(self.current_input.horizontal_arc) * rate * elapsed_s;
        }
    }

    fn current_output(
        &mut self,
        source_time_ns: i64,
        transformed_time_ns: i64,
    ) -> RelativeToolMotion {
        self.output_sequence += 1;
        let session = self.session.as_ref().expect("active output has a session");

        let mut translation_m = match (&session.pose, &self.current_pose) {
            (Some(anchor), Some(current)) => match (anchor.position_m, current.position_m) {
                (Some(anchor_position), Some(current_position)) => {
                    let tracking_delta =
                        Vector3::from(current_position) - Vector3::from(anchor_position);
                    let axes =
                        Matrix3::from_row_slice(self.config.base_from_tracking_axes.as_flattened());
                    (axes * tracking_delta * self.config.translation_scale).into()
                }
                _ => session.action_translation_m,
            },
            _ => session.action_translation_m,
        };
        let (mut front_pitch_rad, mut horizontal_arc_rad) =
            match (&session.pose, &self.current_pose) {
                (Some(anchor), Some(current)) => {
                    match (&anchor.orientation, &current.orientation) {
                        (Some(anchor_orientation), Some(current_orientation)) => {
                            let relative_rotation =
                                anchor_orientation.inverse() * current_orientation;
                            let local_scaled_axis = relative_rotation.scaled_axis();
                            (-local_scaled_axis.x, local_scaled_axis.z)
                        }
                        _ => (
                            session.action_front_pitch_rad,
                            session.action_horizontal_arc_rad,
                        ),
                    }
                }
                _ => (
                    session.action_front_pitch_rad,
                    session.action_horizontal_arc_rad,
                ),
            };

        if !self.config.switches.translation {
            translation_m = [0.0; 3];
        }
        if !self.config.switches.front_pitch {
            front_pitch_rad = 0.0;
        }
        if !self.config.switches.horizontal_arc {
            horizontal_arc_rad = 0.0;
        }

        RelativeToolMotion {
            schema_version: SCHEMA_VERSION,
            sequence: self.output_sequence,
            source_time_ns,
            transformed_time_ns,
            control_session_id: Some(session.id),
            active: true,
            translation_m,
            front_pitch_rad,
            horizontal_arc_rad,
            primary_tool_open: pressed(self.current_input.primary_tool_open),
            primary_tool_value: active_value(self.current_input.primary_tool),
        }
    }

    fn inactive_output(
        &mut self,
        source_time_ns: i64,
        transformed_time_ns: i64,
    ) -> RelativeToolMotion {
        self.output_sequence += 1;
        RelativeToolMotion {
            schema_version: SCHEMA_VERSION,
            sequence: self.output_sequence,
            source_time_ns,
            transformed_time_ns,
            control_session_id: None,
            active: false,
            translation_m: [0.0; 3],
            front_pitch_rad: 0.0,
            horizontal_arc_rad: 0.0,
            primary_tool_open: pressed(self.current_input.primary_tool_open),
            primary_tool_value: active_value(self.current_input.primary_tool),
        }
    }
}

fn unit_quaternion(xyzw: [f64; 4]) -> UnitQuaternion<f64> {
    UnitQuaternion::new_normalize(Quaternion::new(xyzw[3], xyzw[0], xyzw[1], xyzw[2]))
}

fn pressed(sample: robot_arm_messages::BooleanActionSample) -> bool {
    sample.is_active && sample.changed_since_last_sync && sample.value
}

fn active_value(sample: robot_arm_messages::FloatActionSample) -> f64 {
    if sample.is_active { sample.value } else { 0.0 }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::FRAC_PI_2;

    use super::*;
    use nalgebra::Vector3;
    use robot_arm_messages::{
        BooleanActionSample, FloatActionSample, PoseFlags, SpatialComponentSwitches,
    };
    use serde::Deserialize;

    #[derive(Deserialize)]
    struct SyntheticFixture {
        schema_version: u32,
        translation_scale: f64,
        baseline: SyntheticPose,
        cases: Vec<SyntheticCase>,
    }

    #[derive(Deserialize)]
    struct SyntheticPose {
        position_m: [f64; 3],
        orientation_xyzw: [f64; 4],
    }

    #[derive(Deserialize)]
    struct SyntheticCase {
        name: String,
        position_m: [f64; 3],
        orientation_xyzw: [f64; 4],
        expected_translation_m: [f64; 3],
        expected_front_pitch_rad: f64,
        expected_horizontal_arc_rad: f64,
    }

    fn pose(
        sequence: u64,
        position_m: [f64; 3],
        orientation: UnitQuaternion<f64>,
    ) -> AbsolutePoseFrame {
        let quaternion = orientation.quaternion();
        AbsolutePoseFrame {
            schema_version: SCHEMA_VERSION,
            sequence,
            source_time_ns: sequence as i64 * 1_000_000_000,
            received_time_ns: sequence as i64 * 1_000_000_000,
            position_source_id: Some("source".into()),
            orientation_source_id: Some("source".into()),
            position_source_capable: true,
            orientation_source_capable: true,
            reference_space: "local".into(),
            position_m,
            orientation_xyzw: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
            flags: PoseFlags {
                position_valid: true,
                position_tracked: true,
                orientation_valid: true,
                orientation_tracked: true,
            },
        }
    }

    fn control(sequence: u64, active: bool) -> ControlInputFrame {
        ControlInputFrame {
            schema_version: SCHEMA_VERSION,
            sequence,
            source_time_ns: sequence as i64 * 1_000_000_000,
            received_time_ns: sequence as i64 * 1_000_000_000,
            control_active: BooleanActionSample {
                is_active: true,
                changed_since_last_sync: true,
                value: active,
            },
            ..ControlInputFrame::default()
        }
    }

    fn fixture_pose(sequence: u64, value: &SyntheticPose) -> AbsolutePoseFrame {
        AbsolutePoseFrame {
            schema_version: SCHEMA_VERSION,
            sequence,
            source_time_ns: sequence as i64 * 10_000_000,
            received_time_ns: sequence as i64 * 10_000_000,
            position_source_id: Some("synthetic-controller".into()),
            orientation_source_id: Some("synthetic-controller".into()),
            position_source_capable: true,
            orientation_source_capable: true,
            reference_space: "local".into(),
            position_m: value.position_m,
            orientation_xyzw: value.orientation_xyzw,
            flags: PoseFlags {
                position_valid: true,
                position_tracked: true,
                orientation_valid: true,
                orientation_tracked: true,
            },
        }
    }

    fn output_after_pose(
        transform: &mut SpatialTransform,
        frame: AbsolutePoseFrame,
        transformed_time_ns: i64,
    ) -> RelativeToolMotion {
        let mut input = control(frame.sequence, true);
        input.source_time_ns = frame.source_time_ns;
        input.received_time_ns = frame.received_time_ns;
        transform.update_pose(frame);
        transform.handle_control(input, transformed_time_ns)
    }

    fn assert_close(actual: f64, expected: f64, case: &str) {
        assert!(
            (actual - expected).abs() < 1e-12,
            "{case}: {actual} != {expected}"
        );
    }

    #[test]
    fn generated_controller_cycle_covers_all_spatial_semantics() {
        let fixture: SyntheticFixture = serde_json::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../../tests/fixtures/controller-input-synthetic-cycle.json"
        )))
        .unwrap();
        assert_eq!(fixture.schema_version, SCHEMA_VERSION);

        for (index, case) in fixture.cases.iter().enumerate() {
            let mut transform = SpatialTransform::new(SpatialConfigState {
                translation_scale: fixture.translation_scale,
                ..Default::default()
            });
            transform.update_pose(fixture_pose(1, &fixture.baseline));
            let active = control(2, true);
            transform.handle_control(active, 20_000_000);
            let output = output_after_pose(
                &mut transform,
                fixture_pose(
                    index as u64 + 3,
                    &SyntheticPose {
                        position_m: case.position_m,
                        orientation_xyzw: case.orientation_xyzw,
                    },
                ),
                (index as i64 + 3) * 10_000_000,
            );
            for (actual, expected) in output
                .translation_m
                .into_iter()
                .zip(case.expected_translation_m)
            {
                assert_close(actual, expected, &case.name);
            }
            assert_close(
                output.front_pitch_rad,
                case.expected_front_pitch_rad,
                &case.name,
            );
            assert_close(
                output.horizontal_arc_rad,
                case.expected_horizontal_arc_rad,
                &case.name,
            );
        }
    }

    #[test]
    fn tool_open_press_is_forwarded_once_even_without_spatial_control() {
        let mut transform = SpatialTransform::new(SpatialConfigState::default());
        let mut press = control(1, false);
        press.primary_tool_open = BooleanActionSample {
            is_active: true,
            changed_since_last_sync: true,
            value: true,
        };
        assert!(transform.handle_control(press, 1).primary_tool_open);

        let mut held = control(2, false);
        held.primary_tool_open = BooleanActionSample {
            is_active: true,
            changed_since_last_sync: false,
            value: true,
        };
        assert!(!transform.handle_control(held, 2).primary_tool_open);
    }

    #[test]
    fn first_active_frame_is_a_zero_relative_anchor() {
        let mut transform = SpatialTransform::new(SpatialConfigState::default());
        transform.update_pose(pose(1, [1.0, 2.0, 3.0], UnitQuaternion::identity()));
        let output = transform.handle_control(control(2, true), 2);
        assert!(output.active);
        assert_eq!(output.translation_m, [0.0; 3]);
        assert_eq!(output.front_pitch_rad, 0.0);
        assert_eq!(output.horizontal_arc_rad, 0.0);
    }

    #[test]
    fn translation_mapping_and_half_scale_are_applied_once() {
        let mut transform = SpatialTransform::new(SpatialConfigState::default());
        transform.update_pose(pose(1, [0.0; 3], UnitQuaternion::identity()));
        transform.handle_control(control(2, true), 2);
        let output = output_after_pose(
            &mut transform,
            pose(3, [0.2, 0.4, -0.6], UnitQuaternion::identity()),
            3,
        );
        assert_eq!(output.translation_m, [0.3, -0.1, 0.2]);
    }

    #[test]
    fn origin_confirmation_does_not_create_motion_or_change_session_delta() {
        let mut transform = SpatialTransform::new(SpatialConfigState::default());
        transform.update_pose(pose(1, [1.0, 0.0, 0.0], UnitQuaternion::identity()));
        assert!(transform.confirm_origin());
        let anchor = transform.handle_control(control(2, true), 2);
        assert_eq!(anchor.translation_m, [0.0; 3]);
        let output = output_after_pose(
            &mut transform,
            pose(3, [1.0, 0.0, -0.2], UnitQuaternion::identity()),
            3,
        );
        assert_eq!(output.translation_m, [0.1, 0.0, 0.0]);
    }

    #[test]
    fn origin_can_be_cleared_through_the_same_config_patch() {
        let mut transform = SpatialTransform::new(SpatialConfigState {
            origin_position_m: Some([1.0, 2.0, 3.0]),
            ..Default::default()
        });
        transform.apply_config(SpatialConfigPatch {
            origin_position_m: Some(None),
            ..Default::default()
        });
        assert_eq!(transform.config().origin_position_m, None);
    }

    #[test]
    fn local_pitch_and_horizontal_arc_keep_their_independent_semantics() {
        let mut transform = SpatialTransform::new(SpatialConfigState::default());
        transform.update_pose(pose(1, [0.0; 3], UnitQuaternion::identity()));
        transform.handle_control(control(2, true), 2);

        let pitch = output_after_pose(
            &mut transform,
            pose(
                3,
                [0.0; 3],
                UnitQuaternion::from_scaled_axis(Vector3::new(-FRAC_PI_2, 0.0, 0.0)),
            ),
            3,
        );
        assert!((pitch.front_pitch_rad - FRAC_PI_2).abs() < 1e-12);
        assert!(pitch.horizontal_arc_rad.abs() < 1e-12);

        transform.handle_control(control(4, false), 4);
        transform.update_pose(pose(5, [0.0; 3], UnitQuaternion::identity()));
        transform.handle_control(control(6, true), 6);
        let arc = output_after_pose(
            &mut transform,
            pose(
                7,
                [0.0; 3],
                UnitQuaternion::from_scaled_axis(Vector3::new(0.0, 0.0, FRAC_PI_2)),
            ),
            7,
        );
        assert!(arc.front_pitch_rad.abs() < 1e-12);
        assert!((arc.horizontal_arc_rad - FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn component_switches_only_zero_their_own_outputs() {
        let config = SpatialConfigState {
            switches: SpatialComponentSwitches {
                translation: false,
                front_pitch: true,
                horizontal_arc: false,
            },
            ..Default::default()
        };
        let mut transform = SpatialTransform::new(config);
        transform.update_pose(pose(1, [0.0; 3], UnitQuaternion::identity()));
        transform.handle_control(control(2, true), 2);
        let output = output_after_pose(
            &mut transform,
            pose(
                3,
                [1.0, 2.0, 3.0],
                UnitQuaternion::from_scaled_axis(Vector3::new(-0.4, 0.0, 0.6)),
            ),
            3,
        );
        assert_eq!(output.translation_m, [0.0; 3]);
        assert!(output.front_pitch_rad != 0.0);
        assert_eq!(output.horizontal_arc_rad, 0.0);
    }

    #[test]
    fn action_only_source_defaults_to_one_centimeter_per_second() {
        let mut transform = SpatialTransform::new(SpatialConfigState::default());
        let mut first = control(1, true);
        first.move_up_down = FloatActionSample {
            is_active: true,
            changed_since_last_sync: true,
            value: 1.0,
        };
        first.front_pitch = FloatActionSample {
            is_active: true,
            changed_since_last_sync: true,
            value: 1.0,
        };
        transform.handle_control(first.clone(), 1);
        let mut second = control(2, true);
        second.move_up_down = first.move_up_down;
        second.front_pitch = first.front_pitch;
        let output = transform.handle_control(second, 2);
        assert_eq!(output.translation_m, [0.0, 0.0, 0.01]);
        assert!((output.front_pitch_rad - 0.10).abs() < 1e-12);
    }

    #[test]
    fn action_only_source_integrates_user_configured_rates() {
        let config = SpatialConfigState {
            action_translation_m_per_s: Some(0.2),
            action_arc_rad_per_s: Some(0.5),
            ..Default::default()
        };
        let mut transform = SpatialTransform::new(config);
        let mut first = control(1, true);
        first.move_forward_back = FloatActionSample {
            is_active: true,
            changed_since_last_sync: true,
            value: 1.0,
        };
        first.front_pitch = FloatActionSample {
            is_active: true,
            changed_since_last_sync: true,
            value: -1.0,
        };
        transform.handle_control(first.clone(), 1);
        let mut second = control(2, true);
        second.move_forward_back = first.move_forward_back;
        second.front_pitch = first.front_pitch;
        let output = transform.handle_control(second, 2);
        assert_eq!(output.translation_m, [0.2, 0.0, 0.0]);
        assert_eq!(output.front_pitch_rad, -0.5);
    }

    #[test]
    fn declared_absolute_sources_disable_action_integration() {
        let config = SpatialConfigState {
            action_translation_m_per_s: Some(0.2),
            action_arc_rad_per_s: Some(0.5),
            ..Default::default()
        };
        let mut transform = SpatialTransform::new(config);
        transform.update_pose(pose(1, [0.0; 3], UnitQuaternion::identity()));

        let mut first = control(2, true);
        first.move_forward_back = FloatActionSample {
            is_active: true,
            changed_since_last_sync: true,
            value: 1.0,
        };
        first.front_pitch = FloatActionSample {
            is_active: true,
            changed_since_last_sync: true,
            value: 1.0,
        };
        transform.handle_control(first.clone(), 2);

        let mut second = control(3, true);
        second.move_forward_back = first.move_forward_back;
        second.front_pitch = first.front_pitch;
        let output = transform.handle_control(second, 3);

        assert_eq!(output.translation_m, [0.0; 3]);
        assert_eq!(output.front_pitch_rad, 0.0);
        assert_eq!(output.horizontal_arc_rad, 0.0);
    }

    #[test]
    fn release_and_reacquire_create_a_new_pose_baseline() {
        let mut transform = SpatialTransform::new(SpatialConfigState::default());
        transform.update_pose(pose(1, [0.0; 3], UnitQuaternion::identity()));
        let first = transform.handle_control(control(2, true), 2);
        transform.update_pose(pose(3, [0.0, 0.0, -0.2], UnitQuaternion::identity()));
        transform.handle_control(control(4, false), 4);
        transform.update_pose(pose(5, [0.0, 0.0, -1.0], UnitQuaternion::identity()));
        let second = transform.handle_control(control(6, true), 6);
        assert_ne!(first.control_session_id, second.control_session_id);
        assert_eq!(second.translation_m, [0.0; 3]);
    }

    #[test]
    fn changing_input_source_cannot_reuse_the_previous_source_baseline() {
        let mut transform = SpatialTransform::new(SpatialConfigState::default());
        transform.update_pose(pose(1, [0.0; 3], UnitQuaternion::identity()));
        transform.handle_control(control(2, true), 2);
        transform.update_pose(pose(3, [0.0, 0.0, -0.2], UnitQuaternion::identity()));

        let mut next_pose = pose(4, [5.0, 6.0, 7.0], UnitQuaternion::identity());
        next_pose.position_source_id = Some("another-source".into());
        next_pose.orientation_source_id = Some("another-source".into());
        transform.update_pose(next_pose);
        let output = transform.handle_control(control(5, true), 5);
        assert_eq!(output.translation_m, [0.0; 3]);
    }
}
