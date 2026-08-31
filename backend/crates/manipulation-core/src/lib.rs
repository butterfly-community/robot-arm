use robot_arm_messages::{
    ManipulationStep, PickPlaceRequest, PlacementRegion, SceneObject, WorldScene,
};
use thiserror::Error;

const PICK_PLACE_STEPS: [ManipulationStep; 9] = [
    ManipulationStep::ApproachObject,
    ManipulationStep::ReachObject,
    ManipulationStep::CloseTool,
    ManipulationStep::AttachObject,
    ManipulationStep::ApproachPlacement,
    ManipulationStep::ReachPlacement,
    ManipulationStep::OpenTool,
    ManipulationStep::DetachObject,
    ManipulationStep::Complete,
];

#[derive(Clone, Debug, PartialEq)]
pub struct PickPlacePlan {
    pub request_id: String,
    pub object: SceneObject,
    pub placement_region: PlacementRegion,
    pub steps: Vec<ManipulationStep>,
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum PlanError {
    #[error("scene does not contain object {0}")]
    ObjectNotFound(String),
    #[error("object {0} is not marked graspable")]
    ObjectNotGraspable(String),
    #[error("scene does not contain placement region {0}")]
    PlacementRegionNotFound(String),
}

pub fn plan_pick_place(
    scene: &WorldScene,
    request: &PickPlaceRequest,
) -> Result<PickPlacePlan, PlanError> {
    let object = scene
        .objects
        .iter()
        .find(|object| object.object_id == request.object_id)
        .ok_or_else(|| PlanError::ObjectNotFound(request.object_id.clone()))?;
    if !object.graspable {
        return Err(PlanError::ObjectNotGraspable(object.object_id.clone()));
    }
    let placement_region = scene
        .placement_regions
        .iter()
        .find(|region| region.region_id == request.placement_region_id)
        .ok_or_else(|| PlanError::PlacementRegionNotFound(request.placement_region_id.clone()))?;
    Ok(PickPlacePlan {
        request_id: request.request_id.clone(),
        object: object.clone(),
        placement_region: placement_region.clone(),
        steps: PICK_PLACE_STEPS.to_vec(),
    })
}

pub fn cartesian_target(plan: &PickPlacePlan, step: ManipulationStep) -> Option<[f64; 3]> {
    let mut position = match step {
        ManipulationStep::ApproachObject | ManipulationStep::ReachObject => {
            plan.object.pose.position_m
        }
        ManipulationStep::ApproachPlacement | ManipulationStep::ReachPlacement => {
            plan.placement_region.pose.position_m
        }
        _ => return None,
    };
    position[2] += match step {
        ManipulationStep::ApproachObject => plan.object.size_m[2],
        ManipulationStep::ReachObject => 0.0,
        ManipulationStep::ApproachPlacement => {
            plan.placement_region.size_m[2] + plan.object.size_m[2]
        }
        ManipulationStep::ReachPlacement => plan.object.size_m[2] / 2.0,
        _ => unreachable!(),
    };
    Some(position)
}

#[cfg(test)]
mod tests {
    use robot_arm_messages::{PlacementRegion, Pose3, SCHEMA_VERSION, SceneObject, WorldScene};

    use super::*;

    fn fixture_scene() -> WorldScene {
        let pose = Pose3 {
            position_m: [0.0; 3],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        };
        WorldScene {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            sample_time_ns: 2,
            frame_id: "base_link".into(),
            objects: vec![SceneObject {
                object_id: "red-cube-0".into(),
                label: "red cube".into(),
                pose: pose.clone(),
                size_m: [0.04; 3],
                confidence: 1.0,
                graspable: true,
            }],
            placement_regions: vec![PlacementRegion {
                region_id: "basket".into(),
                label: "basket interior".into(),
                pose,
                size_m: [0.1; 3],
                source_object_id: None,
            }],
            obstacles: vec![],
        }
    }

    #[test]
    fn plan_is_linear_and_uses_scene_ids() {
        let request = PickPlaceRequest {
            schema_version: SCHEMA_VERSION,
            request_id: "task".into(),
            object_id: "red-cube-0".into(),
            placement_region_id: "basket".into(),
        };
        let plan = plan_pick_place(&fixture_scene(), &request).unwrap();
        assert_eq!(plan.object.object_id, "red-cube-0");
        assert_eq!(plan.placement_region.region_id, "basket");
        assert_eq!(plan.steps, PICK_PLACE_STEPS);
    }

    #[test]
    fn missing_scene_ids_are_reported_without_fallback_selection() {
        let request = PickPlaceRequest {
            schema_version: SCHEMA_VERSION,
            request_id: "task".into(),
            object_id: "missing".into(),
            placement_region_id: "basket".into(),
        };
        assert_eq!(
            plan_pick_place(&fixture_scene(), &request),
            Err(PlanError::ObjectNotFound("missing".into()))
        );
    }

    #[test]
    fn cartesian_targets_come_only_from_scene_geometry() {
        let request = PickPlaceRequest {
            schema_version: SCHEMA_VERSION,
            request_id: "task".into(),
            object_id: "red-cube-0".into(),
            placement_region_id: "basket".into(),
        };
        let mut plan = plan_pick_place(&fixture_scene(), &request).unwrap();
        plan.object.pose.position_m = [0.2, -0.1, 0.08];
        plan.object.size_m = [0.04, 0.04, 0.04];
        plan.placement_region.pose.position_m = [0.25, 0.1, 0.03];
        plan.placement_region.size_m = [0.1, 0.1, 0.06];
        assert_eq!(
            cartesian_target(&plan, ManipulationStep::ApproachObject),
            Some([0.2, -0.1, 0.12])
        );
        assert_eq!(
            cartesian_target(&plan, ManipulationStep::ReachObject),
            Some([0.2, -0.1, 0.08])
        );
        assert_eq!(
            cartesian_target(&plan, ManipulationStep::ApproachPlacement),
            Some([0.25, 0.1, 0.13])
        );
        assert_eq!(
            cartesian_target(&plan, ManipulationStep::ReachPlacement),
            Some([0.25, 0.1, 0.05])
        );
    }
}
