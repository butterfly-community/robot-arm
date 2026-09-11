//! Bind the same human-selected target across two independent observations.
//! Detector instance IDs are frame-local, not tracking IDs. Associate within
//! the original semantic class by nearest 3-D centre; never pick another class
//! merely because it has a grasp candidate. No synthetic pose or stale cloud.
use eyre::{Result, eyre};
use robot_arm_messages::{PickPlaceRequest, PlacementRegion, SCHEMA_VERSION, WorldScene};

pub struct ObservationTarget {
    object_label: String,
    object_position: [f64; 3],
    // The user selected a destination in the scene frame. Re-observation
    // corrects the grasp, not the intent; do not require a second pad detection.
    pub placement: PlacementRegion,
    frame_id: String,
    sequence: u64,
}

fn distance2(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter().zip(b).map(|(a, b)| (a - b).powi(2)).sum()
}

impl ObservationTarget {
    pub fn new(scene: &WorldScene, request: &PickPlaceRequest, placement: &PlacementRegion) -> Result<Self> {
        let object = scene
            .objects
            .iter()
            .find(|o| o.object_id == request.object_id)
            .ok_or_else(|| eyre!("原观测中没有选定目标"))?;
        Ok(Self {
            object_label: object.label.clone(),
            object_position: object.pose.position_m,
            placement: placement.clone(),
            frame_id: scene.frame_id.clone(),
            sequence: scene.sequence,
        })
    }

    pub fn rebind(&self, scene: &WorldScene, request_id: String) -> Result<PickPlaceRequest> {
        if scene.sequence <= self.sequence || scene.frame_id != self.frame_id {
            return Err(eyre!("预抓取二次观测不是同坐标系的新场景"));
        }
        let object = scene
            .objects
            .iter()
            .filter(|o| o.label == self.object_label)
            .min_by(|a, b| {
                distance2(a.pose.position_m, self.object_position)
                    .total_cmp(&distance2(b.pose.position_m, self.object_position))
            })
            .ok_or_else(|| eyre!("预抓取二次观测未识别到目标：{}", self.object_label))?;
        Ok(PickPlaceRequest {
            schema_version: SCHEMA_VERSION,
            request_id,
            scene_sequence: scene.sequence,
            object_id: object.object_id.clone(),
            placement_region_id: self.placement.region_id.clone(),
        })
    }
}
