import {
  schemaVersion,
  type PerceptionState,
  type WorldScene,
} from "@robot/contracts";

export type PickPlaceStep = {
  phase: "generate_grasps" | "mode" | "submit";
  requestId: string;
};

// One UI action, same existing stages/endpoints. Never submit the old scene
// sequence after candidate generation; do not depend on WebSocket render timing.
export async function startPickPlace(
  scene: WorldScene,
  objectId: string,
  regionId: string,
  post: (path: string, body: Record<string, unknown>) => Promise<unknown>,
  makeId: () => string,
  onProgress?: (step: PickPlaceStep) => void,
) {
  const object = scene.objects.find((o) => o.object_id === objectId);
  if (!object || !scene.placement_regions.some((r) => r.region_id === regionId))
    throw Error("请选择当前场景中的抓取物体和放置目标");
  let sequence = scene.sequence;
  if (!object.grasp_candidates.length) {
    const requestId = makeId();
    onProgress?.({ phase: "generate_grasps", requestId });
    const result = (await post("/api/perception/request", {
      schema_version: schemaVersion,
      request_id: requestId,
      action: "generate_grasps",
      input_sequence: sequence,
      object_id: objectId,
    })) as { value: PerceptionState };
    if (
      result.value.last_scene_sequence == null ||
      !result.value.instances.some(
        (i) => i.instance_id === objectId && i.grasp_candidate_count > 0,
      )
    )
      throw Error("选定物体没有可用抓取候选");
    sequence = result.value.last_scene_sequence;
  }
  const modeRequestId = makeId();
  onProgress?.({ phase: "mode", requestId: modeRequestId });
  await post("/api/motion/mode", {
    schema_version: schemaVersion,
    request_id: modeRequestId,
    mode: "perception",
  });
  const requestId = makeId();
  onProgress?.({ phase: "submit", requestId });
  return post("/api/perception/pick-place", {
    schema_version: schemaVersion,
    request_id: requestId,
    object_id: objectId,
    scene_sequence: sequence,
    placement_region_id: regionId,
  });
}
