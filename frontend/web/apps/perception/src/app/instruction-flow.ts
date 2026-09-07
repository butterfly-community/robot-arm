import { schemaVersion, type WorldScene } from "@robot/contracts";
import { z } from "zod";

export const instructionSchema = z.object({
  instruction: z.string().trim().min(1, "请输入要执行的任务"),
});

export const perceptionPlanSchema = z.object({
  action: z.enum(["pick_place", "unsupported"]),
  reason: z.string(),
  perception_prompts: z
    .array(z.string().trim().min(1))
    .describe("开放词汇视觉模型使用的简短英文名词短语"),
  placement_labels: z
    .array(z.string().trim().min(1))
    .describe("perception_prompts 中作为放置目标的词组"),
});

export const sceneSelectionSchema = z.object({
  object_id: z.string().min(1),
  placement_region_id: z.string().min(1),
});

export type PerceptionPlan = z.infer<typeof perceptionPlanSchema>;
export type SceneSelection = z.infer<typeof sceneSelectionSchema>;

export type InstructionResult = PerceptionPlan &
  SceneSelection & {
    request_id: string;
    accepted: true;
  };

export interface InstructionPlanner {
  plan(instruction: string): Promise<PerceptionPlan>;
  select(instruction: string, scene: WorldScene): Promise<SceneSelection>;
}

export interface RobotGateway {
  post(path: string, body: Record<string, unknown>): Promise<unknown>;
  scene(): Promise<WorldScene>;
}

export async function executeInstruction(
  instruction: string,
  planner: InstructionPlanner,
  gateway: RobotGateway,
  makeRequestId: () => string = () => crypto.randomUUID(),
): Promise<InstructionResult> {
  const plan = await planner.plan(instruction);
  if (plan.action !== "pick_place") {
    throw new Error(plan.reason || "当前自然语言入口只执行抓放任务");
  }
  if (
    plan.perception_prompts.length === 0 ||
    plan.placement_labels.length === 0
  ) {
    throw new Error("AI 没有返回抓取目标和放置区域所需的识别提示词");
  }
  const promptSet = new Set(plan.perception_prompts);
  if (plan.placement_labels.some((label) => !promptSet.has(label))) {
    throw new Error("AI 返回的放置区域角色不在识别提示词中");
  }

  await gateway.post("/api/perception/request", {
    schema_version: schemaVersion,
    request_id: makeRequestId(),
    action: "apply",
    classes: plan.perception_prompts,
    placement_labels: plan.placement_labels,
  });
  await gateway.post("/api/perception/request", {
    schema_version: schemaVersion,
    request_id: makeRequestId(),
    action: "refresh",
    classes: null,
    placement_labels: null,
  });

  const scene = await gateway.scene();
  const selection = await planner.select(instruction, scene);
  const object = scene.objects.find(
    (item) => item.object_id === selection.object_id,
  );
  if (!object || object.grasp_candidates.length === 0) {
    throw new Error("AI 选择的抓取目标不存在或没有抓取候选");
  }
  if (
    !scene.placement_regions.some(
      (item) => item.region_id === selection.placement_region_id,
    )
  ) {
    throw new Error("AI 选择的放置区域不存在");
  }

  await gateway.post("/api/motion/mode", {
    schema_version: schemaVersion,
    request_id: makeRequestId(),
    mode: "perception",
  });
  const requestId = makeRequestId();
  await gateway.post("/api/perception/pick-place", {
    schema_version: schemaVersion,
    request_id: requestId,
    object_id: selection.object_id,
    scene_sequence: scene.sequence,
    placement_region_id: selection.placement_region_id,
  });
  return { request_id: requestId, accepted: true, ...plan, ...selection };
}

export function compactScene(scene: WorldScene) {
  return {
    frame_id: scene.frame_id,
    objects: scene.objects.map((object) => ({
      object_id: object.object_id,
      label: object.label,
      confidence: object.confidence,
      position_m: object.pose.position_m,
      size_m: object.size_m,
      grasp_candidate_count: object.grasp_candidates.length,
    })),
    placement_regions: scene.placement_regions.map((region) => ({
      placement_region_id: region.region_id,
      label: region.label,
      position_m: region.pose.position_m,
      size_m: region.size_m,
    })),
  };
}
