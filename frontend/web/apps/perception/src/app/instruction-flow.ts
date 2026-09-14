import {
  schemaVersion,
  type PerceptionModelInfo,
  type PerceptionState,
  type WorldScene,
} from "@robot/contracts";
import { z } from "zod";
import { startPickPlace } from "./start-pick-place";

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
    model: string;
    prompt_free: boolean;
  };

export interface InstructionPlanner {
  plan(
    instruction: string,
    model: PerceptionModelInfo,
  ): Promise<PerceptionPlan>;
  select(instruction: string, scene: WorldScene): Promise<SceneSelection>;
}

export interface RobotGateway {
  model(): Promise<PerceptionModelInfo>;
  post(path: string, body: Record<string, unknown>): Promise<unknown>;
  scene(): Promise<WorldScene>;
}

export async function executeInstruction(
  instruction: string,
  planner: InstructionPlanner,
  gateway: RobotGateway,
  makeRequestId: () => string = () => crypto.randomUUID(),
): Promise<InstructionResult> {
  const model = await gateway.model();
  const plan = await planner.plan(instruction, model);
  if (plan.action !== "pick_place") {
    throw new Error(plan.reason || "当前自然语言入口只执行抓放任务");
  }
  if (
    !model.prompt_free &&
    (plan.perception_prompts.length === 0 || plan.placement_labels.length === 0)
  ) {
    throw new Error("AI 没有返回抓取目标和放置区域所需的识别提示词");
  }
  const promptSet = new Set(plan.perception_prompts);
  if (
    !model.prompt_free &&
    plan.placement_labels.some((label) => !promptSet.has(label))
  ) {
    throw new Error("AI 返回的放置区域角色不在识别提示词中");
  }

  await gateway.post("/api/perception/request", {
    schema_version: schemaVersion,
    request_id: makeRequestId(),
    action: "apply",
    model: model.id,
    classes: model.prompt_free ? null : plan.perception_prompts,
    placement_labels: model.prompt_free ? null : plan.placement_labels,
  });
  const segmented = (await gateway.post("/api/perception/request", {
    schema_version: schemaVersion,
    request_id: makeRequestId(),
    action: "refresh",
    classes: null,
    placement_labels: null,
  })) as { value: PerceptionState };
  await gateway.post("/api/perception/request", {
    schema_version: schemaVersion,
    request_id: makeRequestId(),
    action: "reconstruct",
    input_sequence: segmented.value.last_segmentation_sequence,
  });

  const scene = await gateway.scene();
  const selection = await planner.select(instruction, scene);
  // AI selects IDs; generation, fresh scene binding and submission are exactly
  // the same implementation used by the manual Start button.
  let requestId = "";
  await startPickPlace(
    scene,
    selection.object_id,
    selection.placement_region_id,
    (path, body) => gateway.post(path, body),
    makeRequestId,
    (step) => {
      if (step.phase === "submit") requestId = step.requestId;
    },
  );
  return {
    request_id: requestId,
    accepted: true,
    ...plan,
    ...selection,
    model: model.id,
    prompt_free: model.prompt_free,
    perception_prompts: model.prompt_free ? [] : plan.perception_prompts,
    placement_labels: model.prompt_free ? [] : plan.placement_labels,
  };
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
