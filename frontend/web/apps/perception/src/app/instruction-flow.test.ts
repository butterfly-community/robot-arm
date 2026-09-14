import type { WorldScene } from "@robot/contracts";
import { describe, expect, it, vi } from "vitest";

import {
  executeInstruction,
  type InstructionPlanner,
  type RobotGateway,
} from "./instruction-flow";

const scene: WorldScene = {
  schema_version: 1,
  sequence: 4,
  sample_time_ns: 5,
  frame_id: "base_link",
  objects: [
    {
      object_id: "red-cube-0",
      label: "red cube",
      confidence: 0.9,
      pose: {
        position_m: [0.1, 0.2, 0.03],
        orientation_xyzw: [0, 0, 0, 1],
      },
      size_m: [0.04, 0.04, 0.04],
      grasp_candidates: [],
    },
  ],
  placement_regions: [
    {
      region_id: "gray-bin-interior",
      label: "gray storage bin interior",
      pose: {
        position_m: [0.3, 0.1, 0.08],
        orientation_xyzw: [0, 0, 0, 1],
      },
      size_m: [0.12, 0.1, 0.08],
      source_object_id: "gray-bin-0",
    },
  ],
  obstacles: [],
};
const graspScene: WorldScene = structuredClone(scene);
graspScene.sequence = 5;
graspScene.objects[0].grasp_candidates = [
  {
    confidence: 0.87,
    position_m: [0.1, 0.2, 0.05],
    orientation_xyzw: [0, 0, 0, 1],
  },
];

const stageResult = {
  value: {
    last_segmentation_sequence: 3,
    last_scene_sequence: graspScene.sequence,
    instances: [{ instance_id: "red-cube-0", grasp_candidate_count: 1 }],
  },
};

describe("executeInstruction", () => {
  it("uses the selected automatic model without applying hidden prompts", async () => {
    const model = { id: "automatic", label: "Automatic", prompt_free: true };
    const gateway: RobotGateway = {
      model: vi.fn(async () => model),
      post: vi.fn(async () => stageResult),
      scene: vi.fn().mockResolvedValueOnce(scene).mockResolvedValue(graspScene),
    };
    const planner: InstructionPlanner = {
      plan: vi.fn().mockResolvedValue({
        action: "pick_place",
        reason: "",
        perception_prompts: [],
        placement_labels: [],
      }),
      select: vi.fn().mockResolvedValue({
        object_id: "red-cube-0",
        placement_region_id: "gray-bin-interior",
      }),
    };
    const result = await executeInstruction("把方块放进筐", planner, gateway);
    expect(planner.plan).toHaveBeenCalledWith("把方块放进筐", model);
    expect(gateway.post).toHaveBeenNthCalledWith(
      1,
      "/api/perception/request",
      expect.objectContaining({
        action: "apply",
        model: "automatic",
        classes: null,
        placement_labels: null,
      }),
    );
    expect(result).toMatchObject({
      model: "automatic",
      prompt_free: true,
      perception_prompts: [],
    });
  });

  it("wraps the existing manual perception and pick-place calls in order", async () => {
    const planner: InstructionPlanner = {
      plan: vi.fn().mockResolvedValue({
        action: "pick_place",
        reason: "",
        perception_prompts: ["red cube", "gray storage bin"],
        placement_labels: ["gray storage bin"],
      }),
      select: vi.fn().mockResolvedValue({
        object_id: "red-cube-0",
        placement_region_id: "gray-bin-interior",
      }),
    };
    const calls: string[] = [];
    const gateway: RobotGateway = {
      model: vi.fn(async () => ({
        id: "prompted",
        label: "Prompted",
        prompt_free: false,
      })),
      post: vi.fn(async (path) => {
        calls.push(path);
        return stageResult;
      }),
      scene: vi.fn(async () => {
        calls.push("scene");
        return scene; // Remains old: submission must use the generation response.
      }),
    };
    let sequence = 0;

    const result = await executeInstruction(
      "把红色方块放进灰色筐",
      planner,
      gateway,
      () => `request-${++sequence}`,
    );

    expect(calls).toEqual([
      "/api/perception/request",
      "/api/perception/request",
      "/api/perception/request",
      "scene",
      "/api/perception/request",
      "/api/motion/mode",
      "/api/perception/pick-place",
    ]);
    expect(gateway.post).toHaveBeenLastCalledWith(
      "/api/perception/pick-place",
      expect.objectContaining({ scene_sequence: graspScene.sequence }),
    );
    expect(planner.select).toHaveBeenCalledWith("把红色方块放进灰色筐", scene);
    expect(gateway.post).toHaveBeenNthCalledWith(
      3,
      "/api/perception/request",
      expect.objectContaining({ action: "reconstruct", input_sequence: 3 }),
    );
    expect(gateway.post).toHaveBeenNthCalledWith(
      4,
      "/api/perception/request",
      expect.objectContaining({
        action: "generate_grasps",
        input_sequence: scene.sequence,
        object_id: "red-cube-0",
      }),
    );
    expect(result).toMatchObject({
      request_id: "request-6",
      object_id: "red-cube-0",
      placement_region_id: "gray-bin-interior",
      accepted: true,
    });
  });

  it("never forwards a hallucinated scene id to motion", async () => {
    const gateway: RobotGateway = {
      model: vi.fn(async () => ({
        id: "prompted",
        label: "Prompted",
        prompt_free: false,
      })),
      post: vi.fn(async () => stageResult),
      scene: vi.fn(async () => scene),
    };
    const planner: InstructionPlanner = {
      plan: vi.fn().mockResolvedValue({
        action: "pick_place",
        reason: "",
        perception_prompts: ["red cube", "gray storage bin"],
        placement_labels: ["gray storage bin"],
      }),
      select: vi.fn().mockResolvedValue({
        object_id: "invented-object",
        placement_region_id: "gray-bin-interior",
      }),
    };

    await expect(
      executeInstruction("抓起来", planner, gateway, () => "request"),
    ).rejects.toThrow("请选择当前场景");
    expect(gateway.post).toHaveBeenCalledTimes(3);
  });

  it("rejects unsupported instructions before touching robot services", async () => {
    const gateway: RobotGateway = {
      model: vi.fn(async () => ({
        id: "prompted",
        label: "Prompted",
        prompt_free: false,
      })),
      post: vi.fn(async () => ({})),
      scene: vi.fn(async () => scene),
    };
    const planner: InstructionPlanner = {
      plan: vi.fn().mockResolvedValue({
        action: "unsupported",
        reason: "当前入口只编排抓放任务",
        perception_prompts: [],
        placement_labels: [],
      }),
      select: vi.fn(),
    };

    await expect(
      executeInstruction("看看桌面上有什么", planner, gateway),
    ).rejects.toThrow("当前入口只编排抓放任务");
    expect(gateway.post).not.toHaveBeenCalled();
    expect(gateway.scene).not.toHaveBeenCalled();
  });
});
