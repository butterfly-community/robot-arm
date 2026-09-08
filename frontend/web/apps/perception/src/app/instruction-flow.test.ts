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
      grasp_candidates: [
        {
          confidence: 0.87,
          position_m: [0.1, 0.2, 0.05],
          orientation_xyzw: [0, 0, 0, 1],
        },
      ],
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

describe("executeInstruction", () => {
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
      post: vi.fn(async (path) => {
        calls.push(path);
        return {};
      }),
      scene: vi.fn(async () => {
        calls.push("scene");
        return scene;
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
      "scene",
      "/api/motion/mode",
      "/api/perception/pick-place",
    ]);
    expect(gateway.post).toHaveBeenLastCalledWith(
      "/api/perception/pick-place",
      expect.objectContaining({ scene_sequence: scene.sequence }),
    );
    expect(result).toMatchObject({
      request_id: "request-4",
      object_id: "red-cube-0",
      placement_region_id: "gray-bin-interior",
      accepted: true,
    });
  });

  it("never forwards a hallucinated scene id to motion", async () => {
    const gateway: RobotGateway = {
      post: vi.fn(async () => ({})),
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
    ).rejects.toThrow("不存在或没有抓取候选");
    expect(gateway.post).toHaveBeenCalledTimes(2);
  });

  it("rejects unsupported instructions before touching robot services", async () => {
    const gateway: RobotGateway = {
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
