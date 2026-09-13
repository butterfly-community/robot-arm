import type { WorldScene } from "@robot/contracts";
import { expect, it, vi } from "vitest";
import { startPickPlace } from "./start-pick-place";

const pose = { position_m: [0, 0, 0], orientation_xyzw: [0, 0, 0, 1] } as const;
function fixture() {
  return {
    sequence: 11,
    objects: [{ object_id: "box", grasp_candidates: [] }],
    placement_regions: [{ region_id: "pad" }],
  } as unknown as WorldScene;
}
const generated = {
  value: {
    last_scene_sequence: 12,
    instances: [{ instance_id: "box", grasp_candidate_count: 8 }],
  },
};

it("one Start generates only selected-object candidates and uses the returned sequence, without waiting for WebSocket", async () => {
  const post = vi.fn().mockResolvedValue(generated);
  let id = 0;
  const progress = vi.fn();
  await startPickPlace(
    fixture(),
    "box",
    "pad",
    post,
    () => String(++id),
    progress,
  );
  expect(progress.mock.calls.map(([step]) => step)).toEqual([
    { phase: "generate_grasps", requestId: "1" },
    { phase: "mode", requestId: "2" },
    { phase: "submit", requestId: "3" },
  ]);
  expect(post.mock.calls).toEqual([
    [
      "/api/perception/request",
      expect.objectContaining({
        action: "generate_grasps",
        object_id: "box",
        input_sequence: 11,
        request_id: "1",
      }),
    ],
    [
      "/api/motion/mode",
      expect.objectContaining({ mode: "perception", request_id: "2" }),
    ],
    [
      "/api/perception/pick-place",
      expect.objectContaining({
        object_id: "box",
        placement_region_id: "pad",
        scene_sequence: 12,
        request_id: "3",
      }),
    ],
  ]);
});

it("reuses existing selected-object candidates", async () => {
  const scene = fixture();
  scene.objects[0].grasp_candidates = [
    { ...pose, confidence: 1 },
  ] as unknown as WorldScene["objects"][number]["grasp_candidates"];
  const post = vi.fn().mockResolvedValue({});
  await startPickPlace(scene, "box", "pad", post, () => "id");
  expect(post.mock.calls.map((c) => c[0])).toEqual([
    "/api/motion/mode",
    "/api/perception/pick-place",
  ]);
  expect(post.mock.calls[1][1].scene_sequence).toBe(11);
});

it.each([
  { last_scene_sequence: null, instances: [] },
  {
    last_scene_sequence: 12,
    instances: [{ instance_id: "box", grasp_candidate_count: 0 }],
  },
  {
    last_scene_sequence: 12,
    instances: [{ instance_id: "another", grasp_candidate_count: 99 }],
  },
])(
  "does not execute without this object's generated candidates: %j",
  async (value) => {
    const post = vi.fn().mockResolvedValue({ value });
    await expect(
      startPickPlace(fixture(), "box", "pad", post, () => "id"),
    ).rejects.toThrow("没有可用抓取候选");
    expect(post).toHaveBeenCalledTimes(1);
  },
);

it.each([0, 1])("does not continue when stage %i rejects", async (stage) => {
  const post = vi.fn().mockResolvedValue(generated);
  if (stage) post.mockResolvedValueOnce(generated);
  post.mockRejectedValueOnce(Error("failed"));
  await expect(
    startPickPlace(fixture(), "box", "pad", post, () => "id"),
  ).rejects.toThrow("failed");
  expect(post).toHaveBeenCalledTimes(stage + 1);
});

it.each([
  ["missing", "pad"],
  ["box", "missing"],
])("rejects stale selection %s / %s without writes", async (object, region) => {
  const post = vi.fn();
  await expect(
    startPickPlace(fixture(), object, region, post, () => "id"),
  ).rejects.toThrow("请选择");
  expect(post).not.toHaveBeenCalled();
});
