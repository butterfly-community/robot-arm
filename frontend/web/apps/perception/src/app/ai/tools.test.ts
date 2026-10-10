import { beforeEach, describe, expect, it, vi } from "vitest";
import { robotTools } from "./tools";
import type { RobotToolsContext } from "./robot";
import { perceptionSnapshot, cameraImage } from "./robot";
vi.mock("./store", () => ({
  saveRun: vi.fn(async () => {}),
  getImage: vi.fn(async (id) => ({
    id,
    width: 1920,
    height: 1080,
    mediaType: "image/png",
    bytes: Buffer.from("original"),
  })),
  putImage: vi.fn(async () => ({
    id: "fixture-image",
    width: 1920,
    height: 1080,
    mediaType: "image/png",
  })),
}));
vi.mock("./robot", async (original) => ({
  ...(await original<typeof import("./robot")>()),
  imageBytes: vi.fn(async () => Buffer.from("fixture")),
  cameraImage: vi.fn(async (role) => ({
    bytes: Buffer.from("fixture"),
    metadata: {
      role,
      source_id: "simulation:test",
      sequence: 17,
      received_time_ns: 123,
      width: 1920,
      height: 1080,
      pixel_format: "rgb8",
    },
  })),
  readRobot: vi.fn(async () => ({
    connected: true,
    force: { target: 30, feedback: 29 },
    motion: { current_tool_pose: { position_m: [0, 0, 0] } },
  })),
  robotModel: vi.fn(async () => ({
    model_revision: "test-model",
    named_targets: [
      {
        key: "work",
        joint_positions_rad: { j1: 0 },
        actuator_positions_rad: { gripper: 0 },
      },
    ],
  })),
  perceptionSnapshot: vi.fn(async () => ({
    values: {
      perception_state: {
        available_models: [{ id: "prompt-model", prompt_free: false }],
        last_segmentation_sequence: 42,
      },
    },
  })),
}));
function setup() {
  const command = vi.fn<(...args: unknown[]) => Promise<unknown>>(async () => ({
    value: { last_segmentation_sequence: 43 },
  }));
  const context = {
    command,
    signal: new AbortController().signal,
    run: { calls: [], requests: [] },
  } as unknown as RobotToolsContext;
  return { command, tools: robotTools(context) };
}
describe("AI segmentation uses the same single refresh request as the UI", () => {
  beforeEach(() => vi.clearAllMocks());
  it("recalls the original without capturing new pixels or moving", async () => {
    const { command, tools } = setup();
    const result = await tools.recall_image.execute!(
      { image_id: "old-frame" },
      { toolCallId: "recall", messages: [], context: {} },
    );
    expect(result).toMatchObject({
      image_id: "old-frame",
      width: 1920,
      height: 1080,
    });
    expect(result).not.toHaveProperty("bytes");
    expect(cameraImage).not.toHaveBeenCalled();
    expect(command).not.toHaveBeenCalled();
  });
  it("does not restart a camera that is already exclusively streaming", async () => {
    const { command, tools } = setup();
    vi.mocked(perceptionSnapshot).mockResolvedValueOnce({
      values: {
        camera_state: {
          bindings: [
            { role: "external", enabled: true, streaming: true },
            { role: "wrist", enabled: false, streaming: false },
          ],
        },
      },
    } as never);
    await tools.observe_camera.execute!(
      { role: "external", exclusive: true },
      { toolCallId: "no-switch", messages: [], context: {} },
    );
    expect(command).not.toHaveBeenCalled();
    expect(cameraImage).toHaveBeenCalledTimes(1);
  });
  it("returns post-motion camera and real feedback in one tool turn, after completion", async () => {
    const { command, tools } = setup();
    const result = await tools.move_tcp_relative.execute!(
      {
        frame: "base",
        translation_delta_m: [0, 0, 0.03],
        rotation_delta_xyzw: [0, 0, 0, 1],
        observe_role: "external",
      },
      { toolCallId: "move-see", messages: [], context: {} },
    );
    expect(command.mock.calls.map((c) => c[2])).toEqual([
      "/api/motion/mode",
      "/api/motion/request",
    ]);
    expect(command.mock.invocationCallOrder[1]).toBeLessThan(
      vi.mocked(cameraImage).mock.invocationCallOrder[0],
    );
    expect(result).toMatchObject({
      image_id: "fixture-image",
      role: "external",
      robot: { force: { target: 30, feedback: 29 } },
      result: { value: { last_segmentation_sequence: 43 } },
    });
  });
  it("does not misreport an executed motion as failed when the subsequent camera fails", async () => {
    const { command, tools } = setup();
    vi.mocked(cameraImage).mockRejectedValueOnce(Error("camera disconnected"));
    const result = await tools.gripper.execute!(
      { actuator_key: "gripper", position_rad: 0, observe_role: "wrist" },
      { toolCallId: "close-see", messages: [], context: {} },
    );
    expect(command).toHaveBeenCalledTimes(1);
    expect(command.mock.calls[0][3]).not.toHaveProperty("observe_role");
    expect(result).toMatchObject({
      result: { value: { last_segmentation_sequence: 43 } },
      observation_error: "camera disconnected",
    });
    expect(result).not.toHaveProperty("error");
  });
  it("does not capture an after-frame after failed planning or issue the action twice", async () => {
    const { command, tools } = setup();
    command.mockRejectedValueOnce(Error("IK failed"));
    const result = await tools.move_joints.execute!(
      { joints: [], observe_role: "external" },
      { toolCallId: "failed", messages: [], context: {} },
    );
    expect(result).toMatchObject({ error: "IK failed" });
    expect(command).toHaveBeenCalledTimes(1);
    expect(cameraImage).not.toHaveBeenCalled();
  });
  it("switches an exclusive observation sequentially through the same camera API", async () => {
    const { command, tools } = setup();
    vi.mocked(perceptionSnapshot).mockResolvedValueOnce({
      values: {
        camera_state: {
          bindings: [
            { role: "external", enabled: true, streaming: true },
            { role: "wrist", enabled: false, streaming: false },
          ],
        },
      },
    } as never);
    await tools.observe_camera.execute!(
      { role: "wrist", exclusive: true },
      { toolCallId: "switch-see", messages: [], context: {} },
    );
    expect(command.mock.calls.map((c) => c.slice(2, 4))).toEqual([
      ["/api/perception/camera", { role: "external", action: "disconnect" }],
      ["/api/perception/camera", { role: "wrist", action: "connect" }],
    ]);
    expect(command.mock.invocationCallOrder[1]).toBeLessThan(
      vi.mocked(cameraImage).mock.invocationCallOrder[0],
    );
  });
  it("switches capture through the same camera request without rebinding or moving", async () => {
    const { command, tools } = setup();
    const bindings = [{ role: "external", enabled: false }];
    command.mockResolvedValue({
      request_id: "camera-switch",
      value: { bindings, available_sources: [{ profiles: ["large catalog"] }] },
    });
    const result = await tools.set_camera_capture.execute!(
      { role: "external", enabled: false },
      { toolCallId: "close-camera", messages: [], context: {} },
    );
    await tools.set_camera_capture.execute!(
      { role: "wrist", enabled: true },
      { toolCallId: "open-camera", messages: [], context: {} },
    );
    expect(command.mock.calls.map((c) => c.slice(2, 4))).toEqual([
      ["/api/perception/camera", { role: "external", action: "disconnect" }],
      ["/api/perception/camera", { role: "wrist", action: "connect" }],
    ]);
    expect(result).toEqual({ request_id: "camera-switch", bindings });
  });
  it("observes the requested role without segmentation, depth or robot commands", async () => {
    const { command, tools } = setup();
    for (const role of ["external", "wrist", "depth"]) {
      const result = await tools.observe_camera.execute!(
        { role },
        { toolCallId: `observe-${role}`, messages: [], context: {} },
      );
      expect(result).toMatchObject({
        role,
        sequence: 17,
        image_id: "fixture-image",
        source_id: "simulation:test",
      });
      expect(cameraImage).toHaveBeenLastCalledWith(
        role,
        expect.any(AbortSignal),
      );
    }
    expect(command).not.toHaveBeenCalled();
  });
  it("exposes every tool for direct execution without approval gates", () => {
    const { tools } = setup();
    for (const definition of Object.values(tools)) {
      expect(definition.execute).toBeTypeOf("function");
      expect(definition.needsApproval).toBeUndefined();
    }
    expect(tools.annotate.description).toContain("任务内无需额外授权");
  });
  it("returns the registered models and saved prompt mode with the captured image, without another query", async () => {
    const { command, tools } = setup();
    command.mockResolvedValueOnce({
      value: {
        last_segmentation_sequence: 42,
        available_models: [{ id: "registered-model", prompt_free: false }],
        model: "registered-model",
        classes: ["target"],
        visual_prompt_active: true,
        placement_labels: ["paper"],
      },
    });
    const result = await tools.capture_segmentation.execute!(
      {},
      { toolCallId: "capture", messages: [], context: {} },
    );
    expect(command).toHaveBeenCalledTimes(1);
    expect(result).toMatchObject({
      image_id: "fixture-image",
      sequence: 42,
      available_models: [{ id: "registered-model", prompt_free: false }],
      saved_segmentation: {
        model: "registered-model",
        visual_prompt_active: true,
      },
    });
  });
  it("uses the new result version for annotations after model segmentation", async () => {
    const { command, tools } = setup();
    await tools.segment.execute!(
      {
        model: "prompt-model",
        prompt_mode: "saved_visual",
        prompts: [],
        placement_labels: [],
      },
      { toolCallId: "segment", messages: [], context: {} },
    );
    const current = await perceptionSnapshot();
    vi.mocked(perceptionSnapshot).mockResolvedValueOnce({
      ...current,
      values: {
        ...current.values,
        perception_state: {
          ...current.values.perception_state,
          last_segmentation_sequence: 43,
        },
      },
    });
    await tools.annotate.execute!(
      { regions: [], placement_labels: [] },
      { toolCallId: "annotate", messages: [], context: {} },
    );
    expect(command.mock.calls[1]?.[3]).toMatchObject({ input_sequence: 43 });
  });
  it("keeps the loaded frame and saved visual prompts without an Apply that clears the frame", async () => {
    const { command, tools } = setup();
    const result = await tools.segment.execute!(
      {
        model: "prompt-model",
        prompt_mode: "saved_visual",
        prompts: [],
        placement_labels: ["paper"],
      },
      { toolCallId: "segment", messages: [], context: {} },
    );
    expect(command).toHaveBeenCalledTimes(1);
    expect(result).toEqual({ last_segmentation_sequence: 43 });
    expect(command.mock.calls[0]?.[3]).toEqual({
      action: "refresh",
      input_sequence: 42,
      segmentation_edit: { kind: "model" },
      model: "prompt-model",
      placement_labels: ["paper"],
    });
  });
  it("manual annotations carry roles in the same frame edit, not a separate Apply", async () => {
    const { command, tools } = setup();
    const result = await tools.annotate.execute!(
      { regions: [], placement_labels: ["paper"] },
      { toolCallId: "annotate", messages: [], context: {} },
    );
    expect(command).toHaveBeenCalledTimes(1);
    expect(result).toEqual({ last_segmentation_sequence: 43 });
    expect(command.mock.calls[0]?.[3]).toEqual({
      action: "refresh",
      input_sequence: 42,
      segmentation_edit: { kind: "manual", regions: [] },
      placement_labels: ["paper"],
    });
  });
});
