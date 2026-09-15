import { describe, expect, it, vi } from "vitest";
import { robotTools } from "./tools";
import type { RobotToolsContext } from "./robot";
import { perceptionSnapshot } from "./robot";
vi.mock("./store", () => ({
  saveRun: vi.fn(async () => {}),
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
