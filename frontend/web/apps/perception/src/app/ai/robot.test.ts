import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import {
  RobotToolsContext,
  waitRequest,
  namedTargetRequest,
  tcpRequest,
  compactCamera,
  scene,
} from "./robot";
import type { CameraCaptureState, RobotModelInfo } from "@robot/contracts";
import type { AIRun } from "./types";
import { saveRun } from "./store";
vi.mock("./store", () => ({ saveRun: vi.fn(async () => {}) }));
function run(): AIRun {
  return {
    id: "run",
    sessionId: "session",
    owner: "owner",
    settings: { baseURL: "https://example.com/v1", model: "test", effort: "" },
    state: "running",
    input: "test",
    images: [],
    text: "",
    startedAt: 0,
    updatedAt: 0,
    calls: [],
    requests: [],
    responseModels: [],
    responseIds: [],
    warnings: [],
  };
}
const response = (state = "succeeded") =>
  new Response(
    JSON.stringify({
      state,
      terminal: ["succeeded", "failed", "cancelled"].includes(state),
      value: { result: state },
      original_error: state === "failed" ? "IK failed" : null,
    }),
  );
describe("AI uses the same persisted robot requests", () => {
  it("waits for the acknowledged scene version instead of returning the stale empty scene", async () => {
    const snapshot = (sequence: number) =>
      new Response(
        JSON.stringify({
          values: {
            world_scene: { sequence, objects: [], placement_regions: [] },
          },
        }),
      );
    vi.stubGlobal(
      "fetch",
      vi
        .fn()
        .mockResolvedValueOnce(snapshot(8))
        .mockResolvedValueOnce(snapshot(9)),
    );
    const pending = scene(9);
    await vi.advanceTimersByTimeAsync(500);
    expect((await pending).sequence).toBe(9);
    expect(fetch).toHaveBeenCalledTimes(2);
  });
  it("scene context retains camera status without copying the entire driver catalog", () => {
    const camera = {
      streaming: true,
      selected_source_id: "camera",
      available_sources: [
        {
          source_id: "camera",
          profiles: [{ key: "RGB" }],
          driver_extensions: [],
        },
      ],
    } as unknown as CameraCaptureState;
    const result = compactCamera(camera)!;
    expect(result.streaming).toBe(true);
    expect(result.available_sources[0]).toEqual({
      source_id: "camera",
      profile_count: 1,
      driver_extension_count: 0,
    });
    expect(camera.available_sources[0].profiles).toHaveLength(1);
  });
  it("relative TCP forwards displacement and identity rotation unchanged", () => {
    expect(
      tcpRequest(
        { model_revision: "fixture" } as RobotModelInfo,
        true,
        "base",
        [0, 0, 0.01],
        [0, 0, 0, 1],
      ).tcp_target,
    ).toEqual({
      relative: true,
      pose: {
        frame: "base",
        position_m: [0, 0, 0.01],
        orientation_xyzw: [0, 0, 0, 1],
      },
    });
  });
  it("HTTP error with a known failed original record remains failed, not unknown", async () => {
    const r = run();
    vi.stubGlobal(
      "fetch",
      vi
        .fn()
        .mockResolvedValueOnce(
          new Response(JSON.stringify({ original_error: "IK failed" }), {
            status: 400,
          }),
        )
        .mockResolvedValueOnce(response("failed"))
        .mockResolvedValueOnce(response("failed")),
    );
    await expect(
      new RobotToolsContext(r, new AbortController().signal).command(
        "call",
        0,
        "/api/motion/request",
        {},
        "motion",
      ),
    ).rejects.toThrow("IK failed");
    expect(r.requests[0].state).toBe("failed");
  });
  it("work pose comes from model metadata, not the all-zero relative-control preparation", () => {
    const model = {
      model_revision: "fixture",
      named_targets: [
        {
          key: "default",
          joint_positions_rad: { j3: 0, j4: 0 },
          actuator_positions_rad: { hand: 0 },
        },
        {
          key: "work",
          joint_positions_rad: { j3: -1, j4: 1 },
          actuator_positions_rad: { hand: 0.1 },
        },
      ],
    } as unknown as RobotModelInfo;
    expect(namedTargetRequest(model, "work")).toEqual({
      action: "apply",
      model_revision: "fixture",
      options: {},
      joints: [
        { joint_key: "j3", position_rad: -1 },
        { joint_key: "j4", position_rad: 1 },
      ],
      actuators: [{ actuator_key: "hand", position_rad: 0.1 }],
    });
    expect(() => namedTargetRequest(model, "missing")).toThrow("没有命名姿态");
  });
  beforeEach(() => vi.useFakeTimers());
  afterEach(() => {
    vi.useRealTimers();
    vi.unstubAllGlobals();
    vi.clearAllMocks();
  });
  it("persists the deterministic ID before POST and never treats acceptance as completion", async () => {
    const r = run(),
      saved = vi.mocked(saveRun),
      fetch = vi.fn();
    fetch
      .mockImplementationOnce(async (_url, options) => {
        expect(saved).toHaveBeenCalled();
        expect(JSON.parse(options.body).request_id).toBe("run:call:0");
        return response("accepted");
      })
      .mockResolvedValueOnce(response("executing"))
      .mockResolvedValueOnce(response());
    vi.stubGlobal("fetch", fetch);
    let finished = false;
    const pending = new RobotToolsContext(r, new AbortController().signal)
      .command(
        "call",
        0,
        "/api/motion/request",
        { request_id: "wrong" },
        "motion",
      )
      .then((v) => {
        finished = true;
        return v;
      });
    await vi.advanceTimersByTimeAsync(1);
    expect(finished).toBe(false);
    await vi.advanceTimersByTimeAsync(500);
    expect(await pending).toEqual({
      request_id: "run:call:0",
      value: { result: "succeeded" },
    });
    expect(
      fetch.mock.calls.filter(([, o]) => o?.method === "POST"),
    ).toHaveLength(1);
  });
  it("queries an existing request without replaying it", async () => {
    const r = run();
    r.requests.push({
      id: "run:call:0",
      path: "/api/motion/request",
      label: "motion",
      state: "accepted",
    });
    const fetch = vi.fn(async () => response());
    vi.stubGlobal("fetch", fetch);
    await new RobotToolsContext(r, new AbortController().signal).command(
      "call",
      0,
      "/api/motion/request",
      {},
      "motion",
    );
    expect(fetch.mock.calls).toHaveLength(1);
    expect(fetch.mock.calls[0]).not.toContainEqual(
      expect.objectContaining({ method: "POST" }),
    );
  });
  it("ambiguous POST is queried, never retried", async () => {
    const fetch = vi
      .fn()
      .mockRejectedValueOnce(Error("connection lost"))
      .mockResolvedValueOnce(response("executing"))
      .mockResolvedValueOnce(response());
    vi.stubGlobal("fetch", fetch);
    await new RobotToolsContext(run(), new AbortController().signal).command(
      "call",
      0,
      "/api/motion/request",
      {},
      "motion",
    );
    expect(
      fetch.mock.calls.filter(([, o]) => o?.method === "POST"),
    ).toHaveLength(1);
  });
  it("propagates failed terminal result instead of success", async () => {
    vi.stubGlobal(
      "fetch",
      vi
        .fn()
        .mockResolvedValueOnce(response("accepted"))
        .mockResolvedValueOnce(response("failed")),
    );
    await expect(
      new RobotToolsContext(run(), new AbortController().signal).command(
        "call",
        0,
        "/api/motion/request",
        {},
        "motion",
      ),
    ).rejects.toThrow("IK failed");
  });
  it("never starts a new command after stop", async () => {
    const controller = new AbortController();
    controller.abort();
    const fetch = vi.fn();
    vi.stubGlobal("fetch", fetch);
    await expect(
      new RobotToolsContext(run(), controller.signal).command(
        "call",
        0,
        "/api/motion/request",
        {},
        "motion",
      ),
    ).rejects.toThrow("不再下发");
    expect(fetch).not.toHaveBeenCalled();
  });
  it("cancel acknowledgement is not the original action terminal state", async () => {
    const controller = new AbortController(),
      r = run();
    const fetch = vi
      .fn()
      .mockImplementationOnce(async () => {
        controller.abort();
        return response("accepted");
      })
      .mockResolvedValueOnce(response("executing"))
      .mockResolvedValueOnce(response())
      .mockResolvedValueOnce(response("cancelled"));
    vi.stubGlobal("fetch", fetch);
    const pending = new RobotToolsContext(r, controller.signal).command(
      "call",
      0,
      "/api/motion/request",
      {},
      "motion",
    );
    const rejected = expect(pending).rejects.toThrow("cancelled");
    await vi.advanceTimersByTimeAsync(1);
    expect(r.requests[0].state).toBe("executing");
    await vi.advanceTimersByTimeAsync(500);
    await rejected;
    expect(r.requests[0].state).toBe("cancelled");
    expect(
      fetch.mock.calls.some(([url]) =>
        String(url).endsWith("/api/motion/cancel"),
      ),
    ).toBe(true);
  });
  it("unknown persisted result is not silently called success", async () => {
    vi.stubGlobal(
      "fetch",
      vi.fn(async () => response("unknown")),
    );
    expect((await waitRequest("unknown")).state).toBe("unknown");
  });
});
