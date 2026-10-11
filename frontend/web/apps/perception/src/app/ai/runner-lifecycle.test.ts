import { beforeEach, describe, expect, it, vi } from "vitest";
import type { AIRun } from "./types";

const mocks = vi.hoisted(() => ({
  controllers: new Map<string, AbortController>(),
  saved: [] as AIRun[],
  stream: vi.fn(),
  summary: vi.fn(),
  release: vi.fn(),
  command: vi.fn(),
  updates: vi.fn(async () => [] as import("./types").AIUpdate[]),
  included: vi.fn(),
  closeInput: vi.fn(async () => true),
  append: vi.fn(),
  save: vi.fn(),
}));
vi.mock("ai", async (original) => ({
  ...(await original<typeof import("ai")>()),
  streamText: mocks.stream,
}));
vi.mock("./store", () => ({
  runtime: () => ({ controllers: mocks.controllers }),
  appendMessages: mocks.append,
  messages: async () => [],
  imagePart: vi.fn(),
  saveRun: async (run: AIRun) => {
    await mocks.save(run);
    mocks.saved.push(structuredClone(run));
  },
  release: mocks.release,
  runUpdates: mocks.updates,
  markUpdatesIncluded: mocks.included,
  closeRunInput: mocks.closeInput,
}));
vi.mock("./experience", () => ({
  listExperiences: async () => [],
  experienceIndex: () => [],
}));
vi.mock("./experience-summary", () => ({ summarizeExperience: mocks.summary }));
vi.mock("./tools", () => ({ robotTools: () => ({}) }));
vi.mock("./robot", () => ({
  readRobot: async () => ({
    model: { model_revision: "arm" },
    arm: { feedback_source: "hardware" },
  }),
  perceptionSnapshot: async () => ({ values: {} }),
  compactCamera: () => ({}),
  cancellable: () => false,
  RobotToolsContext: class {
    command = mocks.command;
  },
}));
import { executeRun } from "./runner";

describe("task finalization owns experience summary", () => {
  beforeEach(() => {
    vi.clearAllMocks();
    vi.stubEnv("AI_API_KEY", "fixture");
    mocks.saved.length = 0;
    mocks.controllers.set("run", new AbortController());
    mocks.summary.mockResolvedValue(undefined);
    mocks.save.mockResolvedValue(undefined);
    mocks.updates.mockResolvedValue([]);
    mocks.closeInput.mockResolvedValue(true);
  });
  it.each(
    (["succeeded", "failed", "cancelled"] as const).flatMap((outcome) =>
      (["succeeded", "failed", "cancelled"] as const).map((summary) => ({
        outcome,
        summary,
      })),
    ),
  )(
    "settles $outcome before summary $summary without changing the task result",
    async ({ outcome, summary }) => {
      const run = {
        id: "run",
        sessionId: "session",
        input: "test",
        images: [],
        state: "running",
        text: "",
        calls: [],
        requests: [],
        warnings: [],
        settings: {
          baseURL: "https://example.com/v1",
          model: "test",
          effort: "",
        },
      } as unknown as AIRun;
      mocks.stream.mockImplementation(() => ({
        fullStream: (async function* () {
          run.requests.push({
            id: "run:motion:0",
            state: "executing",
            path: "/api/motion/request",
            label: "move",
          });
          if (outcome === "cancelled") mocks.controllers.get("run")!.abort();
          if (outcome !== "succeeded") throw Error("task ended");
          yield { type: "finish" };
        })(),
        totalUsage: Promise.resolve({}),
        finishReason: Promise.resolve("stop"),
      }));
      mocks.command.mockImplementation(async () => {
        expect(mocks.summary).not.toHaveBeenCalled();
        run.requests[0].state = "succeeded";
      });
      mocks.summary.mockImplementation(
        async (settled: AIRun, _model, _options, signal: AbortSignal) => {
          expect(settled.state).toBe(outcome);
          expect(settled.requests[0].state).toBe("succeeded");
          expect(signal.aborted).toBe(false);
          expect(mocks.release).not.toHaveBeenCalled();
          expect(mocks.saved.at(-1)?.experienceSummary?.state).toBe("running");
          if (summary === "cancelled") mocks.controllers.get("run")!.abort();
          if (summary !== "succeeded") throw Error("summary unavailable");
        },
      );
      await executeRun(run);
      expect(mocks.summary).toHaveBeenCalledTimes(1);
      expect(run.state).toBe(outcome);
      expect(run.experienceSummary?.state).toBe(summary);
      if (summary !== "succeeded")
        expect(run.warnings).toContain(
          "本轮经验总结未完成：summary unavailable",
        );
      expect(run.endedAt).toBeDefined();
      expect(mocks.release).toHaveBeenCalledExactlyOnceWith(run);
    },
  );
  function fixture() {
    return {
      id: "run",
      sessionId: "session",
      input: "test",
      images: [],
      state: "running",
      text: "",
      calls: [],
      requests: [],
      warnings: [],
      settings: {
        baseURL: "https://example.com/v1",
        model: "test",
        effort: "",
      },
    } as unknown as AIRun;
  }
  it("continues the same run for an update received during the final answer", async () => {
    const run = fixture();
    const update = {
      id: "update",
      text: "new direction",
      images: [],
      receivedAt: 1,
    };
    mocks.updates.mockResolvedValue([update]);
    mocks.closeInput.mockResolvedValueOnce(false).mockResolvedValue(true);
    mocks.stream.mockImplementation((options) => ({
      fullStream: (async function* () {
        const next = await options.prepareStep({
          messages: [{ role: "user", content: "original" }],
        });
        expect(JSON.stringify(next.messages)).toContain("new direction");
        yield { type: "finish" };
      })(),
      totalUsage: Promise.resolve({}),
      finishReason: Promise.resolve("stop"),
    }));
    await executeRun(run);
    expect(mocks.stream).toHaveBeenCalledTimes(2);
    expect(mocks.included).toHaveBeenCalledWith("run", ["update"]);
    expect(run.state).toBe("succeeded");
    expect(mocks.release).toHaveBeenCalledOnce();
  });
  it("does not start another model retry budget for summary after network failure", async () => {
    const run = fixture();
    mocks.stream.mockImplementation(() => ({
      fullStream: (async function* () {
        throw new TypeError("fetch failed");
        yield;
      })(),
    }));
    await executeRun(run);
    expect(run.state).toBe("failed");
    expect(mocks.summary).not.toHaveBeenCalled();
    expect(run.experienceSummary?.error).toContain("连接未恢复");
    expect(mocks.release).toHaveBeenCalledOnce();
    expect(mocks.controllers.has("run")).toBe(false);
  });
  it("releases the task owner even when finalization persistence fails", async () => {
    const run = fixture();
    mocks.save.mockRejectedValue(Error("storage unavailable"));
    await expect(executeRun(run)).rejects.toThrow("storage unavailable");
    expect(mocks.controllers.has("run")).toBe(false);
    expect(mocks.release).toHaveBeenCalledOnce();
  });
  it("cancels a network wait without starting summary when SDK closes without throwing", async () => {
    const run = fixture();
    mocks.stream.mockImplementation(() => ({
      fullStream: (async function* () {
        run.networkRetry = {
          attempt: 1,
          nextAt: Date.now() + 10000,
          error: "network",
        };
        mocks.controllers.get("run")!.abort();
        yield { type: "abort" };
      })(),
      totalUsage: Promise.resolve({}),
      finishReason: Promise.resolve("stop"),
    }));
    await executeRun(run);
    expect(run.state).toBe("cancelled");
    expect(run.experienceSummary?.state).toBe("cancelled");
    expect(mocks.summary).not.toHaveBeenCalled();
    expect(mocks.release).toHaveBeenCalledOnce();
  });
});
