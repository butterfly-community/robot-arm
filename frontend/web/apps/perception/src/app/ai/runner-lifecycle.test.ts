import { beforeEach, describe, expect, it, vi } from "vitest";
import type { AIRun } from "./types";

const mocks = vi.hoisted(() => ({
  controllers: new Map<string, AbortController>(),
  saved: [] as AIRun[],
  stream: vi.fn(),
  summary: vi.fn(),
  release: vi.fn(),
  command: vi.fn(),
}));
vi.mock("ai", async (original) => ({
  ...(await original<typeof import("ai")>()),
  streamText: mocks.stream,
}));
vi.mock("./store", () => ({
  runtime: () => ({ controllers: mocks.controllers }),
  appendMessages: vi.fn(),
  messages: async () => [],
  imagePart: vi.fn(),
  saveRun: async (run: AIRun) => {
    mocks.saved.push(structuredClone(run));
  },
  release: mocks.release,
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
});
