import { beforeEach, describe, expect, it, vi } from "vitest";
import { simulateReadableStream } from "ai";
import { MockLanguageModelV4 } from "ai/test";
import { summarizeExperience } from "./experience-summary";
import { listExperiences, saveExperience } from "./experience";
import type { AIRun } from "./types";

vi.mock("./experience", () => ({
  listExperiences: vi.fn(async () => []),
  saveExperience: vi.fn(async () => ({ id: "saved-lesson" })),
}));
const lesson = {
  id: null,
  title: "test",
  conditions: "test",
  lesson: "test",
  nextAction: "test",
  assessment: "hypothesis",
  evidenceCallIds: [],
};
function modelFor(text: string, finish: "stop" | "length" = "stop") {
  return new MockLanguageModelV4({
    doStream: async () => ({
      stream: simulateReadableStream({
        chunks: [
          { type: "text-start", id: "summary" },
          { type: "text-delta", id: "summary", delta: text },
          { type: "text-end", id: "summary" },
          {
            type: "finish",
            finishReason: { unified: finish, raw: undefined },
            usage: {
              inputTokens: {
                total: 1,
                noCache: 1,
                cacheRead: undefined,
                cacheWrite: undefined,
              },
              outputTokens: { total: 1, text: 1, reasoning: undefined },
            },
          },
        ],
      }),
    }),
  });
}
const fixture = (state: AIRun["state"] = "succeeded") =>
  ({
    id: "run",
    input: "task",
    text: "final reply",
    state,
    calls: [{ id: "failed-attempt", error: "IK", endedAt: 1 }],
    requests: [],
    experienceScope: {
      modelRevision: "arm",
      mode: "vision",
      feedbackSource: "hardware",
    },
    experienceSummary: { state: "running", savedIds: [] },
  }) as unknown as AIRun;

describe("one end-of-run summary, never intermediate failure writes", () => {
  beforeEach(() => vi.clearAllMocks());
  it.each(["succeeded", "failed", "cancelled"] as const)(
    "summarizes settled %s results without robot tools",
    async (state) => {
      const run = fixture(state);
      const model = modelFor(JSON.stringify({ lessons: [lesson] }));
      await summarizeExperience(run, model, {}, new AbortController().signal);
      expect(model.doStreamCalls).toHaveLength(1);
      expect(model.doStreamCalls[0].tools ?? []).toEqual([]);
      const instructions = model.doStreamCalls[0].prompt.find(
        (message) => message.role === "system",
      )!.content;
      expect(instructions).toContain("无论目标是否完成，都只复盘已有记录");
      expect(instructions).toContain(
        "reply 含整个过程的公开回复，不全是最终结论",
      );
      expect(instructions).toContain("本次没有提供原图，不能声称重新看图验证");
      expect(instructions).toContain("有依据但原因或效果尚未验证");
      expect(instructions).toContain("没有新结论返回空 lessons");
      expect(instructions).not.toContain("没有完成用户目标不要停止");
      const message = model.doStreamCalls[0].prompt.find(
        (message) => message.role === "user",
      )!;
      expect(message.content).toEqual([
        expect.objectContaining({
          type: "text",
          text: expect.stringContaining(`"state":"${state}"`),
        }),
      ]);
      expect(listExperiences).toHaveBeenCalledWith(run.experienceScope);
      expect(saveExperience).toHaveBeenCalledExactlyOnceWith(run, lesson);
      expect(run.experienceSummary?.savedIds).toEqual(["saved-lesson"]);
    },
  );
  it("does not add experience when the final review has no reusable conclusion", async () => {
    await summarizeExperience(
      fixture(),
      modelFor('{"lessons":[]}'),
      {},
      new AbortController().signal,
    );
    expect(saveExperience).not.toHaveBeenCalled();
  });
  it.each([
    ['{"lessons":[', "stop"],
    [JSON.stringify({ lessons: [lesson] }), "length"],
  ] as const)(
    "does not store incomplete summary output %s",
    async (text, finish) => {
      await expect(
        summarizeExperience(
          fixture(),
          modelFor(text, finish),
          {},
          new AbortController().signal,
        ),
      ).rejects.toThrow();
      expect(saveExperience).not.toHaveBeenCalled();
    },
  );
  it("does not write after the user cancels summary", async () => {
    const controller = new AbortController();
    controller.abort();
    await expect(
      summarizeExperience(
        fixture(),
        modelFor(JSON.stringify({ lessons: [lesson] })),
        {},
        controller.signal,
      ),
    ).rejects.toThrow();
    expect(saveExperience).not.toHaveBeenCalled();
  });
});
