import { describe, expect, it, vi } from "vitest";
import {
  TypeValidationError,
  streamText,
  simulateReadableStream,
  type ModelMessage,
} from "ai";
import { MockLanguageModelV4 } from "ai/test";
import {
  appendReplyText,
  providerOptions,
  publicError,
  responseEffort,
} from "./runner";
import { settingsSchema, startSchema } from "./types";
import {
  depthInstructions,
  visionInstructions,
  promptMode,
  taskInstructions,
  runPrompt,
} from "./prompts";
import type { CameraCaptureState } from "@robot/contracts";
vi.mock("./store", () => ({}));
describe("Responses configuration", () => {
  it("passes live state through SDK instructions without system messages or mutable history", async () => {
    const conversation: ModelMessage[] = [
      { role: "user", content: "inspect only" },
    ];
    const prompt = runPrompt("vision", conversation, {
      camera: { selected_source_id: null },
    });
    conversation.push({ role: "assistant", content: "saved later" });
    expect(prompt.messages).toHaveLength(1);
    const model = new MockLanguageModelV4({
      doStream: async () => ({
        stream: simulateReadableStream({
          chunks: [
            { type: "text-start", id: "t" },
            { type: "text-delta", id: "t", delta: "ok" },
            { type: "text-end", id: "t" },
            {
              type: "finish",
              finishReason: { unified: "stop", raw: undefined },
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
    const response = streamText({ model, ...prompt });
    await response.consumeStream();
    expect(await response.text).toBe("ok");
    expect(model.doStreamCalls).toHaveLength(1);
    expect(JSON.stringify(model.doStreamCalls[0].prompt)).toContain("实时状态");
    expect(JSON.stringify(model.doStreamCalls[0].prompt)).not.toContain(
      "saved later",
    );
  });
  it("selects separate workflows by depth selection, not temporary signal loss", () => {
    expect(promptMode(undefined)).toBe("vision");
    expect(promptMode({ selected_source_id: null } as CameraCaptureState)).toBe(
      "vision",
    );
    expect(
      promptMode({
        selected_source_id: "depth",
        streaming: false,
      } as CameraCaptureState),
    ).toBe("depth");
    expect(taskInstructions("depth")).toBe(depthInstructions);
    expect(taskInstructions("vision")).toBe(visionInstructions);
    expect(depthInstructions).toContain(
      "capture_segmentation → segment 或 annotate → reconstruct → pick_place",
    );
    expect(visionInstructions).not.toContain("capture_segmentation →");
    expect(visionInstructions).toContain("先粗后细");
    expect(visionInstructions).toContain("不固定小步长");
    expect(visionInstructions).toContain("不把像素当米或臆造相机外参");
  });
  it("preserves approach progress across turns and avoids redundant camera switching", () => {
    expect(visionInstructions).toContain("续轮不是重新开始");
    expect(visionInstructions).toContain("之后复用，不从头试探");
    expect(visionInstructions).toContain("不反复切换视角确认");
    expect(visionInstructions).toContain("保持既有力度和速度");
    expect(visionInstructions).toContain("observe_role");
    for (const prompt of [visionInstructions, depthInstructions])
      expect(prompt).toContain("释放物体、撤开夹爪、回到工作位");
  });
  it("authorizes task-scoped tools and visual annotation without repeated permission, while respecting read-only requests", () => {
    expect(depthInstructions).toContain("任务内无需重复征求授权");
    expect(depthInstructions).toContain("基于当前图像补框");
    expect(depthInstructions).toContain("三维坐标由重建计算");
    expect(depthInstructions).toContain("提问或观察请求不运动");
  });
  it("separates model turns without breaking streamed words or adding empty tool-only paragraphs", () => {
    let text = appendReplyText("", "我先", true);
    text = appendReplyText(text, "读取状态。", false);
    expect(text).toBe("我先读取状态。");
    text = appendReplyText(text, "", true);
    text = appendReplyText(text, "状态", true);
    text = appendReplyText(text, "已读取。", false);
    expect(text).toBe("我先读取状态。\n\n状态已读取。");
    expect(appendReplyText("第一轮\n", "第二轮", true)).toBe(
      "第一轮\n\n第二轮",
    );
    expect(appendReplyText("第一轮\n\n", "第二轮", true)).toBe(
      "第一轮\n\n第二轮",
    );
  });
  it("shows the provider's failed-response error without hiding it behind an SDK schema dump", () => {
    const failure = new TypeValidationError({
      value: {
        type: "response.failed",
        response: {
          id: "resp-fixture",
          error: { code: "upstream_error", message: "Upstream request failed" },
        },
      },
      cause: Error("missing stream fields"),
    });
    expect(publicError(failure)).toBe(
      "模型服务 upstream_error: Upstream request failed（resp-fixture）",
    );
    const unrelated = new TypeValidationError({
      value: { type: "unexpected" },
      cause: Error("invalid"),
    });
    expect(publicError(unrelated)).toContain("Type validation failed");
  });
  it("reads effort only from actual SDK-decoded response metadata, never from the requested value", () => {
    expect(
      responseEffort({
        type: "response.completed",
        response: { reasoning: { effort: "low" } },
      }),
    ).toBe("low");
    expect(
      responseEffort({ type: "response.created", response: {} }),
    ).toBeUndefined();
    expect(responseEffort({ reasoning: { effort: "high" } })).toBeUndefined();
  });
  it("uses local context without hosted persistence or parallel robot calls", () => {
    expect(
      providerOptions({
        baseURL: "https://example.com/v1",
        model: "test",
        effort: "",
      }),
    ).toEqual({
      openai: {
        store: false,
        parallelToolCalls: false,
        include: ["reasoning.encrypted_content"],
      },
    });
    expect(
      providerOptions({
        baseURL: "https://example.com/v1",
        model: "test",
        effort: "high",
      }).openai.reasoningEffort,
    ).toBe("high");
  });
  it("does not return the credential in provider errors", () => {
    vi.stubEnv("AI_API_KEY", "secret-fixture");
    expect(publicError(Error("token secret-fixture rejected"))).toBe(
      "token [redacted] rejected",
    );
    vi.unstubAllEnvs();
  });
  it("validates settings and references, not physical motion thresholds", () => {
    expect(
      settingsSchema.parse({
        baseURL: "https://example.com/v1/",
        model: " test ",
        effort: "",
      }).model,
    ).toBe("test");
    expect(
      startSchema.safeParse({
        sessionId: "../secret",
        runId: "run",
        text: "test",
      }).success,
    ).toBe(false);
    expect(
      startSchema.parse({ sessionId: "session", runId: "run", text: "test" })
        .images,
    ).toEqual([]);
  });
});
