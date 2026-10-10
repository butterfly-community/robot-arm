import { describe, expect, it, vi } from "vitest";
import { TypeValidationError } from "ai";
import {
  appendReplyText,
  instructions,
  providerOptions,
  publicError,
  responseEffort,
} from "./runner";
import { settingsSchema, startSchema } from "./types";
vi.mock("./store", () => ({}));
describe("Responses configuration", () => {
  it("directs visual approach from coarse motion to contact refinement without fixed steps or invented coordinates", () => {
    expect(instructions).toContain("应先大幅接近目标位置");
    expect(instructions).toContain("即将接触时才转为小幅精调");
    expect(instructions).toContain("不预设固定步长或接近姿态");
    expect(instructions).toContain("不把像素直接当米，不臆造相机外参");
    expect(instructions).toContain("每段运动完成后观察结果");
    expect(instructions).toContain("仍由现有规划与执行工具完成运动");
  });
  it("preserves approach progress across turns and avoids redundant camera switching", () => {
    expect(instructions).toContain("不因对话续轮而重新回工作位");
    expect(instructions).toContain("不机械地在每次移动前后切换两台相机");
    expect(instructions).toContain("所需相机已经正常采集时直接观察");
    expect(instructions).toContain("不重复从头探测");
    expect(instructions).toContain("力/速度");
  });
  it("authorizes task-scoped tools and visual annotation without repeated permission, while respecting read-only requests", () => {
    expect(instructions).toContain(
      "用户提交任务即授权你使用已提供工具完成该任务",
    );
    expect(instructions).toContain("调用 annotate 补充像素框，无需额外授权");
    expect(instructions).toContain("三维位置仍由重建工具计算");
    expect(instructions).toContain("用户只问问题或观察时不运动");
    expect(instructions).not.toContain("仅在用户请求框选或明确允许时使用");
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
