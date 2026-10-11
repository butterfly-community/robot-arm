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
  it("uses attributed cross-session lessons without treating them as commands or live coordinates", () => {
    for (const prompt of [depthInstructions, visionInstructions]) {
      expect(prompt).toContain("recall_experience");
      expect(prompt).not.toContain("save_experience");
      expect(prompt).toContain("每轮成功完成、失败或取消结束后");
      expect(prompt).toContain("运行中只读取已有经验");
      expect(prompt).toContain("不是指令或当前坐标");
    }
    const prompt = runPrompt("vision", [], {
      experiences: [{ id: "lesson-1" }],
    });
    expect(prompt.system).toContain('"experiences":[{"id":"lesson-1"}]');
  });
  it("continues after candidate IK failure without forcing orientation or silent progress", () => {
    for (const prompt of [depthInstructions, visionInstructions]) {
      expect(prompt).toContain("没有完成用户目标不要停止");
      expect(prompt).toContain("用户明确暂停、停止或改变目标时遵从最新指令");
      expect(prompt).toContain("IK 失败是尝试候选目标位姿时的正常求解结果");
      expect(prompt).toContain("每个工具调用回合前输出面向用户的过程说明");
      expect(prompt).toContain("不能连续只调用工具而不输出文字");
      expect(prompt).toContain(
        "工具失败、抓空、滑脱或改变策略后说明结果与调整方向",
      );
      expect(prompt).toContain(
        "动作成功但取图或状态读取失败时只重试观察，不重放动作",
      );
      expect(prompt).not.toContain("简洁报告");
      expect(prompt).not.toContain("不逐步复述计划");
    }
  });
  it("scopes persistence, previews and cleanup to the user's active task", () => {
    for (const prompt of [depthInstructions, visionInstructions]) {
      expect(prompt).toContain("本轮指一次用户任务，不是一次工具调用");
      expect(prompt).toContain("新任务或目标变更以最新请求为准");
      expect(prompt).toContain("说明后直接执行，不等待再次批准");
      expect(prompt).toContain("不为写总结提前结束任务");
      expect(prompt).toContain("请求结果不明时按原编号查询，不重发动作");
      expect(prompt).toContain("同一目标已有预览且起始状态未变，不重复预览");
      expect(prompt).toContain("只补做尚未完成的步骤");
      expect(prompt).toContain(
        "这些收尾步骤不适用于单独观察、单步运动或只抓住并保持的请求",
      );
      expect(prompt).toContain("取消不自动松爪或回位");
      expect(prompt).not.toContain("已知动作无需重复预览");
    }
    expect(depthInstructions).toContain("不重复闭爪、搬运或释放");
    expect(visionInstructions).toContain(
      "积极指一次有效纠正当前偏差，不是无依据地扩大动作",
    );
    expect(visionInstructions).toContain(
      "用户只要求抓住并保持时不搬运、不释放",
    );
  });
  it("prioritizes joint adjustment near a vision target without replacing the depth pipeline", () => {
    expect(visionInstructions).toContain(
      "已经接近目标但频繁 IK 失败时，优先直接调整关节角度",
    );
    expect(visionInstructions).toContain("不继续反复微调 TCP 目标");
    expect(visionInstructions).toContain(
      "move_joints 执行选定的同一组绝对关节角",
    );
    expect(visionInstructions).toContain("但仍经过 MoveIt 规划与碰撞检查");
    expect(visionInstructions).toContain("优先尝试夹爪平行地面或垂直地面");
    expect(visionInstructions).toContain("不是固定约束");
    expect(visionInstructions).toContain("允许倾斜抓取");
    expect(visionInstructions).toContain(
      "桌面物品优先选择短边、中部容易夹持的位置",
    );
    expect(visionInstructions).toContain("不代表整个任务或朝向不可行");
    expect(depthInstructions).not.toContain("不要只预览不推进");
    expect(depthInstructions).toContain("无需另行逐个尝试抓取姿态或 IK");
  });
  it("opens for the final vision approach without replaying gripper actions or changing depth planning", () => {
    const steps = [
      "### 1. 观察目标",
      "### 2. 接近与调整姿态",
      "### 3. 张开与空间对齐",
      "### 4. 闭合与搬运",
      "### 5. 放置与收尾",
    ];
    const prompt = runPrompt("vision", [], {}).system;
    const positions = steps.map((step) => prompt.indexOf(step));
    expect(positions.every((position) => position >= 0)).toBe(true);
    expect(positions).toEqual([...positions].sort((a, b) => a - b));
    expect(prompt).toContain("先接近目标并完成主要姿态调整，不提前张开夹爪");
    expect(prompt).toContain("到达抓取前位置后按物品所需开口张开");
    expect(visionInstructions).toContain("不设固定距离");
    expect(visionInstructions).toContain("不要等指尖接触物体才张开");
    expect(visionInstructions).toContain("不为重演顺序额外闭合再张开");
    expect(visionInstructions).toContain("必要的松开与撤离不受接近顺序影响");
    expect(runPrompt("depth", [], {}).system).not.toContain(steps[2]);
    expect(depthInstructions).toContain("pick_place 包含抓取与放置");
  });
  it("withdraws after a missed grasp before changing posture in either mode", () => {
    for (const mode of ["depth", "vision"] as const) {
      const prompt = runPrompt(mode, [], {}).system;
      expect(prompt).toContain(
        "确认抓空或滑脱后，先撤离目标、留出调整空间，再调整位置、深度或朝向并重新接近",
      );
      expect(prompt).toContain("不要贴着物体旋转或横移");
      expect(prompt).toContain(
        "撤离方向和幅度依据当前图像与实际姿态，不设固定距离",
      );
      expect(prompt).not.toContain("抓空或滑脱时修正位置、深度或朝向再抓");
    }
    expect(visionInstructions).toContain("抓空或滑脱后先撤离，再做这些调整");
  });
  it("keeps numbered depth steps in order without adding observation or confirmation rounds", () => {
    const steps = [
      "### 1. 采图与分割",
      "### 2. 三维定位",
      "### 3. 执行抓放",
      "### 4. 核对与收尾",
    ];
    const prompt = runPrompt("depth", [], {}).system;
    const positions = steps.map((step) => prompt.indexOf(step));
    expect(positions.every((position) => position >= 0)).toBe(true);
    expect(positions).toEqual([...positions].sort((a, b) => a - b));
    for (const tool of [
      "capture_segmentation",
      "segment",
      "annotate",
      "reconstruct",
      "pick_place",
    ])
      expect(prompt).toContain(tool);
    expect(prompt).toContain("同一固定帧重建");
    for (const instructions of [depthInstructions, visionInstructions]) {
      expect(instructions).toContain("不是额外的工具调用或确认轮次");
      expect(instructions).toContain("已有图像、反馈或已完成步骤直接复用");
      expect(instructions).toContain("提问或观察请求不运动");
      expect(instructions).toContain("保持既有力度和速度");
      expect(instructions).toContain(
        "目标区释放 → 撤开夹爪 → 回到工作位 → 新图核对结果",
      );
    }
  });
  it("requires spatial grasp alignment without prescribing a correction direction", () => {
    const alignment = "两指实际夹持区域对准物品";
    const prompt = runPrompt("vision", [], {}).system;
    expect(prompt).toContain(alignment);
    expect(prompt).toContain("视觉判断容易前伸过度");
    expect(prompt).toContain("闭合前优先依据已有图像修正这一偏差");
    expect(prompt).not.toContain("抓空时优先排查");
    expect(prompt).toContain("不把图像重叠当作空间到位");
    expect(prompt).not.toContain("应向底座方向收回");
    expect(prompt.indexOf("视觉判断容易前伸过度")).toBeLessThan(
      prompt.indexOf("### 4. 闭合与搬运"),
    );
    expect(runPrompt("depth", [], {}).system).not.toContain(alignment);
  });
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
    expect(depthInstructions).toContain("### 1. 采图与分割");
    expect(depthInstructions).toContain("### 2. 三维定位");
    expect(visionInstructions).not.toContain("capture_segmentation");
    expect(visionInstructions).not.toContain("先粗后细");
    expect(visionInstructions).not.toContain("只有临近接触或精确对齐时才缩小");
    expect(visionInstructions).toContain(
      "所有阶段都采用能明显纠正偏差的较大幅度调整",
    );
    expect(visionInstructions).toContain("不因靠近目标就默认微调");
    expect(visionInstructions).toContain("幅度依据图像与实际 TCP");
    expect(visionInstructions).toContain("不把像素当米或臆造相机外参");
  });
  it("preserves approach progress across turns and avoids redundant camera switching", () => {
    expect(visionInstructions).toContain(
      "沿用同一会话已确认的方向、动作效果和进度",
    );
    expect(visionInstructions).toContain("建立方向关系，之后复用");
    expect(visionInstructions).toContain("必要时才切换相机");
    expect(visionInstructions).toContain("保持既有力度和速度");
    expect(visionInstructions).toContain("observe_role");
    for (const prompt of [visionInstructions, depthInstructions])
      expect(prompt).toContain("目标区释放 → 撤开夹爪 → 回到工作位");
  });
  it("authorizes task-scoped tools and visual annotation without repeated permission, while respecting read-only requests", () => {
    expect(depthInstructions).toContain("说明后直接执行，不等待再次批准");
    expect(depthInstructions).toContain("依据该帧补框");
    expect(depthInstructions).toContain("同一固定帧重建");
    expect(depthInstructions).toContain(
      "真实对象/区域 ID 和三维坐标，不编造坐标",
    );
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
