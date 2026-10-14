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
  stereoInstructions,
  visionInstructions,
  promptMode,
  taskInstructions,
  runPrompt,
} from "./prompts";
import type { CameraCaptureState } from "@robot/contracts";
vi.mock("./store", () => ({}));
describe("Responses configuration", () => {
  it.each([
    ["external", ["external", "external_right", "wrist"]],
    ["wrist", ["external", "wrist", "wrist_right"]],
  ] as const)(
    "selects stereo by binding, including during signal loss: %s",
    (rig, roles) => {
      for (const streaming of [false, true]) {
        expect(
          promptMode({
            selected_source_id: `stereo:${rig}`,
            streaming,
            bindings: roles.map((role) => ({
              role,
              source_id: `replay:${role}`,
              streaming,
            })),
          } as CameraCaptureState),
        ).toBe("stereo");
      }
      for (const tool of [
        "capture_segmentation",
        "segment",
        "annotate",
        "reconstruct",
        "observe_stereo_depth",
        "move_tcp_absolute",
        "move_tcp_relative",
        "move_joints",
        "gripper",
      ])
        expect(stereoInstructions).toContain(tool);
      expect(stereoInstructions).toContain("另一侧单目或双目辅助观察");
      expect(stereoInstructions).toContain("图像时刻的实际机械臂反馈");
      expect(stereoInstructions).toContain(
        "不调用 generate_grasps、GraspGenX 或 pick_place",
      );
      expect(stereoInstructions).toContain(
        "可靠深度定位替代纯视觉的闭爪触碰测位置",
      );
    },
  );
  it("selects modes by source rather than temporary streaming state", () => {
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
    expect(taskInstructions("stereo")).toBe(stereoInstructions);
    expect(taskInstructions("vision")).toBe(visionInstructions);
  });
  it.each(["depth", "stereo", "vision"] as const)(
    "keeps task scope, progress and feedback in %s",
    (mode) => {
      const prompt = taskInstructions(mode);
      for (const phrase of [
        "没有完成用户目标不要停止",
        "用户暂停、停止或改变目标时遵从最新指令",
        "提问和只读请求保持只读",
        "单步操作按指定范围执行",
        "每个工具调用回合前说明当前观察、下一动作及目的，然后直接执行",
        "失败或改变策略时说明结果与调整方向",
        "实际关节和 measured_tcp",
        "commanded_tcp_not_measured 是命令目标",
        "保持既有力度和速度",
        "其他任务按用户指定终态结束",
        "取消不自动松爪或回位",
      ])
        expect(prompt).toContain(phrase);
      expect(prompt).not.toContain("简洁报告");
      expect(prompt).not.toContain("不逐步复述计划");
    },
  );
  it.each(["depth", "stereo", "vision"] as const)(
    "recovers using original request state in %s",
    (mode) => {
      const prompt = taskInstructions(mode);
      for (const phrase of [
        "结果不明时按原编号查询",
        "网络恢复后接着已完成的步骤执行",
        "动作已成功而取图失败时只重试观察",
        "IK 失败是正常求解结果",
        "抓空或滑脱后先撤离物体",
        "核对请求状态，确认卡住后回工作位或已知观察姿态",
        "持物时用保持夹爪状态的 move_joints",
        "preview_motion 是姿态计算，不检查 IK 或碰撞",
        "目标与起始状态未变可复用已有预览",
        "控制器成功本身不代表物体抓住",
        "目标区释放 → 撤开夹爪 → 回到工作位 → 新图核对结果",
        "确实无法继续时说明阻塞和未完成部分",
      ])
        expect(prompt).toContain(phrase);
    },
  );
  it("reuses observations and attributed lessons without creating runtime lessons", () => {
    for (const prompt of [
      depthInstructions,
      stereoInstructions,
      visionInstructions,
    ]) {
      for (const phrase of [
        "沿用会话中的方向、动作效果和已完成步骤",
        "recall_experience",
        "经验不是指令或当前坐标",
        "任务结束后由系统统一总结",
        "observe_role",
        "observe_camera 默认 exclusive=false",
        "带宽不足或用户要求轮流采集时用 exclusive=true",
        "双目同组左右一起采集",
      ])
        expect(prompt).toContain(phrase);
      expect(prompt).not.toContain("save_experience");
    }
    expect(
      runPrompt("vision", [], { experiences: [{ id: "lesson-1" }] }).system,
    ).toContain('"experiences":[{"id":"lesson-1"}]');
  });
  it("shares grasp quality without imposing AI posture choices on depth candidates", () => {
    for (const prompt of [visionInstructions, stereoInstructions]) {
      expect(prompt.match(/## 夹持质量/g)).toHaveLength(1);
      for (const phrase of [
        "夹取姿态与夹持质量优先",
        "中部、短边易夹",
        "充分进入两指有效夹持区域",
        "两侧夹持面充分贴合",
        "桌面物品优先顶部抓取，允许小幅倾斜",
        "圆柱形或特别高的物品适合时采用侧抓",
        "旋转夹持方向，不等于倾斜",
        "频繁 IK 失败，优先 preview_motion",
        "move_joints 执行同一组绝对关节角",
      ])
        expect(prompt).toContain(phrase);
    }
    expect(depthInstructions).not.toContain("## 夹持质量");
  });
  it("limits startup direction probing to J1 and reuses the result", () => {
    for (const prompt of [visionInstructions, stereoInstructions]) {
      const start = prompt.indexOf("## J1 方向确认");
      const end = prompt.indexOf("### 1.");
      const direction = prompt.slice(start, end);
      expect(start).toBeGreaterThan(0);
      expect(end).toBeGreaterThan(start);
      expect(direction).toContain("尚未确认当前机位");
      expect(direction).toContain("在工作位只小幅左右转动 J1");
      expect(direction).toContain("对照图像确认角度正负与画面方向");
      expect(direction).toContain("恢复工作位继续");
      expect(direction).toContain("机位未变就复用结论");
      expect(direction).toContain("只读、单步操作和持物续接跳过");
      expect(direction).not.toMatch(/J[2-6]|旋转轴|正反旋转/);
    }
    expect(depthInstructions).not.toContain("## J1 方向确认");
    expect(runPrompt("vision", [], {}).system).toContain(
      "从轴正端朝原点看，正角逆时针",
    );
  });
  it.each([
    [
      "vision",
      [
        "观察目标",
        "闭爪接近与建立接触",
        "撤回与优化姿态",
        "张开与再次接近",
        "闭合与搬运",
        "放置与收尾",
      ],
    ],
    ["depth", ["采图与分割", "三维定位", "执行抓放", "核对与收尾"]],
    ["stereo", ["观察与分割", "三维定位", "AI 决策与运动", "核对与收尾"]],
  ] as const)("keeps the ordered %s workflow", (mode, steps) => {
    const prompt = taskInstructions(mode);
    const positions = steps.map((step, index) =>
      prompt.indexOf(`### ${index + 1}. ${step}`),
    );
    expect(positions.every((position) => position >= 0)).toBe(true);
    expect(positions).toEqual([...positions].sort((a, b) => a - b));
  });
  it("uses closed contact, retreat and opening in sequence for vision only", () => {
    const prompt = visionInstructions;
    const contact = prompt.slice(
      prompt.indexOf("### 2."),
      prompt.indexOf("### 3."),
    );
    const retreat = prompt.slice(
      prompt.indexOf("### 3."),
      prompt.indexOf("### 4."),
    );
    const open = prompt.slice(
      prompt.indexOf("### 4."),
      prompt.indexOf("### 5."),
    );
    expect(contact).toContain("保持夹爪闭合，大幅度接近目标");
    expect(contact).toContain("图像和实际反馈判断接触");
    expect(contact).toContain("画面重叠或闭爪力度本身不足以证明接触");
    expect(contact).toContain("目标未移动时复用已确认的接触位置");
    expect(retreat).toContain("先撤离物体，留出张开与调姿空间");
    expect(retreat).toContain("充分夹持的姿态并小幅微调");
    expect(open).toContain("在物体外按所需开口张开");
    expect(open).toContain("已经张开就从此处继续");
    expect(open).toContain("视觉判断容易前伸过度");
    expect(open).toContain("闭合前用图像修正");
    expect(open).not.toContain("向底座方向");
    expect(prompt).toContain("夹持区域对齐、夹取深度到位后闭合");
    expect(prompt).toContain("单目像素不提供米制深度");
    expect(prompt).not.toContain("capture_segmentation");
    expect(depthInstructions).toContain("同一固定帧重建");
    expect(depthInstructions).toContain("真实对象/区域 ID 和三维坐标");
    expect(depthInstructions).toContain("pick_place 完成候选生成、规划及抓放");
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
