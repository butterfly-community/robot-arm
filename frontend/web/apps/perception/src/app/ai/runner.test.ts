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
    "uses stereo localization and AI motion with a single opposite camera: %s",
    (rig, roles) => {
      for (const streaming of [false, true]) {
        const camera = {
          selected_source_id: `stereo:${rig}`,
          streaming,
          bindings: roles.map((role) => ({
            role,
            source_id: `replay:${role}`,
            streaming,
          })),
        } as CameraCaptureState;
        expect(promptMode(camera)).toBe("stereo");
      }
      const prompt = runPrompt("stereo", [], {}).system;
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
      ]) {
        expect(prompt).toContain(tool);
      }
      expect(prompt).toContain(
        "不调用 generate_grasps、GraspGenX 或 pick_place",
      );
      expect(prompt).toContain("不要求四路齐全");
      expect(prompt).toContain("不要求先闭爪触碰目标测位置");
      expect(prompt).toContain("图像时刻的实际机械臂反馈");
      expect(prompt).not.toContain("pick_place 包含抓取与放置");
      expect(prompt).not.toContain("在建立有效接触前不要打开夹爪");
    },
  );
  it("uses attributed cross-session lessons without treating them as commands or live coordinates", () => {
    for (const prompt of [
      depthInstructions,
      stereoInstructions,
      visionInstructions,
    ]) {
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
    for (const prompt of [
      depthInstructions,
      stereoInstructions,
      visionInstructions,
    ]) {
      expect(prompt).toContain("没有完成用户目标不要停止");
      expect(prompt).toContain("用户明确暂停、停止或改变目标时遵从最新指令");
      expect(prompt).toContain("IK 失败是尝试候选目标位姿时的正常求解结果");
      expect(prompt).toContain("每个工具调用回合前输出面向用户的过程说明");
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
    expect(depthInstructions).toContain("补做尚未完成的收尾");
    expect(visionInstructions).toContain(
      "用户只要求抓住并保持时不搬运、不释放",
    );
  });
  it("prioritizes joint adjustment near a vision target without replacing the depth pipeline", () => {
    expect(visionInstructions).toContain(
      "已经接近目标但频繁 IK 失败时，优先直接调整关节角度",
    );
    expect(visionInstructions).toContain(
      "先 preview_motion，再用 move_joints 执行预览的同一组绝对关节角",
    );
    expect(visionInstructions).toContain(
      "物品放置在桌面时，优先从顶部向下抓取",
    );
    expect(visionInstructions).toContain("接近方向大致垂直桌面");
    expect(visionInstructions).toContain("尽量避免平行桌面的侧向抓取");
    expect(visionInstructions).toContain("这是桌面场景的偏好");
    expect(visionInstructions).toContain(
      "仅圆柱形或特别高、侧面适合稳定夹持的物品才考虑平行桌面的侧向抓取",
    );
    expect(visionInstructions).toContain("不因一次顶部 IK 失败就改为侧抓");
    expect(visionInstructions).not.toContain("顶部不适合时再选择");
    expect(visionInstructions).toContain(
      "大幅倾斜只在物体形状或摆放特别适合、能明显改善夹持时采用",
    );
    expect(visionInstructions).toContain(
      "顶部抓取允许小幅倾斜，不要求精确竖直",
    );
    expect(visionInstructions).not.toContain("可以平行地面、垂直地面");
    expect(visionInstructions).toContain("不为消除小幅倾斜反复调姿");
    expect(visionInstructions).not.toContain("既不水平也不竖直");
    expect(visionInstructions).not.toContain("不选择倾斜抓取");
    expect(visionInstructions).toContain(
      "保持接近方向、为对齐物体而旋转夹持方向不属于倾斜，不限制这种旋转",
    );
    expect(visionInstructions).toContain("不以图像中的斜向外观判断");
    expect(visionInstructions).toContain(
      "桌面物品优先选择短边、中部容易夹持的位置",
    );
    expect(visionInstructions).toContain("不代表整个任务或朝向不可行");
    expect(depthInstructions).not.toContain("不要只预览不推进");
    expect(depthInstructions).toContain("无需另行逐个尝试抓取姿态或 IK");
  });
  it("establishes closed-gripper contact then withdraws before opening and reapproaching", () => {
    const steps = [
      "### 1. 观察目标",
      "### 2. 闭爪接近与建立接触",
      "### 3. 撤回与优化姿态",
      "### 4. 张开与再次接近",
      "### 5. 闭合与搬运",
      "### 6. 放置与收尾",
    ];
    const prompt = runPrompt("vision", [], {}).system;
    const positions = steps.map((step) => prompt.indexOf(step));
    expect(positions.every((position) => position >= 0)).toBe(true);
    expect(positions).toEqual([...positions].sort((a, b) => a - b));
    const contact = prompt.slice(positions[1], positions[2]);
    const retreat = prompt.slice(positions[2], positions[3]);
    const open = prompt.slice(positions[3], positions[4]);
    expect(contact).toContain("保持夹爪闭合，大幅度接近目标");
    expect(contact).toContain("在建立有效接触前不要打开夹爪");
    expect(contact).toContain("撤回后依据接触位置优化夹持姿态");
    expect(retreat).toContain("先撤回离开物体、留出张开和调姿空间");
    expect(retreat).toContain("不要贴着物体张开、旋转或横移");
    expect(retreat).toContain("不固定为朝向底座，也不设固定距离");
    expect(open).toContain("撤回完成后按物品所需开口张开");
    expect(open).toContain("再依据已建立的接触位置重新接近");
    expect(open).toContain("不边贴近物体边张开");
    expect(prompt).not.toContain("不要等指尖接触物体才张开");
    expect(prompt).not.toContain("到达抓取前位置后按物品所需开口张开");
    expect(runPrompt("depth", [], {}).system).not.toContain(steps[2]);
    expect(depthInstructions).toContain("pick_place 包含抓取与放置");
  });
  it("requires contact evidence and reuses it without mistaking it for a completed grasp", () => {
    const prompt = runPrompt("vision", [], {}).system;
    expect(prompt).toContain("有效接触是实际碰到目标并据此确定相对位置");
    expect(prompt).toContain("也不等于已经抓住");
    expect(prompt).toContain("不能单凭闭爪力度或画面重叠宣称接触");
    expect(prompt).toContain("已有本轮有效接触记录且目标未移动时复用");
    expect(prompt).toContain("目标移动后更新接触位置");
    expect(prompt).toContain("已经完成撤回和张开时直接继续，不重新闭爪触碰");
    expect(prompt).toContain("姿态选择以充分夹取深度和稳定夹持为目标");
    expect(prompt).toContain("优先夹住物体中部");
    expect(prompt).toContain("避免仅用指尖或边缘浅夹、只夹角点而滑脱");
  });
  it("withdraws after a missed grasp before changing posture in either mode", () => {
    for (const mode of ["depth", "stereo", "vision"] as const) {
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
    expect(visionInstructions).toContain("抓空或滑脱进入异常处理");
    expect(visionInstructions.match(/先撤离目标/g)).toHaveLength(1);
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
      prompt.indexOf("### 5. 闭合与搬运"),
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
    expect(taskInstructions("stereo")).toBe(stereoInstructions);
    const qualityPriority = "抓取决策最高优先级是夹取姿态与夹取质量";
    expect(visionInstructions).toContain(qualityPriority);
    expect(visionInstructions.indexOf(qualityPriority)).toBeLessThan(
      visionInstructions.indexOf("执行顺序是"),
    );
    expect(visionInstructions).toContain(
      "不为速度、减少动作次数或沿用当前姿态而牺牲夹持质量",
    );
    expect(visionInstructions).toContain(
      "先接触是确定相对位置的步骤，不是降低夹持质量要求",
    );
    expect(visionInstructions).toContain(
      "发现浅夹或边缘夹持时，继续调整后再闭合",
    );
    expect(visionInstructions).not.toContain("不要先花多轮追求最佳姿态");
    expect(visionInstructions).toContain("夹持区域对齐、夹取深度到位后闭合");
    expect(depthInstructions).toContain("### 1. 采图与分割");
    expect(depthInstructions).toContain("### 2. 三维定位");
    expect(visionInstructions).not.toContain("capture_segmentation");
    expect(visionInstructions.indexOf("大幅度接近目标")).toBeLessThan(
      visionInstructions.indexOf("依据接触位置小幅度微调夹爪姿态与位置"),
    );
    expect(visionInstructions).toContain(
      "闭合后两侧夹持面充分贴合、增大有效接触面积",
    );
    expect(visionInstructions).toContain(
      "充分夹取深度指物体进入两指之间的有效夹持区域",
    );
    expect(visionInstructions).toContain("不是让夹爪尽量靠近地面");
    expect(visionInstructions).toContain("避免仅用指尖或边缘浅夹");
    expect(visionInstructions).not.toContain("不固定朝向、角度或步长");
    expect(visionInstructions).not.toContain("## 纯视觉调整策略");
    expect(visionInstructions.match(/增大有效接触面积/g)).toHaveLength(1);
    expect(visionInstructions).not.toContain(
      "所有阶段都采用能明显纠正偏差的较大幅度调整",
    );
    expect(visionInstructions).not.toContain("不因靠近目标就默认微调");
    expect(visionInstructions).not.toContain("方向正确但幅度不足就增大调整");
    expect(visionInstructions).toContain("幅度依据图像与实际 TCP");
    expect(visionInstructions).toContain("不把像素当米或臆造相机外参");
  });
  it("preserves approach progress across turns and avoids redundant camera switching", () => {
    expect(visionInstructions).toContain(
      "沿用同一会话已确认的方向、动作效果和进度",
    );
    expect(visionInstructions).toContain("建立方向关系，之后复用");
    for (const prompt of [
      visionInstructions,
      depthInstructions,
      stereoInstructions,
    ]) {
      expect(prompt).toContain("取图默认使用 observe_camera(exclusive=false)");
      expect(prompt).toContain(
        "仅在确认带宽不足或用户要求轮流采集时使用 exclusive=true",
      );
      expect(prompt).toContain("共享 USB 不等于带宽不足");
    }
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
