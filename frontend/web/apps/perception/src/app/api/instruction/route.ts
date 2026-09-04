import { createOpenAICompatible } from "@ai-sdk/openai-compatible";
import type { WorldScene } from "@robot/contracts";
import { Output, generateText } from "ai";
import { NextResponse } from "next/server";

import {
  compactScene,
  executeInstruction,
  instructionSchema,
  perceptionPlanSchema,
  sceneSelectionSchema,
  type InstructionPlanner,
  type RobotGateway,
} from "../../instruction-flow";

export const dynamic = "force-dynamic";

function requiredEnvironment(name: string) {
  const value = process.env[name]?.trim();
  if (!value) throw new Error(`服务端缺少环境变量 ${name}`);
  return value;
}

function planner(): InstructionPlanner {
  const baseURL = requiredEnvironment("AI_API_BASE_URL");
  const apiKey = requiredEnvironment("AI_API_KEY");
  const modelId = requiredEnvironment("AI_MODEL");
  const authHeader = process.env.AI_API_AUTH_HEADER?.trim() || "Authorization";
  const bearer = authHeader.toLowerCase() === "authorization";
  const provider = createOpenAICompatible({
    name: "configured-openai-compatible",
    baseURL,
    apiKey: bearer ? apiKey : undefined,
    headers: bearer ? undefined : { [authHeader]: apiKey },
    supportsStructuredOutputs: true,
  });
  const model = provider.chatModel(modelId);
  return {
    async plan(instruction) {
      const { output } = await generateText({
        model,
        output: Output.object({ schema: perceptionPlanSchema }),
        system:
          "你是机器人视觉任务解析器。只支持从当前场景抓取一个对象并放入或放到一个区域。不要输出实例 ID、坐标、姿态或运动参数。为用户提到的每个实体生成 2 到 4 个简短英文视觉提示词：从保留颜色或形状的具体描述，扩展到常见开放词汇检测器使用的实体类别和同义词；不要只给用途名称，也不要添加用户没有提到的实体。perception_prompts 必须完整覆盖抓取物体和目标区域并去重；placement_labels 必须逐字取自 perception_prompts，并包含为放置目标生成的全部提示词。可执行时 action 为 pick_place、reason 为空字符串；其他任务 action 为 unsupported 并用中文说明原因。",
        prompt: instruction,
      });
      return output;
    },
    async select(instruction, scene) {
      const { output } = await generateText({
        model,
        output: Output.object({ schema: sceneSelectionSchema }),
        system:
          "你是机器人结构化场景选择器。object_id 和 placement_region_id 必须逐字选自输入场景；抓取对象必须有抓取候选。不要生成坐标、姿态、路径或不存在的 ID。",
        prompt: `用户指令：${instruction}\n当前结构化场景：${JSON.stringify(compactScene(scene))}`,
      });
      return output;
    },
  };
}

function gateway(): RobotGateway {
  const base =
    process.env.GATEWAY_INTERNAL_URL?.replace(/\/$/, "") ||
    "http://web-gateway:8080";
  const request = async (path: string, init?: RequestInit) => {
    const response = await fetch(`${base}${path}`, {
      ...init,
      cache: "no-store",
    });
    const value = (await response.json()) as Record<string, unknown>;
    if (!response.ok || typeof value.original_error === "string") {
      throw new Error(
        typeof value.original_error === "string"
          ? value.original_error
          : `机器人服务请求失败 (${response.status})`,
      );
    }
    return value;
  };
  return {
    post(path, body) {
      return request(path, {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify(body),
      });
    },
    async scene() {
      const snapshot = (await request("/api/perception/state")) as {
        values?: { world_scene?: WorldScene };
      };
      const scene = snapshot.values?.world_scene;
      if (!scene) throw new Error("感知完成后没有返回结构化场景");
      return scene;
    },
  };
}

export async function POST(request: Request) {
  try {
    const { instruction } = instructionSchema.parse(await request.json());
    const result = await executeInstruction(instruction, planner(), gateway());
    return NextResponse.json(result, { status: 202 });
  } catch (error) {
    const originalError =
      error instanceof Error ? error.message : String(error);
    return NextResponse.json(
      { original_error: originalError },
      { status: 400 },
    );
  }
}
