import { createOpenAI } from "@ai-sdk/openai";
import {
  streamText,
  tool,
  isLoopFinished,
  isStepCount,
  TypeValidationError,
  type ModelMessage,
} from "ai";
import { z } from "zod";
import { robotTools } from "./tools";
import {
  cancelRobot,
  cancellable,
  RobotToolsContext,
  perceptionSnapshot,
  compactCamera,
  readRobot,
} from "./robot";
import { runPrompt, promptMode } from "./prompts";
import { modelContext, requestSummary } from "./context";
import {
  getRun,
  imagePart,
  messages,
  register,
  release,
  runtime,
  saveCheck,
  saveMessages,
  saveRun,
  settings,
} from "./store";
import {
  activeRun,
  errorText,
  type AIRun,
  type AISettings,
  type CapabilityCheck,
  type startSchema,
} from "./types";

export function appendReplyText(
  previous: string,
  delta: string,
  newReply: boolean,
) {
  if (!delta) return previous;
  const separator =
    newReply && previous && !previous.endsWith("\n\n")
      ? previous.endsWith("\n")
        ? "\n"
        : "\n\n"
      : "";
  return previous + separator + delta;
}

export function publicError(error: unknown) {
  let message = errorText(error);
  if (TypeValidationError.isInstance(error)) {
    // The SDK still owns SSE decoding. Some compatible providers return a
    // failed Response without the normal stream fields; show its actual error
    // rather than the schema union's entire list of rejected event shapes.
    const failed = z
      .object({
        type: z.literal("response.failed"),
        response: z.object({
          id: z.string(),
          error: z.object({ code: z.string(), message: z.string() }),
        }),
      })
      .safeParse(error.value);
    if (failed.success) {
      const { id, error: cause } = failed.data.response;
      message = `模型服务 ${cause.code}: ${cause.message}（${id}）`;
    }
  }
  const key = process.env.AI_API_KEY;
  return key ? message.split(key).join("[redacted]") : message;
}
export function responseEffort(value: unknown): string | undefined {
  const event = value as { response?: { reasoning?: { effort?: unknown } } };
  const effort = event?.response?.reasoning?.effort;
  return typeof effort === "string" ? effort : undefined;
}
function provider(config: AISettings) {
  const apiKey = process.env.AI_API_KEY;
  if (!apiKey) throw Error("服务端未配置 AI_API_KEY");
  return createOpenAI({ baseURL: config.baseURL, apiKey });
}
export function providerOptions(config: AISettings) {
  return {
    openai: {
      store: false,
      parallelToolCalls: false,
      include: ["reasoning.encrypted_content"],
      ...(config.effort ? { reasoningEffort: config.effort } : {}),
    },
  };
}
export async function createRun(input: z.infer<typeof startSchema>) {
  const previous = await getRun(input.runId);
  if (previous) return { run: previous, created: false };
  const run: AIRun = {
    id: input.runId,
    sessionId: input.sessionId,
    owner: runtime().owner,
    settings: await settings(),
    state: "running",
    input: input.text,
    images: input.images,
    text: "",
    startedAt: Date.now(),
    updatedAt: Date.now(),
    calls: [],
    requests: [],
    responseModels: [],
    responseIds: [],
    warnings: [],
  };
  const created = await register(run);
  if (created) runtime().controllers.set(run.id, new AbortController());
  return { run, created };
}
export async function executeRun(run: AIRun) {
  const controller = runtime().controllers.get(run.id)!;
  try {
    const history = await messages(run.sessionId);
    const input: ModelMessage = {
      role: "user",
      content: [
        { type: "text", text: run.input },
        ...(await Promise.all(run.images.map(imagePart))),
      ],
    };
    const conversation: ModelMessage[] = [...history, input];
    await saveMessages(run.sessionId, conversation);
    if (controller.signal.aborted) throw Error("AI 已停止");
    const [snapshot, robot] = await Promise.all([
      perceptionSnapshot(),
      readRobot(),
    ]);
    const camera = compactCamera(snapshot.values.camera_state);
    run.promptMode = promptMode(snapshot.values.camera_state);
    await saveRun(run);
    let newReply = true;
    const result = streamText({
      model: provider(run.settings).responses(run.settings.model),
      providerOptions: providerOptions(run.settings),
      include: { rawChunks: true },
      ...runPrompt(run.promptMode, conversation, { robot, camera }),
      prepareStep: ({ messages }) => ({ messages: modelContext(messages) }),
      tools: robotTools(new RobotToolsContext(run, controller.signal)),
      stopWhen: isLoopFinished(),
      abortSignal: controller.signal,
      onChunk: async ({ chunk }) => {
        if (chunk.type === "text-delta" && chunk.text) {
          run.firstTokenAt ??= Date.now();
          run.text = appendReplyText(run.text, chunk.text, newReply);
          newReply = false;
          await saveRun(run);
        }
      },
      onStepEnd: async (step) => {
        // A completed model turn, not an individual streaming token, is a paragraph boundary.
        newReply = true;
        (run.steps ??= []).push({
          responseId: step.response.id,
          responseMs: step.performance.responseTimeMs,
          stepMs: step.performance.stepTimeMs,
          inputTokens: step.usage.inputTokens,
          outputTokens: step.usage.outputTokens,
        });
        conversation.push(...step.response.messages);
        await saveMessages(run.sessionId, conversation);
        if (step.response.id) run.responseIds.push(step.response.id);
        if (step.response.modelId)
          run.responseModels.push(step.response.modelId);
        run.warnings.push(
          ...(step.warnings ?? []).map((warning) => JSON.stringify(warning)),
        );
        await saveRun(run);
      },
    });
    // SDK consumes and decodes SSE; the browser receives persisted progress.
    for await (const part of result.fullStream) {
      if (part.type === "error") throw part.error;
      if (part.type === "raw") {
        const effort = responseEffort(part.rawValue);
        if (effort)
          run.reportedEfforts = [
            ...new Set([...(run.reportedEfforts ?? []), effort]),
          ];
      }
    }
    run.usage = await result.totalUsage;
    const finish = await result.finishReason;
    if (controller.signal.aborted) run.state = "cancelled";
    else if (finish !== "stop") throw Error(`模型未正常完成：${finish}`);
    else run.state = "succeeded";
  } catch (error) {
    run.state = controller.signal.aborted ? "cancelled" : "failed";
    run.error = publicError(error);
  } finally {
    // Aborting model generation is not proof a previously submitted robot
    // action stopped. Finish tracking its original ID before a terminal AI state.
    const active = run.requests.filter((r) =>
      ["accepted", "planning", "executing", "idle"].includes(r.state),
    );
    if (active.length) {
      const original = run.state;
      run.state = controller.signal.aborted ? "stopping" : "running";
      await saveRun(run);
      for (const request of active) {
        try {
          if (controller.signal.aborted && cancellable(request.path))
            await cancelRobot(request.id);
          const context = new RobotToolsContext(
            run,
            new AbortController().signal,
          );
          // Query-only: registered request means command() cannot submit again.
          const tail = request.id.slice(run.id.length + 1);
          const colon = tail.lastIndexOf(":");
          await context.command(
            tail.slice(0, colon),
            Number(tail.slice(colon + 1)),
            request.path,
            {},
            request.label,
          );
        } catch (error) {
          run.error = publicError(error);
        }
      }
      run.state = original;
    }
    run.endedAt = Date.now();
    if (run.state !== "succeeded") {
      const history = await messages(run.sessionId);
      history.push({
        role: "assistant",
        content: `运行已结束：${run.state}。${run.error ?? ""}。原机器人请求：${JSON.stringify(requestSummary(run.requests))}。不得自动重放此前指令，后续只处理用户的新请求。`,
      });
      await saveMessages(run.sessionId, history);
    }
    await saveRun(run);
    runtime().controllers.delete(run.id);
    await release(run);
  }
}
export async function stopRun(id: string) {
  const run = await getRun(id);
  if (!run) throw Error("AI 运行不存在");
  if (!activeRun(run)) return run;
  run.state = "stopping";
  await saveRun(run);
  runtime().controllers.get(id)?.abort();
  // RobotToolsContext continues to await terminal results, including after abort.
  for (const request of run.requests.filter(
    (r) =>
      cancellable(r.path) &&
      ["accepted", "planning", "executing"].includes(r.state),
  )) {
    try {
      await cancelRobot(request.id);
    } catch (error) {
      run.error = publicError(error);
    }
  }
  return run;
}
export async function listModels(config: AISettings) {
  const key = process.env.AI_API_KEY;
  if (!key) throw Error("服务端未配置 AI_API_KEY");
  const response = await fetch(`${config.baseURL}/models`, {
    headers: { Authorization: `Bearer ${key}` },
    cache: "no-store",
  });
  const value = await response.json();
  if (!response.ok)
    throw Error(value.error?.message ?? `模型列表 HTTP ${response.status}`);
  return (value.data as { id: string; name?: string }[]).map((m) => ({
    id: m.id,
    label: m.name ?? m.id,
  }));
}
export async function checkCapabilities(config: AISettings, imageId?: string) {
  const check: CapabilityCheck = {
    ...config,
    checkedAt: Date.now(),
    text: false,
    vision: false,
    tools: false,
    streaming: false,
  };
  try {
    const result = streamText({
      model: provider(config).responses(config.model),
      providerOptions: providerOptions(config),
      include: { rawChunks: true },
      messages: [
        {
          role: "user",
          content: [
            {
              type: "text",
              text: imageId
                ? "请用 describe_input 描述提供的图片，然后用中文复述工具返回的描述。"
                : "请用 describe_input 传入‘连接正常’，然后用中文复述工具返回值。",
            },
            ...(imageId ? [await imagePart(imageId)] : []),
          ],
        },
      ],
      tools: {
        describe_input: tool({
          description: "只返回输入描述，不操作设备",
          inputSchema: z.object({ description: z.string() }),
          execute: async (input) => {
            check.tools = true;
            return input;
          },
        }),
      },
      stopWhen: isStepCount(2),
      prepareStep: ({ stepNumber }) => ({
        toolChoice:
          stepNumber === 0
            ? { type: "tool", toolName: "describe_input" }
            : "none",
      }),
    });
    let text = "";
    for await (const part of result.fullStream) {
      if (part.type === "error") throw part.error;
      if (part.type === "raw")
        check.reportedEffort =
          responseEffort(part.rawValue) ?? check.reportedEffort;
      if (part.type === "text-delta") {
        text += part.text;
        check.streaming = true;
      }
    }
    check.text = text.length > 0;
    check.vision = Boolean(imageId) && check.text;
    check.actualModel = (await result.response).modelId;
    check.effortNote = !config.effort
      ? "未指定思考强度，使用模型默认"
      : !check.reportedEffort
        ? "接口未回显思考强度；请求被接受不代表档位生效"
        : check.reportedEffort !== config.effort
          ? `请求 ${config.effort}，服务回显 ${check.reportedEffort}，不一致`
          : `服务回显思考强度：${check.reportedEffort}`;
    if (!check.tools || !check.text)
      throw Error("未完成工具调用和结果回传，不能确认兼容");
    await saveCheck(check);
    return { ...check, description: text };
  } catch (error) {
    check.error = publicError(error);
    await saveCheck(check);
    throw Error(check.error);
  }
}
