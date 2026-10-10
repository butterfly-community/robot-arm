import { createHash } from "node:crypto";
import { z } from "zod";
import { db } from "./store";
import {
  experienceInputSchema,
  type AIExperience,
  type AICall,
  type AIRun,
  type ExperienceScope,
} from "./types";

// Same persistent Redis as conversations; no TTL, files, or extra model calls.
const key = "robot-arm:ai:experiences";
// Compare-and-set protects a human deletion/correction from an in-flight AI
// update. Do not recreate a value read before the user removed it.
async function persist(
  id: string,
  previous: string | null | undefined,
  value: AIExperience,
) {
  const changed = await (
    await db()
  ).eval(
    `local current = redis.call('HGET', KEYS[1], ARGV[1])
     if (current or '') ~= ARGV[2] then return 0 end
     redis.call('HSET', KEYS[1], ARGV[1], ARGV[3])
     return 1`,
    { keys: [key], arguments: [id, previous ?? "", JSON.stringify(value)] },
  );
  if (changed !== 1)
    throw Error("经验已被其他操作更改或删除，请重新读取后再决定是否更新");
}
const sameScope = (a: ExperienceScope, b: ExperienceScope) =>
  a.modelRevision === b.modelRevision &&
  a.mode === b.mode &&
  a.feedbackSource === b.feedbackSource;

function identity(scope: ExperienceScope, topic: string) {
  return createHash("sha256")
    .update(
      JSON.stringify([
        scope.modelRevision,
        scope.mode,
        scope.feedbackSource,
        topic,
      ]),
    )
    .digest("hex");
}

export async function listExperiences(scope?: ExperienceScope, query = "") {
  const values = await (await db()).hVals(key);
  const needle = query.trim().toLocaleLowerCase();
  return values
    .map((value) => JSON.parse(value) as AIExperience)
    .filter((value) => !scope || sameScope(value.scope, scope))
    .filter(
      (value) =>
        !needle ||
        JSON.stringify([
          value.title,
          value.conditions,
          value.reflection,
          value.failure?.error,
        ])
          .toLocaleLowerCase()
          .includes(needle),
    )
    .sort((a, b) => b.updatedAt - a.updatedAt);
}

export function experienceIndex(values: AIExperience[]) {
  return values.map(({ id, title, conditions, reflection }) => ({
    id,
    title,
    conditions,
    assessment: reflection?.assessment ?? "failure_fact",
  }));
}

export async function readExperience(id: string, scope: ExperienceScope) {
  const raw = await (await db()).hGet(key, id);
  if (!raw) throw Error("经验不存在或已删除，请重新读取经验目录");
  const value = JSON.parse(raw) as AIExperience;
  if (!sameScope(value.scope, scope))
    throw Error("经验不属于当前机械臂、模式与反馈来源");
  return value;
}

export async function deleteExperience(id: string) {
  return { deleted: (await (await db()).hDel(key, id)) > 0 };
}

function scopeOf(run: AIRun) {
  if (!run.experienceScope) throw Error("当前任务没有机械臂经验范围");
  return run.experienceScope;
}

function sources(
  previous: AIExperience | undefined,
  run: AIRun,
  callIds: string[],
) {
  const entries = previous?.sources ?? [];
  const current = entries.find((entry) => entry.runId === run.id);
  return [
    ...entries.filter((entry) => entry.runId !== run.id),
    {
      runId: run.id,
      sessionId: run.sessionId,
      callIds: [...new Set([...(current?.callIds ?? []), ...callIds])],
    },
  ];
}

export async function recordFailure(run: AIRun, call: AICall) {
  // Never turn cancellation or a memory-tool error into a robot lesson.
  if (
    !run.experienceScope ||
    !call.error ||
    !run.requests.some(
      (request) =>
        request.id.startsWith(`${run.id}:${call.id}:`) &&
        request.state === "failed",
    )
  )
    return undefined;
  const scope = run.experienceScope;
  const id = identity(scope, `failure:${call.name}:${call.error}`);
  const redis = await db();
  const raw = await redis.hGet(key, id);
  const previous: AIExperience | undefined = raw ? JSON.parse(raw) : undefined;
  const now = Date.now();
  const value: AIExperience = {
    id,
    scope,
    title: previous?.title ?? `${call.name}：${call.error}`,
    conditions:
      previous?.conditions ??
      "仅证明所记录参数的这次请求失败，不代表该方向或整个任务不可行。",
    failure: {
      tool: call.name,
      input: call.input,
      error: call.error,
      runId: run.id,
      callId: call.id,
    },
    reflection: previous?.reflection,
    sources: sources(previous, run, [call.id]),
    createdAt: previous?.createdAt ?? now,
    updatedAt: now,
  };
  await persist(id, raw, value);
  return {
    experience_id: id,
    note: "失败事实已保存；原因及改进效果尚需结合观测判断，可用 save_experience 补充。",
  };
}

export async function saveExperience(
  run: AIRun,
  input: z.infer<typeof experienceInputSchema>,
) {
  const scope = scopeOf(run);
  // References must resolve; successful controller calls are not grasp proof.
  for (const id of input.evidenceCallIds) {
    if (!run.calls.some((call) => call.id === id && call.endedAt != null))
      throw Error(`经验引用了本次任务中不存在或未完成的工具调用：${id}`);
  }
  const redis = await db();
  const id =
    input.id ??
    identity(
      scope,
      `lesson:${input.title.normalize("NFKC").toLocaleLowerCase()}`,
    );
  const raw = await redis.hGet(key, id);
  if (input.id && !raw) throw Error("要更新的经验已删除，请重新读取目录");
  const previous: AIExperience | undefined = raw ? JSON.parse(raw) : undefined;
  if (previous && !sameScope(previous.scope, scope))
    throw Error("不能覆盖其他机械臂或模式的经验");
  const now = Date.now();
  const value: AIExperience = {
    id,
    scope,
    title: input.title,
    conditions: input.conditions,
    failure: previous?.failure,
    reflection: {
      lesson: input.lesson,
      nextAction: input.nextAction,
      assessment: input.assessment,
      model: run.settings.model,
    },
    sources: sources(previous, run, input.evidenceCallIds),
    createdAt: previous?.createdAt ?? now,
    updatedAt: now,
  };
  await persist(id, raw, value);
  return value;
}
