import { createClient } from "redis";
import { createHash, randomUUID } from "node:crypto";
import { mkdir, readFile, writeFile, rename, rm } from "node:fs/promises";
import { join } from "node:path";
import sharp from "sharp";
import type { ModelMessage } from "ai";
import { modelContext } from "./context";
import {
  activeRun,
  type AIImage,
  type AIRun,
  type AIUpdate,
  type AISettings,
  type CapabilityCheck,
} from "./types";

const prefix = "robot-arm:ai:";
const imagePrefix = "robot-image:";
const makeClient = () =>
  createClient({ url: process.env.REDIS_URL ?? "redis://redis:6379" });
type Runtime = {
  owner: string;
  redis: ReturnType<typeof makeClient>;
  connected: Promise<unknown>;
  controllers: Map<string, AbortController>;
};
const globalRuntime = globalThis as typeof globalThis & { robotAI?: Runtime };
export function runtime(): Runtime {
  if (!globalRuntime.robotAI) {
    const redis = makeClient();
    redis.on("error", () => console.error("AI Redis 连接异常，等待原连接恢复"));
    globalRuntime.robotAI = {
      owner: randomUUID(),
      redis,
      connected: redis.connect(),
      controllers: new Map(),
    };
  }
  return globalRuntime.robotAI!;
}
export async function db() {
  const r = runtime();
  await r.connected;
  return r.redis;
}
export async function readJSON<T>(key: string): Promise<T | undefined> {
  const raw = await (await db()).get(prefix + key);
  return raw == null ? undefined : (JSON.parse(raw) as T);
}
export async function writeJSON(key: string, value: unknown) {
  await (await db()).set(prefix + key, JSON.stringify(value));
}
export async function settings(): Promise<AISettings> {
  return (
    (await readJSON<AISettings>("settings")) ?? {
      baseURL: process.env.AI_API_BASE_URL ?? "https://new.fastaicode.top/v1",
      model: process.env.AI_MODEL ?? "gpt-5.6-sol",
      effort: process.env.AI_REASONING_EFFORT ?? "",
    }
  );
}
export async function saveRun(run: AIRun) {
  if (activeRun(run) && runtime().controllers.get(run.id)?.signal.aborted)
    run.state = "stopping";
  run.updatedAt = Date.now();
  await writeJSON(`run:${run.id}`, run);
}
export async function getRun(id: string) {
  const run = await readJSON<AIRun>(`run:${id}`);
  if (
    run &&
    activeRun(run) &&
    (run.owner !== runtime().owner || !runtime().controllers.has(run.id))
  ) {
    run.state = "interrupted";
    run.error =
      "AI 任务执行进程已结束；未确认的机器人动作需查询原请求，不自动续跑或重放。";
    run.acceptingInput = false;
    delete run.networkRetry;
    run.endedAt = Date.now();
    await saveRun(run);
  }
  if (run) run.updates = await runUpdates(run.id);
  return run;
}
export async function runUpdates(id: string): Promise<AIUpdate[]> {
  const redis = await db();
  const [items, included] = await Promise.all([
    redis.lRange(prefix + `run:${id}:updates`, 0, -1),
    redis.hGetAll(prefix + `run:${id}:included`),
  ]);
  return items.map((raw) => {
    const item = JSON.parse(raw) as AIUpdate;
    if (included[item.id]) item.includedAt = Number(included[item.id]);
    return item;
  });
}

export async function addRunUpdate(
  runId: string,
  sessionId: string,
  update: AIUpdate,
) {
  const result = await (
    await db()
  ).eval(
    `
    local updates = redis.call('LRANGE', KEYS[2], 0, -1)
    local raw = redis.call('GET', KEYS[1])
    if not raw then return 0 end
    local run = cjson.decode(raw)
    if run.sessionId ~= ARGV[3] then return 0 end
    for _, rawItem in ipairs(updates) do if cjson.decode(rawItem).id == ARGV[2] then return 1 end end
    if run.state ~= 'running' or run.acceptingInput == false or redis.call('EXISTS', KEYS[3]) == 1
      or (run.experienceSummary and run.experienceSummary.state == 'running') then return 0 end
    redis.call('RPUSH', KEYS[2], ARGV[1])
    return 1
  `,
    {
      keys: [
        prefix + `run:${runId}`,
        prefix + `run:${runId}:updates`,
        prefix + `run:${runId}:input-closed`,
      ],
      arguments: [JSON.stringify(update), update.id, sessionId],
    },
  );
  if (result !== 1)
    throw Error("任务已结束或正在收尾，请保留消息并在结束后发送");
}

export async function markUpdatesIncluded(runId: string, ids: string[]) {
  if (ids.length)
    await (
      await db()
    ).hSet(
      prefix + `run:${runId}:included`,
      Object.fromEntries(ids.map((id) => [id, String(Date.now())])),
    );
}

// Atomically close the inbox only if every accepted update reached a step.
export async function closeRunInput(runId: string): Promise<boolean> {
  return (
    (await (
      await db()
    ).eval(
      `
    local updates = redis.call('LRANGE', KEYS[2], 0, -1)
    for _, rawItem in ipairs(updates) do
      if redis.call('HEXISTS', KEYS[4], cjson.decode(rawItem).id) == 0 then return 0 end
    end
    -- Do not re-encode the run: Lua cjson changes empty arrays into objects.
    redis.call('SET', KEYS[3], '1')
    return 1
  `,
      {
        keys: [
          prefix + `run:${runId}`,
          prefix + `run:${runId}:updates`,
          prefix + `run:${runId}:input-closed`,
          prefix + `run:${runId}:included`,
        ],
        arguments: [],
      },
    )) === 1
  );
}
export async function runs(sessionId: string) {
  const ids = await (
    await db()
  ).lRange(prefix + `session:${sessionId}:runs`, 0, -1);
  return (await Promise.all(ids.map(getRun))).filter((v): v is AIRun =>
    Boolean(v),
  );
}
export async function sessions() {
  const ids = await (
    await db()
  ).zRange(prefix + "sessions", 0, -1, { REV: true });
  return Promise.all(
    ids.map(async (id) => {
      const records = await runs(id);
      return {
        id,
        title: records[0]?.input ?? id,
        updatedAt: records.at(-1)?.updatedAt ?? 0,
      };
    }),
  );
}
export async function register(run: AIRun): Promise<boolean> {
  const redis = await db();
  // One physical task owner, using the existing service rather than a new worker.
  // A new process may replace an interrupted owner, but never replays its work.
  const result = await redis.eval(
    `
    local old = redis.call('GET', KEYS[1])
    if old then return 0 end
    local active = redis.call('GET', KEYS[2])
    if active then
      local a = cjson.decode(active)
      local raw = redis.call('GET', ARGV[6] .. a.id)
      if a.owner == ARGV[2] and raw then
        local previous = cjson.decode(raw)
        if previous.state == 'running' or previous.state == 'stopping' then return -1 end
      end
    end
    redis.call('SET', KEYS[1], ARGV[1])
    redis.call('SET', KEYS[2], cjson.encode({owner=ARGV[2], id=ARGV[3]}))
    redis.call('RPUSH', KEYS[3], ARGV[3])
    redis.call('ZADD', KEYS[4], ARGV[4], ARGV[5])
    return 1
  `,
    {
      keys: [
        prefix + `run:${run.id}`,
        prefix + "active",
        prefix + `session:${run.sessionId}:runs`,
        prefix + "sessions",
      ],
      arguments: [
        JSON.stringify(run),
        run.owner,
        run.id,
        String(run.startedAt),
        run.sessionId,
        prefix + "run:",
      ],
    },
  );
  if (result === -1) throw Error("已有 AI 任务进行中，请查看或停止当前任务");
  return result === 1;
}
export async function release(run: AIRun) {
  await (
    await db()
  ).eval(
    `
    local active = redis.call('GET', KEYS[1])
    if active and cjson.decode(active).id == ARGV[1] then redis.call('DEL', KEYS[1]) end
    return 1
  `,
    { keys: [prefix + "active"], arguments: [run.id] },
  );
}
export async function capabilityChecks() {
  return (await readJSON<CapabilityCheck[]>("checks")) ?? [];
}
export async function saveCheck(check: CapabilityCheck) {
  const previous = await capabilityChecks();
  await writeJSON("checks", [
    ...previous.filter(
      (v) =>
        v.baseURL !== check.baseURL ||
        v.model !== check.model ||
        v.effort !== check.effort,
    ),
    check,
  ]);
}

function imagesDirectory() {
  const directory = process.env.AI_DATA_DIR;
  if (!directory) throw Error("服务端缺少 AI_DATA_DIR 持久目录配置");
  return join(directory, "images");
}
export async function putImage(bytes: Buffer): Promise<AIImage> {
  const metadata = await sharp(bytes).metadata();
  // Normalize formats through the existing mature codec, not handwritten pixels.
  const keep = metadata.format === "png" || metadata.format === "jpeg";
  const data = keep ? bytes : await sharp(bytes).png().toBuffer();
  const id = createHash("sha256").update(data).digest("hex");
  const info = {
    id,
    width: metadata.width!,
    height: metadata.height!,
    mediaType: metadata.format === "jpeg" ? "image/jpeg" : "image/png",
  };
  await mkdir(imagesDirectory(), { recursive: true });
  // History persistence and tool output can access the same content-addressed
  // image concurrently. Never truncate its published file during a rewrite.
  const staging = join(imagesDirectory(), `.${id}.${randomUUID()}`);
  try {
    await writeFile(staging, data);
    await rename(staging, join(imagesDirectory(), id));
  } finally {
    await rm(staging, { force: true });
  }
  await writeJSON(`image:${id}`, info);
  return info;
}
export async function getImage(id: string) {
  if (!/^[a-f0-9]{64}$/.test(id)) throw Error("无效图片编号");
  const info = await readJSON<AIImage>(`image:${id}`);
  if (!info) throw Error("历史图片不存在，不能用当前图片替代");
  return { ...info, bytes: await readFile(join(imagesDirectory(), id)) };
}
export async function imageURL(id: string) {
  const image = await getImage(id);
  return `data:${image.mediaType};base64,${image.bytes.toString("base64")}`;
}
export async function imagePart(id: string) {
  const image = await getImage(id);
  return {
    type: "file" as const,
    mediaType: image.mediaType,
    data: { type: "data" as const, data: image.bytes.toString("base64") },
  };
}
// Keep complete SDK response messages, including reasoning items, but externalize
// image content. Redis must never become the image/video transport.
async function transform(value: unknown, hydrate: boolean): Promise<unknown> {
  if (typeof value === "string") {
    if (hydrate && value.startsWith(imagePrefix))
      return imageURL(value.slice(imagePrefix.length));
    if (!hydrate && /^data:image\/[\w.+-]+;base64,/.test(value)) {
      return (
        imagePrefix +
        (
          await putImage(
            Buffer.from(value.slice(value.indexOf(",") + 1), "base64"),
          )
        ).id
      );
    }
    return value;
  }
  if (Array.isArray(value))
    return Promise.all(value.map((v) => transform(v, hydrate)));
  if (value && typeof value === "object") {
    const item = value as Record<string, unknown>;
    if (
      item.type === "file" &&
      String(item.mediaType).startsWith("image/") &&
      typeof item.data === "object" &&
      item.data
    ) {
      const data = item.data as { type: string; data: string };
      if (data.type === "data") {
        const content = hydrate
          ? ((await transform(data.data, true)) as string).replace(
              /^data:[^,]+,/,
              "",
            )
          : await transform(
              `data:${item.mediaType};base64,${data.data}`,
              false,
            );
        return { ...item, data: { ...data, data: content } };
      }
    }
    return Object.fromEntries(
      await Promise.all(
        Object.entries(item).map(async ([k, v]) => [
          k,
          await transform(v, hydrate),
        ]),
      ),
    );
  }
  return value;
}
export async function messages(sessionId: string): Promise<ModelMessage[]> {
  return (await transform(
    modelContext(
      (await readJSON<ModelMessage[]>(`session:${sessionId}:messages`)) ?? [],
    ),
    true,
  )) as ModelMessage[];
}
export async function appendMessages(sessionId: string, value: ModelMessage[]) {
  // register() permits one active run. Append only its new messages; old image
  // references and complete history stay untouched by model input projection.
  const key = `session:${sessionId}:messages`;
  const previous = (await readJSON<ModelMessage[]>(key)) ?? [];
  const added = (await transform(value, false)) as ModelMessage[];
  await writeJSON(key, [...previous, ...added]);
}
