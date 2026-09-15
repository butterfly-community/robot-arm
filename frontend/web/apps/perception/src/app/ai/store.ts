import { createClient } from "redis";
import { createHash, randomUUID } from "node:crypto";
import { mkdir, readFile, writeFile } from "node:fs/promises";
import { join } from "node:path";
import sharp from "sharp";
import type { ModelMessage } from "ai";
import {
  activeRun,
  type AIImage,
  type AIRun,
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
  if (run && activeRun(run) && run.owner !== runtime().owner) {
    run.state = "interrupted";
    run.error =
      "AI 服务已重启；未确认的机器人动作需查询原请求，不自动续跑或重放。";
    await saveRun(run);
  }
  return run;
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
      if a.owner == ARGV[2] then return -1 end
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
  await writeFile(join(imagesDirectory(), id), data);
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
    (await readJSON(`session:${sessionId}:messages`)) ?? [],
    true,
  )) as ModelMessage[];
}
export async function saveMessages(sessionId: string, value: ModelMessage[]) {
  await writeJSON(
    `session:${sessionId}:messages`,
    await transform(value, false),
  );
}
