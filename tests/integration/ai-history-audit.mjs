// Source: persisted Responses tool results and their original camera image IDs.
// Run through the perception image's Node runtime (stdin); never sends robot commands.
import { createClient } from "redis";
import { readFile } from "node:fs/promises";
import { createHash } from "node:crypto";
import sharp from "sharp";

const [session, mode] = process.argv.slice(2);
if (!session)
  throw Error("Session ID required; optional second argument: repair");
const redis = createClient({ url: process.env.REDIS_URL });
await redis.connect();
try {
  const key = `robot-arm:ai:session:${session}:messages`;
  const raw = await redis.get(key);
  if (!raw) throw Error("Session history missing");
  const history = JSON.parse(raw);
  let count = 0;
  for (const message of history) {
    for (const item of Array.isArray(message.content) ? message.content : []) {
      if (item.type !== "tool-result" || item.output?.type !== "content")
        continue;
      const values = item.output.value;
      const text = values.find((v) => v.type === "text");
      let original;
      try {
        original = JSON.parse(text?.text).image_id;
      } catch {}
      for (const file of values.filter((v) => v.type === "file")) {
        const ref = file.data?.data;
        if (typeof ref !== "string" || !ref.startsWith("robot-image:"))
          continue;
        const id = ref.slice("robot-image:".length);
        const bytes = await readFile(`${process.env.AI_DATA_DIR}/images/${id}`);
        try {
          await sharp(bytes).raw().toBuffer();
          continue;
        } catch {}
        if (!/^[a-f0-9]{64}$/.test(original))
          throw Error(`No original for ${id}`);
        const full = await readFile(
          `${process.env.AI_DATA_DIR}/images/${original}`,
        );
        if (
          createHash("sha256").update(full).digest("hex") !== original ||
          !full.subarray(0, bytes.length).equals(bytes) ||
          full.length <= bytes.length
        )
          throw Error(`Original does not match truncated image ${id}`);
        await sharp(full).raw().toBuffer();
        if (!(await redis.exists(`robot-arm:ai:image:${original}`)))
          throw Error(`Original metadata missing: ${original}`);
        console.log(
          JSON.stringify({
            tool: item.toolCallId,
            truncated: id,
            original,
            truncatedBytes: bytes.length,
            fullBytes: full.length,
          }),
        );
        file.data.data = `robot-image:${original}`;
        count++;
      }
    }
  }
  if (mode === "repair" && count) {
    await redis.watch(key);
    if ((await redis.get(key)) !== raw)
      throw Error("History changed during audit");
    await redis
      .multi()
      .set(`${key}:before-image-repair:${Date.now()}`, raw)
      .set(key, JSON.stringify(history))
      .exec();
  }
  console.log(JSON.stringify({ repaired: mode === "repair", count }));
} finally {
  await redis.quit();
}
