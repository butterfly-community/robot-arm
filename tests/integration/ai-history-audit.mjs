// Source: persisted Responses tool results and their original camera image IDs.
// Run through the perception image's Node runtime (stdin); never sends robot commands.
import { createClient } from "redis";
import { readFile } from "node:fs/promises";
import { createHash } from "node:crypto";
import sharp from "sharp";

const [session] = process.argv.slice(2);
if (!session) throw Error("Session ID required");
const redis = createClient({ url: process.env.REDIS_URL });
await redis.connect();
try {
  const key = `robot-arm:ai:session:${session}:messages`;
  const raw = await redis.get(key);
  if (!raw) throw Error("Session history missing");
  const ids = new Set();
  function collect(value) {
    if (typeof value === "string" && value.startsWith("robot-image:")) {
      const id = value.slice("robot-image:".length);
      if (!/^[a-f0-9]{64}$/.test(id)) throw Error(`Invalid image ID: ${id}`);
      ids.add(id);
    } else if (value && typeof value === "object") {
      for (const child of Object.values(value)) collect(child);
    }
  }
  collect(JSON.parse(raw));
  for (const id of ids) {
    const bytes = await readFile(`${process.env.AI_DATA_DIR}/images/${id}`);
    if (createHash("sha256").update(bytes).digest("hex") !== id)
      throw Error(`Image hash mismatch: ${id}`);
    const info = JSON.parse(await redis.get(`robot-arm:ai:image:${id}`));
    const decoded = await sharp(bytes)
      .raw()
      .toBuffer({ resolveWithObject: true });
    if (
      !info ||
      info.width !== decoded.info.width ||
      info.height !== decoded.info.height
    )
      throw Error(`Image metadata mismatch: ${id}`);
  }
  console.log(JSON.stringify({ session, verifiedImages: ids.size }));
} finally {
  await redis.quit();
}
