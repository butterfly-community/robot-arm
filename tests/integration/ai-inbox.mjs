// Source: production store.ts Lua scripts, evaluated by the deployed Redis.
// Isolated fixture keys only; supplements, not substitutes, for browser tests.
import { readFile } from "node:fs/promises";
import { execFile } from "node:child_process";
import { promisify } from "node:util";
import { randomUUID } from "node:crypto";
import assert from "node:assert/strict";

const exec = promisify(execFile);
const source = await readFile(
  new URL(
    "../../frontend/web/apps/perception/src/app/ai/store.ts",
    import.meta.url,
  ),
  "utf8",
);
function script(name) {
  const body = source.slice(source.indexOf(`export async function ${name}(`));
  const match = body.match(/\.eval\(\s*`([\s\S]*?)`/);
  assert.ok(match, name);
  return match[1];
}
async function redis(...args) {
  return (
    await exec("docker", [
      "compose",
      "exec",
      "-T",
      "redis",
      "redis-cli",
      "--raw",
      ...args,
    ])
  ).stdout.trimEnd();
}
const root = "robot-arm:ai:test:" + randomUUID();
const keys = [root, root + ":updates", root + ":closed", root + ":included"];
const original = JSON.stringify({
  state: "running",
  sessionId: "test",
  images: [],
  calls: [],
  requests: [],
  acceptingInput: true,
});
const item = (id) =>
  JSON.stringify({ id, text: "reminder", images: [], receivedAt: 1 });
const append = (id, session = "test") =>
  redis(
    "EVAL",
    script("addRunUpdate"),
    "3",
    ...keys.slice(0, 3),
    item(id),
    id,
    session,
  );
const close = () => redis("EVAL", script("closeRunInput"), "4", ...keys);
try {
  await redis("SET", keys[0], original);
  assert.equal(await append("one"), "1");
  assert.equal(await append("one"), "1");
  assert.equal(await append("one", "wrong-session"), "0");
  assert.equal(await redis("LLEN", keys[1]), "1");
  assert.equal(await redis("LINDEX", keys[1], "0"), item("one"));
  assert.equal(await close(), "0");
  await redis("HSET", keys[3], "one", "123");
  assert.equal(await close(), "1");
  assert.equal(await append("one"), "1");
  assert.equal(await append("two"), "0");
  assert.equal(await redis("GET", keys[0]), original);
  await redis("DEL", keys[2]);
  const [added, closed] = await Promise.all([append("two"), close()]);
  assert.ok(
    (added === "1" && closed === "0") || (added === "0" && closed === "1"),
  );
  assert.equal(await redis("GET", keys[0]), original);
  console.log(
    "PASS: Redis inbox idempotency, ownership, pending/closed boundary, array preservation, close/append race",
  );
} finally {
  await redis("DEL", ...keys);
}
