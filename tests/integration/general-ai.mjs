// Source: real general-AI UI. No API writes, request interception, injected
// results or manual pick/place. Captures browser-visible run records + video.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, writeFile } from "node:fs/promises";
import { resolve, join } from "node:path";
import { existsSync } from "node:fs";
const [mode, directory, text = "", imagePath, effort] = process.argv.slice(2);
if (
  !["inspect", "camera", "config", "send", "stop", "new", "history"].includes(
    mode,
  ) ||
  !directory
)
  throw Error(
    "Usage: general-ai.mjs inspect|camera|config|send|stop|new|history OUTPUT [TEXT or SESSION_ID] [IMAGE] [EFFORT]",
  );
const output = resolve(directory);
process.env.TMPDIR = join(output, "browser-temp");
await mkdir(process.env.TMPDIR, { recursive: true });
const browser = await chromium.launch({ headless: true });
const events = [];
let latest,
  started,
  frame = 0,
  recording = false;
const browserState = resolve(
  process.env.AI_BROWSER_STATE ?? "temp/general-ai/browser-state.json",
);
await mkdir(resolve(browserState, ".."), { recursive: true });
const context = await browser.newContext({
  viewport: { width: 1440, height: 1000 },
  ...(existsSync(browserState) ? { storageState: browserState } : {}),
});
try {
  const page = await context.newPage();
  const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
  page.on("response", async (response) => {
    if (
      new URL(response.url()).pathname === "/perception/api/ai/" &&
      response.ok()
    ) {
      try {
        const v = await response.json();
        events.push({ at: Date.now(), response: v });
        if (v.id) started = v;
      } catch {}
    }
  });
  page.on("websocket", (socket) =>
    socket.on("framereceived", (message) => {
      if (typeof message.payload !== "string") return;
      try {
        const v = JSON.parse(message.payload).values;
        if (v?.perception_state) latest = v;
      } catch {}
    }),
  );
  page.on("pageerror", (error) => console.log("PAGE_ERROR", error.message));
  await page.goto(base + "/perception/");
  await expect(page.getByLabel("AI 任务", { exact: true })).toBeVisible({
    timeout: 60000,
  });
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常", {
    timeout: 60000,
  });
  await expect.poll(() => Boolean(latest), { timeout: 60000 }).toBe(true);
  const canvas = page.getByLabel("相机原始彩色视频", { exact: true });
  async function screenshot(name) {
    if (await canvas.isVisible()) {
      if (latest?.perception_state?.source_id) {
        await expect
          .poll(
            () => canvas.evaluate((el) => el.toDataURL("image/jpeg").length),
            { timeout: 15000 },
          )
          .toBeGreaterThan(10000);
      }
      await canvas.screenshot({
        path: join(output, name + ".jpg"),
        type: "jpeg",
        quality: 75,
      });
    }
  }
  if (mode === "camera") {
    const advanced = page.getByRole("button", { name: /^高级设置/ });
    if ((await advanced.getAttribute("aria-expanded")) === "false")
      await advanced.click();
    const camera = page.getByLabel("相机来源", { exact: true });
    await expect(camera).toBeEnabled({ timeout: 300000 });
    const options = await camera.locator("option").evaluateAll((els) =>
      els.map((e) => ({
        value: e.value,
        label: e.textContent,
        disabled: e.disabled,
      })),
    );
    console.log("CAMERAS", JSON.stringify(options));
    const real = options.find(
      (v) => v.value.startsWith("realsense:") && !v.disabled,
    );
    if (!real) throw Error("网页没有可用 RealSense 相机");
    await camera.selectOption(real.value);
    const save = page.getByRole("button", { name: "保存并启用", exact: true });
    await save.click();
    await expect(save).toBeEnabled({ timeout: 60000 });
    await expect
      .poll(() => latest?.perception_state.source_id, { timeout: 60000 })
      .toBe(real.value);
    await screenshot("camera");
  }
  if (["config", "send"].includes(mode) && imagePath)
    await page
      .getByLabel("添加 AI 图片", { exact: true })
      .setInputFiles(resolve(imagePath));
  if (mode === "config") {
    if (
      (await page
        .getByRole("button", { name: /^模型与思考配置/ })
        .getAttribute("aria-expanded")) === "false"
    )
      await page.getByRole("button", { name: /^模型与思考配置/ }).click();
    await page
      .getByRole("button", { name: "刷新通用模型列表", exact: true })
      .click();
    await expect(
      page.getByRole("button", { name: "刷新通用模型列表", exact: true }),
    ).toBeEnabled({ timeout: 180000 });
    console.log("MODELS", await page.locator("#ai-model-list").textContent());
    if (effort !== undefined)
      await page.getByLabel("AI 思考强度", { exact: true }).fill(effort);
    await page
      .getByRole("button", { name: "保存 AI 配置", exact: true })
      .click();
    await expect(
      page.getByRole("button", { name: "保存 AI 配置", exact: true }),
    ).toBeEnabled();
    await page
      .getByRole("button", { name: "验证连接与能力", exact: true })
      .click();
    await expect(
      page.getByRole("button", { name: "验证连接与能力", exact: true }),
    ).toBeEnabled({ timeout: 300000 });
    console.log("CONFIG", await page.locator(".ai-task-panel").innerText());
  }
  if (mode === "send") {
    await screenshot("before");
    await page.getByLabel("AI 任务", { exact: true }).fill(text);
    await page
      .getByRole("button", { name: "发送 AI 任务", exact: true })
      .click();
    await expect.poll(() => started?.id).toBeTruthy();
    console.log("START", started.id);
    if (process.env.AI_RELOAD_AFTER_START === "1") {
      await page.reload();
      await expect(page.getByLabel("AI 任务", { exact: true })).toBeVisible();
      console.log("RELOADED_RUNNING_TASK", started.id);
    }
    const run = page.locator(`[data-run-id="${started.id}"]`);
    let previousProgress = "";
    let progressAt = 0;
    const timer = setInterval(async () => {
      if (recording) return;
      recording = true;
      try {
        await screenshot(`frame-${String(frame++).padStart(4, "0")}`);
        const progress = await run.innerText();
        const visible = progress.replace(started.input, "").slice(0, 500);
        const stable = visible.replace(/· \d+ 秒/g, "");
        if (stable !== previousProgress || Date.now() - progressAt >= 30000) {
          events.push({ at: Date.now(), runId: started.id, progress: visible });
          console.log("PROGRESS", visible);
          previousProgress = stable;
          progressAt = Date.now();
        }
      } catch {
      } finally {
        recording = false;
      }
    }, 1000);
    try {
      await expect(run).toHaveAttribute(
        "data-run-state",
        /^(succeeded|failed|cancelled|interrupted)$/,
        { timeout: 1800000 },
      );
      console.log("RESULT", await run.innerText());
      await screenshot("after");
      await writeFile(join(output, "result.txt"), await run.innerText());
      // Read the same state the browser reload uses; never submit via API.
      const session = await page.evaluate(() =>
        localStorage.getItem("robot-arm:ai-session"),
      );
      const snapshot = await page.evaluate(
        async (session) =>
          (await fetch("/perception/api/ai/?session=" + session)).json(),
        session,
      );
      await writeFile(
        join(output, "snapshot.json"),
        JSON.stringify(snapshot, null, 2),
      );
      const record = snapshot.runs.find((item) => item.id === started.id);
      const calls = record.calls.map((call) => ({
        name: call.name,
        start_s: (call.startedAt - record.startedAt) / 1000,
        duration_s:
          call.endedAt == null ? null : (call.endedAt - call.startedAt) / 1000,
        error: call.error,
      }));
      const total_s = (record.endedAt - record.startedAt) / 1000;
      const tools_s = calls.reduce(
        (sum, call) => sum + (call.duration_s ?? 0),
        0,
      );
      await writeFile(
        join(output, "timing.json"),
        JSON.stringify(
          {
            run_id: record.id,
            state: record.state,
            total_s,
            tools_s,
            // Tools are serial. This remainder also includes local orchestration;
            // it is not a server-side measurement of model inference alone.
            model_and_orchestration_s: total_s - tools_s,
            calls,
            physical_success:
              "requires review of frames; not inferred from AI state",
          },
          null,
          2,
        ),
      );
    } finally {
      clearInterval(timer);
    }
  }
  if (mode === "stop") {
    await page
      .getByRole("button", { name: "停止 AI 与当前动作", exact: true })
      .click();
    await expect(
      page.locator('[data-run-state="running"],[data-run-state="stopping"]'),
    ).toHaveCount(0, { timeout: 180000 });
  }
  if (mode === "new") {
    await page.getByRole("button", { name: "新会话", exact: true }).click();
    await expect(page.locator("[data-run-id]")).toHaveCount(0);
  }
  if (mode === "history") {
    await page.getByLabel("AI 历史会话", { exact: true }).selectOption(text);
    await expect
      .poll(() => page.locator("[data-run-id]").count())
      .toBeGreaterThan(0);
    console.log(
      "RESTORED_HISTORY",
      text,
      await page.locator("[data-run-id]").count(),
    );
  }
  await writeFile(join(output, "events.json"), JSON.stringify(events, null, 2));
  await writeFile(join(output, "state.json"), JSON.stringify(latest, null, 2));
  await page.screenshot({
    path: join(output, "page.jpg"),
    type: "jpeg",
    quality: 70,
    fullPage: true,
  });
  if (mode === "inspect") {
    await screenshot("camera");
    console.log((await page.locator("body").innerText()).slice(0, 16000));
  }
} finally {
  await context.storageState({ path: browserState });
  await browser.close();
}
