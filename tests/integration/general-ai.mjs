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
  ![
    "inspect",
    "camera",
    "settings",
    "reply-layout",
    "config",
    "send",
    "stop",
    "new",
    "history",
  ].includes(mode) ||
  !directory
)
  throw Error(
    "Usage: general-ai.mjs inspect|camera|settings|reply-layout|config|send|stop|new|history OUTPUT [TEXT or SESSION_ID] [IMAGE] [EFFORT]",
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
  if (["inspect", "camera", "send"].includes(mode)) {
    const expand = page.getByRole("button", { name: "展开", exact: true });
    if (await expand.isVisible()) await expand.click();
  }
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
  if (mode === "reply-layout") {
    // Reopen an actual completed multi-step conversation; never inject its text.
    const minimizeVideo = page.getByRole("button", {
      name: "收起",
      exact: true,
    });
    if (await minimizeVideo.isVisible()) await minimizeVideo.click();
    const card = page.locator(".ai-run").last();
    await expect(card).toHaveAttribute("data-run-state", "succeeded");
    const reply = card.locator(".ai-response");
    const content = await reply.textContent();
    expect(content).toContain("\n\n");
    const conversation = page.getByLabel("AI 会话记录", { exact: true });
    await expect(conversation).toHaveCount(1);
    const inputs = await conversation
      .locator(".ai-user-text")
      .allTextContents();
    const replies = await conversation
      .locator(".ai-response")
      .allTextContents();
    for (const width of [1440, 390]) {
      await page.setViewportSize({ width, height: 1000 });
      const disclosure = card.locator(":scope > .disclosure");
      const gap = await disclosure.evaluate(
        (el) =>
          el.getBoundingClientRect().top -
          el.previousElementSibling.getBoundingClientRect().bottom,
      );
      expect(gap).toBeGreaterThanOrEqual(20);
      await expect(reply).toHaveCSS("white-space", "pre-wrap");
      await expect(card).toHaveCSS("border-top-width", "0px");
      await expect(card.getByLabel("AI 状态", { exact: true })).toContainText(
        "本轮回复已结束",
      );
      await card.screenshot({ path: join(output, `reply-${width}.png`) });
      console.log("REPLY_LAYOUT", JSON.stringify({ width, gap }));
    }
    await page.reload();
    await expect(reply).toHaveText(content);
    await expect(conversation.locator(".ai-user-text")).toHaveText(inputs);
    await expect(conversation.locator(".ai-response")).toHaveText(replies);
    await page.setViewportSize({ width: 1440, height: 1000 });
    await conversation.screenshot({
      path: join(output, "conversation.jpg"),
      type: "jpeg",
      quality: 70,
    });
    console.log("CONVERSATION_PERSISTED", inputs.length);
    console.log("REPLY_LAYOUT_PASS", JSON.stringify(content));
  }
  if (mode === "settings") {
    // Real settings round trip only: no task, image upload, or motion request.
    const toggle = page.getByRole("button", { name: /^模型与思考配置/ });
    async function openSettings() {
      if ((await toggle.getAttribute("aria-expanded")) === "false")
        await toggle.click();
    }
    await openSettings();
    const model = page.getByLabel("通用 AI 模型", { exact: true });
    const reasoning = page.getByLabel("AI 思考强度", { exact: true });
    const original = {
      model: await model.inputValue(),
      effort: await reasoning.inputValue(),
    };
    async function save() {
      const saved = page.waitForResponse((response) => {
        const request = response.request();
        return (
          new URL(response.url()).pathname === "/perception/api/ai/" &&
          request.method() === "POST" &&
          request.postDataJSON()?.action === "settings"
        );
      });
      await page
        .getByRole("button", { name: "保存 AI 配置", exact: true })
        .click();
      expect((await saved).ok()).toBe(true);
      await expect(
        page.getByRole("button", { name: "保存 AI 配置", exact: true }),
      ).toBeEnabled();
      await expect(page.locator(".ai-task-panel")).not.toContainText(
        "有未保存修改",
      );
    }
    try {
      await page
        .getByRole("button", { name: "选择通用 AI 模型", exact: true })
        .click();
      const options = page.getByRole("listbox").getByRole("option");
      await expect(options.first()).toBeVisible({
        timeout: 180000,
      });
      const offered = await options.allTextContents();
      console.log("MODEL_OPTIONS", offered.length, JSON.stringify(offered));
      await options.first().click();
      await expect(model).not.toHaveValue("");
      await model.fill("custom-ui-draft");
      await model.press("Escape");
      await reasoning.fill("custom-ui-effort");
      await reasoning.press("Escape");
      await expect(model).toHaveValue("custom-ui-draft");
      await expect(reasoning).toHaveValue("custom-ui-effort");
      await page.reload();
      await openSettings();
      await expect(model).toHaveValue(original.model);
      await expect(reasoning).toHaveValue(original.effort);
      await page
        .getByRole("button", { name: "选择AI 思考强度", exact: true })
        .click();
      await page.getByRole("option", { name: "模型默认", exact: true }).click();
      await expect(reasoning).toHaveValue("");
      await save();
      await page.reload();
      await openSettings();
      await expect(reasoning).toHaveValue("");
      await model.fill(original.model);
      await model.press("Escape");
      await reasoning.fill(original.effort);
      await reasoning.press("Escape");
      await save();
      await page.reload();
      await openSettings();
      await expect(model).toHaveValue(original.model);
      await expect(reasoning).toHaveValue(original.effort);
      for (const width of [1440, 390]) {
        await page.setViewportSize({ width, height: 1000 });
        await page
          .getByRole("button", { name: "选择AI 思考强度", exact: true })
          .click();
        const popup = page.locator(".editable-select-popup");
        await expect(popup).toBeVisible();
        const box = await popup.boundingBox();
        expect(box.x).toBeGreaterThanOrEqual(0);
        expect(box.x + box.width).toBeLessThanOrEqual(width);
        await page.screenshot({
          path: join(output, `settings-${width}.jpg`),
          type: "jpeg",
          quality: 75,
        });
        await reasoning.press("Escape");
      }
      console.log("SETTINGS_UI_PASS", JSON.stringify(original));
    } finally {
      await model.fill(original.model);
      await model.press("Escape");
      await reasoning.fill(original.effort);
      await reasoning.press("Escape");
      await save();
    }
  }
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
    await page
      .getByRole("button", { name: "选择通用 AI 模型", exact: true })
      .click();
    console.log("MODELS", await page.getByRole("listbox").innerText());
    await page.getByLabel("通用 AI 模型", { exact: true }).press("Escape");
    if (effort !== undefined) {
      await page.getByLabel("AI 思考强度", { exact: true }).fill(effort);
      await page.getByLabel("AI 思考强度", { exact: true }).press("Escape");
    }
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
