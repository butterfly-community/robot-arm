// Source: actual AI page and configured provider; formal replay images only.
// Verifies the model's default camera observation policy, without robot motion.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, writeFile } from "node:fs/promises";
import { resolve } from "node:path";

const output = resolve(process.argv[2] ?? "temp/ai-camera-parallel");
process.env.TMPDIR = resolve("temp/ai-camera-parallel-browser");
await mkdir(output, { recursive: true });
await mkdir(process.env.TMPDIR, { recursive: true });
const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
const browser = await chromium.launch();
const page = await browser.newPage({ viewport: { width: 1600, height: 1100 } });
const titles = {
  external: "外部摄像头 1（左）",
  external_right: "外部摄像头 2（右）",
  wrist: "腕部摄像头 1（左）",
  wrist_right: "腕部摄像头 2（右）",
};
const readCamera = async () =>
  (await (await page.request.get(`${base}/api/perception/state`)).json()).values
    .camera_state;
const enabled = [];
let run;
let terminal = false;
let interruptedCapture = false;
let observing = false;
page.on("websocket", (socket) =>
  socket.on("framereceived", ({ payload }) => {
    if (!observing || typeof payload !== "string") return;
    const camera = JSON.parse(payload).values?.camera_state;
    if (
      camera?.bindings?.some((b) => b.source_id && (!b.enabled || !b.streaming))
    )
      interruptedCapture = true;
  }),
);
try {
  await page.goto(`${base}/perception/`);
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常", {
    timeout: 60000,
  });
  const before = await readCamera();
  expect(before.selected_source_id).toBeNull();
  expect(before.bindings.every((b) => b.source_id?.startsWith("replay:"))).toBe(
    true,
  );
  expect(before.bindings.some((b) => b.role === "external")).toBe(true);
  expect(before.bindings.some((b) => b.role === "wrist")).toBe(true);
  const execution = (
    await (await page.request.get(`${base}/api/arm-execution/state`)).json()
  ).values;
  expect(execution.arm_state.feedback_source).toBe("software");
  await writeFile(
    resolve(output, "before.json"),
    JSON.stringify(before, null, 2),
  );
  for (const binding of before.bindings) {
    if (binding.enabled) continue;
    await page
      .getByRole("button", {
        name: `开启${titles[binding.role]}采集`,
        exact: true,
      })
      .click();
    enabled.push(binding.role);
  }
  await expect
    .poll(async () => (await readCamera()).bindings.every((b) => b.streaming), {
      timeout: 60000,
    })
    .toBe(true);
  for (const binding of before.bindings) {
    const preview = page.getByLabel(`${titles[binding.role]}浮动窗口`, {
      exact: true,
    });
    await expect(preview).toBeVisible();
    const fold = preview.getByRole("button", { name: "收起", exact: true });
    if (await fold.isVisible()) await fold.click();
  }
  const panel = page.locator(".ai-task-panel");
  await panel.getByRole("button", { name: "新会话", exact: true }).click();
  await page
    .getByLabel("AI 任务", { exact: true })
    .fill(
      "分别查看外部摄像头和腕部摄像头的新画面，各说明图中看到了什么。只做观察，不运动、不抓放、不运行分割，完成两个观察后结束。",
    );
  const response = page.waitForResponse(
    (r) =>
      r.request().method() === "POST" &&
      new URL(r.url()).pathname === "/perception/api/ai/",
  );
  observing = true;
  await panel
    .getByRole("button", { name: "发送 AI 任务", exact: true })
    .click();
  const submitted = await response;
  expect(submitted.ok()).toBe(true);
  run = await submitted.json();
  await writeFile(
    resolve(output, "started.json"),
    JSON.stringify({ id: run.id, sessionId: run.sessionId }),
  );
  console.log("START", run.id);
  const article = panel.locator(`[data-run-id="${run.id}"]`);
  await expect(article).toHaveAttribute(
    "data-run-state",
    /^(succeeded|failed|cancelled|interrupted)$/,
    { timeout: 360000 },
  );
  terminal = true;
  observing = false;
  const snapshot = await (
    await page.request.get(
      `${base}/perception/api/ai/?session=${encodeURIComponent(run.sessionId)}`,
    )
  ).json();
  const result = snapshot.runs.find((r) => r.id === run.id);
  await writeFile(
    resolve(output, "result.json"),
    JSON.stringify(result, null, 2),
  );
  await article.screenshot({ path: resolve(output, "result.png") });
  expect(result.state, result.error).toBe("succeeded");
  expect(result.requests).toHaveLength(0);
  for (const role of ["external", "wrist"]) {
    const call = result.calls.find(
      (c) => c.name === "observe_camera" && c.input.role === role && !c.error,
    );
    expect(call).toBeTruthy();
    expect(call.input.exclusive).not.toBe(true);
    expect(call.result.image_id).toBeTruthy();
  }
  expect(interruptedCapture).toBe(false);
  expect(
    (await readCamera()).bindings.every((b) => b.enabled && b.streaming),
  ).toBe(true);
  console.log(
    "PASS: AI observed both roles; all streams remained open; no control requests",
  );
} finally {
  observing = false;
  if (run && !terminal) {
    await page
      .getByRole("button", { name: "停止 AI 与当前动作", exact: true })
      .click();
    await expect(page.locator(`[data-run-id="${run.id}"]`)).toHaveAttribute(
      "data-run-state",
      /^(succeeded|failed|cancelled|interrupted)$/,
      { timeout: 60000 },
    );
  }
  for (const role of enabled) {
    const off = page.getByRole("button", {
      name: `关闭${titles[role]}采集`,
      exact: true,
    });
    if (await off.isVisible()) await off.click();
  }
  await browser.close();
}
