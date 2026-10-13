import { chooseOption, readOptions } from "./select-controls.mjs";
// Actual AI page, configured provider, generated camera images only. Observation
// request, no movement. Run after stereo-camera's calibration/restart acceptance.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, writeFile } from "node:fs/promises";
import { resolve } from "node:path";

const output = resolve(process.argv[2] ?? "temp/stereo-ai");
const [sessionId, runId] = process.argv.slice(3);
process.env.TMPDIR = resolve("temp/stereo-ai-browser");
await mkdir(output, { recursive: true });
await mkdir(process.env.TMPDIR, { recursive: true });
const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
const browser = await chromium.launch();
const page = await browser.newPage({ viewport: { width: 1600, height: 1100 } });
let enabledSource = false;
try {
  await page.goto(`${base}/perception/`);
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常", {
    timeout: 60000,
  });
  const state = (
    await (await page.request.get(`${base}/api/perception/state`)).json()
  ).values;
  expect(state.camera_state.bindings.filter((b) => b.streaming)).toHaveLength(
    4,
  );
  expect(state.camera_state.streaming).toBe(false);
  // This harness may send only generated images, never private live-camera data.
  expect(
    state.camera_state.bindings
      .filter((b) => b.streaming)
      .every((b) => b.source_id.startsWith("replay:")),
  ).toBe(true);
  const execution = (
    await (await page.request.get(`${base}/api/arm-execution/state`)).json()
  ).values;
  expect(execution.arm_state.feedback_source).toBe("software");
  for (const preview of await page.locator(".floating-camera-monitor").all()) {
    const fold = preview.getByRole("button", { name: "收起", exact: true });
    if (await fold.isVisible()) await fold.click();
  }
  // Select the actual depth source through the same controls as a user.
  // A disabled source need not remain selected after a service restart.
  await page.getByRole("button", { name: "刷新相机列表", exact: true }).click();
  expect(
    (await readOptions(page.getByLabel("相机来源", { exact: true }))).some(
      (option) => option.value === "stereo:wrist" && !option.disabled,
    ),
  ).toBe(true);
  await chooseOption(
    page.getByLabel("相机来源", { exact: true }),
    "stereo:wrist",
  );
  await page.getByRole("button", { name: "保存并启用", exact: true }).click();
  enabledSource = true;
  await expect
    .poll(
      async () => {
        const camera = (
          await (await page.request.get(`${base}/api/perception/state`)).json()
        ).values.camera_state;
        return camera.streaming && camera.selected_source_id === "stereo:wrist";
      },
      { timeout: 60000 },
    )
    .toBe(true);
  const panel = page.locator(".ai-task-panel");
  let run;
  if (sessionId && runId) {
    await chooseOption(
      page.getByLabel("AI 历史会话", { exact: true }),
      sessionId,
    );
    run = { id: runId, sessionId };
  } else {
    await panel.getByRole("button", { name: "新会话", exact: true }).click();
    await page
      .getByLabel("AI 任务", { exact: true })
      .fill(
        "只读双目测试验收：当前四路都是生成的测试画面。分别使用 observe_stereo_depth 查看 external 和 wrist 的校正图中心（u=0.5,v=0.5），报告两组中心深度、基座坐标和有效像素数。说明当前模式如何分割定位、由谁决定抓取动作，以及只有一组双目另一侧单目是否可用。不要运动，不要切换或修改配置，不要抓放；完成两个观察与说明后结束。",
      );
    const response = page.waitForResponse(
      (r) =>
        r.request().method() === "POST" &&
        new URL(r.url()).pathname === "/perception/api/ai/",
    );
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
  }
  console.log("START", run.id);
  const article = panel.locator(`[data-run-id="${run.id}"]`);
  await expect(article).toHaveAttribute(
    "data-run-state",
    /^(succeeded|failed|cancelled|interrupted)$/,
    { timeout: 360000 },
  );
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
  expect(result.promptMode).toBe("stereo");
  expect(
    result.calls.some((c) =>
      ["generate_grasps", "pick_place"].includes(c.name),
    ),
  ).toBe(false);
  expect(await article.getAttribute("data-run-state")).toBe(result.state);
  expect(result.requests).toHaveLength(0);
  for (const rig of ["external", "wrist"]) {
    const call = result.calls.find(
      (c) =>
        c.name === "observe_stereo_depth" && c.input.rig === rig && !c.error,
    );
    expect(call, `${rig} successful stereo tool call`).toBeTruthy();
    expect(call.result.point_in_base_m).toHaveLength(3);
    expect(call.result.valid_pixels).toBeGreaterThan(0);
    expect(
      Math.abs(call.result.point_in_rectified_camera_m[2] - 0.5),
    ).toBeLessThan(0.001);
  }
  console.log(
    "PASS: configured model observed both stereo rigs through the actual AI page; no robot requests",
  );
} finally {
  if (enabledSource) {
    await page.getByRole("button", { name: "停用相机", exact: true }).click();
    await expect
      .poll(
        async () =>
          (
            await (
              await page.request.get(`${base}/api/perception/state`)
            ).json()
          ).values.camera_state.streaming,
        { timeout: 30000 },
      )
      .toBe(false);
  }
  await browser.close();
}
