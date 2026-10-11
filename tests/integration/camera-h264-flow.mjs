// Actual page operations, no interception or internal API writes. Software
// execution is explicitly selected before calibration or grasp motion.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, readFile, writeFile } from "node:fs/promises";
import { resolve } from "node:path";

const output = resolve(process.argv[2] ?? "temp/h264-flow");
process.env.TMPDIR = resolve(output, "browser");
await mkdir(process.env.TMPDIR, { recursive: true });
const browser = await chromium.launch();
const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
const page = await browser.newPage({ viewport: { width: 1600, height: 1100 } });
const execution = await browser.newPage();
const events = [];
const state = async (namespace = "perception") =>
  (await (await page.request.get(`${base}/api/${namespace}/state`)).json())
    .values;
let original, originalExecution;
async function expand(p, name) {
  const button = p.getByRole("button", { name: new RegExp(`^${name}`) });
  if ((await button.getAttribute("aria-expanded")) === "false")
    await button.click();
}
async function waitFor(read, predicate, timeout = 180000) {
  await expect
    .poll(async () => predicate(await read()), { timeout, intervals: [500] })
    .toBe(true);
}
async function click(name) {
  console.log("UI", name);
  await page.getByRole("button", { name, exact: true }).click();
}
try {
  await page.goto(`${base}/perception/`);
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常", {
    timeout: 60000,
  });
  original = await state();
  originalExecution = await state("arm-execution");
  await writeFile(
    resolve(output, "before.json"),
    JSON.stringify({ original, originalExecution }),
  );
  page.on("request", (request) => {
    if (request.method() === "POST")
      events.push({
        at: Date.now(),
        url: request.url(),
        body: request.postDataJSON(),
      });
  });
  await execution.goto(`${base}/arm-execution/`);
  await expand(execution, "执行连接");
  await execution
    .getByLabel("执行模式", { exact: true })
    .selectOption("software");
  const apply = execution.getByRole("button", {
    name: "应用执行模式",
    exact: true,
  });
  if (await apply.isVisible()) await apply.click();
  await waitFor(
    () => state("arm-execution"),
    (s) =>
      s.transport_state.selected_endpoint === null &&
      s.arm_state.feedback_source === "software",
  );

  await page
    .getByLabel("相机来源", { exact: true })
    .selectOption("simulation:pick-place-scene");
  for (const [label, value] of [
    ["彩色流", "color:1920x1080:rgb8:10"],
    ["深度流", "depth:1920x1080:z16le:10"],
  ])
    await page
      .getByRole("combobox", { name: label, exact: true })
      .selectOption(value);
  await page.getByRole("spinbutton", { name: "上送频率" }).fill("1");
  await click("保存并启用");
  await waitFor(
    state,
    (s) =>
      s.camera_state.streaming &&
      s.perception_state.color_frame?.encoding === "h264",
  );
  await click("刷新图像");
  await expect(
    page.getByRole("img", { name: "深度图", exact: true }),
  ).toBeVisible();

  if (process.env.H264_REUSE_CALIBRATION !== "1") {
    const saved = page.getByRole("button", { name: "重新标定", exact: true });
    await click((await saved.isVisible()) ? "重新标定" : "开始自动标定");
    await waitFor(state, (s) =>
      ["awaiting_confirmation", "failed"].includes(s.calibration_state.phase),
    );
    const completed = await state();
    expect(
      completed.calibration_state.phase,
      completed.calibration_state.stage_message,
    ).toBe("awaiting_confirmation");
    await click("确认并应用标定");
  }
  await waitFor(state, (s) => s.calibration_state.phase === "applied");
  await writeFile(
    resolve(output, "calibration.json"),
    JSON.stringify((await state()).calibration_state, null, 2),
  );
  const solved = (await state()).calibration_state.solved_result;
  const oracle = JSON.parse(
    await readFile(
      "backend/nodes/camera/assets/simulation-cameras.json",
      "utf8",
    ),
  ).find((camera) => camera.source_id === "simulation:pick-place-scene");
  const positionError = Math.hypot(
    ...solved.camera_in_base.position_m.map(
      (value, index) => value - oracle.camera_in_base.position_m[index],
    ),
  );
  expect(solved.sample_count).toBe(9);
  expect(positionError).toBeLessThanOrEqual(0.001);
  expect(Math.max(...solved.translation_residuals_m)).toBeLessThanOrEqual(
    0.001,
  );
  console.log("CALIBRATION_POSITION_ERROR_MM", positionError * 1000);

  await click("载入新分割帧");
  await expand(page, "提示词分割");
  await page.getByLabel("提示方式", { exact: true }).selectOption("text");
  await page
    .getByLabel("识别与分割提示词", { exact: true })
    .fill("red cube, dark rectangle");
  // The synthetic tray is matched by its appearance, not its semantic name.
  await page.getByLabel("放置区域角色", { exact: true }).fill("dark rectangle");
  // Run with the editor's draft fields. Do not replace the user's saved visual
  // examples merely to exercise text segmentation on a synthetic scene.
  await click("运行提示词分割");
  await waitFor(state, (s) => s.perception_state.instances?.length >= 2);
  await click("三维定位");
  await waitFor(
    state,
    (s) =>
      s.world_scene?.objects.length > 0 &&
      s.world_scene?.placement_regions.length > 0,
  );
  let current = await state();
  await writeFile(
    resolve(output, "scene.json"),
    JSON.stringify(current, null, 2),
  );
  await expand(page, "抓放场景");
  const target = current.world_scene.objects.find(
    (o) => o.label === "red cube",
  );
  expect(target).toBeTruthy();
  await page
    .getByLabel("抓取目标", { exact: true })
    .selectOption(target.object_id);
  await page
    .getByLabel("放置区域", { exact: true })
    .selectOption(current.world_scene.placement_regions[0].region_id);
  await click("启动");
  await waitFor(
    async () => events,
    (items) => items.some((e) => e.url.endsWith("/api/perception/pick-place")),
    180000,
  );
  const requestId = events.find((e) =>
    e.url.endsWith("/api/perception/pick-place"),
  ).body.request_id;
  await waitFor(
    state,
    (s) =>
      s.manipulation_state.request_id === requestId &&
      ["succeeded", "failed", "cancelled"].includes(s.manipulation_state.state),
    300000,
  );
  current = await state();
  expect(
    current.manipulation_state.state,
    JSON.stringify(current.manipulation_state),
  ).toBe("succeeded");
  await expect(
    page.getByText("执行完成", { exact: true }).first(),
  ).toBeVisible();
  await expect(
    page.getByText(requestId, { exact: true }).first(),
  ).toBeVisible();
  await writeFile(
    resolve(output, "result.json"),
    JSON.stringify(current, null, 2),
  );
  await page.screenshot({
    path: resolve(output, "result.png"),
    fullPage: true,
  });
  console.log(
    "SIMULATION_H264_FLOW_PASSED",
    JSON.stringify(current.manipulation_state),
  );
} finally {
  await writeFile(
    resolve(output, "events.json"),
    JSON.stringify(events, null, 2),
  );
  if (original) {
    if (
      ["planning", "executing"].includes(
        (await state()).manipulation_state.state,
      )
    ) {
      await expand(page, "抓放场景");
      await click("取消抓放");
      await waitFor(
        state,
        (s) => !["planning", "executing"].includes(s.manipulation_state.state),
      );
    }
    const stop = page.getByRole("button", {
      name: "终止本次标定",
      exact: true,
    });
    if ((await stop.isVisible()) && (await stop.isEnabled()))
      await stop.click();
    // Restore through the same controls; leave the new simulation calibration
    // attached to its simulation source, never overwrite real-camera extrinsics.
    await page
      .getByLabel("相机来源", { exact: true })
      .selectOption(original.camera_state.selected_source_id ?? "");
    await waitFor(
      state,
      (s) =>
        s.camera_state.selected_source_id ===
        original.camera_state.selected_source_id,
    );
  }
  if (originalExecution?.transport_state.selected_endpoint) {
    await execution
      .getByLabel("执行模式", { exact: true })
      .selectOption("hardware");
    await execution
      .getByLabel("串口路径（可手动输入）", { exact: true })
      .fill(originalExecution.transport_state.selected_endpoint);
    if (originalExecution.transport_state.connected) {
      await execution
        .getByRole("button", { name: "连接真机", exact: true })
        .click();
      await waitFor(
        () => state("arm-execution"),
        (s) =>
          s.transport_state.connected &&
          s.transport_state.selected_endpoint ===
            originalExecution.transport_state.selected_endpoint,
      );
    }
  }
  await browser.close();
}
