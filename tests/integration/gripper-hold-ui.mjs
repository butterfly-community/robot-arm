// Real browser test: existing grasp pipeline plus the existing force settings.
// Requires authorization, an inspected scene and the robot at its work pose.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, writeFile } from "node:fs/promises";
import { resolve, join } from "node:path";
import { expandSegmentation } from "../../tools/diagnostics/segmentation-ui.mjs";
const [directory, objectId, regionId] = process.argv.slice(2);
if (!directory || !objectId || !regionId)
  throw Error("Usage: gripper-hold-ui.mjs OUTPUT OBJECT_ID REGION_ID");
const output = resolve(directory);
process.env.TMPDIR = join(output, "browser-temp");
await mkdir(process.env.TMPDIR, { recursive: true });
const browser = await chromium.launch({ headless: true });
let page,
  execution,
  values,
  transport,
  requestId,
  original,
  sawOpen = false,
  stop = false,
  recording;
const events = [],
  changes = [];
try {
  const context = await browser.newContext({
    viewport: { width: 1440, height: 1000 },
  });
  page = await context.newPage();
  execution = await context.newPage();
  const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
  page.on("websocket", (socket) =>
    socket.on("framereceived", (frame) => {
      if (typeof frame.payload !== "string") return;
      const v = JSON.parse(frame.payload).values;
      if (v?.perception_state) values = v;
    }),
  );
  execution.on("websocket", (socket) =>
    socket.on("framereceived", (frame) => {
      if (typeof frame.payload !== "string") return;
      const v = JSON.parse(frame.payload).values;
      if (v?.transport_state) {
        transport = v.transport_state;
        if (
          requestId &&
          values?.manipulation_state?.request_id === requestId &&
          values.manipulation_state.state === "executing" &&
          transport.gripper_control_power_mw == null
        )
          sawOpen = true;
        events.push({
          at: Date.now(),
          transport,
          arm: v.arm_state,
          task: values?.manipulation_state,
        });
      }
    }),
  );
  page.on("request", (request) => {
    if (
      request.method() === "POST" &&
      new URL(request.url()).pathname === "/api/perception/pick-place"
    )
      requestId = request.postDataJSON().request_id;
  });
  await execution.goto(`${base}/arm-execution/`);
  await expect(execution.locator(".topbar-state")).toHaveText("实时连接正常");
  const connectionPanel = execution.getByRole("button", { name: /^执行连接/ });
  if ((await connectionPanel.getAttribute("aria-expanded")) !== "true")
    await connectionPanel.click();
  await expect.poll(() => transport?.connected).toBe(true);
  const strength = execution.getByLabel("夹持反馈目标（0–100）", {
    exact: true,
  });
  original = await strength.inputValue();
  await strength.scrollIntoViewIfNeeded();
  await page.goto(`${base}/perception/`);
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
  await expandSegmentation(page);
  await expandSegmentation(page, "抓放场景");
  await page.getByLabel("抓取目标", { exact: true }).selectOption(objectId);
  await page.getByLabel("放置区域", { exact: true }).selectOption(regionId);
  recording = (async () => {
    let index = 0;
    const canvas = page.getByLabel("相机原始彩色视频", { exact: true });
    while (!stop) {
      if (await canvas.isVisible()) {
        try {
          await canvas.screenshot({
            path: join(output, `frame-${String(index++).padStart(3, "0")}.jpg`),
            type: "jpeg",
            quality: 70,
            timeout: 2000,
          });
        } catch {
          /* A frame may be repainted during screenshot; never alter robot flow. */
        }
      }
      await new Promise((resolve) => setTimeout(resolve, 500));
    }
  })();
  await page.getByRole("button", { name: "启动", exact: true }).click();
  await expect
    .poll(
      () =>
        requestId &&
        values?.manipulation_state?.request_id === requestId &&
        ((sawOpen &&
          transport?.gripper_control_power_mw != null &&
          transport.gripper_strength_feedback_percent > 0) ||
          ["failed", "succeeded"].includes(values.manipulation_state.state)),
      { timeout: 180000 },
    )
    .toBe(true);
  expect(values.manipulation_state.state).toBe("executing");
  for (const target of ["20", original]) {
    await strength.fill(target);
    const save = execution.getByRole("button", {
      name: "保存执行配置",
      exact: true,
    });
    await save.click();
    await expect
      .poll(() => transport?.gripper_strength_percent)
      .toBe(Number(target));
    changes.push({
      at: Date.now(),
      target: Number(target),
      transport: structuredClone(transport),
    });
  }
  await expect
    .poll(() => values?.manipulation_state?.state, { timeout: 180000 })
    .toBe("succeeded");
  expect(changes[0].transport.gripper_control_power_mw).not.toBeNull();
  await page.screenshot({
    path: join(output, "result.jpg"),
    type: "jpeg",
    quality: 65,
    fullPage: true,
  });
  console.log(
    JSON.stringify({
      task: values.manipulation_state,
      changes: changes.map((c) => ({
        target: c.target,
        actual: c.transport.gripper_strength_feedback_percent,
        power: c.transport.gripper_control_power_mw,
      })),
    }),
  );
} finally {
  if (
    execution &&
    original != null &&
    transport?.gripper_strength_percent !== Number(original)
  ) {
    await execution
      .getByLabel("夹持反馈目标（0–100）", { exact: true })
      .fill(original);
    await execution
      .getByRole("button", { name: "保存执行配置", exact: true })
      .click();
  }
  if (
    page &&
    requestId &&
    requestId === values?.manipulation_state?.request_id &&
    ["planning", "executing"].includes(values.manipulation_state.state)
  ) {
    await page.getByRole("button", { name: "取消抓放", exact: true }).click();
    await expect
      .poll(
        () =>
          ["cancelled", "failed", "succeeded"].includes(
            values?.manipulation_state?.state,
          ),
        { timeout: 180000 },
      )
      .toBe(true);
  }
  stop = true;
  await recording;
  await writeFile(
    join(output, "results.json"),
    JSON.stringify({ requestId, changes, events, values }, null, 2),
  );
  await browser.close();
}
