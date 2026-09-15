// Authorized real browser acceptance. Does not intercept or fabricate commands/results.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, writeFile } from "node:fs/promises";
import { resolve, join } from "node:path";
import {
  captureSegmentationFrame,
  selectSegmentationModel,
  expandSegmentation,
} from "../../tools/diagnostics/segmentation-ui.mjs";

const [mode, directory, objectId, regionId] = process.argv.slice(2);
if (
  !["observe", "queued", "planning", "executing", "complete"].includes(mode) ||
  !directory
)
  throw Error(
    "Usage: grasp-cancel.mjs observe|queued|planning|executing|complete OUTPUT [OBJECT_ID REGION_ID]",
  );
const output = resolve(directory);
process.env.TMPDIR = join(output, "browser-temp");
await mkdir(process.env.TMPDIR, { recursive: true });
const browser = await chromium.launch({ headless: true });
let page, values, graspId, motionPage, motionValues, motionId;
const events = [],
  records = [];
try {
  const context = await browser.newContext({
    viewport: { width: 1440, height: 1000 },
  });
  page = await context.newPage();
  const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
  page.on("websocket", (socket) =>
    socket.on("framereceived", (frame) => {
      if (typeof frame.payload !== "string") return;
      const v = JSON.parse(frame.payload).values;
      if (!v?.perception_state) return;
      values = v;
      const task = v.manipulation_state;
      if (task?.request_id === graspId)
        events.push({
          at: Date.now(),
          task,
          arm: v.arm_state,
          transport: v.transport_state,
        });
    }),
  );
  page.on("request", (request) => {
    if (request.method() !== "POST") return;
    const path = new URL(request.url()).pathname,
      body = request.postDataJSON();
    if (path === "/api/perception/pick-place") graspId = body.request_id;
    events.push({ at: Date.now(), path, body });
  });
  page.on("response", async (response) => {
    if (
      new URL(response.url()).pathname.startsWith("/api/requests/") &&
      response.ok()
    ) {
      try {
        records.push(await response.json());
      } catch {
        /* navigation */
      }
    }
  });
  await page.goto(`${base}/perception/`);
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常", {
    timeout: 60000,
  });
  await expect.poll(() => Boolean(values?.perception_state)).toBe(true);
  await expandSegmentation(page);
  if (mode === "observe") {
    await captureSegmentationFrame(page);
    const run = await selectSegmentationModel(page, values.perception_state);
    await run.click();
    await expect(run).toBeEnabled({ timeout: 180000 });
    const locate = page.getByRole("button", { name: "三维定位", exact: true });
    await expect(locate).toBeEnabled();
    await locate.click();
    await expect(locate).toBeEnabled({ timeout: 180000 });
    await expect
      .poll(
        () =>
          values?.world_scene?.sequence ===
          values?.perception_state.last_scene_sequence,
      )
      .toBe(true);
    console.log(
      JSON.stringify({
        objects: values.world_scene.objects,
        regions: values.world_scene.placement_regions,
      }),
    );
    await expandSegmentation(page, "分割结果");
    await page
      .getByRole("group", { name: "全部分割结果图", exact: true })
      .screenshot({
        path: join(output, "frame.jpg"),
        type: "jpeg",
        quality: 65,
      });
  } else {
    if (!objectId || !regionId)
      throw Error("Select inspected object/region IDs from observe output");
    if (mode === "queued") {
      motionPage = await page.context().newPage();
      motionPage.on("websocket", (socket) =>
        socket.on("framereceived", (frame) => {
          if (typeof frame.payload === "string") {
            const next = JSON.parse(frame.payload).values;
            if (next?.motion_state) motionValues = next;
          }
        }),
      );
      motionPage.on("request", (request) => {
        if (
          request.method() === "POST" &&
          new URL(request.url()).pathname === "/api/motion/request"
        )
          motionId = request.postDataJSON().request_id;
      });
      await motionPage.goto(`${base}/motion/`);
      await expect(motionPage.locator(".topbar-state")).toHaveText(
        "实时连接正常",
      );
      await expandSegmentation(motionPage, "控制模式 / 规划");
      await motionPage
        .getByRole("button", { name: "工作位", exact: true })
        .click();
      await expect
        .poll(
          () =>
            motionId &&
            motionValues?.motion_state?.latest_motion?.request_id ===
              motionId &&
            motionValues.motion_state.latest_motion.state,
          { timeout: 60000 },
        )
        .toBe("succeeded");
      await expandSegmentation(motionPage, "关节与夹爪状态 / 手动目标");
      for (let i = 0; i < 4; i++)
        await motionPage
          .getByRole("slider", { name: "J1", exact: true })
          .press("ArrowRight");
      await motionPage.getByLabel("速度倍率", { exact: true }).fill("0.01");
      const before = motionId;
      await motionPage
        .getByRole("button", { name: "执行目标", exact: true })
        .click();
      await expect
        .poll(
          () =>
            motionId !== before &&
            motionValues?.motion_state?.latest_motion?.request_id ===
              motionId &&
            motionValues.motion_state.latest_motion.state,
          { timeout: 60000 },
        )
        .toBe("executing");
    }
    await expandSegmentation(page, "抓放场景");
    await page.getByLabel("抓取目标", { exact: true }).selectOption(objectId);
    await page.getByLabel("放置区域", { exact: true }).selectOption(regionId);
    await page.getByRole("button", { name: "启动", exact: true }).click();
    await expect.poll(() => Boolean(graspId), { timeout: 180000 }).toBe(true);
    if (mode !== "complete") {
      await expect
        .poll(
          () =>
            values?.manipulation_state?.request_id === graspId &&
            values?.manipulation_state?.state,
          { timeout: 180000 },
        )
        .toBe(mode === "queued" ? "planning" : mode);
      if (mode === "queued") {
        expect(motionValues.motion_state.latest_motion.state).toBe("executing");
        expect(values.manipulation_state.stage).toBe("等待 MTC 规划");
      }
      if (mode === "planning")
        await expect
          .poll(() => values?.manipulation_state?.stage, {
            timeout: 180000,
            intervals: [50],
          })
          .toBe("search complete task solutions");
      await page.getByRole("button", { name: "取消抓放", exact: true }).click();
    }
    await expect
      .poll(
        () =>
          values?.manipulation_state?.request_id === graspId &&
          values?.manipulation_state?.state,
        { timeout: 180000 },
      )
      .toBe(mode === "complete" ? "succeeded" : "cancelled");
    await expandSegmentation(page, "请求结果");
    await page.getByLabel("查询请求编号").fill(graspId);
    await page.getByRole("button", { name: "查看请求结果" }).click();
    await expect
      .poll(() => records.findLast((r) => r.request_id === graspId)?.state)
      .toBe(mode === "complete" ? "succeeded" : "cancelled");
    console.log(JSON.stringify(values.manipulation_state));
  }
  await page.screenshot({ path: join(output, `${mode}.png`), fullPage: true });
} finally {
  if (
    page &&
    graspId &&
    values?.manipulation_state?.request_id === graspId &&
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
  if (motionPage) {
    if (
      ["planning", "executing"].includes(
        motionValues?.motion_state?.latest_motion?.state,
      )
    ) {
      await motionPage
        .getByRole("button", { name: "取消普通运动", exact: true })
        .click();
      await expect
        .poll(
          () =>
            ["cancelled", "succeeded"].includes(
              motionValues?.motion_state?.latest_motion?.state,
            ),
          { timeout: 60000 },
        )
        .toBe(true);
    }
    await motionPage.getByLabel("速度倍率", { exact: true }).fill("");
    const before = motionId;
    await motionPage
      .getByRole("button", { name: "工作位", exact: true })
      .click();
    await expect
      .poll(
        () =>
          motionId !== before &&
          motionValues?.motion_state?.latest_motion?.request_id === motionId &&
          motionValues.motion_state.latest_motion.state,
        { timeout: 60000 },
      )
      .toBe("succeeded");
  }
  await writeFile(
    join(output, `${mode}.json`),
    JSON.stringify({ values, events, records }, null, 2),
  );
  await browser.close();
}
