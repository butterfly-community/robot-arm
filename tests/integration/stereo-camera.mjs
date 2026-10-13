import { chooseOption } from "./select-controls.mjs";
// Real browser controls only. No intercepted responses, calibration injection,
// internal write APIs or hardware motion. Four explicit file-backed RGB inputs
// go through normal capture, H.264, OpenCV and the normal RGB-D scene contract.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, readFile, writeFile } from "node:fs/promises";
import { resolve } from "node:path";

const output = resolve(process.argv[2] ?? "temp/stereo-ui");
const verifyOnly = process.argv.includes("--verify-only");
process.env.TMPDIR = resolve(output, "browser");
await mkdir(process.env.TMPDIR, { recursive: true });
const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
const truth = JSON.parse(
  await readFile("backend/nodes/camera/assets/stereo/truth.json", "utf8"),
);
const browser = await chromium.launch();
const context = await browser.newContext({
  viewport: { width: 1600, height: 1100 },
});
const page = await context.newPage();
const execution = await context.newPage();
const motion = await context.newPage();
const reports = [];
const requests = [];
const labels = {
  external: "外部摄像头 1（左）",
  external_right: "外部摄像头 2（右）",
  wrist: "腕部摄像头 1（左）",
  wrist_right: "腕部摄像头 2（右）",
};
const state = async (area = "perception") =>
  (await (await page.request.get(`${base}/api/${area}/state`)).json()).values;
const stereo = async () =>
  await (await page.request.get(`${base}/api/camera/stereo`)).json();
const wait = async (fn, predicate, timeout = 180000) =>
  expect
    .poll(async () => predicate(await fn()), { timeout, intervals: [500] })
    .toBe(true);
async function expand(p, name) {
  const b = p.getByRole("button", { name: new RegExp(`^${name}`) });
  if ((await b.getAttribute("aria-expanded")) === "false") await b.click();
}
const card = (name) =>
  page.locator("section.card").filter({
    has: page
      .locator(".card-title-text")
      .filter({ hasText: new RegExp(`^${name}$`) }),
  });
async function cameraAction(name) {
  console.log("UI camera", name);
  const path =
    name === "刷新图像" ? "/api/perception/request" : "/api/perception/camera";
  const response = page.waitForResponse(
    (r) =>
      r.request().method() === "POST" && new URL(r.url()).pathname === path,
  );
  await page.getByRole("button", { name, exact: true }).click();
  const received = await response;
  const value = await received.json();
  expect(received.ok(), JSON.stringify(value)).toBe(true);
  expect(value.original_error, JSON.stringify(value)).toBeFalsy();
}
async function bind(role, frame, width) {
  const label = labels[role];
  const section = page.getByRole("region", {
    name: `${label}绑定`,
    exact: true,
  });
  const rig = role.startsWith("external") ? "external" : "wrist";
  const source = `replay:${rig}-${role.endsWith("right") ? "right" : "left"}`;
  const profile = `${frame}:${width}x${width === 1280 ? 720 : 1080}:rgb8:10`;
  await chooseOption(
    section.getByLabel(`${label}来源`, { exact: true }),
    source,
  );
  await chooseOption(
    section.getByLabel(`${label}流配置`, { exact: true }),
    profile,
  );
  await section
    .getByRole("button", { name: `保存${label}绑定`, exact: true })
    .click();
  await wait(state, (s) =>
    s.camera_state.bindings.some(
      (b) =>
        b.role === role &&
        b.source_id === source &&
        b.color_profile_key === profile &&
        b.streaming,
    ),
  );
}
async function act(rig, name, predicate) {
  console.log("UI", rig, name);
  const c = card(rig === "external" ? "外部双目标定" : "腕部双目标定");
  await c.getByRole("button", { name, exact: true }).click();
  await wait(
    stereo,
    (s) => !s[rig].busy && (s[rig].original_error || predicate(s[rig])),
  );
  const result = (await stereo())[rig];
  expect(result.original_error, result.message).toBeFalsy();
  await expect(c.getByRole("status")).toHaveText(result.message);
  return result;
}
async function pose(joints) {
  const before = requests.length;
  for (let i = 0; i < 6; i++) {
    // Exercise the slider's DOM input event and React handler, not app state.
    await motion
      .getByRole("slider")
      .nth(i)
      .evaluate(
        (el, value) => {
          Object.getOwnPropertyDescriptor(
            HTMLInputElement.prototype,
            "value",
          ).set.call(el, String(value));
          el.dispatchEvent(new Event("input", { bubbles: true }));
          el.dispatchEvent(new Event("change", { bubbles: true }));
        },
        (joints[i] * Math.PI) / 180,
      );
  }
  await motion.getByRole("button", { name: "执行目标", exact: true }).click();
  await wait(
    async () => requests.slice(before),
    (rs) => rs.some((r) => r.path === "/api/motion/request"),
  );
  const id = requests
    .slice(before)
    .find((r) => r.path === "/api/motion/request").body.request_id;
  await wait(
    () => state("motion"),
    (s) =>
      s.motion_state.latest_motion?.request_id === id &&
      ["succeeded", "failed", "cancelled"].includes(
        s.motion_state.latest_motion.state,
      ),
  );
  const s = await state("motion");
  expect(
    s.motion_state.latest_motion.state,
    JSON.stringify(s.motion_state.latest_motion),
  ).toBe("succeeded");
  expect(s.arm_state.feedback_source).toBe("software");
  s.arm_state.joints_rad
    .slice(0, 6)
    .forEach((v, i) => expect(v).toBeCloseTo((joints[i] * Math.PI) / 180, 4));
}
try {
  for (const p of [page, execution, motion])
    p.on("request", (r) => {
      if (r.method() === "POST")
        requests.push({
          path: new URL(r.url()).pathname,
          body: r.postDataJSON(),
        });
    });
  await page.goto(`${base}/perception/`);
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常", {
    timeout: 60000,
  });
  await writeFile(
    resolve(output, "before.json"),
    JSON.stringify({
      camera: await state(),
      execution: await state("arm-execution"),
    }),
  );
  await execution.goto(`${base}/arm-execution/`);
  await expand(execution, "执行连接");
  await chooseOption(
    execution.getByLabel("执行模式", { exact: true }),
    "software",
  );
  const apply = execution.getByRole("button", {
    name: "应用执行模式",
    exact: true,
  });
  if (await apply.isVisible()) await apply.click();
  await wait(
    () => state("arm-execution"),
    (s) =>
      s.transport_state.selected_endpoint === null &&
      s.arm_state.feedback_source === "software",
  );
  await motion.goto(`${base}/motion/`);
  await cameraAction("刷新相机列表");
  for (const role of Object.keys(labels)) {
    if (verifyOnly) {
      // Do not re-save bindings: verify that the service restored them itself.
      await wait(state, (s) =>
        s.camera_state.bindings.some(
          (b) =>
            b.role === role &&
            b.streaming &&
            b.source_id?.startsWith("replay:"),
        ),
      );
    } else await bind(role, "scene", role.startsWith("external") ? 1280 : 1920);
  }
  for (const label of Object.values(labels)) {
    const preview = page.getByLabel(`${label}浮动窗口`, { exact: true });
    await expect(preview).toBeVisible();
    const fold = preview.getByRole("button", { name: "收起", exact: true });
    if (await fold.isVisible()) await fold.click();
  }
  for (const rig of ["external", "wrist"]) {
    const width = rig === "external" ? 1280 : 1920;
    const title = rig === "external" ? "外部双目标定" : "腕部双目标定";
    await expand(page, title);
    if (!verifyOnly) {
      await act(rig, "重新标定", (s) => s.sample_count === 0 && !s.candidate);
      for (let i = 1; i <= 24; i++) {
        const frame = `board-${String(i).padStart(2, "0")}`;
        await bind(rig, frame, width);
        await bind(`${rig}_right`, frame, width);
        await act(rig, "采集双目样本", (s) => s.sample_count === i);
      }
      await act(rig, "求解双目参数", (s) => !!s.candidate);
      await act(rig, "保存并应用标定", (s) => !!s.applied && !s.candidate);
      for (let i = 0; i < truth.handeye.views.length; i++) {
        await pose(truth.handeye.views[i].joints_deg);
        const frame = `handeye-${String(i + 1).padStart(2, "0")}`;
        await bind(rig, frame, width);
        await bind(`${rig}_right`, frame, width);
        await act(rig, "采集外参样本", (s) => s.pose_sample_count === i + 1);
      }
      await act(rig, "求解机械臂外参", (s) => !!s.candidate?.camera_pose);
      await act(
        rig,
        "保存并应用标定",
        (s) => !!s.applied?.camera_pose && !s.candidate,
      );
    }
    const result = (await stereo())[rig];
    await expect(
      card(title).getByText("上次应用结果", { exact: true }),
    ).toBeVisible();
    const saved = result.applied;
    const baselineError = Math.abs(
      Math.hypot(...saved.optics.left_to_right.position_m) -
        truth.rigs[rig].baseline_m,
    );
    const oraclePose =
      truth.handeye[rig === "external" ? "camera_in_base" : "camera_in_tool"];
    const positionError = Math.hypot(
      ...saved.camera_pose.position_m.map((v, i) => v - oraclePose[i][3]),
    );
    expect(baselineError, `${rig} baseline error (m)`).toBeLessThan(0.001);
    expect(positionError, `${rig} hand-eye position error (m)`).toBeLessThan(
      0.001,
    );
    reports.push({ rig, baselineError, positionError });
    reports.push({ rig, result });
    await card(title).screenshot({
      path: resolve(output, `${rig}-calibration.png`),
    });
    if (!verifyOnly) {
      await bind(rig, "scene", width);
      await bind(`${rig}_right`, "scene", width);
    }
    const responsePromise = page.waitForResponse((r) =>
      r.url().includes(`/api/camera/stereo/depth?rig=${rig}`),
    );
    await card(title)
      .getByRole("button", { name: "检查深度", exact: true })
      .click();
    const response = await responsePromise;
    const depth = await response.json();
    expect(response.ok(), JSON.stringify(depth)).toBe(true);
    expect(depth.valid_pixels).toBeGreaterThan(0);
    expect(depth.point_in_base_m).toHaveLength(3);
    expect(
      Math.abs(depth.point_in_rectified_camera_m[2] - 0.5),
      `${rig} scene centre depth error (m)`,
    ).toBeLessThan(0.001);
    reports.push({ rig, depth });
    await expect(
      card(title).getByText("有效深度像素", { exact: true }),
    ).toBeVisible();
    await cameraAction("刷新相机列表");
    await chooseOption(
      page.getByLabel("相机来源", { exact: true }),
      `stereo:${rig}`,
    );
    await page.getByRole("spinbutton", { name: "上送频率" }).fill("1");
    await cameraAction("保存并启用");
    await wait(
      state,
      (s) =>
        s.camera_state.streaming &&
        s.camera_state.selected_source_id === `stereo:${rig}` &&
        s.perception_state.camera_calibration?.source_id === `stereo:${rig}`,
    );
    await cameraAction("刷新图像");
    await expand(page, "深度图");
    const img = page.getByRole("img", { name: "深度图", exact: true });
    await expect
      .poll(() => img.evaluate((i) => i.complete && i.naturalWidth > 0))
      .toBe(true);
    await img.screenshot({ path: resolve(output, `${rig}-depth.png`) });
    if (verifyOnly) {
      // A normal user-drawn region must reach the existing scene reconstruction.
      for (const preview of await page
        .locator(".floating-camera-monitor")
        .all()) {
        const fold = preview.getByRole("button", { name: "收起", exact: true });
        if (await fold.isVisible()) await fold.click();
      }
      const sequence = (await state()).perception_state
        .last_segmentation_sequence;
      await page
        .getByRole("button", { name: "载入新分割帧", exact: true })
        .click();
      await wait(
        state,
        (s) => s.perception_state.last_segmentation_sequence > sequence,
      );
      await expand(page, "手动分割");
      await page
        .getByLabel("新标注名称", { exact: true })
        .fill(`${rig}-depth-patch`);
      const editor = page.getByRole("img", {
        name: "手动分割框选区域",
        exact: true,
      });
      await expect(editor).toHaveAttribute("aria-busy", "false");
      await editor.scrollIntoViewIfNeeded();
      const box = await editor.boundingBox();
      await page.mouse.move(
        box.x + box.width * 0.48,
        box.y + box.height * 0.48,
      );
      await page.mouse.down();
      await page.mouse.move(
        box.x + box.width * 0.52,
        box.y + box.height * 0.52,
        { steps: 4 },
      );
      await page.mouse.up();
      await expect(page.getByLabel("标注 1 名称", { exact: true })).toHaveValue(
        `${rig}-depth-patch`,
      );
      await page
        .getByRole("button", { name: "应用手动标注", exact: true })
        .click();
      await wait(state, (s) =>
        s.perception_state.instances?.some(
          (v) => v.label === `${rig}-depth-patch`,
        ),
      );
      await page.getByRole("button", { name: "三维定位", exact: true }).click();
      await wait(state, (s) =>
        s.world_scene?.objects.some((v) => v.label === `${rig}-depth-patch`),
      );
      const scene = (await state()).world_scene;
      expect(scene.frame_id).toBe("base_link");
      const object = scene.objects.find(
        (v) => v.label === `${rig}-depth-patch`,
      );
      const error = Math.hypot(
        ...object.pose.position_m.map((v, i) => v - depth.point_in_base_m[i]),
      );
      // Region centroid vs a rounded centre pixel: allow their different sampling locations.
      expect(
        error,
        `${rig} reconstructed centroid versus depth tool point (m)`,
      ).toBeLessThan(0.003);
      reports.push({
        rig,
        reconstructedObject: object,
        depthPointDistanceM: error,
      });
    }
    await cameraAction("停用相机");
  }
  if (verifyOnly) {
    // Visible errors must agree with failed depth requests; do not use old frames.
    const c = card("腕部双目标定");
    const checkError = async (pattern) => {
      const response = page.waitForResponse((r) =>
        r.url().includes("/api/camera/stereo/depth?rig=wrist"),
      );
      await c.getByRole("button", { name: "检查深度", exact: true }).click();
      const received = await response;
      expect(received.ok()).toBe(false);
      expect((await received.json()).original_error).toMatch(pattern);
      await expect(c.getByText(pattern)).toBeVisible();
      await expect(c.getByText("有效深度像素", { exact: true })).toHaveCount(0);
    };
    await bind("wrist_right", "scene", 1280);
    await checkError(/相同图像尺寸/);
    await bind("wrist_right", "scene", 1920);
    const binding = page.getByRole("region", {
      name: `${labels.wrist_right}绑定`,
      exact: true,
    });
    await binding
      .getByRole("button", {
        name: `关闭${labels.wrist_right}采集`,
        exact: true,
      })
      .click();
    await wait(state, (s) =>
      s.camera_state.bindings.some(
        (b) => b.role === "wrist_right" && !b.streaming,
      ),
    );
    await checkError(/绑定并开启同组左右相机/);
    await binding
      .getByRole("button", {
        name: `开启${labels.wrist_right}采集`,
        exact: true,
      })
      .click();
    await wait(state, (s) =>
      s.camera_state.bindings.some(
        (b) => b.role === "wrist_right" && b.streaming,
      ),
    );
    const response = page.waitForResponse((r) =>
      r.url().includes("/api/camera/stereo/depth?rig=wrist"),
    );
    await c.getByRole("button", { name: "检查深度", exact: true }).click();
    expect((await response).ok()).toBe(true);
    reports.push({
      negativeChecks: ["mismatched dimensions", "stopped peer"],
      restored: true,
    });
    // Exercise image-time feedback sampling on the restarted build as well.
    await pose(truth.handeye.views[0].joints_deg);
    for (const rig of ["external", "wrist"]) {
      const width = rig === "external" ? 1280 : 1920;
      await bind(rig, "handeye-01", width);
      await bind(`${rig}_right`, "handeye-01", width);
      await act(rig, "采集外参样本", (s) => s.pose_sample_count === 1);
      await act(
        rig,
        "重新标定",
        (s) => s.pose_sample_count === 0 && !!s.applied?.camera_pose,
      );
      await bind(rig, "scene", width);
      await bind(`${rig}_right`, "scene", width);
    }
  }
  await page.reload();
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
  for (const rig of ["external", "wrist"]) {
    await expand(page, rig === "external" ? "外部双目标定" : "腕部双目标定");
    expect((await stereo())[rig].applied.camera_pose).toBeTruthy();
    await expect(
      card(rig === "external" ? "外部双目标定" : "腕部双目标定").getByText(
        "上次应用结果",
        { exact: true },
      ),
    ).toBeVisible();
  }
  await pose([0, 0, -60, 60, 0, 0]);
  await page.screenshot({ path: resolve(output, "page.png"), fullPage: true });
  console.log(
    "PASS: four source bindings, previews, both stereo + hand-eye calibrations, metric depth, normal RGB-D pipeline, reload persistence",
  );
} finally {
  await writeFile(
    resolve(output, "report.json"),
    JSON.stringify({ reports, requests }, null, 2),
  );
  await page
    .screenshot({ path: resolve(output, "last-page.png"), fullPage: true })
    .catch(() => {});
  await browser.close();
}
