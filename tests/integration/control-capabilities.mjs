// Real deployed browser path: no POST interception, API injection or fake feedback.
// Run only with authorization to move the connected arm and an empty gripper.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, writeFile, readFile } from "node:fs/promises";
import { resolve, join } from "node:path";

const [mode = "motion", destination = "temp/control-capabilities/browser"] =
  process.argv.slice(2);
if (!["motion", "gripper", "history"].includes(mode))
  throw Error("Use motion, gripper or history");
const output = resolve(destination);
await mkdir(output, { recursive: true });
process.env.TMPDIR = join(output, "browser-temp");
await mkdir(process.env.TMPDIR, { recursive: true });
const browser = await chromium.launch({ headless: true });
const records = [],
  posts = [],
  errors = [];
try {
  const page = await browser.newPage({
    viewport: { width: 1440, height: 1000 },
  });
  const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
  let values, id;
  page.on("pageerror", (e) => errors.push(e.message));
  page.on("websocket", (socket) =>
    socket.on("framereceived", (frame) => {
      if (typeof frame.payload !== "string") return;
      const v = JSON.parse(frame.payload).values;
      if (v?.motion_state) values = v;
    }),
  );
  page.on("request", (request) => {
    if (request.method() !== "POST") return;
    const path = new URL(request.url()).pathname,
      body = request.postDataJSON();
    posts.push({ path, body });
    if (
      [
        "/api/motion/request",
        "/api/motion/actuator",
        "/api/motion/prepare-relative",
      ].includes(path)
    )
      id = body.request_id;
  });
  page.on("response", async (response) => {
    if (
      new URL(response.url()).pathname.startsWith("/api/requests/") &&
      response.ok()
    ) {
      try {
        records.push(await response.json());
      } catch {
        /* page reload */
      }
    }
  });
  async function expand(title) {
    const button = page.getByRole("button", {
      name: new RegExp(`^${title}(?: 英文：|$)`),
    });
    await expect(button).toBeVisible();
    if ((await button.getAttribute("aria-expanded")) !== "true")
      await button.click();
  }
  async function ready() {
    await expect(page.locator(".topbar-state")).toHaveText("实时连接正常", {
      timeout: 60000,
    });
    await expect
      .poll(() => Boolean(values?.motion_state.current_tool_pose), {
        timeout: 60000,
      })
      .toBe(true);
  }
  async function result(requestId, state = "succeeded") {
    await expand("动作结果查询");
    await expand("请求结果");
    await page.getByLabel("查询请求编号").fill(requestId);
    await page
      .getByRole("button", { name: "查看请求结果", exact: true })
      .click();
    await expect
      .poll(
        () => records.findLast((r) => r.request_id === requestId)?.terminal,
        { timeout: 180000 },
      )
      .toBe(true);
    const record = records.findLast((r) => r.request_id === requestId);
    expect(record.state, JSON.stringify(record)).toBe(state);
    return record;
  }
  await page.goto(`${base}/motion/`);
  await ready();
  if (mode === "history") {
    const saved = JSON.parse(
      await readFile(join(output, "results.json"), "utf8"),
    );
    for (const record of saved.results)
      await result(record.request_id, record.state);
    expect(posts).toHaveLength(0);
  } else {
    await expand("控制模式 / 规划");
    await page.getByRole("button", { name: "工作位", exact: true }).click();
    await expect.poll(() => id).toBeTruthy();
    const results = [await result(id)];
    const initial = structuredClone(values.motion_state.current_tool_pose);
    if (mode === "motion") {
      await expand("TCP 定点运动");
      const card = page.locator("section").filter({
        has: page.getByRole("button", { name: "TCP 定点运动", exact: true }),
      });
      async function tcp(
        relative,
        frame,
        position,
        quaternion,
        refresh = false,
        expected = "succeeded",
      ) {
        await card
          .getByLabel("目标方式")
          .selectOption(relative ? "relative" : "absolute");
        await card.getByLabel("目标坐标系").selectOption(frame);
        for (const [i, name] of ["X / m", "Y / m", "Z / m"].entries())
          await card
            .getByLabel(name, { exact: true })
            .fill(String(position[i]));
        for (const [i, name] of ["qx", "qy", "qz", "qw"].entries())
          await card
            .getByLabel(name, { exact: true })
            .fill(String(quaternion[i]));
        const oldId = id;
        await card
          .getByRole("button", { name: "执行 TCP 目标", exact: true })
          .click();
        await expect.poll(() => id && id !== oldId).toBe(true);
        const requestId = id;
        if (refresh) {
          await page.reload();
          await ready();
        }
        results.push(await result(requestId, expected));
        await writeFile(
          join(output, `feedback-${results.length}.json`),
          JSON.stringify(values),
        );
      }
      const model = values.robot_model_info;
      // The working pose has J2 at its limit; measured feedback may lie just
      // outside it. First move into the workspace, then test absolute targets.
      await tcp(true, model.base_frame, [0, 0, 0.02], [0, 0, 0, 1], true);
      const interior = structuredClone(values.motion_state.current_tool_pose);
      await tcp(
        false,
        model.base_frame,
        interior.position_m,
        interior.orientation_xyzw,
      );
      const halfAngle = Math.PI / 180;
      await tcp(
        true,
        model.tcp_frame,
        [0, 0, 0],
        [0, 0, Math.sin(halfAngle), Math.cos(halfAngle)],
      );
      await tcp(
        false,
        model.base_frame,
        interior.position_m,
        interior.orientation_xyzw,
      );
      // Invalid quaternion exercises the normal form and rejects before motion.
      await tcp(
        true,
        model.base_frame,
        [0, 0, 0],
        [0, 0, 0, 0],
        false,
        "failed",
      );
      await card.getByRole("button", { name: "恢复当前 TCP" }).click();
    }
    await expand("关节与夹爪状态 / 手动目标");
    const grip = page.getByRole("slider").last();
    for (const angle of [10, 0]) {
      await grip.press("Home");
      if (angle) for (let i = 0; i < 10; i++) await grip.press("ArrowRight");
      const previous = id;
      await page
        .getByRole("button", { name: "仅执行夹爪目标", exact: true })
        .click();
      await expect.poll(() => id !== previous).toBe(true);
      results.push(await result(id));
    }
    const beforeReturn = id;
    await page.getByRole("button", { name: "工作位", exact: true }).click();
    await expect.poll(() => id !== beforeReturn).toBe(true);
    results.push(await result(id));
    await writeFile(
      join(output, "results.json"),
      JSON.stringify({ results, posts, errors, initial }, null, 2),
    );
    console.log(
      JSON.stringify({
        results: results.map(({ request_id, state }) => ({
          request_id,
          state,
        })),
        errors,
      }),
    );
  }
  expect(errors).toEqual([]);
  await page.screenshot({ path: join(output, `${mode}.png`), fullPage: true });
} finally {
  await writeFile(
    join(output, `${mode}-observed.json`),
    JSON.stringify({ records, posts, errors }, null, 2),
  );
  await browser.close();
}
