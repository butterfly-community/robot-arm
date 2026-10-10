// Source: execution page's existing "全部上力" and information-refresh buttons.
// Restores torque at the current pose; does not release torque or change limits.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, writeFile } from "node:fs/promises";
import { resolve, join } from "node:path";

const output = resolve(process.argv[2] ?? "temp/execution-hold");
process.env.TMPDIR = join(output, "browser-temp");
await mkdir(process.env.TMPDIR, { recursive: true });
const browser = await chromium.launch({ headless: true });
try {
  const page = await browser.newPage();
  let transport;
  const samples = [];
  page.on("websocket", (socket) =>
    socket.on("framereceived", (message) => {
      if (typeof message.payload !== "string") return;
      const next = JSON.parse(message.payload).values?.transport_state;
      if (!next) return;
      transport = next;
      samples.push({
        at: Date.now(),
        target: next.gripper_strength_percent,
        feedback: next.gripper_strength_feedback_percent,
        power: next.gripper_control_power_mw,
        gripper: next.gripper_feedback_telemetry,
        error: next.last_error,
      });
    }),
  );
  await page.goto(
    (process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765") +
      "/arm-execution/",
  );
  await expect.poll(() => transport?.connected, { timeout: 30000 }).toBe(true);
  const toggle = page.getByRole("button", { name: /执行连接/ });
  if ((await toggle.getAttribute("aria-expanded")) === "false")
    await toggle.click();
  const hold = page.getByRole("button", { name: "全部上力", exact: true });
  const request = page.waitForResponse(
    (r) =>
      r.request().method() === "POST" &&
      new URL(r.url()).pathname === "/api/arm-execution/parameters" &&
      r.request().postDataJSON()?.fields?.torque_mode === "hold",
  );
  await hold.click();
  const response = await request;
  expect(response.ok()).toBe(true);
  expect((await response.json()).original_error ?? null).toBeNull();
  await expect(hold).toBeEnabled({ timeout: 30000 });
  const refresh = page.getByRole("button", {
    name: "读取全部舵机信息",
    exact: true,
  });
  await refresh.click();
  await expect
    .poll(() => transport?.parameter_reading, { timeout: 10000 })
    .toBe(true);
  await expect
    .poll(() => transport?.parameter_reading, { timeout: 60000 })
    .toBe(false);
  await page.screenshot({
    path: join(output, "execution.png"),
    fullPage: true,
  });
  await writeFile(
    join(output, "result.json"),
    JSON.stringify({ transport, samples }, null, 2),
  );
  console.log(
    JSON.stringify({
      connected: transport.connected,
      error: transport.last_error,
      gripper: transport.gripper_feedback_telemetry,
      parameters: transport.parameter_values?.filter(
        (p) => p.actuator_key === "gripper",
      ),
    }),
  );
} finally {
  await browser.close();
}
