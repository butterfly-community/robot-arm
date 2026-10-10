// Explicitly connect the user's selected real serial endpoint through the UI.
// Does not command joints or the gripper.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir } from "node:fs/promises";
import { resolve, join } from "node:path";
const [endpoint, directory] = process.argv.slice(2);
if (!endpoint || !directory)
  throw Error("Usage: connect-execution.mjs ENDPOINT OUTPUT");
const output = resolve(directory);
process.env.TMPDIR = join(output, "browser-temp");
await mkdir(process.env.TMPDIR, { recursive: true });
const browser = await chromium.launch({ headless: true });
try {
  const page = await browser.newPage();
  let transport;
  page.on("websocket", (socket) =>
    socket.on("framereceived", (message) => {
      if (typeof message.payload !== "string") return;
      const value = JSON.parse(message.payload).values?.transport_state;
      if (value) transport = value;
    }),
  );
  await page.goto(
    (process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765") +
      "/arm-execution/",
  );
  await expect.poll(() => Boolean(transport), { timeout: 60000 }).toBe(true);
  const toggle = page.getByRole("button", { name: /执行连接/ });
  if ((await toggle.getAttribute("aria-expanded")) === "false")
    await toggle.click();
  if (!transport.connected) {
    await page.getByLabel("执行模式", { exact: true }).selectOption("hardware");
    await page
      .getByLabel("串口路径（可手动输入）", { exact: true })
      .fill(endpoint);
    await page.getByRole("button", { name: "连接真机", exact: true }).click();
  }
  await expect.poll(() => transport.connected, { timeout: 30000 }).toBe(true);
  console.log(
    JSON.stringify({
      connected: transport.connected,
      endpoint: transport.selected_endpoint,
      feedback: transport.feedback_summary,
      error: transport.last_error,
    }),
  );
  await page.screenshot({
    path: join(output, "execution.png"),
    fullPage: true,
  });
} finally {
  await browser.close();
}
