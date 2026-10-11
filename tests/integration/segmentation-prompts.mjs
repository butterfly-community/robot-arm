// Compare text prompts through the normal frozen-frame editor, without saving
// over the user's visual examples. No capture, motion or internal API writes.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir } from "node:fs/promises";
import { resolve } from "node:path";
process.env.TMPDIR = resolve("temp/segmentation-prompts/browser");
await mkdir(process.env.TMPDIR, { recursive: true });
const browser = await chromium.launch();
try {
  const page = await browser.newPage();
  const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
  await page.goto(`${base}/perception/`);
  const section = page.getByRole("button", { name: /^提示词分割/ });
  if ((await section.getAttribute("aria-expanded")) === "false")
    await section.click();
  await page.getByLabel("提示方式", { exact: true }).selectOption("text");
  for (const label of process.argv.slice(2)) {
    await page
      .getByLabel("识别与分割提示词", { exact: true })
      .fill(`red cube, ${label}`);
    await page.getByLabel("放置区域角色", { exact: true }).fill(label);
    const response = page.waitForResponse(
      (r) =>
        r.url().endsWith("/api/perception/request") &&
        r.request().method() === "POST",
    );
    await page
      .getByRole("button", { name: "运行提示词分割", exact: true })
      .click();
    const result = await response;
    expect(result.ok()).toBe(true);
    const state = (
      await (await page.request.get(`${base}/api/perception/state`)).json()
    ).values.perception_state;
    console.log(
      JSON.stringify({
        label,
        state: state.task_state,
        error: state.original_error,
        instances: state.instances.map((i) => ({
          label: i.label,
          bbox: i.bbox_xyxy,
        })),
      }),
    );
    if (state.instances.some((i) => i.label === label)) break;
  }
} finally {
  await browser.close();
}
