// Read-only diagnostics; never substitutes for camera-bindings page acceptance.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, writeFile } from "node:fs/promises";
import { resolve } from "node:path";
const output = resolve(process.argv[2] ?? "temp/browser-video-diagnostics");
await mkdir(output, { recursive: true });
// Chromium's Unix singleton socket must fit within sockaddr_un.sun_path.
process.env.TMPDIR = resolve("temp/b");
await mkdir(process.env.TMPDIR, { recursive: true });
const browser = await chromium.launch({
  ...(process.env.CAMERA_BROWSER_CHANNEL
    ? { channel: process.env.CAMERA_BROWSER_CHANNEL }
    : {}),
});
try {
  const system = await browser.newBrowserCDPSession();
  const info = await system.send("SystemInfo.getInfo");
  const page = await browser.newPage({
    viewport: { width: 1440, height: 1000 },
  });
  const media = await page.context().newCDPSession(page);
  const events = [];
  await media.send("Media.enable");
  for (const event of [
    "playerPropertiesChanged",
    "playerErrorsRaised",
    "playerEventsAdded",
  ])
    media.on(`Media.${event}`, (data) => events.push({ event, data }));
  const measureAnimation = () =>
    page.evaluate(async () => {
      let frames = 0,
        callback;
      const start = performance.now();
      const next = () => {
        frames++;
        callback = requestAnimationFrame(next);
      };
      callback = requestAnimationFrame(next);
      await new Promise((resolve) => setTimeout(resolve, 5000));
      cancelAnimationFrame(callback);
      return (frames * 1000) / (performance.now() - start);
    });
  const emptyPageFps = await measureAnimation();
  await page.goto("http://192.168.100.10:8765/perception/");
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常", {
    timeout: 60000,
  });
  const preview = page.getByLabel("外部摄像头浮动窗口", { exact: true });
  const expand = preview.getByRole("button", { name: "展开", exact: true });
  if (await expand.isVisible()) await expand.click();
  await expect
    .poll(() => preview.locator("video").evaluate((v) => v.readyState), {
      timeout: 30000,
    })
    .toBeGreaterThanOrEqual(2);
  const videoPageFps = await measureAnimation();
  // One-factor rendering diagnostic; no frame, rate or resolution changes.
  await page.addStyleTag({
    content:
      ".floating-camera-monitor { backdrop-filter: none !important; background: #101419 !important; }",
  });
  const withoutBlurFps = await measureAnimation();
  await writeFile(
    resolve(output, "report.json"),
    JSON.stringify(
      { emptyPageFps, videoPageFps, withoutBlurFps, info, events },
      null,
      2,
    ),
  );
  console.log(
    JSON.stringify(
      {
        emptyPageFps,
        videoPageFps,
        withoutBlurFps,
        gpu: info.gpu.auxAttributes.displayType,
        events,
      },
      null,
      2,
    ),
  );
} finally {
  await browser.close();
}
