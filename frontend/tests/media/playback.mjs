// Real FFmpeg packets through the production server and React video component.
// This isolated media test does not replace deployed page / hardware acceptance.
import { chromium, expect } from "@playwright/test";
import { mkdirSync } from "node:fs";
import { fileURLToPath } from "node:url";
process.env.TMPDIR = fileURLToPath(
  new URL("../../../temp/playwright-browser", import.meta.url),
);
mkdirSync(process.env.TMPDIR, { recursive: true });
const browser = await chromium.launch();
try {
  const page = await browser.newPage();
  const errors = [];
  if (process.env.MEDIA_DEBUG) {
    const media = await page.context().newCDPSession(page);
    await media.send("Media.enable");
    media.on("Media.playerPropertiesChanged", (value) =>
      console.log(JSON.stringify(value)),
    );
  }
  page.on("pageerror", (e) => errors.push(e.message));
  page.on("console", (message) => {
    if (message.type() === "error") console.error(message.text());
  });
  await page.goto("http://192.168.100.10:38765/tests/media/index.html");
  for (const [role, width, channel] of [
    ["external", 1920, 1],
    ["depth", 1280, 0],
    ["wrist", 1280, 2],
  ]) {
    await page.getByLabel("相机角色").selectOption(role);
    const video = page.getByLabel("相机彩色视频");
    await expect
      .poll(() => video.evaluate((v) => v.readyState >= 2 && v.videoWidth), {
        timeout: 15000,
      })
      .toBe(width)
      .catch(async (error) => {
        console.error(await page.locator("body").innerText(), errors);
        console.error(
          await video.evaluate((v) => ({
            error: v.error?.message,
            network: v.networkState,
            ready: v.readyState,
            src: v.currentSrc,
          })),
        );
        throw error;
      });
    const before = await video.evaluate((v) => v.currentTime);
    await expect
      .poll(() => video.evaluate((v) => v.currentTime), { timeout: 15000 })
      .toBeGreaterThan(before + 1);
    const pixel = await video.evaluate((v) => {
      const c = document.createElement("canvas");
      c.width = v.videoWidth;
      c.height = v.videoHeight;
      c.getContext("2d").drawImage(v, 0, 0);
      return [...c.getContext("2d").getImageData(10, 10, 1, 1).data];
    });
    expect(pixel[channel]).toBeGreaterThan(170);
    expect(pixel[(channel + 1) % 3]).toBeLessThan(65);
    const expected = {
      external: [20, 210, 30],
      depth: [210, 30, 20],
      wrist: [20, 30, 210],
    }[role];
    console.log(JSON.stringify({ role, pixel, expected }));
    // Display conversion is not bit-exact RGB: Chromium's MP4 renderer uses
    // its own display color transform. Keep a separate display tolerance from
    // the tighter FFmpeg snapshot check below; never compare channel order only.
    expected.forEach((value, i) =>
      expect(Math.abs(pixel[i] - value)).toBeLessThan(12),
    );
    await expect(page.locator(".camera-video-rate")).toContainText("FPS 实测");
    const playbackRate = await page.locator(".camera-video-rate").innerText();
    const response = await page.request.get(
      `http://192.168.100.10:38765/snapshot?role=${role}`,
    );
    expect(response.status()).toBe(200);
    const metadata = JSON.parse(response.headers()["x-camera-frame"]);
    expect(metadata.video_codec).toBe("h264");
    expect(metadata.width).toBe(width);
    const snapshotPixel = await page.evaluate(async (role) => {
      const response = await fetch(`/snapshot?role=${role}`);
      const bitmap = await createImageBitmap(await response.blob());
      const canvas = document.createElement("canvas");
      canvas.width = bitmap.width;
      canvas.height = bitmap.height;
      canvas.getContext("2d").drawImage(bitmap, 0, 0);
      bitmap.close();
      return [...canvas.getContext("2d").getImageData(10, 10, 1, 1).data];
    }, role);
    expected.forEach((value, i) =>
      expect(Math.abs(snapshotPixel[i] - value)).toBeLessThan(8),
    );
    await page.getByRole("button", { name: "收起" }).click();
    await expect(video).toHaveCount(0);
    await page.getByRole("button", { name: "展开" }).click();
    await expect
      .poll(() => video.evaluate((v) => v.readyState >= 2), { timeout: 15000 })
      .toBe(true);
    console.log(
      JSON.stringify({
        role,
        width,
        pixel,
        snapshotPixel,
        playbackRate,
        snapshot: "passed",
        reconnect: "passed",
      }),
    );
  }
  // Wrist fixture changes stream id every four seconds. Playback must recover.
  await page.waitForTimeout(6000);
  await expect(page.locator(".camera-video-error")).toHaveCount(0);
  expect(errors).toEqual([]);
} finally {
  await browser.close();
}
