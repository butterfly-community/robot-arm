// Real page controls only; use discover before choosing device keys and profiles.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, writeFile } from "node:fs/promises";
import { resolve, join } from "node:path";
const [mode, directory, role, source, profile] = process.argv.slice(2);
if (
  !["discover", "bind", "unbind", "on", "off", "cycle"].includes(mode) ||
  !directory
)
  throw Error(
    "Usage: camera-bindings.mjs discover|bind|unbind|on|off|cycle OUTPUT [external|wrist SOURCE PROFILE]",
  );
const output = resolve(directory);
process.env.TMPDIR = join(output, "browser-temp");
await mkdir(process.env.TMPDIR, { recursive: true });
const browser = await chromium.launch({ headless: true });
try {
  const page = await browser.newPage({
    viewport: { width: 1440, height: 1000 },
  });
  await page.goto(
    (process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765") +
      "/perception/",
  );
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
  await page.getByRole("button", { name: "刷新相机列表", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "刷新相机列表", exact: true }),
  ).toBeEnabled();
  const label = role === "wrist" ? "腕部摄像头" : "外部摄像头";
  const section = page.getByRole("region", {
    name: `${label}绑定`,
    exact: true,
  });
  if (mode === "cycle") {
    const state = async () =>
      page.evaluate(
        async () =>
          (await (await fetch("/api/perception/state")).json()).values
            .camera_state.bindings,
      );
    const saved = await state();
    const titles = { external: "外部摄像头", wrist: "腕部摄像头" };
    await expect(page.getByText(/共享 USB 带宽不足时/)).toBeVisible();
    for (let round = 0; round < 2; round++) {
      for (const target of ["wrist", "external"]) {
        for (const binding of await state()) {
          if (!binding.streaming) continue;
          const name = titles[binding.role];
          const region = page.getByRole("region", {
            name: `${name}绑定`,
            exact: true,
          });
          await region
            .getByRole("button", { name: `关闭${name}采集`, exact: true })
            .click();
          await expect(
            region.getByText("采集已关闭，绑定与分辨率保留", { exact: true }),
          ).toBeVisible();
        }
        const name = titles[target];
        const region = page.getByRole("region", {
          name: `${name}绑定`,
          exact: true,
        });
        const requestedAt = Date.now();
        await region
          .getByRole("button", { name: `开启${name}采集`, exact: true })
          .click();
        await expect(region.getByText("正在采集", { exact: true })).toBeVisible(
          { timeout: 30000 },
        );
        const bindings = await state();
        for (const previous of saved) {
          expect(bindings.find((b) => b.role === previous.role)).toMatchObject({
            source_id: previous.source_id,
            color_profile_key: previous.color_profile_key,
          });
        }
        const current = bindings.find((b) => b.role === target);
        expect(current.last_frame_time_ns / 1e6).toBeGreaterThan(requestedAt);
        expect(bindings.filter((b) => b.streaming).map((b) => b.role)).toEqual([
          target,
        ]);
        const preview = page.getByLabel(`${name}浮动窗口`, { exact: true });
        const expand = preview.getByRole("button", {
          name: "展开",
          exact: true,
        });
        if (await expand.isVisible()) await expand.click();
        const canvas = preview.locator("canvas");
        await expect
          .poll(() => canvas.evaluate((c) => c.width))
          .toBe(current.frame.width);
        const pixels = await canvas.evaluate(
          (c) => c.toDataURL("image/png").split(",")[1],
        );
        await writeFile(
          join(output, `${round}-${target}.png`),
          Buffer.from(pixels, "base64"),
        );
        console.log("SWITCH_PASS", JSON.stringify(current));
      }
    }
    await page.reload();
    await expect(
      page
        .getByRole("region", { name: "腕部摄像头绑定", exact: true })
        .getByText("采集已关闭，绑定与分辨率保留", { exact: true }),
    ).toBeVisible();
    console.log("RELOAD_PRESERVED_CAPTURE_SWITCH");
  }
  if (mode === "on" || mode === "off") {
    await section
      .getByRole("button", {
        name: `${mode === "on" ? "开启" : "关闭"}${label}采集`,
        exact: true,
      })
      .click();
    await expect(
      section.getByText(
        mode === "on" ? "正在采集" : "采集已关闭，绑定与分辨率保留",
        { exact: true },
      ),
    ).toBeVisible({ timeout: 30000 });
  }
  if (mode === "unbind") {
    await section
      .getByRole("button", { name: "解除绑定", exact: true })
      .click();
    await expect(
      section.getByLabel(`${label}来源`, { exact: true }),
    ).toHaveValue("");
  }
  if (mode === "bind") {
    if (!["external", "wrist"].includes(role) || !source || !profile)
      throw Error("missing binding arguments");
    await section
      .getByLabel(`${label}来源`, { exact: true })
      .selectOption(source);
    await section
      .getByLabel(`${label}流配置`, { exact: true })
      .selectOption(profile);
    await section
      .getByRole("button", { name: `保存${label}绑定`, exact: true })
      .click();
    await expect(section.getByText("正在采集", { exact: true })).toBeVisible({
      timeout: 30000,
    });
  }
  if (mode === "bind" || mode === "on") {
    const preview = page.getByLabel(`${label}浮动窗口`, { exact: true });
    await expect(preview).toBeVisible();
    const expand = preview.getByRole("button", { name: "展开", exact: true });
    if (await expand.isVisible()) await expand.click();
    await expect
      .poll(() => preview.locator("canvas").evaluate((c) => c.width))
      .toBeGreaterThan(300);
    await preview
      .locator("canvas")
      .screenshot({ path: join(output, `${role}.png`) });
    const pixels = await preview
      .locator("canvas")
      .evaluate((c) => c.toDataURL("image/png").split(",")[1]);
    await writeFile(
      join(output, `${role}-native.png`),
      Buffer.from(pixels, "base64"),
    );
  }
  // Read the same state the page consumes, only after the UI operation ends.
  const s = await page.evaluate(
    async () =>
      (await (await fetch("/api/perception/state")).json()).values.camera_state,
  );
  console.log(
    JSON.stringify(
      {
        bindings: s.bindings,
        sources: s.available_sources.filter((s) => s.driver_id === "v4l2"),
        error: s.original_error,
      },
      null,
      2,
    ),
  );
  await page.screenshot({ path: join(output, "page.png"), fullPage: true });
} finally {
  await browser.close();
}
