import { expect, test, type Locator, type Page } from "@playwright/test";
import {
  chooseOption,
  openOptions,
  readOptions,
} from "../../../tests/integration/select-controls.mjs";

// No routes, response mocks or state injection. All choices use pointer events.
async function pointAt(page: Page, option: Locator, fraction: number) {
  await option.scrollIntoViewIfNeeded();
  const box = await option.boundingBox();
  expect(box).not.toBeNull();
  const point = {
    x: box!.x + box!.width / 2,
    y: box!.y + box!.height * fraction,
  };
  expect(
    await option.evaluate(
      (el, p) => el.contains(document.elementFromPoint(p.x, p.y)),
      point,
    ),
  ).toBe(true);
  await page.mouse.move(point.x, point.y);
  await expect(option).toHaveAttribute("data-highlighted", "");
  return point;
}

for (const path of ["tracking", "spatial", "motion", "arm-execution"]) {
  test(`live ${path} menus use the same visible pointer targets`, async ({
    page,
  }) => {
    test.setTimeout(60000);
    await page.goto(`/${path}/`);
    await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
    for (const toggle of await page
      .locator('.card-toggle[aria-expanded="false"]')
      .all())
      await toggle.click();
    if (path === "tracking")
      await page.locator("details.action-binding-item summary").first().click();
    let tested = 0;
    for (const trigger of await page.locator(".select-trigger").all()) {
      if (!(await trigger.isVisible()) || !(await trigger.isEnabled()))
        continue;
      const original = await trigger.getAttribute("data-value");
      const popup = await openOptions(trigger);
      const option = popup.locator(
        `[role="option"][data-value=${JSON.stringify(original)}]`,
      );
      if ((await option.count()) && (await option.isEnabled())) {
        const p = await pointAt(page, option, 0.5);
        await page.mouse.click(p.x, p.y);
        await expect(trigger).toHaveAttribute("data-value", original!);
        tested++;
      } else await page.keyboard.press("Escape");
    }
    expect(tested).toBeGreaterThan(0);
  });
}

for (const [width, zoom] of [
  [1600, 1],
  [1600, 1.25],
  [1024, 1.5],
  [390, 1],
]) {
  test(`pointer matches every profile at ${width}px and ${zoom} zoom`, async ({
    page,
  }) => {
    test.setTimeout(90000);
    await page.setViewportSize({ width, height: 1000 });
    await page.goto("/perception/");
    await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
    // Layout-only zoom: no business state is altered.
    await page.evaluate((z) => {
      document.documentElement.style.zoom = String(z);
    }, zoom);
    // Use the formal replay source as an unsaved form draft; no camera needed.
    await chooseOption(
      page.getByLabel("外部摄像头 1（左）来源", { exact: true }),
      "replay:external-left",
    );
    const trigger = page.getByLabel("外部摄像头 1（左）流配置", {
      exact: true,
    });
    const original = await trigger.getAttribute("data-value");
    const options = (await readOptions(trigger)).filter((o) => !o.disabled);
    expect(options.length).toBeGreaterThan(2);
    for (const [i, item] of options.entries()) {
      const popup = await openOptions(trigger);
      const option = popup.locator(
        `[role="option"][data-value=${JSON.stringify(item.value)}]`,
      );
      await pointAt(page, option, 0.15);
      const point = await pointAt(page, option, 0.85);
      // Cross several live telemetry updates before clicking the same position.
      if (i === 1) await page.waitForTimeout(2000);
      await expect(option).toHaveAttribute("data-highlighted", "");
      await page.mouse.click(point.x, point.y);
      await expect(trigger).toHaveAttribute("data-value", item.value!);
      await expect(trigger).toContainText(item.text!);
      await expect(page.getByRole("listbox")).toHaveCount(0);
    }
    await chooseOption(trigger, original);
    console.log(
      `Pointer alignment: ${options.length} options × 2 hit positions, width=${width}, zoom=${zoom}`,
    );
    await page.screenshot({
      path: test.info().outputPath("selected-profile.png"),
    });
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth > innerWidth,
      ),
    ).toBe(false);
  });
}

test("scrolling, disabled options, keyboard and cancellation", async ({
  page,
}) => {
  await page.goto("/perception/");
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
  const trigger = page.getByLabel("相机来源", { exact: true });
  const original = await trigger.getAttribute("data-value");
  let popup = await openOptions(trigger);
  const disabled = popup.locator('[role="option"][aria-disabled="true"]');
  if (await disabled.count()) {
    const option = disabled.first();
    await option.scrollIntoViewIfNeeded();
    const b = await option.boundingBox();
    await page.mouse.click(b!.x + b!.width / 2, b!.y + b!.height / 2);
    await expect(trigger).toHaveAttribute("data-value", original!);
    await expect(popup).toBeVisible();
  }
  await page.keyboard.press("Escape");
  await expect(trigger).toBeFocused();
  popup = await openOptions(trigger);
  await popup.hover();
  await page.mouse.wheel(0, 500);
  const last = popup
    .locator('[role="option"]:not([aria-disabled="true"])')
    .last();
  const target = await last.getAttribute("data-value");
  const point = await pointAt(page, last, 0.5);
  await page.mouse.click(point.x, point.y);
  await expect(trigger).toHaveAttribute("data-value", target!);
  await trigger.focus();
  await page.keyboard.press("Enter");
  await page.keyboard.press("Home");
  await page.keyboard.press("Enter");
  await expect(trigger).toHaveAttribute("data-value", "");
  await chooseOption(trigger, original);
});

test("menu follows its trigger during page scroll on a high-density display", async ({
  browser,
}) => {
  const context = await browser.newContext({
    viewport: { width: 1440, height: 1000 },
    deviceScaleFactor: 2,
  });
  try {
    const page = await context.newPage();
    await page.goto("/perception/");
    await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
    await chooseOption(
      page.getByLabel("外部摄像头 1（左）来源", { exact: true }),
      "replay:external-left",
    );
    const trigger = page.getByLabel("外部摄像头 1（左）流配置", {
      exact: true,
    });
    const original = await trigger.getAttribute("data-value");
    const popup = await openOptions(trigger);
    await page.mouse.move(1300, 500);
    await page.mouse.wheel(0, 100);
    await page.waitForTimeout(300);
    const option = popup.getByRole("option").nth(1);
    const value = await option.getAttribute("data-value");
    const point = await pointAt(page, option, 0.5);
    await page.mouse.click(point.x, point.y);
    await expect(trigger).toHaveAttribute("data-value", value!);
    await chooseOption(trigger, original);
  } finally {
    await context.close();
  }
});

test("mouse source selection saves, streams, reloads and restores through the page", async ({
  page,
}) => {
  test.setTimeout(90000);
  await page.goto("/perception/");
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
  const source = page.getByLabel("相机来源", { exact: true });
  // The test intentionally requires an idle, unselected depth source.
  await expect(source).toHaveAttribute("data-value", "");
  try {
    await chooseOption(source, "simulation:pick-place-scene");
    for (const [label, value] of [
      ["彩色流", "color:1280x720:rgb8:10"],
      ["深度流", "depth:1280x720:z16le:10"],
    ]) {
      await chooseOption(page.getByLabel(label, { exact: true }), value);
    }
    await page.getByRole("button", { name: "保存并启用", exact: true }).click();
    const monitor = page.getByLabel("深度相机彩色画面浮动窗口", {
      exact: true,
    });
    await expect(monitor).toBeVisible({ timeout: 30000 });
    const expand = monitor.getByRole("button", { name: "展开", exact: true });
    if (await expand.isVisible()) await expand.click();
    const video = monitor.locator("video");
    await expect
      .poll(() => video.evaluate((v: HTMLVideoElement) => v.videoWidth), {
        timeout: 30000,
      })
      .toBe(1280);
    await expect
      .poll(() => video.evaluate((v: HTMLVideoElement) => v.currentTime), {
        timeout: 30000,
      })
      .toBeGreaterThan(0);
    await page.reload();
    await expect(source).toHaveAttribute(
      "data-value",
      "simulation:pick-place-scene",
    );
    await expect(monitor).toBeVisible();
    await page.screenshot({
      path: test.info().outputPath("saved-preview.png"),
    });
  } finally {
    const unselected = page.waitForResponse(
      (response) =>
        response.url().endsWith("/api/perception/camera") &&
        response.request().method() === "POST",
    );
    await chooseOption(source, "");
    expect((await unselected).ok()).toBe(true);
    await page.reload();
    await expect(source).toHaveAttribute("data-value", "");
  }
});
