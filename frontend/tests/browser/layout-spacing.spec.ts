import { expect, test } from "@playwright/test";
import {
  inspectLayoutSpacing,
  setLayoutDisclosureState,
} from "../../../tools/diagnostics/layout-spacing.mjs";

// Only unfold controls; never change configuration or send motion commands.
for (const name of [
  "tracking",
  "spatial",
  "perception",
  "motion",
  "arm-execution",
]) {
  test(`${name}: painted borders stay apart across widths and fold states`, async ({
    page,
  }) => {
    test.setTimeout(90000);
    await page.route("**/api/**", (route) =>
      ["GET", "HEAD"].includes(route.request().method())
        ? route.fallback()
        : route.abort(),
    );
    await page.goto(`/${name}/`);
    await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
    for (const expanded of [false, true]) {
      await setLayoutDisclosureState(page, expanded);
      for (const width of [1600, 1024, 390]) {
        await page.setViewportSize({ width, height: 1100 });
        await expect
          .poll(
            async () => {
              const report = await page.evaluate(inspectLayoutSpacing);
              return { overflow: report.overflow, touching: report.touching };
            },
            { message: `${name} / ${width} / expanded=${expanded}` },
          )
          .toEqual({ overflow: false, touching: [] });
      }
    }
    if (name === "perception") {
      const depth = page
        .getByText("深度图", { exact: true })
        .locator("xpath=ancestor::div[contains(@class, 'disclosure')][1]");
      const gap = await depth.evaluate(
        (e) =>
          e.getBoundingClientRect().top -
          e.previousElementSibling!.getBoundingClientRect().bottom,
      );
      expect(gap).toBeGreaterThanOrEqual(20);
    }
  });
}
