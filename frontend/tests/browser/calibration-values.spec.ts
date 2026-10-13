import { expect, test } from "@playwright/test";

// Saved results, real live telemetry, no mocked responses or calibration moves.
for (const width of [1600, 390]) {
  test(`calibration values scroll without clipping at ${width}px`, async ({
    page,
  }) => {
    await page.setViewportSize({ width, height: 1100 });
    await page.goto("/perception/");
    await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
    const toggle = page.getByText("标定详细数值", { exact: true }).first();
    await expect(toggle).toBeVisible();
    await toggle.click();
    const values = page
      .getByRole("region", { name: "标定详细数值", exact: true })
      .first();
    await values.scrollIntoViewIfNeeded();
    await expect(values).toBeVisible();
    const dimensions = await values.evaluate((el) => ({
      width: el.clientWidth,
      scroll: el.scrollWidth,
      overflow: getComputedStyle(el).overflowX,
    }));
    expect(dimensions.overflow).toBe("auto");
    expect(dimensions.scroll).toBeGreaterThan(dimensions.width);
    for (const value of await values.locator(".key-value strong").all()) {
      expect(
        await value.evaluate((el) => getComputedStyle(el).textOverflow),
      ).toBe("clip");
    }
    await values.hover();
    await page.mouse.wheel(600, 0);
    await expect
      .poll(() => values.evaluate((el) => el.scrollLeft))
      .toBeGreaterThan(0);
    await page.mouse.wheel(-600, 0);
    await expect.poll(() => values.evaluate((el) => el.scrollLeft)).toBe(0);
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth > innerWidth,
      ),
    ).toBe(false);
    await page.screenshot({
      path: test.info().outputPath("calibration-values.png"),
    });

    for (const title of ["外部双目标定", "腕部双目标定"]) {
      const card = page.locator("section.card").filter({
        has: page.locator(".card-title-text").getByText(title, { exact: true }),
      });
      const button = card.locator(".card-toggle");
      if ((await button.getAttribute("aria-expanded")) === "false")
        await button.click();
      const details = card.getByRole("button", {
        name: "完整标定参数",
        exact: true,
      });
      if ((await details.getAttribute("aria-expanded")) === "false")
        await details.click();
      const region = card.getByRole("region", {
        name: "完整标定参数",
        exact: true,
      });
      await region.hover();
      await page.mouse.wheel(600, 0);
      await expect
        .poll(() => region.evaluate((el) => el.scrollLeft))
        .toBeGreaterThan(0);
    }
  });
}
