import { expect, test } from "@playwright/test";

// Human page layout and collapse interactions only; no inference or robot writes.
for (const width of [1600, 390]) {
  test(`camera row and unified two-column AI at ${width}px`, async ({
    page,
  }) => {
    await page.setViewportSize({ width, height: 1100 });
    await page.route("**/api/**", (route) =>
      route.request().method() === "POST" ? route.abort() : route.fallback(),
    );
    await page.goto("/perception/");
    await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
    const card = (title: string) =>
      page.locator("section.card").filter({
        has: page
          .locator(".card-title-text")
          .filter({ hasText: new RegExp(`^${title}$`) }),
      });
    const fold = (title: string) =>
      page
        .getByText(title, { exact: true })
        .locator("xpath=ancestor::div[contains(@class, 'disclosure')][1]");
    const camera = card("相机来源与配置"),
      calibration = card("相机外参标定"),
      ai = card("AI");
    await expect(page.locator(".perception-section-heading h2")).toHaveText([
      "相机与标定",
      "AI 与任务",
    ]);
    await expect(camera.getByText("深度图", { exact: true })).toBeVisible();
    await expect(
      calibration.getByText("相机内参与外参", { exact: true }),
    ).toBeVisible();
    for (const title of [
      "深度图",
      "相机内参与外参",
      "识别与分割叠加图",
      "结构化三维场景",
      "分割",
    ])
      await expect(card(title)).toHaveCount(0);
    const c = (await camera.boundingBox())!,
      k = (await calibration.boundingBox())!,
      a = (await ai.boundingBox())!;
    if (width > 900) {
      expect(c.x).toBeLessThan(k.x);
      expect(Math.abs(c.y - k.y)).toBeLessThan(2);
      expect(Math.abs(a.x - c.x)).toBeLessThan(2);
      expect(Math.abs(a.width - (k.x + k.width - c.x))).toBeLessThan(2);
    } else expect(k.y).toBeGreaterThan(c.y);
    expect(a.y).toBeGreaterThanOrEqual(
      Math.max(c.y + c.height, k.y + k.height),
    );
    const columns = ai.locator(".ai-columns > .ai-workspace");
    await expect(columns).toHaveCount(2);
    await expect(
      columns.nth(0).getByText("抓放场景", { exact: true }),
    ).toBeVisible();
    for (const title of ["自动分割", "提示词分割", "手动分割", "分割结果"])
      await expect(
        columns.nth(1).getByText(title, { exact: true }),
      ).toBeVisible();
    const left = (await columns.nth(0).boundingBox())!,
      right = (await columns.nth(1).boundingBox())!;
    if (width > 900) {
      expect(right.x - left.x - left.width).toBeGreaterThanOrEqual(20);
      expect(Math.abs(left.y - right.y)).toBeLessThan(2);
    } else expect(right.y).toBeGreaterThan(left.y + left.height);
    for (const title of ["抓放场景", "相机内参与外参", "分割结果"])
      await page.getByText(title, { exact: true }).click();
    await expect(fold("抓放场景").locator("select")).toHaveCount(2);
    await expect(
      fold("抓放场景").locator(".disclosure-content button"),
    ).toHaveCount(3);
    await expect(
      fold("抓放场景").getByRole("button", { name: "启动", exact: true }),
    ).toBeVisible();
    await expect(
      fold("相机内参与外参").getByText("内参 K", { exact: true }),
    ).toBeVisible();
    await expect(page.getByText("结构化三维场景", { exact: true })).toHaveCount(
      0,
    );
    await expect(
      fold("分割结果").getByText("场景坐标系", { exact: true }),
    ).toBeVisible();
    await expect(fold("分割结果").locator("table")).toHaveCount(0);
    await page.reload();
    await expect(
      page.getByRole("button", { name: "启动", exact: true }),
    ).toBeVisible();
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth > innerWidth,
      ),
    ).toBe(false);
  });
}
