import { expect, test } from "@playwright/test";

// Read-only UI inspection. Numerical and save/run acceptance is in stereo-camera.mjs.
for (const width of [1600, 390]) {
  test(`four camera bindings and stereo calibration layout at ${width}px`, async ({
    page,
  }) => {
    await page.setViewportSize({ width, height: 1100 });
    await page.goto("/perception/");
    await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
    if (process.env.STEREO_ACCEPTANCE_TEST === "1") {
      // Previews mount after their first state update. Do not let an empty
      // locator list silently skip the camera-header regression assertions.
      await expect(page.locator(".floating-camera-monitor")).toHaveCount(4);
    }
    for (const preview of await page
      .locator(".floating-camera-monitor")
      .all()) {
      const fold = preview.getByRole("button", { name: "收起", exact: true });
      if (await fold.isVisible()) await fold.click();
      const title = preview.locator(".floating-camera-header strong");
      await expect(title).toHaveAttribute(
        "title",
        (await title.textContent()) ?? "",
      );
      for (const node of [
        title,
        preview.locator(".floating-camera-header span"),
      ]) {
        const style = await node.evaluate((el) => ({
          wrap: getComputedStyle(el).whiteSpace,
          height: el.getBoundingClientRect().height,
          lineHeight: Number.parseFloat(getComputedStyle(el).lineHeight),
        }));
        expect(style.wrap).toBe("nowrap");
        expect(style.height).toBeLessThanOrEqual(style.lineHeight + 1);
      }
    }
    for (const label of [
      "外部摄像头 1（左）",
      "外部摄像头 2（右）",
      "腕部摄像头 1（左）",
      "腕部摄像头 2（右）",
    ]) {
      await expect(
        page.getByRole("region", { name: `${label}绑定`, exact: true }),
      ).toBeVisible();
    }
    const cardLocators = [];
    for (const title of ["外部双目标定", "腕部双目标定"]) {
      const card = page.locator("section.card").filter({
        has: page.locator(".card-title-text").getByText(title, { exact: true }),
      });
      const toggle = card.locator(".card-toggle");
      if ((await toggle.getAttribute("aria-expanded")) === "false")
        await toggle.click();
      await expect(
        card.getByRole("button", { name: "采集双目样本", exact: true }),
      ).toBeVisible();
      await expect(
        card.getByRole("button", { name: "采集外参样本", exact: true }),
      ).toBeVisible();
      const bounds = (await card.boundingBox())!;
      cardLocators.push(card);
      const sections = await card
        .locator(".card-content-inner > .camera-binding")
        .evaluateAll((nodes) =>
          nodes.map((node) => {
            const rect = node.getBoundingClientRect();
            return { top: rect.top, bottom: rect.bottom };
          }),
        );
      for (let i = 1; i < sections.length; i++)
        expect(sections[i].top - sections[i - 1].bottom).toBeGreaterThanOrEqual(
          20,
        );
      for (const button of await card.getByRole("button").all()) {
        const b = await button.boundingBox();
        if (b) {
          expect(b.x).toBeGreaterThanOrEqual(bounds.x);
          expect(b.x + b.width).toBeLessThanOrEqual(
            bounds.x + bounds.width + 1,
          );
        }
      }
    }
    // Clicking the second card can scroll the viewport: measure both afterwards.
    const cards = await Promise.all(
      cardLocators.map(async (card) => (await card.boundingBox())!),
    );
    if (width > 900) {
      expect(cards[1].x - cards[0].x - cards[0].width).toBeGreaterThanOrEqual(
        16,
      );
      expect(Math.abs(cards[0].y - cards[1].y)).toBeLessThan(2);
    } else expect(cards[1].y).toBeGreaterThan(cards[0].y + cards[0].height);
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth > innerWidth,
      ),
    ).toBe(false);
    await page.screenshot({
      path: test.info().outputPath("stereo-layout.png"),
    });
  });
}
