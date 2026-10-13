import { expect, test } from "@playwright/test";

// Real page operations, no mocked responses. Run after stereo-camera.mjs;
// only the generated wrist-right stream is temporarily stopped and restored.
test("failed stereo check clears the previous depth result", async ({
  page,
}) => {
  test.skip(
    process.env.STEREO_ACCEPTANCE_TEST !== "1",
    "先完成 stereo-camera.mjs，再显式运行双目状态回归",
  );
  test.setTimeout(60000);
  await page.goto("/perception/");
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
  const state = await (await page.request.get("/api/perception/state")).json();
  const peer = state.values.camera_state.bindings.find(
    (b: { role: string }) => b.role === "wrist_right",
  );
  expect(peer.source_id).toBe("replay:wrist-right");
  expect(peer.streaming).toBe(true);
  for (const preview of await page.locator(".floating-camera-monitor").all()) {
    const fold = preview.getByRole("button", { name: "收起", exact: true });
    if (await fold.isVisible()) await fold.click();
  }
  const card = page.locator("section.card").filter({
    has: page
      .locator(".card-title-text")
      .getByText("腕部双目标定", { exact: true }),
  });
  const toggle = card.locator(".card-toggle");
  if ((await toggle.getAttribute("aria-expanded")) === "false")
    await toggle.click();
  const check = card.getByRole("button", { name: "检查深度", exact: true });
  const pixels = card.getByText("有效深度像素", { exact: true });
  await check.click();
  await expect(pixels).toBeVisible({ timeout: 20000 });
  const binding = page.getByRole("region", {
    name: "腕部摄像头 2（右）绑定",
    exact: true,
  });
  try {
    await binding
      .getByRole("button", { name: "关闭腕部摄像头 2（右）采集", exact: true })
      .click();
    await expect(
      binding.getByText("采集已关闭，绑定与分辨率保留", { exact: true }),
    ).toBeVisible();
    await check.click();
    await expect(card.getByRole("alert")).toContainText(
      "绑定并开启同组左右相机",
    );
    await expect(pixels).toHaveCount(0);
  } finally {
    await binding
      .getByRole("button", { name: "开启腕部摄像头 2（右）采集", exact: true })
      .click();
    await expect(binding.getByText("正在采集", { exact: true })).toBeVisible({
      timeout: 20000,
    });
  }
  await check.click();
  await expect(pixels).toBeVisible({ timeout: 20000 });
  await expect(card.getByRole("alert")).toHaveCount(0);
});
