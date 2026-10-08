import { expect, test } from "@playwright/test";

test("real input page shows live no-gamepad discovery and survives refresh", async ({
  page,
  request,
}, testInfo) => {
  const response = await request.get("/api/tracking/state");
  expect(response.ok()).toBe(true);
  const state = (await response.json()).values.discovery_state;
  expect(
    state.drivers.find(
      (d: { driver_id: string }) => d.driver_id === "sdl3-gamepad",
    ).original_error,
  ).toBeNull();
  expect(
    state.sources.some(
      (s: { driver_id: string; active: boolean }) =>
        s.driver_id === "sdl3-gamepad" && s.active,
    ),
  ).toBe(false);
  await page.goto("/tracking/");
  const toggle = page.locator(".card-toggle").filter({ hasText: "输入源" });
  // No mocked data and no intercepted requests in this acceptance test.
  const notice = page
    .getByRole("status")
    .filter({ hasText: "SDL 未发现可用手柄" });
  await expect(notice).toBeVisible();
  await toggle.click();
  await expect(notice).toBeHidden();
  await toggle.click();
  await expect(notice).toBeVisible();
  for (const width of [1440, 390]) {
    await page.setViewportSize({ width, height: 1000 });
    await expect(notice).toBeVisible();
    await notice.scrollIntoViewIfNeeded();
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth <= innerWidth,
      ),
    ).toBe(true);
    await page.screenshot({
      path: testInfo.outputPath(`input-discovery-${width}.png`),
    });
  }
  await page.reload();
  await expect(notice).toBeVisible();
});

test("injected failure and recovery messages remain distinct from missing devices", async ({
  page,
}) => {
  const snapshot = {
    namespace: "tracking",
    values: {
      discovery_state: {
        drivers: [
          { driver_id: "sdl3-gamepad", original_error: null as string | null },
          { driver_id: "nolo-cv1-hid", original_error: null },
        ],
        sources: [] as Record<string, unknown>[],
      },
    },
  };
  let publish = () => {};
  await page.route("**/api/**", (route) =>
    route.request().method() === "GET"
      ? route.fulfill({ json: snapshot })
      : route.abort(),
  );
  await page.routeWebSocket("**/ws/tracking", (socket) => {
    publish = () => socket.send(JSON.stringify(snapshot));
    publish();
  });
  await page.goto("/tracking/");
  await expect(
    page.getByRole("status").filter({ hasText: "SDL 未发现可用手柄" }),
  ).toBeVisible();
  for (const failure of [
    "打开 SDL 手柄失败：Operation not permitted",
    "启用陀螺仪失败（DualSense）：Operation not supported",
  ]) {
    snapshot.values.discovery_state.drivers[0].original_error = `SDL3 gamepad 采集线程已停止：${failure}`;
    publish();
    await expect(
      page.getByRole("alert").filter({ hasText: failure }),
    ).toBeVisible();
    await expect(
      page.getByText("SDL 未发现可用手柄", { exact: false }),
    ).toHaveCount(0);
  }
  snapshot.values.discovery_state.drivers[0].original_error = null;
  snapshot.values.discovery_state.sources = [
    {
      source_id: "test-pad",
      display_name: "测试手柄",
      driver_id: "sdl3-gamepad",
      active: true,
    },
  ];
  publish();
  await expect(
    page.getByText("测试手柄", { exact: true }).first(),
  ).toBeVisible();
  await expect(
    page.getByRole("alert").filter({ hasText: "SDL3 gamepad" }),
  ).toHaveCount(0);
  await expect(
    page.getByText("SDL 未发现可用手柄", { exact: false }),
  ).toHaveCount(0);
});
