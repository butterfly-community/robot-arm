import { expect, test } from "@playwright/test";
import type { ArmState } from "@robot/contracts";

// Live user-path test, without response interception or synthetic feedback.
// Changes the saved execution mode and leaves it in simulation. Never commands
// hardware motion; the only motion request is after software mode is confirmed.
test("stopped hardware can explicitly switch to persistent simulator mode", async ({
  page,
}, info) => {
  test.setTimeout(120_000);
  let transport:
    { selected_endpoint: string | null; connected: boolean } | undefined;
  let arm: ArmState | undefined;
  let motion:
    { latest_motion?: { request_id: string; state: string } } | undefined;
  const requests: { path: string; body: Record<string, unknown> }[] = [];
  const errors: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  page.on("request", (request) => {
    if (request.method() === "POST") {
      requests.push({
        path: new URL(request.url()).pathname,
        body: request.postDataJSON(),
      });
    }
  });
  page.on("websocket", (socket) =>
    socket.on("framereceived", (frame) => {
      if (typeof frame.payload !== "string") return;
      const values = JSON.parse(frame.payload).values;
      if (values?.transport_state) transport = values.transport_state;
      if (values?.arm_state) arm = values.arm_state;
      if (values?.motion_state) motion = values.motion_state;
    }),
  );
  await page.goto("/arm-execution/");
  await expect.poll(() => Boolean(transport && arm)).toBe(true);
  const mode = page.getByLabel("执行模式", { exact: true });
  const expand = async () => {
    const toggle = page.getByRole("button", { name: /执行连接/ });
    if ((await toggle.getAttribute("aria-expanded")) === "false")
      await toggle.click();
  };
  await expand();
  const initial = JSON.parse(JSON.stringify({ transport, arm }));
  if (transport!.selected_endpoint != null) {
    await mode.selectOption("software");
    // Selecting a draft alone must not modify the active endpoint.
    expect(transport!.selected_endpoint).toBe(
      initial.transport.selected_endpoint,
    );
    await page
      .getByRole("button", { name: "应用执行模式", exact: true })
      .click();
  }
  await expect.poll(() => transport?.selected_endpoint).toBeNull();
  await expect.poll(() => arm?.feedback_source).toBe("software");
  await expect(
    page.getByRole("button", { name: "模拟器已启用" }),
  ).toBeDisabled();

  // Reproduce failed/stopped hardware through the real connection form.
  await mode.selectOption("hardware");
  const discovery = page.waitForResponse(
    (response) =>
      response.url().endsWith("/api/arm-execution/endpoints") &&
      response.request().method() === "POST",
  );
  await page.getByRole("button", { name: "刷新串口", exact: true }).click();
  const discovered = await (await discovery).json();
  expect(discovered.original_error).toBeNull();
  expect(discovered.acknowledged_action).toBe("discover");
  expect(transport!.selected_endpoint).toBeNull();
  const missingPort = `/dev/robot-arm-missing-${Date.now()}`;
  await page
    .getByLabel("串口路径（可手动输入）", { exact: true })
    .fill(missingPort);
  await page.getByRole("button", { name: "连接真机", exact: true }).click();
  await expect(page.getByLabel("机械臂连接异常")).toBeVisible();
  await expect.poll(() => transport?.selected_endpoint).toBe(missingPort);
  expect(transport!.connected).toBe(false);
  await page.reload();
  await expand();
  await expect(mode).toHaveValue("hardware");
  await expect(
    page.getByLabel("串口路径（可手动输入）", { exact: true }),
  ).toHaveValue(missingPort);

  const observedPose = [...arm!.joints_rad];
  await mode.selectOption("software");
  await page.getByRole("button", { name: "应用执行模式", exact: true }).click();
  await expect.poll(() => transport?.selected_endpoint).toBeNull();
  await expect.poll(() => arm?.feedback_source).toBe("software");
  expect(arm!.joints_rad).toEqual(observedPose);
  await expect(page.getByLabel("机械臂连接异常")).toHaveCount(0);
  await page.reload();
  await expand();
  await expect(mode).toHaveValue("software");
  await expect(
    page.getByRole("button", { name: "模拟器已启用" }),
  ).toBeDisabled();
  for (const width of [1440, 390]) {
    await page.setViewportSize({ width, height: 1000 });
    await expect
      .poll(() =>
        page.evaluate(() => document.documentElement.scrollWidth <= innerWidth),
      )
      .toBe(true);
    await page
      .locator("section.card")
      .filter({ has: mode })
      .screenshot({
        path: info.outputPath(`mode-${width}.png`),
      });
  }

  // The same motion page and MoveIt path must still produce software feedback.
  await page.goto("/motion/");
  const sliders = page.getByRole("slider");
  await expect(sliders.first()).toBeVisible();
  const before = arm!.sequence;
  await sliders.first().press("ArrowRight");
  const target = Number(await sliders.first().inputValue());
  await page.getByRole("button", { name: "执行目标", exact: true }).click();
  await expect
    .poll(
      () =>
        requests.findLast((r) => r.path === "/api/motion/request")?.body
          .request_id,
    )
    .toBeTruthy();
  const motionId = requests.findLast((r) => r.path === "/api/motion/request")!
    .body.request_id;
  await expect
    .poll(() => motion?.latest_motion?.request_id, { timeout: 60_000 })
    .toBe(motionId);
  await expect
    .poll(() => motion?.latest_motion?.state, { timeout: 60_000 })
    .toBe("succeeded");
  expect(arm!.feedback_source).toBe("software");
  expect(arm!.sequence).toBeGreaterThan(before);
  expect(arm!.joints_rad[0]).toBeCloseTo(target, 4);
  expect(errors).toEqual([]);
  await info.attach("mode-switch-result", {
    body: JSON.stringify({
      initial,
      transport,
      arm,
      motion: motion?.latest_motion,
      requests,
    }),
    contentType: "application/json",
  });
});

// Supplemental failure/pending regression only. Does not count as live mode
// switching: all writes and snapshots in this browser are intercepted.
test("failed simulator activation retains the hardware mode and permits retry", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/arm-execution/state")).json();
  snapshot.values.transport_state.connected = true;
  snapshot.values.transport_state.selected_endpoint = "/dev/test-fixture";
  snapshot.values.transport_state.last_error = null;
  snapshot.values.arm_state.feedback_source = "hardware";
  let publish = () => {};
  await page.route("**/api/**", (route) =>
    route.request().method() === "GET" ? route.fallback() : route.abort(),
  );
  await page.route("**/api/arm-execution/state", (route) =>
    route.fulfill({ json: snapshot }),
  );
  await page.routeWebSocket("**/ws/arm-execution", (socket) => {
    publish = () => socket.send(JSON.stringify(snapshot));
    publish();
  });
  let fail = true;
  let release: (() => void) | undefined;
  await page.route("**/api/arm-execution/disconnect", async (route) => {
    expect(route.request().postDataJSON().action).toBe("disconnect");
    if (fail) {
      await new Promise<void>((resolve) => {
        release = resolve;
      });
      return route.fulfill({
        json: { original_error: "配置保存失败（测试响应）" },
      });
    }
    snapshot.values.transport_state.connected = false;
    snapshot.values.transport_state.selected_endpoint = null;
    snapshot.values.arm_state.feedback_source = "software";
    publish();
    return route.fulfill({ json: { original_error: null } });
  });
  await page.goto("/arm-execution/");
  const mode = page.getByLabel("执行模式", { exact: true });
  await expect(mode).toHaveValue("hardware");
  await mode.selectOption("software");
  await page.getByRole("button", { name: "应用执行模式", exact: true }).click();
  await expect(page.getByRole("button", { name: "正在切换…" })).toBeDisabled();
  await expect(mode).toBeDisabled();
  await expect.poll(() => Boolean(release)).toBe(true);
  release!();
  await expect(page.locator('p.error[role="alert"]')).toContainText(
    "配置保存失败",
  );
  await expect(mode).toHaveValue("software");
  await expect(
    page.getByRole("button", { name: "应用执行模式", exact: true }),
  ).toBeEnabled();
  await expect(
    page.locator(".key-value").filter({ hasText: "当前生效模式" }),
  ).toContainText("真机执行反馈模式");
  fail = false;
  await page.getByRole("button", { name: "应用执行模式", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "模拟器已启用" }),
  ).toBeDisabled();
  await expect(page.locator('p.error[role="alert"]')).toHaveCount(0);
});
