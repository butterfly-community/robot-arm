import { expect, test, type WebSocketRoute } from "@playwright/test";

test("the initial page state comes only from the WebSocket snapshot", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/motion/state")).json();
  snapshot.values.arm_state.joints_rad[0] = 0.2;
  let duplicateSnapshotRequests = 0;
  await page.route("**/api/motion/state", (route) => {
    duplicateSnapshotRequests += 1;
    return route.abort();
  });
  await page.routeWebSocket("**/ws/motion", (route) =>
    route.send(JSON.stringify(snapshot)),
  );
  await page.goto("/motion/");
  await expect(page.locator(".joint-gauge strong").first()).toHaveText("11.5°");
  expect(duplicateSnapshotRequests).toBe(0);
});

// Browser-only fault injection: never send motion, camera or model commands to
// the running system. Keep real model meshes and the normal page contracts.
test("selected diagnostics do not freeze feedback and disconnect is visible", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/motion/state")).json();
  let socket: WebSocketRoute;
  let disconnected = false;
  await page.route("**/api/motion/state", (route) =>
    route.fulfill({ json: snapshot }),
  );
  await page.routeWebSocket("**/ws/motion", (route) => {
    if (disconnected) return route.close();
    socket = route;
    route.send(JSON.stringify(snapshot));
  });
  await page.goto("/motion/");
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
  const diagnostics = page
    .locator("section.card")
    .filter({ has: page.getByText("排障数据", { exact: true }) });
  await diagnostics.locator(".card-toggle").click();
  await diagnostics.locator("summary").first().click();
  const pre = diagnostics.locator("pre").first();
  await expect(pre).not.toHaveText("null");
  await pre.selectText();
  const selected = await page.evaluate(() => window.getSelection()?.toString());
  snapshot.values.arm_state.joints_rad[0] = 0.123;
  socket!.send(JSON.stringify(snapshot));
  await expect(page.locator(".joint-gauge strong").first()).toHaveText("7.0°");
  expect(await page.evaluate(() => window.getSelection()?.toString())).toBe(
    selected,
  );
  await page.locator("h1").click();
  // Stop reconnecting to exercise the stale-data indicator deterministically.
  disconnected = true;
  socket!.close();
  await expect(page.locator("p.error[role=alert]")).toContainText(
    "当前显示最后收到的数据",
  );
});

test("ordinary motion can be cancelled while its request is awaiting completion", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/motion/state")).json();
  await page.route("**/api/motion/state", (route) =>
    route.fulfill({ json: snapshot }),
  );
  await page.routeWebSocket("**/ws/motion", (route) =>
    route.send(JSON.stringify(snapshot)),
  );
  let release!: () => void;
  let requested = false;
  let cancelled = false;
  const completion = new Promise<void>((resolve) => {
    release = resolve;
  });
  await page.route("**/api/motion/*", async (route) => {
    if (route.request().method() !== "POST") return route.fallback();
    const path = new URL(route.request().url()).pathname;
    if (path.endsWith("/request")) {
      requested = true;
      await completion;
    }
    if (path.endsWith("/cancel")) {
      cancelled = true;
      release();
    }
    await route.fulfill({ json: { original_error: null } });
  });
  await page.goto("/motion/");
  await page.getByRole("button", { name: "执行目标", exact: true }).click();
  await expect.poll(() => requested).toBe(true);
  await page.getByRole("button", { name: "取消普通运动", exact: true }).click();
  await expect.poll(() => cancelled).toBe(true);
});

test("perception supports empty roles, rejects stale selections and stops after mode failure", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/perception/state")).json();
  const pose = { position_m: [0.1, 0, 0.1], orientation_xyzw: [0, 0, 0, 1] };
  snapshot.values.world_scene = {
    schema_version: 3,
    sequence: 1,
    sample_time_ns: 0,
    frame_id: "base_link",
    objects: [
      {
        object_id: "object-a",
        label: "object",
        pose,
        size_m: [0.03, 0.03, 0.03],
        confidence: 0.9,
        grasp_candidates: [{ ...pose, confidence: 0.87 }],
      },
    ],
    placement_regions: [
      {
        region_id: "region-a",
        label: "region",
        pose,
        size_m: [0.1, 0.1, 0.05],
      },
    ],
    obstacles: [],
  };
  snapshot.values.manipulation_state = {
    state: "failed",
    stage: "candidate IK",
    original_error: "测试规划错误",
  };
  let socket: WebSocketRoute;
  await page.route("**/api/perception/state", (route) =>
    route.fulfill({ json: snapshot }),
  );
  await page.routeWebSocket("**/ws/perception", (route) => {
    socket = route;
    route.send(JSON.stringify(snapshot));
  });
  const commands: { path: string; body: Record<string, unknown> }[] = [];
  await page.route("**/api/**", (route) => {
    if (route.request().method() !== "POST") return route.fallback();
    const path = new URL(route.request().url()).pathname;
    commands.push({ path, body: route.request().postDataJSON() });
    return route.fulfill({
      json: { original_error: path.endsWith("/mode") ? "模式切换失败" : null },
    });
  });
  const errors: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.goto("/perception/");
  await expect(
    page.getByRole("alert").filter({ hasText: "测试规划错误" }),
  ).toBeVisible();
  await page.getByText("抓放详细配置", { exact: true }).click();
  const roles = page.getByLabel("放置区域角色", { exact: true });
  await roles.fill("");
  await expect(roles).toHaveValue("");
  await page.getByRole("button", { name: "保存模型配置", exact: true }).click();
  await expect.poll(() => commands.length).toBe(1);
  expect(commands[0].body.placement_labels).toEqual([]);
  await page.getByRole("button", { name: "执行抓放", exact: true }).click();
  await expect.poll(() => commands.length).toBe(2);
  expect(commands[1].path).toBe("/api/motion/mode");
  await expect(
    page.getByText("Error: 模式切换失败", { exact: true }),
  ).toBeVisible();
  await page.getByLabel("抓取目标", { exact: true }).selectOption("object-a");
  snapshot.values.world_scene.objects[0].object_id = "object-b";
  socket!.send(JSON.stringify(snapshot));
  await expect(page.getByLabel("抓取目标", { exact: true })).toHaveValue("");
  await expect(
    page.getByRole("button", { name: "执行抓放", exact: true }),
  ).toBeDisabled();
  expect(commands).toHaveLength(2);
  expect(errors).toEqual([]);
});

test("failed spatial save preserves the edited matrix", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/spatial/state")).json();
  await page.route("**/api/spatial/state", (route) =>
    route.fulfill({ json: snapshot }),
  );
  await page.routeWebSocket("**/ws/spatial", (route) =>
    route.send(JSON.stringify(snapshot)),
  );
  await page.route("**/api/spatial/config", (route) =>
    route.fulfill({ json: { original_error: "保存失败（测试）" } }),
  );
  await page.goto("/spatial/");
  const input = page.getByLabel("映射 1,1", { exact: true });
  await input.fill("0.75");
  await input.press("Tab");
  await expect(page.locator(".error")).toContainText("保存失败（测试）");
  await expect(input).toHaveValue("0.75");
});

test("input rename failure remains retryable without an unhandled rejection", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/tracking/state")).json();
  snapshot.values.discovery_state.sources = [
    {
      source_id: "simulation:review",
      driver_id: "simulation",
      display_name: "测试输入",
      custom_name: null,
      position_capable: false,
      orientation_capable: false,
      capabilities: [],
    },
  ];
  await page.route("**/api/tracking/state", (route) =>
    route.fulfill({ json: snapshot }),
  );
  await page.routeWebSocket("**/ws/tracking", (route) =>
    route.send(JSON.stringify(snapshot)),
  );
  await page.route("**/api/tracking/source-name", (route) =>
    route.fulfill({ json: { original_error: "改名失败（测试）" } }),
  );
  const errors: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.goto("/tracking/");
  await page.getByLabel("测试输入 自定义名称").fill("新名称");
  await page.getByRole("button", { name: "保存名称", exact: true }).click();
  await expect(page.locator("p.error[role=alert]")).toContainText(
    "改名失败（测试）",
  );
  await expect(
    page.getByRole("button", { name: "保存名称", exact: true }),
  ).toBeEnabled();
  expect(errors).toEqual([]);
});

test("video orderly disconnect reports reconnection instead of silently showing a stale frame", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/perception/state")).json();
  snapshot.values.camera_state.streaming = true;
  await page.route("**/api/perception/state", (route) =>
    route.fulfill({ json: snapshot }),
  );
  await page.routeWebSocket("**/ws/perception", (route) =>
    route.send(JSON.stringify(snapshot)),
  );
  await page.routeWebSocket("**/ws/camera-video", (route) =>
    route.close({ code: 1000 }),
  );
  await page.goto("/perception/");
  const monitor = page.getByLabel("彩色视频浮动窗口");
  await expect(
    monitor.getByText("相机视频连接中断，正在重连", { exact: true }),
  ).toBeVisible();
});

test("robot asset failure is visible and page reload recovers the preview", async ({
  page,
}) => {
  test.setTimeout(60_000);
  await page.route("**/api/motion/assets/**", (route) =>
    route.fulfill({ status: 404, body: "asset unavailable (test)" }),
  );
  await page.goto("/motion/");
  await expect(page.locator(".robot-load-status[role=alert]")).toContainText(
    "机械臂模型加载失败",
  );
  await page.unroute("**/api/motion/assets/**");
  await page.reload();
  await expect(page.getByLabel("机械臂三维反馈与目标预览")).toHaveAttribute(
    "data-models-loaded",
    "2",
  );
  await expect(page.locator(".robot-load-status")).toHaveCount(0);
});

for (const path of [
  "tracking",
  "spatial",
  "motion",
  "arm-execution",
  "perception",
]) {
  test(`${path} layout fits desktop and mobile`, async ({ page }, testInfo) => {
    test.setTimeout(60_000);
    const errors: string[] = [];
    page.on("pageerror", (error) => errors.push(error.message));
    await page.goto(`/${path}/`);
    await expect(page.locator(".topbar-state")).toHaveText("实时连接正常");
    if (path === "motion" || path === "arm-execution") {
      await expect(page.getByLabel("机械臂三维反馈与目标预览")).toHaveAttribute(
        "data-models-loaded",
        "2",
      );
    }
    for (const width of [1920, 1280, 390]) {
      await page.setViewportSize({ width, height: 1000 });
      await expect
        .poll(() =>
          page.evaluate(
            () => document.documentElement.scrollWidth <= innerWidth,
          ),
        )
        .toBe(true);
      await page.screenshot({
        path: testInfo.outputPath(`${path}-${width}.png`),
      });
    }
    expect(errors).toEqual([]);
  });
}
