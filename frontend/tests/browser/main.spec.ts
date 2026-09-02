import { expect, test } from "@playwright/test";

test.beforeEach(async ({ request }) => {
  await expect
    .poll(
      async () => {
        const response = await request.get("/api/system/readiness");
        if (!response.ok()) return false;
        const state = await response.json();
        return state.values.system_readiness?.ready === true;
      },
      { timeout: 60_000 },
    )
    .toBe(true);
});

const pages = [
  ["tracking", "手动控制绑定"],
  ["spatial", "空间转换"],
  ["perception", "场景感知"],
  ["motion", "机械臂运动"],
  ["arm-execution", "机械臂执行"],
] as const;

for (const [path, title] of pages) {
  test(`${path} page is independently routed and labelled`, async ({
    page,
  }) => {
    const errors: string[] = [];
    page.on("pageerror", (error) => errors.push(error.message));
    await page.goto(`/${path}/`);
    await expect(page.getByRole("heading", { level: 1 })).toHaveText(title);
    const navigation = page.getByRole("navigation");
    await expect(navigation.getByRole("link")).toHaveCount(5);
    await expect(navigation.locator('[aria-current="page"]')).toHaveAttribute(
      "href",
      `/${path}/`,
    );
    const card = page.locator("section.card").first();
    const toggle = card.locator(".card-toggle");
    await expect(toggle).toHaveAttribute("aria-expanded", "true");
    await toggle.click();
    await expect(toggle).toHaveAttribute("aria-expanded", "false");
    await expect(card.locator(".card-content")).toBeHidden();
    await toggle.click();
    await expect(card.locator(".card-content")).toBeVisible();
    const diagnosticsTitle = path === "tracking" ? "实时诊断" : "排障数据";
    const diagnostics = page
      .locator(".card-toggle")
      .filter({ hasText: diagnosticsTitle });
    await expect(diagnostics).toHaveAttribute("aria-expanded", "false");
    await diagnostics.click();
    await expect(
      page.locator("details.diagnostics").first(),
    ).not.toHaveAttribute("open", "");
    await expect(page.locator("main")).toBeVisible();
    await page.reload();
    await expect(page.getByRole("heading", { level: 1 })).toHaveText(title);
    await diagnostics.click();
    await expect(page.locator("pre").first()).not.toHaveText("null");
    expect(errors).toEqual([]);
  });
}

test("cross-service navigation does not report socket teardown as an error", async ({
  page,
}) => {
  const consoleErrors: string[] = [];
  page.on("console", (message) => {
    if (message.type() === "error") consoleErrors.push(message.text());
  });
  await page.goto("/tracking/");
  await page.getByRole("link", { name: /02 空间/ }).click();
  await expect(page.getByRole("heading", { level: 1 })).toHaveText("空间转换");
  await expect(page.locator("p.error")).toHaveCount(0);
  expect(consoleErrors).toEqual([]);
});

for (const [path] of pages) {
  test(`${path} live snapshots do not clear selected text`, async ({
    page,
  }) => {
    await page.goto(`/${path}/`);
    const diagnosticsTitle = path === "tracking" ? "实时诊断" : "排障数据";
    const diagnosticsCard = page.locator("section.card").filter({
      has: page.getByText(diagnosticsTitle, { exact: true }),
    });
    await diagnosticsCard.locator(".card-toggle").click();
    const details = diagnosticsCard.locator("details.diagnostics").first();
    await details.locator("summary").click();
    const state = details.locator("pre");
    await expect(state).not.toHaveText("null");
    await state.selectText();
    const selected = await page.evaluate(() =>
      window.getSelection()?.toString(),
    );
    expect(selected?.length).toBeGreaterThan(0);
    await page.waitForTimeout(500);
    expect(await page.evaluate(() => window.getSelection()?.toString())).toBe(
      selected,
    );
    await page.locator("h1").click();
    await expect
      .poll(() => page.evaluate(() => window.getSelection()?.toString()))
      .toBe("");
  });
}

test("tracking exposes the complete action catalog and sends every demo item", async ({
  page,
}) => {
  const actions = [
    ["纵向平移", "move_forward_back", true, "演示动作"],
    ["横向平移", "move_left_right", true, "演示动作"],
    ["垂直平移", "move_up_down", true, "演示动作"],
    ["定点垂直旋转", "tool_pitch", true, "演示动作"],
    ["定点水平旋转", "tool_yaw", true, "演示动作"],
    ["轴向旋转", "tool_roll", true, "演示动作"],
    ["垂直圆弧", "front_pitch", true, "演示动作"],
    ["水平圆弧", "horizontal_arc", true, "演示动作"],
    ["夹爪张开", "primary_tool_open", true, "测试动作"],
    ["夹爪开合", "primary_tool", true, "演示动作"],
    ["接管控制", "start_stop", true, "测试动作"],
    ["急停", "emergency_stop", true, "测试动作"],
    ["工具轴向平移", "tool_axis_translation", true, "演示动作"],
    ["工具轴向螺旋", "tool_helical_motion", true, "演示动作"],
  ] as const;
  const simulationRequests: Array<Record<string, unknown>> = [];
  let prepareRequests = 0;
  await page.route("**/api/motion/prepare-relative", async (route) => {
    prepareRequests += 1;
    await route.fulfill({
      contentType: "application/json",
      body: JSON.stringify({ original_error: null }),
    });
  });
  await page.route("**/api/tracking/simulation", async (route) => {
    simulationRequests.push(route.request().postDataJSON());
    await route.fulfill({
      contentType: "application/json",
      body: JSON.stringify({ original_error: null }),
    });
  });
  await page.goto("/tracking/");

  const bindingCard = page.locator("section.card").filter({
    has: page.getByText("功能与反馈绑定", { exact: true }),
  });
  await expect(bindingCard.locator(".action-group > h3")).toHaveText([
    "TCP 基础自由度",
    "圆弧复合",
    "夹爪动作",
    "控制动作",
    "力度反馈",
    "其他复合",
  ]);
  const details = bindingCard.locator("details.action-binding-item");
  await expect(details).toHaveCount(15);
  for (let index = 0; index < 15; index += 1) {
    await expect(details.nth(index)).not.toHaveAttribute("open", "");
  }

  const firstPair = bindingCard.locator(".action-group-items").first();
  const firstItem = firstPair.locator("details").nth(0);
  const secondItem = firstPair.locator("details").nth(1);
  const collapsedSecondHeight = await secondItem.evaluate(
    (element) => element.getBoundingClientRect().height,
  );
  await firstItem.locator("summary").click();
  await expect(firstItem).toHaveAttribute("open", "");
  await expect
    .poll(() =>
      secondItem.evaluate((element) => element.getBoundingClientRect().height),
    )
    .toBe(collapsedSecondHeight);
  await firstItem.locator("summary").click();

  for (const [label, item, , buttonName] of actions) {
    const previousRequestCount = simulationRequests.length;
    const detail = details.filter({
      has: page.getByText(label, { exact: true }),
    });
    await detail.locator("summary").click();
    await expect(detail.getByText("输入语义", { exact: true })).toBeVisible();
    await expect(detail.getByText("保持不变", { exact: true })).toBeVisible();
    await detail.getByRole("button", { name: buttonName }).click();
    await expect
      .poll(() => simulationRequests.length)
      .toBe(previousRequestCount + 1);
    expect(simulationRequests.at(-1)).toMatchObject({
      schema_version: 3,
      enabled: true,
      item,
    });
    await detail.locator("summary").click();
  }

  const feedbackDetail = details.filter({
    has: page.getByText("夹爪力度反馈", { exact: true }),
  });
  await feedbackDetail.locator("summary").click();
  await feedbackDetail.getByRole("button", { name: "测试反馈" }).click();
  expect(simulationRequests.at(-1)).toMatchObject({
    schema_version: 3,
    enabled: true,
    item: "primary_tool_feedback",
  });
  expect(simulationRequests).toHaveLength(15);
  expect(prepareRequests).toBe(
    actions.filter(([, , prepare]) => prepare).length,
  );

  const diagnostics = page
    .locator("section.card")
    .filter({ has: page.getByText("实时诊断", { exact: true }) });
  await expect(diagnostics.locator(".card-toggle")).toHaveAttribute(
    "aria-expanded",
    "false",
  );
  await expect(page.getByText("启动模拟数据", { exact: true })).toHaveCount(0);
});

test("tracking page applies and displays a controller binding", async ({
  page,
  request,
}) => {
  await page.goto("/tracking/");
  const before = await (await request.get("/api/tracking/state")).json();
  const sources = before.values.discovery_state?.sources ?? [];
  const sourceCard = page.locator("section.card").filter({
    has: page.getByText("输入源", { exact: true }),
  });
  await expect(
    sourceCard.getByRole("button", {
      name: /^(用作空间来源|当前空间来源)$/,
    }),
  ).toHaveCount(
    sources.filter((source: { position_capable?: boolean }) =>
      Boolean(source.position_capable),
    ).length,
  );
  await expect(
    sourceCard.getByRole("button", {
      name: /^(用作姿态来源|当前姿态来源)$/,
    }),
  ).toHaveCount(
    sources.filter((source: { orientation_capable?: boolean }) =>
      Boolean(source.orientation_capable),
    ).length,
  );
  const sourceSummaryBox = await sourceCard
    .locator(".key-value")
    .last()
    .boundingBox();
  const firstSourceBox = await sourceCard
    .locator(".source-item")
    .first()
    .boundingBox();
  expect(sourceSummaryBox).not.toBeNull();
  expect(firstSourceBox).not.toBeNull();
  expect(
    (firstSourceBox?.y ?? 0) -
      ((sourceSummaryBox?.y ?? 0) + (sourceSummaryBox?.height ?? 0)),
    "来源摘要分隔线与设备卡片之间应保留明确间距",
  ).toBeGreaterThanOrEqual(16);

  for (const name of ["清除空间来源", "清除姿态来源"]) {
    await expect(sourceCard.getByRole("button", { name })).toHaveClass(
      /button-outline/,
    );
  }

  const bindingCard = page.locator("section.card").filter({
    has: page.getByText("功能与反馈绑定", { exact: true }),
  });
  const verticalBinding = bindingCard
    .locator("details.action-binding-item")
    .filter({
      has: page.getByText("垂直平移", { exact: true }),
    });
  await verticalBinding.locator("summary").click();
  await expect(verticalBinding.getByText("正向上移，负向下移")).toBeVisible();
  await expect(
    verticalBinding.getByText("TCP 沿竖直方向直线平移"),
  ).toBeVisible();
  await verticalBinding.getByLabel("垂直平移输入方式").selectOption("buttons");
  await expect(verticalBinding.getByLabel("垂直平移下移")).toBeVisible();
  await expect(verticalBinding.getByLabel("垂直平移上移")).toBeVisible();
  const summaryBox = await verticalBinding.locator("summary").boundingBox();
  const contentBox = await verticalBinding
    .locator(".action-description")
    .boundingBox();
  expect(summaryBox).not.toBeNull();
  expect(contentBox).not.toBeNull();
  expect(
    (contentBox?.y ?? 0) - ((summaryBox?.y ?? 0) + (summaryBox?.height ?? 0)),
    "动作摘要与展开内容之间应保留明确间距",
  ).toBeGreaterThanOrEqual(8);

  const selectedRuntimeSource = before.values.discovery_state?.sources?.find(
    (source: { available_components?: Array<{ action_type?: string }> }) =>
      (source.available_components ?? []).filter(
        (component) => component.action_type === "boolean",
      ).length >= 1,
  );
  test.skip(
    !selectedRuntimeSource,
    "当前没有可用控制器输入端点，实体绑定验收留给设备在线时执行",
  );
  const original = (before.values.discovery_state?.bindings ?? [])
    .filter(
      (binding: { configured_components: string[] }) =>
        binding.configured_components.length > 0,
    )
    .map(
      (binding: {
        action: string;
        action_type: string;
        configured_components: string[];
        invert: boolean;
        source_id: string | null;
      }) => ({
        action: binding.action,
        action_type: binding.action_type,
        source_id: binding.source_id,
        component_paths: binding.configured_components,
        invert: binding.invert,
      }),
    );
  try {
    const field = bindingCard.locator("details.action-binding-item").filter({
      has: page.getByText("接管控制", { exact: true }),
    });
    await field.locator("summary").click();
    const sourceSelect = field.getByLabel("接管控制输入设备");
    await sourceSelect.selectOption(selectedRuntimeSource.source_id);
    await expect(sourceSelect).toHaveValue(selectedRuntimeSource.source_id);
    const componentSelect = field.getByLabel("接管控制设备输入");
    await expect
      .poll(() => componentSelect.locator("option").count())
      .toBeGreaterThan(1);
    const componentPath = await componentSelect
      .locator("option")
      .nth(1)
      .getAttribute("value");
    expect(componentPath).toBeTruthy();
    await componentSelect.selectOption(componentPath!);
    await page.getByRole("button", { name: "应用绑定" }).click();
    await expect(
      page.getByRole("button", { name: "绑定已应用" }),
    ).toBeVisible();
    await expect
      .poll(async () => {
        const current = await (await request.get("/api/tracking/state")).json();
        const binding = current.values.discovery_state.bindings.find(
          (item: { action: string }) => item.action === "start_stop",
        );
        return {
          configured: binding.configured_components,
          sourceId: binding.source_id,
        };
      })
      .toEqual({
        configured: [componentPath],
        sourceId: selectedRuntimeSource.source_id,
      });
  } finally {
    const current = await (await request.get("/api/tracking/state")).json();
    const feedbackBindings =
      current.values.discovery_state.feedback_bindings?.map(
        (binding: {
          action: string;
          source_id: string;
          capability_path: string;
        }) => ({
          action: binding.action,
          source_id: binding.source_id,
          capability_path: binding.capability_path,
        }),
      ) ?? [];
    await request.post("/api/tracking/bindings", {
      data: {
        schema_version: 3,
        request_id: "browser-bindings-restore",
        bindings: original,
        feedback_bindings: feedbackBindings,
      },
    });
  }
});

test("tracking page persists a custom input device name", async ({
  page,
  request,
}) => {
  const before = await (await request.get("/api/tracking/state")).json();
  const source = before.values.discovery_state?.sources?.find(
    (candidate: { driver_id?: string }) =>
      candidate.driver_id === "sdl3-gamepad",
  ) as
    | {
        source_id: string;
        display_name: string;
        custom_name: string | null;
      }
    | undefined;
  test.skip(!source, "需要一个在线 SDL3 输入设备");

  try {
    await page.goto("/tracking/");
    const editor = page.getByLabel(`${source!.display_name} 自定义名称`);
    await editor.fill("浏览器名称测试");
    const saved = page.waitForResponse(
      (response) =>
        response.url().endsWith("/api/tracking/source-name") &&
        response.request().method() === "POST",
    );
    await editor
      .locator("xpath=..")
      .getByRole("button", { name: "保存名称" })
      .click();
    await saved;
    await expect(
      editor
        .locator("xpath=../..")
        .getByText("浏览器名称测试", { exact: true }),
    ).toBeVisible();

    await page.reload();
    await expect(
      page.getByLabel(`${source!.display_name} 自定义名称`),
    ).toHaveValue("浏览器名称测试");
  } finally {
    await request.post("/api/tracking/source-name", {
      data: {
        schema_version: 3,
        request_id: "browser-device-name-restore",
        source_id: source!.source_id,
        custom_name: source!.custom_name,
      },
    });
  }
});

test("perception page uses the simulation camera through the canonical path", async ({
  page,
  request,
}) => {
  const before = await (await request.get("/api/perception/state")).json();
  const original = before.values.perception_state;
  try {
    await request.post("/api/perception/request", {
      data: {
        schema_version: 3,
        request_id: "browser-perception-prompts",
        action: "apply",
        source_id: null,
        depth_scale_m: null,
        classes: ["red cube", "gray storage bin"],
        placement_labels: ["gray storage bin"],
      },
    });
    await page.goto("/perception/");
    const camera = page.getByLabel("相机来源");
    await camera.selectOption("simulation:depth-grid");
    const refreshImage = page.getByRole("button", { name: "刷新图像" });
    await expect(refreshImage).toBeEnabled();
    await refreshImage.click();
    await expect
      .poll(async () => {
        const state = await (await request.get("/api/perception/state")).json();
        return {
          sourceId: state.values.perception_state?.source_id,
          pointCount: state.values.perception_state?.point_count,
          objectCount: state.values.world_scene?.objects.length,
        };
      })
      .toEqual({
        sourceId: "simulation:depth-grid",
        pointCount: 497,
        objectCount: 0,
      });
    await camera.selectOption("simulation:pick-place-scene");
    await expect(page.getByText("运行", { exact: true })).toBeVisible();
    await expect(page.getByText("red cube", { exact: true })).toBeVisible();
    await camera.selectOption("");
    await expect(camera).toHaveValue("");
    await expect
      .poll(async () => {
        const state = await (await request.get("/api/perception/state")).json();
        return {
          sourceId: state.values.perception_state?.source_id,
          objectCount: state.values.world_scene?.objects.length,
        };
      })
      .toEqual({ sourceId: null, objectCount: 0 });
    await expect(page.getByText("停用", { exact: true })).toBeVisible();
    await page.reload();
    await expect(camera).toHaveValue("");
    await camera.selectOption("simulation:pick-place-scene");
    await page.reload();
    await expect(page.getByText("运行", { exact: true })).toBeVisible();
  } finally {
    await request.post("/api/perception/request", {
      data: {
        schema_version: 3,
        request_id: "browser-perception-stop",
        action: "disconnect",
        source_id: null,
        depth_scale_m: null,
        classes: null,
        placement_labels: null,
      },
    });
    if (original?.enabled)
      await request.post("/api/perception/request", {
        data: {
          schema_version: 3,
          request_id: "browser-perception-restore",
          action: "apply",
          source_id: original.source_id,
          depth_scale_m: original.depth_scale_m,
          classes: original.classes,
          placement_labels: original.placement_labels,
        },
      });
  }
});

test("card status precedes the collapse button and collapse state persists", async ({
  page,
}) => {
  await page.goto("/perception/");
  await page.evaluate(() =>
    localStorage.removeItem("robot-arm:card:/perception/:相机外参标定"),
  );
  await page.reload();

  const card = page.locator("section.card").filter({
    has: page.getByText("相机外参标定", { exact: true }),
  });
  const status = card.locator(":scope > .card-heading > .card-action");
  const collapse = card.locator(
    ":scope > .card-heading > .card-collapse-toggle",
  );
  const statusBox = await status.boundingBox();
  const collapseBox = await collapse.boundingBox();
  expect(statusBox).not.toBeNull();
  expect(collapseBox).not.toBeNull();
  expect(statusBox!.x + statusBox!.width).toBeLessThanOrEqual(collapseBox!.x);

  await collapse.click();
  await expect(card).toHaveAttribute("data-state", "closed");
  await page.reload();
  await expect(card).toHaveAttribute("data-state", "closed");
});

test("spatial renders a neutral position and orientation before valid input", async ({
  page,
}) => {
  await page.goto("/spatial/");
  const viewer = page.getByLabel("空间节点转换后的空间位置和设备自身姿态");
  await expect(viewer).toHaveAttribute("data-pose-ready", "true");
  await expect(viewer.locator(".pose-displacement")).toContainText(
    "X 0.000 · Y 0.000 · Z 0.000 m",
  );
});

test("motion control modes use separate buttons", async ({ page }) => {
  await page.goto("/motion/");
  const card = page.locator("section.card").filter({
    has: page.getByText("控制模式 / 规划", { exact: true }),
  });
  const buttons = card.locator(".card-actions").first().locator(".button");
  await expect(buttons).toHaveCount(3);
  const boxes = await buttons.evaluateAll((items) =>
    items.map((item) => item.getBoundingClientRect().toJSON()),
  );
  expect(boxes[1].left - boxes[0].right).toBeGreaterThanOrEqual(10);
  expect(boxes[2].left - boxes[1].right).toBeGreaterThanOrEqual(10);
});

test("perception layout groups camera workflow and structured results", async ({
  page,
}) => {
  await page.goto("/perception/");
  const card = (title: string) =>
    page.locator("section.card").filter({
      has: page.getByText(title, { exact: true }),
    });
  const camera = await card("相机来源与配置").boundingBox();
  const task = await card("感知任务").boundingBox();
  const calibration = await card("相机外参标定").boundingBox();
  const parameters = await card("相机内参与外参").boundingBox();
  expect(camera).not.toBeNull();
  expect(task).not.toBeNull();
  expect(calibration).not.toBeNull();
  expect(parameters).not.toBeNull();
  expect(camera!.x).toBeLessThan(task!.x);
  expect(Math.abs(camera!.y - task!.y)).toBeLessThan(2);
  expect(calibration!.x).toBeLessThan(parameters!.x);
  expect(Math.abs(calibration!.y - parameters!.y)).toBeLessThan(2);

  await expect(card("识别与分割模型")).toHaveCount(0);
  const modelSelect = card("感知任务").locator(
    'select[aria-label="提示词模型"]',
  );
  await expect(modelSelect).toHaveValue(/.+/);
  await expect(modelSelect.locator("option")).toHaveCount(1);
  const savePromptButton = await card("感知任务")
    .getByRole("button", { name: "保存提示词配置" })
    .boundingBox();
  const executeButton = await card("感知任务")
    .getByRole("button", { name: "执行抓放" })
    .boundingBox();
  expect(savePromptButton).not.toBeNull();
  expect(executeButton).not.toBeNull();
  expect(
    Math.abs(
      savePromptButton!.x +
        savePromptButton!.width -
        (executeButton!.x + executeButton!.width),
    ),
  ).toBeLessThan(2);
  const overlayCard = card("识别与分割叠加图");
  const overlayImage = overlayCard.getByAltText("识别与分割叠加图");
  if (await overlayImage.count()) {
    await expect(overlayImage).toHaveAttribute("loading", "eager");
    await expect
      .poll(() => overlayImage.evaluate((image) => image.naturalWidth))
      .toBeGreaterThan(0);
  } else {
    await expect(overlayCard.getByText("等待模型输出")).toBeVisible();
  }
  await expect(
    card("识别与分割叠加图").locator("table.data-table"),
  ).toHaveCount(0);
  await expect(card("结构化三维场景").locator("table.data-table")).toHaveCount(
    1,
  );
  const overlay = await card("识别与分割叠加图").boundingBox();
  const structuredScene = await card("结构化三维场景").boundingBox();
  expect(overlay).not.toBeNull();
  expect(structuredScene).not.toBeNull();
  expect(overlay!.x).toBeLessThan(structuredScene!.x);
  expect(Math.abs(overlay!.y - structuredScene!.y)).toBeLessThan(2);
});

test("virtual feedback is selectable, draggable, and visible across pages", async ({
  page,
  request,
}) => {
  const before = await (await request.get("/api/tracking/state")).json();
  const inputBindings = (before.values.discovery_state?.bindings ?? [])
    .filter(
      (binding: { configured_components: string[] }) =>
        binding.configured_components.length > 0,
    )
    .map(
      (binding: {
        action: string;
        action_type: string;
        configured_components: string[];
        invert: boolean;
        source_id: string | null;
      }) => ({
        action: binding.action,
        action_type: binding.action_type,
        source_id: binding.source_id,
        component_paths: binding.configured_components,
        invert: binding.invert,
      }),
    );
  const feedbackBindings = (
    before.values.discovery_state?.feedback_bindings ?? []
  ).map(
    (binding: {
      action: string;
      source_id: string;
      capability_path: string;
    }) => ({
      action: binding.action,
      source_id: binding.source_id,
      capability_path: binding.capability_path,
    }),
  );

  try {
    await page.goto("/tracking/");
    const field = page.locator("details.action-binding-item").filter({
      has: page.getByText("夹爪力度反馈", { exact: true }),
    });
    await field.locator("summary").click();
    await field.getByLabel("夹爪力度反馈设备").selectOption("virtual-feedback");
    const applied = page.waitForResponse(
      (response) =>
        response.url().endsWith("/api/tracking/bindings") &&
        response.request().method() === "POST",
    );
    await page.getByRole("button", { name: "应用绑定" }).click();
    await applied;

    const meter = page.getByRole("meter", { name: "网页虚拟力度反馈" });
    await expect(meter).toBeVisible();
    const beforeDrag = await meter.boundingBox();
    expect(beforeDrag).not.toBeNull();
    await meter.hover();
    await page.mouse.down();
    await page.mouse.move(beforeDrag!.x + 70, beforeDrag!.y + 50);
    await page.mouse.up();
    const afterDrag = await meter.boundingBox();
    expect(afterDrag?.x).not.toBe(beforeDrag?.x);
    expect(afterDrag?.y).not.toBe(beforeDrag?.y);

    await page.goto("/spatial/");
    await expect(
      page.getByRole("meter", { name: "网页虚拟力度反馈" }),
    ).toBeVisible();
  } finally {
    await request.post("/api/tracking/bindings", {
      data: {
        schema_version: 3,
        request_id: "browser-virtual-feedback-restore",
        bindings: inputBindings,
        feedback_bindings: feedbackBindings,
      },
    });
  }
});

test("execution page renders colored feedback and command models", async ({
  page,
  request,
}) => {
  await request.post("/api/arm-execution/disconnect", {
    data: {
      schema_version: 3,
      request_id: "browser-disconnect",
      action: "disconnect",
      fields: {},
    },
  });
  await page.goto("/arm-execution/");
  const viewer = page.getByLabel("机械臂三维反馈与目标预览");
  await expect(viewer).toHaveAttribute("data-models-loaded", "2");
  await expect(viewer).toHaveAttribute("data-feedback-ready", "true");
  await expect(viewer).toHaveAttribute("data-pick-point-visible", "false");
  await expect(viewer).toHaveAttribute("data-place-point-visible", "false");
  await expect(page.getByText("红点：抓取目标", { exact: true })).toBeVisible();
  await expect(page.getByText("绿点：放置目标", { exact: true })).toBeVisible();
  await expect
    .poll(async () =>
      Number((await viewer.getAttribute("data-styled-meshes")) ?? 0),
    )
    .toBeGreaterThan(0);
  await expect(viewer).not.toHaveAttribute("data-error", /.+/);
  const labels = page.getByRole("checkbox", {
    name: "显示关节编号和本次真机参数",
  });
  await labels.focus();
  await page.keyboard.press("Space");
  await expect(labels).toBeChecked();
});

test("execution viewer shows the selected pick and placement points only during manipulation", async ({
  page,
  request,
}) => {
  const before = await (await request.get("/api/perception/state")).json();
  const original = before.values.perception_state;
  try {
    await request.post("/api/arm-execution/disconnect", {
      data: {
        schema_version: 3,
        request_id: "browser-pick-points-disconnect",
        action: "disconnect",
        fields: {},
      },
    });
    await request.post("/api/perception/request", {
      data: {
        schema_version: 3,
        request_id: "browser-pick-points-scene",
        action: "apply",
        source_id: "simulation:pick-place-scene",
        depth_scale_m: null,
        classes: ["red cube", "gray storage bin"],
        placement_labels: ["gray storage bin"],
      },
    });
    await expect
      .poll(
        async () => {
          const state = await (
            await request.get("/api/perception/state")
          ).json();
          return state.values.world_scene;
        },
        { timeout: 30_000 },
      )
      .toMatchObject({
        objects: expect.arrayContaining([
          expect.objectContaining({
            grasp_candidates: expect.arrayContaining([expect.any(Object)]),
          }),
        ]),
        placement_regions: expect.arrayContaining([expect.any(Object)]),
      });
    const state = await (await request.get("/api/perception/state")).json();
    const currentScene = state.values.world_scene;
    const object = currentScene.objects.find(
      (candidate: { grasp_candidates: unknown[] }) =>
        candidate.grasp_candidates.length > 0,
    );
    const placement = currentScene.placement_regions[0];
    await request.post("/api/motion/mode", {
      data: {
        schema_version: 3,
        request_id: "browser-pick-points-mode",
        mode: "perception",
      },
    });
    await page.goto("/arm-execution/");
    const viewer = page.getByLabel("机械臂三维反馈与目标预览");
    const response = await request.post("/api/perception/pick-place", {
      data: {
        schema_version: 3,
        request_id: "browser-pick-points",
        object_id: object.object_id,
        placement_region_id: placement.region_id,
      },
    });
    expect(response.ok()).toBe(true);
    await expect(viewer).toHaveAttribute("data-pick-point-visible", "true", {
      timeout: 15_000,
    });
    await expect(viewer).toHaveAttribute("data-place-point-visible", "true");
    await expect(viewer).toHaveAttribute(
      "data-pick-point-z",
      object.pose.position_m[2].toFixed(3),
    );
    const support = currentScene.objects.find(
      (candidate: { object_id: string }) =>
        candidate.object_id === placement.source_object_id,
    );
    const expectedPlaceZ = support
      ? support.pose.position_m[2] +
        support.size_m[2] / 2 +
        object.size_m[2] / 2
      : placement.pose.position_m[2];
    await expect(viewer).toHaveAttribute(
      "data-place-point-z",
      expectedPlaceZ.toFixed(3),
    );
    await expect
      .poll(
        async () => {
          const result = await (
            await request.get("/api/arm-execution/state")
          ).json();
          const manipulation = result.values.manipulation_state;
          return manipulation?.request_id === "browser-pick-points"
            ? manipulation.state
            : undefined;
        },
        { timeout: 180_000 },
      )
      .toMatch(/^(succeeded|failed|cancelled)$/);
    await expect(viewer).toHaveAttribute("data-pick-point-visible", "false", {
      timeout: 15_000,
    });
    await expect(viewer).toHaveAttribute("data-place-point-visible", "false");
  } finally {
    await request.post("/api/perception/request", {
      data: original?.enabled
        ? {
            schema_version: 3,
            request_id: "browser-pick-points-restore",
            action: "apply",
            source_id: original.source_id,
            depth_scale_m: original.depth_scale_m,
            classes: original.classes,
            placement_labels: original.placement_labels,
          }
        : {
            schema_version: 3,
            request_id: "browser-pick-points-restore",
            action: "disconnect",
            source_id: null,
            depth_scale_m: null,
            classes: null,
            placement_labels: null,
          },
    });
  }
});

test("spatial component switches use the server state and keyboard", async ({
  page,
  request,
}) => {
  const initial = await (await request.get("/api/spatial/state")).json();
  const original = initial.values.spatial_config_state.switches;
  await page.goto("/spatial/");
  const translation = page.getByRole("checkbox", { name: "底座坐标平移" });
  try {
    await expect(translation).toBeChecked({ checked: original.translation });
    await translation.focus();
    await page.keyboard.press("Space");
    await expect(translation).toBeChecked({ checked: !original.translation });
    await expect
      .poll(async () => {
        const state = await (await request.get("/api/spatial/state")).json();
        return state.values.spatial_config_state.switches.translation;
      })
      .toBe(!original.translation);
  } finally {
    await request.patch("/api/spatial/config", {
      data: {
        schema_version: 3,
        request_id: "browser-switch-restore",
        patch: { switches: original },
      },
    });
  }
});

test("motion mode keyboard action round-trips through the service", async ({
  page,
  request,
}) => {
  await page.goto("/motion/");
  const modeCard = page
    .locator("section.card")
    .filter({ hasText: "模式 / 规划" });
  const currentMode = modeCard.locator(".key-value strong").first();
  const manual = page.getByRole("button", { name: "手动控制" });
  await manual.focus();
  await page.keyboard.press("Enter");
  await expect(currentMode).toHaveText("手动控制");
  await page.getByRole("button", { name: "相对控制", exact: true }).click();
  await expect(currentMode).toHaveText("相对控制");
  const state = await (await request.get("/api/motion/state")).json();
  expect(state.values.motion_state.control_mode).toBe("relative");
});

test("motion named target is submitted by the metadata-driven page", async ({
  page,
  request,
}) => {
  await request.post("/api/arm-execution/disconnect", {
    data: {
      schema_version: 3,
      request_id: "browser-motion-disconnect",
      action: "disconnect",
      fields: {},
    },
  });
  const before = await (await request.get("/api/motion/state")).json();
  const model = before.values.robot_model_info;
  const previousRequest = before.values.motion_state.latest_motion.request_id;
  const originalMode = before.values.motion_state.control_mode;
  try {
    await request.post("/api/motion/mode", {
      data: {
        schema_version: 3,
        request_id: "browser-named-target-manual-mode",
        mode: "manual",
      },
    });
    await request.post("/api/motion/actuator", {
      data: {
        schema_version: 3,
        request_id: "browser-named-target-open-tool",
        model_revision: model.model_revision,
        actuator_key: model.tool_actuators[0].key,
        position_rad: Math.PI / 4,
      },
    });
    await expect
      .poll(async () => {
        const state = await (await request.get("/api/motion/state")).json();
        return Math.abs(state.values.arm_state.actuators_rad[0] - Math.PI / 4);
      })
      .toBeLessThanOrEqual((2 * Math.PI) / 180);
    await page.goto("/motion/");
    await expect(
      page.getByRole("button", { name: "当前为手动控制", exact: true }),
    ).toBeDisabled();
    await expect(
      page.getByRole("button", { name: "准备相对控制" }),
    ).toBeVisible();
    await expect(page.getByRole("button", { name: "默认位" })).toBeVisible();
    await expect(page.getByRole("button", { name: "工作位" })).toBeVisible();
    await page.getByRole("button", { name: "工作位" }).click();
    await expect
      .poll(
        async () => {
          const state = await (await request.get("/api/motion/state")).json();
          const motion = state.values.motion_state.latest_motion;
          return {
            changed: motion.request_id !== previousRequest,
            action: motion.acknowledged_action,
            state: motion.state,
          };
        },
        { timeout: 60_000 },
      )
      .toEqual({ changed: true, action: "apply", state: "succeeded" });
    await expect
      .poll(async () => {
        const state = await (await request.get("/api/motion/state")).json();
        return Math.abs(state.values.arm_state.actuators_rad[0]);
      })
      .toBeLessThanOrEqual((2 * Math.PI) / 180);
  } finally {
    await request.post("/api/motion/mode", {
      data: {
        schema_version: 3,
        request_id: "browser-motion-mode-restore",
        mode: originalMode,
      },
    });
  }
});

test("motion actuator slider submits and restores a software command", async ({
  page,
  request,
}) => {
  await request.post("/api/arm-execution/disconnect", {
    data: {
      schema_version: 3,
      request_id: "browser-actuator-disconnect",
      action: "disconnect",
      fields: {},
    },
  });
  await expect
    .poll(async () => {
      const state = await (await request.get("/api/motion/state")).json();
      return state.values.motion_state.service.has_output;
    })
    .toBe(true);
  await page.goto("/motion/");
  await page.getByRole("button", { name: "手动控制" }).click();
  const slider = page.locator('input[type="range"]').last();
  await expect(slider).toBeVisible();
  const original = Number(await slider.inputValue());
  const minimum = Number(await slider.getAttribute("min"));
  const maximum = Number(await slider.getAttribute("max"));
  const endpoint =
    Math.abs(original - minimum) >= Math.abs(maximum - original)
      ? minimum
      : maximum;
  const commitEndpoint = async (value: number) => {
    await slider.evaluate((element) => {
      const input = element as HTMLInputElement;
      const capture = (event: KeyboardEvent) => {
        if (!event.key.startsWith("Arrow")) return;
        input.dataset.committedValue = input.value;
        input.removeEventListener("keyup", capture);
      };
      input.addEventListener("keyup", capture);
    });
    await slider.focus();
    await page.keyboard.press(value === minimum ? "Home" : "End");
    await page.keyboard.press(value === minimum ? "ArrowRight" : "ArrowLeft");
    return Number(await slider.getAttribute("data-committed-value"));
  };
  let target = endpoint;
  try {
    target = await commitEndpoint(endpoint);
    await expect
      .poll(async () => {
        const state = await (
          await request.get("/api/arm-execution/state")
        ).json();
        return state.values.arm_state.actuators_rad[0];
      })
      .toBeCloseTo(target, 6);
    const atTarget = await (
      await request.get("/api/arm-execution/state")
    ).json();
    expect(atTarget.values.arm_state.feedback_source).toBe("software");
    expect(atTarget.values.motion_state.latest_actuator?.state).toBe(
      "succeeded",
    );
  } finally {
    const current = await (
      await request.get("/api/arm-execution/state")
    ).json();
    await request.post("/api/motion/mode", {
      data: {
        schema_version: 3,
        request_id: "browser-actuator-restore-manual-mode",
        mode: "manual",
      },
    });
    await request.post("/api/motion/actuator", {
      data: {
        schema_version: 3,
        request_id: "browser-actuator-restore",
        model_revision: current.values.robot_model_info.model_revision,
        actuator_key: current.values.robot_model_info.tool_actuators.at(-1).key,
        position_rad: original,
      },
    });
    await expect
      .poll(async () => {
        const state = await (
          await request.get("/api/arm-execution/state")
        ).json();
        return state.values.arm_state.actuators_rad[0];
      })
      .toBeCloseTo(original, 5);
  }
});

test("execution serial discovery is explicit and connection errors stay visible", async ({
  page,
  request,
}) => {
  await request.post("/api/arm-execution/disconnect", {
    data: {
      schema_version: 3,
      request_id: "browser-error-disconnect",
      action: "disconnect",
      fields: {},
    },
  });
  await page.goto("/arm-execution/");
  await page.getByRole("button", { name: "刷新串口", exact: true }).click();
  await expect
    .poll(async () => {
      const state = await (
        await request.get("/api/arm-execution/state")
      ).json();
      return {
        action: state.values.execution_request_result?.acknowledged_action,
        error: state.values.execution_request_result?.original_error,
      };
    })
    .toEqual({ action: "discover", error: null });
  await page.getByRole("button", { name: "连接真机", exact: true }).click();
  await expect(page.locator(".error")).toContainText("连接请求缺少 port 字段");
});
