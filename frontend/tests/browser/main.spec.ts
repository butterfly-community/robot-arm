import { expect, test } from "@playwright/test";
import { readFileSync } from "node:fs";
import { resolve } from "node:path";

const fixture = JSON.parse(
  readFileSync(
    resolve(
      __dirname,
      "../../../backend/nodes/camera/assets/pick-place-scene-geometry.json",
    ),
    "utf8",
  ),
);
const simulationPrompts = [
  fixture.cube.recognition_prompt,
  fixture.bin.placement_region_prompt,
];
const simulationPlacementLabels = [fixture.bin.placement_region_prompt];

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
    await expect(diagnostics).toHaveAttribute("aria-expanded", "true");
    await expect(page.locator("pre").first()).not.toHaveText("null");
    expect(errors).toEqual([]);
  });
}

test("motion controls use the model-declared motor limits", async ({
  page,
}) => {
  await page.goto("/motion/");
  const sliders = page.getByRole("slider");
  await expect(sliders).toHaveCount(7);
  const ranges = await sliders.evaluateAll((inputs) =>
    inputs.slice(0, 7).map((input) => {
      const range = input as HTMLInputElement;
      return [Number(range.min), Number(range.max)];
    }),
  );
  const degrees = ranges.map(([minimum, maximum]) => [
    (minimum * 180) / Math.PI,
    (maximum * 180) / Math.PI,
  ]);
  const expected = [
    [-110, 110],
    [0, 180],
    [-270, 0],
    [-90, 90],
    [-65, 65],
    [-150, 150],
    [0, 90],
  ];
  expect(degrees).toHaveLength(expected.length);
  for (const [index, limits] of expected.entries()) {
    expect(degrees[index][0]).toBeCloseTo(limits[0], 6);
    expect(degrees[index][1]).toBeCloseTo(limits[1], 6);
  }
});

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
  test.setTimeout(180_000);
  const before = await (await request.get("/api/perception/state")).json();
  const original = before.values.perception_state;
  const originalCamera = before.values.camera_state;
  try {
    await request.post("/api/perception/request", {
      data: {
        schema_version: 3,
        request_id: "browser-perception-prompts",
        action: "apply",
        source_id: null,
        classes: simulationPrompts,
        placement_labels: simulationPlacementLabels,
      },
    });
    await page.goto("/perception/");
    const manualSettings = page
      .getByText("抓放详细配置", { exact: true })
      .locator(
        "xpath=ancestor::div[contains(concat(' ', normalize-space(@class), ' '), ' disclosure ')][1]",
      );
    await expect(manualSettings.locator(".disclosure-toggle")).toHaveAttribute(
      "aria-expanded",
      "false",
    );
    const instructionBox = await page.getByLabel("自然语言任务").boundingBox();
    const instructionButtonBox = await page
      .getByRole("button", { name: "用 AI 执行抓放" })
      .boundingBox();
    expect(instructionBox).not.toBeNull();
    expect(instructionButtonBox).not.toBeNull();
    expect(
      Math.abs(
        instructionBox!.y +
          instructionBox!.height -
          (instructionButtonBox!.y + instructionButtonBox!.height),
      ),
    ).toBeLessThan(2);
    await manualSettings.locator(".disclosure-toggle").click();
    const camera = page.getByLabel("相机来源");
    await page.getByRole("button", { name: "刷新相机列表" }).click();
    await expect(
      camera.locator('option[value="simulation:depth-grid"]'),
    ).toHaveCount(1);
    await camera.selectOption("simulation:depth-grid");
    await expect(page.getByText(/驱动支持的流配置 · \d+ 项/)).toBeVisible();
    // Saved profiles remain visible even when a driver no longer reports them.
    // Select current capabilities explicitly, as a user would after an update.
    for (const label of ["彩色流", "深度流"]) {
      const stream = page.getByRole("combobox", { name: label, exact: true });
      const supported = await stream
        .locator('option:not([disabled]):not([value=""])')
        .first()
        .getAttribute("value");
      await stream.selectOption(supported!);
    }
    await page.getByRole("spinbutton", { name: "上送频率" }).fill("1");
    await page.getByRole("button", { name: "保存并启用" }).click();
    await expect
      .poll(async () => {
        const state = await (await request.get("/api/perception/state")).json();
        return state.values.camera_state?.output_frames_per_second;
      })
      .toBe(1);
    await expect
      .poll(
        async () => {
          const state = await (
            await request.get("/api/perception/state")
          ).json();
          return state.values.camera_state?.skipped_output_frame_count ?? 0;
        },
        { timeout: 4_000 },
      )
      .toBeGreaterThan(0);
    const refreshImage = page.getByRole("button", { name: "刷新图像" });
    await expect(refreshImage).toBeEnabled();
    await refreshImage.click();
    const depthPreview = page.getByRole("img", { name: "深度图" });
    await expect(depthPreview).toBeVisible();
    const depthSource = await depthPreview.getAttribute("src");
    await page.waitForTimeout(1_200);
    await expect(depthPreview).toHaveAttribute("src", depthSource!);
    const runPerception = page.getByRole("button", {
      name: "运行一次感知",
    });
    const runPerceptionOnce = async () => {
      const completed = page.waitForResponse(
        (response) =>
          response.url().endsWith("/api/perception/request") &&
          response.request().postDataJSON()?.action === "refresh",
        { timeout: 60_000 },
      );
      await runPerception.click();
      expect((await (await completed).json()).original_error).toBeNull();
    };
    await expect(runPerception).toBeEnabled();
    await runPerceptionOnce();
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
        pointCount: 0,
        objectCount: 0,
      });
    await camera.selectOption("simulation:pick-place-scene");
    await page.getByRole("button", { name: "保存并启用" }).click();
    await expect(runPerception).toBeEnabled();
    await runPerceptionOnce();
    await expect(
      page.getByText("已配置", { exact: true }).first(),
    ).toBeVisible();
    await expect(page.getByText("red cube", { exact: true })).toBeVisible();
    await page.route("**/perception/api/instruction/", async (route) => {
      await route.fulfill({
        status: 202,
        contentType: "application/json",
        body: JSON.stringify({
          action: "pick_place",
          reason: "",
          perception_prompts: ["red cube", "gray storage bin"],
          placement_labels: ["gray storage bin"],
          object_id: "red-cube-0",
          placement_region_id: "gray-bin-0-interior",
          request_id: "browser-natural-language-task",
          accepted: true,
        }),
      });
    });
    await page.getByLabel("自然语言任务").fill("把红色方块放进灰色置物筐");
    await page.getByRole("button", { name: "用 AI 执行抓放" }).click();
    await expect(
      page.getByText("最近一次 AI 编排", { exact: true }),
    ).toBeVisible();
    await expect(
      page.getByText("red-cube-0 → gray-bin-0-interior"),
    ).toBeVisible();
    await expect(page.getByLabel("识别与分割提示词")).toHaveValue(
      "red cube, gray storage bin",
    );
    await expect(page.getByLabel("放置区域角色")).toHaveValue(
      "gray storage bin",
    );
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
    await expect(page.getByText("未启用", { exact: true })).toBeVisible();
    await page.reload();
    await expect(camera).toHaveValue("");
    await camera.selectOption("simulation:depth-grid");
    await expect(
      page.getByRole("spinbutton", { name: "上送频率" }),
    ).toHaveValue("1");
    await camera.selectOption("");
  } finally {
    await request.post("/api/perception/camera", {
      data: {
        schema_version: 3,
        request_id: "browser-perception-camera-stop",
        action: "unselect",
        source_id: null,
        color_profile_key: null,
        depth_profile_key: null,
      },
    });
    if (originalCamera?.selected_source_id) {
      await request.post("/api/perception/camera", {
        data: {
          schema_version: 3,
          request_id: "browser-perception-camera-restore-select",
          action: "select",
          source_id: originalCamera.selected_source_id,
          color_profile_key: originalCamera.selected_color_profile_key,
          depth_profile_key: originalCamera.selected_depth_profile_key,
          output_frames_per_second:
            originalCamera.output_frames_per_second ?? null,
          driver_parameters:
            originalCamera.configurations?.find(
              (configuration: { source_id: string }) =>
                configuration.source_id === originalCamera.selected_source_id,
            )?.driver_parameters ?? null,
        },
      });
      if (originalCamera.streaming)
        await request.post("/api/perception/camera", {
          data: {
            schema_version: 3,
            request_id: "browser-perception-camera-restore-connect",
            action: "connect",
            source_id: originalCamera.selected_source_id,
            color_profile_key: null,
            depth_profile_key: null,
          },
        });
    }
    if (original?.enabled)
      await request.post("/api/perception/request", {
        data: {
          schema_version: 3,
          request_id: "browser-perception-restore",
          action: "apply",
          source_id: null,
          classes: original.classes,
          placement_labels: original.placement_labels,
        },
      });
  }
});

test("color video floats, drags, collapses, and restores browser state", async ({
  page,
}) => {
  await page.goto("/perception/");
  await page.evaluate(() => {
    localStorage.removeItem("robot-arm:perception:color-video-position");
    localStorage.removeItem("robot-arm:perception:color-video-collapsed");
  });
  await page.reload();

  const monitor = page.getByLabel("彩色视频浮动窗口");
  await expect(monitor).toBeVisible();
  const before = await monitor.boundingBox();
  const header = monitor.locator(".floating-camera-header");
  const headerBox = await header.boundingBox();
  expect(before).not.toBeNull();
  expect(headerBox).not.toBeNull();
  await page.mouse.move(headerBox!.x + 30, headerBox!.y + 24);
  await page.mouse.down();
  await page.mouse.move(headerBox!.x - 50, headerBox!.y - 26, { steps: 5 });
  await page.mouse.up();
  const moved = await monitor.boundingBox();
  expect(moved).not.toBeNull();
  expect(moved!.x).toBeLessThan(before!.x - 40);
  expect(moved!.y).toBeLessThan(before!.y - 20);

  await page.reload();
  const restored = await page.getByLabel("彩色视频浮动窗口").boundingBox();
  expect(restored).not.toBeNull();
  expect(Math.abs(restored!.x - moved!.x)).toBeLessThan(2);
  expect(Math.abs(restored!.y - moved!.y)).toBeLessThan(2);

  await page.getByRole("button", { name: "收起" }).click();
  await expect(
    monitor.getByRole("button", { name: "展开", exact: true }),
  ).toBeVisible();
  await expect(page.locator(".floating-camera-content")).toHaveCount(0);
  await page.reload();
  await expect(
    page
      .getByLabel("彩色视频浮动窗口")
      .getByRole("button", { name: "展开", exact: true }),
  ).toBeVisible();
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
  const task = await card("抓放场景").boundingBox();
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
  await expect(
    card("相机外参标定").getByRole("button", { name: "开始自动标定" }),
  ).toBeVisible();
  await expect(
    card("相机外参标定").getByRole("button", { name: "记录当前姿态样本" }),
  ).toHaveCount(0);
  await expect(
    card("相机外参标定").getByRole("button", { name: "用当前样本求解" }),
  ).toHaveCount(0);

  await expect(card("识别与分割模型")).toHaveCount(0);
  const manualSettings = page
    .getByText("抓放详细配置", { exact: true })
    .locator(
      "xpath=ancestor::div[contains(concat(' ', normalize-space(@class), ' '), ' disclosure ')][1]",
    );
  await expect(manualSettings.locator(".disclosure-toggle")).toHaveAttribute(
    "aria-expanded",
    "false",
  );
  await manualSettings.locator(".disclosure-toggle").click();
  const modelSelect = card("抓放场景").locator(
    'select[aria-label="提示词模型"]',
  );
  await expect(modelSelect).toHaveValue(/.+/);
  await expect(modelSelect.locator("option")).toHaveCount(1);
  // Radix mounts the body while opening; measure after its layout settles.
  await card("抓放场景")
    .getByRole("button", { name: "保存模型配置" })
    .scrollIntoViewIfNeeded();
  const savePromptButton = await card("抓放场景")
    .getByRole("button", { name: "保存模型配置" })
    .boundingBox();
  const runPerceptionButton = await card("抓放场景")
    .getByRole("button", { name: "运行一次感知" })
    .boundingBox();
  const executeButton = await card("抓放场景")
    .getByRole("button", { name: "执行抓放", exact: true })
    .boundingBox();
  expect(savePromptButton).not.toBeNull();
  expect(runPerceptionButton).not.toBeNull();
  expect(executeButton).not.toBeNull();
  expect(
    runPerceptionButton!.x - (savePromptButton!.x + savePromptButton!.width),
  ).toBeGreaterThanOrEqual(10);
  expect(
    Math.abs(
      runPerceptionButton!.x +
        runPerceptionButton!.width -
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
  await expect(page.getByRole("button", { name: "全部卸力" })).toBeDisabled();
  await expect(page.getByRole("button", { name: "全部上力" })).toBeDisabled();
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
  // This test includes real CPU inference and MTC planning (one measured
  // execution took ~60 s), not just DOM interaction. Keep other tests at 30 s.
  test.setTimeout(180_000);
  const before = await (await request.get("/api/perception/state")).json();
  const original = before.values.perception_state;
  const originalCamera = before.values.camera_state;
  try {
    await request.post("/api/arm-execution/disconnect", {
      data: {
        schema_version: 3,
        request_id: "browser-pick-points-disconnect",
        action: "disconnect",
        fields: {},
      },
    });
    await request.post("/api/perception/camera", {
      data: {
        schema_version: 3,
        request_id: "browser-pick-points-camera-refresh",
        action: "refresh",
        source_id: null,
        color_profile_key: null,
        depth_profile_key: null,
      },
    });
    await request.post("/api/perception/camera", {
      data: {
        schema_version: 3,
        request_id: "browser-pick-points-camera-select",
        action: "select",
        source_id: "simulation:pick-place-scene",
        color_profile_key: null,
        depth_profile_key: null,
      },
    });
    await request.post("/api/perception/camera", {
      data: {
        schema_version: 3,
        request_id: "browser-pick-points-camera-connect",
        action: "connect",
        source_id: "simulation:pick-place-scene",
        color_profile_key: null,
        depth_profile_key: null,
      },
    });
    await request.post("/api/perception/request", {
      data: {
        schema_version: 3,
        request_id: "browser-pick-points-scene",
        action: "apply",
        source_id: null,
        classes: simulationPrompts,
        placement_labels: simulationPlacementLabels,
      },
    });
    await expect
      .poll(async () => {
        const state = await (await request.get("/api/perception/state")).json();
        return state.values.perception_state?.color_frame != null;
      })
      .toBe(true);
    await request.post("/api/perception/request", {
      data: {
        schema_version: 3,
        request_id: "browser-pick-points-run",
        action: "refresh",
        source_id: null,
        classes: null,
        placement_labels: null,
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
    const expectedPlaceZ = placement.pose.position_m[2] + 0.07;
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
    await request.post("/api/perception/camera", {
      data: {
        schema_version: 3,
        request_id: "browser-pick-points-camera-unselect",
        action: "unselect",
        source_id: null,
        color_profile_key: null,
        depth_profile_key: null,
      },
    });
    if (originalCamera?.selected_source_id) {
      await request.post("/api/perception/camera", {
        data: {
          schema_version: 3,
          request_id: "browser-pick-points-camera-restore-refresh",
          action: "refresh",
          source_id: null,
          color_profile_key: null,
          depth_profile_key: null,
        },
      });
      await request.post("/api/perception/camera", {
        data: {
          schema_version: 3,
          request_id: "browser-pick-points-camera-restore-select",
          action: "select",
          source_id: originalCamera.selected_source_id,
          color_profile_key: originalCamera.selected_color_profile_key,
          depth_profile_key: originalCamera.selected_depth_profile_key,
          output_frames_per_second:
            originalCamera.output_frames_per_second ?? null,
          driver_parameters:
            originalCamera.configurations?.find(
              (configuration: { source_id: string }) =>
                configuration.source_id === originalCamera.selected_source_id,
            )?.driver_parameters ?? null,
        },
      });
      if (originalCamera.streaming)
        await request.post("/api/perception/camera", {
          data: {
            schema_version: 3,
            request_id: "browser-pick-points-camera-restore-connect",
            action: "connect",
            source_id: originalCamera.selected_source_id,
            color_profile_key: null,
            depth_profile_key: null,
          },
        });
    }
    await request.post("/api/perception/request", {
      data: original?.enabled
        ? {
            schema_version: 3,
            request_id: "browser-pick-points-restore",
            action: "apply",
            source_id: null,
            classes: original.classes,
            placement_labels: original.placement_labels,
          }
        : {
            schema_version: 3,
            request_id: "browser-pick-points-restore",
            action: "disconnect",
            source_id: null,
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
        return Math.abs(state.values.arm_state.actuators_rad[0] - target);
      })
      // Planning feedback is not a bit-exact echo of the requested slider value.
      // Use the user's existing 1–2 degree acceptance, not a new motion limit.
      .toBeLessThanOrEqual((2 * Math.PI) / 180);
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
        return Math.abs(state.values.arm_state.actuators_rad[0] - original);
      })
      .toBeLessThanOrEqual((2 * Math.PI) / 180);
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
