import { expect, test } from "@playwright/test";

type DiscoveryConfig = {
  position_source_id?: string;
  orientation_source_id?: string;
  bindings?: Array<{
    action: string;
    source_id?: string;
    configured_components: string[];
    invert: boolean;
  }>;
  feedback_bindings?: Array<{
    action: string;
    source_id?: string;
    capability_path?: string;
  }>;
};

function inputConfig(discovery: DiscoveryConfig) {
  return {
    positionSource: discovery.position_source_id ?? null,
    orientationSource: discovery.orientation_source_id ?? null,
    bindings: (discovery.bindings ?? []).map((binding) => ({
      action: binding.action,
      sourceId: binding.source_id ?? null,
      components: binding.configured_components,
      invert: binding.invert,
    })),
    feedbackBindings: (discovery.feedback_bindings ?? []).map((binding) => ({
      action: binding.action,
      sourceId: binding.source_id ?? null,
      capabilityPath: binding.capability_path ?? null,
    })),
  };
}

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
  ["tracking", "输入采集"],
  ["spatial", "空间转换"],
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
    await expect(page.getByRole("navigation").getByRole("link")).toHaveCount(4);
    const card = page.locator("section.card").first();
    const toggle = card.locator(".card-toggle");
    await expect(toggle).toHaveAttribute("aria-expanded", "true");
    await toggle.click();
    await expect(toggle).toHaveAttribute("aria-expanded", "false");
    await expect(card.locator(".card-content")).toBeHidden();
    await toggle.click();
    await expect(card.locator(".card-content")).toBeVisible();
    const diagnostics = page
      .locator(".card-toggle")
      .filter({ hasText: "排障数据" });
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
    await page.locator(".card-toggle").filter({ hasText: "排障数据" }).click();
    const details = page.locator("details.diagnostics").first();
    await details.locator("summary").click();
    const state = page.locator("pre").first();
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

test("simulation control uses the normal input and spatial path", async ({
  page,
  request,
}) => {
  const initialTracking = await (
    await request.get("/api/tracking/state")
  ).json();
  const initialDiscovery = initialTracking.values.discovery_state;
  const originalInputConfig = inputConfig(initialDiscovery);
  await request.post("/api/arm-execution/disconnect", {
    data: {
      schema_version: 2,
      request_id: "browser-simulation-disconnect",
      action: "disconnect",
      fields: {},
    },
  });
  await page.goto("/tracking/");
  try {
    const simulationStarted = page.waitForResponse(
      (response) =>
        response.url().endsWith("/api/tracking/simulation") &&
        response.request().method() === "POST",
    );
    await page.getByRole("button", { name: "启动模拟数据" }).click();
    await simulationStarted;
    await expect(
      page.getByRole("button", { name: "停止模拟数据" }),
    ).toBeVisible();
    const renderCadence = await page.evaluate(async () => {
      const metrics = [...document.querySelectorAll(".metric")];
      const rate = metrics
        .find((metric) => metric.textContent?.includes("采集频率"))
        ?.querySelector("strong");
      if (!rate) throw new Error("缺少采集频率指标");
      let rateRenders = 0;
      const rateObserver = new MutationObserver(() => rateRenders++);
      rateObserver.observe(rate, {
        childList: true,
        subtree: true,
        characterData: true,
      });
      await new Promise((resolve) => window.setTimeout(resolve, 1200));
      rateObserver.disconnect();
      return { rateRenders };
    });
    expect(renderCadence.rateRenders).toBeLessThanOrEqual(2);
    const inputTest = page.locator("section.card").filter({
      has: page.getByText("输入测试", { exact: true }),
    });
    await inputTest
      .getByLabel("选择测试设备")
      .selectOption("simulation:generic-6dof-cycle");
    await expect(inputTest.getByText(/button\/start_stop/)).toBeVisible();
    await expect
      .poll(async () => {
        const response = await request.get("/api/spatial/state");
        const state = await response.json();
        return {
          positionSource: state.values.spatial_config_state?.position_source_id,
          orientationSource:
            state.values.spatial_config_state?.orientation_source_id,
          active: state.values.transformed_control?.active,
        };
      })
      .toEqual({
        positionSource: "simulation:generic-6dof-cycle",
        orientationSource: "simulation:generic-6dof-cycle",
        active: true,
      });
    await expect
      .poll(async () => {
        const response = await request.get("/api/tracking/state");
        const state = await response.json();
        const discovery = state.values.discovery_state;
        const source = discovery.sources.find(
          (candidate: { source_id: string }) =>
            candidate.source_id === "simulation:generic-6dof-cycle",
        );
        return {
          positionCapable: source?.position_capable,
          orientationCapable: source?.orientation_capable,
          components: source?.available_components?.map(
            (component: { path: string }) => component.path,
          ),
          controlBinding: discovery.bindings.find(
            (binding: { action: string }) => binding.action === "start_stop",
          )?.configured_components,
          primaryToolBinding: discovery.bindings.find(
            (binding: { action: string }) => binding.action === "primary_tool",
          )?.configured_components,
          feedbackBinding: discovery.feedback_bindings.find(
            (binding: { action: string }) => binding.action === "primary_tool",
          ),
        };
      })
      .toMatchObject({
        positionCapable: true,
        orientationCapable: true,
        components: ["button/start_stop", "axis/primary_tool"],
        controlBinding: ["button/start_stop"],
        primaryToolBinding: ["axis/primary_tool"],
        feedbackBinding: {
          source_id: "virtual-feedback",
          capability_path: "feedback/virtual",
          applicable: true,
        },
      });
    await expect(
      page.getByRole("meter", { name: "网页虚拟力度反馈" }),
    ).toBeVisible();
    await expect(page.getByLabel("三维空间位置和设备自身姿态")).toHaveCount(0);
    await page.goto("/spatial/");
    const spatialViewer = page.getByLabel(
      "空间节点转换后的空间位置和设备自身姿态",
    );
    await expect(spatialViewer).toHaveAttribute("data-pose-ready", "true");
    for (const label of ["前后", "左右", "上下"]) {
      await expect(page.getByText(label, { exact: true })).toBeVisible();
    }
  } finally {
    await request.post("/api/tracking/simulation", {
      data: {
        schema_version: 2,
        request_id: "browser-simulation-stop",
        enabled: false,
      },
    });
  }
  await page.goto("/tracking/");
  await expect(
    page.getByRole("button", { name: "启动模拟数据" }),
  ).toBeVisible();
  await expect
    .poll(async () => {
      const response = await request.get("/api/tracking/state");
      const state = await response.json();
      const discovery = state.values.discovery_state;
      return {
        active: discovery?.simulation?.active,
        hasSimulationSource: discovery?.sources?.some(
          (source: { source_id: string }) =>
            source.source_id === "simulation:generic-6dof-cycle",
        ),
        inputConfig: inputConfig(discovery),
      };
    })
    .toEqual({
      active: false,
      hasSimulationSource: false,
      inputConfig: originalInputConfig,
    });
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
  const firstBinding = bindingCard.locator(".binding-grid .field").first();
  const firstBindingControls = firstBinding.locator(".binding-row");
  await expect(firstBinding).toHaveCSS("border-top-width", "0px");
  await expect(firstBindingControls).toHaveCSS("border-top-width", "1px");
  const titleBox = await firstBinding
    .locator("legend .localized-label > span")
    .first()
    .boundingBox();
  const controlsBox = await firstBindingControls.boundingBox();
  expect(titleBox).not.toBeNull();
  expect(controlsBox).not.toBeNull();
  expect(
    (controlsBox?.y ?? 0) - ((titleBox?.y ?? 0) + (titleBox?.height ?? 0)),
    "功能标题与控件线框之间应保留明确间距",
  ).toBeGreaterThanOrEqual(10);

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
    const field = page.locator("fieldset.field").filter({
      has: page.getByText("启动和停止控制", { exact: true }),
    });
    const sourceSelect = field.locator("select").nth(0);
    await sourceSelect.selectOption(selectedRuntimeSource.source_id);
    await page.waitForTimeout(300);
    await expect(sourceSelect).toHaveValue(selectedRuntimeSource.source_id);
    const componentSelect = field.locator("select").nth(2);
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
          active: binding.active,
        };
      })
      .toEqual({
        configured: [componentPath],
        active: true,
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
        schema_version: 2,
        request_id: "browser-bindings-restore",
        bindings: original,
        feedback_bindings: feedbackBindings,
      },
    });
  }
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
    const field = page.locator("fieldset.field").filter({
      has: page.getByText("夹爪力度回馈", { exact: true }),
    });
    await field.locator("select").first().selectOption("virtual-feedback");
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
        schema_version: 2,
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
      schema_version: 2,
      request_id: "browser-disconnect",
      action: "disconnect",
      fields: {},
    },
  });
  await page.goto("/arm-execution/");
  const viewer = page.getByLabel("机械臂三维反馈与目标预览");
  await expect(viewer).toHaveAttribute("data-models-loaded", "2");
  await expect(viewer).toHaveAttribute("data-feedback-ready", "true");
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

test("spatial component switches use the server state and keyboard", async ({
  page,
  request,
}) => {
  const initial = await (await request.get("/api/spatial/state")).json();
  const original = initial.values.spatial_config_state.switches;
  await page.goto("/spatial/");
  const translation = page.getByRole("checkbox", { name: "空间位置移动" });
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
        schema_version: 2,
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
      schema_version: 2,
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
    await request.post("/api/motion/actuator", {
      data: {
        schema_version: 2,
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
    await page.getByRole("button", { name: "手动控制" }).click();
    await expect(
      page.getByRole("button", { name: "准备相对控制" }),
    ).toBeVisible();
    await expect(page.getByRole("button", { name: "默认位" })).toBeVisible();
    await expect(page.getByRole("button", { name: "测试位" })).toBeVisible();
    await page.getByRole("button", { name: "测试位" }).click();
    await expect
      .poll(async () => {
        const state = await (await request.get("/api/motion/state")).json();
        const motion = state.values.motion_state.latest_motion;
        return {
          changed: motion.request_id !== previousRequest,
          action: motion.acknowledged_action,
          state: motion.state,
        };
      })
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
        schema_version: 2,
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
      schema_version: 2,
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
    await request.post("/api/motion/actuator", {
      data: {
        schema_version: 2,
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
      schema_version: 2,
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
