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
  const initialSpatial = await (await request.get("/api/spatial/state")).json();
  const originalOrigin =
    initialSpatial.values.spatial_config_state.origin_position_m;
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
    await page.getByRole("button", { name: "启动模拟数据" }).click();
    await expect(
      page.getByRole("button", { name: "停止模拟数据" }),
    ).toBeVisible();
    await expect
      .poll(async () => {
        const response = await request.get("/api/spatial/state");
        const state = await response.json();
        return {
          source: state.values.spatial_config_state?.selected_source_id,
          active: state.values.relative_motion?.active,
        };
      })
      .toEqual({ source: "simulation:standard-spatial-cycle", active: true });
    const trackingViewer = page.getByLabel("三维空间位置和设备自身姿态");
    await expect(trackingViewer).toHaveAttribute("data-pose-ready", "true");
    await expect(trackingViewer.locator("canvas")).toBeVisible();
    await page.goto("/spatial/");
    const spatialViewer = page.getByLabel("映射后空间位置和设备自身姿态");
    await expect(spatialViewer).toHaveAttribute("data-pose-ready", "true");
    await page.getByRole("button", { name: "确认当前位置为原点" }).click();
    await expect
      .poll(async () => {
        const response = await request.get("/api/spatial/state");
        const state = await response.json();
        return {
          origin: state.values.spatial_config_state?.origin_position_m,
          error: state.values.spatial_request_result?.original_error,
        };
      })
      .toMatchObject({ origin: expect.any(Array), error: null });
  } finally {
    await request.post("/api/tracking/simulation", {
      data: {
        schema_version: 2,
        request_id: "browser-simulation-stop",
        enabled: false,
      },
    });
    await request.patch("/api/spatial/config", {
      data: {
        schema_version: 2,
        request_id: "browser-origin-restore",
        patch: { origin_position_m: originalOrigin },
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
      return state.values.discovery_state?.simulation?.active;
    })
    .toBe(false);
});

test("tracking page applies and displays a controller binding", async ({
  page,
  request,
}) => {
  await page.goto("/tracking/");
  const before = await (await request.get("/api/tracking/state")).json();
  const selectedRuntimeSource = before.values.discovery_state?.sources?.find(
    (source: { source_id: string }) =>
      source.source_id === before.values.discovery_state?.selected_source_id,
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
      }) => ({
        action: binding.action,
        action_type: binding.action_type,
        component_paths: binding.configured_components,
        invert: binding.invert,
      }),
    );
  try {
    const state = await (await request.get("/api/tracking/state")).json();
    expect(state.values.discovery_state.selected_source).toBeTruthy();
    const field = page.locator("fieldset.field").filter({
      has: page.getByText("接管控制", { exact: true }),
    });
    const componentSelect = field.locator("select").nth(1);
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
    await expect
      .poll(async () => {
        const current = await (await request.get("/api/tracking/state")).json();
        const binding = current.values.discovery_state.bindings.find(
          (item: { action: string }) => item.action === "control_active",
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
    await request.post("/api/tracking/bindings", {
      data: {
        schema_version: 2,
        request_id: "browser-bindings-restore",
        source_id: current.values.discovery_state.selected_source.source_id,
        bindings: original,
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
  await page.getByRole("button", { name: "相对控制" }).click();
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
  const previousRequest = before.values.motion_state.latest_motion.request_id;
  const originalMode = before.values.motion_state.control_mode;
  try {
    await page.goto("/motion/");
    await page.getByRole("button", { name: "手动控制" }).click();
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

test("execution connection failure exposes the original service error", async ({
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
  await page.getByRole("button", { name: "连接真机", exact: true }).click();
  await expect(page.locator(".error")).toContainText("连接请求缺少 port 字段");
});
