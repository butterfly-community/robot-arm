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
    await expect(page.locator("main")).toBeVisible();
    await page.reload();
    await expect(page.getByRole("heading", { level: 1 })).toHaveText(title);
    await expect(page.locator("pre").first()).not.toHaveText("null");
    expect(errors).toEqual([]);
  });
}

for (const [path] of pages) {
  test(`${path} live snapshots do not clear selected text`, async ({
    page,
  }) => {
    await page.goto(`/${path}/`);
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
  await request.post("/api/arm-execution/disconnect", {
    data: {
      schema_version: 1,
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
    await page.goto("/spatial/");
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
        schema_version: 1,
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
      return state.values.discovery_state?.simulation?.active;
    })
    .toBe(false);
});

test("tracking page applies and displays a Runtime binding", async ({
  page,
  request,
}) => {
  await page.goto("/tracking/");
  const before = await (await request.get("/api/tracking/state")).json();
  const selectedRuntimeSource =
    before.values.discovery_state?.runtime_sources?.find(
      (source: { source_id: string }) =>
        source.source_id === before.values.discovery_state?.selected_source_id,
    );
  test.skip(
    !selectedRuntimeSource,
    "当前 OpenXR Runtime 没有可用输入端点，实体绑定验收留给设备在线时执行",
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
          bound: binding.bound_sources,
          active: binding.active,
        };
      })
      .toEqual({
        configured: [componentPath],
        bound: [componentPath],
        active: true,
      });
  } finally {
    const current = await (await request.get("/api/tracking/state")).json();
    await request.post("/api/tracking/bindings", {
      data: {
        schema_version: 1,
        request_id: "browser-bindings-restore",
        interaction_profile:
          current.values.discovery_state.selected_source.interaction_profile,
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
      schema_version: 1,
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
        schema_version: 1,
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
  const manual = page.getByRole("button", { name: "手动控制" });
  await manual.focus();
  await page.keyboard.press("Enter");
  await expect(page.getByText("当前：manual")).toBeVisible();
  await page.getByRole("button", { name: "相对控制" }).click();
  await expect(page.getByText("当前：relative")).toBeVisible();
  const state = await (await request.get("/api/motion/state")).json();
  expect(state.values.motion_state.control_mode).toBe("relative");
});

test("motion named target is submitted by the metadata-driven page", async ({
  page,
  request,
}) => {
  await request.post("/api/arm-execution/disconnect", {
    data: {
      schema_version: 1,
      request_id: "browser-motion-disconnect",
      action: "disconnect",
      fields: {},
    },
  });
  const before = await (await request.get("/api/motion/state")).json();
  const previousRequest = before.values.motion_state.latest_motion.request_id;
  await page.goto("/motion/");
  await page.getByRole("button", { name: "手动控制" }).click();
  await page.getByRole("button", { name: "起始位" }).click();
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
});

test("execution actuator slider submits and restores a software command", async ({
  page,
  request,
}) => {
  await request.post("/api/arm-execution/disconnect", {
    data: {
      schema_version: 1,
      request_id: "browser-actuator-disconnect",
      action: "disconnect",
      fields: {},
    },
  });
  await page.goto("/arm-execution/");
  const slider = page.locator('input[type="range"]').last();
  await expect(slider).toBeVisible();
  const original = Number(await slider.inputValue());
  const minimum = Number(await slider.getAttribute("min"));
  const maximum = Number(await slider.getAttribute("max"));
  const target = original === minimum ? maximum : minimum;
  const commit = async (value: number) =>
    slider.evaluate((element, next) => {
      const input = element as HTMLInputElement;
      input.value = String(next);
      input.dispatchEvent(new Event("input", { bubbles: true }));
      input.dispatchEvent(new PointerEvent("pointerup", { bubbles: true }));
    }, value);
  try {
    await commit(target);
    await expect
      .poll(async () => {
        const state = await (
          await request.get("/api/arm-execution/state")
        ).json();
        return {
          source: state.values.arm_state.feedback_source,
          status: state.values.motion_state.latest_actuator?.state,
        };
      })
      .toEqual({ source: "software", status: "succeeded" });
    const atTarget = await (
      await request.get("/api/arm-execution/state")
    ).json();
    expect(atTarget.values.arm_state.actuators_rad[0]).toBeCloseTo(target, 12);
  } finally {
    await commit(original);
    await expect
      .poll(async () => {
        const state = await (
          await request.get("/api/arm-execution/state")
        ).json();
        return state.values.arm_state.actuators_rad[0];
      })
      .toBeCloseTo(original, 12);
  }
});

test("execution connection failure exposes the original service error", async ({
  page,
  request,
}) => {
  await request.post("/api/arm-execution/disconnect", {
    data: {
      schema_version: 1,
      request_id: "browser-error-disconnect",
      action: "disconnect",
      fields: {},
    },
  });
  await page.goto("/arm-execution/");
  await page.getByRole("button", { name: "连接", exact: true }).click();
  await expect(page.locator(".error")).toContainText("连接请求缺少 port 字段");
});
