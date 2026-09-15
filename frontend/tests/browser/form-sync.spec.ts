import {
  expect,
  test,
  type Page,
  type APIRequestContext,
} from "@playwright/test";

// These replay service snapshots into the real pages and intercept ALL writes.
// The read-only live-page audit is separate; no motor or calibration is started.
async function fixture(
  page: Page,
  request: APIRequestContext,
  namespace: string,
) {
  const snapshot = await (await request.get(`/api/${namespace}/state`)).json();
  let publish = () => {};
  await page.routeWebSocket(`**/ws/${namespace}`, (socket) => {
    publish = () => socket.send(JSON.stringify(snapshot));
    publish();
  });
  await page.route("**/api/**", (route) =>
    ["GET", "HEAD"].includes(route.request().method())
      ? route.fallback()
      : route.abort(),
  );
  return { snapshot, publish: () => publish() };
}

test("AI task input and execute button have a real gap at desktop and mobile widths", async ({
  page,
}) => {
  await page.route("**/api/**", (route) =>
    ["GET", "HEAD"].includes(route.request().method())
      ? route.fallback()
      : route.abort(),
  );
  await page.goto("/perception/");
  for (const width of [1600, 1024, 390]) {
    await page.setViewportSize({ width, height: 1000 });
    const input = await page
      .getByLabel("AI 任务", { exact: true })
      .boundingBox();
    const button = await page
      .getByRole("button", { name: "发送 AI 任务", exact: true })
      .boundingBox();
    expect(input).not.toBeNull();
    expect(button).not.toBeNull();
    const gap = button!.y - input!.y - input!.height;
    expect(gap).toBeGreaterThanOrEqual(11);
  }
});

test("binding drafts survive remote updates and failed saves, and can be restored", async ({
  page,
  request,
}) => {
  const f = await fixture(page, request, "tracking");
  const state = f.snapshot.values.discovery_state;
  state.feedback_bindings = [];
  let fail = true;
  await page.route("**/api/tracking/bindings", (route) => {
    if (fail)
      return route.fulfill({ json: { original_error: "测试绑定失败" } });
    state.feedback_bindings = route.request().postDataJSON().feedback_bindings;
    f.publish();
    return route.fulfill({ json: { original_error: null } });
  });
  await page.goto("/tracking/");
  await page
    .locator("details.action-binding-item")
    .filter({ hasText: "夹爪力度反馈" })
    .locator("summary")
    .click();
  const target = page.getByLabel("夹爪力度反馈设备", { exact: true });
  await target.selectOption({ label: "网页虚拟反馈" });
  const chosen = await target.inputValue();
  // Another client changes an unrelated binding while this draft is unsaved.
  state.bindings = [
    {
      action: "translation_x",
      action_type: "float",
      source_id: null,
      invert: true,
      configured_components: [],
    },
  ];
  f.publish();
  await expect(target).toHaveValue(chosen);
  await page.getByRole("button", { name: "应用绑定", exact: true }).click();
  await expect(page.locator('p.error[role="alert"]')).toContainText(
    "测试绑定失败",
  );
  await expect(target).toHaveValue(chosen);
  fail = false;
  await page
    .getByRole("button", { name: "应用失败，重试", exact: true })
    .click();
  await expect(
    page.getByRole("button", { name: "绑定已应用", exact: true }),
  ).toBeDisabled();
  await expect(target).toHaveValue(chosen);
  state.feedback_bindings = [];
  f.publish();
  await expect(target).toHaveValue("");
  await target.selectOption({ label: "网页虚拟反馈" });
  await page
    .getByRole("button", { name: "恢复已保存绑定", exact: true })
    .click();
  await expect(target).toHaveValue("");
});

test("a missing depth preview loads after the snapshot button, without running inference", async ({
  page,
  request,
}) => {
  const f = await fixture(page, request, "perception");
  f.snapshot.values.camera_state.streaming = true;
  f.snapshot.values.perception_state.color_frame = null;
  f.snapshot.values.perception_state.depth_frame = null;
  let ready = false;
  await page.route("**/api/perception/assets/depth.png?*", (route) =>
    ready
      ? route.fulfill({
          contentType: "image/png",
          body: Buffer.from(
            "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVQIHWP4z8DwHwAFgAI/ScLbtAAAAABJRU5ErkJggg==",
            "base64",
          ),
        })
      : route.fulfill({ status: 404, body: "尚未生成" }),
  );
  const actions: string[] = [];
  await page.route("**/api/perception/request", (route) => {
    const body = route.request().postDataJSON();
    actions.push(body.action);
    ready = true;
    f.snapshot.values.perception_request_result = {
      request_id: body.request_id,
    };
    f.publish();
    return route.fulfill({ json: { original_error: null } });
  });
  await page.goto("/perception/");
  const update = page.getByRole("button", {
    name: "更新深度预览",
    exact: true,
  });
  await expect(update).toBeDisabled();
  await expect(update).toHaveAttribute("title", "等待相机首帧");
  expect(actions).toEqual([]);
  Object.assign(f.snapshot.values.perception_state, {
    color_frame: { width: 1280, height: 720, encoding: "rgb8" },
    depth_frame: { width: 1280, height: 720, encoding: "z16le" },
  });
  f.publish();
  await expect(update).toBeEnabled();
  await expect(
    page.getByText("尚未生成预览，点击更新深度预览", { exact: true }),
  ).toBeVisible();
  await page.getByRole("button", { name: "更新深度预览", exact: true }).click();
  const img = page.getByRole("img", { name: "深度图", exact: true });
  await expect
    .poll(() =>
      img.evaluate((image) => (image as HTMLImageElement).naturalWidth),
    )
    .toBe(1);
  expect(actions).toEqual(["snapshot"]);
});

test("AI model default saves independently; failure preserves edits and never runs", async ({
  page,
  request,
}) => {
  const f = await fixture(page, request, "perception");
  const state = f.snapshot.values.perception_state;
  Object.assign(state, {
    model: "text-model",
    available_models: [
      { id: "text-model", label: "Text", prompt_free: false },
      { id: "auto-model", label: "Auto", prompt_free: true },
    ],
    classes: ["old"],
    placement_labels: [],
    calibrated: true,
    color_frame: { width: 1280, height: 720 },
    depth_frame: { width: 1280, height: 720 },
    task_state: "idle",
  });
  f.snapshot.values.camera_state.streaming = true;
  const actions: string[] = [];
  let fail = true;
  await page.route("**/api/perception/request", (route) => {
    const body = route.request().postDataJSON();
    actions.push(body.action);
    if (fail)
      return route.fulfill({ json: { original_error: "测试保存失败" } });
    if (body.action === "apply") {
      state.model = body.model;
      f.publish();
    }
    return route.fulfill({ json: { original_error: null } });
  });
  await page.goto("/perception/");
  await page.getByText("高级设置", { exact: true }).click();
  const model = page.getByLabel("AI 默认分割模型", { exact: true });
  await model.selectOption("auto-model");
  await expect(page.getByText("有待保存修改", { exact: true })).toBeVisible();
  await page.getByRole("button", { name: "保存设置", exact: true }).click();
  await expect(page.locator('p.error[role="alert"]')).toContainText(
    "测试保存失败",
  );
  expect(actions).toEqual(["apply"]);
  await expect(model).toHaveValue("auto-model");
  fail = false;
  await page.getByRole("button", { name: "保存设置", exact: true }).click();
  await expect.poll(() => actions.length).toBe(2);
  expect(actions).toEqual(["apply", "apply"]);
  await expect(page.getByText("有待保存修改", { exact: true })).toHaveCount(0);
  state.model = "text-model";
  state.classes = ["updated remotely"];
  f.publish();
  await expect(model).toHaveValue("text-model");
  await page
    .getByText("提示词分割", { exact: true })
    .locator("xpath=ancestor::button[1]")
    .click();
  await expect(
    page.getByLabel("识别与分割提示词", { exact: true }),
  ).toHaveValue("updated remotely");
});

test("execution settings follow feedback after save without erasing a pending edit", async ({
  page,
  request,
}) => {
  const f = await fixture(page, request, "arm-execution");
  const state = f.snapshot.values.transport_state;
  state.gripper_strength_percent = 20;
  await page.route("**/api/arm-execution/config", (route) => {
    state.gripper_strength_percent = Number(
      route.request().postDataJSON().fields.gripper_strength_percent,
    );
    f.publish();
    return route.fulfill({ json: { original_error: null } });
  });
  await page.goto("/arm-execution/");
  const strength = page.getByLabel("夹持反馈目标（0–100）", { exact: true });
  await expect(strength).toHaveValue("20");
  await strength.fill("21.0");
  state.gripper_strength_percent = 22;
  f.publish();
  await expect(strength).toHaveValue("21.0");
  await page.getByRole("button", { name: "保存执行配置", exact: true }).click();
  await expect(
    page.getByRole("button", { name: "执行配置已保存", exact: true }),
  ).toBeDisabled();
  state.gripper_strength_percent = 23;
  f.publish();
  await expect(strength).toHaveValue("23");
});

test("spatial numbers keep failed edits but follow acknowledged and remote values", async ({
  page,
  request,
}) => {
  const f = await fixture(page, request, "spatial");
  const config = f.snapshot.values.spatial_config_state;
  config.translation_scale = 0.5;
  let fail = true;
  await page.route("**/api/spatial/config", (route) => {
    if (fail)
      return route.fulfill({ json: { original_error: "测试保存失败" } });
    Object.assign(config, route.request().postDataJSON().patch);
    f.publish();
    return route.fulfill({ json: { original_error: null } });
  });
  await page.goto("/spatial/");
  const scale = page.getByLabel("平移倍率", { exact: true });
  await expect(scale).toHaveValue("0.5");
  await scale.fill("0.75");
  await scale.press("Tab");
  await expect(page.locator('p.error[role="alert"]')).toContainText(
    "测试保存失败",
  );
  await expect(scale).toHaveValue("0.75");
  fail = false;
  await scale.focus();
  await scale.press("Tab");
  await expect(page.locator('p.error[role="alert"]')).toHaveCount(0);
  config.translation_scale = 0.6;
  f.publish();
  await expect(scale).toHaveValue("0.6");
});

test("camera profile switching preserves 1 FPS output and shows source-specific calibration", async ({
  page,
  request,
}) => {
  const f = await fixture(page, request, "perception");
  const camera = f.snapshot.values.camera_state;
  const source = structuredClone(
    camera.available_sources.find(
      (item: { source_id: string }) =>
        item.source_id === camera.selected_source_id,
    ) ?? camera.available_sources[0],
  );
  Object.assign(source, {
    source_id: "uncalibrated-fixture",
    available: true,
    profiles: [
      {
        key: "color-30",
        stream: "color",
        width: 1280,
        height: 720,
        frames_per_second: 30,
        pixel_format: "rgb8",
        available: true,
        is_default: true,
      },
      {
        key: "color-60",
        stream: "color",
        width: 1280,
        height: 720,
        frames_per_second: 60,
        pixel_format: "rgb8",
        available: true,
        is_default: false,
      },
      {
        key: "depth-60",
        stream: "depth",
        width: 1280,
        height: 720,
        frames_per_second: 60,
        pixel_format: "z16le",
        available: true,
        is_default: true,
      },
    ],
  });
  camera.available_sources.push(source);
  camera.configurations.push({
    source_id: source.source_id,
    color_profile_key: "color-30",
    depth_profile_key: "depth-60",
    output_frames_per_second: 1,
    driver_parameters: [],
  });
  await page.goto("/perception/");
  await page
    .getByLabel("相机来源", { exact: true })
    .selectOption(source.source_id);
  await page.getByLabel("彩色流", { exact: true }).selectOption("color-60");
  const fps = page.getByLabel("上送频率", { exact: true });
  await expect(fps).toHaveValue("1");
  const status = page.locator(".key-value").filter({ hasText: "参数与标定" });
  await expect(status).toContainText("未标定");
});
