import { expect, test } from "@playwright/test";

test("AI groups segmentation and a minimal Start that generates selected-object grasps before execution", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/perception/state")).json();
  const state = snapshot.values.perception_state;
  Object.assign(state, {
    model: "auto",
    available_models: [{ id: "auto", label: "Automatic", prompt_free: true }],
    task_state: "idle",
    calibrated: false,
    color_frame: {
      width: 1280,
      height: 720,
      encoding: "rgb8",
      frame_id: "optical",
    },
    last_scene_sequence: null,
    last_segmentation_sequence: 9,
    segmentation_frame: {
      width: 1280,
      height: 720,
      encoding: "rgb8",
      frame_id: "optical",
    },
    instances: [],
  });
  snapshot.values.camera_state.streaming = true;
  snapshot.values.world_scene = {
    schema_version: 3,
    sequence: 1,
    frame_id: "base_link",
    sample_time_ns: 1,
    objects: [],
    placement_regions: [],
    obstacles: [],
  };
  let publish = () => {};
  await page.routeWebSocket("**/ws/perception", (socket) => {
    publish = () => socket.send(JSON.stringify(snapshot));
    publish();
  });
  const writes: Record<string, any>[] = [];
  await page.route("**/api/**", async (route) => {
    if (route.request().method() !== "POST") return route.fallback();
    const body = route.request().postDataJSON();
    writes.push(body);
    const path = new URL(route.request().url()).pathname;
    if (path === "/api/motion/mode") {
      expect(body.mode).toBe("perception");
      return route.fulfill({ json: { original_error: null } });
    }
    if (path === "/api/perception/pick-place") {
      expect(body).toMatchObject({
        scene_sequence: 12,
        object_id: "two",
        placement_region_id: "pad",
      });
      return route.fulfill({ json: { original_error: null } });
    }
    expect(new URL(route.request().url()).pathname).toBe(
      "/api/perception/request",
    );
    state.task_state = "succeeded";
    state.task_action = body.action;
    if (body.action === "refresh") {
      state.last_segmentation_sequence = 10;
      state.instances = [
        {
          instance_id: "one",
          label: "one",
          confidence: 0.9,
          bounding_box_xyxy: [1, 2, 3, 4],
          position_m: null,
          size_m: null,
          grasp_candidate_count: 0,
        },
      ];
    } else if (body.action === "reconstruct") {
      expect(body.input_sequence).toBe(10);
      state.last_scene_sequence = 11;
      const pose = { position_m: [0, 0, 0], orientation_xyzw: [0, 0, 0, 1] };
      Object.assign(snapshot.values.world_scene, {
        sequence: 11,
        objects: ["one", "two"].map((id) => ({
          object_id: id,
          label: id,
          confidence: 0.9,
          pose,
          size_m: [0.1, 0.1, 0.1],
          grasp_candidates: [],
        })),
        placement_regions: [
          { region_id: "pad", label: "pad", pose, size_m: [0.2, 0.2, 0.01] },
        ],
      });
    } else if (body.action === "generate_grasps") {
      expect(body.input_sequence).toBe(11);
      expect(body.object_id).toBe("two");
      snapshot.values.world_scene.objects[1].grasp_candidates = [
        {
          confidence: 0.9,
          pose: { position_m: [0, 0, 0], orientation_xyzw: [0, 0, 0, 1] },
        },
      ];
      state.last_scene_sequence = snapshot.values.world_scene.sequence = 12;
      state.instances = [{ instance_id: "two", grasp_candidate_count: 8 }];
      // HTTP is authoritative even when the UI has not received the new scene.
      return route.fulfill({ json: { original_error: null, value: state } });
    } else throw Error(`Unexpected action ${body.action}`);
    publish();
    await route.fulfill({ json: { original_error: null, value: state } });
  });
  await page.goto("/perception/");
  const ai = page.locator("section.card").filter({
    has: page.locator(".card-title-text").filter({ hasText: /^AI$/ }),
  });
  for (const title of ["自动分割", "提示词分割", "手动分割", "抓放场景"])
    await expect(ai.getByText(title, { exact: true })).toBeVisible();
  await page.getByText("抓放场景", { exact: true }).click();
  const grab = page
    .getByText("抓放场景", { exact: true })
    .locator("xpath=ancestor::div[contains(@class, 'disclosure')][1]");
  await expect(grab.locator("select")).toHaveCount(2);
  await expect(grab.locator(".disclosure-content button")).toHaveCount(1);
  await page.getByText("自动分割", { exact: true }).click();
  const segment = page.getByRole("button", {
    name: "运行自动分割",
    exact: true,
  });
  const reconstruct = page.getByRole("button", {
    name: "三维定位",
    exact: true,
  });
  await expect(
    page.getByRole("button", { name: "生成抓取候选", exact: true }),
  ).toHaveCount(0);
  const execute = page.getByRole("button", { name: "启动", exact: true });
  await expect(segment).toBeEnabled(); // No camera extrinsics required.
  await expect(reconstruct).toBeDisabled();
  await segment.click();
  await expect(segment).toBeEnabled();
  await expect(reconstruct).toBeDisabled();
  await expect(execute).toBeDisabled();
  expect(writes.map((x) => x.action)).toEqual(["refresh"]);
  state.calibrated = true;
  publish();
  await reconstruct.click();
  await expect(execute).toBeEnabled();
  expect(writes.map((x) => x.action)).toEqual(["refresh", "reconstruct"]);
  await page.getByLabel("抓取目标", { exact: true }).selectOption("two");
  await execute.click();
  await expect.poll(() => writes.length).toBe(5);
  // HTTP acceptance is not completion. Only matching terminal feedback
  // releases the button; all task writes remain intercepted in this test.
  await expect(execute).toBeDisabled();
  snapshot.values.manipulation_state = {
    ...snapshot.values.manipulation_state,
    request_id: writes[4].request_id,
    state: "succeeded",
    stage: "complete",
    original_error: null,
  };
  publish();
  await expect(execute).toBeEnabled();
  expect(writes.map((x) => x.action)).toEqual([
    "refresh",
    "reconstruct",
    "generate_grasps",
    undefined,
    undefined,
  ]);
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth > innerWidth,
    ),
  ).toBe(false);
});

test("visual reference editor saves image-space prompts through the normal form without inference or motion", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/perception/state")).json();
  const state = snapshot.values.perception_state;
  state.model = "prompted-fixture";
  state.available_models = [
    { id: "prompted-fixture", label: "Prompted", prompt_free: false },
  ];
  state.classes = ["box", "pad"];
  state.last_segmentation_sequence = 1;
  state.segmentation_frame = {
    width: 1280,
    height: 720,
    encoding: "rgb8",
    frame_id: "optical",
  };
  state.placement_labels = ["pad"];
  state.task_state = "idle";
  state.visual_prompt_active = false;
  await page.routeWebSocket("**/ws/perception", (socket) =>
    socket.send(JSON.stringify(snapshot)),
  );
  const png = await page.evaluate(() => {
    const canvas = document.createElement("canvas");
    canvas.width = 1280;
    canvas.height = 720;
    canvas.getContext("2d")!.fillRect(0, 0, 1280, 720);
    return canvas.toDataURL("image/png").split(",")[1];
  });
  const requests: Record<string, any>[] = [];
  await page.route("**/api/**", (route) => {
    if (route.request().method() === "POST") {
      requests.push(route.request().postDataJSON());
      return route.fulfill({ json: { original_error: null } });
    }
    if (
      new URL(route.request().url()).pathname.endsWith(
        "/assets/segmentation-color.png",
      )
    )
      return route.fulfill({
        contentType: "image/png",
        body: Buffer.from(png, "base64"),
      });
    return route.fallback();
  });
  await page.goto("/perception/");
  await page.getByText("高级设置", { exact: true }).click();
  await page.getByText("提示词分割", { exact: true }).click();
  await page.getByLabel("提示方式", { exact: true }).selectOption("visual");
  await page.getByText("视觉示例提示", { exact: true }).click();
  await page
    .getByRole("button", { name: "载入采集图作为示例", exact: true })
    .click();
  const region = page.getByRole("img", {
    name: "视觉示例框选区域",
    exact: true,
  });
  await expect(region).toBeVisible();
  const save = page.getByRole("button", { name: "保存视觉示例", exact: true });
  await expect(save).toBeDisabled();
  await region.scrollIntoViewIfNeeded();
  const bounds = (await region.boundingBox())!;
  async function draw(a: number[], b: number[]) {
    await page.mouse.move(
      bounds.x + a[0] * bounds.width,
      bounds.y + a[1] * bounds.height,
    );
    await page.mouse.down();
    await page.mouse.move(
      bounds.x + b[0] * bounds.width,
      bounds.y + b[1] * bounds.height,
      { steps: 3 },
    );
    await page.mouse.up();
  }
  await draw([0.1, 0.1], [0.3, 0.3]);
  await page.getByLabel("示例类别", { exact: true }).selectOption("1");
  // Reverse dragging must still serialize ordered pixel bounds.
  await region.scrollIntoViewIfNeeded();
  const nextBounds = (await region.boundingBox())!;
  Object.assign(bounds, nextBounds);
  await draw([0.8, 0.7], [0.6, 0.5]);
  await save.click();
  await expect.poll(() => requests.length).toBe(1);
  expect(requests[0]).toMatchObject({
    action: "apply",
    model: "prompted-fixture",
    classes: ["box", "pad"],
    placement_labels: ["pad"],
  });
  expect(requests[0].prompt).toMatchObject({
    kind: "visual",
    reference_image_base64: png,
    class_ids: [0, 1],
  });
  expect(requests[0].prompt.bboxes[0][0]).toBeCloseTo(128, 0);
  expect(requests[0].prompt.bboxes[1][0]).toBeCloseTo(768, 0);
  expect(requests[0].prompt.bboxes[1][3]).toBeCloseTo(504, 0);
  await expect(page.getByText("已保存视觉示例", { exact: true })).toBeVisible();
  await page
    .getByRole("button", { name: "载入采集图作为示例", exact: true })
    .click();
  await expect(save).toBeDisabled();
  expect(requests).toHaveLength(1);
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth > window.innerWidth,
    ),
  ).toBe(false);
});

// All POSTs are intercepted: this tests the real UI, not robot motion or inference.
test("AI default never hides either segmentation model or its configuration", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/perception/state")).json();
  const state = snapshot.values.perception_state;
  state.model = "prompted-fixture";
  state.available_models = [
    { id: "prompted-fixture", label: "Prompted", prompt_free: false },
    { id: "automatic-fixture", label: "Automatic", prompt_free: true },
  ];
  state.classes = ["pager", "square paper"];
  state.placement_labels = ["square paper"];
  state.task_state = "idle";
  state.visual_prompt_active = true;
  await page.routeWebSocket("**/ws/perception", (socket) =>
    socket.send(JSON.stringify(snapshot)),
  );
  const requests: Record<string, unknown>[] = [];
  await page.route("**/api/**", (route) => {
    if (route.request().method() !== "POST") return route.fallback();
    requests.push(route.request().postDataJSON());
    return route.fulfill({ json: { original_error: null } });
  });
  await page.goto("/perception/");
  await page.getByText("高级设置", { exact: true }).click();
  for (const name of ["自动分割", "提示词分割"])
    await page.getByText(name, { exact: true }).click();
  const model = page.getByLabel("AI 默认分割模型", { exact: true });
  const prompts = page.getByLabel("识别与分割提示词", { exact: true });
  const roles = page.getByLabel("放置区域角色", { exact: true });
  await expect(prompts).toHaveValue("pager, square paper");
  await expect(page.getByLabel("提示方式", { exact: true })).toBeVisible();
  await model.selectOption("automatic-fixture");
  await expect(prompts).toBeVisible();
  await expect(roles).toBeVisible();
  await expect(page.getByLabel("提示方式", { exact: true })).toHaveValue(
    "saved",
  );
  expect(requests).toHaveLength(0);
  await page.getByRole("button", { name: "保存设置", exact: true }).click();
  await expect.poll(() => requests.length).toBe(1);
  expect(requests[0]).toMatchObject({
    model: "automatic-fixture",
    classes: null,
    placement_labels: null,
  });
  await model.selectOption("prompted-fixture");
  await expect(prompts).toHaveValue("pager, square paper");
  await expect(roles).toHaveValue("square paper");
  expect(requests).toHaveLength(1);
  const overflow = await page.evaluate(
    () => document.documentElement.scrollWidth > window.innerWidth,
  );
  expect(overflow).toBe(false);
});
