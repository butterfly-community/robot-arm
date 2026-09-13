import { expect, test } from "@playwright/test";

const calibrationFixture = {
  schema_version: 3,
  camera_source_id: "calibration-fixture",
  robot_model_revision: "fixture-v1",
  calibration_tool_id: "fixture-tool",
  board: {
    pattern: "charuco",
    dictionary: "DICT_4X4_50",
    squares_x: 5,
    squares_y: 5,
    square_size_m: 0.015,
    marker_size_m: 0.011,
    measured_width_m: 0.075,
    measured_height_m: 0.075,
  },
  camera_in_base: {
    position_m: [0.3, 0.05, 0.79],
    orientation_xyzw: [0, 0, 0, 1],
  },
  board_in_calibration_tool: {
    position_m: [0, 0, 0],
    orientation_xyzw: [0, 0, 0, 1],
  },
  solver: "fixture-solver",
  solved_at_ns: 1788886732742560000,
  sample_count: 2,
  translation_residuals_m: [0.003, 0.004],
  rotation_residuals_rad: [0.01, 0.02],
};

// UI transitions use the real page with intercepted calibration POSTs: no robot motion.
test("saved calibration survives reload and requires explicit recalibration", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/perception/state")).json();
  const result = structuredClone(calibrationFixture);
  const camera = snapshot.values.camera_state;
  camera.selected_source_id = null;
  camera.saved_calibrations = [result];
  camera.available_sources = [
    {
      source_id: result.camera_source_id,
      display_name: "测试相机",
      available: true,
      profiles: [],
      sensors: [],
      driver_extensions: [],
    },
    {
      source_id: "uncalibrated",
      display_name: "未标定测试相机",
      available: true,
      profiles: [],
      sensors: [],
      driver_extensions: [],
    },
  ];
  camera.streaming = false;
  const session = snapshot.values.calibration_state;
  Object.assign(session, {
    active: false,
    phase: "idle",
    solved_result: null,
    camera_source_id: null,
    observations: [],
    target_count: 0,
  });
  const actions: string[] = [];
  let publish = () => {};
  await page.routeWebSocket("**/ws/perception", (socket) => {
    publish = () => socket.send(JSON.stringify(snapshot));
    publish();
  });
  await page.route("**/api/**", async (route) => {
    if (route.request().method() !== "POST") return route.fallback();
    expect(route.request().url()).toContain("/api/perception/calibration");
    const { action } = route.request().postDataJSON();
    actions.push(action);
    if (action === "start")
      Object.assign(session, {
        active: true,
        phase: "moving",
        camera_source_id: result.camera_source_id,
      });
    if (action === "cancel")
      Object.assign(session, {
        active: false,
        phase: "idle",
        solved_result: null,
      });
    if (action === "apply") {
      camera.saved_calibrations = [session.solved_result];
      Object.assign(session, { active: false, phase: "applied" });
    }
    publish();
    await route.fulfill({ json: { original_error: null } });
  });
  await page.goto("/perception/");
  const saved = page.getByRole("region", { name: "上次已保存标定" });
  await expect(saved).toContainText("0.300000 / 0.050000 / 0.790000");
  await expect(saved).toContainText("3.54 / 4.00");
  await page.reload();
  await expect(saved).toContainText("0.300000 / 0.050000 / 0.790000");
  camera.selected_source_id = result.camera_source_id;
  camera.streaming = true;
  publish();
  await expect(
    page.getByRole("button", { name: "已标定", exact: true }),
  ).toBeDisabled();
  const restart = page.getByRole("button", { name: "重新标定", exact: true });
  await expect(restart).toBeEnabled();
  await restart.click();
  await expect(restart).toBeDisabled();
  await expect(saved).toContainText("0.300000 / 0.050000 / 0.790000");
  await page.getByRole("button", { name: "终止本次标定", exact: true }).click();
  await expect(restart).toBeEnabled();
  await expect(saved).toContainText("3.54 / 4.00");
  await restart.click();
  Object.assign(session, {
    phase: "moving",
    current_target_index: null,
    current_target_key: "work",
    target_count: 2,
    observations: [{}, {}],
    solved_result: {
      ...result,
      camera_in_base: {
        ...result.camera_in_base,
        position_m: [0.31, 0.05, 0.79],
      },
    },
  });
  publish();
  await expect(
    page.getByText("2 / 2 · 2 个样本", { exact: true }),
  ).toBeVisible();
  await expect(
    page.getByRole("button", { name: "确认并应用标定", exact: true }),
  ).toBeDisabled();
  session.phase = "awaiting_confirmation";
  publish();
  await expect(saved).toContainText("0.300000 / 0.050000 / 0.790000");
  await page
    .getByRole("button", { name: "确认并应用标定", exact: true })
    .click();
  await expect(saved).toContainText("0.310000 / 0.050000 / 0.790000");
  await page.reload();
  await expect(saved).toContainText("0.310000 / 0.050000 / 0.790000");
  await page
    .getByLabel("相机来源", { exact: true })
    .selectOption("uncalibrated");
  await expect(saved).toHaveCount(0);
  await expect(restart).toHaveCount(0);
  expect(actions).toEqual(["start", "cancel", "start", "apply"]);
});

test("serial fault is visible above collapsed calibration, preserves saved results and does not auto-resume", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/perception/state")).json();
  const camera = snapshot.values.camera_state;
  camera.saved_calibrations = [structuredClone(calibrationFixture)];
  camera.selected_source_id = calibrationFixture.camera_source_id;
  camera.streaming = true;
  const savedBefore = JSON.stringify(camera.saved_calibrations);
  const session = snapshot.values.calibration_state;
  const fault = "Broken pipe；重新打开同一串口失败：No such file or directory";
  snapshot.values.transport_state = {
    connected: false,
    selected_endpoint: "/dev/ttyUSB0",
    last_error: fault,
  };
  Object.assign(session, {
    active: false,
    phase: "failed",
    current_target_index: 1,
    current_target_key: "calibration-2",
    target_count: 9,
    stage_message: `机械臂连接已断开：${fault}`,
    original_error: `机械臂连接已断开：${fault}。请重新连接后重新开始标定`,
    solved_result: null,
    observations: [{}],
  });
  let publish = () => {};
  const mutations: string[] = [];
  await page.routeWebSocket("**/ws/perception", (socket) => {
    publish = () => socket.send(JSON.stringify(snapshot));
    publish();
  });
  await page.route("**/api/**", (route) => {
    if (route.request().method() !== "POST") return route.fallback();
    mutations.push(route.request().url());
    expect(route.request().url()).toContain("/api/perception/calibration");
    expect(route.request().postDataJSON().action).toBe("start");
    return route.fulfill({ json: { original_error: session.original_error } });
  });
  await page.addInitScript(() => {
    localStorage.setItem("robot-arm:card:/perception/:相机外参标定", "closed");
  });
  await page.goto("/perception/");
  const alert = page.getByRole("alert", { name: "机械臂连接异常" });
  await expect(alert).toBeVisible();
  await expect(alert).toContainText(fault);
  await expect(
    alert.getByRole("link", { name: "机械臂执行页" }),
  ).toHaveAttribute("href", "/arm-execution/");
  await page.getByRole("button", { name: /相机外参标定/ }).click();
  await expect(
    page.getByRole("button", { name: "终止本次标定", exact: true }),
  ).toBeDisabled();
  const restart = page.getByRole("button", { name: "重新标定", exact: true });
  await restart.click();
  await expect.poll(() => mutations.length).toBe(1);
  await expect(restart).toBeEnabled();
  await expect(page.locator('.error[role="alert"]')).toHaveCount(1);
  snapshot.values.transport_state.connected = true;
  snapshot.values.transport_state.last_error = null;
  publish();
  await expect(alert).toHaveCount(0);
  await expect(page.locator('p.error[role="alert"]')).toContainText(
    "本轮标定失败",
  );
  expect(session.active).toBe(false);
  expect(session.observations).toHaveLength(1);
  expect(JSON.stringify(camera.saved_calibrations)).toBe(savedBefore);
  expect(mutations).toHaveLength(1);
});
