import { expect, test } from "@playwright/test";

// UI transitions use the real page with intercepted calibration POSTs: no robot motion.
test("saved calibration survives reload and requires explicit recalibration", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/perception/state")).json();
  const result = {
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
    phase: "awaiting_confirmation",
    solved_result: {
      ...result,
      camera_in_base: {
        ...result.camera_in_base,
        position_m: [0.31, 0.05, 0.79],
      },
    },
  });
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
