import { expect, test, type WebSocketRoute } from "@playwright/test";

// Use the deployed model and meshes; intercept only this browser's control API
// and feedback. Planning failure coverage must not move a connected robot.
test("manual target previews without motion and survives planning failure and feedback", async ({
  page,
  request,
}, testInfo) => {
  // Software WebGL plus desktop/mobile screenshots can exceed the default 30 s.
  test.setTimeout(60_000);
  const snapshot = await (await request.get("/api/motion/state")).json();
  const model = snapshot.values.robot_model_info;
  expect(model.joints.length).toBeGreaterThan(0);
  const feedback = snapshot.values.arm_state;
  snapshot.values.motion_state.latest_motion = { state: "idle" };
  snapshot.values.motion_state.control_mode = "manual";
  let socket: WebSocketRoute | undefined;
  await page.route("**/api/motion/state", (route) =>
    route.fulfill({ json: snapshot }),
  );
  await page.routeWebSocket("**/ws/motion", (route) => {
    socket = route;
    route.send(JSON.stringify(snapshot));
  });
  const commands: {
    path: string;
    body: {
      joints?: { joint_key: string; position_rad: number }[];
      actuators?: { actuator_key: string; position_rad: number }[];
    };
  }[] = [];
  let fail = true;
  await page.route("**/api/motion/*", async (route) => {
    if (route.request().method() !== "POST") return route.fallback();
    const path = new URL(route.request().url()).pathname;
    commands.push({ path, body: route.request().postDataJSON() });
    await route.fulfill({
      json: {
        original_error:
          path.endsWith("/request") && fail
            ? "规划失败：目标不可达（测试响应）"
            : null,
      },
    });
  });
  const errors: string[] = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.setViewportSize({ width: 1600, height: 1100 });
  await page.goto("/motion/");
  const viewer = page.getByLabel("机械臂三维反馈与目标预览");
  await expect(viewer).toHaveAttribute("data-models-loaded", "2");
  await expect(viewer).toHaveAttribute("data-command-visible", "true");
  await expect
    .poll(async () => Number(await viewer.getAttribute("data-preview-meshes")))
    .toBeGreaterThan(0);
  const canvas = viewer.locator("canvas");
  const before = await canvas.screenshot();
  const joints = page.getByRole("slider");
  const first = joints.first();
  // Exercise pointer release as well as keyboard commits.
  await first.click({ position: { x: 20, y: 5 } });
  await first.press("End");
  await first.press("ArrowLeft");
  const target = Number(await first.inputValue());
  const gripper = joints.last();
  await gripper.press("End");
  await gripper.press("ArrowLeft");
  const gripperTarget = Number(await gripper.inputValue());
  expect(commands).toEqual([]);
  const after = await canvas.screenshot({
    path: testInfo.outputPath("edited-target.png"),
  });
  expect(after.equals(before)).toBe(false);

  // Actual feedback keeps changing, but it must not overwrite the draft.
  feedback.joints_rad[0] = model.joints[0].minimum;
  snapshot.values.arm_state = feedback;
  socket!.send(JSON.stringify(snapshot));
  await expect(page.locator(".joint-gauge strong").first()).toHaveText(
    `${((feedback.joints_rad[0] * 180) / Math.PI).toFixed(1)}°`,
  );
  await expect(first).toHaveValue(String(target));
  await expect(gripper).toHaveValue(String(gripperTarget));
  expect(commands).toEqual([]);

  await page.getByRole("button", { name: "执行目标", exact: true }).click();
  await expect(page.locator("p.error[role=alert]")).toContainText(
    "规划失败：目标不可达",
  );
  expect(commands.map((command) => command.path)).toEqual([
    "/api/motion/mode",
    "/api/motion/request",
  ]);
  expect(commands[1].body.joints![0]).toEqual({
    joint_key: model.joints[0].key,
    position_rad: target,
  });
  expect(commands[1].body.actuators!.at(-1)!.position_rad).toBe(gripperTarget);
  socket!.send(JSON.stringify(snapshot));
  await expect(first).toHaveValue(String(target));
  await expect(viewer).toHaveAttribute("data-command-visible", "true");

  fail = false;
  await page.getByRole("button", { name: "执行目标", exact: true }).click();
  await expect(page.locator("p.error[role=alert]")).toHaveCount(0);
  await expect.poll(() => commands.length).toBe(4);
  await expect(first).toHaveValue(String(target));
  await page.getByRole("button", { name: "恢复当前姿态", exact: true }).click();
  await expect(first).toHaveValue(String(feedback.joints_rad[0]));
  expect(commands.length).toBe(4);

  // Named positions remain one-click commands, using the model's own keys.
  const named = model.named_targets[0];
  await page.getByRole("button", { name: named.label, exact: true }).click();
  await expect.poll(() => commands.length).toBe(6);
  expect(
    Object.fromEntries(
      commands[5].body.joints!.map((joint) => [
        joint.joint_key,
        joint.position_rad,
      ]),
    ),
  ).toEqual(named.joint_positions_rad);

  for (const width of [1600, 390]) {
    await page.setViewportSize({ width, height: 1100 });
    await expect
      .poll(() =>
        page.evaluate(
          () => document.documentElement.scrollWidth <= window.innerWidth,
        ),
      )
      .toBe(true);
    await page.screenshot({
      path: testInfo.outputPath(`layout-${width}.png`),
      fullPage: false,
    });
  }
  expect(errors).toEqual([]);
});
