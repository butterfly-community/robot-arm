import { expect, test } from "@playwright/test";

test("saved bindings recover after a whole-stack restart", async ({ page }) => {
  test.skip(
    process.env.CAMERA_RESTART_TEST !== "1",
    "先绑定，再整套重启后单独运行",
  );
  await page.goto("/perception/");
  for (const [name, source] of [
    ["外部摄像头", "simulation:pick-place-scene"],
    ["腕部摄像头", "simulation:depth-grid"],
  ]) {
    await expect(page.getByLabel(`${name}来源`, { exact: true })).toHaveValue(
      source,
    );
    const section = page.getByRole("region", {
      name: `${name}绑定`,
      exact: true,
    });
    await expect(section.getByText("正在采集", { exact: true })).toBeVisible({
      timeout: 30000,
    });
    await expect(
      page.getByLabel(`${name}浮动窗口`, { exact: true }),
    ).toBeVisible();
  }
});

// Actual page -> gateway -> camera workers -> RGB WebSockets. No interception.
// Simulation sources exercise the formal RGB contract, not physical UVC capture.
test("bind two camera roles, preview independently, reload and unbind via the page", async ({
  page,
}) => {
  test.setTimeout(120_000);
  await page.goto("/perception/");
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常", {
    timeout: 60000,
  });
  const depth = page.getByLabel("相机来源", { exact: true });
  await expect(depth).toBeVisible();
  await depth.selectOption("");
  await page.getByRole("button", { name: "刷新相机列表", exact: true }).click();
  await expect(
    page.getByRole("status").filter({ hasText: "当前未选择深度相机" }),
  ).toBeVisible();
  const roles = ["外部摄像头", "腕部摄像头"];
  const sources = ["simulation:pick-place-scene", "simulation:depth-grid"];
  const images: string[] = [];
  for (let i = 0; i < 2; i++) {
    const section = page.getByRole("region", {
      name: `${roles[i]}绑定`,
      exact: true,
    });
    const unbind = section.getByRole("button", {
      name: "解除绑定",
      exact: true,
    });
    if (await unbind.isVisible()) await unbind.click();
    await expect(
      page.getByLabel(`${roles[i]}浮动窗口`, { exact: true }),
    ).toHaveCount(0);
    await section
      .getByLabel(`${roles[i]}来源`, { exact: true })
      .selectOption(sources[i]);
    await section
      .getByLabel(`${roles[i]}流配置`, { exact: true })
      .selectOption("color:1280x720:rgb8:10");
    await section
      .getByRole("button", { name: `保存${roles[i]}绑定`, exact: true })
      .click();
    await expect(section.getByText("正在采集", { exact: true })).toBeVisible({
      timeout: 30000,
    });
    const monitor = page.getByLabel(`${roles[i]}浮动窗口`, { exact: true });
    await expect(monitor).toBeVisible();
    const expand = monitor.getByRole("button", { name: "展开", exact: true });
    if (await expand.isVisible()) await expand.click();
    await expect
      .poll(() =>
        monitor.locator("canvas").evaluate((c: HTMLCanvasElement) => c.width),
      )
      .toBe(1280);
    images.push(
      await monitor
        .locator("canvas")
        .evaluate((c: HTMLCanvasElement) => c.toDataURL()),
    );
    await monitor.getByRole("button", { name: "收起", exact: true }).click();
  }
  // The scene is colored; the depth-grid fixture intentionally has black RGB.
  // Width alone cannot detect accidentally subscribing both roles to one stream.
  expect(images[0]).not.toBe(images[1]);
  await page.reload();
  for (let i = 0; i < 2; i++) {
    await expect(
      page.getByLabel(`${roles[i]}来源`, { exact: true }),
    ).toHaveValue(sources[i]);
    await expect(
      page
        .getByLabel(`${roles[i]}浮动窗口`, { exact: true })
        .getByRole("button", { name: "展开", exact: true }),
    ).toBeVisible();
  }
  await page.setViewportSize({ width: 390, height: 900 });
  expect(
    await page.evaluate(
      () => document.documentElement.scrollWidth > innerWidth,
    ),
  ).toBe(false);
  await page.screenshot({
    path: "../temp/camera-bindings/mobile.png",
    fullPage: true,
  });
  await page.setViewportSize({ width: 1600, height: 1100 });
  // Move each independently and verify the persisted location of both windows.
  const positions: number[] = [];
  for (let i = 0; i < 2; i++) {
    const monitor = page.getByLabel(`${roles[i]}浮动窗口`, { exact: true });
    const header = await monitor.locator("header").boundingBox();
    await page.mouse.move(header!.x + 50, header!.y + 20);
    await page.mouse.down();
    await page.mouse.move(130 + i * 450, 200, { steps: 5 });
    await page.mouse.up();
    positions.push((await monitor.boundingBox())!.x);
  }
  await page.reload();
  for (let i = 0; i < 2; i++) {
    const monitor = page.getByLabel(`${roles[i]}浮动窗口`, { exact: true });
    await expect(monitor).toBeVisible();
    expect((await monitor.boundingBox())!.x).toBeCloseTo(positions[i], 0);
    await monitor.getByRole("button", { name: "展开", exact: true }).click();
    await expect
      .poll(() =>
        monitor.locator("canvas").evaluate((c: HTMLCanvasElement) => c.width),
      )
      .toBe(1280);
  }
  await page.screenshot({
    path: "../temp/camera-bindings/desktop.png",
    fullPage: true,
  });
  // Keep bindings for the optional AI observation check, otherwise clean them.
  if (process.env.CAMERA_BINDINGS_KEEP !== "1") {
    for (const name of roles) {
      await page
        .getByRole("region", { name: `${name}绑定`, exact: true })
        .getByRole("button", { name: "解除绑定", exact: true })
        .click();
      await expect(
        page.getByLabel(`${name}浮动窗口`, { exact: true }),
      ).toHaveCount(0);
    }
    await page.reload();
    for (const name of roles)
      await expect(page.getByLabel(`${name}来源`, { exact: true })).toHaveValue(
        "",
      );
  }
});

test("depth preview still starts and stops through the page", async ({
  page,
}) => {
  test.skip(
    process.env.CAMERA_BINDINGS_KEEP === "1",
    "保留普通角色时不抢占该模拟来源",
  );
  await page.goto("/perception/");
  const source = page.getByLabel("相机来源", { exact: true });
  await source.selectOption("simulation:pick-place-scene");
  await page
    .getByRole("combobox", { name: "彩色流", exact: true })
    .selectOption("color:1280x720:rgb8:10");
  await page
    .getByRole("combobox", { name: "深度流", exact: true })
    .selectOption("depth:1280x720:z16le:10");
  await page.getByRole("spinbutton", { name: "上送频率" }).fill("1");
  await page.getByRole("button", { name: "保存并启用", exact: true }).click();
  const monitor = page.getByLabel("深度相机彩色画面浮动窗口", { exact: true });
  await expect(monitor).toBeVisible();
  await expect
    .poll(() =>
      monitor.locator("canvas").evaluate((c: HTMLCanvasElement) => c.width),
    )
    .toBe(1280);
  await page.getByRole("button", { name: "停用相机", exact: true }).click();
  await expect(monitor.getByText("信号中断", { exact: true })).toBeVisible();
  await expect(monitor.locator("canvas")).toHaveCount(0);
  await source.selectOption("");
  await expect(monitor).toHaveCount(0);
});

test("AI observes both bound roles through the actual conversation", async ({
  page,
}) => {
  test.skip(
    process.env.CAMERA_AI_TEST !== "1",
    "显式启用才向当前模型服务发送模拟相机图像",
  );
  test.setTimeout(240_000);
  await page.goto("/perception/");
  await expect(page.getByLabel("AI 任务", { exact: true })).toBeVisible();
  await page.getByRole("button", { name: "新会话", exact: true }).click();
  await page
    .getByLabel("AI 任务", { exact: true })
    .fill(
      "只读验收，严禁运动和操作夹爪。查询相机绑定，再分别用 observe_camera 读取 external 和 wrist 的新画面。简述每幅图像内容、来源和分辨率，并说明两次采集不是同步双目。不要分割、标定或抓放。",
    );
  await page.getByRole("button", { name: "发送 AI 任务", exact: true }).click();
  const run = page.locator(".ai-run").last();
  await expect(run).toHaveAttribute(
    "data-run-state",
    /^(succeeded|failed|cancelled)$/,
    {
      timeout: 210000,
    },
  );
  expect(await run.getAttribute("data-run-state"), await run.innerText()).toBe(
    "succeeded",
  );
  const details = run.getByRole("button", { name: /工具与执行详情/ });
  if ((await details.getAttribute("aria-expanded")) === "false")
    await details.click();
  const results = await run.locator(".ai-tool details pre").allTextContents();
  for (const role of ["external", "wrist"]) {
    expect(
      results
        .map((text) => JSON.parse(text))
        .some(
          (call) =>
            call.input?.role === role &&
            call.result?.role === role &&
            call.result?.image_id &&
            !call.error,
        ),
    ).toBe(true);
  }
  expect(await run.locator(".ai-tool strong").allTextContents()).not.toEqual(
    expect.arrayContaining([
      expect.stringMatching(/move_|gripper|pick_place|work_pose/),
    ]),
  );
  await page.screenshot({
    path: "../temp/camera-bindings/ai-observation.png",
    fullPage: true,
  });
});
