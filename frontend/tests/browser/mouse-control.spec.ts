import { expect, test } from "@playwright/test";

// Intercept this browser's writes. No physical actuator or model inference.
test("mouse pad maps all directions and releases outside the button", async ({
  page,
}, testInfo) => {
  const commands: {
    path: string;
    body: { enabled?: boolean; item?: string; value?: number; mode?: string };
  }[] = [];
  await page.route("**/api/**", (route) => {
    if (route.request().method() !== "POST") return route.fallback();
    commands.push({
      path: new URL(route.request().url()).pathname,
      body: route.request().postDataJSON(),
    });
    return route.fulfill({ json: { original_error: null } });
  });
  await page.goto("/tracking/");
  const directions = [
    ["前进", "move_forward_back", 1],
    ["后退", "move_forward_back", -1],
    ["左移", "move_left_right", 1],
    ["右移", "move_left_right", -1],
    ["上移", "move_up_down", 1],
    ["下移", "move_up_down", -1],
    ["俯仰抬起", "tool_pitch", 1],
    ["俯仰往下", "tool_pitch", -1],
    ["向左转", "tool_yaw", 1],
    ["向右转", "tool_yaw", -1],
    ["轴向逆时针", "tool_roll", 1],
    ["轴向顺时针", "tool_roll", -1],
  ] as const;
  for (const [label, item, value] of directions) {
    const before = commands.length;
    const button = page.getByRole("button", { name: label, exact: true });
    await button.scrollIntoViewIfNeeded();
    const box = (await button.boundingBox())!;
    await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
    await page.mouse.down();
    await expect.poll(() => commands.length).toBe(before + 2);
    expect(commands[before].body.mode).toBe("relative");
    expect(commands[before + 1].body).toMatchObject({
      enabled: true,
      item,
      value,
    });
    await expect(button).toHaveAttribute("aria-pressed", "true");
    await page.mouse.move(0, 0);
    await page.mouse.up();
    await expect.poll(() => commands.length).toBe(before + 3);
    expect(commands.at(-1)!.body.enabled).toBe(false);
    await expect(button).toHaveAttribute("aria-pressed", "false");
  }
  expect(
    commands.every(
      (command) =>
        !command.path.includes("prepare-relative") &&
        !command.path.endsWith("/request"),
    ),
  ).toBe(true);
  for (const width of [1280, 390]) {
    await page.setViewportSize({ width, height: 1000 });
    await page
      .getByText("鼠标方向控制", { exact: true })
      .scrollIntoViewIfNeeded();
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth <= innerWidth,
      ),
    ).toBe(true);
    await page.screenshot({
      path: testInfo.outputPath(`mouse-pad-${width}.png`),
    });
  }
});

test("release during mode switch prevents a delayed movement and keyboard blur releases", async ({
  page,
}) => {
  let finishMode!: () => void;
  let modeSeen = false;
  let delayMode = true;
  const pendingMode = new Promise<void>((resolve) => {
    finishMode = resolve;
  });
  const inputs: { enabled: boolean }[] = [];
  await page.route("**/api/motion/mode", async (route) => {
    modeSeen = true;
    if (delayMode) await pendingMode;
    await route.fulfill({ json: { original_error: null } });
  });
  await page.route("**/api/tracking/simulation", (route) => {
    inputs.push(route.request().postDataJSON());
    return route.fulfill({ json: { original_error: null } });
  });
  await page.goto("/tracking/");
  const up = page.getByRole("button", { name: "上移", exact: true });
  await up.focus();
  await page.keyboard.down("Space");
  await expect.poll(() => modeSeen).toBe(true);
  await page.keyboard.up("Space");
  finishMode();
  delayMode = false;
  await up.focus();
  await page.keyboard.down("Enter");
  await expect.poll(() => inputs.length).toBe(1);
  await page.evaluate(() => window.dispatchEvent(new Event("blur")));
  await expect.poll(() => inputs.length).toBe(2);
  expect(inputs.map((input) => input.enabled)).toEqual([true, false]);
  await page.keyboard.up("Enter");
});
