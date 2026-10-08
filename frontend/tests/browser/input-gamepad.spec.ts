import { expect, test } from "@playwright/test";
import { execFileSync } from "node:child_process";
import { readlinkSync, readFileSync } from "node:fs";

// Real connected hardware, no intercepted requests or injected discovery data.
// Renaming is restored; no action bindings or robot commands are changed.
const name = process.env.INPUT_GAMEPAD_NAME;

test("connected gamepad is visible, editable and keeps sampling after refresh", async ({
  page,
  request,
}, testInfo) => {
  test.skip(!name, "Set INPUT_GAMEPAD_NAME to the SDL device name");
  await page.goto("/tracking/");
  const editor = page.getByRole("textbox", {
    name: `${name} 自定义名称`,
    exact: true,
  });
  await expect(editor).toBeVisible();
  await expect(
    page.getByText("SDL 未发现可用手柄", { exact: false }),
  ).toHaveCount(0);
  const source = editor.locator(
    "xpath=ancestor::div[contains(@class,'source-item')]",
  );
  const original = await editor.inputValue();
  const renamed = `${name} 浏览器验收`;
  try {
    await editor.fill(renamed);
    await source.getByRole("button", { name: "保存名称", exact: true }).click();
    await expect(
      source.getByRole("button", { name: "已保存", exact: true }),
    ).toBeVisible();
    await page.reload();
    await expect(editor).toHaveValue(renamed);
    await expect(source.locator("strong")).toHaveText(renamed);
    // Supplement the actual UI operations with a read-only sample counter check.
    const discovery = async () =>
      (await (await request.get("/api/tracking/state")).json()).values
        .discovery_state;
    const initial = await discovery();
    const pad = initial.sources.find(
      (s: { display_name: string }) => s.display_name === name,
    );
    expect(pad.active).toBe(true);
    expect(pad.original_error).toBeNull();
    const frames = initial.diagnostics.find(
      (d: { source_id: string }) => d.source_id === pad.source_id,
    ).received_frames;
    await expect
      .poll(
        async () =>
          (await discovery()).diagnostics.find(
            (d: { source_id: string }) => d.source_id === pad.source_id,
          ).received_frames,
      )
      .toBeGreaterThan(frames);
    await source.scrollIntoViewIfNeeded();
    await page.screenshot({
      path: testInfo.outputPath("connected-gamepad.png"),
    });
  } finally {
    await editor.fill(original);
    const save = source.getByRole("button", { name: "保存名称", exact: true });
    if (await save.count()) await save.click();
    await expect(
      source.getByRole("button", { name: "已保存", exact: true }),
    ).toBeVisible();
  }
});

test("Xbox reconnect appears on the same page without restarting services", async ({
  page,
}, testInfo) => {
  const usbInterface = process.env.INPUT_GAMEPAD_USB_INTERFACE;
  test.skip(
    !name || !usbInterface,
    "Set INPUT_GAMEPAD_NAME and explicit Xbox USB interface for reconnect acceptance",
  );
  // Test-only driver rebind: equivalent input-device removal/addition, never a
  // broad USB reset. Resolve and validate the exact interface before writing.
  expect(usbInterface).toMatch(/^\d+-[\d.]+:\d+\.\d+$/);
  const target = `/sys/bus/usb/devices/${usbInterface}`;
  expect(readlinkSync(`${target}/driver`).split("/").pop()).toBe("xpad");
  expect(readFileSync(`${target}/../idVendor`, "utf8").trim()).toBe("045e");
  const bind = (action: "bind" | "unbind") =>
    execFileSync("sudo", ["-n", "tee", `/sys/bus/usb/drivers/xpad/${action}`], {
      input: usbInterface,
      stdio: ["pipe", "pipe", "pipe"],
    });
  await page.goto("/tracking/");
  const editor = page.getByRole("textbox", {
    name: `${name} 自定义名称`,
    exact: true,
  });
  await expect(editor).toBeVisible();
  let detached = false;
  try {
    bind("unbind");
    detached = true;
    await expect(editor).toHaveCount(0, { timeout: 15000 });
    await page.screenshot({
      path: testInfo.outputPath("gamepad-disconnected.png"),
    });
    bind("bind");
    detached = false;
    await expect(editor).toBeVisible({ timeout: 15000 });
    await expect(
      page.getByText("SDL 未发现可用手柄", { exact: false }),
    ).toHaveCount(0);
    await editor.scrollIntoViewIfNeeded();
    await page.screenshot({
      path: testInfo.outputPath("gamepad-reconnected.png"),
    });
    await page.reload();
    await expect(editor).toBeVisible();
  } finally {
    if (detached) bind("bind");
  }
});
