import { test, expect } from "@playwright/test";

// UI error/state regression only. Every write is blocked; not a hardware test.
test("request lookup renders independent results and refreshes the same ID", async ({
  page,
}) => {
  await page.route("**/api/**", (route) =>
    route.request().method() === "GET" ? route.fallback() : route.abort(),
  );
  let state = "executing",
    terminal = false;
  await page.route("**/api/requests/test-result", (route) =>
    route.fulfill({
      json: {
        schema_version: 3,
        request_id: "test-result",
        session_id: "test",
        operation: "motion_request",
        state,
        terminal,
        value: { result_message: "该请求的控制器结果" },
        original_error: null,
        updated_at_ms: Date.now(),
      },
    }),
  );
  await page.goto("/motion/");
  for (const title of [/^动作结果查询/, /^请求结果/]) {
    const button = page.getByRole("button", { name: title });
    if ((await button.getAttribute("aria-expanded")) !== "true")
      await button.click();
  }
  await page.getByLabel("查询请求编号").fill("test-result");
  const query = page.getByRole("button", { name: "查看请求结果", exact: true });
  const panel = page.locator(".disclosure").filter({ has: query });
  await query.click();
  await expect(panel).toContainText("执行中");
  state = "cancelled";
  terminal = true;
  await expect(panel).toContainText("已取消");
  state = "unknown";
  terminal = false;
  await query.click();
  await expect(panel).toContainText("结果未确认");
  await expect(panel).toContainText("该请求的控制器结果");
});
