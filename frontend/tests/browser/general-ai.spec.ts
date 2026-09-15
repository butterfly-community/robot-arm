import { expect, test } from "@playwright/test";
// UI regressions only: every mutating request is intercepted, no hardware actions.
test("AI configuration keeps failed drafts and surfaces unsupported effort without changing model", async ({
  page,
}) => {
  const settings = {
    baseURL: "https://example.com/v1",
    model: "fixture",
    effort: "",
  };
  await page.route("**/api/**", (route) =>
    ["GET", "HEAD"].includes(route.request().method())
      ? route.fallback()
      : route.abort(),
  );
  await page.route("**/perception/api/ai/events/**", (route) =>
    route.fulfill({ contentType: "text/event-stream", body: "data: []\n\n" }),
  );
  await page.route("**/perception/api/ai/?**", (route) =>
    route.fulfill({
      json: {
        settings,
        keyConfigured: true,
        checks: [],
        sessions: [],
        runs: [],
      },
    }),
  );
  let fail = true;
  await page.route("**/perception/api/ai/", (route) => {
    if (route.request().method() !== "POST") return route.fallback();
    const body = route.request().postDataJSON();
    if (body.action === "settings" && !fail) {
      Object.assign(settings, body.settings);
      return route.fulfill({ json: settings });
    }
    return route.fulfill({
      status: 400,
      json: {
        original_error:
          body.action === "check"
            ? "Unsupported reasoning effort: fixture-level"
            : "保存失败测试",
      },
    });
  });
  await page.goto("/perception/");
  if (
    (await page
      .getByRole("button", { name: /^模型与思考配置/ })
      .getAttribute("aria-expanded")) === "false"
  )
    await page.getByRole("button", { name: /^模型与思考配置/ }).click();
  const effort = page.getByLabel("AI 思考强度", { exact: true });
  await effort.fill("fixture-level");
  await page.getByRole("button", { name: "保存 AI 配置", exact: true }).click();
  await expect(page.locator(".ai-task-panel")).toContainText("保存失败测试");
  await expect(effort).toHaveValue("fixture-level");
  await expect(page.getByLabel("通用 AI 模型", { exact: true })).toHaveValue(
    "fixture",
  );
  fail = false;
  await page.getByRole("button", { name: "保存 AI 配置", exact: true }).click();
  await page
    .getByRole("button", { name: "验证连接与能力", exact: true })
    .click();
  await expect(page.locator(".ai-task-panel")).toContainText(
    "Unsupported reasoning effort: fixture-level",
  );
  await page.reload();
  if (
    (await page
      .getByRole("button", { name: /^模型与思考配置/ })
      .getAttribute("aria-expanded")) === "false"
  )
    await page.getByRole("button", { name: /^模型与思考配置/ }).click();
  await expect(effort).toHaveValue("fixture-level");
  await expect(page.getByLabel("通用 AI 模型", { exact: true })).toHaveValue(
    "fixture",
  );
});

test("running robot task survives browser reload and cancellation is not immediately terminal", async ({
  page,
}) => {
  const run = {
    id: "fixture-run",
    sessionId: "fixture-session",
    owner: "fixture",
    settings: {
      baseURL: "https://example.com/v1",
      model: "fixture",
      effort: "",
    },
    state: "running",
    input: "测试任务",
    images: [],
    text: "",
    startedAt: Date.now(),
    updatedAt: Date.now(),
    calls: [],
    requests: [
      {
        id: "original",
        path: "/api/motion/request",
        label: "TCP",
        state: "executing",
      },
    ],
    responseModels: [],
    responseIds: [],
    warnings: [],
  };
  await page.route("**/api/**", (route) =>
    ["GET", "HEAD"].includes(route.request().method())
      ? route.fallback()
      : route.abort(),
  );
  await page.route("**/perception/api/ai/events/**", (route) =>
    route.fulfill({
      contentType: "text/event-stream",
      body: "data: " + JSON.stringify([run]) + "\n\n",
    }),
  );
  await page.route("**/perception/api/ai/?**", (route) =>
    route.fulfill({
      json: {
        settings: run.settings,
        keyConfigured: true,
        checks: [],
        sessions: [
          { id: run.sessionId, title: run.input, updatedAt: run.updatedAt },
        ],
        runs: [run],
      },
    }),
  );
  let starts = 0;
  await page.route("**/perception/api/ai/", (route) => {
    const body = route.request().postDataJSON();
    if (body.action === "start") starts++;
    if (body.action === "stop") run.state = "stopping";
    return route.fulfill({ json: run });
  });
  await page.addInitScript(() =>
    localStorage.setItem("robot-arm:ai-session", "fixture-session"),
  );
  await page.goto("/perception/");
  await expect(page.locator('[data-run-id="fixture-run"]')).toContainText(
    "正在处理",
  );
  // Generic AI and manual pick/place have independent progress. The enclosing
  // card must not repeat the manual task's old idle/success badge for AI work.
  await expect(
    page.locator(".perception-task-card > .card-heading > .card-action"),
  ).toHaveCount(0);
  await page.reload();
  await expect(
    page.getByRole("button", { name: "发送 AI 任务", exact: true }),
  ).toBeDisabled();
  await page
    .getByRole("button", { name: "停止 AI 与当前动作", exact: true })
    .click();
  await expect(page.locator('[data-run-id="fixture-run"]')).toContainText(
    "正在停止，等待当前动作结果",
  );
  run.state = "cancelled";
  run.requests[0].state = "cancelled";
  await expect(page.locator('[data-run-id="fixture-run"]')).toContainText(
    "已取消",
    { timeout: 15000 },
  );
  expect(starts).toBe(0);
  await page
    .getByLabel("AI 历史会话", { exact: true })
    .selectOption(run.sessionId);
  await expect(page.locator('[data-run-id="fixture-run"]')).toContainText(
    "已取消",
  );
});
