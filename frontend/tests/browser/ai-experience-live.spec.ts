import { expect, test, type Page } from "@playwright/test";
import { randomUUID } from "node:crypto";
import type { AIRun } from "../../web/apps/perception/src/app/ai/types";

// Opt-in: configured model receives test text + initial robot state, no images.
// Requires an idle AI task slot and approval for the configured model endpoint.
// Uses actual UI clicks and persisted service results; no interception or seeding.
test("experience survives a new conversation and refresh, and can be deleted in the UI", async ({
  page,
}) => {
  test.skip(
    process.env.AI_EXPERIENCE_LIVE !== "1",
    "Requires explicit live-model acceptance",
  );
  test.setTimeout(600_000);
  const title = `经验验收-${randomUUID()}`;
  await page.goto("/perception/");

  async function send(page: Page, text: string) {
    await page.getByRole("button", { name: "新会话", exact: true }).click();
    await page.getByLabel("AI 任务", { exact: true }).fill(text);
    const accepted = page.waitForResponse(
      (response) =>
        response.url().endsWith("/perception/api/ai/") &&
        response.request().method() === "POST" &&
        response.request().postDataJSON()?.action === "start",
    );
    await page
      .getByRole("button", { name: "发送 AI 任务", exact: true })
      .click();
    const response = await accepted;
    expect(response.ok()).toBe(true);
    let run = (await response.json()) as AIRun;
    await expect
      .poll(
        async () => {
          const state = await (
            await page.request.get(
              `/perception/api/ai/?session=${run.sessionId}`,
            )
          ).json();
          run = state.runs.find((item: AIRun) => item.id === run.id);
          return Boolean(run.endedAt);
        },
        { timeout: 240_000 },
      )
      .toBe(true);
    expect(run.state, run.error).toBe("succeeded");
    expect(run.requests).toHaveLength(0);
    expect(
      run.calls.every((call) =>
        ["recall_experience", "save_experience"].includes(call.name),
      ),
    ).toBe(true);
    expect(run.calls.every((call) => !call.error)).toBe(true);
    await expect(
      page.locator(`[data-run-id="${run.id}"] .ai-response`),
    ).toHaveText(run.text);
    return run;
  }

  const first = await send(
    page,
    `只做经验库功能验收，不运动、不取图、不调用其他工具或继续历史任务。用 save_experience 新增标题为“${title}”的测试经验，条件注明“仅验收数据，不用于机械臂控制”，结论“测试假设尚未验证”，下一步“核对跨会话读取后删除”，assessment=hypothesis，evidenceCallIds=[]。保存后报告编号并结束。`,
  );
  const saved = first.calls.find((call) => call.name === "save_experience");
  expect(saved).toBeTruthy();
  const id = (saved!.result as { id: string }).id;
  expect(id).toBeTruthy();
  const second = await send(
    page,
    `只做经验读取验收，不运动、不取图、不修改经验、不调用其他工具或继续历史任务。请从当前经验目录中找到标题“${title}”，用 recall_experience 读取详情，说明它是未验证的测试假设，报告来源任务编号后结束。`,
  );
  expect(second.sessionId).not.toBe(first.sessionId);
  expect(
    second.calls.some(
      (call) =>
        call.name === "recall_experience" &&
        (call.result as { id?: string })?.id === id,
    ),
  ).toBe(true);
  await page.reload();
  const disclosure = page.getByRole("button", { name: /^任务经验/ });
  if ((await disclosure.getAttribute("aria-expanded")) !== "true")
    await disclosure.click();
  await page.getByLabel("查找经验", { exact: true }).fill(title);
  await page.getByRole("button", { name: title, exact: true }).click();
  await expect(
    page.getByText("AI 推测 · 待验证", { exact: true }),
  ).toBeVisible();
  await page
    .getByRole("button", { name: `删除经验：${title}`, exact: true })
    .click();
  await expect(
    page.getByRole("button", { name: title, exact: true }),
  ).toHaveCount(0);
  await page.reload();
  // Redis, not just component state, must have been updated by the UI deletion.
  const remaining = await (
    await page.request.get("/perception/api/ai/experiences/")
  ).json();
  expect(remaining.some((item: { id: string }) => item.id === id)).toBe(false);
});
