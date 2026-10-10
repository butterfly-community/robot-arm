import { expect, test } from "@playwright/test";
import type {
  AIExperience,
  AIRun,
} from "../../web/apps/perception/src/app/ai/types";

// Supplemental UI regression only: all APIs/WebSockets are intercepted. This
// does not replace the opt-in unmocked ai-experience-live acceptance test.
for (const width of [1600, 390]) {
  test(`experience entry, provenance, search, refresh and deletion at ${width}px`, async ({
    page,
  }, testInfo) => {
    const errors: string[] = [];
    page.on("pageerror", (error) => errors.push(error.message));
    await page.setViewportSize({ width, height: 1100 });
    await page.routeWebSocket("**/ws/**", (socket) => socket.close());
    let failDelete = true;
    let failRead = false;
    const run: AIRun = {
      id: "summary-run",
      sessionId: "fixture-session",
      owner: "fixture",
      settings: {
        baseURL: "https://example.com/v1",
        model: "fixture",
        effort: "",
      },
      state: "running",
      input: "任务结束总结",
      images: [],
      text: "动作流程已结束。",
      startedAt: Date.now(),
      updatedAt: Date.now(),
      calls: [],
      requests: [],
      responseModels: [],
      responseIds: [],
      warnings: [],
      experienceSummary: { state: "running", savedIds: [] },
    };
    let items: AIExperience[] = [
      {
        id: "fixture-lesson",
        scope: {
          modelRevision: "fixture-arm",
          mode: "vision",
          feedbackSource: "hardware",
        },
        title: "接近目标时调整关节构型",
        conditions:
          "夹爪已接近目标但 TCP IK 反复失败；适用条件必须结合当前物体与相机重新判断。",
        reflection: {
          lesson: "这是待验证的调整建议，不是抓取成功证明。",
          nextAction: "预览关节姿态后再规划。",
          assessment: "hypothesis",
          model: "fixture-model",
        },
        failure: {
          tool: "move_tcp_absolute",
          input: { position_m: [0.2, 0, 0.18] },
          error: "IK -31",
          runId: "source-run",
          callId: "source-call",
        },
        sources: [
          {
            runId: "source-run",
            sessionId: "source-session",
            callIds: ["source-call"],
          },
        ],
        createdAt: 1,
        updatedAt: 2,
      },
    ];
    await page.route("**/api/**", (route) => {
      const url = new URL(route.request().url());
      if (url.pathname === "/perception/api/ai/experiences/") {
        if (route.request().method() === "DELETE") {
          if (failDelete)
            return route.fulfill({
              status: 400,
              json: { original_error: "删除失败测试" },
            });
          const { id } = route.request().postDataJSON();
          items = items.filter((item) => item.id !== id);
          return route.fulfill({ json: { deleted: true } });
        }
        return failRead
          ? route.fulfill({
              status: 400,
              json: { original_error: "读取失败测试" },
            })
          : route.fulfill({ json: items });
      }
      if (url.pathname.includes("/api/ai/events/"))
        return route.fulfill({
          contentType: "text/event-stream",
          body: `data: ${JSON.stringify([run])}\n\n`,
        });
      if (url.pathname === "/perception/api/ai/")
        return route.fulfill({
          json: {
            settings: {
              baseURL: "https://example.com/v1",
              model: "fixture",
              effort: "",
            },
            keyConfigured: false,
            checks: [],
            sessions: [],
            runs: [run],
          },
        });
      if (route.request().method() !== "GET") return route.abort();
      return route.fulfill({ json: { values: {} } });
    });
    await page.goto("/perception/");
    await expect(
      page.getByText("动作流程已结束，正在总结本轮经验", { exact: true }),
    ).toBeVisible();
    const toggle = page.getByRole("button", { name: /^任务经验/ });
    await expect(toggle).toBeVisible();
    await toggle.click();
    const panel = toggle.locator("..");
    await expect(
      panel.getByText("1 / 1 条经验", { exact: true }),
    ).toBeVisible();
    await page.getByLabel("查找经验", { exact: true }).fill("不会匹配");
    await expect(
      panel.getByText("0 / 1 条经验", { exact: true }),
    ).toBeVisible();
    await page.getByLabel("查找经验", { exact: true }).fill("IK");
    await panel
      .getByRole("button", { name: "接近目标时调整关节构型", exact: true })
      .click();
    await expect(
      panel.getByText("AI 推测 · 待验证", { exact: true }),
    ).toBeVisible();
    await panel.getByText("来源任务与工具调用", { exact: true }).click();
    await expect(
      panel.locator("pre").filter({ hasText: "source-session" }),
    ).toBeVisible();
    await panel.getByText("最近一次失败事实", { exact: true }).click();
    await expect(
      panel.locator("pre").filter({ hasText: "IK -31" }),
    ).toBeVisible();
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth > innerWidth,
      ),
    ).toBe(false);
    const refresh = panel.getByRole("button", {
      name: "刷新经验",
      exact: true,
    });
    const search = panel.getByLabel("查找经验", { exact: true });
    const sr = (await search.boundingBox())!,
      rr = (await refresh.boundingBox())!;
    expect(rr.y - sr.y - sr.height).toBeGreaterThanOrEqual(12);
    await panel.screenshot({
      path: testInfo.outputPath(`experience-${width}.png`),
    });
    const remove = panel.getByRole("button", {
      name: "删除经验：接近目标时调整关节构型",
      exact: true,
    });
    await remove.click();
    await expect(panel.getByRole("alert")).toHaveText("删除失败测试");
    await expect(remove).toBeEnabled();
    expect(items).toHaveLength(1);
    failRead = true;
    await refresh.click();
    await expect(panel.getByRole("alert")).toHaveText("读取失败测试");
    failRead = false;
    await refresh.click();
    await expect(panel.getByRole("alert")).toHaveCount(0);
    failDelete = false;
    await remove.click();
    await expect(
      panel.getByText("0 / 0 条经验", { exact: true }),
    ).toBeVisible();
    await expect(remove).toHaveCount(0);
    await page.reload();
    await expect(page.getByText("0 / 0 条经验", { exact: true })).toBeVisible();
    run.state = "succeeded";
    run.endedAt = Date.now();
    run.experienceSummary = { state: "succeeded", savedIds: [] };
    await page.reload();
    await expect(
      page.getByText("已总结，无新增经验", { exact: true }),
    ).toBeVisible();
    run.experienceSummary = {
      state: "succeeded",
      savedIds: ["fixture-lesson"],
    };
    await page.reload();
    await expect(
      page.getByText("已总结并保存 1 条经验", { exact: true }),
    ).toBeVisible();
    run.experienceSummary = {
      state: "failed",
      savedIds: [],
      error: "总结失败测试",
    };
    run.warnings = ["本轮经验总结未完成：总结失败测试"];
    await page.reload();
    await expect(
      page.getByText("总结失败，原任务结果不变", { exact: true }),
    ).toBeVisible();
    await expect(page.locator('[data-run-id="summary-run"]')).toHaveAttribute(
      "data-run-state",
      "succeeded",
    );
    expect(errors).toEqual([]);
  });
}
