// Source: actual AI page and configured model. No request interception, injected
// replies, API writes or robot movement. Read-only posture data goes to the model.
import {
  chromium,
  expect,
} from "../../frontend/node_modules/@playwright/test/index.mjs";
import { mkdir, writeFile } from "node:fs/promises";
import { resolve, join } from "node:path";

const output = resolve(process.argv[2] ?? "temp/ai-conversation");
process.env.TMPDIR = join(output, "browser-temp");
await mkdir(process.env.TMPDIR, { recursive: true });
const base = process.env.SERVICES_BASE_URL ?? "http://192.168.100.10:8765";
const browser = await chromium.launch({ headless: true });
const context = await browser.newContext({
  viewport: { width: 1440, height: 1080 },
});
const records = [];
try {
  const page = await context.newPage();
  const errors = [];
  page.on("pageerror", (error) => errors.push(error.message));
  await page.goto(base + "/perception/");
  const input = page.getByLabel("AI 任务", { exact: true });
  await expect(input).toBeVisible({ timeout: 60000 });
  await expect(page.locator(".topbar-state")).toHaveText("实时连接正常", {
    timeout: 60000,
  });
  const panel = page.locator(".ai-task-panel");
  await panel.getByRole("button", { name: "新会话", exact: true }).click();
  async function send(text, supplement = false) {
    await input.fill(text);
    const response = page.waitForResponse(
      (r) =>
        r.request().method() === "POST" &&
        new URL(r.url()).pathname === "/perception/api/ai/",
    );
    await panel
      .getByRole("button", {
        name: supplement ? "发送补充" : "发送 AI 任务",
        exact: true,
      })
      .click();
    const result = await response;
    expect(result.ok()).toBe(true);
    const run = await result.json();
    console.log("SENT", run.id, supplement ? "append" : "start");
    return run;
  }
  async function settled(run) {
    const article = panel.locator(`[data-run-id="${run.id}"]`);
    await expect(article).toHaveAttribute(
      "data-run-state",
      /^(succeeded|failed|cancelled|interrupted)$/,
      { timeout: 360000 },
    );
    const response = await page.request.get(
      base + "/perception/api/ai/?session=" + encodeURIComponent(run.sessionId),
    );
    expect(response.ok()).toBe(true);
    const record = (await response.json()).runs.find((r) => r.id === run.id);
    expect(record.state).toBe(await article.getAttribute("data-run-state"));
    expect(record.requests).toHaveLength(0);
    expect(
      record.calls.every((call) =>
        ["read_robot", "recall_experience"].includes(call.name),
      ),
    ).toBe(true);
    records.push(record);
    console.log(
      "SETTLED",
      record.id,
      record.state,
      "updates",
      record.updates?.length ?? 0,
    );
    return record;
  }
  if (process.argv[3] === "network") {
    const toggle = panel.getByRole("button", { name: /^模型与思考配置/ });
    if ((await toggle.getAttribute("aria-expanded")) === "false")
      await toggle.click();
    const address = panel.getByLabel("AI API 地址", { exact: true });
    const original = await address.inputValue();
    async function saveAddress(value) {
      await address.fill(value);
      const saved = page.waitForResponse(
        (r) =>
          r.request().method() === "POST" &&
          new URL(r.url()).pathname === "/perception/api/ai/",
      );
      await panel
        .getByRole("button", { name: "保存 AI 配置", exact: true })
        .click();
      expect((await saved).ok()).toBe(true);
    }
    try {
      // Deliberate unavailable local endpoint, not a replacement model/source.
      // No server listens here and no images or physical commands are sent.
      await saveAddress("http://127.0.0.1:1/v1");
      const failed = await send(
        "网络恢复网页验收，只回复文字；禁止运动、拍照或修改任何配置。",
      );
      const article = panel.locator(`[data-run-id="${failed.id}"]`);
      const seen = [];
      for (let attempt = 1; attempt <= 15; attempt++) {
        await expect(article).toContainText(`网络中断，重试 ${attempt}/15`, {
          timeout: 20000,
        });
        seen.push({ attempt, at: Date.now() });
        console.log("RETRY_VISIBLE", attempt);
        if (attempt === 1) {
          await send("补充只读提醒：不执行动作；网络恢复后只回复文字。", true);
          await page.reload();
          await expect(article).toContainText("网络中断，重试", {
            timeout: 60000,
          });
        }
      }
      const result = await settled(failed);
      expect(result.state).toBe("failed");
      expect(result.calls).toHaveLength(0);
      expect(result.endedAt - result.startedAt).toBeGreaterThanOrEqual(150000);
      expect(result.experienceSummary?.state).toBe("failed");
      expect(result.updates).toHaveLength(1);
      expect(result.updates[0].includedAt).toBeUndefined();
      await page.screenshot({
        path: join(output, "network-exhausted.png"),
        fullPage: true,
      });
      await writeFile(
        join(output, "retry-times.json"),
        JSON.stringify(seen, null, 2),
      );
      const cancelRetry = await send(
        "只读网络等待取消测试：不要运动、拍照或调用工具，只回复文字。",
      );
      await expect(
        panel.locator(`[data-run-id="${cancelRetry.id}"]`),
      ).toContainText("网络中断，重试 1/15", { timeout: 20000 });
      await panel
        .getByRole("button", { name: "停止 AI 与当前动作", exact: true })
        .click();
      const cancelled = await settled(cancelRetry);
      expect(cancelled.state).toBe("cancelled");
      expect(cancelled.experienceSummary?.state).toBe("cancelled");
    } finally {
      // A failed assertion must not leave this read-only fixture occupying AI.
      const stop = panel.getByRole("button", {
        name: "停止 AI 与当前动作",
        exact: true,
      });
      if (await stop.isEnabled()) {
        await stop.click();
        await expect(
          panel.locator(
            '[data-run-state="running"], [data-run-state="stopping"]',
          ),
        ).toHaveCount(0, { timeout: 180000 });
      }
      if ((await toggle.getAttribute("aria-expanded")) === "false")
        await toggle.click();
      await saveAddress(original);
    }
    const resumed = await send(
      "前一轮网络测试已结束。现在只回复‘网络测试后输入已恢复’，禁止工具、拍照和运动。",
    );
    const result = await settled(resumed);
    expect(result.state).toBe("succeeded");
    expect(result.text).toContain("网络测试后输入已恢复");
    expect(errors).toEqual([]);
    console.log(
      "PASS",
      "15 visible retries, pending supplement, reload, terminal failure, restored endpoint and input",
    );
    process.exitCode = 0;
  } else {
    const marker = "补充消息验收" + Date.now();
    const first = await send(
      "只读网页验收，不执行任何动作、不拍照、不改配置。请调用 read_robot 读取当前姿态，说明反馈来源；随后再读取一次，对比状态。执行期间我会发送补充。",
    );
    const added = await send(
      `补充：保持只读、不拍照、不运动。完成后必须在回复中包含‘${marker}’，并说明已收到本条补充。`,
      true,
    );
    expect(added.id).toBe(first.id);
    const done = await settled(first);
    expect(done.state).toBe("succeeded");
    expect(done.updates).toHaveLength(1);
    expect(done.updates[0].includedAt).toBeGreaterThan(0);
    expect(done.text).toContain(marker);
    await expect(
      panel.getByText("已加入模型上下文", { exact: true }),
    ).toBeVisible();
    await page.screenshot({
      path: join(output, "supplement.png"),
      fullPage: true,
    });
    await page.reload();
    await expect(panel.locator(`[data-run-id="${first.id}"]`)).toHaveAttribute(
      "data-run-state",
      "succeeded",
      { timeout: 60000 },
    );
    await expect(
      panel.getByText("已加入模型上下文", { exact: true }),
    ).toBeVisible();
    const second = await send(
      "继续只读验收。用一句话回复上一条补充中的验收标记，不调用工具、不拍照、不运动。",
    );
    expect(second.sessionId).toBe(first.sessionId);
    expect((await settled(second)).text).toContain(marker);
    const stopping = await send(
      "只读测试，不拍照、不运动。请依次调用 read_robot 五次，每次说明当前状态；不要调用其他工具。",
    );
    await panel
      .getByRole("button", { name: "停止 AI 与当前动作", exact: true })
      .click();
    expect((await settled(stopping)).state).toBe("cancelled");
    const afterStop = await send(
      "停止后继续发送验收。只回复‘输入已恢复’，不调用工具、不拍照、不运动。",
    );
    const restored = await settled(afterStop);
    expect(restored.state).toBe("succeeded");
    expect(restored.text).toContain("输入已恢复");
    expect(errors).toEqual([]);
    await page.screenshot({
      path: join(output, "continued.png"),
      fullPage: true,
    });
    console.log(
      "PASS",
      "same-run supplement, refresh, follow-up, cancellation, post-cancel input",
    );
  }
} finally {
  await writeFile(join(output, "runs.json"), JSON.stringify(records, null, 2));
  await browser.close();
}
