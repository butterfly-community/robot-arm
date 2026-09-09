import { expect, test } from "@playwright/test";

// All POSTs are intercepted: this tests the real UI, not robot motion or inference.
test("model capabilities switch visible settings without discarding prompted configuration", async ({
  page,
  request,
}) => {
  const snapshot = await (await request.get("/api/perception/state")).json();
  const state = snapshot.values.perception_state;
  state.model = "prompted-fixture";
  state.available_models = [
    { id: "prompted-fixture", label: "Prompted", prompt_free: false },
    { id: "automatic-fixture", label: "Automatic", prompt_free: true },
  ];
  state.classes = ["pager", "square paper"];
  state.placement_labels = ["square paper"];
  state.task_state = "idle";
  state.visual_prompt_active = true;
  await page.routeWebSocket("**/ws/perception", (socket) =>
    socket.send(JSON.stringify(snapshot)),
  );
  const requests: Record<string, unknown>[] = [];
  await page.route("**/api/**", (route) => {
    if (route.request().method() !== "POST") return route.fallback();
    requests.push(route.request().postDataJSON());
    return route.fulfill({ json: { original_error: null } });
  });
  await page.goto("/perception/");
  await page.getByText("抓放详细配置", { exact: true }).click();
  const model = page.getByLabel("识别与分割模型", { exact: true });
  const prompts = page.getByLabel("识别与分割提示词", { exact: true });
  const roles = page.getByLabel("放置区域角色", { exact: true });
  await expect(prompts).toHaveValue("pager, square paper");
  await expect(
    page.getByText("已启用视觉示例提示", { exact: true }),
  ).toBeVisible();
  await model.selectOption("automatic-fixture");
  await expect(prompts).toHaveCount(0);
  await expect(roles).toHaveCount(0);
  await expect(
    page.getByText("已启用视觉示例提示", { exact: true }),
  ).toHaveCount(0);
  expect(requests).toHaveLength(0);
  await page.getByRole("button", { name: "保存模型配置", exact: true }).click();
  await expect.poll(() => requests.length).toBe(1);
  expect(requests[0]).toMatchObject({
    model: "automatic-fixture",
    classes: null,
    placement_labels: null,
  });
  await model.selectOption("prompted-fixture");
  await expect(prompts).toHaveValue("pager, square paper");
  await expect(roles).toHaveValue("square paper");
  expect(requests).toHaveLength(1);
  const overflow = await page.evaluate(
    () => document.documentElement.scrollWidth > window.innerWidth,
  );
  expect(overflow).toBe(false);
});
