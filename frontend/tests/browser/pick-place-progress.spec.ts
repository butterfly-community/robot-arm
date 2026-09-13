import { expect, test } from "@playwright/test";

// Click the actual Start button; all writes are intercepted, no real arm motion.
for (const width of [1440, 390])
  test(`grab progress follows preparation, planning, execution and failure at ${width}`, async ({
    page,
    request,
  }) => {
    const snapshot = await (await request.get("/api/perception/state")).json();
    const p = snapshot.values.perception_state;
    const pose = { position_m: [0.2, 0, 0.03], orientation_xyzw: [0, 0, 0, 1] };
    Object.assign(p, {
      task_state: "idle",
      task_action: null,
      instances: [
        { instance_id: "box", label: "box", grasp_candidate_count: 0 },
      ],
      original_error: null,
    });
    snapshot.values.world_scene = {
      sequence: 1,
      frame_id: "base_link",
      objects: [
        {
          object_id: "box",
          label: "box",
          pose,
          size_m: [0.03, 0.03, 0.03],
          grasp_candidates: [],
        },
      ],
      placement_regions: [
        { region_id: "pad", label: "pad", pose, size_m: [0.1, 0.1, 0.01] },
      ],
      obstacles: [],
    };
    snapshot.values.manipulation_state = {
      request_id: "previous",
      state: "failed",
      original_error: "previous error",
    };
    let publish = () => {};
    await page.routeWebSocket("**/ws/perception", (socket) => {
      publish = () => socket.send(JSON.stringify(snapshot));
      publish();
    });
    await page.routeWebSocket("**/ws/camera-video", () => {});
    let releaseGeneration = () => {},
      releaseMode = () => {};
    const generation = new Promise<void>((resolve) => {
      releaseGeneration = resolve;
    });
    const mode = new Promise<void>((resolve) => {
      releaseMode = resolve;
    });
    let accepted = false;
    let taskId = "",
      fail = false;
    await page.route("**/api/**", async (route) => {
      if (route.request().method() !== "POST") return route.fallback();
      const body = route.request().postDataJSON(),
        path = new URL(route.request().url()).pathname;
      if (path === "/api/perception/request") {
        expect(body.action).toBe("generate_grasps");
        await generation;
        if (fail)
          return route.fulfill({
            status: 400,
            json: { original_error: "候选生成失败（测试）" },
          });
        p.last_scene_sequence = 2;
        p.instances[0].grasp_candidate_count = 12;
        return route.fulfill({ json: { value: p, original_error: null } });
      }
      if (path === "/api/motion/mode") {
        await mode;
        return route.fulfill({ json: { original_error: null } });
      }
      expect(path).toBe("/api/perception/pick-place");
      expect(body.scene_sequence).toBe(2);
      taskId = body.request_id;
      snapshot.values.manipulation_state = {
        request_id: taskId,
        state: "planning",
        stage: "build planning scene",
        solution_count: null,
      };
      await route.fulfill({
        status: 202,
        json: { accepted: true, request_id: taskId },
      });
      accepted = true;
    });
    await page.setViewportSize({ width, height: 1000 });
    await page.goto("/perception/");
    await page.getByText("抓放场景", { exact: true }).click();
    await page.getByLabel("抓取目标", { exact: true }).selectOption("box");
    await page.getByLabel("放置区域", { exact: true }).selectOption("pad");
    await page.getByRole("button", { name: "启动", exact: true }).click();
    const progress = page.getByLabel("抓放进度", { exact: true });
    await expect(progress).toContainText("生成并筛选抓取候选");
    await expect(progress).not.toContainText("previous error");
    await expect(progress).toContainText("1 秒", { timeout: 5000 });
    releaseGeneration();
    await expect(progress).toContainText("切换到感知控制模式");
    releaseMode();
    await expect.poll(() => accepted).toBe(true);
    await expect(progress).toContainText("等待运动服务状态");
    await expect(
      page.getByRole("button", { name: "启动", exact: true }),
    ).toBeDisabled();
    const elapsed = async () =>
      Number((await progress.innerText()).match(/(\d+) 秒/)?.[1]);
    const beforeFeedback = await elapsed();
    await expect
      .poll(elapsed, { timeout: 5000 })
      .toBeGreaterThan(beforeFeedback);
    publish();
    await expect(progress).toContainText("构建碰撞场景与初始化任务");
    snapshot.values.manipulation_state.stage = "search complete task solutions";
    snapshot.values.manipulation_state.solution_count = 3;
    publish();
    await expect(progress).toContainText("搜索并排序完整抓放方案");
    await expect(progress).toContainText("12 / 3");
    snapshot.values.manipulation_state.state = "executing";
    snapshot.values.manipulation_state.stage = "执行：所选完整方案";
    publish();
    await expect(progress).toContainText("正在执行");
    await page.reload();
    await expect(progress).toContainText("正在执行");
    snapshot.values.manipulation_state.state = "succeeded";
    publish();
    await expect(progress).toContainText("执行完成");
    fail = true;
    await page.getByRole("button", { name: "启动", exact: true }).click();
    await expect(progress).toContainText("启动失败");
    await expect(progress).toContainText("候选生成失败（测试）");
    snapshot.values.manipulation_state = {
      request_id: "rejected-before-planning",
      state: "failed",
      stage: null,
      original_error: "机械臂执行连接已断开（测试）",
    };
    await page.reload();
    await expect(progress).toContainText("抓放请求失败");
    await expect(progress).toContainText("机械臂执行连接已断开（测试）");
    await expect(progress).not.toContainText("等待运动服务反馈");
    expect(
      await page.evaluate(
        () => document.documentElement.scrollWidth > innerWidth,
      ),
    ).toBe(false);
  });
