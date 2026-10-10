import { expect, test } from "@playwright/test";
import { randomUUID } from "node:crypto";

// Supplemental API checks; the separate test below remains the user-path acceptance.
test("preview remains read-only for apply actions and invalid targets", async ({
  request,
}) => {
  test.skip(process.env.AI_POSTURE_LIVE !== "1", "Requires live motion FK");
  test.setTimeout(120_000);
  await expect
    .poll(
      async () => {
        const state = (await (await request.get("/api/motion/state")).json())
          .values;
        return Boolean(
          state.robot_model_info &&
          state.arm_state &&
          state.motion_state?.current_tool_pose,
        );
      },
      { timeout: 60_000 },
    )
    .toBe(true);
  const before = (await (await request.get("/api/motion/state")).json()).values;
  const envelope = {
    schema_version: 3,
    model_revision: before.robot_model_info.model_revision,
    action: "apply",
    joints: [],
    actuators: [],
    options: {},
  };
  const preview = await request.post("/api/motion/preview", {
    data: { ...envelope, request_id: randomUUID() },
  });
  expect(preview.ok()).toBe(true);
  expect(await preview.json()).toMatchObject({
    acknowledged_action: "snapshot",
    value: {
      motion_executed: false,
      ik_checked: false,
      collision_checked: false,
    },
  });
  for (const invalid of [
    { model_revision: "wrong-model" },
    { joints: [{ joint_key: "unknown", position_rad: 0 }] },
    {
      joints: [
        { joint_key: "joint1", position_rad: 0 },
        { joint_key: "joint1", position_rad: 1 },
      ],
    },
    {
      tcp_target: {
        relative: true,
        pose: {
          frame: before.robot_model_info.base_frame,
          position_m: [0, 0, 0],
          orientation_xyzw: [0, 0, 0, 0],
        },
      },
    },
  ]) {
    const response = await request.post("/api/motion/preview", {
      data: { ...envelope, ...invalid, request_id: randomUUID() },
    });
    // The shared gateway returns the node result envelope with HTTP 200;
    // original_error, not HTTP delivery, is the operation outcome.
    const result = await response.json();
    expect(result.original_error).toBeTruthy();
    expect(result.value).toBeNull();
    await expect
      .poll(async () => {
        const record = await request.get(`/api/requests/${result.request_id}`);
        return (await record.json()).state;
      })
      .toBe("failed");
  }
  const after = (await (await request.get("/api/motion/state")).json()).values;
  expect(after.motion_state.control_mode).toBe(
    before.motion_state.control_mode,
  );
  expect(after.motion_status).toEqual(before.motion_status);
  expect(after.arm_command?.sequence).toBe(before.arm_command?.sequence);
});

// Opt-in real model + real service test, without motion. No API interception.
// AI_POSTURE_LIVE=1 pnpm exec playwright test ai-posture-live.spec.ts
test("AI previews actual posture through the normal conversation UI without motion", async ({
  page,
}, testInfo) => {
  test.skip(
    process.env.AI_POSTURE_LIVE !== "1",
    "Requires live model service and motion FK",
  );
  test.setTimeout(600_000);
  await page.goto("/perception/");
  await page.getByRole("button", { name: "新会话", exact: true }).click();
  const instruction =
    "本次仅做姿态只读验收，不执行机械臂或夹爪动作、不切换模式或相机、不继续历史抓放任务。先调用 read_robot 获取实际反馈。然后调用 preview_motion 完成四项计算：1) joints=[]、tcp_target=null 获取当前 FK；2) 仅把模型第一个关节的绝对角改成当前实际值加 0.01 弧度，其他关节不填，TCP 为 null；3) 底座系相对 X 增加 0.01 米，旋转增量为单位四元数；4) 工具系相对 X 增加 0.01 米，旋转增量为单位四元数。只预览，绝对不要执行这些目标。最后报告实际反馈来源、关节角、当前 TCP、预览目标与实际的区别，并解释关节直接运动不依赖 TCP IK 但仍需规划。四项完成即可结束本次验收。";
  await page.getByLabel("AI 任务", { exact: true }).fill(instruction);
  const accepted = page.waitForResponse(
    (response) =>
      response.url().endsWith("/perception/api/ai/") &&
      response.request().method() === "POST" &&
      response.request().postDataJSON()?.action === "start",
  );
  await page.getByRole("button", { name: "发送 AI 任务", exact: true }).click();
  const response = await accepted;
  expect(response.ok()).toBe(true);
  const run = await response.json();
  const card = page.locator(`[data-run-id="${run.id}"]`);
  await expect(card).toBeVisible();
  let latest = run;
  const deadline = Date.now() + 540_000;
  let lastLog = 0;
  while (!latest.endedAt && Date.now() < deadline) {
    const state = await (
      await page.request.get(
        `/perception/api/ai/?session=${encodeURIComponent(run.sessionId)}`,
      )
    ).json();
    latest =
      state.runs?.find((item: { id: string }) => item.id === run.id) ?? latest;
    if (Date.now() - lastLog > 30_000) {
      console.log(
        JSON.stringify({
          run: run.id,
          state: latest.state,
          calls: latest.calls.map((call: { name: string; error?: string }) => ({
            name: call.name,
            error: call.error,
          })),
        }),
      );
      lastLog = Date.now();
    }
    if (!latest.endedAt) await page.waitForTimeout(1000);
  }
  await testInfo.attach("actual-run", {
    body: JSON.stringify(latest, null, 2),
    contentType: "application/json",
  });
  expect(latest.state, latest.error).toBe("succeeded");
  expect(
    latest.calls.every((call: { name: string }) =>
      ["read_robot", "preview_motion"].includes(call.name),
    ),
  ).toBe(true);
  const previews = latest.calls.filter(
    (call: { name: string }) => call.name === "preview_motion",
  );
  expect(previews).toHaveLength(4);
  for (const preview of previews) {
    expect(preview.error).toBeUndefined();
    expect(preview.result).toMatchObject({
      motion_executed: false,
      ik_checked: false,
      collision_checked: false,
    });
    expect(preview.result.current_tcp.position_mm).toHaveLength(3);
    expect(preview.result.feedback.feedback_source).toMatch(
      /hardware|software/,
    );
  }
  const joint = previews.find(
    (call: { input: { joints: unknown[] } }) => call.input.joints.length > 0,
  );
  expect(joint.result.target_joints).not.toBeNull();
  const tcp = previews.filter(
    (call: { input: { tcp_target: unknown } }) => call.input.tcp_target,
  );
  expect(tcp).toHaveLength(2);
  for (const preview of tcp) expect(preview.result.target_joints).toBeNull();
  await expect(card).toContainText("本轮回复已结束");
  await expect(card.locator(".ai-response")).toHaveText(latest.text);
  await card.getByRole("button", { name: /^工具与执行详情/ }).click();
  await expect(card).toContainText("preview_motion");
  await page.screenshot({
    path: testInfo.outputPath("posture-preview.png"),
    fullPage: true,
  });
  await page.reload();
  await expect(card.locator(".ai-response")).toHaveText(latest.text);
});
