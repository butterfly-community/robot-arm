import { expect, it } from "vitest";
import type { ManipulationTaskState, PerceptionState } from "@robot/contracts";
import { pickPlaceStatus, type PickPlaceAttempt } from "./pick-place-progress";

it("shows candidate preparation after reload, without calling it execution", () => {
  const state = {
    task_action: "generate_grasps",
    task_state: "executing",
    task_request_id: "gen",
  } as PerceptionState;
  expect(pickPlaceStatus(undefined, state)).toMatchObject({
    stage: "生成并筛选抓取候选",
    active: true,
  });
});
it("ignores the previous task while the new request is being submitted", () => {
  const attempt = {
    phase: "submit",
    requestId: "new",
    startedAt: 0,
  } as PickPlaceAttempt;
  const old = {
    request_id: "old",
    state: "succeeded",
  } as ManipulationTaskState;
  expect(pickPlaceStatus(attempt, undefined, old)).toMatchObject({
    status: "等待反馈",
    active: true,
  });
});
it("uses matching live feedback after HTTP acknowledgement", () => {
  const attempt = {
    phase: "accepted",
    requestId: "new",
    startedAt: 0,
  } as PickPlaceAttempt;
  expect(
    pickPlaceStatus(attempt, undefined, {
      request_id: "previous",
      state: "succeeded",
    } as ManipulationTaskState),
  ).toMatchObject({
    status: "等待反馈",
    active: true,
    requestId: "new",
  });
  const task = {
    request_id: "new",
    state: "planning",
    stage: "search complete task solutions",
    solution_count: 3,
  } as ManipulationTaskState;
  expect(pickPlaceStatus(attempt, undefined, task)).toMatchObject({
    status: "正在规划",
    stage: "搜索并排序完整抓放方案",
    solutions: 3,
    active: true,
  });
  expect(
    pickPlaceStatus(attempt, undefined, {
      ...task,
      state: "failed",
      original_error: "no solution",
    }),
  ).toMatchObject({ active: false, error: "no solution" });
});
