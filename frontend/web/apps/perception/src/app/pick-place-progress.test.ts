import { expect, it } from "vitest";
import type { ManipulationTaskState, PerceptionState } from "@robot/contracts";
import { pickPlaceStatus, type PickPlaceAttempt } from "./pick-place-progress";

it("does not show waiting after reloading a terminal request without a stage", () => {
  for (const [state, stage] of [
    ["failed", "抓放请求失败"],
    ["cancelled", "抓放已取消"],
    ["succeeded", "抓放流程完成"],
  ]) {
    expect(
      pickPlaceStatus(undefined, undefined, {
        request_id: "restored",
        state,
        stage: null,
      } as ManipulationTaskState),
    ).toMatchObject({ stage, active: false });
  }
});

it("retains the failed preparation stage and uses only matching backend errors", () => {
  const attempt = {
    phase: "failed",
    failedPhase: "generate_grasps",
    requestId: "new",
    startedAt: 0,
    error: "current failure",
  } as PickPlaceAttempt;
  expect(
    pickPlaceStatus(attempt, undefined, {
      request_id: "old",
      original_error: "old failure",
    } as ManipulationTaskState),
  ).toMatchObject({
    stage: "生成抓取候选失败",
    error: "current failure",
    active: false,
  });
  expect(
    pickPlaceStatus({ ...attempt, failedPhase: "submit" }, undefined, {
      request_id: "new",
      original_error: "scene mismatch",
    } as ManipulationTaskState),
  ).toMatchObject({ stage: "提交抓放请求失败", error: "scene mismatch" });
});

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
