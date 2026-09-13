"use client";

import { useEffect, useState } from "react";
import type { ManipulationTaskState, PerceptionState } from "@robot/contracts";
import { KeyValue } from "@robot/ui";
import type { PickPlaceStep } from "./start-pick-place";

export type PickPlaceAttempt = {
  phase: PickPlaceStep["phase"] | "accepted" | "failed";
  requestId: string;
  startedAt: number;
  error?: string;
  failedPhase?: PickPlaceStep["phase"];
};

const stages: Record<string, string> = {
  "build planning scene": "构建碰撞场景与初始化任务",
  "search complete task solutions": "搜索并排序完整抓放方案",
};
const states = {
  idle: "等待任务",
  planning: "正在规划",
  executing: "正在执行",
  succeeded: "执行完成",
  failed: "执行失败",
  cancelled: "已取消",
};

export function pickPlaceStatus(
  attempt?: PickPlaceAttempt,
  perception?: PerceptionState,
  task?: ManipulationTaskState,
) {
  if (attempt?.phase === "failed")
    return {
      status: "启动失败",
      stage:
        attempt.failedPhase === "generate_grasps"
          ? "生成抓取候选失败"
          : attempt.failedPhase === "mode"
            ? "切换感知控制模式失败"
            : "提交抓放请求失败",
      active: false,
      error:
        task?.request_id === attempt.requestId
          ? (task.original_error ?? attempt.error)
          : attempt.error,
      requestId: attempt.requestId,
    };
  if (
    attempt?.phase === "generate_grasps" ||
    (!attempt &&
      perception?.task_action === "generate_grasps" &&
      perception.task_state === "executing")
  )
    return {
      status: "正在准备",
      stage: "生成并筛选抓取候选",
      active: true,
      requestId: attempt?.requestId ?? perception?.task_request_id ?? "",
    };
  if (attempt?.phase === "mode")
    return {
      status: "正在准备",
      stage: "切换到感知控制模式",
      active: true,
      requestId: attempt.requestId,
    };
  // A prior task's success/failure must never overwrite this attempt's progress.
  if (task?.request_id && (!attempt || task.request_id === attempt.requestId))
    return {
      status: states[task.state],
      stage:
        stages[task.stage ?? ""] ??
        task.stage ??
        (task.state === "failed"
          ? "抓放请求失败"
          : task.state === "cancelled"
            ? "抓放已取消"
            : task.state === "succeeded"
              ? "抓放流程完成"
              : "等待运动服务反馈"),
      active: task.state === "planning" || task.state === "executing",
      error: task.original_error,
      requestId: task.request_id,
      solutions: task.solution_count,
      cost: task.selected_cost,
    };
  if (attempt)
    return {
      status: "等待反馈",
      stage: "已提交抓放请求，等待运动服务状态",
      active: true,
      requestId: attempt.requestId,
    };
  return {
    status: "等待任务",
    stage: "选择目标后启动",
    active: false,
    requestId: "",
  };
}

function Elapsed({
  active,
  startedAt,
}: {
  active: boolean;
  startedAt?: number;
}) {
  const [seconds, setSeconds] = useState(0);
  useEffect(() => {
    if (!active) return;
    const start = startedAt ?? Date.now();
    const timer = window.setInterval(
      () => setSeconds(Math.max(0, Math.floor((Date.now() - start) / 1000))),
      1000,
    );
    return () => window.clearInterval(timer);
  }, [active, startedAt]);
  return (
    <KeyValue
      label="本页已等待"
      value={`${seconds} 秒`}
      hint="从本页发起或接收到任务状态起计时，不是剩余时间。"
    />
  );
}

export function PickPlaceProgress({
  attempt,
  perception,
  task,
  objectId,
}: {
  attempt?: PickPlaceAttempt;
  perception?: PerceptionState;
  task?: ManipulationTaskState;
  objectId: string;
}) {
  const view = pickPlaceStatus(attempt, perception, task);
  const count = perception?.instances.find(
    (i) => i.instance_id === objectId,
  )?.grasp_candidate_count;
  return (
    <div aria-label="抓放进度" style={{ marginTop: "var(--space-section)" }}>
      <div role="status" aria-live="polite">
        <KeyValue label="任务状态" value={view.status} />
        <KeyValue label="当前阶段" value={view.stage} />
      </div>
      {view.requestId && (
        <>
          <Elapsed
            key={attempt?.startedAt ?? view.requestId}
            active={view.active}
            startedAt={attempt?.startedAt}
          />
          <KeyValue
            label="抓取候选 / 完整方案"
            value={`${count ?? "—"} / ${view.solutions ?? "—"}`}
          />
          {view.cost != null && (
            <KeyValue label="所选方案代价" value={String(view.cost)} />
          )}
          <KeyValue label="请求编号" value={view.requestId} />
        </>
      )}
      {view.error && (
        <p className="error" role="alert">
          {view.error}
        </p>
      )}
    </div>
  );
}
