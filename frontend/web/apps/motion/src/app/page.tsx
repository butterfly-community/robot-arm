"use client";

import {
  schemaVersion,
  type ArmState,
  type MotionState,
  type RobotModelInfo,
} from "@robot/contracts";
import {
  post,
  prepareRelativeControl,
  requestId,
  useGateway,
} from "@robot/gateway-client";
import {
  Button,
  Card,
  Field,
  HelpDot,
  Input,
  JsonView,
  KeyValue,
  Metric,
  RangeControls,
  RequestStatus,
  Shell,
  StatusBadge,
} from "@robot/ui";
import { useState } from "react";
import { RobotViewer } from "@robot/visualization/robot-viewer";
import { TcpTarget } from "./tcp-target";

type ManualTarget = {
  model_revision: string;
  positions: Record<string, number>;
  actuators: Record<string, number>;
};

function degrees(value: number | undefined) {
  return Number.isFinite(value)
    ? ((Number(value) * 180) / Math.PI).toFixed(1)
    : "—";
}

export default function Page() {
  const { snapshot, error, setError, connection } = useGateway("motion");
  const values = snapshot?.values ?? {};
  const model = values.robot_model_info as unknown as
    RobotModelInfo | undefined;
  const arm = values.arm_state as unknown as ArmState | undefined;
  const motion = values.motion_state as unknown as
    (MotionState & Record<string, unknown>) | undefined;
  const latestMotion = (motion?.latest_motion ??
    values.motion_status ??
    {}) as Record<string, unknown>;
  const diagnostics = (motion?.diagnostics ?? []) as Array<
    Record<string, unknown>
  >;
  const [draft, setDraft] = useState<ManualTarget>();
  const [options, setOptions] = useState<Record<string, number | undefined>>(
    {},
  );
  const [pendingAction, setPendingAction] = useState<string>();
  const [cancelling, setCancelling] = useState(false);
  const actuatorExecuting =
    (motion?.latest_actuator as { state?: string } | undefined)?.state ===
      "executing" &&
    (motion?.latest_actuator as { request_id?: string } | undefined)
      ?.request_id !== "input-action";

  // Feedback follows the arm until edited. A draft survives feedback and failures.
  // A different model must never inherit joint keys from the previous model.
  const manualTarget =
    draft?.model_revision === model?.model_revision && draft
      ? draft
      : {
          model_revision: model?.model_revision ?? "",
          positions: Object.fromEntries(
            (model?.joints ?? []).map((joint, index) => [
              joint.key,
              arm?.joints_rad[index] ?? 0,
            ]),
          ),
          actuators: Object.fromEntries(
            (model?.tool_actuators ?? []).map((actuator, index) => [
              actuator.key,
              arm?.actuators_rad[index] ?? 0,
            ]),
          ),
        };
  const { positions, actuators } = manualTarget;
  const preview = model
    ? {
        model_revision: model.model_revision,
        joints_rad: model.joints.map((joint) => positions[joint.key]),
        actuators_rad: model.tool_actuators.map(
          (actuator) => actuators[actuator.key],
        ),
      }
    : undefined;
  function editJoint(key: string, value: number) {
    setDraft({ ...manualTarget, positions: { ...positions, [key]: value } });
  }
  function editActuator(key: string, value: number) {
    setDraft({ ...manualTarget, actuators: { ...actuators, [key]: value } });
  }

  async function mode(value: "relative" | "manual") {
    setError(undefined);
    setPendingAction(`mode:${value}`);
    try {
      await post("/api/motion/mode", {
        schema_version: schemaVersion,
        request_id: requestId(),
        mode: value,
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setPendingAction(undefined);
    }
  }

  async function prepareRelative() {
    setError(undefined);
    setPendingAction("prepare-relative");
    try {
      await prepareRelativeControl();
    } catch (reason) {
      setError(String(reason));
    } finally {
      setPendingAction(undefined);
    }
  }

  async function cancel() {
    setError(undefined);
    setCancelling(true);
    try {
      await post("/api/motion/cancel", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action: "cancel",
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setCancelling(false);
    }
  }

  async function manualMode() {
    await post("/api/motion/mode", {
      schema_version: schemaVersion,
      request_id: requestId(),
      mode: "manual",
    });
  }

  async function move(
    jointTarget = positions,
    actuatorTarget = actuators,
    action = "manual-motion",
  ) {
    if (!model) return;
    setDraft({
      model_revision: model.model_revision,
      positions: jointTarget,
      actuators: actuatorTarget,
    });
    setError(undefined);
    setPendingAction(action);
    try {
      await manualMode();
      await post("/api/motion/request", {
        schema_version: schemaVersion,
        request_id: requestId(),
        model_revision: model.model_revision,
        joints: Object.entries(jointTarget).map(
          ([joint_key, position_rad]) => ({ joint_key, position_rad }),
        ),
        actuators: Object.entries(actuatorTarget).map(
          ([actuator_key, position_rad]) => ({ actuator_key, position_rad }),
        ),
        options: Object.fromEntries(
          Object.entries(options).filter((entry): entry is [string, number] =>
            Number.isFinite(entry[1]),
          ),
        ),
        action: "apply",
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setPendingAction(undefined);
    }
  }

  const tool = motion?.current_tool_pose;
  const target = motion?.target_tool_pose;
  const state = String(latestMotion.state ?? "idle");
  const motionBusy = state === "planning" || state === "executing";

  return (
    <Shell
      connection={connection}
      section="04 / MOVEIT MOTION"
      title="机械臂运动"
      description="相对输入由 Servo 连续处理；手动和感知任务顺序执行。三种模式独立，共用同一套机械臂模型和执行反馈。"
    >
      <div className="dashboard-grid">
        <div className="span-12 metric-grid">
          <Metric
            label="控制模式"
            value={String(motion?.control_mode ?? "—").toUpperCase()}
            tone="cyan"
          />
          <Metric
            label="反馈来源"
            value={String(arm?.feedback_source ?? "WAIT").toUpperCase()}
            tone={arm?.feedback_source === "hardware" ? "green" : undefined}
          />
          <Metric
            label="MoveIt 状态"
            value={state.toUpperCase()}
            tone={
              state === "failed"
                ? "red"
                : state === "succeeded"
                  ? "green"
                  : "amber"
            }
          />
          <Metric
            label="TCP X"
            value={tool?.position_m?.[0]?.toFixed(3) ?? "—"}
            unit="m"
            tone="red"
          />
          <Metric
            label="TCP Y"
            value={tool?.position_m?.[1]?.toFixed(3) ?? "—"}
            unit="m"
            tone="green"
          />
          <Metric
            label="TCP Z"
            value={tool?.position_m?.[2]?.toFixed(3) ?? "—"}
            unit="m"
            tone="blue"
          />
        </div>

        <Card
          className="span-6 aligned-row-card"
          eyebrow="Joint workspace"
          title="关节与夹爪状态 / 手动目标"
          action={
            <StatusBadge tone={arm ? "good" : "warning"}>
              {model?.display_name ?? "等待模型"}
            </StatusBadge>
          }
        >
          <div className="joint-strip" aria-label="关节角度图形">
            {(model?.joints ?? []).map((joint, index) => {
              const value = arm?.joints_rad[index] ?? 0;
              const fraction =
                (value - joint.minimum) / (joint.maximum - joint.minimum);
              return (
                <div className="joint-gauge" key={joint.key}>
                  <span>{joint.label}</span>
                  <div>
                    <i style={{ left: `${fraction * 100}%` }} />
                  </div>
                  <strong>{degrees(value)}°</strong>
                </div>
              );
            })}
          </div>
          <RangeControls
            items={model?.joints ?? []}
            values={positions}
            onChange={editJoint}
            onCommit={editJoint}
          />
          <RangeControls
            items={model?.tool_actuators ?? []}
            values={actuators}
            onChange={editActuator}
            onCommit={editActuator}
          />
          <div className="card-actions">
            <Button
              disabled={!model || Boolean(pendingAction)}
              onClick={() => move()}
            >
              {pendingAction === "manual-motion" ? "正在执行…" : "执行目标"}
            </Button>
            <Button
              variant="outline"
              disabled={!arm}
              onClick={() => {
                setDraft(undefined);
                setError(undefined);
              }}
            >
              恢复当前姿态
            </Button>
            <Button
              disabled={!model || Boolean(pendingAction) || actuatorExecuting}
              onClick={async () => {
                if (!model) return;
                setPendingAction("actuator");
                setError(undefined);
                try {
                  await manualMode();
                  for (const actuator of model.tool_actuators) {
                    await post("/api/motion/actuator", {
                      schema_version: schemaVersion,
                      request_id: requestId(),
                      model_revision: model.model_revision,
                      actuator_key: actuator.key,
                      position_rad: actuators[actuator.key],
                    });
                  }
                } catch (reason) {
                  setError(String(reason));
                } finally {
                  setPendingAction(undefined);
                }
              }}
            >
              {pendingAction === "actuator"
                ? "正在提交夹爪目标…"
                : actuatorExecuting
                  ? "夹爪正在执行…"
                  : "仅执行夹爪目标"}
            </Button>
          </div>
          {error && (
            <p className="error" role="alert">
              {error}
            </p>
          )}
        </Card>

        <Card
          className="span-6 aligned-row-card viewer-fill-card robot-fill-card"
          title="实际姿态 / 目标姿态"
        >
          {model ? (
            <RobotViewer
              model={model}
              arm={arm}
              command={preview}
              parameters={[]}
              showLabels={false}
            />
          ) : (
            <p className="status">等待运动节点发布模型</p>
          )}
          <div className="viewer-legend">
            <span>
              <i className="feedback-dot" />
              实体：实际反馈
            </span>
            <span>
              <i className="command-dot" />
              半透明：目标姿态
              <HelpDot text="编辑不会发送运动；目标不是规划成功保证，执行失败后仍保留，便于继续调整。恢复当前姿态只重置编辑值，不取消正在执行的运动。" />
            </span>
          </div>
        </Card>

        <Card
          className="span-12"
          eyebrow="Control ownership"
          title="控制模式 / 规划"
        >
          <div className="card-actions card-actions-leading">
            <Button
              variant={
                motion?.control_mode === "relative" ? "default" : "outline"
              }
              disabled={
                Boolean(pendingAction) || motion?.control_mode === "relative"
              }
              onClick={() => mode("relative")}
            >
              {pendingAction === "mode:relative"
                ? "正在切换…"
                : motion?.control_mode === "relative"
                  ? "当前为相对控制"
                  : "相对控制"}
            </Button>
            <Button
              variant={
                motion?.control_mode === "manual" ? "default" : "outline"
              }
              disabled={
                Boolean(pendingAction) || motion?.control_mode === "manual"
              }
              onClick={() => mode("manual")}
            >
              {pendingAction === "mode:manual"
                ? "正在切换…"
                : motion?.control_mode === "manual"
                  ? "当前为手动控制"
                  : "手动控制"}
            </Button>
            <Button
              asChild
              variant={
                motion?.control_mode === "perception" ? "default" : "outline"
              }
            >
              <a href="/perception/">感知控制</a>
            </Button>
          </div>
          <KeyValue
            label="当前"
            value={String(motion?.control_mode ?? "未知")}
          />
          <KeyValue
            label="模式语义"
            value={
              motion?.control_mode === "relative"
                ? "接收空间转换后的相对输入"
                : motion?.control_mode === "perception"
                  ? "由结构化场景创建抓放任务"
                  : "接收关节、夹爪与命名位置命令"
            }
          />
          <KeyValue
            label="请求"
            value={String(latestMotion.request_id ?? "—")}
          />
          <KeyValue
            label="结果"
            value={String(latestMotion.result_message ?? "—")}
          />
          <KeyValue
            label="轨迹点"
            value={String(latestMotion.trajectory_points ?? "—")}
          />
          <KeyValue
            label="结果码"
            value={String(latestMotion.result_code ?? "—")}
          />
          <KeyValue
            label="计划时长 / 秒"
            value={String(latestMotion.planned_duration_s ?? "—")}
          />
          {(model?.motion_options ?? []).map((option) => (
            <Field
              key={option.key}
              label={option.label}
              englishLabel={`${option.key} · ${option.unit}`}
            >
              <Input
                type="number"
                step="any"
                min={option.minimum ?? undefined}
                max={option.maximum ?? undefined}
                value={options[option.key] ?? ""}
                aria-label={option.label}
                placeholder="留空使用服务默认值"
                title="仅覆盖下一次普通运动请求；留空不覆盖服务默认值，不是当前实测速度。"
                onChange={(event) =>
                  setOptions({
                    ...options,
                    [option.key]:
                      event.currentTarget.value === ""
                        ? undefined
                        : event.currentTarget.valueAsNumber,
                  })
                }
              />
            </Field>
          ))}
          {Object.values(options).some((value) => value !== undefined) && (
            <div className="card-actions">
              <Button variant="outline" onClick={() => setOptions({})}>
                恢复服务默认参数
              </Button>
            </div>
          )}
          <div className="target-list">
            <Button
              disabled={motionBusy || Boolean(pendingAction)}
              onClick={prepareRelative}
            >
              {pendingAction === "prepare-relative"
                ? "正在准备…"
                : "准备相对控制"}
            </Button>
            {(model?.named_targets ?? []).map((item) => (
              <Button
                key={item.key}
                disabled={motionBusy || Boolean(pendingAction)}
                onClick={() => {
                  void move(
                    item.joint_positions_rad,
                    item.actuator_positions_rad,
                    `target:${item.key}`,
                  );
                }}
              >
                {pendingAction === `target:${item.key}`
                  ? `正在前往${item.label}…`
                  : item.label}
              </Button>
            ))}
            <Button
              variant="danger"
              disabled={cancelling}
              onClick={cancel}
              title="取消普通运动请求，不是断电急停，也不取消 MTC 抓放任务。具体停止进度以运动节点反馈为准。"
            >
              {cancelling ? "正在取消…" : "取消普通运动"}
            </Button>
          </div>
        </Card>

        {model && (
          <TcpTarget
            key={model.model_revision}
            model={model}
            arm={arm}
            motion={motion}
            busy={motionBusy || Boolean(pendingAction)}
          />
        )}
        <Card className="span-12" title="动作结果查询">
          <RequestStatus scope="motion" />
        </Card>
        <Card className="span-6" eyebrow="Tool pose" title="末端当前 / 目标">
          <div className="pose-comparison">
            <div>
              <StatusBadge tone="good">当前 TCP</StatusBadge>
              <KeyValue
                label="位置"
                value={
                  tool
                    ? tool.position_m
                        .map((value) => value.toFixed(4))
                        .join(" / ")
                    : "—"
                }
              />
              <KeyValue
                label="姿态 XYZW"
                value={
                  tool
                    ? tool.orientation_xyzw
                        .map((value) => value.toFixed(3))
                        .join(" / ")
                    : "—"
                }
              />
            </div>
            <div>
              <StatusBadge tone={target ? "cyan" : "neutral"}>
                目标 TCP
              </StatusBadge>
              <KeyValue
                label="位置"
                value={
                  target
                    ? target.position_m
                        .map((value) => value.toFixed(4))
                        .join(" / ")
                    : "—"
                }
              />
              <KeyValue
                label="姿态 XYZW"
                value={
                  target
                    ? target.orientation_xyzw
                        .map((value) => value.toFixed(3))
                        .join(" / ")
                    : "—"
                }
              />
            </div>
          </div>
        </Card>

        <Card
          className="span-6"
          eyebrow="Runtime diagnostics"
          title="MoveIt / Servo"
        >
          <div className="parameter-list">
            {diagnostics.map((item) => (
              <div className="parameter-item" key={String(item.key)}>
                <KeyValue
                  label={String(item.key)}
                  value={String(item.value ?? "—")}
                />
              </div>
            ))}
          </div>
          {!diagnostics.length && (
            <p className="status">等待 Motion 节点诊断</p>
          )}
        </Card>

        <Card
          className="span-12"
          eyebrow="Troubleshooting"
          title="排障数据"
          defaultOpen={false}
        >
          <div className="diagnostic-grid">
            <JsonView title="MotionState 原始数据" value={motion} />
            <JsonView title="请求结果" value={values.motion_request_result} />
            <JsonView title="型号元数据" value={model} />
          </div>
        </Card>
      </div>
    </Shell>
  );
}
