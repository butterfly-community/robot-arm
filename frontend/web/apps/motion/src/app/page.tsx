"use client";

import type { ArmState, MotionState, RobotModelInfo } from "@robot/contracts";
import { post, requestId, useGateway } from "@robot/gateway-client";
import {
  Button,
  Card,
  Field,
  Input,
  JsonView,
  KeyValue,
  Metric,
  RangeControls,
  Shell,
  StatusBadge,
} from "@robot/ui";
import { useState } from "react";

function degrees(value: number | undefined) {
  return Number.isFinite(value)
    ? ((Number(value) * 180) / Math.PI).toFixed(1)
    : "—";
}

export default function Page() {
  const { snapshot, error, setError } = useGateway("motion");
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
  const armKey = JSON.stringify([
    model?.model_revision,
    arm?.joints_rad,
    arm?.actuators_rad,
  ]);
  const [previousArmKey, setPreviousArmKey] = useState("");
  const [positions, setPositions] = useState<Record<string, number>>({});
  const [actuators, setActuators] = useState<Record<string, number>>({});
  const [editing, setEditing] = useState<string>();
  const [options, setOptions] = useState<Record<string, number | undefined>>(
    {},
  );

  if (arm && !editing && previousArmKey !== armKey) {
    setPreviousArmKey(armKey);
    setPositions(
      Object.fromEntries(
        (model?.joints ?? []).map((joint, index) => [
          joint.key,
          arm.joints_rad[index] ?? 0,
        ]),
      ),
    );
    setActuators(
      Object.fromEntries(
        (model?.tool_actuators ?? []).map((actuator, index) => [
          actuator.key,
          arm.actuators_rad[index] ?? 0,
        ]),
      ),
    );
  }

  async function mode(value: "relative" | "manual") {
    setError(undefined);
    try {
      await post("/api/motion/mode", {
        schema_version: 2,
        request_id: requestId(),
        mode: value,
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  async function cancel() {
    setError(undefined);
    try {
      await post("/api/motion/cancel", {
        schema_version: 2,
        request_id: requestId(),
        action: "cancel",
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  async function move(target = positions) {
    if (!model) return;
    setError(undefined);
    try {
      await post("/api/motion/request", {
        schema_version: 2,
        request_id: requestId(),
        model_revision: model.model_revision,
        joints: Object.entries(target).map(([joint_key, position_rad]) => ({
          joint_key,
          position_rad,
        })),
        options: Object.fromEntries(
          Object.entries(options).filter((entry): entry is [string, number] =>
            Number.isFinite(entry[1]),
          ),
        ),
        action: "apply",
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  const tool = motion?.current_tool_pose;
  const target = motion?.target_tool_pose;
  const state = String(latestMotion.state ?? "idle");

  return (
    <Shell
      section="03 / MOVEIT MOTION"
      title="机械臂运动"
      description="关节反馈、手动目标、末端位置和 MoveIt 状态直接形成操作台；完整 DTO 只在底部排障区查看。"
    >
      <div className="dashboard-grid">
        <div className="span-12 metric-grid">
          <Metric
            label="Control mode"
            value={String(motion?.control_mode ?? "—").toUpperCase()}
            tone="cyan"
          />
          <Metric
            label="Feedback"
            value={String(arm?.feedback_source ?? "WAIT").toUpperCase()}
            tone={arm?.feedback_source === "hardware" ? "green" : undefined}
          />
          <Metric
            label="MoveIt state"
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
          className="span-8 aligned-row-card"
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
            onBegin={setEditing}
            onChange={(key, value) =>
              setPositions({ ...positions, [key]: value })
            }
            onCommit={(key, value) => {
              const next = { ...positions, [key]: value };
              setEditing(undefined);
              void move(next);
            }}
          />
          <RangeControls
            items={model?.tool_actuators ?? []}
            values={actuators}
            onBegin={setEditing}
            onChange={(key, value) =>
              setActuators({ ...actuators, [key]: value })
            }
            onCommit={(key, value) => {
              setEditing(undefined);
              setError(undefined);
              void post("/api/motion/actuator", {
                schema_version: 2,
                request_id: requestId(),
                model_revision: model!.model_revision,
                actuator_key: key,
                position_rad: value,
                action: "apply",
              }).catch((reason) => setError(String(reason)));
            }}
          />
        </Card>

        <Card
          className="span-4 aligned-row-card"
          eyebrow="Motion command"
          title="模式 / 规划"
        >
          <div className="card-actions card-actions-leading">
            <Button
              variant={
                motion?.control_mode === "relative" ? "default" : "outline"
              }
              onClick={() => mode("relative")}
            >
              相对控制
            </Button>
            <Button
              variant={
                motion?.control_mode === "manual" ? "default" : "outline"
              }
              onClick={() => mode("manual")}
            >
              手动控制
            </Button>
            <Button variant="danger" onClick={cancel}>
              取消普通运动
            </Button>
          </div>
          <KeyValue
            label="当前"
            value={String(motion?.control_mode ?? "未知")}
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
          <div className="target-list">
            {(model?.named_targets ?? []).map((item) => (
              <Button
                key={item.key}
                onClick={() => {
                  setPositions(item.positions_rad);
                  void move(item.positions_rad);
                }}
              >
                {item.label}
              </Button>
            ))}
          </div>
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
      {error && <p className="error">{error}</p>}
    </Shell>
  );
}
