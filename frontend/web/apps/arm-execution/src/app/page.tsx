"use client";

import type {
  ArmCommand,
  ArmState,
  ExecutionInfo,
  ManipulationTaskState,
  MotionState,
  ParameterValue,
  RobotModelInfo,
} from "@robot/contracts";
import { schemaVersion } from "@robot/contracts";
import { post, requestId, useGateway } from "@robot/gateway-client";
import {
  Button,
  Card,
  Field,
  Input,
  JsonView,
  KeyValue,
  LocalizedLabel,
  Metric,
  Shell,
  StatusBadge,
} from "@robot/ui";
import { useState } from "react";
import { RobotViewer } from "@robot/visualization/robot-viewer";

type ExecutionAction =
  | "connect"
  | "disconnect"
  | "discover"
  | "refresh"
  | "save-config"
  | "torque-release"
  | "torque-hold";

function angle(value: number | undefined) {
  return Number.isFinite(value)
    ? ((Number(value) * 180) / Math.PI).toFixed(2)
    : "—";
}

export default function Page() {
  const { snapshot, error, setError, connection } = useGateway("arm-execution");
  const values = snapshot?.values ?? {};
  const model = values.robot_model_info as unknown as
    RobotModelInfo | undefined;
  const arm = values.arm_state as unknown as ArmState | undefined;
  const motion = values.motion_state as unknown as MotionState | undefined;
  const manipulation = values.manipulation_state as unknown as
    ManipulationTaskState | undefined;
  const execution = values.execution_info as unknown as
    ExecutionInfo | undefined;
  const transport = (values.transport_state ?? {}) as Record<string, unknown>;
  const parameters = (transport.parameter_values ?? []) as ParameterValue[];
  const command = transport.last_command as ArmCommand | undefined;
  const [connectionFields, setConnectionFields] = useState<
    Record<string, string>
  >({});
  const [showLabels, setShowLabels] = useState(false);
  const [feedbackIntervalOverride, setFeedbackIntervalOverride] = useState<
    string | undefined
  >();
  const [pendingAction, setPendingAction] = useState<ExecutionAction>();
  const [strengthOverride, setStrengthOverride] = useState<string>();
  const strengthTarget =
    strengthOverride ?? String(transport.gripper_strength_percent ?? "");
  const configChanged =
    feedbackIntervalOverride !== undefined || strengthOverride !== undefined;
  const feedbackIntervalMs =
    feedbackIntervalOverride ?? String(transport.feedback_interval_ms ?? "");

  async function requestExecution(
    action: "connect" | "disconnect" | "discover" | "refresh",
  ) {
    setError(undefined);
    setPendingAction(action);
    try {
      const endpoint = {
        connect: "connect",
        disconnect: "disconnect",
        discover: "endpoints",
        refresh: "parameters",
      }[action];
      await post(`/api/arm-execution/${endpoint}`, {
        schema_version: schemaVersion,
        request_id: requestId(),
        action,
        fields: connectionFields,
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setPendingAction(undefined);
    }
  }

  async function saveExecutionConfig() {
    setError(undefined);
    setPendingAction("save-config");
    try {
      await post("/api/arm-execution/config", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action: "apply",
        fields: {
          feedback_interval_ms: feedbackIntervalMs,
          gripper_strength_percent: strengthTarget,
        },
      });
      setFeedbackIntervalOverride(undefined);
      setStrengthOverride(undefined);
    } catch (reason) {
      setError(String(reason));
    } finally {
      setPendingAction(undefined);
    }
  }

  async function requestTorque(mode: "release" | "hold") {
    setError(undefined);
    setPendingAction(mode === "release" ? "torque-release" : "torque-hold");
    try {
      await post("/api/arm-execution/parameters", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action: "apply",
        fields: { torque_mode: mode },
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setPendingAction(undefined);
    }
  }

  const connected = Boolean(transport.connected);
  const hardwareSelected = transport.selected_endpoint != null;
  const parameterColumns = Array.from(
    new Map(
      parameters.map((item) => [
        item.field_key,
        { key: item.field_key, unit: item.unit },
      ]),
    ).values(),
  );
  const parameterActuators = Array.from(
    new Set(parameters.map((item) => item.actuator_key)),
  );
  const feedbackRows = [
    ...(model?.joints ?? []).map((joint, index) => ({
      key: "joint:" + joint.key,
      label: execution?.actuator_labels[index] ?? joint.label,
      feedback: arm?.joints_rad[index],
      requested: command?.joints_rad[index],
    })),
    ...(model?.tool_actuators ?? []).map((actuator, index) => ({
      key: "actuator:" + actuator.key,
      label:
        execution?.actuator_labels[(model?.joints.length ?? 0) + index] ??
        actuator.label,
      feedback: arm?.actuators_rad[index],
      requested: command?.actuators_rad[index],
    })),
  ];

  return (
    <Shell
      connection={connection}
      section="05 / ARM EXECUTION"
      title="机械臂执行"
      description="三维反馈、最后命令、串口状态和舵机参数形成同一高密度执行台；原始连接状态与消息只在排障区展开。"
    >
      {error && (
        <p className="error" role="alert">
          {error}
        </p>
      )}
      <div className="dashboard-grid">
        <div className="span-12 metric-grid">
          <Metric
            label="执行连接"
            value={
              !arm
                ? "等待状态"
                : connected
                  ? "真机已连接"
                  : hardwareSelected
                    ? "真机未连接"
                    : "软件模拟"
            }
            tone={connected ? "green" : "cyan"}
          />
          <Metric
            label="反馈来源"
            value={
              arm?.feedback_source === "hardware"
                ? "真机反馈"
                : arm?.feedback_source === "software"
                  ? "软件模拟"
                  : (arm?.feedback_source ?? "等待反馈")
            }
            tone={arm?.feedback_source === "hardware" ? "green" : undefined}
          />
          <Metric
            label="连接端点"
            value={String(transport.selected_endpoint ?? "—")}
          />
          <Metric
            label="Command seq"
            value={String(command?.sequence ?? "—")}
            tone="amber"
          />
          <Metric
            label="Joint count"
            value={String(model?.joints.length ?? "—")}
          />
          <Metric
            label="Parameter values"
            value={String(
              parameters.filter((item) => item.value != null).length,
            )}
          />
        </div>

        <Card
          className="span-8 aligned-row-card viewer-fill-card robot-fill-card"
          eyebrow="Three.js model feedback"
          title="机械臂反馈 / 命令 / 工具目标"
          action={
            <label className="compact-check">
              <input
                type="checkbox"
                checked={showLabels}
                onChange={(event) => setShowLabels(event.currentTarget.checked)}
              />
              显示关节编号和本次真机参数
            </label>
          }
        >
          {model ? (
            <RobotViewer
              model={model}
              arm={arm}
              command={command}
              motion={motion}
              manipulation={manipulation}
              execution={execution}
              parameters={parameters}
              showLabels={showLabels}
            />
          ) : (
            <p className="status">等待运动节点发布模型</p>
          )}
          <div className="viewer-legend">
            <span>
              <i className="feedback-dot" />
              <LocalizedLabel text="实体模型" english="ArmState feedback" />
            </span>
            <span>
              <i className="command-dot" />
              半透明模型：最后命令
            </span>
            <span>
              <i className="target-dot" />
              橙色坐标：工具目标
            </span>
            <span>
              <i className="pick-dot" />
              红点：抓取目标
            </span>
            <span>
              <i className="place-dot" />
              绿点：放置目标
            </span>
          </div>
        </Card>

        <div className="span-4 card-stack execution-card-stack">
          <Card
            eyebrow="Serial transport"
            title="执行连接"
            action={
              <StatusBadge tone={connected ? "good" : "cyan"}>
                {connected
                  ? "真机"
                  : hardwareSelected
                    ? "真机已停止"
                    : "软件反馈"}
              </StatusBadge>
            }
          >
            {connected ? (
              <>
                <KeyValue
                  label="已连接"
                  value={String(transport.selected_endpoint ?? "")}
                />
                <KeyValue
                  label="反馈"
                  value={String(transport.feedback_summary ?? "—")}
                />
              </>
            ) : (
              <>
                {execution?.connection_fields.map((field) => (
                  <Field key={field.key} label={field.label}>
                    {field.field_type === "endpoint" ? (
                      <>
                        <Input
                          required={field.required}
                          list={`connection-${field.key}`}
                          value={connectionFields[field.key] ?? ""}
                          onChange={(event) =>
                            setConnectionFields({
                              ...connectionFields,
                              [field.key]: event.currentTarget.value,
                            })
                          }
                        />
                        <datalist id={`connection-${field.key}`}>
                          {(
                            (transport.discovered_endpoints ?? []) as Array<
                              Record<string, unknown>
                            >
                          ).map((endpoint) => (
                            <option
                              key={String(endpoint.key)}
                              value={String(endpoint.key)}
                            >
                              {String(endpoint.label ?? endpoint.key)}
                            </option>
                          ))}
                          {field.options.map((option) => (
                            <option key={option} value={option}>
                              {option}
                            </option>
                          ))}
                        </datalist>
                      </>
                    ) : (
                      <Input
                        required={field.required}
                        value={connectionFields[field.key] ?? ""}
                        onChange={(event) =>
                          setConnectionFields({
                            ...connectionFields,
                            [field.key]: event.currentTarget.value,
                          })
                        }
                      />
                    )}
                  </Field>
                ))}
              </>
            )}
            <Field label="真机反馈周期（ms）">
              <Input
                type="number"
                value={feedbackIntervalMs}
                onChange={(event) =>
                  setFeedbackIntervalOverride(event.currentTarget.value)
                }
              />
            </Field>
            <Field label="夹持反馈目标（0–100）">
              <Input
                type="number"
                min={0}
                max={100}
                value={strengthTarget}
                title="使用已有负载反馈刻度，不是牛顿。夹住后持续调节，运输中仍保持；显式张开或卸力退出。"
                onChange={(event) =>
                  setStrengthOverride(event.currentTarget.value)
                }
              />
              <span className="muted">
                实测：
                {transport.connected &&
                typeof transport.gripper_strength_feedback_percent === "number"
                  ? transport.gripper_strength_feedback_percent.toFixed(1)
                  : "—"}
                {transport.gripper_control_power_mw != null
                  ? " · 持续力度调节中"
                  : " · 未进入夹持调节"}
              </span>
            </Field>
            <Field label="全部电机力矩">
              <div className="card-actions">
                <Button
                  variant="outline"
                  disabled={!connected || Boolean(pendingAction)}
                  title="停止所有电机并释放锁力"
                  onClick={() => requestTorque("release")}
                >
                  {pendingAction === "torque-release"
                    ? "正在卸力…"
                    : "全部卸力"}
                </Button>
                <Button
                  variant="outline"
                  disabled={!connected || Boolean(pendingAction)}
                  title="让所有电机在当前位置重新建立锁力"
                  onClick={() => requestTorque("hold")}
                >
                  {pendingAction === "torque-hold" ? "正在上力…" : "全部上力"}
                </Button>
              </div>
            </Field>
            <div className="card-actions">
              <Button
                variant={connected ? "danger" : "default"}
                disabled={Boolean(pendingAction)}
                onClick={() =>
                  requestExecution(connected ? "disconnect" : "connect")
                }
              >
                {pendingAction === "connect"
                  ? "正在连接…"
                  : pendingAction === "disconnect"
                    ? "正在断开…"
                    : connected
                      ? "断开"
                      : "连接真机"}
              </Button>
              <Button
                variant="outline"
                disabled={Boolean(pendingAction)}
                onClick={() => requestExecution("discover")}
              >
                {pendingAction === "discover" ? "正在刷新…" : "刷新串口"}
              </Button>
              <Button
                variant="outline"
                disabled={Boolean(pendingAction)}
                onClick={() => requestExecution("refresh")}
              >
                {pendingAction === "refresh" ? "正在读取…" : "读取参数"}
              </Button>
              <Button
                variant="outline"
                disabled={Boolean(pendingAction) || !configChanged}
                onClick={saveExecutionConfig}
              >
                {pendingAction === "save-config"
                  ? "正在保存…"
                  : !configChanged
                    ? "执行配置已保存"
                    : "保存执行配置"}
              </Button>
            </div>
            {transport.last_error != null && (
              <p className="error">{String(transport.last_error)}</p>
            )}
          </Card>

          <Card eyebrow="Joint telemetry" title="反馈 / 最后命令对照">
            <table className="telemetry-table">
              <thead>
                <tr>
                  <th>执行器</th>
                  <th>反馈</th>
                  <th>命令</th>
                  <th>差值</th>
                </tr>
              </thead>
              <tbody>
                {feedbackRows.map((row) => (
                  <tr key={row.key}>
                    <td>{row.label}</td>
                    <td>{angle(row.feedback)}°</td>
                    <td>{angle(row.requested)}°</td>
                    <td>
                      {row.feedback != null && row.requested != null
                        ? angle(row.requested - row.feedback)
                        : "—"}
                      °
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </Card>
        </div>

        <Card
          className="span-12"
          eyebrow="Live servo parameters"
          title="真机舵机参数"
        >
          {parameters.length ? (
            <div className="table-scroll">
              <table className="telemetry-table parameter-table">
                <thead>
                  <tr>
                    <th>执行器</th>
                    {parameterColumns.map((column) => (
                      <th key={column.key}>
                        <span className="parameter-heading">
                          {column.key}
                          {column.unit && <small>{column.unit}</small>}
                        </span>
                      </th>
                    ))}
                  </tr>
                </thead>
                <tbody>
                  {parameterActuators.map((actuator) => (
                    <tr key={actuator}>
                      <td>{actuator}</td>
                      {parameterColumns.map((column) => {
                        const item = parameters.find(
                          (candidate) =>
                            candidate.actuator_key === actuator &&
                            candidate.field_key === column.key,
                        );
                        return (
                          <td
                            key={column.key}
                            title={item?.original_error ?? undefined}
                          >
                            {item?.value ?? item?.original_error ?? "—"}
                          </td>
                        );
                      })}
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          ) : (
            <p className="status">连接真机后点击“读取参数”显示实际舵机参数。</p>
          )}
        </Card>

        <Card
          className="span-12"
          eyebrow="Troubleshooting"
          title="排障数据"
          defaultOpen={false}
        >
          <div className="diagnostic-grid">
            <JsonView title="Transport 原始状态" value={transport} />
            <JsonView title="唯一 ArmState" value={arm} />
            <JsonView title="最后 ArmCommand" value={command} />
            <JsonView title="型号 / 执行元数据" value={{ model, execution }} />
          </div>
        </Card>
      </div>
    </Shell>
  );
}
