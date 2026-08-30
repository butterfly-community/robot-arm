"use client";

import type {
  ArmCommand,
  ArmState,
  ExecutionInfo,
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
import { RobotViewer } from "./robot-viewer";

function angle(value: number | undefined) {
  return Number.isFinite(value)
    ? ((Number(value) * 180) / Math.PI).toFixed(2)
    : "—";
}

export default function Page() {
  const { snapshot, error, setError } = useGateway("arm-execution");
  const values = snapshot?.values ?? {};
  const model = values.robot_model_info as unknown as
    RobotModelInfo | undefined;
  const arm = values.arm_state as unknown as ArmState | undefined;
  const motion = values.motion_state as unknown as MotionState | undefined;
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
  const feedbackIntervalMs =
    feedbackIntervalOverride ?? String(transport.feedback_interval_ms ?? "");

  async function requestExecution(
    action: "connect" | "disconnect" | "discover" | "refresh",
  ) {
    setError(undefined);
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
    }
  }

  async function saveExecutionConfig() {
    setError(undefined);
    try {
      await post("/api/arm-execution/config", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action: "apply",
        fields: { feedback_interval_ms: feedbackIntervalMs },
      });
    } catch (reason) {
      setError(String(reason));
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
      section="04 / ARM EXECUTION"
      title="机械臂执行"
      description="三维反馈、最后命令、串口状态和舵机参数形成同一高密度执行台；原始连接状态与消息只在排障区展开。"
    >
      <div className="dashboard-grid">
        <div className="span-12 metric-grid">
          <Metric
            label="Transport"
            value={
              connected
                ? "CONNECTED"
                : hardwareSelected
                  ? "STOPPED"
                  : "SIMULATION"
            }
            tone={connected ? "green" : "cyan"}
          />
          <Metric
            label="Feedback"
            value={String(arm?.feedback_source ?? "WAIT").toUpperCase()}
            tone={arm?.feedback_source === "hardware" ? "green" : undefined}
          />
          <Metric
            label="Endpoint"
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
            <div className="card-actions">
              <Button
                variant={connected ? "danger" : "default"}
                onClick={() =>
                  requestExecution(connected ? "disconnect" : "connect")
                }
              >
                {connected ? "断开" : "连接真机"}
              </Button>
              <Button
                variant="outline"
                onClick={() => requestExecution("discover")}
              >
                刷新串口
              </Button>
              <Button
                variant="outline"
                onClick={() => requestExecution("refresh")}
              >
                读取参数
              </Button>
              <Button variant="outline" onClick={saveExecutionConfig}>
                保存反馈周期
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
      {error && <p className="error">{error}</p>}
    </Shell>
  );
}
