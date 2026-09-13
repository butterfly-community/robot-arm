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
import {
  post,
  requestId,
  useGateway,
  useDraftValue,
} from "@robot/gateway-client";
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
import { parameterText, servoStatus, type ServoFeedback } from "./servo-info";
import { ParameterEditor } from "./parameter-editor";

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
  const readingInformation = Boolean(transport.parameter_reading);
  const servoTelemetry = transport.arm_telemetry as
    { sample_time_ns: number; actuators: ServoFeedback[] } | undefined;
  const command = transport.last_command as ArmCommand | undefined;
  const [connectionFields, setConnectionFields] = useState<
    Record<string, string>
  >({});
  const resolvedConnectionFields = {
    port: String(transport.selected_endpoint ?? ""),
    ...connectionFields,
  };
  // A connected port is reported state, not an editable draft. Another browser
  // may have changed the connection while this page retained a local draft.
  if (transport.connected) {
    resolvedConnectionFields.port = String(transport.selected_endpoint ?? "");
  }
  const discoveredEndpoints = (transport.discovered_endpoints ?? []) as Array<{
    key: string;
    label: string;
  }>;
  const [showLabels, setShowLabels] = useState(false);
  const [feedbackIntervalOverride, setFeedbackIntervalOverride] = useDraftValue(
    String(transport.feedback_interval_ms ?? ""),
  );
  const [pendingAction, setPendingAction] = useState<ExecutionAction>();
  const [strengthOverride, setStrengthOverride] = useDraftValue(
    String(transport.gripper_strength_percent ?? ""),
  );
  const strengthTarget =
    strengthOverride ?? String(transport.gripper_strength_percent ?? "");
  const configChanged =
    feedbackIntervalOverride !== String(transport.feedback_interval_ms ?? "") ||
    strengthOverride !== String(transport.gripper_strength_percent ?? "");
  const feedbackIntervalMs =
    feedbackIntervalOverride ?? String(transport.feedback_interval_ms ?? "");

  async function requestExecution(
    action: "connect" | "disconnect" | "discover" | "refresh",
  ) {
    setError(undefined);
    if (action === "disconnect") setConnectionFields(resolvedConnectionFields);
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
        fields: resolvedConnectionFields,
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
      setFeedbackIntervalOverride(String(Number(feedbackIntervalMs)));
      setStrengthOverride(String(Number(strengthTarget)));
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
  const connectionError =
    !connected && hardwareSelected
      ? String(transport.last_error ?? "无法取得新的电机反馈")
      : undefined;
  const requestError =
    error && (!connectionError || !error.includes(connectionError))
      ? error
      : undefined;
  const parameterColumns = Array.from(
    new Map(
      parameters.map((item) => [
        item.field_key,
        {
          key: item.field_key,
          unit: item.unit,
          label:
            execution?.parameter_fields.find(
              (field) => field.key === item.field_key,
            )?.label ?? item.field_key,
        },
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
      {requestError && (
        <p className="error" role="alert">
          {requestError}
        </p>
      )}
      {connectionError && (
        <div
          className="error connection-alert"
          role="alert"
          aria-label="机械臂连接异常"
        >
          <strong>
            机械臂连接已断开 · {String(transport.selected_endpoint)}
          </strong>
          <p>{connectionError}</p>
          <p>
            设备恢复后，请在下方“执行连接”栏目点击“连接真机”。连接不会重新执行之前的运动任务。
          </p>
        </div>
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
            ) : null}
            <div className="card-stack">
              {execution?.connection_fields.map((field) => (
                <div key={field.key} className="card-stack">
                  {field.field_type === "endpoint" ? (
                    <>
                      <Field label={`${field.label}选择`}>
                        <select
                          aria-label={`${field.label}选择`}
                          disabled={connected || Boolean(pendingAction)}
                          value={
                            discoveredEndpoints.some(
                              (endpoint) =>
                                endpoint.key === resolvedConnectionFields.port,
                            )
                              ? resolvedConnectionFields.port
                              : ""
                          }
                          onChange={(event) =>
                            setConnectionFields({
                              ...connectionFields,
                              [field.key]: event.currentTarget.value,
                            })
                          }
                        >
                          <option value="" disabled>
                            {discoveredEndpoints.length
                              ? "选择串口，或在下方手动输入"
                              : "未枚举到串口，可刷新或手动输入"}
                          </option>
                          {discoveredEndpoints.map((endpoint) => (
                            <option key={endpoint.key} value={endpoint.key}>
                              {endpoint.label}
                            </option>
                          ))}
                        </select>
                      </Field>
                      <Field label={`${field.label}路径（可手动输入）`}>
                        <Input
                          aria-label={`${field.label}路径（可手动输入）`}
                          required={field.required}
                          disabled={connected || Boolean(pendingAction)}
                          placeholder="/dev/ttyUSB0 或 /dev/serial/by-id/…"
                          value={resolvedConnectionFields.port}
                          onChange={(event) =>
                            setConnectionFields({
                              ...connectionFields,
                              [field.key]: event.currentTarget.value,
                            })
                          }
                        />
                      </Field>
                    </>
                  ) : (
                    <Field label={field.label}>
                      <Input
                        aria-label={field.label}
                        required={field.required}
                        disabled={connected || Boolean(pendingAction)}
                        value={connectionFields[field.key] ?? ""}
                        onChange={(event) =>
                          setConnectionFields({
                            ...connectionFields,
                            [field.key]: event.currentTarget.value,
                          })
                        }
                      />
                    </Field>
                  )}
                </div>
              ))}
              <p className="status">
                选择和手动输入使用同一个路径，连接时保存；已连接时请先断开再更换串口。
              </p>
            </div>
            <Field label="真机反馈周期（ms）">
              <Input
                aria-label="真机反馈周期（ms）"
                disabled={Boolean(pendingAction)}
                type="number"
                value={feedbackIntervalMs}
                onChange={(event) =>
                  setFeedbackIntervalOverride(event.currentTarget.value)
                }
              />
            </Field>
            <Field label="夹持反馈目标（0–100）">
              <Input
                aria-label="夹持反馈目标（0–100）"
                disabled={Boolean(pendingAction)}
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
                disabled={
                  !connected || Boolean(pendingAction) || readingInformation
                }
                onClick={() => requestExecution("refresh")}
              >
                {pendingAction === "refresh" || readingInformation
                  ? `正在读取 ${transport.parameter_read_completed ?? 0}/${transport.parameter_read_total ?? 0}…`
                  : "读取全部舵机信息"}
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
              {configChanged && (
                <Button
                  variant="outline"
                  disabled={Boolean(pendingAction)}
                  onClick={() => {
                    setFeedbackIntervalOverride(undefined);
                    setStrengthOverride(undefined);
                  }}
                >
                  恢复执行配置
                </Button>
              )}
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
          eyebrow="Servo monitor"
          title="全部舵机实时反馈"
        >
          <p className="muted">
            {connected
              ? "原始电机反馈，不裁剪角度或圈数；温度保留 ADC，不假定摄氏度换算。"
              : "未连接：不能将历史反馈当作当前状态。"}
          </p>
          <div className="table-scroll">
            <table className="telemetry-table">
              <thead>
                <tr>
                  <th>执行器</th>
                  <th>电机角度</th>
                  <th>圈数</th>
                  <th>电压 mV</th>
                  <th>电流 mA</th>
                  <th>功率 mW</th>
                  <th>温度 ADC</th>
                  <th>状态</th>
                </tr>
              </thead>
              <tbody>
                {(servoTelemetry?.actuators ?? []).map((servo) => (
                  <tr key={servo.actuator_key}>
                    <td>{servo.actuator_key}</td>
                    <td>
                      {connected && servo.position_tenths_degree != null
                        ? `${(servo.position_tenths_degree / 10).toFixed(1)}°`
                        : "—"}
                    </td>
                    <td>{connected ? (servo.turns ?? "—") : "—"}</td>
                    <td>{connected ? servo.voltage_mv : "—"}</td>
                    <td>{connected ? servo.current_ma : "—"}</td>
                    <td>{connected ? servo.power_mw : "—"}</td>
                    <td>{connected ? servo.temperature_raw : "—"}</td>
                    <td
                      title={`原始状态字节：0x${servo.status.toString(16).padStart(2, "0")}`}
                    >
                      {connected ? servoStatus(servo.status) : "历史数据"}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </Card>

        <Card
          className="span-12"
          eyebrow="Live servo parameters"
          title="舵机设备信息与参数"
        >
          <ParameterEditor
            info={execution}
            parameters={parameters}
            connected={connected}
            transport={transport}
          />
          <p className="muted">
            {readingInformation
              ? `正在读取 ${transport.parameter_read_completed}/${transport.parameter_read_total} 项；实时反馈继续更新。`
              : connected
                ? "显示最近一次实读结果；鼠标移到数值可查看读取时间或错误。"
                : "未连接，以下仅为历史缓存。"}{" "}
            固件显示原始版本编码，内部参数格式版本不是固件版本。
          </p>
          {transport.parameter_error != null && (
            <p className="error">
              部分字段读取失败：{String(transport.parameter_error)}
            </p>
          )}
          {parameters.length ? (
            <div className="table-scroll">
              <table className="telemetry-table parameter-table">
                <thead>
                  <tr>
                    <th>执行器</th>
                    {parameterColumns.map((column) => (
                      <th
                        key={column.key}
                        title={execution?.parameter_help?.[column.key]}
                      >
                        <span className="parameter-heading">
                          <LocalizedLabel
                            text={column.label}
                            english={column.key}
                          />
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
                            title={
                              item
                                ? `${item.original_error ?? ""} 读取时间：${new Date(item.read_time_ns / 1e6).toLocaleString()}`
                                : undefined
                            }
                          >
                            {parameterText(item)}
                          </td>
                        );
                      })}
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          ) : (
            <p className="status">
              连接真机后自动读取，或点击“读取全部舵机信息”刷新。
            </p>
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
