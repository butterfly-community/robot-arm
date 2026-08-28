"use client";

import {
  virtualFeedbackTarget,
  type AbsolutePoseFrame,
  type ControlInputFrame,
} from "@robot/contracts";
import { post, requestId, useGateway } from "@robot/gateway-client";
import {
  Button,
  Card,
  Field,
  JsonView,
  KeyValue,
  LocalizedLabel,
  Metric,
  Shell,
  StatusBadge,
} from "@robot/ui";
import { PoseViewer } from "@robot/visualization";
import { useState } from "react";

const feedbackActions = [["primary_tool", "夹爪力度回馈"]] as const;

const actions = [
  ["control_active", "接管控制", "boolean"],
  ["confirm_origin", "确认原点", "boolean"],
  ["primary_tool_open", "打开夹爪", "boolean"],
  ["primary_tool", "夹爪连续控制", "float"],
  ["move_forward_back", "前后移动", "float"],
  ["move_left_right", "左右移动", "float"],
  ["move_up_down", "上下移动", "float"],
  ["front_pitch", "前部抬起 / 往下", "float"],
  ["horizontal_arc", "左旋 / 右旋", "float"],
] as const;

function sampleValue(input: ControlInputFrame | undefined, key: string) {
  const sample = input?.[key as keyof ControlInputFrame] as
    { value?: number | boolean; is_active?: boolean } | undefined;
  return {
    value:
      typeof sample?.value === "boolean"
        ? sample.value
          ? 1
          : 0
        : Number(sample?.value ?? 0),
    active: Boolean(sample?.is_active),
  };
}

function positionValue(pose: AbsolutePoseFrame | undefined, index: number) {
  const value = pose?.position_m[index];
  return Number.isFinite(value) ? Number(value).toFixed(3) : "—";
}

export default function Page() {
  const { snapshot, error, setError } = useGateway("tracking");
  const values = snapshot?.values ?? {};
  const discovery = (values.discovery_state ?? {}) as Record<string, unknown>;
  const pose = values.absolute_pose as unknown as AbsolutePoseFrame | undefined;
  const input = values.control_input as unknown as
    ControlInputFrame | undefined;
  const sources = (discovery.sources ?? []) as Array<Record<string, unknown>>;
  const bindingStates = (discovery.bindings ?? []) as Array<
    Record<string, unknown>
  >;
  const feedbackStates = (discovery.feedback_bindings ?? []) as Array<
    Record<string, unknown>
  >;
  const configKey = JSON.stringify([bindingStates, feedbackStates]);
  const [previousConfigKey, setPreviousConfigKey] = useState("");
  const [paths, setPaths] = useState<Record<string, string>>({});
  const [sourceIds, setSourceIds] = useState<Record<string, string>>({});
  const [inverted, setInverted] = useState<Record<string, boolean>>({});
  const [types, setTypes] = useState<Record<string, "boolean" | "float">>({});
  const [feedbackSourceIds, setFeedbackSourceIds] = useState<
    Record<string, string>
  >({});
  const [feedbackPaths, setFeedbackPaths] = useState<Record<string, string>>(
    {},
  );
  const positionSource = sources.find(
    (source) => source.source_id === discovery.position_source_id,
  );
  const orientationSource = sources.find(
    (source) => source.source_id === discovery.orientation_source_id,
  );
  const simulation = (discovery.simulation ?? {}) as Record<string, unknown>;
  const diagnostics = (
    (discovery.diagnostics ?? []) as Array<Record<string, unknown>>
  )[0];
  const controlActive = Boolean(input?.control_active?.value);

  if (previousConfigKey !== configKey) {
    setPreviousConfigKey(configKey);
    setPaths(
      Object.fromEntries(
        bindingStates.map((binding) => [
          String(binding.action),
          ((binding.configured_components ?? []) as unknown[])
            .map(String)
            .join(", "),
        ]),
      ),
    );
    setSourceIds(
      Object.fromEntries(
        bindingStates.map((binding) => [
          String(binding.action),
          String(binding.source_id ?? ""),
        ]),
      ),
    );
    setInverted(
      Object.fromEntries(
        bindingStates.map((binding) => [
          String(binding.action),
          Boolean(binding.invert),
        ]),
      ),
    );
    setTypes(
      Object.fromEntries(
        bindingStates.map((binding) => [
          String(binding.action),
          binding.action_type === "boolean" ? "boolean" : "float",
        ]),
      ),
    );
    setFeedbackSourceIds(
      Object.fromEntries(
        feedbackStates.map((binding) => [
          String(binding.action),
          String(binding.source_id ?? ""),
        ]),
      ),
    );
    setFeedbackPaths(
      Object.fromEntries(
        feedbackStates.map((binding) => [
          String(binding.action),
          String(binding.capability_path ?? ""),
        ]),
      ),
    );
  }

  async function select(
    component: "position" | "orientation",
    source: Record<string, unknown>,
  ) {
    setError(undefined);
    try {
      await post("/api/tracking/pose-source", {
        schema_version: 2,
        request_id: requestId(),
        action: "select",
        component,
        driver_id: String(source.driver_id),
        device_id: String(source.device_id),
        source_id: String(source.source_id),
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  async function unselect(component: "position" | "orientation") {
    setError(undefined);
    try {
      await post("/api/tracking/pose-source", {
        schema_version: 2,
        request_id: requestId(),
        action: "unselect",
        component,
        driver_id: "",
        device_id: "",
        source_id: "",
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  function setPath(action: string, index: number, value: string) {
    const next = (paths[action] ?? "").split(",").map((item) => item.trim());
    next[index] = value;
    setPaths({ ...paths, [action]: next.filter(Boolean).join(", ") });
  }

  async function applyBindings() {
    setError(undefined);
    try {
      await post("/api/tracking/bindings", {
        schema_version: 2,
        request_id: requestId(),
        bindings: actions
          .map(([action, , defaultType]) => ({
            action,
            action_type: types[action] ?? defaultType,
            source_id: sourceIds[action] ?? "",
            component_paths: (paths[action] ?? "")
              .split(",")
              .map((value) => value.trim())
              .filter(Boolean),
            invert: Boolean(inverted[action]),
          }))
          .filter(
            (binding) =>
              binding.source_id && binding.component_paths.length > 0,
          ),
        feedback_bindings: feedbackActions
          .map(([action]) => ({
            action,
            source_id: feedbackSourceIds[action] ?? "",
            capability_path: feedbackPaths[action] ?? "",
          }))
          .filter((binding) => binding.source_id && binding.capability_path),
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  async function setSimulation(enabled: boolean) {
    setError(undefined);
    try {
      if (enabled)
        await post("/api/motion/mode", {
          schema_version: 2,
          request_id: requestId(),
          mode: "relative",
        });
      await post("/api/tracking/simulation", {
        schema_version: 2,
        request_id: requestId(),
        enabled,
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  return (
    <Shell
      section="01 / INPUT ACQUISITION"
      title="输入采集"
      description="空间位置和设备自身姿态是主视图；驱动、设备和原始消息只作为连接配置与排障依据。"
    >
      <div className="dashboard-grid">
        <div className="span-12 metric-grid">
          <Metric
            label="Position X"
            value={positionValue(pose, 0)}
            unit="m"
            tone="red"
          />
          <Metric
            label="Position Y"
            value={positionValue(pose, 1)}
            unit="m"
            tone="green"
          />
          <Metric
            label="Position Z"
            value={positionValue(pose, 2)}
            unit="m"
            tone="blue"
          />
          <Metric
            label="Pose tracking"
            value={pose?.flags?.position_tracked ? "TRACKED" : "WAIT"}
            tone={pose?.flags?.position_tracked ? "cyan" : "amber"}
          />
          <Metric
            label="Control"
            value={controlActive ? "ACTIVE" : "IDLE"}
            tone={controlActive ? "green" : undefined}
          />
          <Metric
            label="Observed rate"
            value={
              Number.isFinite(Number(diagnostics?.observed_rate_hz))
                ? Number(diagnostics?.observed_rate_hz).toFixed(1)
                : "—"
            }
            unit="Hz"
          />
        </div>

        <Card
          className="span-8 aligned-row-card viewer-fill-card"
          eyebrow="Three.js realtime view"
          title="空间位置 / 自身姿态"
          action={
            <StatusBadge tone={pose ? "good" : "warning"}>
              {pose
                ? [pose.position_source_id, pose.orientation_source_id]
                    .filter(Boolean)
                    .join(" + ") || "位姿分量未绑定"
                : "等待位姿"}
            </StatusBadge>
          }
        >
          <PoseViewer pose={pose} active={controlActive} />
        </Card>

        <Card
          className="span-4 aligned-row-card"
          eyebrow="Semantic actions"
          title="控制分量"
          action={
            <StatusBadge tone={controlActive ? "good" : "neutral"}>
              {controlActive ? "接管中" : "未接管"}
            </StatusBadge>
          }
        >
          <div className="action-list">
            {actions.map(([key, label]) => {
              const sample = sampleValue(input, key);
              return (
                <div className="action-item" key={key}>
                  <LocalizedLabel text={label} english={key} />
                  <div className="action-meter">
                    <i style={{ width: `${Math.abs(sample.value) * 100}%` }} />
                  </div>
                  <StatusBadge tone={sample.active ? "cyan" : "neutral"}>
                    {sample.value.toFixed(2)}
                  </StatusBadge>
                </div>
              );
            })}
          </div>
          <div className="card-actions">
            <Button onClick={() => setSimulation(!Boolean(simulation.active))}>
              {simulation.active ? "停止模拟数据" : "启动模拟数据"}
            </Button>
            <StatusBadge tone={simulation.active ? "cyan" : "neutral"}>
              {simulation.active
                ? String(simulation.phase ?? "运行中")
                : "模拟关闭"}
            </StatusBadge>
          </div>
        </Card>

        <Card className="span-4" eyebrow="Controller devices" title="输入源">
          <KeyValue
            label="空间位置来源"
            value={String(
              positionSource?.display_name ??
                discovery.position_source_id ??
                "未选择",
            )}
          />
          <KeyValue
            label="设备自身姿态来源"
            value={String(
              orientationSource?.display_name ??
                discovery.orientation_source_id ??
                "未选择",
            )}
          />
          <div className="source-list">
            {sources.map((source) => {
              const positionCurrent =
                source.source_id === discovery.position_source_id;
              const orientationCurrent =
                source.source_id === discovery.orientation_source_id;
              return (
                <div className="source-item" key={String(source.source_id)}>
                  <div className="row-spread">
                    <div>
                      <strong>
                        {String(source.display_name ?? source.source_id)}
                      </strong>
                      <small>
                        {String(source.driver_id ?? "未知驱动")} · 空间
                        {source.position_capable ? "有" : "无"} · 姿态
                        {source.orientation_capable ? "有" : "无"}
                      </small>
                    </div>
                  </div>
                  <div className="card-actions">
                    {Boolean(source.position_capable) && (
                      <Button
                        variant="outline"
                        onClick={() => select("position", source)}
                      >
                        {positionCurrent ? "当前空间来源" : "用作空间来源"}
                      </Button>
                    )}
                    {Boolean(source.orientation_capable) && (
                      <Button
                        variant="outline"
                        onClick={() => select("orientation", source)}
                      >
                        {orientationCurrent ? "当前姿态来源" : "用作姿态来源"}
                      </Button>
                    )}
                  </div>
                </div>
              );
            })}
          </div>
          <div className="card-actions">
            <Button variant="ghost" onClick={() => unselect("position")}>
              清除空间来源
            </Button>
            <Button variant="ghost" onClick={() => unselect("orientation")}>
              清除姿态来源
            </Button>
          </div>
        </Card>

        <Card
          className="span-8"
          eyebrow="Action mapping"
          title="功能与反馈绑定"
        >
          <div className="binding-grid">
            {actions.map(([action, label, defaultType]) => {
              const source = sources.find(
                (candidate) => candidate.source_id === sourceIds[action],
              );
              const components = (source?.available_components ?? []) as Array<
                Record<string, unknown>
              >;
              return (
                <Field key={action} label={label} englishLabel={action}>
                  <div className="binding-row">
                    <select
                      value={sourceIds[action] ?? ""}
                      onChange={(event) => {
                        setSourceIds({
                          ...sourceIds,
                          [action]: event.currentTarget.value,
                        });
                        setPaths({ ...paths, [action]: "" });
                      }}
                    >
                      <option value="">选择输入设备</option>
                      {sources.map((candidate) => (
                        <option
                          key={String(candidate.source_id)}
                          value={String(candidate.source_id)}
                        >
                          {String(
                            candidate.display_name ?? candidate.source_id,
                          )}
                        </option>
                      ))}
                    </select>
                    <select
                      value={types[action] ?? defaultType}
                      disabled={defaultType === "boolean"}
                      onChange={(event) =>
                        setTypes({
                          ...types,
                          [action]: event.currentTarget.value as
                            "boolean" | "float",
                        })
                      }
                    >
                      <option value="boolean">开关 / 正负按钮对</option>
                      <option value="float">连续轴</option>
                    </select>
                    {Array.from({
                      length:
                        defaultType === "float" && types[action] === "boolean"
                          ? 2
                          : 1,
                    }).map((_, index) => (
                      <select
                        key={index}
                        value={
                          (paths[action] ?? "").split(",")[index]?.trim() ?? ""
                        }
                        onChange={(event) =>
                          setPath(action, index, event.currentTarget.value)
                        }
                      >
                        <option value="">
                          {index === 0 &&
                          types[action] === "boolean" &&
                          defaultType === "float"
                            ? "负方向按钮"
                            : index === 1
                              ? "正方向按钮"
                              : "选择设备输入"}
                        </option>
                        {components.map((component) => (
                          <option
                            key={String(component.path)}
                            value={String(component.path)}
                          >
                            {String(component.localized_name ?? component.path)}
                          </option>
                        ))}
                      </select>
                    ))}
                    <label className="compact-check">
                      <input
                        type="checkbox"
                        checked={Boolean(inverted[action])}
                        onChange={(event) =>
                          setInverted({
                            ...inverted,
                            [action]: event.currentTarget.checked,
                          })
                        }
                      />
                      反向
                    </label>
                  </div>
                </Field>
              );
            })}
          </div>
          <div className="binding-grid">
            {feedbackActions.map(([action, label]) => {
              const virtualSelected =
                feedbackSourceIds[action] === virtualFeedbackTarget.sourceId;
              const source = sources.find(
                (candidate) =>
                  candidate.source_id === feedbackSourceIds[action],
              );
              const capabilities = virtualSelected
                ? [
                    {
                      path: virtualFeedbackTarget.capabilityPath,
                      localized_name: "全局浮动圆环",
                    },
                  ]
                : ((source?.available_feedback_capabilities ?? []) as Array<
                    Record<string, unknown>
                  >);
              return (
                <Field
                  key={action}
                  label={label}
                  englishLabel={action + "_feedback"}
                >
                  <div className="binding-row">
                    <select
                      value={feedbackSourceIds[action] ?? ""}
                      onChange={(event) => {
                        const sourceId = event.currentTarget.value;
                        setFeedbackSourceIds({
                          ...feedbackSourceIds,
                          [action]: sourceId,
                        });
                        setFeedbackPaths({
                          ...feedbackPaths,
                          [action]:
                            sourceId === virtualFeedbackTarget.sourceId
                              ? virtualFeedbackTarget.capabilityPath
                              : "",
                        });
                      }}
                    >
                      <option value="">选择反馈设备</option>
                      <option value={virtualFeedbackTarget.sourceId}>
                        网页虚拟反馈
                      </option>
                      {sources
                        .filter(
                          (candidate) =>
                            (
                              candidate.available_feedback_capabilities as
                                unknown[] | undefined
                            )?.length,
                        )
                        .map((candidate) => (
                          <option
                            key={String(candidate.source_id)}
                            value={String(candidate.source_id)}
                          >
                            {String(
                              candidate.display_name ?? candidate.source_id,
                            )}
                          </option>
                        ))}
                    </select>
                    <select
                      value={feedbackPaths[action] ?? ""}
                      onChange={(event) =>
                        setFeedbackPaths({
                          ...feedbackPaths,
                          [action]: event.currentTarget.value,
                        })
                      }
                    >
                      <option value="">选择设备反馈能力</option>
                      {capabilities.map((capability) => (
                        <option
                          key={String(capability.path)}
                          value={String(capability.path)}
                        >
                          {String(capability.localized_name ?? capability.path)}
                        </option>
                      ))}
                    </select>
                  </div>
                </Field>
              );
            })}
          </div>
          <div className="card-actions">
            <Button onClick={applyBindings}>应用绑定</Button>
          </div>
        </Card>

        <Card
          className="span-12"
          eyebrow="Troubleshooting"
          title="排障数据"
          defaultOpen={false}
        >
          <div className="diagnostic-grid">
            <JsonView title="原始绝对位姿" value={pose} />
            <JsonView title="原始 Action" value={input} />
            <JsonView title="设备发现" value={discovery} />
          </div>
        </Card>
      </div>
      {error && <p className="error">{error}</p>}
    </Shell>
  );
}
