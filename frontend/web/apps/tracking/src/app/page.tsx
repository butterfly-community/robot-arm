"use client";

import type { AbsolutePoseFrame, ControlInputFrame } from "@robot/contracts";
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

const actions = [
  ["control_active", "接管控制", "boolean"],
  ["confirm_origin", "确认原点", "boolean"],
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
  const bindingsKey = JSON.stringify(bindingStates);
  const [previousBindingsKey, setPreviousBindingsKey] = useState("");
  const [paths, setPaths] = useState<Record<string, string>>({});
  const [inverted, setInverted] = useState<Record<string, boolean>>({});
  const [types, setTypes] = useState<Record<string, "boolean" | "float">>({});
  const selected =
    sources.find(
      (source) => source.source_id === discovery.selected_source_id,
    ) ?? (discovery.selected_source as Record<string, unknown> | undefined);
  const components = (selected?.available_components ?? []) as Array<
    Record<string, unknown>
  >;
  const simulation = (discovery.simulation ?? {}) as Record<string, unknown>;
  const diagnostics = (
    (discovery.diagnostics ?? []) as Array<Record<string, unknown>>
  )[0];
  const controlActive = Boolean(input?.control_active?.value);

  if (previousBindingsKey !== bindingsKey) {
    setPreviousBindingsKey(bindingsKey);
    const current = JSON.parse(bindingsKey) as Array<Record<string, unknown>>;
    setPaths(
      Object.fromEntries(
        current.map((binding) => [
          String(binding.action),
          ((binding.configured_components ?? []) as unknown[])
            .map(String)
            .join(", "),
        ]),
      ),
    );
    setInverted(
      Object.fromEntries(
        current.map((binding) => [
          String(binding.action),
          Boolean(binding.invert),
        ]),
      ),
    );
    setTypes(
      Object.fromEntries(
        current.map((binding) => [
          String(binding.action),
          binding.action_type === "boolean" ? "boolean" : "float",
        ]),
      ),
    );
  }

  async function select(source: Record<string, unknown>) {
    setError(undefined);
    try {
      await post("/api/tracking/source", {
        schema_version: 2,
        request_id: requestId(),
        action: "select",
        driver_id: String(source.driver_id),
        device_id: String(source.device_id),
        source_id: String(source.source_id),
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  async function unselect() {
    if (!selected) return;
    setError(undefined);
    try {
      await post("/api/tracking/source", {
        schema_version: 2,
        request_id: requestId(),
        action: "unselect",
        driver_id: String(selected.driver_id),
        device_id: String(selected.device_id),
        source_id: String(selected.source_id),
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
        source_id: String(selected?.source_id ?? ""),
        bindings: actions
          .map(([action, , defaultType]) => ({
            action,
            action_type: types[action] ?? defaultType,
            component_paths: (paths[action] ?? "")
              .split(",")
              .map((value) => value.trim())
              .filter(Boolean),
            invert: Boolean(inverted[action]),
          }))
          .filter((binding) => binding.component_paths.length > 0),
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
              {pose ? pose.source_id : "等待位姿"}
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
            label="驱动"
            value={String(selected?.driver_id ?? "未选择")}
          />
          <KeyValue label="设备" value={String(selected?.device_id ?? "—")} />
          <div className="source-list">
            {sources.map((source) => {
              const current = source.source_id === discovery.selected_source_id;
              return (
                <div className="source-item" key={String(source.source_id)}>
                  <div className="row-spread">
                    <div>
                      <strong>
                        {String(source.display_name ?? source.source_id)}
                      </strong>
                      <small>{String(source.driver_id ?? "未知驱动")}</small>
                    </div>
                    <Button
                      variant={current ? "outline" : "default"}
                      onClick={() => select(source)}
                    >
                      {current ? "当前输入" : "使用"}
                    </Button>
                  </div>
                </div>
              );
            })}
          </div>
          {selected && (
            <div className="card-actions">
              <Button variant="ghost" onClick={unselect}>
                停止使用当前输入源
              </Button>
            </div>
          )}
        </Card>

        <Card className="span-8" eyebrow="Action mapping" title="功能绑定">
          <div className="binding-grid">
            {actions.map(([action, label, defaultType]) => (
              <Field key={action} label={label} englishLabel={action}>
                <div className="binding-row">
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
                    <option value="boolean">按钮 / 正负按钮对</option>
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
                      {components
                        .filter(
                          (component) =>
                            component.action_type ===
                            (types[action] ?? defaultType),
                        )
                        .map((component) => (
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
            ))}
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
