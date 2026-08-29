"use client";

import {
  virtualFeedbackTarget,
  type AbsolutePoseFrame,
  type ControlInputFrame,
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
  JsonView,
  KeyValue,
  LocalizedLabel,
  Metric,
  Shell,
  StatusBadge,
} from "@robot/ui";
import { useEffect, useRef, useState } from "react";

const feedbackActions = [["primary_tool", "夹爪力度回馈"]] as const;

const actions = [
  ["start_stop", "启动和停止控制", "boolean"],
  ["emergency_stop", "急停", "boolean"],
  ["primary_tool_open", "打开夹爪", "boolean"],
  ["primary_tool", "夹爪连续控制", "float"],
  ["move_forward_back", "前后移动", "float"],
  ["move_left_right", "左右移动", "float"],
  ["move_up_down", "上下移动", "float"],
  ["front_pitch", "前部抬起 / 往下", "float"],
  ["horizontal_arc", "左旋 / 右旋", "float"],
] as const;

const actionDirections: Partial<
  Record<(typeof actions)[number][0], readonly [string, string]>
> = {
  move_forward_back: ["后退", "前进"],
  move_left_right: ["右移", "左移"],
  move_up_down: ["下移", "上移"],
  front_pitch: ["前部往下", "前部抬起"],
  horizontal_arc: ["右旋", "左旋"],
};

const armMotionSemantics = [
  [
    "启动和停止控制",
    "每按下一次切换开始或停止",
    "开始时以当前夹爪末端位置和姿态建立本轮相对原点",
    "按钮本身不产生位移；停止后不再发送新的相对目标",
  ],
  [
    "急停",
    "按下时结束当前控制过程",
    "停止发送本轮相对运动目标",
    "下一次启动会重新建立相对原点",
  ],
  ["打开夹爪", "按下时触发一次", "夹爪张开到机械角 90°", "J1–J6 的目标不变"],
  [
    "夹爪连续控制",
    "0 为张开，1 为闭合",
    "输入从 0 到 1，对应夹爪从 90° 到 0° 线性运动",
    "J1–J6 的目标不变",
  ],
  [
    "前后移动",
    "正向前进，负向后退",
    "夹爪末端沿机器人前后方向直线平移",
    "左右位置、高度和夹爪朝向不变",
  ],
  [
    "左右移动",
    "正向左移，负向右移",
    "夹爪末端沿机器人左右方向直线平移",
    "前后位置、高度和夹爪朝向不变",
  ],
  [
    "上下移动",
    "正向上移，负向下移",
    "夹爪末端沿竖直方向直线平移",
    "水平面内的前后、左右位置和夹爪朝向不变",
  ],
  [
    "前部抬起 / 往下",
    "正向抬起，负向往下",
    "夹爪尖端绕工具后部枢轴在竖直平面走圆弧",
    "后部枢轴不动；不是夹爪整体上下平移",
  ],
  [
    "左旋 / 右旋",
    "正向左旋，负向右旋",
    "夹爪尖端绕工具后部枢轴在水平面走圆弧",
    "后部枢轴和尖端高度不变；不是夹爪自身旋转",
  ],
  [
    "夹爪力度反馈",
    "输出 0–100 的力度值",
    "不驱动机械臂，只把当前夹爪力度发送到所选反馈目标",
    "可以绑定扳机反馈、手柄振动或网页浮动圆环",
  ],
] as const;

type InputMode = "button" | "buttons" | "axis";

function componentLabel(component: Record<string, unknown>) {
  const path = String(component.path ?? "");
  const localized = String(component.localized_name ?? "");
  return localized && localized !== path ? `${localized} · ${path}` : path;
}

function sampleValue(input: ControlInputFrame | undefined, key: string) {
  const samples =
    key === "primary_tool" || key === "primary_tool_open"
      ? input?.actuator_actions
      : input;
  const sample = samples?.[key as keyof typeof samples] as
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

function ObservedRate({ value }: { value: string }) {
  const latest = useRef(value);
  const [displayed, setDisplayed] = useState(value);

  useEffect(() => {
    latest.current = value;
  }, [value]);

  useEffect(() => {
    const timer = window.setInterval(() => setDisplayed(latest.current), 1000);
    return () => window.clearInterval(timer);
  }, []);

  return <Metric label="Observed rate" value={displayed} unit="Hz" />;
}

export default function Page() {
  const { snapshot, error, setError } = useGateway("tracking");
  const values = snapshot?.values ?? {};
  const discovery = (values.discovery_state ?? {}) as Record<string, unknown>;
  const pose = values.absolute_pose as unknown as AbsolutePoseFrame | undefined;
  const input = values.control_input as unknown as
    ControlInputFrame | undefined;
  const sources = (discovery.sources ?? []) as Array<Record<string, unknown>>;
  const drivers = (discovery.drivers ?? []) as Array<Record<string, unknown>>;
  const bindingStates = (discovery.bindings ?? []) as Array<
    Record<string, unknown>
  >;
  const feedbackStates = (discovery.feedback_bindings ?? []) as Array<
    Record<string, unknown>
  >;
  const configKey = JSON.stringify([
    bindingStates.map((binding) => [
      binding.action,
      binding.action_type,
      binding.source_id,
      binding.invert,
      binding.configured_components,
    ]),
    feedbackStates.map((binding) => [
      binding.action,
      binding.source_id,
      binding.capability_path,
    ]),
  ]);
  const [previousConfigKey, setPreviousConfigKey] = useState("");
  const [paths, setPaths] = useState<Record<string, string>>({});
  const [sourceIds, setSourceIds] = useState<Record<string, string>>({});
  const [inverted, setInverted] = useState<Record<string, boolean>>({});
  const [inputModes, setInputModes] = useState<Record<string, InputMode>>({});
  const [testSourceId, setTestSourceId] = useState("");
  const [simulationStarting, setSimulationStarting] = useState(false);
  const [bindingsApplying, setBindingsApplying] = useState(false);
  const [bindingsResult, setBindingsResult] = useState<
    "idle" | "success" | "error"
  >("idle");
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
    setInputModes(
      Object.fromEntries(
        bindingStates.map((binding) => [
          String(binding.action),
          binding.action_type === "boolean"
            ? "button"
            : ((binding.configured_components ?? []) as unknown[]).length === 2
              ? "buttons"
              : "axis",
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
    setPaths({ ...paths, [action]: next.join(", ") });
  }

  async function applyBindings() {
    setError(undefined);
    setBindingsApplying(true);
    setBindingsResult("idle");
    try {
      await post("/api/tracking/bindings", {
        schema_version: 2,
        request_id: requestId(),
        bindings: actions
          .map(([action, , actionType]) => ({
            action,
            action_type: actionType,
            source_id: sourceIds[action] ?? "",
            component_paths: (paths[action] ?? "")
              .split(",")
              .map((value) => value.trim())
              .filter(Boolean),
            invert: actionType === "float" && Boolean(inverted[action]),
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
      setBindingsResult("success");
    } catch (reason) {
      setBindingsResult("error");
      setError(String(reason));
    } finally {
      setBindingsApplying(false);
    }
  }

  const actionSources = sources.filter((source) => source.action_capable);
  const currentTestSourceId = actionSources.some(
    (source) => source.source_id === testSourceId,
  )
    ? testSourceId
    : String(actionSources[0]?.source_id ?? "");
  const testSource = actionSources.find(
    (source) => source.source_id === currentTestSourceId,
  );
  const liveValues = (discovery.live_component_values ?? {}) as Record<
    string,
    Record<string, number>
  >;
  const testComponents = (
    (testSource?.available_components ?? []) as Array<Record<string, unknown>>
  ).filter((component) => {
    const value = Number(
      liveValues[currentTestSourceId]?.[String(component.path)] ?? 0,
    );
    return value !== 0;
  });

  async function setSimulation(enabled: boolean) {
    setError(undefined);
    setSimulationStarting(enabled);
    try {
      if (enabled) {
        await prepareRelativeControl();
      }
      await post("/api/tracking/simulation", {
        schema_version: 2,
        request_id: requestId(),
        enabled,
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setSimulationStarting(false);
    }
  }

  return (
    <Shell
      section="01 / INPUT ACQUISITION"
      title="输入采集"
      description="连接输入设备，选择位置与姿态来源，并把实际按钮和轴绑定为设备无关的功能动作。"
    >
      <div className="dashboard-grid">
        <div className="span-12 metric-grid">
          <Metric label="输入驱动" value={String(drivers.length)} tone="cyan" />
          <Metric label="输入设备" value={String(sources.length)} tone="cyan" />
          <Metric
            label="位置来源"
            value={positionSource ? "已选择" : "未选择"}
            tone={positionSource ? "green" : undefined}
          />
          <Metric
            label="姿态来源"
            value={orientationSource ? "已选择" : "未选择"}
            tone={orientationSource ? "green" : undefined}
          />
          <ObservedRate
            value={
              Number.isFinite(Number(diagnostics?.observed_rate_hz))
                ? Number(diagnostics?.observed_rate_hz).toFixed(1)
                : "—"
            }
          />
        </div>

        <Card
          className="span-12"
          eyebrow="Semantic actions"
          title="控制分量"
          action={<StatusBadge tone="neutral">业务动作</StatusBadge>}
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
            <Button
              disabled={simulationStarting}
              onClick={() => setSimulation(!Boolean(simulation.active))}
            >
              {simulationStarting
                ? "正在回到默认位"
                : simulation.active
                  ? "停止模拟数据"
                  : "启动模拟数据"}
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
            <Button variant="outline" onClick={() => unselect("position")}>
              清除空间来源
            </Button>
            <Button variant="outline" onClick={() => unselect("orientation")}>
              清除姿态来源
            </Button>
          </div>
        </Card>

        <Card
          className="span-8"
          eyebrow="Live input inspector"
          title="输入测试"
          action={
            <StatusBadge tone={testComponents.length ? "cyan" : "neutral"}>
              {testComponents.length
                ? `${testComponents.length} 个输入有值`
                : "等待输入"}
            </StatusBadge>
          }
        >
          <div className="input-test-toolbar">
            <select
              aria-label="选择测试设备"
              value={currentTestSourceId}
              onChange={(event) => setTestSourceId(event.currentTarget.value)}
            >
              <option value="">选择要测试的输入设备</option>
              {actionSources.map((source) => (
                <option
                  key={String(source.source_id)}
                  value={String(source.source_id)}
                >
                  {String(source.display_name ?? source.source_id)}
                </option>
              ))}
            </select>
            <p>
              按住一个按钮，或推动摇杆、扳机；这里实时显示设备实际上报的组件和值。
            </p>
          </div>
          <div className="input-test-values" aria-live="polite">
            {testComponents.length ? (
              testComponents.map((component) => {
                const path = String(component.path);
                const value = Number(
                  liveValues[currentTestSourceId]?.[path] ?? 0,
                );
                return (
                  <div className="input-test-value" key={path}>
                    <span>{componentLabel(component)}</span>
                    <strong>{value.toFixed(3)}</strong>
                  </div>
                );
              })
            ) : (
              <p className="empty-state">
                {testSource
                  ? "当前没有按钮按下，轴也位于零位"
                  : "没有可测试的输入设备"}
              </p>
            )}
          </div>
        </Card>

        <Card
          className="span-12"
          eyebrow="Action mapping"
          title="功能与反馈绑定"
        >
          <div className="binding-guide">
            <div>
              <strong>单个按钮</strong>
              <span>按下为开，松开为关，用于独立业务动作。</span>
            </div>
            <div>
              <strong>正负按钮对</strong>
              <span>第一个按钮输出负方向，第二个按钮输出正方向。</span>
            </div>
            <div>
              <strong>连续轴</strong>
              <span>摇杆或扳机的连续数值；反转方向会把输出乘以 -1。</span>
            </div>
          </div>
          <h3 className="binding-section-heading">机械臂动作语义</h3>
          <p className="binding-section-description">
            方向以机器人底座为准：前后、左右位于水平面，上下是竖直方向；不是屏幕或手柄自身方向。
          </p>
          <div className="table-scroll">
            <table className="telemetry-table motion-semantics-table">
              <thead>
                <tr>
                  <th>功能</th>
                  <th>输入 / 输出含义</th>
                  <th>机械臂实际动作</th>
                  <th>保持不变 / 补充</th>
                </tr>
              </thead>
              <tbody>
                {armMotionSemantics.map(([name, direction, motion, fixed]) => (
                  <tr key={name}>
                    <td>{name}</td>
                    <td>{direction}</td>
                    <td>{motion}</td>
                    <td>{fixed}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
          <h3 className="binding-section-heading">功能输入</h3>
          <div className="binding-grid">
            {actions.map(([action, label, actionType]) => {
              const directions = actionDirections[action];
              const source = sources.find(
                (candidate) => candidate.source_id === sourceIds[action],
              );
              const mode =
                actionType === "boolean"
                  ? "button"
                  : (inputModes[action] ?? "axis");
              const components = (
                (source?.available_components ?? []) as Array<
                  Record<string, unknown>
                >
              ).filter((component) =>
                mode === "axis"
                  ? component.action_type === "float"
                  : component.action_type === "boolean",
              );
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
                      {actionSources.map((candidate) => (
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
                      aria-label={`${label}输入方式`}
                      value={mode}
                      disabled={actionType === "boolean"}
                      onChange={(event) => {
                        setInputModes({
                          ...inputModes,
                          [action]: event.currentTarget.value as InputMode,
                        });
                        setPaths({ ...paths, [action]: "" });
                      }}
                    >
                      {actionType === "boolean" ? (
                        <option value="button">单个按钮</option>
                      ) : null}
                      {actionType === "float" && (
                        <>
                          <option value="axis">连续轴</option>
                          <option value="buttons">正负按钮对</option>
                        </>
                      )}
                    </select>
                    <div className="binding-components">
                      {Array.from({
                        length: mode === "buttons" ? 2 : 1,
                      }).map((_, index) => (
                        <select
                          key={index}
                          aria-label={`${label}${
                            mode === "buttons"
                              ? index === 0
                                ? `${directions?.[0] ?? "负方向"}按钮`
                                : `${directions?.[1] ?? "正方向"}按钮`
                              : "设备输入"
                          }`}
                          value={
                            (paths[action] ?? "").split(",")[index]?.trim() ??
                            ""
                          }
                          onChange={(event) =>
                            setPath(action, index, event.currentTarget.value)
                          }
                        >
                          <option value="">
                            {mode === "buttons"
                              ? index === 0
                                ? `${directions?.[0] ?? "负方向"}按钮`
                                : `${directions?.[1] ?? "正方向"}按钮`
                              : "选择设备输入"}
                          </option>
                          {components.map((component) => (
                            <option
                              key={String(component.path)}
                              value={String(component.path)}
                            >
                              {componentLabel(component)}
                            </option>
                          ))}
                        </select>
                      ))}
                    </div>
                    {actionType === "float" && (
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
                        {mode === "buttons" ? "交换正负" : "反转方向"}
                      </label>
                    )}
                  </div>
                </Field>
              );
            })}
          </div>
          <h3 className="binding-section-heading">力度反馈目标</h3>
          <div className="feedback-binding-grid">
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
                  <div className="feedback-binding-row">
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
            <Button disabled={bindingsApplying} onClick={applyBindings}>
              {bindingsApplying
                ? "正在应用绑定"
                : bindingsResult === "success"
                  ? "绑定已应用"
                  : bindingsResult === "error"
                    ? "应用失败，重试"
                    : "应用绑定"}
            </Button>
            {bindingsResult !== "idle" && (
              <StatusBadge
                tone={bindingsResult === "success" ? "good" : "warning"}
              >
                {bindingsResult === "success" ? "绑定已应用" : "应用失败"}
              </StatusBadge>
            )}
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
