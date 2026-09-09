"use client";

import {
  actionGroupLabels,
  actionGroupOrder,
  feedbackActionCatalog,
  inputActionCatalog,
  schemaVersion,
  virtualFeedbackTarget,
  type AbsolutePoseFrame,
  type ControlInputFrame,
  type InputSimulationItem,
  type InputSimulationState,
} from "@robot/contracts";
import {
  post,
  prepareRelativeControl,
  requestId,
  useGateway,
  useDraftValue,
} from "@robot/gateway-client";
import {
  Button,
  Card,
  Input,
  JsonView,
  KeyValue,
  LocalizedLabel,
  Metric,
  Shell,
  StatusBadge,
} from "@robot/ui";
import { useEffect, useMemo, useRef, useState } from "react";
import { MouseControl } from "./mouse-control";

type InputMode = "button" | "buttons" | "axis";
type ActionDefinition = (typeof inputActionCatalog)[number];
const simulationPhaseLabels: Record<string, string> = {
  held: "鼠标控制中",
  starting: "正在开始",
  lifting: "正在抬升",
  outbound: "正在执行",
  return: "正在返回",
  opening: "正在打开",
  opened: "已经打开",
  closing: "正在闭合",
  active: "正在执行",
  increasing: "反馈增强",
  decreasing: "反馈减弱",
  complete: "演示完成",
};
type BindingDraft = {
  paths: Record<string, string>;
  sourceIds: Record<string, string>;
  inverted: Record<string, boolean>;
  inputModes: Record<string, InputMode>;
  feedbackSourceIds: Record<string, string>;
  feedbackPaths: Record<string, string>;
};

function decodeBindingDraft(configKey: string): BindingDraft {
  type SerializedBinding = [string, string, string | null, boolean, string[]];
  type SerializedFeedback = [string, string | null, string | null];
  const [bindings, feedback] = JSON.parse(configKey) as [
    SerializedBinding[],
    SerializedFeedback[],
  ];
  return {
    paths: Object.fromEntries(
      bindings.map(([action, , , , components]) => [
        action,
        components.join(", "),
      ]),
    ),
    sourceIds: Object.fromEntries(
      bindings.map(([action, , sourceId]) => [action, sourceId ?? ""]),
    ),
    inverted: Object.fromEntries(
      bindings.map(([action, , , invert]) => [action, invert]),
    ),
    inputModes: Object.fromEntries(
      bindings.map(([action, actionType, , , components]) => [
        action,
        actionType === "boolean"
          ? "button"
          : components.length === 2
            ? "buttons"
            : "axis",
      ]),
    ),
    feedbackSourceIds: Object.fromEntries(
      feedback.map(([action, sourceId]) => [action, sourceId ?? ""]),
    ),
    feedbackPaths: Object.fromEntries(
      feedback.map(([action, , path]) => [action, path ?? ""]),
    ),
  };
}

function componentLabel(component: Record<string, unknown>) {
  const path = String(component.path ?? "");
  const localized = String(component.localized_name ?? "");
  return localized && localized !== path ? `${localized} · ${path}` : path;
}

function sourceLabel(
  source: Record<string, unknown> | undefined,
  fallback = "",
) {
  return String(
    source?.custom_name ??
      source?.display_name ??
      source?.source_id ??
      fallback,
  );
}

function simulationPhaseLabel(phase: string | null | undefined) {
  return simulationPhaseLabels[phase ?? ""] ?? "演示中";
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

  return <Metric label="采集频率" value={displayed} unit="Hz" />;
}

function DeviceNameEditor({
  source,
  onSave,
}: {
  source: Record<string, unknown>;
  onSave: (sourceId: string, customName: string) => Promise<boolean>;
}) {
  const savedName = String(source.custom_name ?? "");
  const [name, setName] = useDraftValue(savedName);
  const [saving, setSaving] = useState(false);

  return (
    <div className="device-name-editor">
      <Input
        aria-label={`${String(source.display_name ?? source.source_id)} 自定义名称`}
        placeholder="自定义设备名称"
        value={name}
        disabled={saving}
        onChange={(event) => setName(event.currentTarget.value)}
      />
      <Button
        variant="outline"
        disabled={saving || name === savedName}
        onClick={async () => {
          setSaving(true);
          try {
            if (await onSave(String(source.source_id), name))
              setName(name.trim());
          } finally {
            setSaving(false);
          }
        }}
      >
        {saving ? "保存中…" : name === savedName ? "已保存" : "保存名称"}
      </Button>
    </div>
  );
}

export default function Page() {
  const { snapshot, error, setError, connection } = useGateway("tracking");
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
  const simulation = (discovery.simulation ??
    {}) as unknown as InputSimulationState;
  const diagnostics = (
    (discovery.diagnostics ?? []) as Array<Record<string, unknown>>
  )[0];
  const liveValues = (discovery.live_component_values ?? {}) as Record<
    string,
    Record<string, number>
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
  const backendDraft = useMemo(
    () => decodeBindingDraft(configKey),
    [configKey],
  );
  const [editedDraft, setEditedDraft] = useState<BindingDraft>();
  const draft = editedDraft ?? backendDraft;
  const {
    paths,
    sourceIds,
    inverted,
    inputModes,
    feedbackSourceIds,
    feedbackPaths,
  } = draft;
  const [testSourceId, setTestSourceId] = useState("");
  const [simulationRequest, setSimulationRequest] = useState<string>();
  const [poseSourceRequest, setPoseSourceRequest] = useState<
    "position" | "orientation"
  >();
  const [bindingsApplying, setBindingsApplying] = useState(false);
  const [bindingsResult, setBindingsResult] = useState<
    "idle" | "success" | "error"
  >("idle");
  const updateDraft = (patch: Partial<BindingDraft>) => {
    setEditedDraft({ ...draft, ...patch });
    setBindingsResult("idle");
  };

  const positionSource = sources.find(
    (source) => source.source_id === discovery.position_source_id,
  );
  const orientationSource = sources.find(
    (source) => source.source_id === discovery.orientation_source_id,
  );
  const bindingStateByAction = Object.fromEntries(
    bindingStates.map((binding) => [String(binding.action), binding]),
  );

  async function select(
    component: "position" | "orientation",
    source: Record<string, unknown>,
  ) {
    setError(undefined);
    setPoseSourceRequest(component);
    try {
      await post("/api/tracking/pose-source", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action: "select",
        component,
        driver_id: String(source.driver_id),
        device_id: String(source.device_id),
        source_id: String(source.source_id),
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setPoseSourceRequest(undefined);
    }
  }

  async function unselect(component: "position" | "orientation") {
    setError(undefined);
    setPoseSourceRequest(component);
    try {
      await post("/api/tracking/pose-source", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action: "unselect",
        component,
        driver_id: "",
        device_id: "",
        source_id: "",
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setPoseSourceRequest(undefined);
    }
  }

  async function renameSource(sourceId: string, customName: string) {
    setError(undefined);
    try {
      await post("/api/tracking/source-name", {
        schema_version: schemaVersion,
        request_id: requestId(),
        source_id: sourceId,
        custom_name: customName.trim() || null,
      });
      return true;
    } catch (reason) {
      setError(String(reason));
      return false;
    }
  }

  function setPath(action: string, index: number, value: string) {
    const next = (paths[action] ?? "").split(",").map((item) => item.trim());
    next[index] = value;
    updateDraft({ paths: { ...paths, [action]: next.join(", ") } });
  }

  async function applyBindings() {
    setError(undefined);
    setBindingsApplying(true);
    setBindingsResult("idle");
    try {
      await post("/api/tracking/bindings", {
        schema_version: schemaVersion,
        request_id: requestId(),
        bindings: inputActionCatalog
          .map((definition) => ({
            action: definition.key,
            action_type: definition.actionType,
            source_id: sourceIds[definition.key] ?? "",
            component_paths: (paths[definition.key] ?? "")
              .split(",")
              .map((value) => value.trim())
              .filter(Boolean),
            invert:
              definition.actionType === "float" &&
              Boolean(inverted[definition.key]),
          }))
          .filter(
            (binding) =>
              binding.source_id && binding.component_paths.length > 0,
          ),
        feedback_bindings: feedbackActionCatalog
          .map((definition) => ({
            action: definition.key,
            source_id: feedbackSourceIds[definition.key] ?? "",
            capability_path: feedbackPaths[definition.key] ?? "",
          }))
          .filter((binding) => binding.source_id && binding.capability_path),
      });
      setEditedDraft(undefined);
      setBindingsResult("success");
    } catch (reason) {
      setBindingsResult("error");
      setError(String(reason));
    } finally {
      setBindingsApplying(false);
    }
  }

  async function runSimulation(item: InputSimulationItem, prepare: boolean) {
    setError(undefined);
    setSimulationRequest(item);
    try {
      if (simulation.active) {
        await post("/api/tracking/simulation", {
          schema_version: schemaVersion,
          request_id: requestId(),
          enabled: false,
          item: null,
        });
        if (simulation.item === item) return;
      }
      if (prepare) await prepareRelativeControl();
      await post("/api/tracking/simulation", {
        schema_version: schemaVersion,
        request_id: requestId(),
        enabled: true,
        item,
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setSimulationRequest(undefined);
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
  const testComponents = (
    (testSource?.available_components ?? []) as Array<Record<string, unknown>>
  ).filter((component) => {
    const value = Number(
      liveValues[currentTestSourceId]?.[String(component.path)] ?? 0,
    );
    return value !== 0;
  });

  function bindingSummary(definition: ActionDefinition) {
    const state = bindingStateByAction[definition.key];
    if (!state?.source_id) return "未绑定";
    const source = sources.find(
      (candidate) => candidate.source_id === state.source_id,
    );
    return `${sourceLabel(source, String(state.source_id))} · ${(
      (state.configured_components ?? []) as unknown[]
    )
      .map(String)
      .join(" / ")}`;
  }

  function arbitrationSummary(definition: ActionDefinition) {
    const occupied = definition.domains.flatMap((domain) => {
      if (domain === "position" && positionSource) {
        return [sourceLabel(positionSource)];
      }
      if (domain === "orientation" && orientationSource) {
        return [sourceLabel(orientationSource)];
      }
      return [];
    });
    if (occupied.length) {
      return `位姿域已由 ${[...new Set(occupied)].join("、")} 接管，按钮/轴不叠加`;
    }
    return definition.domains.length
      ? "当前按钮/轴绑定可生效"
      : "不参与位姿来源仲裁";
  }

  function bindingEditor(definition: ActionDefinition) {
    const key = definition.key;
    const mode: InputMode =
      definition.actionType === "boolean"
        ? "button"
        : (inputModes[key] ?? "axis");
    const requiredComponentType = mode === "axis" ? "float" : "boolean";
    const candidateSources = actionSources.filter((source) =>
      (
        (source.available_components ?? []) as Array<Record<string, unknown>>
      ).some((component) => component.action_type === requiredComponentType),
    );
    const source = candidateSources.find(
      (candidate) => candidate.source_id === sourceIds[key],
    );
    const components = (
      (source?.available_components ?? []) as Array<Record<string, unknown>>
    ).filter((component) => component.action_type === requiredComponentType);
    const directions = "directions" in definition ? definition.directions : [];

    return (
      <div className="action-binding-editor">
        <label>
          <span>输入设备</span>
          <select
            aria-label={`${definition.label}输入设备`}
            value={sourceIds[key] ?? ""}
            onChange={(event) => {
              updateDraft({
                sourceIds: {
                  ...sourceIds,
                  [key]: event.currentTarget.value,
                },
                paths: { ...paths, [key]: "" },
              });
            }}
          >
            <option value="">选择输入设备</option>
            {candidateSources.map((candidate) => (
              <option
                key={String(candidate.source_id)}
                value={String(candidate.source_id)}
              >
                {sourceLabel(candidate)}
              </option>
            ))}
          </select>
        </label>
        <label>
          <span>输入方式</span>
          <select
            aria-label={`${definition.label}输入方式`}
            value={mode}
            disabled={definition.actionType === "boolean"}
            onChange={(event) => {
              updateDraft({
                inputModes: {
                  ...inputModes,
                  [key]: event.currentTarget.value as InputMode,
                },
                sourceIds: { ...sourceIds, [key]: "" },
                paths: { ...paths, [key]: "" },
              });
            }}
          >
            {definition.actionType === "boolean" ? (
              <option value="button">单个按钮</option>
            ) : (
              <>
                <option value="axis">连续轴</option>
                <option value="buttons">正负按钮对</option>
              </>
            )}
          </select>
        </label>
        {Array.from({ length: mode === "buttons" ? 2 : 1 }).map((_, index) => {
          const direction =
            mode === "buttons"
              ? index === 0
                ? (directions[0] ?? "负向")
                : (directions[1] ?? "正向")
              : "设备输入";
          return (
            <label key={index}>
              <span>{direction}</span>
              <select
                aria-label={`${definition.label}${direction}`}
                value={(paths[key] ?? "").split(",")[index]?.trim() ?? ""}
                onChange={(event) =>
                  setPath(key, index, event.currentTarget.value)
                }
              >
                <option value="">选择{direction}</option>
                {components.map((component) => (
                  <option
                    key={String(component.path)}
                    value={String(component.path)}
                  >
                    {componentLabel(component)}
                  </option>
                ))}
              </select>
            </label>
          );
        })}
        {definition.actionType === "float" && (
          <label className="compact-check action-invert">
            <input
              type="checkbox"
              checked={Boolean(inverted[key])}
              onChange={(event) =>
                updateDraft({
                  inverted: {
                    ...inverted,
                    [key]: event.currentTarget.checked,
                  },
                })
              }
            />
            {mode === "buttons" ? "交换两侧" : "反转方向"}
          </label>
        )}
      </div>
    );
  }

  return (
    <Shell
      connection={connection}
      section="01 / MANUAL CONTROL BINDINGS"
      title="手动控制绑定"
      description="发现输入设备，选择位置与姿态能力，把按钮和轴绑定为设备无关的控制动作，并测试或模拟输入。"
    >
      {error && (
        <p className="error" role="alert">
          {error}
        </p>
      )}
      <div className="dashboard-grid">
        <MouseControl onError={setError} />
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

        <Card className="span-4" eyebrow="Controller devices" title="输入源">
          <KeyValue
            label="空间位置来源"
            value={sourceLabel(
              positionSource,
              String(discovery.position_source_id ?? "未选择"),
            )}
          />
          <KeyValue
            label="设备自身姿态来源"
            value={sourceLabel(
              orientationSource,
              String(discovery.orientation_source_id ?? "未选择"),
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
                        {String(
                          source.custom_name ??
                            source.display_name ??
                            source.source_id,
                        )}
                      </strong>
                      <small>
                        {source.custom_name
                          ? `${String(source.display_name)} · `
                          : ""}
                        {String(source.driver_id ?? "未知驱动")} · 空间
                        {source.position_capable ? "有" : "无"} · 姿态
                        {source.orientation_capable ? "有" : "无"}
                      </small>
                    </div>
                  </div>
                  <DeviceNameEditor
                    key={String(source.source_id)}
                    source={source}
                    onSave={renameSource}
                  />
                  <div className="card-actions">
                    {Boolean(source.position_capable) && (
                      <Button
                        variant="outline"
                        disabled={Boolean(poseSourceRequest) || positionCurrent}
                        onClick={() => select("position", source)}
                      >
                        {poseSourceRequest === "position"
                          ? "正在设置…"
                          : positionCurrent
                            ? "当前空间来源"
                            : "用作空间来源"}
                      </Button>
                    )}
                    {Boolean(source.orientation_capable) && (
                      <Button
                        variant="outline"
                        disabled={
                          Boolean(poseSourceRequest) || orientationCurrent
                        }
                        onClick={() => select("orientation", source)}
                      >
                        {poseSourceRequest === "orientation"
                          ? "正在设置…"
                          : orientationCurrent
                            ? "当前姿态来源"
                            : "用作姿态来源"}
                      </Button>
                    )}
                  </div>
                </div>
              );
            })}
          </div>
          <div className="card-actions">
            <Button
              variant="outline"
              disabled={Boolean(poseSourceRequest) || !positionSource}
              onClick={() => unselect("position")}
            >
              {poseSourceRequest === "position" ? "正在清除…" : "清除空间来源"}
            </Button>
            <Button
              variant="outline"
              disabled={Boolean(poseSourceRequest) || !orientationSource}
              onClick={() => unselect("orientation")}
            >
              {poseSourceRequest === "orientation"
                ? "正在清除…"
                : "清除姿态来源"}
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
                  {sourceLabel(source)}
                </option>
              ))}
            </select>
            <p>
              按住按钮或推动摇杆、扳机，这里实时显示设备实际触发的组件和值。
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
          action={
            simulation.active ? (
              <StatusBadge tone="cyan">
                {simulationPhaseLabel(simulation.phase)}
              </StatusBadge>
            ) : undefined
          }
        >
          <div className="binding-guide">
            <div>
              <strong>单个按钮</strong>
              <span>按下触发离散动作。</span>
            </div>
            <div>
              <strong>正负按钮对</strong>
              <span>两个按钮分别对应详情中的两个人类方向。</span>
            </div>
            <div>
              <strong>连续轴</strong>
              <span>摇杆或扳机的连续值；反转只交换动作方向。</span>
            </div>
          </div>

          <fieldset
            className="action-groups config-fields"
            disabled={bindingsApplying}
          >
            {actionGroupOrder.map((group) => {
              const definitions = inputActionCatalog.filter(
                (definition) => definition.group === group,
              );
              if (group === "feedback") {
                const feedback = feedbackActionCatalog[0];
                const selectedSource = sources.find(
                  (source) =>
                    source.source_id === feedbackSourceIds[feedback.key],
                );
                const virtualSelected =
                  feedbackSourceIds[feedback.key] ===
                  virtualFeedbackTarget.sourceId;
                const capabilities = virtualSelected
                  ? [
                      {
                        path: virtualFeedbackTarget.capabilityPath,
                        localized_name: "全局浮动圆环",
                      },
                    ]
                  : ((selectedSource?.available_feedback_capabilities ??
                      []) as Array<Record<string, unknown>>);
                return (
                  <section className="action-group" key={group}>
                    <h3>{actionGroupLabels[group]}</h3>
                    <details className="action-binding-item">
                      <summary>
                        <span>
                          <strong>{feedback.label}</strong>
                          <small>
                            {feedbackSourceIds[feedback.key]
                              ? String(
                                  sourceLabel(
                                    selectedSource,
                                    virtualSelected
                                      ? "网页虚拟反馈"
                                      : feedbackSourceIds[feedback.key],
                                  ),
                                )
                              : "未绑定"}
                          </small>
                        </span>
                        <StatusBadge tone="neutral">输出</StatusBadge>
                      </summary>
                      <div className="action-binding-content">
                        <dl className="action-description">
                          <div>
                            <dt>数值语义</dt>
                            <dd>{feedback.semantics}</dd>
                          </div>
                          <div>
                            <dt>系统行为</dt>
                            <dd>{feedback.motion}</dd>
                          </div>
                          <div>
                            <dt>保持不变</dt>
                            <dd>{feedback.invariant}</dd>
                          </div>
                          <div>
                            <dt>目标</dt>
                            <dd>{feedback.reference}</dd>
                          </div>
                        </dl>
                        <div className="action-binding-editor feedback-editor">
                          <label>
                            <span>反馈设备</span>
                            <select
                              aria-label="夹爪力度反馈设备"
                              value={feedbackSourceIds[feedback.key] ?? ""}
                              onChange={(event) => {
                                const sourceId = event.currentTarget.value;
                                updateDraft({
                                  feedbackSourceIds: {
                                    ...feedbackSourceIds,
                                    [feedback.key]: sourceId,
                                  },
                                  feedbackPaths: {
                                    ...feedbackPaths,
                                    [feedback.key]:
                                      sourceId ===
                                      virtualFeedbackTarget.sourceId
                                        ? virtualFeedbackTarget.capabilityPath
                                        : "",
                                  },
                                });
                              }}
                            >
                              <option value="">选择反馈设备</option>
                              <option value={virtualFeedbackTarget.sourceId}>
                                网页虚拟反馈
                              </option>
                              {sources
                                .filter(
                                  (source) =>
                                    (
                                      source.available_feedback_capabilities as
                                        unknown[] | undefined
                                    )?.length,
                                )
                                .map((source) => (
                                  <option
                                    key={String(source.source_id)}
                                    value={String(source.source_id)}
                                  >
                                    {sourceLabel(source)}
                                  </option>
                                ))}
                            </select>
                          </label>
                          <label>
                            <span>反馈能力</span>
                            <select
                              aria-label="夹爪力度反馈能力"
                              value={feedbackPaths[feedback.key] ?? ""}
                              onChange={(event) =>
                                updateDraft({
                                  feedbackPaths: {
                                    ...feedbackPaths,
                                    [feedback.key]: event.currentTarget.value,
                                  },
                                })
                              }
                            >
                              <option value="">选择反馈能力</option>
                              {capabilities.map((capability) => (
                                <option
                                  key={String(capability.path)}
                                  value={String(capability.path)}
                                >
                                  {String(
                                    capability.localized_name ??
                                      capability.path,
                                  )}
                                </option>
                              ))}
                            </select>
                          </label>
                        </div>
                        <div className="action-detail-actions">
                          <Button
                            variant="outline"
                            disabled={Boolean(simulationRequest)}
                            onClick={() =>
                              runSimulation(feedback.item, feedback.prepare)
                            }
                          >
                            {simulation.active &&
                            simulation.item === feedback.item
                              ? "停止测试"
                              : "测试反馈"}
                          </Button>
                        </div>
                      </div>
                    </details>
                  </section>
                );
              }
              if (!definitions.length) return null;
              return (
                <section className="action-group" key={group}>
                  <h3>{actionGroupLabels[group]}</h3>
                  <div className="action-group-items">
                    {definitions.map((definition) => {
                      const sample = sampleValue(input, definition.key);
                      return (
                        <details
                          className="action-binding-item"
                          key={definition.key}
                        >
                          <summary>
                            <span>
                              <strong>{definition.label}</strong>
                              <small>{bindingSummary(definition)}</small>
                            </span>
                            <StatusBadge
                              tone={sample.active ? "cyan" : "neutral"}
                            >
                              {sample.active
                                ? sample.value.toFixed(2)
                                : "未触发"}
                            </StatusBadge>
                          </summary>
                          <div className="action-binding-content">
                            <dl className="action-description">
                              <div>
                                <dt>输入语义</dt>
                                <dd>{definition.semantics}</dd>
                              </div>
                              <div>
                                <dt>机械臂动作</dt>
                                <dd>{definition.motion}</dd>
                              </div>
                              <div>
                                <dt>保持不变</dt>
                                <dd>{definition.invariant}</dd>
                              </div>
                              <div>
                                <dt>参考</dt>
                                <dd>{definition.reference}</dd>
                              </div>
                              <div>
                                <dt>当前来源</dt>
                                <dd>{arbitrationSummary(definition)}</dd>
                              </div>
                              {"components" in definition && (
                                <div>
                                  <dt>组成分量</dt>
                                  <dd>{definition.components.join(" + ")}</dd>
                                </div>
                              )}
                            </dl>
                            {bindingEditor(definition)}
                            <div className="action-detail-actions">
                              <Button
                                variant="outline"
                                disabled={Boolean(simulationRequest)}
                                onClick={() =>
                                  runSimulation(
                                    definition.key as InputSimulationItem,
                                    definition.prepare,
                                  )
                                }
                              >
                                {simulation.active &&
                                simulation.item === definition.key
                                  ? "停止演示"
                                  : definition.actionType === "boolean"
                                    ? "测试动作"
                                    : "演示动作"}
                              </Button>
                              {simulation.active &&
                                simulation.item === definition.key && (
                                  <StatusBadge tone="cyan">
                                    {simulationPhaseLabel(simulation.phase)}
                                  </StatusBadge>
                                )}
                            </div>
                          </div>
                        </details>
                      );
                    })}
                  </div>
                </section>
              );
            })}
          </fieldset>

          <div className="card-actions binding-submit">
            <Button
              disabled={bindingsApplying || bindingsResult === "success"}
              onClick={applyBindings}
            >
              {bindingsApplying
                ? "正在应用绑定"
                : bindingsResult === "success"
                  ? "绑定已应用"
                  : bindingsResult === "error"
                    ? "应用失败，重试"
                    : "应用绑定"}
            </Button>
            {editedDraft && (
              <Button
                variant="outline"
                disabled={bindingsApplying}
                onClick={() => {
                  setEditedDraft(undefined);
                  setBindingsResult("idle");
                }}
              >
                恢复已保存绑定
              </Button>
            )}
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
          eyebrow="Live diagnostics"
          title="实时诊断"
          defaultOpen={false}
        >
          <div className="action-list">
            {inputActionCatalog.map((definition) => {
              const sample = sampleValue(input, definition.key);
              return (
                <div className="action-item" key={definition.key}>
                  <LocalizedLabel
                    text={definition.label}
                    english={definition.key}
                  />
                  <div className="action-meter">
                    <i
                      style={{
                        width: `${Math.min(Math.abs(sample.value) * 100, 100)}%`,
                      }}
                    />
                  </div>
                  <StatusBadge tone={sample.active ? "cyan" : "neutral"}>
                    {sample.value.toFixed(2)}
                  </StatusBadge>
                </div>
              );
            })}
          </div>
          <div className="diagnostic-grid">
            <JsonView title="原始绝对位姿" value={pose} />
            <JsonView title="控制输入" value={input} />
            <JsonView title="设备发现与请求状态" value={discovery} />
          </div>
        </Card>
      </div>
    </Shell>
  );
}
