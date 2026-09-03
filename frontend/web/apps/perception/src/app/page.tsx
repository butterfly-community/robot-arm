"use client";

import {
  schemaVersion,
  type CameraCaptureState,
  type CalibrationSessionState,
  type ManipulationTaskState,
  type PerceptionState,
  type RobotModelInfo,
  type WorldScene,
} from "@robot/contracts";
import { post, requestId, useGateway } from "@robot/gateway-client";
import Image from "next/image";
import {
  Button,
  Card,
  Field,
  HelpDot,
  Input,
  JsonView,
  KeyValue,
  Metric,
  Shell,
  StatusBadge,
} from "@robot/ui";
import { useState } from "react";

const initialBoard = {
  squaresX: "5",
  squaresY: "5",
  squareMm: "15",
  markerMm: "11",
  measuredWidthMm: "75",
  measuredHeightMm: "75",
  toolId: "charuco-test-tool",
};

const manipulationStateLabels: Record<ManipulationTaskState["state"], string> =
  {
    idle: "等待任务",
    planning: "正在规划",
    executing: "正在执行",
    succeeded: "执行完成",
    failed: "执行失败",
    cancelled: "已取消",
  };

type CalibrationAction = "start" | "apply" | "cancel";

const calibrationPhaseLabels: Record<CalibrationSessionState["phase"], string> =
  {
    idle: "等待开始",
    preparing: "准备运动",
    moving: "移动到标定姿态",
    detecting: "识别 ChArUco",
    solving: "求解外参",
    awaiting_confirmation: "等待确认",
    applied: "已应用",
    failed: "执行失败",
  };

function numbers(values: number[] | undefined, digits = 3) {
  return values?.map((value) => value.toFixed(digits)).join(" / ") ?? "—";
}

function degrees(values: number[] | undefined) {
  return (
    values
      ?.map((value) => `${((value * 180) / Math.PI).toFixed(2)}°`)
      .join(" / ") ?? "—"
  );
}

function driverParameterKey(namespace: string, key: string) {
  return `${namespace}\u0000${key}`;
}

function PerceptionSectionHeading({
  index,
  title,
  description,
}: {
  index: string;
  title: string;
  description: string;
}) {
  return (
    <header className="perception-section-heading">
      <span>{index}</span>
      <div>
        <h2>{title}</h2>
        <p>{description}</p>
      </div>
    </header>
  );
}

export default function Page() {
  const { snapshot, error, setError } = useGateway("perception");
  const values = snapshot?.values ?? {};
  const perception = values.perception_state as unknown as
    PerceptionState | undefined;
  const camera = values.camera_state as unknown as
    CameraCaptureState | undefined;
  const scene = values.world_scene as unknown as WorldScene | undefined;
  const calibration = values.calibration_state as unknown as
    CalibrationSessionState | undefined;
  const manipulation = values.manipulation_state as unknown as
    ManipulationTaskState | undefined;
  const model = values.robot_model_info as unknown as
    RobotModelInfo | undefined;
  const [sourceId, setSourceId] = useState<string>();
  const [colorProfileKey, setColorProfileKey] = useState<string>();
  const [depthProfileKey, setDepthProfileKey] = useState<string>();
  const [outputFps, setOutputFps] = useState<string>();
  const [driverParameterChanges, setDriverParameterChanges] = useState<
    Record<string, string>
  >({});
  const [objectId, setObjectId] = useState("");
  const [regionId, setRegionId] = useState("");
  const [board, setBoard] = useState(initialBoard);
  const [promptText, setPromptText] = useState("");
  const [placementLabels, setPlacementLabels] = useState("");
  const [pending, setPending] = useState(false);
  const [pendingPerceptionAction, setPendingPerceptionAction] =
    useState<string>();
  const [pendingCalibrationAction, setPendingCalibrationAction] =
    useState<CalibrationAction>();

  const selectedSourceId = sourceId ?? camera?.selected_source_id ?? "";
  const selectedPrompts = promptText || perception?.classes.join(", ") || "";
  const selectedPlacementLabels =
    placementLabels || perception?.placement_labels.join(", ") || "";
  const imageVersion =
    perception?.last_frame_time_ns ?? perception?.last_scene_sequence ?? 0;
  const asset = (name: string) =>
    `/api/perception/assets/${name}?v=${imageVersion}`;
  const selectedObject =
    objectId ||
    scene?.objects.find((item) => item.grasp_candidates.length)?.object_id ||
    "";
  const selectedRegion =
    regionId || scene?.placement_regions[0]?.region_id || "";
  const activeSource = camera?.available_sources.find(
    (item) => item.source_id === camera?.selected_source_id,
  );
  const selectedSource = camera?.available_sources.find(
    (item) => item.source_id === selectedSourceId,
  );
  const selectedConfiguration = camera?.configurations.find(
    (item) => item.source_id === selectedSourceId,
  );
  const selectedColorProfileKey =
    colorProfileKey ??
    (selectedSourceId === camera?.selected_source_id
      ? camera?.selected_color_profile_key
      : selectedConfiguration?.color_profile_key) ??
    "";
  const selectedDepthProfileKey =
    depthProfileKey ??
    (selectedSourceId === camera?.selected_source_id
      ? camera?.selected_depth_profile_key
      : selectedConfiguration?.depth_profile_key) ??
    "";
  const activeColorProfile = activeSource?.profiles.find(
    (profile) => profile.key === camera?.selected_color_profile_key,
  );
  const activeDepthProfile = activeSource?.profiles.find(
    (profile) => profile.key === camera?.selected_depth_profile_key,
  );
  const selectedColorProfile = selectedSource?.profiles.find(
    (profile) => profile.key === selectedColorProfileKey,
  );
  const selectedDepthProfile = selectedSource?.profiles.find(
    (profile) => profile.key === selectedDepthProfileKey,
  );
  const maximumOutputFps = Math.min(
    selectedColorProfile?.frames_per_second ?? 0,
    selectedDepthProfile?.frames_per_second ?? 0,
  );
  const selectedOutputFps =
    outputFps ??
    (selectedSourceId === camera?.selected_source_id
      ? String(camera?.output_frames_per_second ?? (maximumOutputFps || ""))
      : String(
          selectedConfiguration?.output_frames_per_second ??
            (maximumOutputFps || ""),
        ));
  const selectedOutputFpsNumber = Number(selectedOutputFps);
  const outputFpsValid =
    selectedSource?.available === true &&
    selectedColorProfile?.available === true &&
    selectedDepthProfile?.available === true &&
    selectedOutputFps !== "" &&
    Number.isFinite(selectedOutputFpsNumber) &&
    selectedOutputFpsNumber > 0 &&
    selectedOutputFpsNumber <= maximumOutputFps;
  const driverParameterInfo = new Map(
    selectedSource?.driver_extensions.flatMap((extension) =>
      extension.parameters.map(
        (parameter) =>
          [
            driverParameterKey(extension.namespace, parameter.key),
            parameter,
          ] as const,
      ),
    ) ?? [],
  );
  const driverParameterChangesValid = Object.entries(
    driverParameterChanges,
  ).every(([key, text]) => {
    const parameter = driverParameterInfo.get(key);
    const value = Number(text);
    return (
      parameter !== undefined &&
      !parameter.read_only &&
      Number.isFinite(value) &&
      value >= parameter.minimum &&
      value <= parameter.maximum
    );
  });
  const graspCandidateCount =
    scene?.objects.reduce(
      (sum, item) => sum + item.grasp_candidates.length,
      0,
    ) ?? 0;

  async function send(path: string, body: Record<string, unknown>) {
    setPending(true);
    setError(undefined);
    try {
      await post(path, body as never);
      return true;
    } catch (reason) {
      setError(String(reason));
      return false;
    } finally {
      setPending(false);
    }
  }

  async function perceptionRequest(
    action:
      "apply" | "disconnect" | "unselect" | "refresh" | "snapshot" | "reset",
    target: "camera" | "model" = "camera",
  ) {
    const appliesModel = action === "apply" && target === "model";
    setPendingPerceptionAction(`${target}:${action}`);
    try {
      if (target === "camera") {
        if (action === "snapshot") {
          await send("/api/perception/request", {
            schema_version: schemaVersion,
            request_id: requestId(),
            action: "snapshot",
            source_id: null,
            classes: null,
            placement_labels: null,
          });
          return;
        }
        const cameraAction = action === "apply" ? "select" : action;
        const accepted = await send("/api/perception/camera", {
          schema_version: schemaVersion,
          request_id: requestId(),
          action: cameraAction,
          source_id: action === "refresh" ? null : selectedSourceId || null,
          color_profile_key:
            action === "apply" ? selectedColorProfileKey || null : null,
          depth_profile_key:
            action === "apply" ? selectedDepthProfileKey || null : null,
          output_frames_per_second:
            action === "apply" ? selectedOutputFpsNumber : null,
          driver_parameters:
            action === "apply"
              ? selectedSource?.driver_extensions.flatMap((extension) =>
                  extension.parameters.flatMap((parameter) => {
                    const value =
                      driverParameterChanges[
                        driverParameterKey(extension.namespace, parameter.key)
                      ];
                    return value === undefined
                      ? []
                      : [
                          {
                            namespace: extension.namespace,
                            key: parameter.key,
                            value: Number(value),
                          },
                        ];
                  }),
                )
              : null,
        });
        if (!accepted) return;
        if (action === "reset") {
          await send("/api/perception/request", {
            schema_version: schemaVersion,
            request_id: requestId(),
            action: "reset",
            source_id: selectedSourceId,
            classes: null,
            placement_labels: null,
          });
        }
        if (action === "apply") {
          const connected = await send("/api/perception/camera", {
            schema_version: schemaVersion,
            request_id: requestId(),
            action: "connect",
            source_id: selectedSourceId,
            color_profile_key: null,
            depth_profile_key: null,
            output_frames_per_second: null,
            driver_parameters: null,
          });
          if (connected) setDriverParameterChanges({});
        }
        return;
      }
      await send("/api/perception/request", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action,
        source_id: null,
        classes: appliesModel
          ? selectedPrompts
              .split(",")
              .map((value) => value.trim())
              .filter(Boolean)
          : null,
        placement_labels: appliesModel
          ? selectedPlacementLabels
              .split(",")
              .map((value) => value.trim())
              .filter(Boolean)
          : null,
      });
      if (action === "disconnect") setSourceId("");
    } finally {
      setPendingPerceptionAction(undefined);
    }
  }

  async function selectCamera(nextSourceId: string) {
    setSourceId(nextSourceId);
    const nextSource = camera?.available_sources.find(
      (source) => source.source_id === nextSourceId,
    );
    const saved = camera?.configurations.find(
      (configuration) => configuration.source_id === nextSourceId,
    );
    const initialProfile = (stream: "color" | "depth") => {
      const savedKey =
        stream === "color"
          ? saved?.color_profile_key
          : saved?.depth_profile_key;
      return (
        nextSource?.profiles.find(
          (profile) => profile.stream === stream && profile.key === savedKey,
        )?.key ??
        nextSource?.profiles.find(
          (profile) =>
            profile.stream === stream &&
            profile.is_default &&
            profile.available,
        )?.key ??
        nextSource?.profiles.find(
          (profile) => profile.stream === stream && profile.available,
        )?.key ??
        ""
      );
    };
    setColorProfileKey(initialProfile("color"));
    setDepthProfileKey(initialProfile("depth"));
    const color = nextSource?.profiles.find(
      (profile) => profile.key === initialProfile("color"),
    );
    const depth = nextSource?.profiles.find(
      (profile) => profile.key === initialProfile("depth"),
    );
    const commonFps = Math.min(
      color?.frames_per_second ?? 0,
      depth?.frames_per_second ?? 0,
    );
    setOutputFps(
      nextSourceId === camera?.selected_source_id
        ? String(camera?.output_frames_per_second ?? (commonFps || ""))
        : String(saved?.output_frames_per_second ?? (commonFps || "")),
    );
    setDriverParameterChanges({});
    if (!nextSourceId && camera?.selected_source_id) {
      await perceptionRequest("unselect", "camera");
    }
  }

  function changeProfile(stream: "color" | "depth", key: string) {
    if (stream === "color") setColorProfileKey(key);
    else setDepthProfileKey(key);
    const changed = selectedSource?.profiles.find(
      (profile) => profile.key === key,
    );
    const other =
      stream === "color" ? selectedDepthProfile : selectedColorProfile;
    const commonFps = Math.min(
      changed?.frames_per_second ?? 0,
      other?.frames_per_second ?? 0,
    );
    setOutputFps(String(commonFps || ""));
  }

  function driverParameterValue(
    namespace: string,
    key: string,
    currentValue: number,
  ) {
    const local = driverParameterChanges[driverParameterKey(namespace, key)];
    if (local !== undefined) return local;
    const configured = selectedConfiguration?.driver_parameters.find(
      (parameter) => parameter.namespace === namespace && parameter.key === key,
    );
    if (configured) return String(configured.value);
    return String(currentValue);
  }

  function changeDriverParameter(
    namespace: string,
    key: string,
    value: string,
  ) {
    setDriverParameterChanges((current) => ({
      ...current,
      [driverParameterKey(namespace, key)]: value,
    }));
  }

  async function pickPlace() {
    if (!selectedObject || !selectedRegion) return;
    setPendingPerceptionAction("pick-place");
    try {
      await send("/api/motion/mode", {
        schema_version: schemaVersion,
        request_id: requestId(),
        mode: "perception",
      });
      await send("/api/perception/pick-place", {
        schema_version: schemaVersion,
        request_id: requestId(),
        object_id: selectedObject,
        placement_region_id: selectedRegion,
      });
    } finally {
      setPendingPerceptionAction(undefined);
    }
  }

  async function calibrate(action: CalibrationAction) {
    setPendingCalibrationAction(action);
    try {
      await send("/api/perception/calibration", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action,
        board:
          action === "start"
            ? {
                pattern: "charuco",
                dictionary: "DICT_4X4_50",
                squares_x: Number(board.squaresX),
                squares_y: Number(board.squaresY),
                square_size_m: Number(board.squareMm) / 1000,
                marker_size_m: Number(board.markerMm) / 1000,
                measured_width_m: Number(board.measuredWidthMm) / 1000,
                measured_height_m: Number(board.measuredHeightMm) / 1000,
              }
            : null,
        camera_source_id: camera?.selected_source_id ?? null,
        robot_model_revision: model?.model_revision ?? null,
        calibration_tool_id: action === "start" ? board.toolId : null,
      });
    } finally {
      setPendingCalibrationAction(undefined);
    }
  }

  const capturedFrames = [
    ["彩色图", "Color frame", "color.png", perception?.color_frame],
    ["深度图", "Depth frame", "depth.png", perception?.depth_frame],
  ] as const;

  return (
    <Shell
      section="03 / PERCEPTION"
      title="场景感知"
      description="深度相机、提示词识别与分割、深度、三维场景和标定集中在同一页面；结构化结果可交给下游动作使用。"
    >
      <div className="dashboard-grid">
        <div className="span-12 metric-grid">
          <Metric
            label="感知状态"
            value={perception?.enabled ? "已配置" : "停用"}
            tone={perception?.enabled ? "green" : undefined}
          />
          <Metric
            label="场景序号"
            value={String(perception?.last_scene_sequence ?? "—")}
            tone="cyan"
          />
          <Metric
            label="二维实例"
            value={String(perception?.instances.length ?? 0)}
            tone="amber"
          />
          <Metric
            label="三维物体"
            value={String(scene?.objects.length ?? 0)}
            tone="blue"
          />
          <Metric
            label="抓取候选"
            value={String(graspCandidateCount)}
            tone="green"
          />
          <Metric
            label="实例三维点"
            value={String(perception?.point_count ?? 0)}
            unit="points"
          />
        </div>

        <section className="perception-section">
          <PerceptionSectionHeading
            index="01"
            title="感知任务与相机"
            description="左侧选择相机并维护配置，右侧配置通用提示词模型并按需使用结构化结果；下方并排显示标定流程和相机参数。"
          />
          <Card
            className="span-6 aligned-row-card perception-task-card"
            eyebrow="Prompt-driven perception"
            title="感知任务"
            action={
              <StatusBadge
                tone={
                  manipulation?.state === "failed"
                    ? "bad"
                    : manipulation?.state === "succeeded"
                      ? "good"
                      : "neutral"
                }
              >
                {manipulationStateLabels[manipulation?.state ?? "idle"]}
              </StatusBadge>
            }
          >
            <div className="perception-task-summary">
              <div>
                <span>当前相机</span>
                <strong>{camera?.selected_source_id ?? "未选择"}</strong>
              </div>
              <div>
                <span>二维实例</span>
                <strong>{perception?.instances.length ?? 0}</strong>
              </div>
              <div>
                <span>抓取候选</span>
                <strong>{graspCandidateCount}</strong>
              </div>
              <div>
                <span>放置区域</span>
                <strong>{scene?.placement_regions.length ?? 0}</strong>
              </div>
              <div>
                <span>场景序号</span>
                <strong>{perception?.last_scene_sequence ?? "—"}</strong>
              </div>
            </div>
            <div className="perception-model-settings">
              <Field
                label="提示词模型"
                hint="来自感知计算服务实际加载的开放词汇模型；模型按提示词识别和分割画面，不预设抓放场景。"
              >
                <select
                  key={perception?.model ?? "waiting"}
                  aria-label="提示词模型"
                  defaultValue={perception?.model ?? ""}
                >
                  {!perception?.model && <option value="">等待模型</option>}
                  {perception?.model && (
                    <option value={perception.model}>{perception.model}</option>
                  )}
                </select>
              </Field>
              <Field
                label="识别与分割提示词"
                hint="逗号分隔的开放词汇提示词，直接传给当前模型；内容由当前任务决定，不绑定方块、置物筐或抓放场景。"
              >
                <Input
                  value={selectedPrompts}
                  onChange={(event) => setPromptText(event.currentTarget.value)}
                />
              </Field>
              <Field
                label="放置区域角色"
                hint="可选的下游场景角色。填写已识别实例的提示词后，这些实例可作为放置区域；它不参与模型推理，也不是模型类别。"
              >
                <Input
                  value={selectedPlacementLabels}
                  onChange={(event) =>
                    setPlacementLabels(event.currentTarget.value)
                  }
                />
              </Field>
            </div>
            <div className="card-actions perception-task-actions">
              <Button
                variant="outline"
                disabled={pending}
                onClick={() => perceptionRequest("apply", "model")}
              >
                {pendingPerceptionAction === "model:apply"
                  ? "正在保存…"
                  : "保存提示词配置"}
              </Button>
              <Button
                variant="outline"
                disabled={
                  pending ||
                  !camera?.streaming ||
                  !perception?.color_frame ||
                  !perception?.depth_frame ||
                  !perception?.calibrated
                }
                onClick={() => perceptionRequest("refresh", "model")}
              >
                {pendingPerceptionAction === "model:refresh"
                  ? "正在运行…"
                  : "运行一次感知"}
              </Button>
            </div>
            <div className="perception-task-grid">
              <Field label="抓取目标">
                <select
                  value={selectedObject}
                  onChange={(event) => setObjectId(event.currentTarget.value)}
                >
                  <option value="">选择有抓取候选的实例</option>
                  {scene?.objects
                    .filter((item) => item.grasp_candidates.length)
                    .map((item) => (
                      <option key={item.object_id} value={item.object_id}>
                        {item.label} · {item.object_id}
                      </option>
                    ))}
                </select>
              </Field>
              <Field label="放置区域">
                <select
                  value={selectedRegion}
                  onChange={(event) => setRegionId(event.currentTarget.value)}
                >
                  <option value="">选择放置区域</option>
                  {scene?.placement_regions.map((item) => (
                    <option key={item.region_id} value={item.region_id}>
                      {item.label} · {item.region_id}
                    </option>
                  ))}
                </select>
              </Field>
            </div>
            <div className="card-actions perception-task-actions">
              <Button
                variant="outline"
                disabled={pending || !selectedObject || !selectedRegion}
                onClick={pickPlace}
              >
                {pendingPerceptionAction === "pick-place"
                  ? "正在提交…"
                  : "执行抓放"}
              </Button>
            </div>
            <KeyValue
              label="阶段 / 方案 / 代价"
              value={`${manipulation?.stage ?? "—"} / ${manipulation?.solution_count ?? "—"} / ${manipulation?.selected_cost?.toFixed(3) ?? "—"}`}
              hint="阶段来自运动节点当前进度；方案数和代价来自 MTC 规划结果，用于说明最终选择了哪一条方案。"
            />
            <KeyValue
              label="抓取 / 放置位置"
              value={`${numbers(manipulation?.pick_position_m ?? undefined)} / ${numbers(manipulation?.place_position_m ?? undefined)}`}
              hint="来自当前选中物体和放置区域的结构化三维场景，单位为米，坐标系与场景坐标系一致。"
            />
            <KeyValue
              label="错误"
              value={manipulation?.original_error ?? "无"}
            />
          </Card>
          <Card
            className="span-6 aligned-row-card perception-camera-source"
            eyebrow="Camera source"
            title="相机来源与配置"
            action={
              <StatusBadge
                tone={
                  camera?.original_error
                    ? "bad"
                    : camera?.streaming
                      ? "good"
                      : "neutral"
                }
              >
                {camera?.original_error
                  ? "异常"
                  : camera?.streaming
                    ? "已启用"
                    : "未启用"}
              </StatusBadge>
            }
          >
            <Field
              label="相机来源"
              hint="列表来自相机采集节点的主动发现结果；不选择时不会打开设备或发布图像。"
            >
              <select
                aria-label="相机来源"
                value={selectedSourceId}
                onChange={(event) => {
                  void selectCamera(event.currentTarget.value);
                }}
              >
                <option value="">不选择深度相机</option>
                {camera?.available_sources.map((source) => (
                  <option
                    disabled={!source.available}
                    key={source.source_id}
                    value={source.source_id}
                  >
                    {source.display_name}
                    {source.available ? "" : " · 当前不可用（配置已保留）"}
                  </option>
                ))}
              </select>
            </Field>
            <KeyValue
              label="设备型号"
              value={selectedSource?.device_model ?? "—"}
            />
            <KeyValue
              label="序列号"
              value={selectedSource?.serial_number ?? "—"}
            />
            <KeyValue
              label="固件版本"
              value={selectedSource?.firmware_version ?? "—"}
            />
            <KeyValue
              label="连接"
              value={selectedSource?.connection_type ?? "—"}
            />
            <KeyValue
              label="传感器"
              value={selectedSource?.sensors.join("、") || "—"}
            />
            <KeyValue label="驱动" value={selectedSource?.driver_id ?? "—"} />
            <KeyValue
              label="来源 ID"
              value={selectedSource?.source_id ?? "—"}
            />
            <KeyValue
              label="物理端口"
              value={selectedSource?.physical_port ?? "—"}
            />
            <KeyValue
              label="参数与标定"
              value={perception?.calibrated ? "已配置" : "未配置"}
            />
            <Field
              label="彩色流"
              hint="显示驱动当前报告和后端保留的彩色配置；当前不可用的配置会置灰并保留，不会因刷新丢失。"
            >
              <select
                aria-label="彩色流"
                value={selectedColorProfileKey}
                onChange={(event) =>
                  changeProfile("color", event.currentTarget.value)
                }
              >
                <option value="">由驱动能力选择</option>
                {selectedSource?.profiles
                  .filter((profile) => profile.stream === "color")
                  .map((profile) => (
                    <option
                      disabled={!profile.available}
                      key={profile.key}
                      value={profile.key}
                    >
                      {profile.width}×{profile.height} ·{" "}
                      {profile.frames_per_second} FPS · {profile.pixel_format}
                      {profile.available
                        ? ""
                        : ` · 不可用（${profile.unavailable_reason ?? "未知原因"}）`}
                    </option>
                  ))}
              </select>
            </Field>
            <Field
              label="深度流"
              hint="深度比例由活动设备直接报告，不由页面填写。"
            >
              <select
                aria-label="深度流"
                value={selectedDepthProfileKey}
                onChange={(event) =>
                  changeProfile("depth", event.currentTarget.value)
                }
              >
                <option value="">由驱动能力选择</option>
                {selectedSource?.profiles
                  .filter((profile) => profile.stream === "depth")
                  .map((profile) => (
                    <option
                      disabled={!profile.available}
                      key={profile.key}
                      value={profile.key}
                    >
                      {profile.width}×{profile.height} ·{" "}
                      {profile.frames_per_second} FPS · {profile.pixel_format}
                      {profile.available
                        ? ""
                        : ` · 不可用（${profile.unavailable_reason ?? "未知原因"}）`}
                    </option>
                  ))}
              </select>
            </Field>
            <Field
              label="上送频率 · FPS"
              hint="设备仍按所选 profile 的采集频率持续读取完整 RGB-D 帧束；采集节点只按这里的频率把最新帧束送给上层，避免慢消费者反压设备。识别与抓取模型仍只在手动点击时运行。"
            >
              <Input
                aria-label="上送频率"
                type="number"
                min="0"
                max={maximumOutputFps || undefined}
                step="any"
                value={selectedOutputFps}
                onChange={(event) => setOutputFps(event.currentTarget.value)}
              />
            </Field>
            {selectedSource?.driver_extensions.map((extension) => (
              <details
                className="diagnostics camera-driver-extension"
                key={extension.namespace}
              >
                <summary>
                  {extension.display_name} · {extension.parameters.length} 项
                </summary>
                <p>
                  这些字段仅由当前驱动按设备实际能力报告；只读项用于诊断，可编辑项在“保存并启用”后由驱动应用。
                </p>
                <div className="camera-driver-parameter-grid">
                  {extension.parameters.map((parameter) => (
                    <Field
                      key={parameter.key}
                      label={`${parameter.sensor_name} · ${parameter.display_name}`}
                      hint={`范围 ${parameter.minimum}–${parameter.maximum}，步长 ${parameter.step || "连续"}，驱动默认 ${parameter.default_value}${parameter.read_only ? "；只读" : ""}`}
                    >
                      {parameter.kind === "boolean" && !parameter.read_only ? (
                        <select
                          aria-label={`${parameter.sensor_name} · ${parameter.display_name}`}
                          value={driverParameterValue(
                            extension.namespace,
                            parameter.key,
                            parameter.current_value,
                          )}
                          onChange={(event) =>
                            changeDriverParameter(
                              extension.namespace,
                              parameter.key,
                              event.currentTarget.value,
                            )
                          }
                        >
                          <option value="1">启用</option>
                          <option value="0">停用</option>
                        </select>
                      ) : (
                        <Input
                          aria-label={`${parameter.sensor_name} · ${parameter.display_name}`}
                          type="number"
                          min={parameter.minimum}
                          max={parameter.maximum}
                          step={parameter.step || "any"}
                          readOnly={parameter.read_only}
                          value={driverParameterValue(
                            extension.namespace,
                            parameter.key,
                            parameter.current_value,
                          )}
                          onChange={(event) =>
                            changeDriverParameter(
                              extension.namespace,
                              parameter.key,
                              event.currentTarget.value,
                            )
                          }
                        />
                      )}
                    </Field>
                  ))}
                </div>
              </details>
            ))}
            {selectedSource && (
              <details className="diagnostics camera-profile-capabilities">
                <summary>
                  驱动支持的流配置 · {selectedSource.profiles.length} 项
                </summary>
                <div className="camera-profile-table">
                  {selectedSource.profiles.map((profile) => (
                    <div key={profile.key}>
                      <span>
                        {profile.stream === "color" ? "彩色" : "深度"}
                      </span>
                      <strong>
                        {profile.width}×{profile.height} ·{" "}
                        {profile.frames_per_second} FPS · {profile.pixel_format}
                        {profile.is_default ? " · 驱动默认" : ""}
                        {profile.available
                          ? ""
                          : ` · 不可用：${profile.unavailable_reason ?? "未知原因"}`}
                      </strong>
                    </div>
                  ))}
                </div>
              </details>
            )}
            <div className="card-actions card-actions-leading">
              <Button
                variant="outline"
                disabled={pending}
                onClick={() => perceptionRequest("refresh")}
              >
                {pendingPerceptionAction === "camera:refresh"
                  ? "正在刷新…"
                  : "刷新相机列表"}
              </Button>
              <Button
                variant="outline"
                disabled={
                  pending ||
                  !camera?.streaming ||
                  !perception?.color_frame ||
                  !perception?.depth_frame
                }
                onClick={() => perceptionRequest("snapshot")}
              >
                {pendingPerceptionAction === "camera:snapshot"
                  ? "正在刷新…"
                  : "刷新图像"}
              </Button>
              <Button
                variant="outline"
                disabled={
                  pending ||
                  !selectedSourceId ||
                  !selectedColorProfileKey ||
                  !selectedDepthProfileKey ||
                  !outputFpsValid ||
                  !driverParameterChangesValid
                }
                onClick={() => perceptionRequest("apply")}
              >
                {pendingPerceptionAction === "camera:apply"
                  ? "正在保存…"
                  : "保存并启用"}
              </Button>
              <Button
                variant="outline"
                disabled={pending || !selectedSourceId}
                onClick={() => perceptionRequest("reset")}
              >
                {pendingPerceptionAction === "camera:reset"
                  ? "正在重置…"
                  : "重置当前相机"}
              </Button>
              <Button
                variant="outline"
                disabled={pending || !camera?.streaming}
                onClick={() => perceptionRequest("disconnect")}
              >
                {pendingPerceptionAction === "camera:disconnect"
                  ? "正在停用…"
                  : "停用相机"}
              </Button>
            </div>
            <KeyValue
              label="当前运行来源"
              value={camera?.selected_source_id ?? "未选择"}
            />
            <KeyValue
              label="活动彩色 / 深度流"
              value={
                activeColorProfile && activeDepthProfile
                  ? `${activeColorProfile.width}×${activeColorProfile.height} @ ${activeColorProfile.frames_per_second} / ${activeDepthProfile.width}×${activeDepthProfile.height} @ ${activeDepthProfile.frames_per_second}`
                  : "—"
              }
            />
            <KeyValue
              label="采集 / 上送实测 FPS"
              value={`${camera?.measured_frames_per_second?.toFixed(1) ?? "—"} / ${camera?.measured_output_frames_per_second?.toFixed(1) ?? "—"}`}
              hint="采集 FPS 表示驱动实际取出的完整帧束；上送 FPS 表示进入 Dora 上层链路的帧束。"
            />
            <KeyValue
              label="设备缺帧 / 主动略过"
              value={`${camera?.dropped_frame_count ?? 0} / ${camera?.skipped_output_frame_count ?? 0}`}
              hint="设备缺帧来自序号跳变；主动略过是采集频率高于配置上送频率时有意不发送的中间帧，两者不混算。"
            />
            <KeyValue
              label="错误"
              value={
                camera?.original_error ??
                perception?.original_error ??
                error ??
                "无"
              }
            />
          </Card>

          <Card
            className="span-6 aligned-row-card perception-camera-parameters"
            eyebrow="Camera parameters"
            title="相机内参与外参"
          >
            <KeyValue
              label="彩色 / 深度流"
              value={`${camera?.selected_color_profile_key ?? "—"} / ${camera?.selected_depth_profile_key ?? "—"}`}
              hint="来自所选驱动声明，决定读取哪两路图像。图像流必须与相机参数流的分辨率配套，否则像素和三维点会错位。"
            />
            <KeyValue
              label="相机参数流"
              value={activeSource ? "随原子 RGB-D 帧提供" : "—"}
              hint="来自所选驱动声明，提供与图像分辨率对应的 K、D、P 标定数据。"
            />
            <KeyValue
              label="内参 K"
              value={numbers(perception?.camera_calibration?.camera_matrix)}
              hint="K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]，由相机参数流提供。fx、fy 是像素焦距，cx、cy 是光心；错误会改变点云的横向尺度和位置。"
            />
            <KeyValue
              label="畸变模型 / D"
              value={
                perception?.camera_calibration
                  ? `${perception.camera_calibration.distortion_model} · ${numbers(perception.camera_calibration.distortion)}`
                  : "—"
              }
              hint="D 由相机参数流提供，元素含义由畸变模型决定；plumb_bob 通常依次为 k1、k2、t1、t2、k3，用于修正径向和切向畸变。"
            />
            <KeyValue
              label="投影矩阵 P"
              value={numbers(perception?.camera_calibration?.projection_matrix)}
              hint="P = [fx′, 0, cx′, Tx; 0, fy′, cy′, Ty; 0, 0, 1, 0]，由相机参数流提供，决定校正后的三维点投影到哪个像素。"
            />
            <KeyValue
              label="深度比例"
              value={`${perception?.depth_scale_m ?? "—"} m / unit`}
              hint="由活动相机驱动随帧报告。原始深度值乘以它得到米，并等比例决定点云、物体距离和尺寸。"
            />
            <KeyValue
              label="外参：平移 / 四元数"
              value={
                perception?.camera_calibration
                  ? `${numbers(perception.camera_calibration.translation_m)} / ${numbers(perception.camera_calibration.orientation_xyzw)}`
                  : "—"
              }
              hint="由下方标定流程求得。平移 x/y/z 是相机在机械臂底座坐标中的米制位置，四元数 x/y/z/w 是朝向；共同把相机数据转换到底座坐标。"
            />
          </Card>

          <Card
            className="span-6 perception-camera-calibration"
            eyebrow="Camera extrinsic calibration"
            title="相机外参标定"
            action={
              <StatusBadge tone={perception?.calibrated ? "good" : "neutral"}>
                {perception?.calibrated ? "当前相机已有外参" : "当前相机未标定"}
              </StatusBadge>
            }
          >
            <div className="calibration-context">
              <KeyValue
                label="当前阶段"
                value={calibrationPhaseLabels[calibration?.phase ?? "idle"]}
              />
              <KeyValue
                label="标定相机"
                value={
                  calibration?.camera_source_id ??
                  camera?.selected_source_id ??
                  "未选择"
                }
              />
              <KeyValue
                label="自动进度"
                value={
                  calibration?.target_count
                    ? `${Math.min((calibration.current_target_index ?? 0) + 1, calibration.target_count)} / ${calibration.target_count} · ${calibration.observations.length} 个样本`
                    : `0 / ${model?.calibration_targets?.length ?? 0} · 0 个样本`
                }
              />
            </div>

            <div className="calibration-flow">
              <section className="calibration-step">
                <span className="calibration-step-index">1</span>
                <div className="calibration-step-body">
                  <div className="calibration-step-heading">
                    <div>
                      <h3 className="label-with-help">
                        <span>确认参数并开始</span>
                        <HelpDot text="机械臂型号提供完整的关节标定姿态组。开始后会自动逐个执行、识别和采样；重新开始会清空当前未应用结果。" />
                      </h3>
                    </div>
                    <StatusBadge
                      tone={
                        model?.calibration_targets?.length ? "cyan" : "neutral"
                      }
                    >
                      {model?.calibration_targets?.length
                        ? `${model.calibration_targets.length} 个姿态`
                        : "等待姿态配置"}
                    </StatusBadge>
                  </div>
                  <details className="calibration-board-parameters">
                    <summary>
                      标定板参数 · {board.squaresX} × {board.squaresY} / 单格{" "}
                      {board.squareMm} mm / Marker {board.markerMm} mm
                    </summary>
                    <div className="calibration-fixed-parameters">
                      <KeyValue
                        label="图案"
                        value="ChArUco"
                        hint="必须与打印标定板的 Pattern 一致。"
                      />
                      <KeyValue
                        label="字典"
                        value="DICT_4X4_50"
                        hint="必须与打印文件采用的 ArUco dictionary 一致。"
                      />
                    </div>
                    <div className="calibration-parameter-grid">
                      <Field label="横向格数" hint="打印生成器的 Squares X。">
                        <Input
                          type="number"
                          disabled={calibration?.active}
                          value={board.squaresX}
                          onChange={(event) =>
                            setBoard({
                              ...board,
                              squaresX: event.currentTarget.value,
                            })
                          }
                        />
                      </Field>
                      <Field label="纵向格数" hint="打印生成器的 Squares Y。">
                        <Input
                          type="number"
                          disabled={calibration?.active}
                          value={board.squaresY}
                          onChange={(event) =>
                            setBoard({
                              ...board,
                              squaresY: event.currentTarget.value,
                            })
                          }
                        />
                      </Field>
                      <Field
                        label="单格边长 · mm"
                        hint="决定标定结果的米制尺度。"
                      >
                        <Input
                          type="number"
                          disabled={calibration?.active}
                          value={board.squareMm}
                          onChange={(event) =>
                            setBoard({
                              ...board,
                              squareMm: event.currentTarget.value,
                            })
                          }
                        />
                      </Field>
                      <Field
                        label="Marker 边长 · mm"
                        hint="黑色 ArUco Marker 的外边长。"
                      >
                        <Input
                          type="number"
                          disabled={calibration?.active}
                          value={board.markerMm}
                          onChange={(event) =>
                            setBoard({
                              ...board,
                              markerMm: event.currentTarget.value,
                            })
                          }
                        />
                      </Field>
                      <Field
                        label="实测板宽 · mm"
                        hint="有效方格区域的实测总宽度。"
                      >
                        <Input
                          type="number"
                          disabled={calibration?.active}
                          value={board.measuredWidthMm}
                          onChange={(event) =>
                            setBoard({
                              ...board,
                              measuredWidthMm: event.currentTarget.value,
                            })
                          }
                        />
                      </Field>
                      <Field
                        label="实测板高 · mm"
                        hint="有效方格区域的实测总高度。"
                      >
                        <Input
                          type="number"
                          disabled={calibration?.active}
                          value={board.measuredHeightMm}
                          onChange={(event) =>
                            setBoard({
                              ...board,
                              measuredHeightMm: event.currentTarget.value,
                            })
                          }
                        />
                      </Field>
                      <Field
                        label="标定夹具标识"
                        hint="仅用于记录标定板安装夹具。"
                      >
                        <Input
                          disabled={calibration?.active}
                          value={board.toolId}
                          onChange={(event) =>
                            setBoard({
                              ...board,
                              toolId: event.currentTarget.value,
                            })
                          }
                        />
                      </Field>
                    </div>
                  </details>
                  <div className="calibration-step-actions">
                    <Button
                      variant="outline"
                      disabled={
                        pending ||
                        !camera?.selected_source_id ||
                        !model?.calibration_targets?.length
                      }
                      title="自动切换到手动关节控制，依次执行型号声明的标定姿态"
                      onClick={() => calibrate("start")}
                    >
                      {pendingCalibrationAction === "start"
                        ? "正在启动…"
                        : calibration?.active
                          ? "重新开始自动标定"
                          : "开始自动标定"}
                    </Button>
                  </div>
                </div>
              </section>

              <section className="calibration-step">
                <span className="calibration-step-index">2</span>
                <div className="calibration-step-body">
                  <div className="calibration-step-heading">
                    <div>
                      <h3 className="label-with-help">
                        <span>自动移动、识别与求解</span>
                        <HelpDot text="每个姿态通过现有手动关节运动接口执行。运动完成后持续读取新相机帧，ChArUco 识别成功才记录实际 TCP 位姿并进入下一姿态。" />
                      </h3>
                    </div>
                    <StatusBadge
                      tone={
                        calibration?.phase === "failed"
                          ? "bad"
                          : calibration?.active
                            ? "cyan"
                            : "neutral"
                      }
                    >
                      {calibrationPhaseLabels[calibration?.phase ?? "idle"]}
                    </StatusBadge>
                  </div>
                  <KeyValue
                    label="当前姿态"
                    value={
                      calibration?.current_target_key
                        ? `${(calibration.current_target_index ?? 0) + 1} / ${calibration.target_count} · ${calibration.current_target_key}`
                        : "—"
                    }
                  />
                  <KeyValue
                    label="流程消息"
                    value={calibration?.stage_message ?? "等待开始"}
                  />
                  <KeyValue
                    label="已记录样本"
                    value={String(calibration?.observations.length ?? 0)}
                  />
                </div>
              </section>

              <section className="calibration-step">
                <span className="calibration-step-index">3</span>
                <div className="calibration-step-body">
                  <div className="calibration-step-heading">
                    <div>
                      <h3 className="label-with-help">
                        <span>检查结果并确认应用</span>
                        <HelpDot text="全部姿态采样后自动使用 OpenCV Robot-World/Hand-Eye SHAH 求解。确认后才会保存到当前相机并发布 base_link 到相机坐标系的外参。" />
                      </h3>
                    </div>
                    <StatusBadge
                      tone={calibration?.solved_result ? "good" : "neutral"}
                    >
                      {calibration?.solved_result ? "可以确认" : "等待求解"}
                    </StatusBadge>
                  </div>
                  <KeyValue
                    label="求解器"
                    value={calibration?.solved_result?.solver ?? "—"}
                  />
                  <KeyValue
                    label="相机在底座中：位置 / 四元数"
                    value={
                      calibration?.solved_result
                        ? `${numbers(calibration.solved_result.camera_in_base.position_m)} / ${numbers(calibration.solved_result.camera_in_base.orientation_xyzw)}`
                        : "—"
                    }
                  />
                  <KeyValue
                    label="标定板在夹具中：位置 / 四元数"
                    value={
                      calibration?.solved_result
                        ? `${numbers(calibration.solved_result.board_in_calibration_tool.position_m)} / ${numbers(calibration.solved_result.board_in_calibration_tool.orientation_xyzw)}`
                        : "—"
                    }
                  />
                  <KeyValue
                    label="各样本平移残差 · m"
                    value={numbers(
                      calibration?.solved_result?.translation_residuals_m,
                    )}
                  />
                  <KeyValue
                    label="各样本旋转残差 · rad / °"
                    value={`${numbers(calibration?.solved_result?.rotation_residuals_rad)} / ${degrees(calibration?.solved_result?.rotation_residuals_rad)}`}
                  />
                  <div className="calibration-step-actions">
                    <Button
                      variant="outline"
                      disabled={
                        pending ||
                        calibration?.phase !== "awaiting_confirmation"
                      }
                      title="确认求解结果，保存到本次标定的相机来源"
                      onClick={() => calibrate("apply")}
                    >
                      {pendingCalibrationAction === "apply"
                        ? "正在保存…"
                        : "确认并应用标定"}
                    </Button>
                    <Button
                      variant="outline"
                      disabled={pending || !calibration?.active}
                      title="终止当前自动标定并清除本次样本；已经应用的历史外参不会被删除"
                      onClick={() => calibrate("cancel")}
                    >
                      {pendingCalibrationAction === "cancel"
                        ? "正在终止…"
                        : "终止本次标定"}
                    </Button>
                  </div>
                  <KeyValue
                    label="标定错误"
                    value={calibration?.original_error ?? "无"}
                  />
                </div>
              </section>
            </div>
          </Card>
        </section>

        <section className="perception-section">
          <PerceptionSectionHeading
            index="02"
            title="采集数据"
            description="这里仅展示相机驱动直接产生的彩色帧和深度帧；对齐、识别、分割与三维实例属于下方 AI 结果。"
          />

          {capturedFrames.map(([title, english, name, frame]) => (
            <Card
              className="span-6 aligned-row-card"
              key={name}
              eyebrow={english}
              title={title}
            >
              {frame ? (
                <Image
                  className="perception-preview"
                  src={asset(name)}
                  alt={title}
                  width={frame.width}
                  height={frame.height}
                  loading="eager"
                  unoptimized
                />
              ) : (
                <div className="visual-empty">等待图像</div>
              )}
              <KeyValue
                label="尺寸 / 编码"
                value={
                  frame
                    ? `${frame.width} × ${frame.height} · ${frame.encoding}`
                    : "—"
                }
              />
              <KeyValue label="坐标系" value={frame?.frame_id ?? "—"} />
            </Card>
          ))}
        </section>

        <section className="perception-section">
          <PerceptionSectionHeading
            index="03"
            title="AI 模型与结果"
            description="推理叠加图、二维实例、三维定位和抓取候选集中在这一组，便于核对结构化输出。"
          />

          <Card
            className="span-6 aligned-row-card"
            eyebrow="Inference overlay"
            title="识别与分割叠加图"
          >
            {perception?.last_scene_sequence != null &&
            perception.color_frame ? (
              <Image
                className="perception-preview"
                src={asset("overlay.png")}
                alt="识别与分割叠加图"
                width={perception.color_frame.width}
                height={perception.color_frame.height}
                loading="eager"
                unoptimized
              />
            ) : (
              <div className="visual-empty">等待模型输出</div>
            )}
            <KeyValue
              label="二维实例 / 三维物体"
              value={`${perception?.instances.length ?? 0} / ${scene?.objects.length ?? 0}`}
            />
            <KeyValue
              label="抓取候选"
              value={String(
                scene?.objects.reduce(
                  (sum, item) => sum + item.grasp_candidates.length,
                  0,
                ) ?? 0,
              )}
            />
            <KeyValue
              label="实例三维点"
              value={String(perception?.point_count ?? 0)}
              hint="由本次识别实例的分割区域与对齐深度生成，仅在点击运行一次感知后更新。"
            />
          </Card>

          <Card
            className="span-6 aligned-row-card"
            eyebrow="Structured 3D scene"
            title="结构化三维场景"
          >
            <div className="perception-scene-results">
              <KeyValue label="场景坐标系" value={scene?.frame_id ?? "—"} />
              <KeyValue
                label="物体 / 放置区 / 显式障碍"
                value={`${scene?.objects.length ?? 0} / ${scene?.placement_regions.length ?? 0} / ${scene?.obstacles.length ?? 0}`}
              />
              <div className="table-scroll perception-instance-table">
                <table className="data-table">
                  <thead>
                    <tr>
                      <th>实例</th>
                      <th>置信度</th>
                      <th>二维框 x1/y1/x2/y2</th>
                      <th>三维中心 m</th>
                      <th>尺寸 m</th>
                      <th>抓取候选</th>
                    </tr>
                  </thead>
                  <tbody>
                    {perception?.instances.map((item) => (
                      <tr key={item.instance_id}>
                        <td>
                          <strong>{item.label}</strong>
                          <small>{item.instance_id}</small>
                        </td>
                        <td>{(item.confidence * 100).toFixed(1)}%</td>
                        <td>{numbers(item.bounding_box_xyxy, 0)}</td>
                        <td>{numbers(item.position_m ?? undefined)}</td>
                        <td>{numbers(item.size_m ?? undefined)}</td>
                        <td>{item.grasp_candidate_count}</td>
                      </tr>
                    ))}
                    {!perception?.instances.length && (
                      <tr>
                        <td colSpan={6}>尚无实例</td>
                      </tr>
                    )}
                  </tbody>
                </table>
              </div>
              {scene?.objects.map((item) => (
                <KeyValue
                  key={item.object_id}
                  label={`${item.label} · ${item.object_id}`}
                  value={`中心 ${numbers(item.pose.position_m)} m · 尺寸 ${numbers(item.size_m)} m · 候选 ${item.grasp_candidates.length}`}
                />
              ))}
              {scene?.placement_regions.map((item) => (
                <KeyValue
                  key={item.region_id}
                  label={`放置区 · ${item.label}`}
                  value={`中心 ${numbers(item.pose.position_m)} m · 来源 ${item.source_object_id ?? "无"}`}
                />
              ))}
            </div>
          </Card>
        </section>

        <Card
          className="span-12"
          eyebrow="Raw DTO"
          title="排障数据"
          defaultOpen={false}
        >
          <JsonView value={{ perception, scene, calibration, manipulation }} />
        </Card>
      </div>
    </Shell>
  );
}
