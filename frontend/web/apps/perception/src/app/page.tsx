"use client";

import {
  schemaVersion,
  type CameraCaptureState,
  type CalibrationResult,
  type CalibrationSessionState,
  type ManipulationTaskState,
  type PerceptionState,
  type SegmentationEdit,
  type RobotModelInfo,
  type WorldScene,
} from "@robot/contracts";
import {
  post,
  requestId,
  useGateway,
  useDraftValue,
} from "@robot/gateway-client";
import Image from "next/image";
import {
  Button,
  Card,
  Disclosure,
  Field,
  HelpDot,
  Input,
  JsonView,
  KeyValue,
  Metric,
  RequestStatus,
  Shell,
  StatusBadge,
} from "@robot/ui";
import { useState } from "react";

import { FloatingCameraVideo } from "./camera-video";
import { SegmentationEditor } from "./segmentation-editor";
import { startPickPlace } from "./start-pick-place";
import {
  PickPlaceProgress,
  pickPlaceStatus,
  type PickPlaceAttempt,
} from "./pick-place-progress";
import { AIPanel } from "./ai-panel";

const initialBoard = {
  squaresX: "5",
  squaresY: "5",
  squareMm: "15",
  markerMm: "11",
  measuredWidthMm: "75",
  measuredHeightMm: "75",
  toolId: "charuco-test-tool",
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

function PerceptionAssetImage({
  src,
  alt,
  width,
  height,
  empty,
}: {
  src: string;
  alt: string;
  width: number;
  height: number;
  empty: string;
}) {
  const [failedSource, setFailedSource] = useState<string>();
  return (
    <div className="perception-preview-frame">
      {failedSource === src ? (
        <div className="visual-empty">{empty}</div>
      ) : (
        <Image
          className="perception-preview"
          src={src}
          alt={alt}
          width={width}
          height={height}
          loading="eager"
          unoptimized
          onError={() => setFailedSource(src)}
        />
      )}
    </div>
  );
}

function CalibrationResultValues({ result }: { result: CalibrationResult }) {
  const residuals = result.translation_residuals_m;
  const rms = residuals.length
    ? Math.sqrt(
        residuals.reduce((sum, value) => sum + value * value, 0) /
          residuals.length,
      ) * 1000
    : undefined;
  return (
    <>
      <KeyValue
        label="求解时间 / 样本数"
        value={`${new Date(result.solved_at_ns / 1e6).toLocaleString("zh-CN")} / ${result.sample_count}`}
      />
      <KeyValue
        label="平移拟合 RMS / 最大残差 · mm"
        value={
          rms === undefined
            ? "—"
            : `${rms.toFixed(2)} / ${(Math.max(...residuals) * 1000).toFixed(2)}`
        }
        hint="各样本拟合平移残差的均方根与最大值，不代表独立测量的绝对定位精度。"
      />
      <KeyValue
        label="相机在底座中：位置 · m"
        value={numbers(result.camera_in_base.position_m, 6)}
      />
      <KeyValue
        label="相机朝向：四元数 x/y/z/w"
        value={numbers(result.camera_in_base.orientation_xyzw, 6)}
      />
      <details className="calibration-board-parameters">
        <summary>标定详细数值</summary>
        <KeyValue label="求解器" value={result.solver} />
        <KeyValue
          label="标定板在夹具中：位置 · m / 四元数"
          value={`${numbers(result.board_in_calibration_tool.position_m, 6)} / ${numbers(result.board_in_calibration_tool.orientation_xyzw, 6)}`}
        />
        <KeyValue
          label="各样本平移残差 · mm"
          value={numbers(
            residuals.map((value) => value * 1000),
            2,
          )}
        />
        <KeyValue
          label="各样本旋转残差 · °"
          value={degrees(result.rotation_residuals_rad)}
        />
        <KeyValue
          label="标定板格数 / 单格 / Marker · mm"
          value={`${result.board.squares_x} × ${result.board.squares_y} / ${result.board.square_size_m * 1000} / ${result.board.marker_size_m * 1000}`}
        />
      </details>
    </>
  );
}

export default function Page() {
  const { snapshot, error, setError, connection } = useGateway("perception");
  const values = snapshot?.values ?? {};
  const transport = values.transport_state as
    | {
        connected: boolean;
        selected_endpoint?: string | null;
        last_error?: string | null;
      }
    | undefined;
  const executionDisconnected =
    transport &&
    !transport.connected &&
    (transport.selected_endpoint != null || transport.last_error != null);
  const perception = values.perception_state as unknown as
    PerceptionState | undefined;
  const camera = values.camera_state as unknown as
    CameraCaptureState | undefined;
  const scene = values.world_scene as unknown as WorldScene | undefined;
  const calibration = values.calibration_state as unknown as
    CalibrationSessionState | undefined;
  const calibrationFailure =
    calibration?.phase === "failed" ? calibration.original_error : undefined;
  const displayedFailure = executionDisconnected
    ? transport.last_error
    : calibrationFailure;
  const requestError =
    error && (!displayedFailure || !error.includes(displayedFailure))
      ? error
      : undefined;
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
  // Follow the accepted task selection while preserving later human edits.
  const [objectId, setObjectId] = useDraftValue<string | undefined>(
    manipulation?.object_id ?? undefined,
  );
  const [regionId, setRegionId] = useDraftValue<string | undefined>(
    manipulation?.placement_region_id ?? undefined,
  );
  const [board, setBoard] = useState(initialBoard);
  const [segmentationModel, setSegmentationModel] = useDraftValue(
    perception?.model ?? "",
  );
  const [graspCollisionDistance, setGraspCollisionDistance] = useDraftValue(
    perception ? String(perception.grasp_collision_distance_m * 1000) : "",
  );
  const [aiBusy, setAIBusy] = useState(false);
  const [cancellingGrasp, setCancellingGrasp] = useState(false);
  const [requestPending, setPending] = useState(false);
  const [pickPlaceAttempt, setPickPlaceAttempt] = useState<PickPlaceAttempt>();
  const [pendingPerceptionAction, setPendingPerceptionAction] =
    useState<string>();
  const [pendingCalibrationAction, setPendingCalibrationAction] =
    useState<CalibrationAction>();
  const pending =
    requestPending ||
    Boolean(pendingPerceptionAction) ||
    Boolean(pendingCalibrationAction) ||
    aiBusy;

  const selectedSourceId = sourceId ?? camera?.selected_source_id ?? "";
  const savedCalibrations = camera?.saved_calibrations ?? [];
  const savedCalibration = savedCalibrations.find(
    (result) => result.camera_source_id === selectedSourceId,
  );
  const visibleSavedCalibrations = selectedSourceId
    ? savedCalibrations.filter(
        (result) => result.camera_source_id === selectedSourceId,
      )
    : savedCalibrations;
  const selectedModelId = segmentationModel ?? perception?.model ?? "";
  const selectedGraspCollisionDistance =
    graspCollisionDistance ??
    (perception ? String(perception.grasp_collision_distance_m * 1000) : "");
  const perceptionRequestResult = values.perception_request_result as
    { request_id?: string } | undefined;
  const canSnapshot = Boolean(
    camera?.streaming && perception?.color_frame && perception?.depth_frame,
  );
  const imageVersion =
    perceptionRequestResult?.request_id ?? perception?.last_scene_sequence ?? 0;
  const asset = (name: string) =>
    `/api/perception/assets/${name}?v=${imageVersion}`;
  const selectedObject =
    objectId === undefined
      ? (scene?.objects[0]?.object_id ?? "")
      : scene?.objects.some((item) => item.object_id === objectId)
        ? objectId
        : "";
  const selectedRegion =
    regionId === undefined
      ? (scene?.placement_regions[0]?.region_id ?? "")
      : scene?.placement_regions.some((item) => item.region_id === regionId)
        ? regionId
        : "";
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
  const modelDirty =
    Boolean(perception) &&
    (selectedModelId !== perception?.model ||
      Number(selectedGraspCollisionDistance) !==
        (perception?.grasp_collision_distance_m ?? 0) * 1000);
  function restoreModelConfig() {
    setSegmentationModel(undefined);
    setGraspCollisionDistance(undefined);
  }
  function restoreCameraConfig() {
    setSourceId(undefined);
    setColorProfileKey(undefined);
    setDepthProfileKey(undefined);
    setOutputFps(undefined);
    setDriverParameterChanges({});
  }

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
      | "apply"
      | "disconnect"
      | "unselect"
      | "refresh"
      | "snapshot"
      | "reset"
      | "reconstruct"
      | "generate_grasps",
    target: "camera" | "model" = "camera",
  ) {
    const appliesModel = action === "apply" && target === "model";
    setPendingPerceptionAction(`${target}:${action}`);
    try {
      if (target === "camera") {
        if (action === "snapshot") {
          return await send("/api/perception/request", {
            schema_version: schemaVersion,
            request_id: requestId(),
            action: "snapshot",
            classes: null,
            placement_labels: null,
          });
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
        if (!accepted) return false;
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
          if (connected) restoreCameraConfig();
          return connected;
        }
        if (
          action === "reset" ||
          action === "unselect" ||
          action === "disconnect"
        )
          restoreCameraConfig();
        return true;
      }
      const accepted = await send("/api/perception/request", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action,
        input_sequence:
          action === "reconstruct"
            ? perception?.last_segmentation_sequence
            : action === "generate_grasps"
              ? scene?.sequence
              : null,
        object_id: action === "generate_grasps" ? selectedObject : null,
        model: appliesModel ? selectedModelId : null,
        classes: null,
        placement_labels: null,
        grasp_collision_distance_m:
          appliesModel && selectedGraspCollisionDistance !== ""
            ? Number(selectedGraspCollisionDistance) / 1000
            : null,
      });
      if (accepted && appliesModel) {
        setGraspCollisionDistance(
          String(Number(selectedGraspCollisionDistance)),
        );
      }
      return accepted;
    } finally {
      setPendingPerceptionAction(undefined);
    }
  }

  async function editSegmentation(
    edit: SegmentationEdit,
    fields?: Record<string, unknown>,
  ) {
    setPendingPerceptionAction(`segmentation:${edit.kind}`);
    try {
      return await send("/api/perception/request", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action: "refresh",
        input_sequence:
          edit.kind === "capture"
            ? null
            : perception?.last_segmentation_sequence,
        segmentation_edit: edit,
        ...fields,
      });
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
      if (!(await perceptionRequest("unselect", "camera")))
        restoreCameraConfig();
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
    setOutputFps(
      String(
        commonFps
          ? Math.min(Number(selectedOutputFps) || commonFps, commonFps)
          : "",
      ),
    );
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
    if (!scene || !selectedObject || !selectedRegion) return;
    setPendingPerceptionAction("pick-place");
    setError(undefined);
    const startedAt = Date.now();
    try {
      await startPickPlace(
        scene,
        selectedObject,
        selectedRegion,
        (path, body) => post(path, body as never),
        requestId,
        (step) => setPickPlaceAttempt({ ...step, startedAt }),
      );
      setPickPlaceAttempt((current) =>
        current ? { ...current, phase: "accepted" } : current,
      );
    } catch (reason) {
      setError(String(reason));
      setPickPlaceAttempt((current) => ({
        phase: "failed",
        failedPhase:
          current?.phase === "generate_grasps" ||
          current?.phase === "mode" ||
          current?.phase === "submit"
            ? current.phase
            : undefined,
        requestId: current?.requestId ?? "",
        startedAt,
        error: String(reason),
      }));
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

  return (
    <Shell
      connection={connection}
      section="03 / PERCEPTION"
      title="场景感知"
      description="深度相机、提示词识别与分割、深度、三维场景和标定集中在同一页面；结构化结果可交给下游动作使用。"
    >
      {requestError && (
        <p className="error" role="alert">
          {requestError}
        </p>
      )}
      {executionDisconnected ? (
        <div
          className="error connection-alert"
          role="alert"
          aria-label="机械臂连接异常"
        >
          <strong>
            机械臂连接已断开 · {transport.selected_endpoint ?? "执行连接"}
          </strong>
          <p>{transport.last_error ?? "无法取得新的电机反馈及 TCP"}</p>
          <p>
            请到<a href="/arm-execution/">机械臂执行页</a>
            重新连接，再重新开始标定。上次保存的标定仍然保留；页面实时连接正常不代表串口正常。
          </p>
        </div>
      ) : calibrationFailure ? (
        <p className="error connection-alert" role="alert">
          本轮标定失败：{calibrationFailure}
        </p>
      ) : null}
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
            title="相机与标定"
            description="相机来源、采集数据、内外参与标定统一管理。"
          />
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
                disabled={pending}
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
              value={
                savedCalibration
                  ? "已有保存的标定"
                  : selectedSourceId === perception?.source_id &&
                      perception.calibrated
                    ? "已有输入外参"
                    : "未标定"
              }
            />
            <Field
              label="彩色流"
              hint="显示驱动当前报告和后端保留的彩色配置；当前不可用的配置会置灰并保留，不会因刷新丢失。"
            >
              <select
                aria-label="彩色流"
                disabled={pending}
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
                disabled={pending}
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
                disabled={pending}
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
                          disabled={pending}
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
                          disabled={pending}
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
            <div className="card-actions">
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
                disabled={pending || !canSnapshot}
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
                title="删除当前相机已保存的配置与标定；不是撤销未保存编辑"
                onClick={() => perceptionRequest("reset")}
              >
                {pendingPerceptionAction === "camera:reset"
                  ? "正在重置…"
                  : "重置当前相机"}
              </Button>
              {(sourceId !== undefined ||
                colorProfileKey !== undefined ||
                depthProfileKey !== undefined ||
                outputFps !== undefined ||
                Object.keys(driverParameterChanges).length > 0) && (
                <Button
                  variant="outline"
                  disabled={pending}
                  onClick={restoreCameraConfig}
                >
                  撤销相机编辑
                </Button>
              )}
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
              label="最近采集帧序号"
              value={String(camera?.last_sequence ?? "—")}
              hint="相机实际发布的帧序号，与运行感知后产生的场景序号不同；停用后保留最后一帧。"
            />
            <KeyValue
              label="错误"
              value={
                camera?.original_error ?? perception?.original_error ?? "无"
              }
            />
            <Disclosure
              title="深度图"
              defaultOpen
              englishTitle="点击更新当前深度快照；彩色视频保留在可拖动浮窗中。"
            >
              <div className="card-actions card-actions-leading">
                <Button
                  variant="outline"
                  disabled={pending || !canSnapshot}
                  title={canSnapshot ? undefined : "等待相机首帧"}
                  onClick={() => perceptionRequest("snapshot")}
                >
                  更新深度预览
                </Button>
              </div>
              {perception?.depth_frame ? (
                <PerceptionAssetImage
                  src={asset("depth.png")}
                  alt="深度图"
                  width={perception.depth_frame.width}
                  height={perception.depth_frame.height}
                  empty="尚未生成预览，点击更新深度预览"
                />
              ) : (
                <div className="visual-empty">等待图像</div>
              )}
              <KeyValue
                label="尺寸 / 编码"
                value={
                  perception?.depth_frame
                    ? `${perception.depth_frame.width} × ${perception.depth_frame.height} · ${perception.depth_frame.encoding}`
                    : "—"
                }
              />
              <KeyValue
                label="坐标系"
                value={perception?.depth_frame?.frame_id ?? "—"}
              />
            </Disclosure>
          </Card>

          <Card
            className="span-6 perception-camera-calibration"
            eyebrow="Camera extrinsic calibration"
            title="相机外参标定"
            action={
              <StatusBadge
                tone={visibleSavedCalibrations.length ? "good" : "neutral"}
              >
                {savedCalibration
                  ? "当前相机已有外参"
                  : !selectedSourceId
                    ? savedCalibrations.length
                      ? "已有保存的标定"
                      : "未选择相机"
                    : "当前相机未标定"}
              </StatusBadge>
            }
          >
            {visibleSavedCalibrations.map((result) => (
              <section
                key={result.camera_source_id}
                aria-label="上次已保存标定"
              >
                <h3>上次已保存标定</h3>
                <KeyValue
                  label="所属相机"
                  value={
                    camera?.available_sources.find(
                      (source) => source.source_id === result.camera_source_id,
                    )?.display_name ?? result.camera_source_id
                  }
                />
                <CalibrationResultValues result={result} />
              </section>
            ))}
            <div className="calibration-context">
              <KeyValue
                label="本轮阶段"
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
                    ? `${Math.min(Math.max(calibration.observations.length, calibration.current_target_index == null ? 0 : calibration.current_target_index + 1), calibration.target_count)} / ${calibration.target_count} · ${calibration.observations.length} 个样本`
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
                          aria-label="横向格数"
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
                          aria-label="纵向格数"
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
                          aria-label="单格边长 · mm"
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
                          aria-label="Marker 边长 · mm"
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
                          aria-label="实测板宽 · mm"
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
                          aria-label="实测板高 · mm"
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
                          aria-label="标定夹具标识"
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
                        Boolean(savedCalibration) ||
                        Boolean(calibration?.active) ||
                        !camera?.selected_source_id ||
                        selectedSourceId !== camera.selected_source_id ||
                        !model?.calibration_targets?.length
                      }
                      title={
                        savedCalibration
                          ? "已有保存的外参；需要更新时使用下方的重新标定"
                          : "自动切换到手动关节控制，依次执行型号声明的标定姿态"
                      }
                      onClick={() => calibrate("start")}
                    >
                      {pendingCalibrationAction === "start"
                        ? "正在启动…"
                        : calibration?.active
                          ? "正在自动标定"
                          : savedCalibration
                            ? "已标定"
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
                        ? calibration.current_target_index == null
                          ? `工作位 · ${calibration.current_target_key}`
                          : `${calibration.current_target_index + 1} / ${calibration.target_count} · ${calibration.current_target_key}`
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
                  {(calibration?.observations.length ?? 0) > 0 &&
                    perception?.color_frame && (
                      <details className="calibration-board-parameters">
                        <summary>本轮 ChArUco 识别叠加图</summary>
                        <PerceptionAssetImage
                          src={`/api/perception/assets/calibration.png?v=${calibration?.run_id}-${calibration?.observations.length}`}
                          alt="本轮 ChArUco 识别叠加图"
                          width={perception.color_frame.width}
                          height={perception.color_frame.height}
                          empty="等待本轮标定图像"
                        />
                      </details>
                    )}
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
                      {calibration?.phase === "applied"
                        ? "本轮已保存"
                        : calibration?.solved_result
                          ? "可以确认"
                          : "等待本轮求解"}
                    </StatusBadge>
                  </div>
                  {calibration?.solved_result ? (
                    <CalibrationResultValues
                      result={calibration.solved_result}
                    />
                  ) : (
                    <p className="status">
                      尚无本轮求解结果
                      {savedCalibration ? "；上方已保存的标定仍然有效" : ""}。
                    </p>
                  )}
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
                    {savedCalibration && (
                      <Button
                        variant="outline"
                        disabled={
                          pending ||
                          Boolean(calibration?.active) ||
                          !camera?.streaming ||
                          selectedSourceId !== camera.selected_source_id ||
                          !model?.calibration_targets?.length
                        }
                        title="开始新一轮标定；旧外参保持生效，确认应用新结果后才替换"
                        onClick={() => calibrate("start")}
                      >
                        {pendingCalibrationAction === "start"
                          ? "正在启动…"
                          : "重新标定"}
                      </Button>
                    )}
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
            <Disclosure
              className="calibration-parameters"
              title="相机内参与外参"
              englishTitle="当前 RGB-D 帧内参、深度比例与生效外参。"
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
                value={numbers(
                  perception?.camera_calibration?.projection_matrix,
                )}
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
                hint="由本卡片的标定流程求得。平移 x/y/z 是相机在机械臂底座坐标中的米制位置，四元数 x/y/z/w 是朝向；共同把相机数据转换到底座坐标。"
              />
            </Disclosure>
          </Card>
        </section>

        <section className="perception-section">
          <PerceptionSectionHeading
            index="02"
            title="AI 与任务"
            description="左侧选择任务并启动，右侧独立或组合使用三种分割。"
          />
          <Card
            className="span-12 perception-task-card"
            eyebrow="自然语言任务、分割与应用场景"
            title="AI"
          >
            <div className="ai-columns">
              <div className="ai-workspace">
                <AIPanel onBusy={setAIBusy} />
                <Disclosure
                  title="抓放场景"
                  englishTitle="选择已三维定位的物体和目标后启动。缺少候选时先生成候选，成功后使用同一场景执行抓放；不会重新运行分割。"
                >
                  <div className="perception-task-grid">
                    <Field label="抓取目标">
                      <select
                        aria-label="抓取目标"
                        disabled={pending}
                        value={selectedObject}
                        onChange={(event) =>
                          setObjectId(event.currentTarget.value)
                        }
                      >
                        <option value="">选择已三维定位的实例</option>
                        {scene?.objects.map((item) => (
                          <option key={item.object_id} value={item.object_id}>
                            {item.label} · {item.object_id}
                          </option>
                        ))}
                      </select>
                    </Field>
                    <Field label="放置区域">
                      <select
                        aria-label="放置区域"
                        disabled={pending}
                        value={selectedRegion}
                        onChange={(event) =>
                          setRegionId(event.currentTarget.value)
                        }
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
                      disabled={
                        pending ||
                        pickPlaceStatus(
                          pickPlaceAttempt,
                          perception,
                          manipulation,
                        ).active ||
                        perception?.task_state === "executing" ||
                        manipulation?.state === "planning" ||
                        manipulation?.state === "executing" ||
                        !selectedObject ||
                        !selectedRegion
                      }
                      onClick={pickPlace}
                    >
                      {pendingPerceptionAction === "pick-place"
                        ? `${pickPlaceStatus(pickPlaceAttempt, perception, manipulation).status}…`
                        : "启动"}
                    </Button>
                  </div>
                  <div className="card-actions">
                    <Button
                      disabled={
                        cancellingGrasp ||
                        !manipulation ||
                        !["planning", "executing"].includes(manipulation.state)
                      }
                      onClick={async () => {
                        if (!manipulation) return;
                        setCancellingGrasp(true);
                        try {
                          await post("/api/motion/cancel", {
                            schema_version: schemaVersion,
                            request_id: requestId(),
                            action: "cancel",
                            target_request_id: manipulation.request_id,
                          });
                        } catch (reason) {
                          setError(String(reason));
                        } finally {
                          setCancellingGrasp(false);
                        }
                      }}
                    >
                      {cancellingGrasp ? "正在请求取消…" : "取消抓放"}
                    </Button>
                  </div>
                  <RequestStatus scope="perception" />
                  <PickPlaceProgress
                    attempt={pickPlaceAttempt}
                    perception={perception}
                    task={manipulation}
                    objectId={selectedObject}
                  />
                </Disclosure>
                <Disclosure
                  title="高级设置"
                  englishTitle="AI 默认模型与抓取模型参数；不影响三种分割入口是否可用。"
                >
                  <div className="perception-model-settings">
                    <Field
                      label="AI 默认分割模型"
                      hint="仅指定 AI 自然语言任务默认使用的模型。三种分割始终独立可用，不受此选择限制。"
                    >
                      <select
                        aria-label="AI 默认分割模型"
                        value={selectedModelId}
                        disabled={pending}
                        onChange={(event) =>
                          setSegmentationModel(event.currentTarget.value)
                        }
                      >
                        {!perception?.available_models?.length && (
                          <option value={selectedModelId}>等待模型目录</option>
                        )}
                        {perception?.available_models?.map((item) => (
                          <option key={item.id} value={item.id}>
                            {item.prompt_free ? "自动分割" : "提示词分割"} ·{" "}
                            {item.label}
                          </option>
                        ))}
                      </select>
                    </Field>
                    <Field
                      label="抓取点云邻近距离 · mm"
                      hint="GraspGenX 官方场景筛选参数：张开夹爪表面采样点与环境点云小于此距离时排除候选。它不是实体碰撞或 MoveIt 膨胀量；过大会排除实际离地的姿态。保存于后端，与模拟或真机来源无关。"
                    >
                      <Input
                        type="number"
                        aria-label="抓取点云邻近距离"
                        disabled={pending}
                        min={0}
                        step="any"
                        value={selectedGraspCollisionDistance}
                        onChange={(event) =>
                          setGraspCollisionDistance(event.currentTarget.value)
                        }
                      />
                    </Field>
                  </div>
                  <div className="card-actions perception-task-actions">
                    {modelDirty && (
                      <StatusBadge tone="warning">有待保存修改</StatusBadge>
                    )}
                    {modelDirty && (
                      <Button
                        variant="outline"
                        disabled={pending}
                        onClick={restoreModelConfig}
                      >
                        恢复已保存配置
                      </Button>
                    )}
                    <Button
                      variant="outline"
                      disabled={pending}
                      onClick={() => perceptionRequest("apply", "model")}
                    >
                      {pendingPerceptionAction === "model:apply"
                        ? "正在保存…"
                        : "保存设置"}
                    </Button>
                  </div>
                </Disclosure>
              </div>
              <div className="ai-workspace">
                <SegmentationEditor
                  state={perception}
                  scene={scene}
                  busy={pending || perception?.task_state === "executing"}
                  canCapture={Boolean(
                    camera?.streaming && perception?.color_frame,
                  )}
                  onEdit={editSegmentation}
                  onSave={(fields) =>
                    send("/api/perception/request", {
                      schema_version: schemaVersion,
                      request_id: requestId(),
                      action: "apply",
                      ...fields,
                    })
                  }
                  onReconstruct={() =>
                    perceptionRequest("reconstruct", "model")
                  }
                />
              </div>
            </div>
            {manipulation?.original_error && (
              <p className="error" role="alert">
                {manipulation.original_error}
              </p>
            )}
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
      <FloatingCameraVideo
        streaming={camera?.streaming ?? false}
        profile={
          activeColorProfile
            ? `${activeColorProfile.width} × ${activeColorProfile.height} · ${activeColorProfile.pixel_format}`
            : "等待彩色流配置"
        }
        rates={`${activeColorProfile?.frames_per_second ?? "—"} / ${camera?.output_frames_per_second ?? "—"} FPS`}
      />
    </Shell>
  );
}
