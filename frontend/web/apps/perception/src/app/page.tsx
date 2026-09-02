"use client";

import {
  schemaVersion,
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

type CalibrationAction = "start" | "capture" | "solve" | "apply" | "cancel";

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
  const scene = values.world_scene as unknown as WorldScene | undefined;
  const calibration = values.calibration_state as unknown as
    CalibrationSessionState | undefined;
  const manipulation = values.manipulation_state as unknown as
    ManipulationTaskState | undefined;
  const model = values.robot_model_info as unknown as
    RobotModelInfo | undefined;
  const [sourceId, setSourceId] = useState<string>();
  const [objectId, setObjectId] = useState("");
  const [regionId, setRegionId] = useState("");
  const [board, setBoard] = useState(initialBoard);
  const [promptText, setPromptText] = useState("");
  const [placementLabels, setPlacementLabels] = useState("");
  const [depthScale, setDepthScale] = useState("");
  const [pending, setPending] = useState(false);
  const [pendingPerceptionAction, setPendingPerceptionAction] =
    useState<string>();
  const [pendingCalibrationAction, setPendingCalibrationAction] =
    useState<CalibrationAction>();

  const selectedSourceId = sourceId ?? perception?.source_id ?? "";
  const selectedPrompts = promptText || perception?.classes.join(", ") || "";
  const selectedPlacementLabels =
    placementLabels || perception?.placement_labels.join(", ") || "";
  const imageVersion = perception?.last_scene_sequence ?? 0;
  const asset = (name: string) =>
    `/api/perception/assets/${name}?v=${imageVersion}`;
  const selectedObject =
    objectId ||
    scene?.objects.find((item) => item.grasp_candidates.length)?.object_id ||
    "";
  const selectedRegion =
    regionId || scene?.placement_regions[0]?.region_id || "";
  const activeSource = perception?.available_sources.find(
    (item) => item.source_id === perception?.source_id,
  );
  const selectedSource = perception?.available_sources.find(
    (item) => item.source_id === selectedSourceId,
  );
  const selectedDepthScale =
    depthScale || String(selectedSource?.depth_scale_m ?? "");
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
    } catch (reason) {
      setError(String(reason));
    } finally {
      setPending(false);
    }
  }

  async function perceptionRequest(
    action: "apply" | "disconnect" | "refresh" | "reset",
    target: "camera" | "model" = "camera",
  ) {
    const appliesCamera = action === "apply" && target === "camera";
    const appliesModel = action === "apply" && target === "model";
    setPendingPerceptionAction(`${target}:${action}`);
    try {
      await send("/api/perception/request", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action,
        source_id:
          target === "camera" && action !== "refresh"
            ? selectedSourceId || null
            : null,
        depth_scale_m:
          appliesCamera && selectedDepthScale
            ? Number(selectedDepthScale)
            : null,
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
      if (action === "reset") setDepthScale("");
      if (action === "disconnect") setSourceId("");
    } finally {
      setPendingPerceptionAction(undefined);
    }
  }

  async function selectCamera(nextSourceId: string) {
    setSourceId(nextSourceId);
    setDepthScale("");
    await send("/api/perception/request", {
      schema_version: schemaVersion,
      request_id: requestId(),
      action: nextSourceId ? "apply" : "disconnect",
      source_id: nextSourceId || null,
      depth_scale_m: null,
      classes: null,
      placement_labels: null,
    });
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
        camera_source_id: perception?.source_id ?? null,
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
            value={perception?.enabled ? "运行" : "停用"}
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
            label="未结构化点云"
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
                <strong>{perception?.source_id ?? "未选择"}</strong>
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
                  perception?.original_error
                    ? "bad"
                    : perception?.enabled
                      ? "good"
                      : "neutral"
                }
              >
                {perception?.original_error
                  ? "异常"
                  : perception?.enabled
                    ? "已启用"
                    : "未启用"}
              </StatusBadge>
            }
          >
            <Field
              label="相机来源"
              hint="列表来自感知节点当前发现的驱动；不选择时不会发布相机感知数据。"
            >
              <select
                aria-label="相机来源"
                value={selectedSourceId}
                onChange={(event) => {
                  void selectCamera(event.currentTarget.value);
                }}
              >
                <option value="">不选择深度相机</option>
                {perception?.available_sources.map((source) => (
                  <option key={source.source_id} value={source.source_id}>
                    {source.display_name} · {source.source_id}
                  </option>
                ))}
              </select>
            </Field>
            <KeyValue label="驱动" value={selectedSource?.driver_id ?? "—"} />
            <KeyValue
              label="参数与标定"
              value={selectedSource?.calibrated ? "已配置" : "未配置"}
            />
            <Field
              label="深度比例 · m / unit"
              hint="来自相机驱动或设备资料，用来把深度图中的原始整数换算成米；它会直接缩放点云和三维位置。"
            >
              <Input
                type="number"
                value={selectedDepthScale}
                onChange={(event) => setDepthScale(event.currentTarget.value)}
              />
            </Field>
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
                disabled={pending || !selectedSourceId}
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
                disabled={pending || !perception?.enabled}
                onClick={() => perceptionRequest("disconnect")}
              >
                {pendingPerceptionAction === "camera:disconnect"
                  ? "正在停用…"
                  : "停用相机"}
              </Button>
            </div>
            <KeyValue
              label="当前运行来源"
              value={perception?.source_id ?? "未选择"}
            />
            <KeyValue
              label="错误"
              value={perception?.original_error ?? error ?? "无"}
            />
          </Card>

          <Card
            className="span-6 aligned-row-card perception-camera-parameters"
            eyebrow="Camera parameters"
            title="相机内参与外参"
          >
            <KeyValue
              label="彩色 / 深度流"
              value={`${activeSource?.color_stream ?? "—"} / ${activeSource?.depth_stream ?? "—"}`}
              hint="来自所选驱动声明，决定读取哪两路图像。图像流必须与相机参数流的分辨率配套，否则像素和三维点会错位。"
            />
            <KeyValue
              label="相机参数流"
              value={activeSource?.camera_info_stream ?? "—"}
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
              hint="来自相机驱动或当前相机保存配置。原始深度值乘以它得到米；修改会等比例缩放点云、物体距离和尺寸。"
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
                label="标定相机"
                value={
                  calibration?.camera_source_id ??
                  perception?.source_id ??
                  "未选择"
                }
              />
              <KeyValue
                label="机械臂模型"
                value={
                  calibration?.robot_model_revision ??
                  model?.model_revision ??
                  "等待反馈"
                }
              />
              <KeyValue
                label="当前进度"
                value={`${calibration?.active ? "会话进行中" : "未开始会话"} · ${calibration?.observations.length ?? 0} 个样本`}
              />
            </div>

            <div className="calibration-flow">
              <section className="calibration-step">
                <span className="calibration-step-index">1</span>
                <div className="calibration-step-body">
                  <div className="calibration-step-heading">
                    <div>
                      <h3 className="label-with-help">
                        <span>确认标定板参数</span>
                        <HelpDot text="默认使用已经约定的 ChArUco 参数；只有更换标定板时才需要展开修改。开始新会话会清空上一次尚未应用的采样。" />
                      </h3>
                    </div>
                    <StatusBadge
                      tone={calibration?.active ? "cyan" : "neutral"}
                    >
                      {calibration?.active ? "已开始" : "待开始"}
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
                        hint="来自标定板生成器中的 Pattern，必须与打印的标定板一致。"
                      />
                      <KeyValue
                        label="字典"
                        value="DICT_4X4_50"
                        hint="来自标定板生成器中的 ArUco dictionary，决定 Marker 编码，必须与打印文件一致。"
                      />
                    </div>
                    <div className="calibration-parameter-grid">
                      <Field
                        label="横向格数"
                        hint="来自生成器 Squares X。决定横向方格和角点编号，填错会导致图像角点无法对应到实体板。"
                      >
                        <Input
                          type="number"
                          value={board.squaresX}
                          onChange={(event) =>
                            setBoard({
                              ...board,
                              squaresX: event.currentTarget.value,
                            })
                          }
                        />
                      </Field>
                      <Field
                        label="纵向格数"
                        hint="来自生成器 Squares Y。决定纵向方格和角点编号，必须与实体板一致。"
                      >
                        <Input
                          type="number"
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
                        hint="打印后用尺或卡尺测量一个完整方格。OpenCV 用它建立标定板的米制几何，误差会按比例传到相机距离和外参平移。"
                      >
                        <Input
                          type="number"
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
                        hint="打印后测量黑色 ArUco Marker 的外边长。OpenCV 用它识别 Marker 与插值 ChArUco 角点，必须与生成器设置一致。"
                      >
                        <Input
                          type="number"
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
                        hint="尺量有效方格区域的总宽度，用于记录打印缩放是否正确；当前求解器的几何尺度由格数和单格边长确定。"
                      >
                        <Input
                          type="number"
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
                        hint="尺量有效方格区域的总高度，用于记录和检查打印比例；当前不单独参与 OpenCV 几何求解。"
                      >
                        <Input
                          type="number"
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
                        hint="给安装标定板的测试爪或夹具命名，随结果保存以便追溯；名称本身不参与几何计算。"
                      >
                        <Input
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
                      disabled={pending}
                      title={
                        calibration?.active
                          ? "再次点击会清空当前样本，并用页面中的参数重新开始标定"
                          : "使用页面中的标定板参数创建标定会话"
                      }
                      onClick={() => calibrate("start")}
                    >
                      {pendingCalibrationAction === "start"
                        ? "正在开始…"
                        : calibration?.active
                          ? "重新开始并清空样本"
                          : "使用当前参数开始标定"}
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
                        <span>从不同机械臂姿态采样</span>
                        <HelpDot text="保持标定板固定在夹具上并完整出现在彩色图中。移动到不同位置和朝向后逐次采样；每次同时记录 TCP 在底座中的位姿和相机看到的标定板位姿。页面不预设样本数量门限。" />
                      </h3>
                    </div>
                    <StatusBadge
                      tone={
                        calibration?.observations.length ? "cyan" : "neutral"
                      }
                    >
                      {calibration?.observations.length ?? 0} 个样本
                    </StatusBadge>
                  </div>
                  <div className="calibration-step-actions">
                    <Button
                      variant="outline"
                      disabled={pending || !calibration?.active}
                      title="再次点击会在当前会话中继续增加一个姿态样本"
                      onClick={() => calibrate("capture")}
                    >
                      {pendingCalibrationAction === "capture"
                        ? "正在记录…"
                        : calibration?.observations.length
                          ? `继续采样（已有 ${calibration.observations.length} 个）`
                          : "记录当前姿态样本"}
                    </Button>
                  </div>
                </div>
              </section>

              <section className="calibration-step">
                <span className="calibration-step-index">3</span>
                <div className="calibration-step-body">
                  <div className="calibration-step-heading">
                    <div>
                      <h3 className="label-with-help">
                        <span>求解并检查结果</span>
                        <HelpDot text="OpenCV Robot-World/Hand-Eye SHAH 求解器使用全部样本，同时计算相机在底座中和标定板在夹具中的固定变换。" />
                      </h3>
                    </div>
                    <StatusBadge
                      tone={calibration?.solved_result ? "good" : "neutral"}
                    >
                      {calibration?.solved_result ? "已求解" : "待求解"}
                    </StatusBadge>
                  </div>
                  <div className="calibration-step-actions">
                    <Button
                      variant="outline"
                      disabled={pending || !calibration?.active}
                      title="使用当前全部样本求解；再次点击会用最新样本替换当前求解结果"
                      onClick={() => calibrate("solve")}
                    >
                      {pendingCalibrationAction === "solve"
                        ? "正在求解…"
                        : calibration?.solved_result
                          ? "用当前样本重新求解"
                          : "用当前样本求解"}
                    </Button>
                  </div>
                  <KeyValue
                    label="求解器"
                    value={calibration?.solved_result?.solver ?? "—"}
                    hint="由后端实际求解实现报告；当前使用 OpenCV Robot-World/Hand-Eye SHAH。"
                  />
                  <KeyValue
                    label="相机在底座中：位置 / 四元数"
                    value={
                      calibration?.solved_result
                        ? `${numbers(calibration.solved_result.camera_in_base.position_m)} / ${numbers(calibration.solved_result.camera_in_base.orientation_xyzw)}`
                        : "—"
                    }
                    hint="全部样本共同求得的相机外参；应用后用于把相机输出转换为机械臂底座坐标。"
                  />
                  <KeyValue
                    label="标定板在夹具中：位置 / 四元数"
                    value={
                      calibration?.solved_result
                        ? `${numbers(calibration.solved_result.board_in_calibration_tool.position_m)} / ${numbers(calibration.solved_result.board_in_calibration_tool.orientation_xyzw)}`
                        : "—"
                    }
                    hint="同一次求解得到的标定板与 TCP 固定安装偏移，因此标定板不必精确贴在夹爪中心。"
                  />
                  <KeyValue
                    label="各样本平移残差 · m"
                    value={numbers(
                      calibration?.solved_result?.translation_residuals_m,
                    )}
                    hint="每个样本的预测平移与观测平移之差，用于比较样本一致性；页面不设置通过门限。"
                  />
                  <KeyValue
                    label="各样本旋转残差 · rad / °"
                    value={`${numbers(calibration?.solved_result?.rotation_residuals_rad)} / ${degrees(calibration?.solved_result?.rotation_residuals_rad)}`}
                    hint="每个样本的预测旋转与观测旋转之差，同时显示弧度和角度；页面不设置通过门限。"
                  />
                </div>
              </section>

              <section className="calibration-step">
                <span className="calibration-step-index">4</span>
                <div className="calibration-step-body">
                  <div className="calibration-step-heading">
                    <div>
                      <h3 className="label-with-help">
                        <span>应用到当前相机</span>
                        <HelpDot text="应用后，外参保存到当前相机配置，用于把点云、物体和放置区从相机坐标转换为机械臂底座坐标。" />
                      </h3>
                    </div>
                    <StatusBadge
                      tone={perception?.calibrated ? "good" : "neutral"}
                    >
                      {perception?.calibrated ? "已有已应用外参" : "尚未应用"}
                    </StatusBadge>
                  </div>
                  <div className="calibration-step-actions">
                    <Button
                      variant="outline"
                      disabled={pending || !calibration?.solved_result}
                      title="保存当前求解结果并应用到所选相机；再次点击会重新写入当前结果"
                      onClick={() => calibrate("apply")}
                    >
                      {pendingCalibrationAction === "apply"
                        ? "正在保存…"
                        : perception?.calibrated
                          ? "再次应用并保存"
                          : "应用并保存到当前相机"}
                    </Button>
                    <Button
                      variant="outline"
                      disabled={pending || !calibration?.active}
                      title="放弃当前会话和尚未应用的样本，之后需要重新开始标定"
                      onClick={() => calibrate("cancel")}
                    >
                      {pendingCalibrationAction === "cancel"
                        ? "正在取消…"
                        : "放弃本次会话"}
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
            description="这里仅展示相机和驱动直接产生的数据：彩色帧、深度帧和未结构化点云。AI 推理结果不混在这一组。"
          />

          {capturedFrames.map(([title, english, name, frame]) => (
            <Card
              className="span-4 aligned-row-card"
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

          <Card
            className="span-4 aligned-row-card"
            eyebrow="Raw point cloud"
            title="未结构化点云"
          >
            <div className="point-cloud-summary">
              <strong>{perception?.point_count ?? 0}</strong>
              <span>points</span>
            </div>
            <KeyValue
              label="来源"
              value={perception?.source_id ?? "未选择相机"}
            />
            <KeyValue
              label="坐标系"
              value={perception?.depth_frame?.frame_id ?? "—"}
            />
            <KeyValue
              label="场景序号"
              value={String(perception?.last_scene_sequence ?? "—")}
            />
            <div className="card-actions">
              <Button asChild variant="outline">
                <a
                  href="http://192.168.100.10:6080"
                  target="_blank"
                  rel="noreferrer"
                >
                  在 RViz 查看点云
                </a>
              </Button>
            </div>
          </Card>
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
            {perception?.color_frame ? (
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
