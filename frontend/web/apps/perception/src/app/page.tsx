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
  Input,
  JsonView,
  KeyValue,
  Metric,
  Shell,
  StatusBadge,
} from "@robot/ui";
import { useMemo, useState } from "react";

const initialBoard = {
  squaresX: "5",
  squaresY: "5",
  squareMm: "15",
  markerMm: "11",
  measuredWidthMm: "75",
  measuredHeightMm: "75",
  toolId: "charuco-test-tool",
};

function numbers(values: number[] | undefined, digits = 3) {
  return values?.map((value) => value.toFixed(digits)).join(" / ") ?? "—";
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
  const [sourceId, setSourceId] = useState("");
  const [objectId, setObjectId] = useState("");
  const [regionId, setRegionId] = useState("");
  const [board, setBoard] = useState(initialBoard);
  const [classes, setClasses] = useState("");
  const [placementLabels, setPlacementLabels] = useState("");
  const [depthScale, setDepthScale] = useState("");
  const [pending, setPending] = useState(false);

  const selectedSourceId = sourceId || perception?.source_id || "";
  const selectedClasses = classes || perception?.classes.join(", ") || "";
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
  ) {
    const appliesConfiguration = action === "apply";
    await send("/api/perception/request", {
      schema_version: schemaVersion,
      request_id: requestId(),
      action,
      source_id: action === "refresh" ? null : selectedSourceId || null,
      depth_scale_m:
        appliesConfiguration && selectedDepthScale
          ? Number(selectedDepthScale)
          : null,
      classes: appliesConfiguration
        ? selectedClasses
            .split(",")
            .map((value) => value.trim())
            .filter(Boolean)
        : null,
      placement_labels: appliesConfiguration
        ? selectedPlacementLabels
            .split(",")
            .map((value) => value.trim())
            .filter(Boolean)
        : null,
    });
    if (action === "reset") setDepthScale("");
    if (action === "disconnect") setSourceId("");
  }

  async function pickPlace() {
    if (!selectedObject || !selectedRegion) return;
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
  }

  function calibrate(
    action: "start" | "capture" | "solve" | "apply" | "cancel",
  ) {
    return send("/api/perception/calibration", {
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
  }

  const previews = useMemo(
    () =>
      [
        ["彩色图", "Color", "color.png", perception?.color_frame],
        ["识别与分割", "Detections", "overlay.png", perception?.color_frame],
        ["深度图", "Depth", "depth.png", perception?.depth_frame],
      ] as const,
    [perception?.color_frame, perception?.depth_frame],
  );

  return (
    <Shell
      section="03 / PERCEPTION"
      title="场景感知"
      description="深度相机、二维识别、深度、三维场景、标定与感知抓放集中在同一页面；点云复用 RViz。"
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
            value={String(
              scene?.objects.reduce(
                (sum, item) => sum + item.grasp_candidates.length,
                0,
              ) ?? 0,
            )}
            tone="green"
          />
          <Metric
            label="未结构化点云"
            value={String(perception?.point_count ?? 0)}
            unit="points"
          />
        </div>

        <Card
          className="span-12"
          eyebrow="Source"
          title="深度相机与数据来源"
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
          <div className="diagnostic-grid">
            <Field label="深度相机">
              <select
                value={selectedSourceId}
                onChange={(event) => {
                  setSourceId(event.currentTarget.value);
                  setDepthScale("");
                }}
              >
                <option value="">选择深度相机</option>
                {perception?.available_sources.map((source) => (
                  <option key={source.source_id} value={source.source_id}>
                    {source.display_name} · {source.source_id}
                  </option>
                ))}
              </select>
            </Field>
            <KeyValue label="计算模型" value={perception?.model ?? "—"} />
            <KeyValue label="驱动" value={selectedSource?.driver_id ?? "—"} />
            <KeyValue
              label="参数与标定"
              value={selectedSource?.calibrated ? "已配置" : "未配置"}
            />
            <Field label="深度比例 · m / unit">
              <Input
                type="number"
                value={selectedDepthScale}
                onChange={(event) => setDepthScale(event.currentTarget.value)}
              />
            </Field>
            <Field label="识别类别">
              <Input
                value={selectedClasses}
                onChange={(event) => setClasses(event.currentTarget.value)}
              />
            </Field>
            <Field label="放置区域类别">
              <Input
                value={selectedPlacementLabels}
                onChange={(event) =>
                  setPlacementLabels(event.currentTarget.value)
                }
              />
            </Field>
          </div>
          <div className="card-actions card-actions-leading">
            <Button
              variant="outline"
              disabled={pending}
              onClick={() => perceptionRequest("refresh")}
            >
              刷新相机
            </Button>
            <Button
              variant="outline"
              disabled={pending || !selectedSourceId}
              onClick={() => perceptionRequest("apply")}
            >
              保存配置并启用
            </Button>
            <Button
              variant="outline"
              disabled={pending || !selectedSourceId}
              onClick={() => perceptionRequest("reset")}
            >
              重置当前相机配置
            </Button>
            <Button
              variant="outline"
              disabled={pending || !perception?.enabled}
              onClick={() => perceptionRequest("disconnect")}
            >
              停用感知
            </Button>
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
          <KeyValue
            label="当前来源"
            value={perception?.source_id ?? "未选择"}
          />
          <KeyValue
            label="配置管理"
            value="参数与标定由感知服务按相机保存；重置只作用于当前相机"
          />
          <KeyValue
            label="错误"
            value={perception?.original_error ?? error ?? "无"}
          />
        </Card>

        {previews.map(([title, english, name, frame]) => (
          <Card className="span-4" key={name} eyebrow={english} title={title}>
            {frame ? (
              <Image
                className="perception-preview"
                src={asset(name)}
                alt={title}
                width={frame.width}
                height={frame.height}
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
          className="span-12"
          eyebrow="Instances"
          title="识别、分割与三维实例"
        >
          <div className="table-scroll">
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
        </Card>

        <Card
          className="span-7 aligned-row-card"
          eyebrow="Scene"
          title="结构化三维场景"
        >
          <KeyValue label="坐标系" value={scene?.frame_id ?? "—"} />
          <KeyValue
            label="物体 / 放置区 / 显式障碍"
            value={`${scene?.objects.length ?? 0} / ${scene?.placement_regions.length ?? 0} / ${scene?.obstacles.length ?? 0}`}
          />
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
        </Card>

        <Card
          className="span-5 aligned-row-card"
          eyebrow="Camera parameters"
          title="相机参数"
        >
          <KeyValue label="彩色流" value={activeSource?.color_stream ?? "—"} />
          <KeyValue label="深度流" value={activeSource?.depth_stream ?? "—"} />
          <KeyValue
            label="相机参数流"
            value={activeSource?.camera_info_stream ?? "—"}
          />
          <KeyValue
            label="内参 K"
            value={numbers(perception?.camera_calibration?.camera_matrix)}
          />
          <KeyValue
            label="畸变模型 / D"
            value={
              perception?.camera_calibration
                ? `${perception.camera_calibration.distortion_model} · ${numbers(perception.camera_calibration.distortion)}`
                : "—"
            }
          />
          <KeyValue
            label="投影矩阵 P"
            value={numbers(perception?.camera_calibration?.projection_matrix)}
          />
          <KeyValue
            label="深度比例"
            value={`${perception?.depth_scale_m ?? "—"} m / unit`}
          />
          <KeyValue
            label="外参平移 / 四元数"
            value={
              perception?.camera_calibration
                ? `${numbers(perception.camera_calibration.translation_m)} / ${numbers(perception.camera_calibration.orientation_xyzw)}`
                : "—"
            }
          />
        </Card>

        <Card
          className="span-6 aligned-row-card"
          eyebrow="Perception control"
          title="感知抓放"
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
              {manipulation?.state ?? "idle"}
            </StatusBadge>
          }
        >
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
          <div className="card-actions card-actions-leading">
            <Button
              variant="outline"
              disabled={pending || !selectedObject || !selectedRegion}
              onClick={pickPlace}
            >
              执行抓放
            </Button>
          </div>
          <KeyValue
            label="阶段 / 方案 / 代价"
            value={`${manipulation?.stage ?? "—"} / ${manipulation?.solution_count ?? "—"} / ${manipulation?.selected_cost?.toFixed(3) ?? "—"}`}
          />
          <KeyValue
            label="抓取 / 放置位置"
            value={`${numbers(manipulation?.pick_position_m ?? undefined)} / ${numbers(manipulation?.place_position_m ?? undefined)}`}
          />
          <KeyValue label="错误" value={manipulation?.original_error ?? "无"} />
        </Card>

        <Card
          className="span-6 aligned-row-card"
          eyebrow="Calibration"
          title="相机外参标定"
          defaultOpen={false}
          action={
            <StatusBadge tone={perception?.calibrated ? "good" : "neutral"}>
              {perception?.calibrated ? "已有外参" : "未标定"}
            </StatusBadge>
          }
        >
          <div className="diagnostic-grid">
            {(
              [
                ["横向格数", "squaresX"],
                ["纵向格数", "squaresY"],
                ["方格 mm", "squareMm"],
                ["Marker mm", "markerMm"],
                ["板宽 mm", "measuredWidthMm"],
                ["板高 mm", "measuredHeightMm"],
              ] as const
            ).map(([label, key]) => (
              <Field key={key} label={label}>
                <Input
                  type="number"
                  value={board[key]}
                  onChange={(event) =>
                    setBoard({ ...board, [key]: event.currentTarget.value })
                  }
                />
              </Field>
            ))}
            <Field label="测试爪标识">
              <Input
                value={board.toolId}
                onChange={(event) =>
                  setBoard({ ...board, toolId: event.currentTarget.value })
                }
              />
            </Field>
          </div>
          <div className="card-actions card-actions-leading">
            <Button
              variant="outline"
              disabled={pending}
              onClick={() => calibrate("start")}
            >
              开始
            </Button>
            <Button
              variant="outline"
              disabled={pending || !calibration?.active}
              onClick={() => calibrate("capture")}
            >
              采样
            </Button>
            <Button
              variant="outline"
              disabled={pending || !calibration?.active}
              onClick={() => calibrate("solve")}
            >
              求解
            </Button>
            <Button
              variant="outline"
              disabled={pending || !calibration?.solved_result}
              onClick={() => calibrate("apply")}
            >
              应用并保存
            </Button>
            <Button
              variant="outline"
              disabled={pending || !calibration?.active}
              onClick={() => calibrate("cancel")}
            >
              取消
            </Button>
          </div>
          <KeyValue
            label="样本 / 求解器"
            value={`${calibration?.observations.length ?? 0} / ${calibration?.solved_result?.solver ?? "—"}`}
          />
          <KeyValue
            label="平移残差 m"
            value={numbers(calibration?.solved_result?.translation_residuals_m)}
          />
          <KeyValue
            label="旋转残差 rad"
            value={numbers(calibration?.solved_result?.rotation_residuals_rad)}
          />
          <KeyValue label="错误" value={calibration?.original_error ?? "无"} />
        </Card>

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
