"use client";

import {
  schemaVersion,
  type ArmState,
  type CalibrationSessionState,
  type ManipulationTaskState,
  type MotionState,
  type PerceptionSourceKind,
  type PerceptionState,
  type RobotModelInfo,
  type WorldScene,
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
  Input,
  JsonView,
  KeyValue,
  Metric,
  RangeControls,
  Shell,
  StatusBadge,
} from "@robot/ui";
import { useState } from "react";

function degrees(value: number | undefined) {
  return Number.isFinite(value)
    ? ((Number(value) * 180) / Math.PI).toFixed(1)
    : "—";
}

export default function Page() {
  const { snapshot, error, setError } = useGateway("motion");
  const values = snapshot?.values ?? {};
  const model = values.robot_model_info as unknown as
    RobotModelInfo | undefined;
  const arm = values.arm_state as unknown as ArmState | undefined;
  const motion = values.motion_state as unknown as
    (MotionState & Record<string, unknown>) | undefined;
  const perception = values.perception_state as unknown as
    PerceptionState | undefined;
  const scene = perception?.enabled
    ? (values.world_scene as unknown as WorldScene | undefined)
    : undefined;
  const calibration = values.calibration_state as unknown as
    CalibrationSessionState | undefined;
  const manipulation = values.manipulation_state as unknown as
    ManipulationTaskState | undefined;
  const latestMotion = (motion?.latest_motion ??
    values.motion_status ??
    {}) as Record<string, unknown>;
  const diagnostics = (motion?.diagnostics ?? []) as Array<
    Record<string, unknown>
  >;
  const armKey = JSON.stringify([
    model?.model_revision,
    arm?.joints_rad,
    arm?.actuators_rad,
  ]);
  const [previousArmKey, setPreviousArmKey] = useState("");
  const [positions, setPositions] = useState<Record<string, number>>({});
  const [actuators, setActuators] = useState<Record<string, number>>({});
  const [editing, setEditing] = useState<string>();
  const [options, setOptions] = useState<Record<string, number | undefined>>(
    {},
  );
  const [perceptionPending, setPerceptionPending] = useState(false);
  const [calibrationPending, setCalibrationPending] = useState(false);
  const [pickPlacePending, setPickPlacePending] = useState(false);
  const [objectId, setObjectId] = useState("");
  const [placementRegionId, setPlacementRegionId] = useState("");
  const [calibrationForm, setCalibrationForm] = useState({
    squaresX: "5",
    squaresY: "5",
    squareMm: "15",
    markerMm: "11",
    measuredWidthMm: "75",
    measuredHeightMm: "75",
    toolId: "charuco-test-gripper",
  });

  if (arm && !editing && previousArmKey !== armKey) {
    setPreviousArmKey(armKey);
    setPositions(
      Object.fromEntries(
        (model?.joints ?? []).map((joint, index) => [
          joint.key,
          arm.joints_rad[index] ?? 0,
        ]),
      ),
    );
    setActuators(
      Object.fromEntries(
        (model?.tool_actuators ?? []).map((actuator, index) => [
          actuator.key,
          arm.actuators_rad[index] ?? 0,
        ]),
      ),
    );
  }

  async function mode(value: "relative" | "manual") {
    setError(undefined);
    try {
      await post("/api/motion/mode", {
        schema_version: schemaVersion,
        request_id: requestId(),
        mode: value,
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  async function prepareRelative() {
    setError(undefined);
    try {
      await prepareRelativeControl();
    } catch (reason) {
      setError(String(reason));
    }
  }

  async function cancel() {
    setError(undefined);
    try {
      await post("/api/motion/cancel", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action: "cancel",
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  async function setPerception(
    enabled: boolean,
    sourceKind?: PerceptionSourceKind,
  ) {
    setError(undefined);
    setPerceptionPending(true);
    try {
      await post("/api/motion/perception", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action: enabled ? "apply" : "disconnect",
        source_kind: sourceKind ?? null,
        source_id:
          sourceKind === "generated_test_scene"
            ? "generated:pick-place-scene"
            : sourceKind === "camera"
              ? "ros:depth-camera"
              : null,
        classes: null,
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setPerceptionPending(false);
    }
  }

  async function calibrate(
    action: "start" | "capture" | "solve" | "apply" | "cancel",
  ) {
    setError(undefined);
    setCalibrationPending(true);
    try {
      await post("/api/motion/perception/calibration", {
        schema_version: schemaVersion,
        request_id: requestId(),
        action,
        board:
          action === "start"
            ? {
                pattern: "charuco",
                dictionary: "DICT_4X4_50",
                squares_x: Number(calibrationForm.squaresX),
                squares_y: Number(calibrationForm.squaresY),
                square_size_m: Number(calibrationForm.squareMm) / 1000,
                marker_size_m: Number(calibrationForm.markerMm) / 1000,
                measured_width_m:
                  Number(calibrationForm.measuredWidthMm) / 1000,
                measured_height_m:
                  Number(calibrationForm.measuredHeightMm) / 1000,
              }
            : null,
        camera_source_id: perception?.source_id ?? null,
        robot_model_revision: model?.model_revision ?? null,
        calibration_tool_id: action === "start" ? calibrationForm.toolId : null,
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setCalibrationPending(false);
    }
  }

  async function pickPlace() {
    const selectedObject =
      objectId || scene?.objects.find((object) => object.graspable)?.object_id;
    const selectedRegion =
      placementRegionId || scene?.placement_regions[0]?.region_id;
    if (!selectedObject || !selectedRegion) return;
    setError(undefined);
    setPickPlacePending(true);
    try {
      await post("/api/motion/pick-place", {
        schema_version: schemaVersion,
        request_id: requestId(),
        object_id: selectedObject,
        placement_region_id: selectedRegion,
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setPickPlacePending(false);
    }
  }

  async function move(jointTarget = positions, actuatorTarget = actuators) {
    if (!model) return;
    setError(undefined);
    try {
      await post("/api/motion/request", {
        schema_version: schemaVersion,
        request_id: requestId(),
        model_revision: model.model_revision,
        joints: Object.entries(jointTarget).map(
          ([joint_key, position_rad]) => ({
            joint_key,
            position_rad,
          }),
        ),
        actuators: Object.entries(actuatorTarget).map(
          ([actuator_key, position_rad]) => ({
            actuator_key,
            position_rad,
          }),
        ),
        options: Object.fromEntries(
          Object.entries(options).filter((entry): entry is [string, number] =>
            Number.isFinite(entry[1]),
          ),
        ),
        action: "apply",
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  const tool = motion?.current_tool_pose;
  const target = motion?.target_tool_pose;
  const state = String(latestMotion.state ?? "idle");
  const motionBusy = state === "planning" || state === "executing";

  return (
    <Shell
      section="03 / MOVEIT MOTION"
      title="机械臂运动"
      description="关节反馈、手动目标、末端位置和 MoveIt 状态直接形成操作台；完整 DTO 只在底部排障区查看。"
    >
      <div className="dashboard-grid">
        <div className="span-12 metric-grid">
          <Metric
            label="Control mode"
            value={String(motion?.control_mode ?? "—").toUpperCase()}
            tone="cyan"
          />
          <Metric
            label="Feedback"
            value={String(arm?.feedback_source ?? "WAIT").toUpperCase()}
            tone={arm?.feedback_source === "hardware" ? "green" : undefined}
          />
          <Metric
            label="MoveIt state"
            value={state.toUpperCase()}
            tone={
              state === "failed"
                ? "red"
                : state === "succeeded"
                  ? "green"
                  : "amber"
            }
          />
          <Metric
            label="TCP X"
            value={tool?.position_m?.[0]?.toFixed(3) ?? "—"}
            unit="m"
            tone="red"
          />
          <Metric
            label="TCP Y"
            value={tool?.position_m?.[1]?.toFixed(3) ?? "—"}
            unit="m"
            tone="green"
          />
          <Metric
            label="TCP Z"
            value={tool?.position_m?.[2]?.toFixed(3) ?? "—"}
            unit="m"
            tone="blue"
          />
        </div>

        <Card
          className="span-12"
          eyebrow="Perception"
          title="场景感知"
          action={
            <StatusBadge tone={perception?.enabled ? "good" : "neutral"}>
              {perception?.enabled ? "已启用" : "未启用"}
            </StatusBadge>
          }
        >
          <div className="card-actions card-actions-leading">
            <Button
              variant="outline"
              disabled={perceptionPending}
              onClick={() => setPerception(true, "generated_test_scene")}
            >
              测试 RGB-D 场景
            </Button>
            <Button
              variant="outline"
              disabled={perceptionPending}
              onClick={() => setPerception(true, "camera")}
            >
              启用深度相机
            </Button>
            <Button
              variant="outline"
              disabled={perceptionPending || !perception?.enabled}
              onClick={() => setPerception(false)}
            >
              停用感知
            </Button>
            <Button asChild variant="outline">
              <a
                href="http://192.168.100.10:6080"
                target="_blank"
                rel="noreferrer"
              >
                打开 RViz 可视化
              </a>
            </Button>
          </div>
          <div className="pose-comparison">
            <div>
              <KeyValue
                label="输入来源"
                value={String(perception?.source_id ?? "未选择")}
              />
              <KeyValue
                label="识别模型"
                value={String(perception?.model ?? "—")}
              />
              <KeyValue
                label="外参"
                value={perception?.calibrated ? "已应用" : "未标定"}
              />
            </div>
            <div>
              <KeyValue
                label="场景坐标系"
                value={String(scene?.frame_id ?? "—")}
              />
              <KeyValue
                label="物体 / 放置区 / 障碍"
                value={`${scene?.objects.length ?? 0} / ${scene?.placement_regions.length ?? 0} / ${scene?.obstacles.length ?? 0}`}
              />
              <KeyValue
                label="错误"
                value={String(perception?.original_error ?? "无")}
              />
            </div>
          </div>
          {scene?.objects.map((object) => (
            <KeyValue
              key={object.object_id}
              label={`${object.label}${object.graspable ? " · 可抓取" : ""}`}
              value={`${object.pose.position_m.map((value) => value.toFixed(3)).join(" / ")} m`}
            />
          ))}
        </Card>

        <Card
          className="span-12"
          eyebrow="Pick and place"
          title="抓取与放置"
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
          <div className="diagnostic-grid">
            <Field label="抓取目标">
              <select
                value={
                  objectId ||
                  scene?.objects.find((object) => object.graspable)
                    ?.object_id ||
                  ""
                }
                onChange={(event) => setObjectId(event.currentTarget.value)}
              >
                {!scene?.objects.some((object) => object.graspable) && (
                  <option value="">没有可抓取目标</option>
                )}
                {scene?.objects
                  .filter((object) => object.graspable)
                  .map((object) => (
                    <option key={object.object_id} value={object.object_id}>
                      {object.label} · {object.object_id}
                    </option>
                  ))}
              </select>
            </Field>
            <Field label="放置区域">
              <select
                value={
                  placementRegionId ||
                  scene?.placement_regions[0]?.region_id ||
                  ""
                }
                onChange={(event) =>
                  setPlacementRegionId(event.currentTarget.value)
                }
              >
                {!scene?.placement_regions.length && (
                  <option value="">没有放置区域</option>
                )}
                {scene?.placement_regions.map((region) => (
                  <option key={region.region_id} value={region.region_id}>
                    {region.label} · {region.region_id}
                  </option>
                ))}
              </select>
            </Field>
          </div>
          <div className="card-actions card-actions-leading">
            <Button
              variant="outline"
              disabled={
                pickPlacePending ||
                manipulation?.state === "executing" ||
                !scene?.objects.some((object) => object.graspable) ||
                !scene?.placement_regions.length
              }
              onClick={pickPlace}
            >
              执行抓放
            </Button>
          </div>
          <div className="pose-comparison">
            <div>
              <KeyValue label="当前步骤" value={manipulation?.step ?? "—"} />
            </div>
            <div>
              <KeyValue
                label="错误"
                value={manipulation?.original_error ?? "无"}
              />
            </div>
          </div>
        </Card>

        <Card
          className="span-12"
          eyebrow="Camera calibration"
          title="相机外参标定"
          defaultOpen={false}
          action={
            <StatusBadge tone={perception?.calibrated ? "good" : "neutral"}>
              {perception?.calibrated ? "已有外参" : "未标定"}
            </StatusBadge>
          }
        >
          <div className="diagnostic-grid">
            <Field label="横向 / 纵向格数">
              <div className="card-actions">
                <Input
                  type="number"
                  value={calibrationForm.squaresX}
                  onChange={(event) =>
                    setCalibrationForm({
                      ...calibrationForm,
                      squaresX: event.currentTarget.value,
                    })
                  }
                />
                <Input
                  type="number"
                  value={calibrationForm.squaresY}
                  onChange={(event) =>
                    setCalibrationForm({
                      ...calibrationForm,
                      squaresY: event.currentTarget.value,
                    })
                  }
                />
              </div>
            </Field>
            <Field label="方格 / Marker 实测毫米">
              <div className="card-actions">
                <Input
                  type="number"
                  value={calibrationForm.squareMm}
                  onChange={(event) =>
                    setCalibrationForm({
                      ...calibrationForm,
                      squareMm: event.currentTarget.value,
                    })
                  }
                />
                <Input
                  type="number"
                  value={calibrationForm.markerMm}
                  onChange={(event) =>
                    setCalibrationForm({
                      ...calibrationForm,
                      markerMm: event.currentTarget.value,
                    })
                  }
                />
              </div>
            </Field>
            <Field label="板宽 / 板高实测毫米">
              <div className="card-actions">
                <Input
                  type="number"
                  value={calibrationForm.measuredWidthMm}
                  onChange={(event) =>
                    setCalibrationForm({
                      ...calibrationForm,
                      measuredWidthMm: event.currentTarget.value,
                    })
                  }
                />
                <Input
                  type="number"
                  value={calibrationForm.measuredHeightMm}
                  onChange={(event) =>
                    setCalibrationForm({
                      ...calibrationForm,
                      measuredHeightMm: event.currentTarget.value,
                    })
                  }
                />
              </div>
            </Field>
            <Field label="测试爪标识">
              <Input
                value={calibrationForm.toolId}
                onChange={(event) =>
                  setCalibrationForm({
                    ...calibrationForm,
                    toolId: event.currentTarget.value,
                  })
                }
              />
            </Field>
          </div>
          <div className="card-actions card-actions-leading">
            <Button
              variant="outline"
              disabled={calibrationPending}
              onClick={() => calibrate("start")}
            >
              开始标定
            </Button>
            <Button
              variant="outline"
              disabled={calibrationPending || !calibration?.active}
              onClick={() => calibrate("capture")}
            >
              采集当前样本
            </Button>
            <Button
              variant="outline"
              disabled={calibrationPending || !calibration?.active}
              onClick={() => calibrate("solve")}
            >
              求解
            </Button>
            <Button
              variant="outline"
              disabled={calibrationPending || !calibration?.solved_result}
              onClick={() => calibrate("apply")}
            >
              应用结果
            </Button>
            <Button
              variant="outline"
              disabled={calibrationPending || !calibration?.active}
              onClick={() => calibrate("cancel")}
            >
              结束会话
            </Button>
          </div>
          <KeyValue
            label="已采样"
            value={String(calibration?.observations.length ?? 0)}
          />
          <KeyValue
            label="求解器"
            value={String(calibration?.solved_result?.solver ?? "—")}
          />
          <KeyValue
            label="错误"
            value={String(calibration?.original_error ?? "无")}
          />
        </Card>

        <Card
          className="span-8 aligned-row-card"
          eyebrow="Joint workspace"
          title="关节与夹爪状态 / 手动目标"
          action={
            <StatusBadge tone={arm ? "good" : "warning"}>
              {model?.display_name ?? "等待模型"}
            </StatusBadge>
          }
        >
          <div className="joint-strip" aria-label="关节角度图形">
            {(model?.joints ?? []).map((joint, index) => {
              const value = arm?.joints_rad[index] ?? 0;
              const fraction =
                (value - joint.minimum) / (joint.maximum - joint.minimum);
              return (
                <div className="joint-gauge" key={joint.key}>
                  <span>{joint.label}</span>
                  <div>
                    <i style={{ left: `${fraction * 100}%` }} />
                  </div>
                  <strong>{degrees(value)}°</strong>
                </div>
              );
            })}
          </div>
          <RangeControls
            items={model?.joints ?? []}
            values={positions}
            onBegin={setEditing}
            onChange={(key, value) =>
              setPositions({ ...positions, [key]: value })
            }
            onCommit={(key, value) => {
              const next = { ...positions, [key]: value };
              setEditing(undefined);
              void move(next);
            }}
          />
          <RangeControls
            items={model?.tool_actuators ?? []}
            values={actuators}
            onBegin={setEditing}
            onChange={(key, value) =>
              setActuators({ ...actuators, [key]: value })
            }
            onCommit={(key, value) => {
              setEditing(undefined);
              setError(undefined);
              void post("/api/motion/actuator", {
                schema_version: schemaVersion,
                request_id: requestId(),
                model_revision: model!.model_revision,
                actuator_key: key,
                position_rad: value,
                action: "apply",
              }).catch((reason) => setError(String(reason)));
            }}
          />
        </Card>

        <Card
          className="span-4 aligned-row-card"
          eyebrow="Motion command"
          title="模式 / 规划"
        >
          <div className="card-actions card-actions-leading">
            <Button
              variant={
                motion?.control_mode === "relative" ? "default" : "outline"
              }
              onClick={() => mode("relative")}
            >
              相对控制
            </Button>
            <Button
              variant={
                motion?.control_mode === "manual" ? "default" : "outline"
              }
              onClick={() => mode("manual")}
            >
              手动控制
            </Button>
            <Button variant="danger" onClick={cancel}>
              取消普通运动
            </Button>
          </div>
          <KeyValue
            label="当前"
            value={String(motion?.control_mode ?? "未知")}
          />
          <KeyValue
            label="请求"
            value={String(latestMotion.request_id ?? "—")}
          />
          <KeyValue
            label="结果"
            value={String(latestMotion.result_message ?? "—")}
          />
          <KeyValue
            label="轨迹点"
            value={String(latestMotion.trajectory_points ?? "—")}
          />
          {(model?.motion_options ?? []).map((option) => (
            <Field
              key={option.key}
              label={option.label}
              englishLabel={`${option.key} · ${option.unit}`}
            >
              <Input
                type="number"
                step="any"
                min={option.minimum ?? undefined}
                max={option.maximum ?? undefined}
                value={options[option.key] ?? ""}
                onChange={(event) =>
                  setOptions({
                    ...options,
                    [option.key]:
                      event.currentTarget.value === ""
                        ? undefined
                        : event.currentTarget.valueAsNumber,
                  })
                }
              />
            </Field>
          ))}
          <div className="target-list">
            <Button disabled={motionBusy} onClick={prepareRelative}>
              准备相对控制
            </Button>
            {(model?.named_targets ?? []).map((item) => (
              <Button
                key={item.key}
                disabled={motionBusy}
                onClick={() => {
                  setPositions(item.joint_positions_rad);
                  setActuators(item.actuator_positions_rad);
                  void move(
                    item.joint_positions_rad,
                    item.actuator_positions_rad,
                  );
                }}
              >
                {item.label}
              </Button>
            ))}
          </div>
        </Card>

        <Card className="span-6" eyebrow="Tool pose" title="末端当前 / 目标">
          <div className="pose-comparison">
            <div>
              <StatusBadge tone="good">当前 TCP</StatusBadge>
              <KeyValue
                label="位置"
                value={
                  tool
                    ? tool.position_m
                        .map((value) => value.toFixed(4))
                        .join(" / ")
                    : "—"
                }
              />
              <KeyValue
                label="姿态 XYZW"
                value={
                  tool
                    ? tool.orientation_xyzw
                        .map((value) => value.toFixed(3))
                        .join(" / ")
                    : "—"
                }
              />
            </div>
            <div>
              <StatusBadge tone={target ? "cyan" : "neutral"}>
                目标 TCP
              </StatusBadge>
              <KeyValue
                label="位置"
                value={
                  target
                    ? target.position_m
                        .map((value) => value.toFixed(4))
                        .join(" / ")
                    : "—"
                }
              />
              <KeyValue
                label="姿态 XYZW"
                value={
                  target
                    ? target.orientation_xyzw
                        .map((value) => value.toFixed(3))
                        .join(" / ")
                    : "—"
                }
              />
            </div>
          </div>
        </Card>

        <Card
          className="span-6"
          eyebrow="Runtime diagnostics"
          title="MoveIt / Servo"
        >
          <div className="parameter-list">
            {diagnostics.map((item) => (
              <div className="parameter-item" key={String(item.key)}>
                <KeyValue
                  label={String(item.key)}
                  value={String(item.value ?? "—")}
                />
              </div>
            ))}
          </div>
          {!diagnostics.length && (
            <p className="status">等待 Motion 节点诊断</p>
          )}
        </Card>

        <Card
          className="span-12"
          eyebrow="Troubleshooting"
          title="排障数据"
          defaultOpen={false}
        >
          <div className="diagnostic-grid">
            <JsonView title="MotionState 原始数据" value={motion} />
            <JsonView title="请求结果" value={values.motion_request_result} />
            <JsonView title="感知状态" value={perception} />
            <JsonView title="结构化场景" value={scene} />
            <JsonView title="标定会话" value={calibration} />
            <JsonView title="抓放任务" value={manipulation} />
            <JsonView title="型号元数据" value={model} />
          </div>
        </Card>
      </div>
      {error && <p className="error">{error}</p>}
    </Shell>
  );
}
