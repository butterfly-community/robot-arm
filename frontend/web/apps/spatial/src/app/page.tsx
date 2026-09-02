"use client";

import type {
  AbsolutePoseFrame,
  Json,
  TransformedControlFrame,
} from "@robot/contracts";
import { schemaVersion } from "@robot/contracts";
import { patch, requestId, useGateway } from "@robot/gateway-client";
import {
  Card,
  Field,
  Input,
  JsonView,
  Metric,
  Shell,
  StatusBadge,
} from "@robot/ui";
import {
  PoseViewer,
  relativeMotionPose,
  type PoseVisualization,
} from "@robot/visualization";
import { useState } from "react";

function centimeters(value: number | undefined) {
  return Number.isFinite(value) ? (Number(value) * 100).toFixed(1) : "—";
}

function degrees(value: number | undefined) {
  return Number.isFinite(value)
    ? ((Number(value) * 180) / Math.PI).toFixed(1)
    : "—";
}

const componentSwitches = [
  ["translation", "底座坐标平移"],
  ["front_pitch", "垂直圆弧"],
  ["horizontal_arc", "水平圆弧"],
  ["tool_pitch", "定点垂直旋转"],
  ["tool_yaw", "定点水平旋转"],
  ["tool_roll", "轴向旋转"],
  ["tool_axis_translation", "工具轴向平移"],
  ["tool_helical_motion", "工具轴向螺旋"],
] as const;

const neutralPose: PoseVisualization = {
  position_m: [0, 0, 0],
  orientation_xyzw: [0, 0, 0, 1],
};

export default function Page() {
  const { snapshot, error, setError } = useGateway("spatial");
  const values = snapshot?.values ?? {};
  const config = (values.spatial_config_state ??
    values.config_state ??
    {}) as Record<string, unknown>;
  const switches = (config.switches ?? {}) as Record<string, unknown>;
  const pose = values.spatial_pose as unknown as AbsolutePoseFrame | undefined;
  const motion = values.transformed_control as unknown as
    TransformedControlFrame | undefined;
  const effectiveAxes = (config.base_from_tracking_axes ?? []) as number[][];
  const [axesDraft, setAxesDraft] = useState<number[][]>();
  const axes = axesDraft ?? effectiveAxes;

  async function update(patchValue: Record<string, Json>) {
    setError(undefined);
    try {
      await patch("/api/spatial/config", {
        schema_version: schemaVersion,
        request_id: requestId(),
        patch: patchValue,
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  const translation = motion?.translation_m;
  const active = Boolean(motion?.active);
  const validPose =
    pose && (pose.flags.position_valid || pose.flags.orientation_valid)
      ? pose
      : neutralPose;
  const displayPose = active && motion ? relativeMotionPose(motion) : validPose;

  return (
    <Shell
      section="02 / SPATIAL TRANSFORM"
      title="空间转换"
      description="空间节点统一完成原点、坐标换基和比例转换；页面只显示转换结果与相对运动。"
    >
      <div className="dashboard-grid">
        <div className="span-12 metric-grid">
          <Metric
            label="前后"
            value={centimeters(translation?.[0])}
            unit="cm"
            tone="red"
          />
          <Metric
            label="左右"
            value={centimeters(translation?.[1])}
            unit="cm"
            tone="green"
          />
          <Metric
            label="上下"
            value={centimeters(translation?.[2])}
            unit="cm"
            tone="blue"
          />
          <Metric
            label="垂直圆弧"
            value={degrees(motion?.front_pitch_rad)}
            unit="deg"
            tone="amber"
          />
          <Metric
            label="水平圆弧"
            value={degrees(motion?.horizontal_arc_rad)}
            unit="deg"
            tone="cyan"
          />
          <Metric
            label="控制过程"
            value={
              active ? String(motion?.control_session_id ?? "运行中") : "未启动"
            }
            tone={active ? "green" : undefined}
          />
        </div>

        <Card
          className="span-8 aligned-row-card viewer-fill-card"
          eyebrow="Mapped coordinate view"
          title="人体空间 / 自身姿态"
          action={
            <StatusBadge tone={active ? "good" : "neutral"}>
              {active ? "相对运动输出中" : "等待启动"}
            </StatusBadge>
          }
        >
          <PoseViewer
            pose={displayPose}
            poseCoordinates="robot"
            active={active}
            ariaLabel="空间节点转换后的空间位置和设备自身姿态"
          />
        </Card>

        <Card
          className="span-4 aligned-row-card"
          eyebrow="Transform configuration"
          title="空间配置"
        >
          <Field label="平移倍率">
            <Input
              key={String(config.translation_scale ?? "")}
              type="number"
              step="any"
              defaultValue={
                config.translation_scale == null
                  ? ""
                  : Number(config.translation_scale)
              }
              onBlur={(event) =>
                update({ translation_scale: event.currentTarget.valueAsNumber })
              }
            />
          </Field>
          <Field label="输入坐标 → 前 / 左 / 上坐标映射">
            <div className="matrix" aria-label="空间坐标映射矩阵">
              {axes.flatMap((row, rowIndex) =>
                row.map((value, columnIndex) => (
                  <Input
                    key={`${rowIndex}-${columnIndex}`}
                    type="number"
                    step="any"
                    value={value}
                    aria-label={`映射 ${rowIndex + 1},${columnIndex + 1}`}
                    onChange={(event) => {
                      const next = axes.map((item) => [...item]);
                      next[rowIndex][columnIndex] =
                        event.currentTarget.valueAsNumber;
                      setAxesDraft(next);
                    }}
                    onBlur={() =>
                      void update({ base_from_tracking_axes: axes }).then(() =>
                        setAxesDraft(undefined),
                      )
                    }
                  />
                )),
              )}
            </div>
          </Field>
          <Field label="采集分量">
            <div className="switch-stack">
              {componentSwitches.map(([key, label]) => (
                <label key={key} className="switch-row">
                  <span>{label}</span>
                  <input
                    type="checkbox"
                    checked={Boolean(switches[key])}
                    onChange={(event) =>
                      update({
                        switches: Object.fromEntries(
                          componentSwitches.map(([switchKey]) => [
                            switchKey,
                            switchKey === key
                              ? event.currentTarget.checked
                              : Boolean(switches[switchKey]),
                          ]),
                        ),
                      })
                    }
                  />
                </label>
              ))}
            </div>
          </Field>
          <Field label="设备垂直姿态语义">
            <select
              aria-label="设备垂直姿态语义"
              value={String(
                (config.orientation_mapping as Record<string, unknown>)
                  ?.vertical ?? "front_pitch",
              )}
              onChange={(event) =>
                update({
                  orientation_mapping: {
                    vertical: event.currentTarget.value,
                    horizontal: String(
                      (config.orientation_mapping as Record<string, unknown>)
                        ?.horizontal ?? "horizontal_arc",
                    ),
                  },
                })
              }
            >
              <option value="front_pitch">垂直圆弧</option>
              <option value="tool_pitch">定点垂直旋转</option>
            </select>
          </Field>
          <Field label="设备水平姿态语义">
            <select
              aria-label="设备水平姿态语义"
              value={String(
                (config.orientation_mapping as Record<string, unknown>)
                  ?.horizontal ?? "horizontal_arc",
              )}
              onChange={(event) =>
                update({
                  orientation_mapping: {
                    vertical: String(
                      (config.orientation_mapping as Record<string, unknown>)
                        ?.vertical ?? "front_pitch",
                    ),
                    horizontal: event.currentTarget.value,
                  },
                })
              }
            >
              <option value="horizontal_arc">水平圆弧</option>
              <option value="tool_yaw">定点水平旋转</option>
            </select>
          </Field>
          <Field
            label="无绝对位置时的平移速度"
            hint="默认 1 cm/s；配置单位为 m/s"
          >
            <Input
              key={String(config.action_translation_m_per_s ?? "")}
              type="number"
              step="any"
              defaultValue={String(config.action_translation_m_per_s ?? "")}
              onBlur={(event) =>
                update({
                  action_translation_m_per_s:
                    event.currentTarget.value === ""
                      ? null
                      : event.currentTarget.valueAsNumber,
                })
              }
            />
          </Field>
          <Field label="无绝对姿态时的圆弧角速度" hint="默认 0.10 rad/s">
            <Input
              key={String(config.action_arc_rad_per_s ?? "")}
              type="number"
              step="any"
              defaultValue={String(config.action_arc_rad_per_s ?? "")}
              onBlur={(event) =>
                update({
                  action_arc_rad_per_s:
                    event.currentTarget.value === ""
                      ? null
                      : event.currentTarget.valueAsNumber,
                })
              }
            />
          </Field>
        </Card>

        <Card
          className="span-12"
          eyebrow="Troubleshooting"
          title="排障数据"
          defaultOpen={false}
        >
          <div className="diagnostic-grid">
            <JsonView title="空间节点转换位姿" value={pose} />
            <JsonView title="相对运动消息" value={motion} />
            <JsonView
              title="空间配置 / 请求结果"
              value={{ config, request: values.spatial_request_result }}
            />
          </div>
        </Card>
      </div>
      {error && <p className="error">{error}</p>}
    </Shell>
  );
}
