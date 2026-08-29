"use client";

import type {
  AbsolutePoseFrame,
  Json,
  RelativeToolMotion,
} from "@robot/contracts";
import { patch, post, requestId, useGateway } from "@robot/gateway-client";
import {
  Button,
  Card,
  Field,
  Input,
  JsonView,
  Metric,
  Shell,
  StatusBadge,
} from "@robot/ui";
import { PoseViewer } from "@robot/visualization";
import { useState } from "react";

function centimeters(value: number | undefined) {
  return Number.isFinite(value) ? (Number(value) * 100).toFixed(1) : "—";
}

function degrees(value: number | undefined) {
  return Number.isFinite(value)
    ? ((Number(value) * 180) / Math.PI).toFixed(1)
    : "—";
}

export default function Page() {
  const { snapshot, error, setError } = useGateway("spatial");
  const values = snapshot?.values ?? {};
  const config = (values.spatial_config_state ??
    values.config_state ??
    {}) as Record<string, unknown>;
  const switches = (config.switches ?? {}) as Record<string, unknown>;
  const pose = values.absolute_pose as unknown as AbsolutePoseFrame | undefined;
  const motion = values.relative_motion as unknown as
    RelativeToolMotion | undefined;
  const effectiveAxes = (config.base_from_tracking_axes ?? []) as number[][];
  const [axesDraft, setAxesDraft] = useState<number[][]>();
  const axes = axesDraft ?? effectiveAxes;

  async function origin() {
    setError(undefined);
    try {
      await post("/api/spatial/origin", {
        schema_version: 2,
        request_id: requestId(),
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  async function update(patchValue: Record<string, Json>) {
    setError(undefined);
    try {
      await patch("/api/spatial/config", {
        schema_version: 2,
        request_id: requestId(),
        patch: patchValue,
      });
    } catch (reason) {
      setError(String(reason));
    }
  }

  const translation = motion?.translation_m;
  const active = Boolean(motion?.active);
  const originM = Array.isArray(config.origin_position_m)
    ? (config.origin_position_m as [number, number, number])
    : ([0, 0, 0] as [number, number, number]);

  return (
    <Shell
      section="02 / SPATIAL TRANSFORM"
      title="空间转换"
      description="图形展示坐标映射后的空间位置与设备姿态；原点、倍率和分量配置紧邻结果，原始消息统一收进排障区。"
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
            label="前部抬起 / 往下"
            value={degrees(motion?.front_pitch_rad)}
            unit="deg"
            tone="amber"
          />
          <Metric
            label="左旋 / 右旋"
            value={degrees(motion?.horizontal_arc_rad)}
            unit="deg"
            tone="cyan"
          />
          <Metric
            label="接管会话"
            value={
              active ? String(motion?.control_session_id ?? "ACTIVE") : "IDLE"
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
              {active ? "相对运动输出中" : "等待接管"}
            </StatusBadge>
          }
        >
          <PoseViewer
            pose={pose}
            motion={motion}
            poseCoordinates="robot"
            originM={originM}
            axes={effectiveAxes}
            translationScale={Number(config.translation_scale ?? 1)}
            active={active}
            ariaLabel="映射后空间位置和设备自身姿态"
          />
        </Card>

        <Card
          className="span-4 aligned-row-card"
          eyebrow="Transform configuration"
          title="空间配置"
          action={<Button onClick={origin}>确认当前位置为原点</Button>}
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
              {[
                ["translation", "空间位置移动"],
                ["front_pitch", "前部抬起 / 前部往下"],
                ["horizontal_arc", "左旋 / 右旋"],
              ].map(([key, label]) => (
                <label key={key} className="switch-row">
                  <span>{label}</span>
                  <input
                    type="checkbox"
                    checked={Boolean(switches[key])}
                    onChange={(event) =>
                      update({
                        switches: {
                          translation: Boolean(switches.translation),
                          front_pitch: Boolean(switches.front_pitch),
                          horizontal_arc: Boolean(switches.horizontal_arc),
                          [key]: event.currentTarget.checked,
                        },
                      })
                    }
                  />
                </label>
              ))}
            </div>
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
            <JsonView title="原始绝对位姿" value={pose} />
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
