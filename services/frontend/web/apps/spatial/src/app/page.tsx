"use client";
import type { Json } from "@robot/contracts";
import { patch, post, requestId, useGateway } from "@robot/gateway-client";
import { Button, Card, Field, Input, JsonView, Shell } from "@robot/ui";
import { useEffect, useState } from "react";
export default function Page() {
  const { snapshot, error, setError } = useGateway("spatial");
  const values = snapshot?.values ?? {};
  const config = (values.spatial_config_state ??
    values.config_state ??
    {}) as Record<string, unknown>;
  const switches = (config.switches ?? {}) as Record<string, unknown>;
  const effectiveAxes = (config.base_from_tracking_axes ?? []) as number[][];
  const axesKey = JSON.stringify(effectiveAxes);
  const [axes, setAxes] = useState<number[][]>(effectiveAxes);
  useEffect(() => setAxes(JSON.parse(axesKey) as number[][]), [axesKey]);
  async function origin() {
    setError(undefined);
    try {
      await post("/api/spatial/origin", {
        schema_version: 1,
        request_id: requestId(),
      });
    } catch (e) {
      setError(String(e));
    }
  }
  async function update(patchValue: Record<string, Json>) {
    setError(undefined);
    try {
      await patch("/api/spatial/config", {
        schema_version: 1,
        request_id: requestId(),
        patch: patchValue,
      });
    } catch (e) {
      setError(String(e));
    }
  }
  return (
    <Shell
      title="空间转换"
      description="这里只处理原点、坐标映射、倍率和三个空间分量，不计算机械臂运动学。"
    >
      <div className="grid">
        <Card title="实际生效配置">
          <Field label="平移倍率（无量纲）">
            <Input
              key={String(config.translation_scale ?? "")}
              type="number"
              step="any"
              defaultValue={
                config.translation_scale == null
                  ? ""
                  : Number(config.translation_scale)
              }
              onBlur={(e) =>
                update({ translation_scale: e.currentTarget.valueAsNumber })
              }
            />
          </Field>
          <Field label="OpenXR 平移到前 / 左 / 上的 3×3 映射">
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
                      setAxes(next);
                    }}
                    onBlur={() => update({ base_from_tracking_axes: axes })}
                  />
                )),
              )}
            </div>
          </Field>
          <Field label="采集分量">
            {[
              ["translation", "空间位置移动"],
              ["front_pitch", "前部抬起 / 前部往下"],
              ["horizontal_arc", "左旋 / 右旋"],
            ].map(([key, label]) => (
              <label key={key} className="row">
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
                {label}
              </label>
            ))}
          </Field>
          <Field label="无绝对位置输入的平移速度（m/s，留空表示未配置）">
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
          <Field label="无绝对姿态输入的圆弧角速度（rad/s，留空表示未配置）">
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
          <Button onClick={origin}>确认当前位置为原点</Button>
          <JsonView value={config} />
        </Card>
        <Card title="原始绝对位姿">
          <JsonView value={values.absolute_pose} />
        </Card>
        <Card title="相对运动">
          <JsonView value={values.relative_motion} />
        </Card>
        <Card title="配置请求结果">
          <JsonView value={values.spatial_request_result} />
        </Card>
      </div>
      {error && <p className="error">{error}</p>}
    </Shell>
  );
}
