"use client";
import type { ArmState, RobotModelInfo } from "@robot/contracts";
import { post, requestId, useGateway } from "@robot/gateway-client";
import {
  Button,
  Card,
  Field,
  Input,
  JsonView,
  RangeControls,
  Shell,
} from "@robot/ui";
import { useEffect, useState } from "react";
export default function Page() {
  const { snapshot, error, setError } = useGateway("motion");
  const values = snapshot?.values ?? {};
  const model = values.robot_model_info as unknown as
    | RobotModelInfo
    | undefined;
  const arm = values.arm_state as unknown as ArmState | undefined;
  const [positions, setPositions] = useState<Record<string, number>>({});
  const [editing, setEditing] = useState<string>();
  const [options, setOptions] = useState<Record<string, number | undefined>>(
    {},
  );
  useEffect(() => {
    if (arm && !editing)
      setPositions(
        Object.fromEntries(
          (model?.joints ?? []).map((joint, index) => [
            joint.key,
            arm.joints_rad[index] ?? 0,
          ]),
        ),
      );
  }, [arm, editing, model]);
  async function mode(value: "relative" | "manual") {
    setError(undefined);
    try {
      await post("/api/motion/mode", {
        schema_version: 1,
        request_id: requestId(),
        mode: value,
      });
    } catch (e) {
      setError(String(e));
    }
  }
  async function cancel() {
    setError(undefined);
    try {
      await post("/api/motion/cancel", {
        schema_version: 1,
        request_id: requestId(),
        action: "cancel",
      });
    } catch (e) {
      setError(String(e));
    }
  }
  async function move(target = positions) {
    if (!model) return;
    setError(undefined);
    try {
      await post("/api/motion/request", {
        schema_version: 1,
        request_id: requestId(),
        model_revision: model.model_revision,
        joints: Object.entries(target).map(([joint_key, position_rad]) => ({
          joint_key,
          position_rad,
        })),
        options: Object.fromEntries(
          Object.entries(options).filter((entry): entry is [string, number] =>
            Number.isFinite(entry[1]),
          ),
        ),
        action: "apply",
      });
    } catch (e) {
      setError(String(e));
    }
  }
  return (
    <Shell
      title="机械臂运动"
      description="关节、命名目标和诊断字段全部来自当前运动型号节点的元数据。"
    >
      <div className="grid">
        <Card title="控制模式">
          <div className="row">
            <Button onClick={() => mode("relative")}>相对控制</Button>
            <Button onClick={() => mode("manual")}>手动控制</Button>
            <Button onClick={cancel}>取消普通运动</Button>
          </div>
          <p className="status">
            当前：
            {String(
              (values.motion_state as Record<string, unknown> | undefined)
                ?.control_mode ?? "未知",
            )}
          </p>
        </Card>
        <Card title="手动关节">
          {model?.motion_options.map((option) => (
            <Field
              key={option.key}
              label={`${option.label}（${option.unit}，留空则不传）`}
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
          <RangeControls
            items={model?.joints ?? []}
            values={positions}
            onBegin={setEditing}
            onChange={(key, value) =>
              setPositions({ ...positions, [key]: value })
            }
            onCommit={(key, value) => {
              const target = { ...positions, [key]: value };
              setEditing(undefined);
              void move(target);
            }}
          />
        </Card>
        <Card title="命名目标">
          <div className="row">
            {model?.named_targets.map((t) => (
              <Button
                key={t.key}
                onClick={() => {
                  setPositions(t.positions_rad);
                  void move(t.positions_rad);
                }}
              >
                {t.label}
              </Button>
            ))}
          </div>
        </Card>
        <Card title="运动状态">
          <JsonView value={values.motion_state} />
        </Card>
        <Card title="请求结果">
          <JsonView value={values.motion_request_result} />
        </Card>
        <Card title="型号元数据">
          <JsonView value={model} />
        </Card>
      </div>
      {error && <p className="error">{error}</p>}
    </Shell>
  );
}
