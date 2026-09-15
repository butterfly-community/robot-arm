"use client";

import { useState } from "react";
import {
  schemaVersion,
  type ArmState,
  type MotionState,
  type RobotModelInfo,
  type TcpMotionTarget,
} from "@robot/contracts";
import { post, requestId } from "@robot/gateway-client";
import { Button, Card, Field, Input } from "@robot/ui";
import {
  RobotViewer,
  tcpTargetPreview,
} from "@robot/visualization/robot-viewer";

export function TcpTarget({
  model,
  arm,
  motion,
  busy,
}: {
  model: RobotModelInfo;
  arm?: ArmState;
  motion?: MotionState;
  busy: boolean;
}) {
  const [draft, setDraft] = useState<TcpMotionTarget>();
  const [pending, setPending] = useState(false);
  const [error, setError] = useState<string>();
  const current = motion?.current_tool_pose;
  const value = draft ?? {
    relative: false,
    pose: current ?? {
      frame: model.base_frame,
      position_m: [0, 0, 0],
      orientation_xyzw: [0, 0, 0, 1],
    },
  };
  const target = current
    ? tcpTargetPreview(current, value, model.base_frame, model.tcp_frame)
    : undefined;
  async function execute() {
    setPending(true);
    setError(undefined);
    try {
      await post("/api/motion/mode", {
        schema_version: schemaVersion,
        request_id: requestId(),
        mode: "manual",
      });
      await post("/api/motion/request", {
        schema_version: schemaVersion,
        request_id: requestId(),
        model_revision: model.model_revision,
        joints: [],
        actuators: [],
        options: {},
        action: "apply",
        tcp_target: {
          relative: value.relative,
          pose: {
            frame: value.pose.frame,
            position_m: value.pose.position_m,
            orientation_xyzw: value.pose.orientation_xyzw,
          },
        },
      });
    } catch (reason) {
      setError(String(reason));
    } finally {
      setPending(false);
    }
  }
  return (
    <Card className="span-12" title="TCP 定点运动">
      <div className="tcp-target-fields">
        <Field label="目标方式">
          <select
            aria-label="目标方式"
            value={value.relative ? "relative" : "absolute"}
            onChange={(event) =>
              setDraft({
                relative: event.target.value === "relative",
                pose:
                  event.target.value === "relative"
                    ? {
                        frame: model.base_frame,
                        position_m: [0, 0, 0],
                        orientation_xyzw: [0, 0, 0, 1],
                      }
                    : (current ?? value.pose),
              })
            }
          >
            <option value="absolute">绝对位姿</option>
            <option value="relative">相对当前实际 TCP</option>
          </select>
        </Field>
        <Field label="目标坐标系">
          <select
            aria-label="目标坐标系"
            value={value.pose.frame}
            onChange={(event) =>
              setDraft({
                ...value,
                pose: { ...value.pose, frame: event.target.value },
              })
            }
          >
            <option value={model.base_frame}>底座 · {model.base_frame}</option>
            {value.relative && (
              <option value={model.tcp_frame}>工具 · {model.tcp_frame}</option>
            )}
          </select>
        </Field>
        {(["X", "Y", "Z"] as const).map((axis, i) => (
          <Field key={axis} label={`${axis} / m`}>
            <Input
              aria-label={`${axis} / m`}
              type="number"
              step="any"
              value={value.pose.position_m[i]}
              onChange={(event) => {
                const position = [...value.pose.position_m] as [
                  number,
                  number,
                  number,
                ];
                position[i] = Number(event.target.value);
                setDraft({
                  ...value,
                  pose: { ...value.pose, position_m: position },
                });
              }}
            />
          </Field>
        ))}
        {(["qx", "qy", "qz", "qw"] as const).map((axis, i) => (
          <Field key={axis} label={axis}>
            <Input
              aria-label={axis}
              type="number"
              step="any"
              value={value.pose.orientation_xyzw[i]}
              onChange={(event) => {
                const q = [...value.pose.orientation_xyzw] as [
                  number,
                  number,
                  number,
                  number,
                ];
                q[i] = Number(event.target.value);
                setDraft({
                  ...value,
                  pose: { ...value.pose, orientation_xyzw: q },
                });
              }}
            />
          </Field>
        ))}
      </div>
      <p>
        相对目标在开始处理该运动时，基于实际关节反馈计算。预览只显示目标坐标轴，不表示
        IK 或路径已通过。
      </p>
      <div className="card-actions">
        <Button disabled={!current || busy || pending} onClick={execute}>
          {pending ? "正在执行 TCP 目标…" : "执行 TCP 目标"}
        </Button>
        <Button disabled={pending} onClick={() => setDraft(undefined)}>
          恢复当前 TCP
        </Button>
      </div>
      {error && <p role="alert">{error}</p>}
      <RobotViewer
        ariaLabel="TCP 目标坐标轴预览"
        model={model}
        arm={arm}
        motion={{
          control_mode: motion?.control_mode ?? "manual",
          current_tool_pose: current,
          target_tool_pose: target,
        }}
        parameters={[]}
        showLabels={false}
      />
    </Card>
  );
}
