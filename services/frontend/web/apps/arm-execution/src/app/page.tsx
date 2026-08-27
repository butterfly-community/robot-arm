"use client";
import type {
  ArmCommand,
  ArmState,
  ExecutionInfo,
  MotionState,
  ParameterValue,
  RobotModelInfo,
} from "@robot/contracts";
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
import { RobotViewer } from "./robot-viewer";
export default function Page() {
  const { snapshot, error, setError } = useGateway("arm-execution");
  const values = snapshot?.values ?? {};
  const model = values.robot_model_info as unknown as
    | RobotModelInfo
    | undefined;
  const arm = values.arm_state as unknown as ArmState | undefined;
  const motion = values.motion_state as unknown as MotionState | undefined;
  const execution = values.execution_info as unknown as
    | ExecutionInfo
    | undefined;
  const transport = (values.transport_state ?? {}) as Record<string, unknown>;
  const parameters = (transport.parameter_values ?? []) as ParameterValue[];
  const command = transport.last_command as ArmCommand | undefined;
  const [connectionFields, setConnectionFields] = useState<
    Record<string, string>
  >({});
  const [actuators, setActuators] = useState<Record<string, number>>({});
  const [editing, setEditing] = useState<string>();
  const [showLabels, setShowLabels] = useState(false);
  useEffect(() => {
    if (!model || !arm || editing) return;
    setActuators(
      Object.fromEntries(
        model.tool_actuators.map((item, index) => [
          item.key,
          arm.actuators_rad[index] ?? 0,
        ]),
      ),
    );
  }, [arm, editing, model]);
  async function connection(action: "connect" | "disconnect" | "refresh") {
    setError(undefined);
    try {
      await post(
        `/api/arm-execution/${action === "refresh" ? "parameters" : action}`,
        {
          schema_version: 1,
          request_id: requestId(),
          action,
          fields: connectionFields,
        },
      );
    } catch (e) {
      setError(String(e));
    }
  }
  return (
    <Shell
      title="机械臂执行"
      description="软件反馈与实际执行端点共用一个执行节点、一个 ArmState；连接只改变反馈和命令去向。"
    >
      <div className="grid">
        <Card title="三维反馈与目标预览" className="viewer-card">
          <label className="row">
            <input
              type="checkbox"
              checked={showLabels}
              onChange={(event) => setShowLabels(event.currentTarget.checked)}
            />
            显示关节编号和本次真机参数
          </label>
          {model ? (
            <RobotViewer
              model={model}
              arm={arm}
              command={command}
              motion={motion}
              execution={execution}
              parameters={parameters}
              showLabels={showLabels}
            />
          ) : (
            <p className="status">等待运动节点发布模型</p>
          )}
          <p className="status">
            实体模型跟随
            ArmState；半透明模型是最后命令；橙色坐标标记是当前工具目标。
          </p>
        </Card>
        <Card title="执行连接">
          {transport.connected ? (
            <>
              <p>已连接：{String(transport.selected_endpoint ?? "")}</p>
              <Button onClick={() => connection("disconnect")}>断开</Button>
            </>
          ) : (
            <>
              {execution?.connection_fields.map((field) => (
                <Field key={field.key} label={field.label}>
                  {field.field_type === "endpoint" ? (
                    <>
                      <Input
                        required={field.required}
                        list={`connection-${field.key}`}
                        value={connectionFields[field.key] ?? ""}
                        onChange={(event) =>
                          setConnectionFields({
                            ...connectionFields,
                            [field.key]: event.currentTarget.value,
                          })
                        }
                      />
                      <datalist id={`connection-${field.key}`}>
                        {(
                          (transport.discovered_endpoints ?? []) as Array<
                            Record<string, unknown>
                          >
                        ).map((endpoint) => (
                          <option
                            key={String(endpoint.key)}
                            value={String(endpoint.key)}
                          >
                            {String(endpoint.label ?? endpoint.key)}
                          </option>
                        ))}
                        {field.options.map((option) => (
                          <option key={option} value={option}>
                            {option}
                          </option>
                        ))}
                      </datalist>
                    </>
                  ) : (
                    <Input
                      required={field.required}
                      value={connectionFields[field.key] ?? ""}
                      onChange={(event) =>
                        setConnectionFields({
                          ...connectionFields,
                          [field.key]: event.currentTarget.value,
                        })
                      }
                    />
                  )}
                </Field>
              ))}
              <Button onClick={() => connection("connect")}>连接</Button>
            </>
          )}{" "}
          <Button onClick={() => connection("refresh")}>读取参数</Button>
          <JsonView value={transport} />
        </Card>
        <Card title="唯一反馈状态">
          <p className="status">来源：{arm?.feedback_source ?? "尚无反馈"}</p>
          <JsonView value={arm} />
        </Card>
        <Card title="型号和执行元数据">
          <JsonView value={{ model, execution_info: execution }} />
        </Card>
        <Card title="最后命令">
          <JsonView value={command} />
        </Card>
        <Card title="执行器">
          {model && (
            <RangeControls
              items={model.tool_actuators}
              values={actuators}
              onBegin={setEditing}
              onChange={(key, value) =>
                setActuators({ ...actuators, [key]: value })
              }
              onCommit={(key, value) => {
                setEditing(undefined);
                setError(undefined);
                void post("/api/arm-execution/actuator", {
                  schema_version: 1,
                  request_id: requestId(),
                  model_revision: model.model_revision,
                  actuator_key: key,
                  position_rad: value,
                  action: "apply",
                }).catch((reason) => setError(String(reason)));
              }}
            />
          )}
        </Card>
      </div>
      {error && <p className="error">{error}</p>}
    </Shell>
  );
}
