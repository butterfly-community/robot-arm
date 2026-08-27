"use client";
import { post, requestId, useGateway } from "@robot/gateway-client";
import { Button, Card, Field, JsonView, Shell } from "@robot/ui";
import { useEffect, useState } from "react";

const actions = [
  ["control_active", "接管控制", "boolean"],
  ["confirm_origin", "确认原点", "boolean"],
  ["primary_tool_active", "主工具动作", "boolean"],
  ["move_forward_back", "前后移动", "float"],
  ["move_left_right", "左右移动", "float"],
  ["move_up_down", "上下移动", "float"],
  ["front_pitch", "前部抬起 / 往下", "float"],
  ["horizontal_arc", "左旋 / 右旋", "float"],
] as const;

export default function Page() {
  const { snapshot, error, setError } = useGateway("tracking");
  const values = snapshot?.values ?? {};
  const discovery = (values.discovery_state ?? {}) as Record<string, unknown>;
  const sources = (discovery.runtime_sources ?? []) as Array<
    Record<string, unknown>
  >;
  const bindingStates = (discovery.bindings ?? []) as Array<
    Record<string, unknown>
  >;
  const bindingsKey = JSON.stringify(bindingStates);
  const [paths, setPaths] = useState<Record<string, string>>({});
  const [inverted, setInverted] = useState<Record<string, boolean>>({});
  const [types, setTypes] = useState<Record<string, "boolean" | "float">>({});
  const [hostAssociation, setHostAssociation] = useState("");
  const confirmedAssociation = String(
    discovery.confirmed_host_association ?? "",
  );
  const selected =
    sources.find(
      (source) => source.source_id === discovery.selected_source_id,
    ) ?? (discovery.selected_source as Record<string, unknown> | undefined);
  const components = (selected?.available_components ?? []) as Array<
    Record<string, unknown>
  >;
  const simulation = (discovery.simulation ?? {}) as Record<string, unknown>;
  useEffect(
    () => setHostAssociation(confirmedAssociation),
    [confirmedAssociation],
  );
  useEffect(() => {
    const current = JSON.parse(bindingsKey) as Array<Record<string, unknown>>;
    setPaths(
      Object.fromEntries(
        current.map((binding) => [
          String(binding.action),
          ((binding.configured_components ?? []) as unknown[])
            .map(String)
            .join(", "),
        ]),
      ),
    );
    setInverted(
      Object.fromEntries(
        current.map((binding) => [
          String(binding.action),
          Boolean(binding.invert),
        ]),
      ),
    );
    setTypes(
      Object.fromEntries(
        current.map((binding) => [
          String(binding.action),
          binding.action_type === "boolean" ? "boolean" : "float",
        ]),
      ),
    );
  }, [bindingsKey]);
  async function select(source: Record<string, unknown>) {
    setError(undefined);
    try {
      await post("/api/tracking/source", {
        schema_version: 1,
        request_id: requestId(),
        action: "select",
        runtime_identity: String(
          (discovery.runtime as Record<string, unknown>)?.runtime_name ?? "",
        ),
        user_path: String(source.user_path),
        interaction_profile:
          source.interaction_profile == null
            ? null
            : String(source.interaction_profile),
        confirmed_host_association: hostAssociation || null,
      });
    } catch (e) {
      setError(String(e));
    }
  }
  async function unselect() {
    if (!selected) return;
    setError(undefined);
    try {
      await post("/api/tracking/source", {
        schema_version: 1,
        request_id: requestId(),
        action: "unselect",
        runtime_identity: String(
          (discovery.runtime as Record<string, unknown>)?.runtime_name ?? "",
        ),
        user_path: String(selected.user_path),
        interaction_profile:
          selected.interaction_profile == null
            ? null
            : String(selected.interaction_profile),
        confirmed_host_association: hostAssociation || null,
      });
    } catch (e) {
      setError(String(e));
    }
  }
  function setPath(action: string, index: number, value: string) {
    const values = (paths[action] ?? "").split(",").map((item) => item.trim());
    values[index] = value;
    setPaths({ ...paths, [action]: values.filter(Boolean).join(", ") });
  }
  async function applyBindings() {
    const profile = String(selected?.interaction_profile ?? "");
    setError(undefined);
    try {
      await post("/api/tracking/bindings", {
        schema_version: 1,
        request_id: requestId(),
        interaction_profile: profile,
        bindings: actions
          .map(([action, , defaultType]) => ({
            action,
            action_type: types[action] ?? defaultType,
            component_paths: (paths[action] ?? "")
              .split(",")
              .map((value) => value.trim())
              .filter(Boolean),
            invert: Boolean(inverted[action]),
          }))
          .filter((binding) => binding.component_paths.length > 0),
      });
    } catch (e) {
      setError(String(e));
    }
  }
  async function setSimulation(enabled: boolean) {
    setError(undefined);
    try {
      if (enabled) {
        await post("/api/motion/mode", {
          schema_version: 1,
          request_id: requestId(),
          mode: "relative",
        });
      }
      await post("/api/tracking/simulation", {
        schema_version: 1,
        request_id: requestId(),
        enabled,
      });
    } catch (e) {
      setError(String(e));
    }
  }
  return (
    <Shell
      title="输入采集"
      description="并列查看主机硬件、OpenXR Runtime 与实际输入端点；配置只保存在采集服务。"
    >
      <div className="grid">
        <Card title="模拟输入测试">
          <p>
            依次平滑执行上/下、左/右、前/后空间移动，以及手柄自身左旋/右旋、前部抬起/往下。
          </p>
          <Button onClick={() => setSimulation(!Boolean(simulation.active))}>
            {simulation.active ? "停止模拟数据" : "启动模拟数据"}
          </Button>
          <p className="status">
            {simulation.active
              ? String(simulation.phase ?? "模拟输入运行中")
              : "模拟输入未运行"}
          </p>
        </Card>
        <Card title="主机硬件">
          <Field label="与当前 Runtime 输入端点人工关联（可选）">
            <select
              value={hostAssociation}
              onChange={(event) =>
                setHostAssociation(event.currentTarget.value)
              }
            >
              <option value="">未关联</option>
              {(
                (discovery.host_devices ?? []) as Array<Record<string, unknown>>
              ).map((device) => (
                <option
                  key={String(device.bus_path)}
                  value={String(device.bus_path)}
                >
                  {String(device.product ?? device.bus_path)} ·{" "}
                  {String(device.vendor_id)}:{String(device.product_id)}
                </option>
              ))}
            </select>
          </Field>
          <JsonView value={discovery.host_devices} />
        </Card>
        <Card title="Runtime / System">
          <JsonView value={discovery.runtime} />
        </Card>
        <Card title="Runtime 输入端点">
          {sources.map((source) => (
            <div key={String(source.source_id)} className="card">
              <strong>
                {String(source.localized_name ?? source.user_path)}
              </strong>
              <JsonView value={source} />
              <Button onClick={() => select(source)}>使用此输入源</Button>
            </div>
          ))}
          {selected && <Button onClick={unselect}>停止使用当前输入源</Button>}
        </Card>
        <Card title="Action 绑定与实时值">
          {actions.map(([action, label, defaultType]) => (
            <Field key={action} label={label}>
              <select
                value={types[action] ?? defaultType}
                disabled={defaultType === "boolean"}
                onChange={(event) =>
                  setTypes({
                    ...types,
                    [action]: event.currentTarget.value as "boolean" | "float",
                  })
                }
              >
                <option value="boolean">按钮 / 正负按钮对</option>
                <option value="float">连续轴</option>
              </select>
              {Array.from({
                length:
                  defaultType === "float" && types[action] === "boolean"
                    ? 2
                    : 1,
              }).map((_, index) => (
                <select
                  key={index}
                  value={(paths[action] ?? "").split(",")[index]?.trim() ?? ""}
                  onChange={(event) =>
                    setPath(action, index, event.currentTarget.value)
                  }
                >
                  <option value="">
                    {index === 0 &&
                    types[action] === "boolean" &&
                    defaultType === "float"
                      ? "选择负方向按钮"
                      : index === 1
                        ? "选择正方向按钮"
                        : "选择 Runtime component"}
                  </option>
                  {components
                    .filter(
                      (component) =>
                        component.action_type ===
                        (types[action] ?? defaultType),
                    )
                    .map((component) => (
                      <option
                        key={String(component.path)}
                        value={String(component.path)}
                      >
                        {String(component.localized_name ?? component.path)}
                      </option>
                    ))}
                </select>
              ))}
              <label className="row">
                <input
                  type="checkbox"
                  checked={Boolean(inverted[action])}
                  onChange={(event) =>
                    setInverted({
                      ...inverted,
                      [action]: event.currentTarget.checked,
                    })
                  }
                />
                反向
              </label>
            </Field>
          ))}
          <Button onClick={applyBindings}>应用绑定</Button>
          <JsonView value={discovery.bindings} />
          <JsonView
            value={
              sources.find(
                (source) => source.source_id === discovery.selected_source_id,
              )?.available_components
            }
          />
        </Card>
        <Card title="原始位姿">
          <JsonView value={values.absolute_pose} />
        </Card>
        <Card title="原始 Action">
          <JsonView value={values.control_input} />
        </Card>
      </div>
      {error && <p className="error">{error}</p>}
    </Shell>
  );
}
