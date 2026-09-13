"use client";
import { useState } from "react";
import { Button, Field, Input } from "@robot/ui";
import { post, requestId } from "@robot/gateway-client";
import {
  schemaVersion,
  type ExecutionInfo,
  type ParameterValue,
} from "@robot/contracts";

export function ParameterEditor({
  info,
  parameters,
  connected,
  transport,
}: {
  info?: ExecutionInfo;
  parameters: ParameterValue[];
  connected: boolean;
  transport: Record<string, unknown>;
}) {
  const [actuator, setActuator] = useState("");
  const [field, setField] = useState("");
  const [draft, setDraft] = useState<string>();
  const [sending, setSending] = useState(false);
  const [error, setError] = useState<string>();
  const [submitted, setSubmitted] = useState(false);
  const [writeId, setWriteId] = useState<string>();
  const actuators = [...new Set(parameters.map((p) => p.actuator_key))];
  const fields =
    info?.parameter_fields.filter((p) =>
      info.writable_parameter_keys?.includes(p.key),
    ) ?? [];
  const selectedActuator = actuator || actuators[0] || "";
  const selectedField = field || fields[0]?.key || "";
  const current = parameters.find(
    (p) => p.actuator_key === selectedActuator && p.field_key === selectedField,
  );
  const value = draft ?? (current?.value == null ? "" : String(current.value));
  const busy = sending || Boolean(transport.parameter_write_pending);
  async function save() {
    setSending(true);
    setError(undefined);
    setSubmitted(false);
    try {
      const id = requestId();
      setWriteId(id);
      await post("/api/arm-execution/parameters", {
        schema_version: schemaVersion,
        request_id: id,
        action: "apply",
        fields: {
          actuator_key: selectedActuator,
          field_key: selectedField,
          value,
        },
      });
      setSubmitted(true);
    } catch (e) {
      setError(String(e));
    } finally {
      setSending(false);
    }
  }
  return (
    <div className="card-stack">
      <p className="muted">
        逐项写入舵机并回读核对；单位沿用下表。ID /
        波特率涉及设备映射，仅在驱动维护接口提供。型号、固件和序列号为只读。
      </p>
      <p className="muted" data-testid="parameter-help">
        {info?.parameter_help?.[selectedField] ?? "等待参数说明"}
      </p>
      <div className="grid">
        <Field label="参数执行器">
          <select
            aria-label="参数执行器"
            value={selectedActuator}
            disabled={busy}
            onChange={(e) => {
              setActuator(e.target.value);
              setError(undefined);
              setDraft(undefined);
              setSubmitted(false);
            }}
          >
            {actuators.map((a) => (
              <option key={a} value={a}>
                {a}
              </option>
            ))}
          </select>
        </Field>
        <Field label="可写参数">
          <select
            aria-label="可写参数"
            value={selectedField}
            disabled={busy}
            onChange={(e) => {
              setField(e.target.value);
              setError(undefined);
              setDraft(undefined);
              setSubmitted(false);
            }}
          >
            {fields.map((f) => (
              <option key={f.key} value={f.key}>
                {f.label}
                {f.unit ? `（${f.unit}）` : ""}
              </option>
            ))}
          </select>
        </Field>
        <Field label="参数值">
          <Input
            aria-label="参数值"
            type="number"
            step="1"
            value={value}
            disabled={busy}
            onChange={(e) => {
              setDraft(e.target.value);
              setError(undefined);
              setSubmitted(false);
            }}
          />
        </Field>
      </div>
      <div className="card-actions">
        <Button
          disabled={!connected || busy || !selectedField || value.trim() === ""}
          onClick={() => void save()}
        >
          {busy ? "正在写入并回读…" : "写入并回读参数"}
        </Button>
        <Button
          disabled={busy}
          onClick={() => {
            setDraft(undefined);
            setSubmitted(false);
            setError(undefined);
          }}
        >
          恢复实读值
        </Button>
      </div>
      <p role="status">
        {busy
          ? "等待舵机写入及实值校验"
          : submitted
            ? transport.parameter_write_request_id !== writeId
              ? "请求已受理，等待回读"
              : transport.parameter_write_error
                ? `写入未确认：${String(transport.parameter_write_error)}`
                : transport.parameter_write_verified
                  ? "参数已写入，回读一致"
                  : "请求已受理，等待回读"
            : `当前实读值：${current?.original_error ?? current?.value ?? "未读取"}`}
      </p>
      {error && (
        <p className="error" role="alert">
          {error}
        </p>
      )}
    </div>
  );
}
