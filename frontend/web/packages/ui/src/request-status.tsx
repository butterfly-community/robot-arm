"use client";

import type { RequestRecord } from "@robot/contracts";
import { useEffect, useState } from "react";
import { Button, Disclosure, Field, Input, KeyValue } from "./index";

const labels: Record<RequestRecord["state"], string> = {
  accepted: "已受理",
  idle: "待执行",
  planning: "规划中",
  executing: "执行中",
  succeeded: "执行成功",
  failed: "执行失败",
  cancelled: "已取消",
  unknown: "结果未确认",
};

export function RequestStatus({ scope }: { scope: "motion" | "perception" }) {
  const [id, setId] = useState("");
  const [draft, setDraft] = useState("");
  const [record, setRecord] = useState<RequestRecord>();
  const [error, setError] = useState<string>();
  const [refresh, setRefresh] = useState(0);
  useEffect(() => {
    const restore = () => {
      const saved =
        localStorage.getItem(`robot-arm:last-request:${scope}`) ?? "";
      setId(saved);
      setDraft(saved);
    };
    restore();
    window.addEventListener("robot-arm:request", restore);
    window.addEventListener("storage", restore);
    return () => {
      window.removeEventListener("robot-arm:request", restore);
      window.removeEventListener("storage", restore);
    };
  }, [scope]);
  useEffect(() => {
    if (!id) return;
    const controller = new AbortController();
    let timer: ReturnType<typeof setTimeout> | undefined;
    const load = async () => {
      let finished = false;
      try {
        const response = await fetch(
          `/api/requests/${encodeURIComponent(id)}`,
          { cache: "no-store", signal: controller.signal },
        );
        const value = await response.json();
        if (!response.ok)
          throw Error(value.original_error ?? `HTTP ${response.status}`);
        setRecord(value as RequestRecord);
        setError(undefined);
        finished = value.terminal || value.state === "unknown";
      } catch (reason) {
        if (!controller.signal.aborted) setError(String(reason));
      }
      if (!controller.signal.aborted && !finished)
        timer = setTimeout(load, 1000);
    };
    void load();
    return () => {
      controller.abort();
      if (timer) clearTimeout(timer);
    };
  }, [id, refresh]);
  const current = record?.request_id === id ? record : undefined;
  return (
    <Disclosure
      title="请求结果"
      englishTitle="按请求编号查询；刷新页面不会重发动作。成功仅表示对应控制流程完成。"
    >
      <Field label="查询请求编号">
        <Input
          aria-label="查询请求编号"
          value={draft}
          onChange={(event) => setDraft(event.target.value)}
        />
      </Field>
      <div className="card-actions">
        <Button
          disabled={!draft.trim()}
          onClick={() => {
            setId(draft.trim());
            setError(undefined);
            setRefresh((value) => value + 1);
          }}
        >
          查看请求结果
        </Button>
      </div>
      <KeyValue label="当前请求" value={id || "尚未发起请求"} />
      <KeyValue
        label="请求状态"
        value={current ? labels[current.state] : id ? "正在查询" : "—"}
      />
      <KeyValue
        label="执行结果"
        value={String(
          current?.original_error ??
            current?.value?.result_message ??
            current?.value?.stage ??
            "—",
        )}
      />
      <KeyValue
        label="记录时间"
        value={current ? new Date(current.updated_at_ms).toLocaleString() : "—"}
      />
      {error && <p role="alert">{error}</p>}
    </Disclosure>
  );
}
