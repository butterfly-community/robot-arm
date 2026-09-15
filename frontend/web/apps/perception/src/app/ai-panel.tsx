"use client";

import { useEffect, useState } from "react";
import Image from "next/image";
import {
  Button,
  Disclosure,
  EditableSelect,
  Field,
  Input,
  KeyValue,
  StatusBadge,
} from "@robot/ui";
import { requestId } from "@robot/gateway-client";
import {
  activeRun,
  errorText,
  type AIImage,
  type AIRun,
  type AISettings,
  type AISnapshot,
} from "./ai/types";

const endpoint = "/perception/api/ai/";
const labels = {
  running: "正在处理",
  stopping: "正在停止，等待当前动作结果",
  succeeded: "本轮回复已结束",
  failed: "失败",
  cancelled: "已取消",
  interrupted: "已中断，需核对原请求",
};
async function request(body?: unknown) {
  const response = await fetch(
    endpoint,
    body
      ? {
          method: "POST",
          headers: { "content-type": "application/json" },
          body: JSON.stringify(body),
        }
      : { cache: "no-store" },
  );
  const result = await response.json();
  if (!response.ok)
    throw Error(result.original_error ?? `HTTP ${response.status}`);
  return result;
}
export function AIPanel({ onBusy }: { onBusy: (busy: boolean) => void }) {
  const [session, setSession] = useState("");
  const [snapshot, setSnapshot] = useState<AISnapshot>();
  const [runs, setRuns] = useState<AIRun[]>([]);
  const [draft, setDraft] = useState<AISettings>();
  const [catalog, setCatalog] = useState<{
    baseURL: string;
    items: { id: string; label: string }[];
  }>();
  const [text, setText] = useState("");
  const [images, setImages] = useState<AIImage[]>([]);
  const [pending, setPending] = useState("");
  const [error, setError] = useState("");
  const [checkResult, setCheckResult] = useState("");
  const [now, setNow] = useState(() => Date.now());
  const [connected, setConnected] = useState(false);
  const current = runs.findLast(activeRun);
  const config = draft ?? snapshot?.settings;
  const models =
    catalog?.baseURL === config?.baseURL ? (catalog?.items ?? []) : [];
  const checkedEfforts = new Set(
    snapshot?.checks
      .filter(
        (c) =>
          c.baseURL === config?.baseURL &&
          c.model === config?.model &&
          c.reportedEffort === c.effort &&
          !c.error,
      )
      .map((c) => c.effort) ?? [],
  );
  // Common Responses values, not a claim that every provider/model supports them.
  const efforts = [
    ...new Set([
      "",
      "none",
      "minimal",
      "low",
      "medium",
      "high",
      "xhigh",
      "max",
      ...checkedEfforts,
    ]),
  ];
  const dirty = Boolean(
    draft && JSON.stringify(draft) !== JSON.stringify(snapshot?.settings),
  );
  useEffect(() => {
    const id = localStorage.getItem("robot-arm:ai-session") ?? requestId();
    localStorage.setItem("robot-arm:ai-session", id);
    queueMicrotask(() => setSession(id));
  }, []);
  useEffect(() => {
    if (!session) return;
    let disposed = false;
    void fetch(`${endpoint}?session=${encodeURIComponent(session)}`, {
      cache: "no-store",
    })
      .then(async (response) => {
        const value = await response.json();
        if (!response.ok) throw Error(value.original_error);
        if (!disposed) {
          setSnapshot(value);
          setRuns(value.runs);
        }
      })
      .catch((e) => {
        if (!disposed) setError(errorText(e));
      });
    const events = new EventSource(
      `${endpoint}events/?session=${encodeURIComponent(session)}`,
    );
    events.onopen = () => setConnected(true);
    events.onmessage = (event) => setRuns(JSON.parse(event.data));
    events.onerror = () => setConnected(false);
    return () => {
      disposed = true;
      events.close();
    };
  }, [session]);
  useEffect(() => {
    onBusy(Boolean(current));
  }, [current, onBusy]);
  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, []);
  function edit(key: keyof AISettings, value: string) {
    if (config) setDraft({ ...config, [key]: value });
  }
  async function operation(name: string, action: () => Promise<void>) {
    setPending(name);
    setError("");
    try {
      await action();
    } catch (e) {
      setError(errorText(e));
    } finally {
      setPending("");
    }
  }
  async function send() {
    await operation("start", async () => {
      const run = await request({
        action: "start",
        runId: requestId(),
        sessionId: session,
        text,
        images: images.map((v) => v.id),
      });
      setRuns((previous) =>
        previous.some((r) => r.id === run.id) ? previous : [...previous, run],
      );
      setText("");
      setImages([]);
    });
  }
  async function refreshModels() {
    if (!config) return;
    await operation("models", async () => {
      const items = await request({ action: "models", settings: config });
      setCatalog({ baseURL: config.baseURL, items });
    });
  }
  return (
    <div className="ai-task-panel">
      <div className="ai-task-heading">
        <div>
          <h3>通用 AI 助手</h3>
          <p>
            可直接读取已连接相机，无需上传图片。任务及使用的画面将发送到配置的模型服务。
          </p>
        </div>
        <StatusBadge tone={connected ? "cyan" : "neutral"}>
          {connected ? "实时进度" : "正在连接进度"}
        </StatusBadge>
      </div>
      <Disclosure
        title="模型与思考配置"
        englishTitle="使用 OpenAI 兼容 Responses API；密钥只在服务端。保存后新任务生效。"
      >
        {config && (
          <div className="ai-settings-grid">
            <Field label="API 地址">
              <Input
                aria-label="AI API 地址"
                value={config.baseURL}
                onChange={(e) => edit("baseURL", e.target.value)}
              />
            </Field>
            <Field
              label="通用模型"
              hint="下拉选择服务提供的模型，或直接输入模型 ID。"
            >
              <EditableSelect
                label="通用 AI 模型"
                value={config.model}
                onChange={(value) => edit("model", value)}
                options={models}
                emptyText={
                  pending === "models"
                    ? "正在查询模型…"
                    : "暂无模型列表，可直接输入或点击刷新"
                }
                onOpen={() => {
                  if (catalog?.baseURL !== config.baseURL && !pending)
                    void refreshModels();
                }}
              />
            </Field>
            <Field
              label="思考强度"
              hint="下拉选择常见档位或直接输入；留空使用模型默认。支持情况取决于模型，可点击验证连接与能力确认。"
            >
              <EditableSelect
                label="AI 思考强度"
                placeholder="模型默认"
                value={config.effort}
                onChange={(value) => edit("effort", value)}
                options={efforts.map((id) => ({
                  id,
                  label: `${id || "模型默认"}${checkedEfforts.has(id) ? " · 已验证" : ""}`,
                }))}
              />
            </Field>
          </div>
        )}
        <div className="card-actions">
          <Button
            disabled={Boolean(pending) || !config}
            onClick={() =>
              operation("save", async () => {
                const saved = await request({
                  action: "settings",
                  settings: config,
                });
                setSnapshot((v) => v && { ...v, settings: saved });
                setDraft(undefined);
              })
            }
          >
            {pending === "save" ? "保存中…" : "保存 AI 配置"}
          </Button>
          <Button
            disabled={Boolean(pending) || !config}
            onClick={refreshModels}
          >
            {pending === "models" ? "查询中…" : "刷新通用模型列表"}
          </Button>
          <Button
            disabled={Boolean(pending) || !config}
            onClick={() =>
              operation("check", async () => {
                const result = await request({
                  action: "check",
                  settings: config,
                  imageId: images[0]?.id,
                });
                setCheckResult(
                  `${result.actualModel ?? config?.model}：文本/流式/工具回传通过${result.vision ? "，已接收附图" : "；未附图，识图未测"}。${result.effortNote ?? "思考强度未核验"}。${result.description}`,
                );
                setSnapshot(
                  (v) => v && { ...v, checks: [...v.checks, result] },
                );
              })
            }
          >
            {pending === "check" ? "正在验证…" : "验证连接与能力"}
          </Button>
        </div>
        <KeyValue
          label="配置状态"
          value={dirty ? "有未保存修改；任务仍使用已保存值" : "已保存"}
        />
        <KeyValue
          label="服务端密钥"
          value={snapshot?.keyConfigured ? "已配置，不向网页返回" : "未配置"}
        />
        <KeyValue
          label="生效模型 / 思考强度"
          value={`${snapshot?.settings.model ?? "—"} / ${snapshot?.settings.effort || "模型默认"}`}
        />
        {checkResult && <p role="status">{checkResult}</p>}
      </Disclosure>
      <Field
        label="AI 任务"
        hint="例如：看看相机里有什么；回到工作位；把紫色海绵放到另一张纸上。只观察不会主动抓放。"
      >
        <textarea
          aria-label="AI 任务"
          rows={3}
          value={text}
          onChange={(e) => setText(e.target.value)}
        />
      </Field>
      <div className="card-actions">
        <label className="ai-upload">
          添加参考图片（可选）
          <input
            aria-label="添加 AI 图片"
            type="file"
            accept="image/*"
            multiple
            disabled={Boolean(pending)}
            onChange={(e) => {
              const files = [...(e.target.files ?? [])];
              e.target.value = "";
              void operation("upload", async () => {
                for (const file of files) {
                  const body = new FormData();
                  body.set("image", file);
                  const response = await fetch(`${endpoint}images/`, {
                    method: "POST",
                    body,
                  });
                  const value = await response.json();
                  if (!response.ok) throw Error(value.original_error);
                  setImages((v) => [...v, value]);
                }
              });
            }}
          />
        </label>
        <Button
          disabled={
            Boolean(pending) || !text.trim() || Boolean(current) || !session
          }
          onClick={send}
        >
          {pending === "start" ? "正在提交…" : "发送 AI 任务"}
        </Button>
        <Button
          disabled={Boolean(pending) || !current}
          onClick={() =>
            operation("stop", async () => {
              await request({ action: "stop", runId: current!.id });
            })
          }
        >
          {pending === "stop" ? "正在请求停止…" : "停止 AI 与当前动作"}
        </Button>
        <Button
          disabled={Boolean(current) || Boolean(pending)}
          onClick={() => {
            const id = requestId();
            localStorage.setItem("robot-arm:ai-session", id);
            setRuns([]);
            setSession(id);
          }}
        >
          新会话
        </Button>
      </div>
      {Boolean(snapshot?.sessions.length) && (
        <Field label="历史会话">
          <select
            aria-label="AI 历史会话"
            value={session}
            disabled={Boolean(current) || Boolean(pending)}
            onChange={(e) => {
              if (e.target.value === session) return;
              localStorage.setItem("robot-arm:ai-session", e.target.value);
              setRuns([]);
              setSession(e.target.value);
            }}
          >
            <option value={session}>当前会话</option>
            {snapshot?.sessions
              .filter((v) => v.id !== session)
              .map((v) => (
                <option key={v.id} value={v.id}>
                  {v.title}
                </option>
              ))}
          </select>
        </Field>
      )}
      {images.length > 0 && (
        <div className="ai-images">
          {images.map((image, index) => (
            <div key={`${image.id}-${index}`}>
              <a
                href={`${endpoint}images/${image.id}/`}
                target="_blank"
                rel="noreferrer"
              >
                <Image
                  unoptimized
                  width={128}
                  height={96}
                  src={`${endpoint}images/${image.id}/`}
                  alt={`待发送图片 ${index + 1}`}
                />
              </a>
              <Button
                onClick={() =>
                  setImages((v) => v.filter((_, i) => i !== index))
                }
              >
                移除图片 {index + 1}
              </Button>
            </div>
          ))}
        </div>
      )}
      {error && (
        <p className="error" role="alert">
          {error}
        </p>
      )}
      <div
        className="ai-conversation"
        aria-label="AI 会话记录"
        aria-live="polite"
      >
        {runs.map((run) => (
          <article
            key={run.id}
            className="ai-run"
            data-run-id={run.id}
            data-run-state={run.state}
          >
            <p className="ai-user-text">{run.input}</p>
            {run.images.length > 0 && (
              <div className="ai-images">
                {run.images.map((id) => (
                  <a
                    key={id}
                    href={`${endpoint}images/${id}/`}
                    target="_blank"
                    rel="noreferrer"
                  >
                    <Image
                      unoptimized
                      width={128}
                      height={96}
                      src={`${endpoint}images/${id}/`}
                      alt="本次输入图片"
                    />
                  </a>
                ))}
              </div>
            )}
            <div className="ai-response">
              {run.text ||
                (activeRun(run) ? "等待模型响应或工具结果…" : "暂无文本回复")}
            </div>
            <p className="ai-run-status" aria-label="AI 状态">
              {labels[run.state]} ·{" "}
              {Math.round(
                ((run.endedAt ?? (activeRun(run) ? now : run.updatedAt)) -
                  run.startedAt) /
                  1000,
              )}{" "}
              秒
            </p>
            {activeRun(run) && (
              <KeyValue
                label="当前步骤"
                value={
                  run.calls.findLast((call) => !call.endedAt)?.name ??
                  (run.calls.length
                    ? `${run.calls.at(-1)!.name} 已结束，等待模型继续`
                    : "等待模型响应")
                }
              />
            )}
            {run.requests
              .filter((request) =>
                ["accepted", "planning", "executing"].includes(request.state),
              )
              .map((request) => (
                <KeyValue
                  key={request.id}
                  label={request.label}
                  value={`${request.state} · ${request.id}`}
                />
              ))}
            {run.error && (
              <p className="error" role="alert">
                {run.error}
              </p>
            )}
            <Disclosure
              title="工具与执行详情"
              englishTitle="模型回复与控制器终态分别记录；动作成功不等于已经通过图像确认抓持。"
            >
              <KeyValue
                label="模型 / 思考强度"
                value={`${run.settings.model} / ${run.settings.effort || "模型默认"}`}
              />
              <KeyValue
                label="服务回显思考强度"
                value={
                  run.reportedEfforts?.join(", ") ||
                  "未回显；不能据此确认档位生效"
                }
              />
              {run.requests.length > 0 && (
                <KeyValue
                  label="最近机器人请求"
                  value={`${run.requests.at(-1)!.label} · ${run.requests.at(-1)!.state}`}
                />
              )}
              <KeyValue label="运行编号" value={run.id} />
              <KeyValue
                label="服务返回模型"
                value={[...new Set(run.responseModels)].join(", ") || "—"}
              />
              {run.calls.map((call) => (
                <div className="ai-tool" key={call.id}>
                  <strong>
                    {call.name} ·{" "}
                    {call.endedAt
                      ? `${((call.endedAt - call.startedAt) / 1000).toFixed(1)} 秒`
                      : "执行中"}
                  </strong>
                  <details>
                    <summary>参数与结果</summary>
                    <pre>
                      {JSON.stringify(
                        {
                          input: call.input,
                          result: call.result,
                          error: call.error,
                        },
                        null,
                        2,
                      )}
                    </pre>
                  </details>
                </div>
              ))}
              {run.requests.map((r) => (
                <KeyValue
                  key={r.id}
                  label={`${r.label} · ${r.state}`}
                  value={r.id}
                />
              ))}
              {run.usage != null && (
                <details>
                  <summary>模型用量</summary>
                  <pre>{JSON.stringify(run.usage, null, 2)}</pre>
                </details>
              )}
              {run.warnings.length > 0 && (
                <p role="status">{run.warnings.join("\n")}</p>
              )}
            </Disclosure>
          </article>
        ))}
      </div>
    </div>
  );
}
