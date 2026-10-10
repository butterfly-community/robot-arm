"use client";

import { useEffect, useState } from "react";
import { Button, Disclosure, Field, Input, KeyValue } from "@robot/ui";
import { errorText, type AIExperience } from "./ai/types";

const endpoint = "/perception/api/ai/experiences/";
const assessments = {
  hypothesis: "AI 推测 · 待验证",
  supported: "AI 判断有观测支持 · 非人工验收",
  refuted: "AI 判断已被后续观测推翻",
};

export function AIExperiencePanel({ revision }: { revision: number }) {
  const [items, setItems] = useState<AIExperience[]>([]);
  const [query, setQuery] = useState("");
  const [refresh, setRefresh] = useState(0);
  const [loading, setLoading] = useState(true);
  const [deleting, setDeleting] = useState("");
  const [error, setError] = useState("");
  useEffect(() => {
    const controller = new AbortController();
    void (async () => {
      setLoading(true);
      try {
        const response = await fetch(endpoint, {
          cache: "no-store",
          signal: controller.signal,
        });
        const result = await response.json();
        if (!response.ok)
          throw Error(result.original_error ?? `HTTP ${response.status}`);
        if (!controller.signal.aborted) {
          setItems(result);
          setError("");
        }
      } catch (e) {
        if (!controller.signal.aborted) setError(errorText(e));
      } finally {
        if (!controller.signal.aborted) setLoading(false);
      }
    })();
    return () => controller.abort();
  }, [revision, refresh]);

  async function remove(id: string) {
    setDeleting(id);
    try {
      const response = await fetch(endpoint, {
        method: "DELETE",
        headers: { "content-type": "application/json" },
        body: JSON.stringify({ id }),
      });
      const result = await response.json();
      if (!response.ok)
        throw Error(result.original_error ?? `HTTP ${response.status}`);
      setItems((values) => values.filter((item) => item.id !== id));
      setRefresh((value) => value + 1);
      setError("");
    } catch (e) {
      setError(errorText(e));
    } finally {
      setDeleting("");
    }
  }

  const needle = query.trim().toLocaleLowerCase();
  const shown = items.filter(
    (item) =>
      !needle ||
      JSON.stringify([item.title, item.conditions, item.reflection, item.scope])
        .toLocaleLowerCase()
        .includes(needle),
  );
  return (
    <Disclosure
      title="任务经验"
      englishTitle="跨会话保存在 Redis；记录失败事实与 AI 经验判断，不代表模型训练或人工验收。"
    >
      <div className="card-stack">
        <p className="muted">
          新任务会读取相同机械臂、深度／纯视觉模式及反馈来源的经验目录。
          可删除错误经验；删除不抹除历史会话，也不能撤回当前任务已经读到的内容。
        </p>
        <Field label="查找经验">
          <Input
            aria-label="查找经验"
            value={query}
            onChange={(event) => setQuery(event.target.value)}
            placeholder="按物体、问题或机械臂查找"
          />
        </Field>
        <div className="card-actions card-actions-leading">
          <Button
            onClick={() => setRefresh((value) => value + 1)}
            disabled={loading}
          >
            刷新经验
          </Button>
          <span role="status">
            {loading
              ? "正在读取经验…"
              : `${shown.length} / ${items.length} 条经验`}
          </span>
        </div>
        {error && (
          <p className="error" role="alert">
            {error}
          </p>
        )}
        {!loading && !error && !shown.length && (
          <p>暂无匹配经验；失败事实和 AI 总结会在任务执行过程中保存。</p>
        )}
        {shown.map((item) => (
          <article className="ai-run" key={item.id}>
            <Disclosure title={item.title}>
              <div className="card-stack">
                <KeyValue
                  label="范围"
                  value={`${item.scope.modelRevision} · ${item.scope.mode === "vision" ? "纯视觉" : "深度"} · ${item.scope.feedbackSource}`}
                />
                <KeyValue
                  label="记录性质"
                  value={
                    item.reflection
                      ? assessments[item.reflection.assessment]
                      : "自动记录的失败事实 · 原因未判断"
                  }
                />
                <p>适用条件：{item.conditions}</p>
                {item.reflection && (
                  <>
                    <p>经验结论：{item.reflection.lesson}</p>
                    <p>下一步：{item.reflection.nextAction}</p>
                    <KeyValue label="总结模型" value={item.reflection.model} />
                  </>
                )}
                {item.failure && (
                  <details>
                    <summary>最近一次失败事实</summary>
                    <pre>{JSON.stringify(item.failure, null, 2)}</pre>
                  </details>
                )}
                <details>
                  <summary>来源任务与工具调用</summary>
                  <pre>{JSON.stringify(item.sources, null, 2)}</pre>
                </details>
                <KeyValue
                  label="更新时间"
                  value={new Date(item.updatedAt).toLocaleString()}
                />
                <div className="card-actions card-actions-leading">
                  <Button
                    aria-label={`删除经验：${item.title}`}
                    disabled={Boolean(deleting)}
                    onClick={() => void remove(item.id)}
                  >
                    {deleting === item.id ? "正在删除…" : "删除经验"}
                  </Button>
                </div>
              </div>
            </Disclosure>
          </article>
        ))}
      </div>
    </Disclosure>
  );
}
