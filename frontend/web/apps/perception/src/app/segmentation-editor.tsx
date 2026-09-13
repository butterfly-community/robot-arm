"use client";

import { useState } from "react";
import type {
  ManualRegion,
  PerceptionState,
  SegmentationEdit,
  WorldScene,
} from "@robot/contracts";
import { Button, Disclosure, Field, Input, StatusBadge } from "@robot/ui";
import { requestId, useDraftValue } from "@robot/gateway-client";
import { ImageBoxCanvas } from "./image-box-canvas";
import { SegmentationResults } from "./segmentation-results";
import { VisualPromptEditor, type VisualPrompt } from "./visual-prompt";

export function SegmentationEditor({
  state,
  scene,
  busy,
  canCapture,
  onEdit,
  onSave,
  onReconstruct,
}: {
  state?: PerceptionState;
  scene?: WorldScene;
  busy: boolean;
  canCapture: boolean;
  onEdit: (
    edit: SegmentationEdit,
    fields?: Record<string, unknown>,
  ) => Promise<boolean>;
  onReconstruct: () => Promise<unknown>;
  onSave: (fields: Record<string, unknown>) => Promise<boolean>;
}) {
  const [captureVersion, setCaptureVersion] = useState(0);
  return (
    <div
      className="segmentation-workspace"
      style={{ display: "grid", gap: "1rem", minWidth: 0 }}
    >
      <div className="card-actions" style={{ marginBottom: "1rem" }}>
        <Button
          variant="outline"
          disabled={busy || !canCapture}
          onClick={async () => {
            if (await onEdit({ kind: "capture" }))
              setCaptureVersion((v) => v + 1);
          }}
        >
          载入新分割帧
        </Button>
        <StatusBadge>
          {busy
            ? "正在处理…"
            : state?.last_segmentation_sequence == null
              ? "尚未载入"
              : `结果序号 ${state.last_segmentation_sequence}`}
        </StatusBadge>
      </div>
      {state && (
        <FrozenEditor
          state={state}
          scene={scene}
          captureVersion={captureVersion}
          busy={busy}
          onEdit={onEdit}
          onSave={onSave}
          onReconstruct={onReconstruct}
        />
      )}
    </div>
  );
}

function FrozenEditor({
  state,
  scene,
  captureVersion,
  busy,
  onEdit,
  onSave,
  onReconstruct,
}: {
  state: PerceptionState;
  scene?: WorldScene;
  captureVersion: number;
  busy: boolean;
  onEdit: (
    edit: SegmentationEdit,
    fields?: Record<string, unknown>,
  ) => Promise<boolean>;
  onReconstruct: () => Promise<unknown>;
  onSave: (fields: Record<string, unknown>) => Promise<boolean>;
}) {
  const [regions, setRegions] = useDraftValue<ManualRegion[]>(
    state.manual_regions ?? [],
  );
  const [name, setName] = useState("");
  const [lastCapture, setLastCapture] = useState(captureVersion);
  if (lastCapture !== captureVersion) {
    setLastCapture(captureVersion);
    setRegions(undefined);
  }
  const [classes, setClasses] = useDraftValue(state.classes.join(", "));
  const [placements, setPlacements] = useDraftValue(
    state.placement_labels.join(", "),
  );
  const [visualPrompt, setVisualPrompt] = useState<VisualPrompt>();
  const [promptKind, setPromptKind] = useDraftValue(
    state.visual_prompt_active ? "saved" : "text",
  );
  const [textModel, setTextModel] = useDraftValue(
    state.available_models.find((m) => !m.prompt_free)?.id ?? "",
  );
  const [autoModel, setAutoModel] = useDraftValue(
    state.available_models.find((m) => m.prompt_free)?.id ?? "",
  );
  const dirty =
    JSON.stringify(regions) !== JSON.stringify(state.manual_regions ?? []);
  const frame = state.segmentation_frame;
  const ready = state.last_segmentation_sequence != null && Boolean(frame);
  const reconstructionHint = busy
    ? "正在处理，请稍候"
    : dirty
      ? "请先应用手动标注，或撤销未应用标注"
      : !ready
        ? "请先载入新分割帧"
        : !state.calibrated
          ? "请先确认并应用当前相机的标定"
          : undefined;
  const split = (value: string) =>
    value
      .split(/[,，]/)
      .map((s) => s.trim())
      .filter(Boolean);
  const textFields = {
    model: textModel,
    classes: promptKind === "saved" ? undefined : split(classes),
    placement_labels: split(placements),
    prompt:
      promptKind === "saved"
        ? undefined
        : promptKind === "visual"
          ? visualPrompt
          : { kind: "text" },
  };
  const textReady = Boolean(
    textModel && classes.trim() && (promptKind !== "visual" || visualPrompt),
  );
  const clearSource = (source: string, label: string) => {
    const ids = state.instances
      .filter((i) => i.segmentation_source === source)
      .map((i) => i.instance_id);
    return (
      <Button
        variant="outline"
        disabled={busy || dirty || !ids.length}
        onClick={() => onEdit({ kind: "remove", instance_ids: ids })}
      >
        清除{label}结果
      </Button>
    );
  };
  const numbers = (bbox: number[]) => bbox.map(Math.round).join(" / ");
  const modelField = (automatic: boolean) => {
    const models = state.available_models.filter(
      (m) => m.prompt_free === automatic,
    );
    const model = automatic ? autoModel : textModel;
    return (
      <Field label={automatic ? "自动分割模型" : "提示词分割模型"}>
        <select
          aria-label={automatic ? "自动分割模型" : "提示词分割模型"}
          value={model}
          disabled={busy || !models.length}
          onChange={(e) =>
            (automatic ? setAutoModel : setTextModel)(e.target.value)
          }
        >
          {!models.length && <option value="">没有可用模型</option>}
          {models.map((m) => (
            <option key={m.id} value={m.id}>
              {m.label}
            </option>
          ))}
        </select>
        <div className="card-actions" style={{ marginTop: "0.75rem" }}>
          <Button
            variant="outline"
            disabled={busy || !ready || !model || (!automatic && !textReady)}
            onClick={() =>
              onEdit(
                { kind: "model" },
                {
                  model,
                  ...(!automatic
                    ? {
                        ...textFields,
                      }
                    : {}),
                },
              )
            }
          >
            {automatic ? "运行自动分割" : "运行提示词分割"}
          </Button>
          {clearSource(model, automatic ? "自动分割" : "提示词分割")}
        </div>
      </Field>
    );
  };
  return (
    <div style={{ display: "grid", gap: "1rem", minWidth: 0 }}>
      <Disclosure
        title="自动分割"
        englishTitle="自动模型始终可用，无需切换或启用。运行只替换该模型结果，保留提示词与手动结果；清除结果不会禁用模型。"
      >
        {modelField(true)}
      </Disclosure>
      <Disclosure
        title="提示词分割"
        englishTitle="可独立使用，也可与自动和手动结果组合。运行只处理当前冻结帧，保存配置不运行模型。"
      >
        <div style={{ display: "grid", gap: "1rem" }}>
          <Field
            label="识别与分割提示词"
            hint="逗号分隔的开放词汇类别，不绑定抓放场景。"
          >
            <Input
              aria-label="识别与分割提示词"
              value={classes}
              disabled={busy}
              onChange={(e) => setClasses(e.target.value)}
            />
          </Field>
          <Field label="放置区域角色" hint="可选的下游角色，不参与模型推理。">
            <Input
              aria-label="放置区域角色"
              value={placements}
              disabled={busy}
              onChange={(e) => setPlacements(e.target.value)}
            />
          </Field>
          <Field label="提示方式">
            <select
              aria-label="提示方式"
              value={promptKind}
              disabled={busy}
              onChange={(e) => setPromptKind(e.target.value)}
            >
              <option value="text">文字提示词</option>
              <option value="visual">视觉示例</option>
              {state.visual_prompt_active && (
                <option value="saved">已保存的视觉示例</option>
              )}
            </select>
          </Field>
          {promptKind === "visual" && (
            <VisualPromptEditor
              imageUrl={`/api/perception/assets/segmentation-color.png?v=${state.last_segmentation_sequence ?? 0}`}
              classes={split(classes)}
              disabled={busy || !ready}
              onSave={async (prompt) => {
                setVisualPrompt(prompt);
                return onSave({ ...textFields, prompt });
              }}
            />
          )}
          {modelField(false)}
          <div className="card-actions">
            <Button
              variant="outline"
              disabled={busy || !textReady}
              onClick={() => onSave(textFields)}
            >
              保存提示词配置
            </Button>
          </div>
        </div>
      </Disclosure>
      <Disclosure
        title="手动分割"
        englishTitle="无需运行分割模型。矩形内部是手动区域，不是精细轮廓；应用只替换手动来源。应用成功后可进行三维定位。"
      >
        <div style={{ display: "grid", gap: "1rem", minWidth: 0 }}>
          <Field
            label="新标注名称"
            hint="可先框选再修改名称。框内全部像素作为手动区域；尽量贴近物体边缘。"
          >
            <Input
              aria-label="新标注名称"
              value={name}
              onChange={(e) => setName(e.target.value)}
              disabled={busy || !ready}
              placeholder="例如：海绵、放置区"
            />
          </Field>
          {ready && frame ? (
            <ImageBoxCanvas
              src={`/api/perception/assets/segmentation-color.png?v=${state.last_segmentation_sequence}`}
              width={frame.width}
              height={frame.height}
              label="手动分割框选区域"
              disabled={busy}
              boxes={regions.map((r) => ({
                id: r.id,
                label: `${r.label} · 手动标注`,
                bbox: r.bounding_box_xyxy,
              }))}
              onDraw={(bbox) =>
                setRegions([
                  ...regions,
                  {
                    id: requestId(),
                    label: name.trim() || `区域 ${regions.length + 1}`,
                    bounding_box_xyxy: bbox,
                  },
                ])
              }
            />
          ) : (
            <div className="visual-empty">载入分割帧后即可框选</div>
          )}
          {regions.map((region, index) => (
            <div
              key={region.id}
              style={{
                display: "flex",
                flexWrap: "wrap",
                alignItems: "center",
                gap: "0.75rem",
              }}
            >
              <Input
                aria-label={`标注 ${index + 1} 名称`}
                value={region.label}
                disabled={busy}
                style={{ flex: "1 1 10rem", minWidth: 0 }}
                onChange={(e) =>
                  setRegions(
                    regions.map((r) =>
                      r.id === region.id ? { ...r, label: e.target.value } : r,
                    ),
                  )
                }
              />
              <span title={numbers(region.bounding_box_xyxy)}>
                {numbers(region.bounding_box_xyxy)}
              </span>
              <Button
                variant="outline"
                disabled={busy}
                onClick={() =>
                  setRegions(regions.filter((r) => r.id !== region.id))
                }
              >
                删除标注 {index + 1}
              </Button>
            </div>
          ))}
          <div className="card-actions" style={{ flexWrap: "wrap" }}>
            <Button
              disabled={
                busy || !ready || !dirty || regions.some((r) => !r.label.trim())
              }
              onClick={async () => {
                // A successful save acknowledges the submitted draft, even if
                // the server serializes its coordinates or object keys differently.
                // Editing is disabled while this request runs; failures keep it.
                if (await onEdit({ kind: "manual", regions }))
                  setRegions(undefined);
              }}
            >
              应用手动标注
            </Button>
            <Button
              variant="outline"
              disabled={busy || !dirty}
              onClick={() => setRegions(state.manual_regions ?? [])}
            >
              撤销未应用标注
            </Button>
            <StatusBadge>{dirty ? "有未应用标注" : "标注已同步"}</StatusBadge>
            {clearSource("manual", "手动分割")}
          </div>
        </div>
      </Disclosure>
      <Disclosure
        title="分割结果"
        englishTitle={`当前 ${state.instances.length} 条结果；原图保留框线与标题，点击标题查看二维与三维详情、来源或移除实例。`}
      >
        <SegmentationResults
          state={state}
          scene={scene}
          busy={busy || dirty}
          onRemove={(id) => onEdit({ kind: "remove", instance_ids: [id] })}
        />
      </Disclosure>
      <div className="card-actions">
        <Button
          variant="outline"
          disabled={Boolean(reconstructionHint)}
          aria-describedby={
            reconstructionHint ? "reconstruction-hint" : undefined
          }
          onClick={onReconstruct}
        >
          三维定位
        </Button>
        {reconstructionHint && (
          <span id="reconstruction-hint" className="muted" role="status">
            {reconstructionHint}
          </span>
        )}
      </div>
    </div>
  );
}
