"use client";

import { useState } from "react";
import { Button, Disclosure, Field, StatusBadge } from "@robot/ui";
import { ImageBoxCanvas } from "./image-box-canvas";

export type VisualPrompt = {
  kind: "visual";
  reference_image_base64: string;
  bboxes: number[][];
  class_ids: number[];
};

// Reference boxes describe this frozen RGB image, not later camera frames.
// They are model prompts, never output masks, world coordinates or robot goals.
export function VisualPromptEditor({
  imageUrl,
  classes,
  disabled,
  onSave,
}: {
  imageUrl: string;
  classes: string[];
  disabled: boolean;
  onSave: (prompt: VisualPrompt) => Promise<boolean>;
}) {
  const [reference, setReference] = useState<{
    url: string;
    width: number;
    height: number;
  }>();
  const [boxes, setBoxes] = useState<{ bbox: number[]; classId: number }[]>([]);
  const [classId, setClassId] = useState(0);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string>();
  const [saved, setSaved] = useState(false);
  const busy = disabled || loading;
  async function loadImage() {
    setLoading(true);
    setError(undefined);
    try {
      const response = await fetch(imageUrl, { cache: "no-store" });
      if (!response.ok) throw Error(`采集图读取失败：${response.status}`);
      const blob = await response.blob();
      const url = await new Promise<string>((resolve, reject) => {
        const reader = new FileReader();
        reader.onload = () => resolve(String(reader.result));
        reader.onerror = () => reject(Error("图像读取失败"));
        reader.readAsDataURL(blob);
      });
      const image = new window.Image();
      image.src = url;
      await image.decode();
      setReference({
        url,
        width: image.naturalWidth,
        height: image.naturalHeight,
      });
      setBoxes([]);
      setSaved(false);
    } catch (reason) {
      setError(String(reason));
    } finally {
      setLoading(false);
    }
  }
  return (
    <Disclosure
      title="视觉示例提示"
      englishTitle="先填写类别，载入最近一次采集的 RGB 图，再为类别框选示例。保存只更新模型提示；仍需运行感知后选择实际实例，不会直接运动。"
    >
      <div className="card-actions">
        <Button
          variant="outline"
          disabled={busy || !classes.length}
          onClick={loadImage}
        >
          {loading ? "正在载入…" : "载入采集图作为示例"}
        </Button>
      </div>
      {reference && (
        <>
          <Field label="示例类别">
            <select
              aria-label="示例类别"
              value={classId}
              disabled={busy}
              onChange={(event) => setClassId(Number(event.target.value))}
            >
              {classes.map((label, index) => (
                <option key={index} value={index}>
                  {label}
                </option>
              ))}
            </select>
          </Field>
          <ImageBoxCanvas
            key={reference.url}
            src={reference.url}
            width={reference.width}
            height={reference.height}
            label="视觉示例框选区域"
            disabled={busy || !classes[classId]}
            boxes={boxes.map((box, index) => ({
              id: String(index),
              bbox: box.bbox,
              label: classes[box.classId] ?? "",
            }))}
            onDraw={(bbox) => {
              setBoxes((previous) => [...previous, { bbox, classId }]);
              setSaved(false);
            }}
          />
          <div
            className="card-actions"
            style={{ marginTop: "1rem", flexWrap: "wrap" }}
          >
            <Button
              variant="outline"
              disabled={busy || !boxes.length}
              onClick={() => {
                setBoxes((previous) => previous.slice(0, -1));
                setSaved(false);
              }}
            >
              撤销最后一个示例
            </Button>
            <Button
              disabled={busy || !boxes.length}
              onClick={async () => {
                setSaved(
                  await onSave({
                    kind: "visual",
                    reference_image_base64: reference.url.split(",")[1],
                    bboxes: boxes.map((box) => box.bbox),
                    class_ids: boxes.map((box) => box.classId),
                  }),
                );
              }}
            >
              {disabled ? "正在保存…" : "保存视觉示例"}
            </Button>
            <StatusBadge>
              {saved ? "已保存视觉示例" : `${boxes.length} 个待保存示例`}
            </StatusBadge>
          </div>
        </>
      )}
      {error && <p role="alert">{error}</p>}
    </Disclosure>
  );
}
