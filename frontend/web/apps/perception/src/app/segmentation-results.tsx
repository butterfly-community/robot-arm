"use client";

import { useEffect, useRef, useState } from "react";
import type { PerceptionState, WorldScene } from "@robot/contracts";
import { Button, KeyValue } from "@robot/ui";
import { ImageBoxCanvas } from "./image-box-canvas";
import { segmentationSource } from "./segmentation-source";

export function SegmentationResults({
  state,
  scene,
  busy,
  onRemove,
}: {
  state: PerceptionState;
  scene?: WorldScene;
  busy: boolean;
  onRemove: (id: string) => Promise<unknown>;
}) {
  const [selection, setSelection] = useState<{
    id: string;
    sequence: number;
  }>();
  const dialog = useRef<HTMLDialogElement>(null);
  const selected =
    selection?.sequence === state.last_segmentation_sequence
      ? state.instances.find((i) => i.instance_id === selection?.id)
      : undefined;
  const open = Boolean(selected);
  useEffect(() => {
    if (open) dialog.current?.showModal();
    else dialog.current?.close();
  }, [open]);
  const frame = state.segmentation_frame;
  // Never present the previous reconstruction as belonging to a new segmentation.
  const currentScene =
    scene?.sequence === state.last_scene_sequence ? scene : undefined;
  const placements =
    currentScene?.placement_regions.filter(
      (p) => p.source_object_id === selected?.instance_id,
    ) ?? [];
  const numbers = (value?: number[] | null) =>
    value?.join(" / ") ?? "尚未三维定位";
  return (
    <>
      {frame && state.last_segmentation_sequence != null ? (
        <ImageBoxCanvas
          src={`/api/perception/assets/segmentation-color.png?v=${state.last_segmentation_sequence}`}
          width={frame.width}
          height={frame.height}
          label="全部分割结果图"
          disabled
          boxes={state.instances.map((item) => ({
            id: item.instance_id,
            label: `${item.label} · ${segmentationSource(item.segmentation_source, state.available_models)}`,
            bbox: item.bounding_box_xyxy,
          }))}
          onSelect={(id) => {
            setSelection({ id, sequence: state.last_segmentation_sequence! });
          }}
        />
      ) : (
        <div className="visual-empty">载入分割帧后显示全部结果</div>
      )}
      <div style={{ marginTop: "var(--space-section)" }}>
        <KeyValue
          label="场景坐标系"
          value={currentScene?.frame_id ?? "尚未三维定位"}
        />
        <KeyValue
          label="物体 / 放置区 / 显式障碍"
          value={
            currentScene
              ? `${currentScene.objects.length} / ${currentScene.placement_regions.length} / ${currentScene.obstacles.length}`
              : "— / — / —"
          }
        />
      </div>
      <dialog
        ref={dialog}
        className="segmentation-detail"
        aria-labelledby="segmentation-detail-title"
        onClose={() => {
          setSelection(undefined);
        }}
      >
        <h3 id="segmentation-detail-title">{selected?.label ?? "分割详情"}</h3>
        {selected && (
          <>
            <KeyValue label="实例编号" value={selected.instance_id} />
            <KeyValue
              label="来源"
              value={segmentationSource(
                selected.segmentation_source,
                state.available_models,
              )}
            />
            <KeyValue
              label="置信度"
              value={
                selected.segmentation_source === "manual"
                  ? "手动标注（非模型置信度）"
                  : `${(selected.confidence * 100).toFixed(1)}%`
              }
            />
            <KeyValue
              label="二维框 x1/y1/x2/y2"
              value={numbers(selected.bounding_box_xyxy)}
            />
            <KeyValue
              label="三维中心 · m"
              value={numbers(selected.position_m)}
            />
            <KeyValue
              label="场景坐标系"
              value={currentScene?.frame_id ?? "尚未三维定位"}
            />
            <KeyValue label="尺寸 · m" value={numbers(selected.size_m)} />
            <KeyValue
              label="抓取候选"
              value={String(selected.grasp_candidate_count)}
            />
            {placements.map((p) => (
              <KeyValue
                key={p.region_id}
                label={`放置区 · ${p.label}`}
                value={`中心 ${numbers(p.pose.position_m)} m · 尺寸 ${numbers(p.size_m)} m`}
              />
            ))}
            <div className="card-actions">
              <Button
                variant="outline"
                disabled={busy}
                aria-label={`移除结果 ${selected.label}`}
                onClick={() => onRemove(selected.instance_id)}
              >
                移除此结果
              </Button>
              <Button variant="outline" onClick={() => dialog.current?.close()}>
                关闭详情
              </Button>
            </div>
          </>
        )}
      </dialog>
    </>
  );
}
