import type { PerceptionState } from "@robot/contracts";

export function segmentationSource(
  source: string | undefined,
  models: PerceptionState["available_models"],
) {
  if (source === "manual") return "手动标注";
  const model = models.find((item) => item.id === source);
  return model
    ? `${model.prompt_free ? "自动" : "提示词"} · ${model.label}`
    : source || "来源未知";
}
