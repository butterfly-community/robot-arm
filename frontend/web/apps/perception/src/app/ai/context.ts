import type { ModelMessage } from "ai";
import type { RobotRequest } from "./types";

export function requestSummary(requests: RobotRequest[]) {
  return requests.map(({ id, path, label, state, result }) => ({
    id,
    path,
    label,
    state,
    error: (result as { original_error?: string } | undefined)?.original_error,
  }));
}

function compactStatus(text: string) {
  const marker = "原机器人请求：";
  const suffix = "。不得自动重放此前指令，后续只处理用户的新请求。";
  if (!text.startsWith("运行已结束：") || !text.endsWith(suffix)) return text;
  const start = text.indexOf(marker);
  if (start < 0) return text;
  try {
    const requests = JSON.parse(
      text.slice(start + marker.length, -suffix.length),
    );
    return (
      text.slice(0, start + marker.length) +
      JSON.stringify(requestSummary(requests)) +
      suffix
    );
  } catch {
    return text;
  }
}

// Keep complete persisted history. Only the model input omits superseded camera
// pixels: retain before/after frames per view for estimating motion. User text,
// tool identities/outcomes and encrypted reasoning remain. Repeated machine
// status payloads are summarized below, not removed from storage. User reference
// images are never removed. Earlier originals remain available via recall_image.
export function modelContext(messages: ModelMessage[]): ModelMessage[] {
  const seen = new Map<string, number>();
  return messages
    .toReversed()
    .map((message) => {
      // Older failed runs embedded entire request payloads in this generated
      // status message. Their original tool results already exist in history.
      if (message.role === "assistant" && typeof message.content === "string")
        return { ...message, content: compactStatus(message.content) };
      if (message.role !== "tool") return message;
      return {
        ...message,
        content: message.content
          .toReversed()
          .map((part) => {
            if (
              part.type === "tool-result" &&
              part.toolName === "set_camera_capture" &&
              part.output.type === "text"
            ) {
              try {
                const value = JSON.parse(part.output.value);
                if (value.value?.bindings)
                  return {
                    ...part,
                    output: {
                      type: "text" as const,
                      value: JSON.stringify({
                        request_id: value.request_id,
                        bindings: value.value.bindings,
                      }),
                    },
                  };
              } catch {
                /* Keep unstructured failures verbatim. */
              }
            }
            if (part.type !== "tool-result" || part.output.type !== "content")
              return part;
            const text = part.output.value.find((v) => v.type === "text");
            let metadata: { image_id?: string; role?: string };
            try {
              metadata = JSON.parse(text?.text ?? "null") ?? {};
            } catch {
              return part;
            }
            if (!metadata.image_id || part.toolName === "recall_image")
              return part;
            const key = metadata.role ?? part.toolName;
            const count = seen.get(key) ?? 0;
            seen.set(key, count + 1);
            if (count < (metadata.role ? 2 : 1)) return part;
            return {
              ...part,
              output: {
                ...part.output,
                value: part.output.value.map((v) =>
                  v.type === "file" && v.mediaType.startsWith("image/")
                    ? {
                        type: "text" as const,
                        text: `较早图像 ${metadata.image_id} 已被同视角新帧替代；需核对原图时用 recall_image。`,
                      }
                    : v,
                ),
              },
            };
          })
          .toReversed(),
      };
    })
    .toReversed();
}
