import { describe, expect, it } from "vitest";
import type { ModelMessage } from "ai";
import { modelContext, requestSummary } from "./context";

const image = {
  type: "file" as const,
  mediaType: "image/png",
  data: { type: "data" as const, data: "fixture" },
};
function frame(id: string, role: string): ModelMessage {
  return {
    role: "tool",
    content: [
      {
        type: "tool-result",
        toolCallId: id,
        toolName: "observe_camera",
        output: {
          type: "content",
          value: [
            {
              type: "text",
              text: JSON.stringify({
                image_id: id,
                role,
                sequence: id,
                robot: { tcp: [1, 2, 3] },
              }),
            },
            image,
          ],
        },
      },
    ],
  };
}
describe("model image window without losing conversation history", () => {
  it("summarizes machine-generated failed-run payloads, retaining IDs and errors rather than duplicating results", () => {
    const requests = [
      {
        id: "failed-id",
        path: "/api/motion/request",
        label: "move",
        state: "failed",
        result: { original_error: "IK failed", value: { large: "payload" } },
      },
    ];
    const text = `运行已结束：failed。IK failed。原机器人请求：${JSON.stringify(requests)}。不得自动重放此前指令，后续只处理用户的新请求。`;
    const history: ModelMessage[] = [
      { role: "assistant", content: text },
      { role: "user", content: text },
    ];
    const result = modelContext(history);
    expect(result[0].content).toContain("failed-id");
    expect(result[0].content).toContain("IK failed");
    expect(result[0].content).not.toContain("payload");
    expect(result[1]).toEqual(history[1]);
    expect(history[0].content).toBe(text);
    expect(requestSummary(requests)).toEqual([
      {
        id: "failed-id",
        path: "/api/motion/request",
        label: "move",
        state: "failed",
        error: "IK failed",
      },
    ]);
  });
  it("projects old camera switch results to bindings without resending driver catalogs", () => {
    const history: ModelMessage[] = [
      {
        role: "tool",
        content: [
          {
            type: "tool-result",
            toolCallId: "switch",
            toolName: "set_camera_capture",
            output: {
              type: "text",
              value: JSON.stringify({
                request_id: "req",
                value: {
                  bindings: [{ role: "external", enabled: false }],
                  available_sources: [{ profiles: ["catalog"] }],
                },
              }),
            },
          },
        ],
      },
    ];
    const result = modelContext(history);
    expect(JSON.stringify(result)).toContain("external");
    expect(JSON.stringify(result)).toContain("req");
    expect(JSON.stringify(result)).not.toContain("catalog");
    expect(JSON.stringify(history)).toContain("catalog");
  });
  it("keeps before/after per camera, all text/results and reasoning, without mutating persistence", () => {
    const user: ModelMessage = {
      role: "user",
      content: [{ type: "text", text: "place at center" }, image],
    };
    const reasoning: ModelMessage = {
      role: "assistant",
      content: [
        {
          type: "reasoning",
          text: "",
          providerOptions: { openai: { reasoningEncryptedContent: "opaque" } },
        },
      ],
    };
    const history = [
      user,
      reasoning,
      frame("e1", "external"),
      frame("w1", "wrist"),
      frame("e2", "external"),
      frame("e3", "external"),
    ];
    const original = structuredClone(history);
    const result = modelContext(history);
    expect(history).toEqual(original);
    expect(result.slice(0, 2)).toEqual([user, reasoning]);
    expect(JSON.stringify(result[2])).not.toContain('"type":"file"');
    expect(JSON.stringify(result[2])).toContain("recall_image");
    expect(JSON.stringify(result[2])).toContain("e1");
    expect(JSON.stringify(result[2])).toContain("tcp");
    expect(result.slice(3)).toEqual(history.slice(3));
    expect(modelContext(result)).toEqual(result);
  });
  it("retains errors and deliberately recalled originals", () => {
    const recalled = frame("old", "external");
    if (recalled.role === "tool" && recalled.content[0].type === "tool-result")
      recalled.content[0].toolName = "recall_image";
    const failed: ModelMessage = {
      role: "tool",
      content: [
        {
          type: "tool-result",
          toolCallId: "failed",
          toolName: "move_joints",
          output: { type: "text", value: "IK failed" },
        },
      ],
    };
    const history = [
      recalled,
      failed,
      frame("a", "external"),
      frame("b", "external"),
    ];
    expect(modelContext(history)).toEqual(history);
  });
});
