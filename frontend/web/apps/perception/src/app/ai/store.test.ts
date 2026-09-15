import { beforeEach, describe, expect, it, vi } from "vitest";
import { resolve } from "node:path";
import sharp from "sharp";
import type { ModelMessage } from "ai";
const data = vi.hoisted(() => new Map<string, string>());
vi.mock("redis", () => ({
  createClient: () => ({
    on: vi.fn(),
    connect: async () => {},
    get: async (key: string) => data.get(key),
    set: async (key: string, value: string) => {
      data.set(key, value);
    },
  }),
}));
import { getImage, imagePart, messages, putImage, saveMessages } from "./store";
describe("complete local Responses context", () => {
  beforeEach(() => {
    data.clear();
    process.env.AI_DATA_DIR = resolve(process.cwd(), "../temp/ai-unit-images");
  });
  it("preserves RGB pixels, dimensions and content-addressed identity through PNG upload", async () => {
    const rgb = Buffer.from([255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0]);
    const bytes = await sharp(rgb, {
      raw: { width: 2, height: 2, channels: 3 },
    })
      .png()
      .toBuffer();
    const image = await putImage(bytes);
    expect(image).toMatchObject({
      width: 2,
      height: 2,
      mediaType: "image/png",
    });
    expect(await putImage(bytes)).toEqual(image);
    expect(
      await sharp((await getImage(image.id)).bytes)
        .raw()
        .toBuffer(),
    ).toEqual(rgb);
  });
  it("externalizes images while preserving tool IDs, encrypted reasoning and full context", async () => {
    const bytes = await sharp({
      create: { width: 1280, height: 720, channels: 3, background: "#ff0000" },
    })
      .png()
      .toBuffer();
    const image = await putImage(bytes),
      part = await imagePart(image.id);
    const history: ModelMessage[] = [
      { role: "user", content: [{ type: "text", text: "test" }, part] },
      {
        role: "assistant",
        content: [
          {
            type: "reasoning",
            text: "",
            providerOptions: { openai: { encryptedContent: "opaque" } },
          },
          {
            type: "tool-call",
            toolCallId: "call",
            toolName: "observe",
            input: {},
          },
        ],
      },
      {
        role: "tool",
        content: [
          {
            type: "tool-result",
            toolCallId: "call",
            toolName: "observe",
            output: { type: "content", value: [part] },
          },
        ],
      },
    ];
    await saveMessages("test", history);
    const persisted = data.get("robot-arm:ai:session:test:messages")!;
    expect(persisted).toContain("robot-image:");
    expect(persisted).not.toContain(bytes.toString("base64"));
    expect(await messages("test")).toEqual(history);
    data.delete("robot-arm:ai:image:" + image.id);
    await expect(messages("test")).rejects.toThrow("历史图片不存在");
  });
  it("retains original JPEG bytes rather than a second lossy encode", async () => {
    const bytes = await sharp({
      create: { width: 1920, height: 1080, channels: 3, background: "#0000ff" },
    })
      .jpeg()
      .toBuffer();
    const image = await putImage(bytes);
    expect(image).toMatchObject({
      width: 1920,
      height: 1080,
      mediaType: "image/jpeg",
    });
    expect((await getImage(image.id)).bytes).toEqual(bytes);
  });
});
