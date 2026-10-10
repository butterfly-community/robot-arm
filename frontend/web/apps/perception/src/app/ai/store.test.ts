import { beforeEach, describe, expect, it, vi } from "vitest";
import { resolve } from "node:path";
import { randomBytes } from "node:crypto";
import { stat } from "node:fs/promises";
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
import {
  getImage,
  imagePart,
  messages,
  putImage,
  appendMessages,
} from "./store";
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
    await appendMessages("test", history);
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
  it("appends new turns without rewriting old images or dropping complete history", async () => {
    const bytes = await sharp({
      create: { width: 2, height: 2, channels: 3, background: "red" },
    })
      .png()
      .toBuffer();
    const image = await putImage(bytes);
    const first: ModelMessage = {
      role: "user",
      content: [await imagePart(image.id)],
    };
    await appendMessages("append", [first]);
    const path = resolve(process.env.AI_DATA_DIR!, "images", image.id);
    const before = await stat(path);
    const reply: ModelMessage = { role: "assistant", content: "observed" };
    await appendMessages("append", [reply]);
    expect(await messages("append")).toEqual([first, reply]);
    expect((await stat(path)).ino).toBe(before.ino);
    // Error/cancellation status must persist even when a prior image is missing.
    data.delete("robot-arm:ai:image:" + image.id);
    await appendMessages("append", [
      { role: "assistant", content: "cancelled" },
    ]);
    const persisted = JSON.parse(
      data.get("robot-arm:ai:session:append:messages")!,
    );
    expect(persisted).toHaveLength(3);
    expect(JSON.stringify(persisted[0])).toContain(`robot-image:${image.id}`);
  });
  it("projects camera history before loading pixels, without modifying stored messages", async () => {
    const frames: ModelMessage[] = [];
    const ids: string[] = [];
    for (const color of ["red", "green", "blue"]) {
      const image = await putImage(
        await sharp({
          create: { width: 2, height: 2, channels: 3, background: color },
        })
          .png()
          .toBuffer(),
      );
      ids.push(image.id);
      frames.push({
        role: "tool",
        content: [
          {
            type: "tool-result",
            toolCallId: color,
            toolName: "observe_camera",
            output: {
              type: "content",
              value: [
                {
                  type: "text",
                  text: JSON.stringify({
                    image_id: image.id,
                    role: "external",
                  }),
                },
                await imagePart(image.id),
              ],
            },
          },
        ],
      });
    }
    await appendMessages("projection", frames);
    const original = data.get("robot-arm:ai:session:projection:messages");
    data.delete("robot-arm:ai:image:" + ids[0]);
    const projected = await messages("projection");
    expect(JSON.stringify(projected[0])).toContain("recall_image");
    expect(projected.slice(1)).toEqual(frames.slice(1));
    expect(data.get("robot-arm:ai:session:projection:messages")).toBe(original);
  });
  it("never publishes partial large images during concurrent history saves and reads", async () => {
    const bytes = await sharp(randomBytes(1920 * 1080 * 3), {
      raw: { width: 1920, height: 1080, channels: 3 },
    })
      .png()
      .toBuffer();
    expect(bytes.length).toBeGreaterThan(512 * 1024);
    const image = await putImage(bytes);
    await Promise.all([
      ...Array.from({ length: 8 }, () => putImage(bytes)),
      ...Array.from({ length: 8 }, async () => {
        for (let i = 0; i < 8; i++) {
          expect((await getImage(image.id)).bytes.equals(bytes)).toBe(true);
        }
      }),
    ]);
    const history: ModelMessage[] = [
      { role: "user", content: [await imagePart(image.id)] },
    ];
    await appendMessages("large", history);
    expect(await messages("large")).toEqual(history);
  });
});
