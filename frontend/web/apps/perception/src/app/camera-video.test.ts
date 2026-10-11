import { describe, expect, it } from "vitest";
import { frameMetadata, videoChunk } from "./camera-video";

const frame = {
  codec: "h264",
  stream_id: 1,
  sequence: 10,
  keyframe: true,
  duration_ms: 1000 / 60,
  width: 1920,
  height: 1080,
};
describe("H.264 video contract", () => {
  it("batches ordered encoded frames without losing bytes or capture duration", () => {
    const packets = [16, 20, 16, 20, 16].map((duration, i) => ({
      video: new Uint8Array([0, 0, 0, 1, i]),
      duration,
      isLastVideoFrameComplete: true,
    }));
    const chunk = videoChunk(packets);
    expect([...chunk.video]).toEqual(packets.flatMap((p) => [...p.video]));
    expect(chunk.duration).toBe(88);
    expect(chunk.isLastVideoFrameComplete).toBe(true);
    expect(packets).toHaveLength(5);
  });
  it.each([
    [1280, 720],
    [1920, 1080],
  ])("retains %dx%d and fractional frame timing", (width, height) => {
    const value = { ...frame, width, height };
    expect(frameMetadata(JSON.stringify(value))).toEqual(value);
  });
  it.each([
    { codec: "rgb8" },
    { duration_ms: 0 },
    { width: 0 },
    { height: -1 },
    { stream_id: null },
    { sequence: 1.5 },
    { keyframe: null },
  ])("rejects invalid metadata %j", (invalid) => {
    expect(() =>
      frameMetadata(JSON.stringify({ ...frame, ...invalid })),
    ).toThrow("相机视频元数据无效");
  });
});
