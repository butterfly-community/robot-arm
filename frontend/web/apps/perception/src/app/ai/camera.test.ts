import { afterEach, describe, expect, it, vi } from "vitest";
import { cameraImage } from "./robot";
afterEach(() => vi.unstubAllGlobals());
describe("AI camera frames", () => {
  it("uses the role endpoint and its atomic frame metadata", async () => {
    const signal = new AbortController().signal;
    const fetcher = vi.fn(
      async () =>
        new Response(new Uint8Array([1, 2, 3]), {
          headers: {
            "x-camera-frame": JSON.stringify({
              role: "wrist",
              source_id: "v4l2:serial",
              sequence: 3,
              received_time_ns: 456,
              width: 1,
              height: 1,
              pixel_format: "rgb8",
            }),
          },
        }),
    );
    vi.stubGlobal("fetch", fetcher);
    const result = await cameraImage("wrist", signal);
    expect(fetcher).toHaveBeenCalledWith(
      expect.stringContaining("/snapshot?role=wrist"),
      { cache: "no-store", signal },
    );
    expect(result.metadata.source_id).toBe("v4l2:serial");
    expect(result.bytes).toEqual(Buffer.from([1, 2, 3]));
  });
  it("does not use another camera or stale perception asset when there is no signal", async () => {
    const fetcher = vi.fn(
      async () => new Response("相机未绑定或没有图像信号", { status: 503 }),
    );
    vi.stubGlobal("fetch", fetcher);
    await expect(cameraImage("external")).rejects.toThrow(
      "相机未绑定或没有图像信号",
    );
    expect(fetcher).toHaveBeenCalledTimes(1);
  });
});
