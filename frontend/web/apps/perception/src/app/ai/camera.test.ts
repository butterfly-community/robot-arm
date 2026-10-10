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
    const fetcher = vi.fn(async (url: string) =>
      url.endsWith("/api/perception/state")
        ? Response.json({ values: { camera_state: { bindings: [] } } })
        : new Response("相机未绑定或没有图像信号", { status: 503 }),
    );
    vi.stubGlobal("fetch", fetcher);
    await expect(cameraImage("external")).rejects.toThrow(
      "相机未绑定或没有图像信号",
    );
    expect(fetcher).toHaveBeenCalledTimes(2);
  });
  it("waits for the first frame after opening without reopening or using an old frame", async () => {
    let snapshots = 0;
    const fetcher = vi.fn(async (url: string) => {
      if (url.endsWith("/api/perception/state"))
        return Response.json({
          values: {
            camera_state: {
              bindings: [
                {
                  role: "wrist",
                  enabled: true,
                  streaming: true,
                  has_signal: false,
                  original_error: null,
                },
              ],
            },
          },
        });
      if (++snapshots === 1)
        return new Response("相机未绑定或没有图像信号", { status: 503 });
      return new Response(new Uint8Array([4, 5, 6]), {
        headers: {
          "x-camera-frame": JSON.stringify({
            role: "wrist",
            source_id: "same-camera",
            sequence: 1,
          }),
        },
      });
    });
    vi.stubGlobal("fetch", fetcher);
    const result = await cameraImage("wrist");
    expect(result.bytes).toEqual(Buffer.from([4, 5, 6]));
    expect(result.metadata.sequence).toBe(1);
    expect(snapshots).toBe(2);
    expect(
      fetcher.mock.calls.every(
        ([url]) =>
          url.endsWith("/snapshot?role=wrist") ||
          url.endsWith("/api/perception/state"),
      ),
    ).toBe(true);
  });
  it("does not retry a failed driver or ignore cancellation", async () => {
    const fetcher = vi.fn(async (url: string) =>
      url.endsWith("/api/perception/state")
        ? Response.json({
            values: {
              camera_state: {
                bindings: [
                  {
                    role: "wrist",
                    enabled: true,
                    streaming: true,
                    original_error: "USB error",
                  },
                ],
              },
            },
          })
        : new Response("no signal", { status: 503 }),
    );
    vi.stubGlobal("fetch", fetcher);
    await expect(cameraImage("wrist")).rejects.toThrow("no signal");
    expect(fetcher).toHaveBeenCalledTimes(2);
    const controller = new AbortController();
    controller.abort();
    await expect(cameraImage("wrist", controller.signal)).rejects.toThrow();
    expect(fetcher).toHaveBeenCalledTimes(2);
  });
});
