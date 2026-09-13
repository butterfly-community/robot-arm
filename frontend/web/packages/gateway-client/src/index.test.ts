import { afterEach, expect, it, vi } from "vitest";
import { post } from "./index";

afterEach(() => vi.unstubAllGlobals());

it("displays the gateway error rather than the full failed task JSON", async () => {
  vi.stubGlobal(
    "fetch",
    vi.fn().mockResolvedValue(
      new Response(
        JSON.stringify({
          request_id: "current",
          value: { state: "failed" },
          original_error: "场景已改变（请求 6，当前 7）",
        }),
        { status: 400 },
      ),
    ),
  );
  await expect(post("/api/perception/pick-place", {})).rejects.toThrow(
    "场景已改变（请求 6，当前 7）",
  );
});

it("preserves non-JSON proxy errors", async () => {
  vi.stubGlobal(
    "fetch",
    vi
      .fn()
      .mockResolvedValue(new Response("upstream unavailable", { status: 502 })),
  );
  await expect(post("/api/perception/pick-place", {})).rejects.toThrow(
    "upstream unavailable",
  );
});
