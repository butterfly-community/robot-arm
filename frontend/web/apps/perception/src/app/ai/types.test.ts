import { describe, expect, it } from "vitest";
import { mergeRuns, type AIRun } from "./types";

const record = (overrides: Partial<AIRun> = {}) =>
  ({
    id: "run",
    sessionId: "session",
    startedAt: 1,
    updatedAt: 1,
    state: "running",
    ...overrides,
  }) as AIRun;
describe("AI page merges HTTP and SSE by persisted revision", () => {
  it("does not let an old start response or snapshot erase the first retry", () => {
    const progress = record({
      updatedAt: 2,
      networkRetry: { attempt: 1, nextAt: 10002, error: "network" },
    });
    expect(mergeRuns([progress], [record()], "session")).toEqual([progress]);
    expect(mergeRuns([progress], [], "session")).toEqual([progress]);
  });
  it("retains pending and included updates even when task updatedAt did not change", () => {
    const pending = record({
      updates: [{ id: "u", receivedAt: 3, text: "reminder", images: [] }],
    });
    const included = record({
      updates: [{ ...pending.updates![0], includedAt: 4 }],
    });
    expect(mergeRuns([pending], [record()], "session")).toEqual([pending]);
    expect(mergeRuns([pending], [included], "session")).toEqual([included]);
    expect(mergeRuns([included], [pending], "session")).toEqual([included]);
  });
  it("does not resurrect terminal tasks and isolates sessions", () => {
    const finished = record({ state: "failed", updatedAt: 10 });
    expect(mergeRuns([finished], [record()], "session")).toEqual([finished]);
    expect(mergeRuns([finished], [], "other")).toEqual([]);
    expect(mergeRuns([], [finished], "other")).toEqual([]);
  });
});
