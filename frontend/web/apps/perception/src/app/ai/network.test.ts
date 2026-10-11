import { afterEach, describe, expect, it, vi } from "vitest";
import { isLoopFinished, simulateReadableStream, streamText, tool } from "ai";
import { MockLanguageModelV4 } from "ai/test";
import { z } from "zod";
import {
  networkRecovery,
  networkRetries,
  resilientModel,
  transientNetworkError,
  type NetworkRetry,
} from "./network";

const finish = (reason: "stop" | "tool-calls") => ({
  type: "finish" as const,
  finishReason: { unified: reason, raw: reason },
  usage: {
    inputTokens: {
      total: 1,
      noCache: 1,
      cacheRead: undefined,
      cacheWrite: undefined,
    },
    outputTokens: { total: 1, text: 1, reasoning: undefined },
  },
});
describe("same-step network recovery", () => {
  afterEach(() => vi.useRealTimers());
  it("retries exactly 15 times at 10 second intervals, without an SDK multiplier", async () => {
    vi.useFakeTimers();
    const operation = vi.fn().mockRejectedValue(new TypeError("fetch failed"));
    const status = vi
      .fn<(status: NetworkRetry) => Promise<void>>()
      .mockResolvedValue(undefined);
    const done = networkRecovery(undefined, status)
      .run(operation)
      .catch((error) => error);
    await vi.advanceTimersByTimeAsync(0);
    expect(operation).toHaveBeenCalledTimes(1);
    await vi.advanceTimersByTimeAsync(9999);
    expect(operation).toHaveBeenCalledTimes(1);
    await vi.advanceTimersByTimeAsync(140001);
    expect(await done).toBeInstanceOf(TypeError);
    expect(operation).toHaveBeenCalledTimes(16);
    expect(status).toHaveBeenCalledTimes(15);
    expect(status.mock.calls.at(-1)?.[0]).toMatchObject({ attempt: 15 });
  });
  it("cancels the retry wait immediately and does not retry invalid requests", async () => {
    vi.useFakeTimers();
    const controller = new AbortController();
    const op = vi.fn().mockRejectedValue(new TypeError("fetch failed"));
    const done = networkRecovery(controller.signal)
      .run(op)
      .catch((e) => e);
    await vi.advanceTimersByTimeAsync(0);
    controller.abort();
    expect(await done).toMatchObject({ name: "AbortError" });
    expect(op).toHaveBeenCalledTimes(1);
    expect(vi.getTimerCount()).toBe(0);
    for (const error of [
      Error("invalid image"),
      { statusCode: 401 },
      { statusCode: 400 },
      { name: "AbortError" },
    ])
      expect(transientNetworkError(error)).toBe(false);
  });
  it("discards incomplete tool calls and retains already executed steps across stream retry", async () => {
    vi.useFakeTimers();
    const action = vi.fn(async () => "done");
    let request = 0;
    const model = new MockLanguageModelV4({
      doStream: async () => {
        request++;
        const chunks =
          request === 1
            ? [
                {
                  type: "tool-call" as const,
                  toolCallId: "first",
                  toolName: "move",
                  input: "{}",
                },
                finish("tool-calls"),
              ]
            : request === 2
              ? [
                  {
                    type: "tool-call" as const,
                    toolCallId: "discarded",
                    toolName: "move",
                    input: "{}",
                  },
                  {
                    type: "error" as const,
                    error: Error("stream disconnected before completion"),
                  },
                ]
              : [
                  { type: "text-start" as const, id: "ok" },
                  { type: "text-delta" as const, id: "ok", delta: "finished" },
                  { type: "text-end" as const, id: "ok" },
                  finish("stop"),
                ];
        return {
          stream: simulateReadableStream<(typeof chunks)[number]>({
            initialDelayInMs: null,
            chunkDelayInMs: null,
            chunks,
          }),
        };
      },
    });
    const controller = new AbortController();
    const onRetry = vi.fn(async () => {});
    const result = streamText({
      model,
      prompt: "test",
      maxRetries: 0,
      streamRetries: networkRetries,
      prepareStep: () => ({
        model: resilientModel(model, controller.signal, onRetry),
      }),
      tools: { move: tool({ inputSchema: z.object({}), execute: action }) },
      stopWhen: isLoopFinished(),
      abortSignal: controller.signal,
    });
    const completed = result.consumeStream();
    await vi.advanceTimersByTimeAsync(0);
    expect(action).toHaveBeenCalledTimes(1);
    expect(request).toBe(2);
    await vi.advanceTimersByTimeAsync(10000);
    await completed;
    expect(await result.text).toBe("finished");
    expect(action).toHaveBeenCalledTimes(1);
    expect(onRetry).toHaveBeenCalledTimes(1);
    expect(JSON.stringify((await result.response).messages)).not.toContain(
      "discarded",
    );
  });
  it.each(["error-event", "reader-error", "missing-finish"])(
    "never executes buffered tools when %s exhausts retries",
    async (failure) => {
      vi.useFakeTimers();
      const action = vi.fn(async () => "done");
      const model = new MockLanguageModelV4({
        doStream: async () => ({
          stream: new ReadableStream({
            start(c) {
              c.enqueue({
                type: "tool-call",
                toolCallId: "incomplete",
                toolName: "move",
                input: "{}",
              });
              if (failure === "reader-error") c.error(Error("socket closed"));
              else {
                if (failure === "error-event")
                  c.enqueue({ type: "error", error: Error("socket closed") });
                c.close();
              }
            },
          }),
        }),
      });
      const result = streamText({
        model: resilientModel(model, new AbortController().signal),
        prompt: "test",
        maxRetries: 0,
        streamRetries: networkRetries,
        tools: { move: tool({ inputSchema: z.object({}), execute: action }) },
      });
      const errors: unknown[] = [];
      const done = result.consumeStream({
        onError: (error) => errors.push(error),
      });
      await vi.advanceTimersByTimeAsync(150000);
      await done;
      expect(model.doStreamCalls).toHaveLength(16);
      expect(action).not.toHaveBeenCalled();
      expect(errors.length).toBeGreaterThan(0);
    },
  );
  it("shares one budget between connection and streaming failures", async () => {
    vi.useFakeTimers();
    let count = 0;
    const model = new MockLanguageModelV4({
      doStream: async () => {
        count++;
        if (count % 2) throw new TypeError("fetch failed");
        return {
          stream: simulateReadableStream({
            chunks: [{ type: "error" as const, error: Error("socket closed") }],
            initialDelayInMs: null,
            chunkDelayInMs: null,
          }),
        };
      },
    });
    const result = streamText({
      model: resilientModel(model, new AbortController().signal),
      prompt: "test",
      maxRetries: 0,
      streamRetries: networkRetries,
    });
    const done = result.consumeStream();
    await vi.advanceTimersByTimeAsync(150000);
    await done;
    expect(count).toBe(16);
  });
});
