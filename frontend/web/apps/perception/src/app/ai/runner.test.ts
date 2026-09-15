import { describe, expect, it, vi } from "vitest";
import { TypeValidationError } from "ai";
import { providerOptions, publicError, responseEffort } from "./runner";
import { settingsSchema, startSchema } from "./types";
vi.mock("./store", () => ({}));
describe("Responses configuration", () => {
  it("shows the provider's failed-response error without hiding it behind an SDK schema dump", () => {
    const failure = new TypeValidationError({
      value: {
        type: "response.failed",
        response: {
          id: "resp-fixture",
          error: { code: "upstream_error", message: "Upstream request failed" },
        },
      },
      cause: Error("missing stream fields"),
    });
    expect(publicError(failure)).toBe(
      "模型服务 upstream_error: Upstream request failed（resp-fixture）",
    );
    const unrelated = new TypeValidationError({
      value: { type: "unexpected" },
      cause: Error("invalid"),
    });
    expect(publicError(unrelated)).toContain("Type validation failed");
  });
  it("reads effort only from actual SDK-decoded response metadata, never from the requested value", () => {
    expect(
      responseEffort({
        type: "response.completed",
        response: { reasoning: { effort: "low" } },
      }),
    ).toBe("low");
    expect(
      responseEffort({ type: "response.created", response: {} }),
    ).toBeUndefined();
    expect(responseEffort({ reasoning: { effort: "high" } })).toBeUndefined();
  });
  it("uses local context without hosted persistence or parallel robot calls", () => {
    expect(
      providerOptions({
        baseURL: "https://example.com/v1",
        model: "test",
        effort: "",
      }),
    ).toEqual({
      openai: {
        store: false,
        parallelToolCalls: false,
        include: ["reasoning.encrypted_content"],
      },
    });
    expect(
      providerOptions({
        baseURL: "https://example.com/v1",
        model: "test",
        effort: "high",
      }).openai.reasoningEffort,
    ).toBe("high");
  });
  it("does not return the credential in provider errors", () => {
    vi.stubEnv("AI_API_KEY", "secret-fixture");
    expect(publicError(Error("token secret-fixture rejected"))).toBe(
      "token [redacted] rejected",
    );
    vi.unstubAllEnvs();
  });
  it("validates settings and references, not physical motion thresholds", () => {
    expect(
      settingsSchema.parse({
        baseURL: "https://example.com/v1/",
        model: " test ",
        effort: "",
      }).model,
    ).toBe("test");
    expect(
      startSchema.safeParse({
        sessionId: "../secret",
        runId: "run",
        text: "test",
      }).success,
    ).toBe(false);
    expect(
      startSchema.parse({ sessionId: "session", runId: "run", text: "test" })
        .images,
    ).toEqual([]);
  });
});
