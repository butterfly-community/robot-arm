import { z } from "zod";

export const settingsSchema = z.object({
  baseURL: z.url().transform((v) => v.replace(/\/$/, "")),
  model: z.string().trim().min(1),
  effort: z.string().trim(),
});
export type AISettings = z.infer<typeof settingsSchema>;
export const idSchema = z.string().regex(/^[A-Za-z0-9_-]+$/);
export const startSchema = z.object({
  sessionId: idSchema,
  runId: idSchema,
  text: z.string().trim().min(1),
  images: z.array(z.string().regex(/^[a-f0-9]{64}$/)).default([]),
});
export type AIState =
  "running" | "stopping" | "succeeded" | "failed" | "cancelled" | "interrupted";
export interface AIImage {
  id: string;
  mediaType: string;
  width: number;
  height: number;
}
export interface AICall {
  id: string;
  name: string;
  input: unknown;
  startedAt: number;
  endedAt?: number;
  result?: unknown;
  error?: string;
}
export interface RobotRequest {
  id: string;
  path: string;
  label: string;
  state: string;
  result?: unknown;
}
export interface ExperienceScope {
  modelRevision: string;
  mode: "depth" | "vision";
  feedbackSource: string;
}
export const experienceInputSchema = z.object({
  id: idSchema.nullable().describe("更新已有经验时填编号；新增填 null"),
  title: z.string().trim().min(1),
  conditions: z
    .string()
    .trim()
    .min(1)
    .describe("适用的物体、相机布局、姿态等条件"),
  lesson: z.string().trim().min(1).describe("公开的经验结论，不是内部推理"),
  nextAction: z.string().trim().min(1).describe("下次应如何调整或验证"),
  assessment: z.enum(["hypothesis", "supported", "refuted"]),
  evidenceCallIds: z
    .array(z.string())
    .describe("本次任务中已完成的相关工具调用编号；用户纠正可为空"),
});
export interface AIExperience {
  id: string;
  scope: ExperienceScope;
  title: string;
  conditions: string;
  // Facts are captured by the application, assessments are attributed to the AI.
  failure?: {
    tool: string;
    input: unknown;
    error: string;
    runId: string;
    callId: string;
  };
  reflection?: {
    lesson: string;
    nextAction: string;
    assessment: "hypothesis" | "supported" | "refuted";
    model: string;
  };
  sources: { runId: string; sessionId: string; callIds: string[] }[];
  createdAt: number;
  updatedAt: number;
}
export interface AIRun {
  id: string;
  sessionId: string;
  owner: string;
  settings: AISettings;
  state: AIState;
  input: string;
  images: string[];
  text: string;
  error?: string;
  startedAt: number;
  updatedAt: number;
  endedAt?: number;
  firstTokenAt?: number;
  calls: AICall[];
  requests: RobotRequest[];
  responseModels: string[];
  responseIds: string[];
  reportedEfforts?: string[];
  usage?: unknown;
  warnings: string[];
  promptMode?: "depth" | "vision";
  experienceScope?: ExperienceScope;
  steps?: {
    responseId?: string;
    responseMs: number;
    stepMs: number;
    inputTokens?: number;
    outputTokens?: number;
  }[];
}
export interface CapabilityCheck {
  baseURL: string;
  model: string;
  effort: string;
  checkedAt: number;
  text: boolean;
  vision: boolean;
  tools: boolean;
  streaming: boolean;
  actualModel?: string;
  reportedEffort?: string;
  effortNote?: string;
  error?: string;
}
export interface AISnapshot {
  settings: AISettings;
  keyConfigured: boolean;
  runs: AIRun[];
  checks: CapabilityCheck[];
  sessions: { id: string; title: string; updatedAt: number }[];
}
export const activeRun = (run: AIRun) =>
  run.state === "running" || run.state === "stopping";
export const errorText = (error: unknown) =>
  error instanceof Error ? error.message : String(error);
