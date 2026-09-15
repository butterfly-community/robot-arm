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
