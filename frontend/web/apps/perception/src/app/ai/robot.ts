import {
  schemaVersion,
  type PerceptionState,
  type WorldScene,
  type RobotModelInfo,
  type RequestRecord,
  type CameraCaptureState,
} from "@robot/contracts";
import { randomUUID } from "node:crypto";
import type { AIRun } from "./types";
import { saveRun } from "./store";

const base = () =>
  process.env.GATEWAY_INTERNAL_URL?.replace(/\/$/, "") ??
  "http://web-gateway:8080";
export async function gateway(path: string, body?: Record<string, unknown>) {
  const response = await fetch(base() + path, {
    method: body ? "POST" : "GET",
    cache: "no-store",
    ...(body
      ? {
          headers: { "content-type": "application/json" },
          body: JSON.stringify(body),
        }
      : {}),
  });
  const result = await response.json();
  if (!response.ok || typeof result.original_error === "string")
    throw Error(result.original_error ?? `机器人服务 HTTP ${response.status}`);
  return result;
}
export async function imageBytes(key: "color.png" | "segmentation-color.png") {
  const response = await fetch(`${base()}/api/perception/assets/${key}`, {
    cache: "no-store",
  });
  if (!response.ok) throw Error(`相机图像读取失败：HTTP ${response.status}`);
  return Buffer.from(await response.arrayBuffer());
}
export async function perceptionSnapshot(): Promise<{
  values: {
    perception_state: PerceptionState;
    world_scene?: WorldScene;
    camera_state?: CameraCaptureState;
  };
}> {
  return gateway("/api/perception/state");
}
export async function scene(
  expectedSequence?: number | null,
  signal?: AbortSignal,
) {
  for (;;) {
    signal?.throwIfAborted();
    const value = (await perceptionSnapshot()).values.world_scene;
    if (expectedSequence == null) {
      if (!value) throw Error("尚无三维场景，请先分割和定位");
      return value;
    }
    if (value?.sequence === expectedSequence) return value;
    if (value && value.sequence > expectedSequence)
      throw Error("场景已更新，请读取当前场景重新选择目标");
    // Request results and scene snapshots travel as separate messages. Wait
    // for the acknowledged version, not an arbitrary delay or a nonempty scene.
    await new Promise((resolve) => setTimeout(resolve, 500));
  }
}
export function compactScene(value: WorldScene) {
  return {
    ...value,
    point_cloud: undefined,
    objects: value.objects.map(({ grasp_candidates, ...object }) => ({
      ...object,
      grasp_candidate_count: grasp_candidates.length,
    })),
  };
}
export function compactCamera(value?: CameraCaptureState) {
  if (!value) return value;
  return {
    ...value,
    available_sources: value.available_sources.map(
      ({ profiles, driver_extensions, ...source }) => ({
        ...source,
        profile_count: profiles.length,
        driver_extension_count: driver_extensions.length,
      }),
    ),
  };
}
export async function robotModel(): Promise<RobotModelInfo> {
  const value = (await gateway("/api/motion/state")).values.robot_model_info;
  if (!value) throw Error("机械臂模型尚未就绪");
  return value;
}
export function namedTargetRequest(model: RobotModelInfo, key: string) {
  const target = model.named_targets.find((target) => target.key === key);
  if (!target) throw Error(`当前模型没有命名姿态 ${key}`);
  return {
    action: "apply",
    model_revision: model.model_revision,
    joints: Object.entries(target.joint_positions_rad).map(
      ([joint_key, position_rad]) => ({ joint_key, position_rad }),
    ),
    actuators: Object.entries(target.actuator_positions_rad).map(
      ([actuator_key, position_rad]) => ({ actuator_key, position_rad }),
    ),
    options: {},
  };
}
export function tcpRequest(
  model: RobotModelInfo,
  relative: boolean,
  frame: string,
  position_m: number[],
  orientation_xyzw: number[],
) {
  return {
    action: "apply",
    model_revision: model.model_revision,
    joints: [],
    actuators: [],
    options: {},
    tcp_target: { relative, pose: { frame, position_m, orientation_xyzw } },
  };
}
export function cancellable(path: string) {
  return [
    "/api/motion/request",
    "/api/motion/prepare-relative",
    "/api/perception/pick-place",
  ].includes(path);
}
export async function cancelRobot(id: string) {
  return gateway("/api/motion/cancel", {
    schema_version: schemaVersion,
    request_id: randomUUID(),
    action: "cancel",
    target_request_id: id,
  });
}
export async function waitRequest(
  id: string,
  update: (record: RequestRecord) => Promise<void> = async () => {},
) {
  for (;;) {
    const response = await fetch(
      `${base()}/api/requests/${encodeURIComponent(id)}`,
      { cache: "no-store" },
    );
    const record = (await response.json()) as RequestRecord;
    if (!response.ok)
      throw Error(
        record.original_error ?? `查询机器人任务失败 (${response.status})`,
      );
    await update(record);
    if (record.terminal || record.state === "unknown") return record;
    await new Promise((resolve) => setTimeout(resolve, 500));
  }
}
export class RobotToolsContext {
  constructor(
    readonly run: AIRun,
    readonly signal: AbortSignal,
  ) {}
  async command(
    callId: string,
    index: number,
    path: string,
    fields: Record<string, unknown>,
    label: string,
  ) {
    if (this.signal.aborted) throw Error("AI 已停止，不再下发新动作");
    const id = `${this.run.id}:${callId}:${index}`;
    let request = this.run.requests.find((r) => r.id === id);
    if (!request) {
      request = { id, path, label, state: "accepted" };
      this.run.requests.push(request);
      // Persist before sending. On ambiguity, only query this ID, never replay.
      await saveRun(this.run);
      try {
        await gateway(path, {
          ...fields,
          schema_version: schemaVersion,
          request_id: id,
        });
      } catch (error) {
        // Some endpoints report a legitimate failed task via HTTP 400. Its
        // persisted original result remains authoritative, including its cause.
        try {
          const record = await fetch(
            `${base()}/api/requests/${encodeURIComponent(id)}`,
            { cache: "no-store" },
          );
          // A found failed record is still known. Do not route this lookup
          // through gateway(), which deliberately throws original_error.
          if (!record.ok) throw Error(`原请求记录不可查询：${record.status}`);
        } catch {
          request.state = "unknown";
          request.result = String(error);
          await saveRun(this.run);
          throw error;
        }
      }
    }
    let cancelSent = false;
    const result = await waitRequest(id, async (result) => {
      request.state = result.state;
      request.result = result.value;
      await saveRun(this.run);
      if (
        !result.terminal &&
        result.state !== "unknown" &&
        this.signal.aborted &&
        !cancelSent &&
        cancellable(path)
      ) {
        cancelSent = true;
        await cancelRobot(id);
      }
    });
    if (result.state !== "succeeded")
      throw Error(result.original_error ?? `机器人任务 ${id} ${result.state}`);
    return { value: result.value, request_id: id };
  }
}
