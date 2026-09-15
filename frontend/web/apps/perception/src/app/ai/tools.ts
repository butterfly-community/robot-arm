import { tool, type ToolSet } from "ai";
import { z } from "zod";
import type { PerceptionState } from "@robot/contracts";
import { startPickPlace } from "../start-pick-place";
import {
  compactScene,
  compactCamera,
  gateway,
  imageBytes,
  perceptionSnapshot,
  robotModel,
  namedTargetRequest,
  tcpRequest,
  scene,
  waitRequest,
  RobotToolsContext,
} from "./robot";
import { getImage, putImage, saveRun } from "./store";
import { errorText } from "./types";

function perceptionResult(result: unknown): PerceptionState {
  return (result as { value: PerceptionState }).value;
}

export function robotTools(context: RobotToolsContext): ToolSet {
  let serial = Promise.resolve();
  function define<T>(
    name: string,
    description: string,
    schema: z.ZodType<T>,
    action: (
      input: T,
      post: (
        path: string,
        fields: Record<string, unknown>,
        label?: string,
      ) => Promise<unknown>,
    ) => Promise<unknown>,
    image = false,
  ) {
    return tool({
      description,
      inputSchema: schema,
      execute: async (input, { toolCallId }) => {
        const previous = serial;
        let release!: () => void;
        serial = new Promise<void>((resolve) => {
          release = resolve;
        });
        await previous;
        const call = {
          id: toolCallId,
          name,
          input,
          startedAt: Date.now(),
        } as (typeof context.run.calls)[number];
        context.run.calls.push(call);
        let index = 0;
        try {
          await saveRun(context.run);
          if (context.signal.aborted) throw Error("AI 已停止");
          const result = await action(input, (path, fields, label = name) =>
            context.command(toolCallId, index++, path, fields, label),
          );
          call.result = result;
          return result;
        } catch (error) {
          call.error = errorText(error);
          return {
            error: call.error,
            tool: name,
            request_ids: context.run.requests
              .filter((r) => r.id.includes(toolCallId))
              .map((r) => r.id),
          };
        } finally {
          call.endedAt = Date.now();
          try {
            await saveRun(context.run);
          } finally {
            release();
          }
        }
      },
      toModelOutput: async ({ output }) => {
        const value = output as { image_id?: string; error?: string };
        if (image && value.image_id && !value.error) {
          const stored = await getImage(value.image_id);
          return {
            type: "content" as const,
            value: [
              { type: "text" as const, text: JSON.stringify(output) },
              {
                type: "file" as const,
                mediaType: stored.mediaType,
                data: {
                  type: "data" as const,
                  data: stored.bytes.toString("base64"),
                },
              },
            ],
          };
        }
        return { type: "text" as const, value: JSON.stringify(output) };
      },
    });
  }
  const empty = z.object({});
  return {
    read_robot: define(
      "read_robot",
      "读取实际关节、TCP、模型关节键/范围、夹爪及力度反馈。只读，不运动。",
      empty,
      async () => {
        const motion = (await gateway("/api/motion/state")).values;
        const execution = (await gateway("/api/arm-execution/state")).values;
        const model = motion.robot_model_info;
        const transport = execution.transport_state;
        return {
          model: {
            model_revision: model?.model_revision,
            base_frame: model?.base_frame,
            tcp_frame: model?.tcp_frame,
            joints: model?.joints,
            tool_actuators: model?.tool_actuators,
            named_targets: model?.named_targets,
          },
          arm: motion.arm_state,
          motion: motion.motion_state,
          connected: transport?.connected,
          force: {
            target: transport?.gripper_strength_percent,
            feedback: transport?.gripper_strength_feedback_percent,
            control_power_mw: transport?.gripper_control_power_mw,
            telemetry: transport?.arm_telemetry,
          },
          note: "负载和控制器成功不等于视觉确认抓住物体",
        };
      },
    ),
    read_scene: define(
      "read_scene",
      "读取相机、分割模型、标注来源和当前三维场景；不触发分割。通常 include_camera_capabilities=false；只有需要查询设备所有分辨率、FPS 和驱动参数时设为 true。",
      z.object({ include_camera_capabilities: z.boolean() }),
      async ({ include_camera_capabilities }) => {
        const v = (await perceptionSnapshot()).values;
        return {
          perception: v.perception_state,
          camera: include_camera_capabilities
            ? v.camera_state
            : compactCamera(v.camera_state),
          scene: v.world_scene ? compactScene(v.world_scene) : null,
        };
      },
    ),
    observe_camera: define(
      "observe_camera",
      "采集当前相机新的真实 RGB 图片并识图，不运行分割、不运动。执行后确认物体时必须重新观察。",
      empty,
      async (_, post) => {
        const result = await post(
          "/api/perception/request",
          { action: "snapshot", classes: null, placement_labels: null },
          "刷新相机图像",
        );
        const state = perceptionResult(result);
        const image = await putImage(await imageBytes("color.png"));
        return {
          image_id: image.id,
          ...image,
          source_id: state.source_id,
          captured_at_ns: state.last_frame_time_ns,
          frame: state.color_frame,
          note: "相机原始 RGB，不是机械臂基座视角；像素位置不是空间坐标",
        };
      },
      true,
    ),
    capture_segmentation: define(
      "capture_segmentation",
      "载入新的固定分割帧并返回图片，等同网页载入新分割帧；不运行模型。新帧使旧标注/旧三维场景失效。",
      empty,
      async (_, post) => {
        const result = await post("/api/perception/request", {
          action: "refresh",
          input_sequence: null,
          segmentation_edit: { kind: "capture" },
        });
        const state = perceptionResult(result);
        const image = await putImage(
          await imageBytes("segmentation-color.png"),
        );
        return {
          image_id: image.id,
          ...image,
          sequence: state.last_segmentation_sequence,
          source_id: state.source_id,
          frame: state.segmentation_frame,
          available_models: state.available_models,
          saved_segmentation: {
            model: state.model,
            classes: state.classes,
            visual_prompt_active: state.visual_prompt_active,
            placement_labels: state.placement_labels,
          },
        };
      },
      true,
    ),
    segment: define(
      "segment",
      "在同一固定帧运行现有模型。自动模型忽略提示词；提示词模型可使用 text 英文短词组或 saved_visual 已保存的视觉示例（read_scene.visual_prompt_active）。只分割、不定位和抓放。",
      z.object({
        model: z.string(),
        prompt_mode: z.enum(["text", "saved_visual"]),
        prompts: z.array(z.string()),
        placement_labels: z.array(z.string()),
      }),
      async (input, post) => {
        const state = (await perceptionSnapshot()).values.perception_state;
        const model = state.available_models.find((m) => m.id === input.model);
        if (!model) throw Error("请从 read_scene 的模型目录选择分割模型");
        const promptFields =
          model.prompt_free || input.prompt_mode === "saved_visual"
            ? {}
            : { classes: input.prompts, prompt: { kind: "text" } };
        const result = await post("/api/perception/request", {
          action: "refresh",
          input_sequence: state.last_segmentation_sequence,
          segmentation_edit: { kind: "model" },
          model: model.id,
          ...promptFields,
          placement_labels: input.placement_labels,
        });
        return perceptionResult(result);
      },
    ),
    annotate: define(
      "annotate",
      "依据当前图像自主框选可见目标，任务内无需额外授权，模型漏检时也可直接使用。复用网页手动框选契约，在最近 capture_segmentation 返回的当前固定帧提交像素框；来源为 manual，不能冒充模型分割。可提交空数组清除。框坐标使用原图分辨率，不能当成三维位置。内部结果序号由工具按当前帧填写，与网页一致。",
      z.object({
        regions: z.array(
          z.object({
            id: z.string(),
            label: z.string(),
            bounding_box_xyxy: z.tuple([
              z.number(),
              z.number(),
              z.number(),
              z.number(),
            ]),
          }),
        ),
        placement_labels: z.array(z.string()),
      }),
      async (input, post) => {
        const state = (await perceptionSnapshot()).values.perception_state;
        const result = await post("/api/perception/request", {
          action: "refresh",
          input_sequence: state.last_segmentation_sequence,
          segmentation_edit: { kind: "manual", regions: input.regions },
          placement_labels: input.placement_labels,
        });
        return perceptionResult(result);
      },
    ),
    reconstruct: define(
      "reconstruct",
      "把已有分割结合深度和标定转换到三维场景，不自动生成抓取候选。",
      empty,
      async (_, post) => {
        const state = (await perceptionSnapshot()).values.perception_state;
        const result = await post("/api/perception/request", {
          action: "reconstruct",
          input_sequence: state.last_segmentation_sequence,
        });
        return compactScene(
          await scene(
            perceptionResult(result).last_scene_sequence,
            context.signal,
          ),
        );
      },
    ),
    generate_grasps: define(
      "generate_grasps",
      "对当前场景指定对象运行现有 GraspGenX 生成候选，不运动。",
      z.object({ object_id: z.string() }),
      async (input, post) => {
        const current = await scene();
        const result = await post("/api/perception/request", {
          action: "generate_grasps",
          object_id: input.object_id,
          input_sequence: current.sequence,
        });
        return compactScene(
          await scene(
            perceptionResult(result).last_scene_sequence,
            context.signal,
          ),
        );
      },
    ),
    pick_place: define(
      "pick_place",
      "复用手动启动：对当前场景对象和区域抓放，缺少候选时生成，等待 MoveIt/控制器最终结果。不得把结果当视觉确认；随后可重新观察。",
      z.object({ object_id: z.string(), placement_region_id: z.string() }),
      async (input, post) => {
        const result = await startPickPlace(
          await scene(),
          input.object_id,
          input.placement_region_id,
          (path, body) => post(path, body),
          () => "assigned-by-run",
        );
        return { result, note: "控制流程已完成；请重新观察核对实际物体" };
      },
    ),
    work_pose: define(
      "work_pose",
      "使用系统已定义的工作位，等待实际运动完成。与网页工作位按钮相同。",
      empty,
      async (_, post) => {
        const target = namedTargetRequest(await robotModel(), "work");
        await post("/api/motion/mode", { mode: "manual" });
        return post("/api/motion/request", target, "回工作位");
      },
    ),
    move_joints: define(
      "move_joints",
      "通过已有 MoveIt 规划执行关节目标；joint_key 来自 read_robot，position_rad 为弧度。未指定的关节保持，不附带夹爪动作。",
      z.object({
        joints: z.array(
          z.object({ joint_key: z.string(), position_rad: z.number() }),
        ),
      }),
      async (input, post) => {
        const model = await robotModel();
        await post("/api/motion/mode", { mode: "manual" });
        return post("/api/motion/request", {
          action: "apply",
          model_revision: model.model_revision,
          joints: input.joints,
          actuators: [],
          options: {},
        });
      },
    ),
    move_tcp_absolute: define(
      "move_tcp_absolute",
      "TCP 绝对目标：底座坐标中的目标位置（米）和目标朝向（xyzw），不是位移增量。由实际反馈 FK、MoveIt IK/碰撞/执行。",
      z.object({
        frame: z.string(),
        position_m: z.tuple([z.number(), z.number(), z.number()]),
        orientation_xyzw: z.tuple([
          z.number(),
          z.number(),
          z.number(),
          z.number(),
        ]),
      }),
      async (pose, post) => {
        const model = await robotModel();
        await post("/api/motion/mode", { mode: "manual" });
        return post(
          "/api/motion/request",
          tcpRequest(
            model,
            false,
            pose.frame,
            pose.position_m,
            pose.orientation_xyzw,
          ),
        );
      },
    ),
    move_tcp_relative: define(
      "move_tcp_relative",
      "TCP 增量运动：只填位移和旋转增量，不填当前位姿或最终绝对位置。底座 Z 向上 1 厘米是 translation_delta_m=[0,0,0.01]；保持朝向 rotation_delta_xyzw=[0,0,0,1]。底座表达 p'=p+dp,R'=dR*R；工具表达 p'=p+R*dp,R'=R*dR。复用同一运动接口。",
      z.object({
        frame: z.string().describe("read_robot 返回的 base_frame 或 tcp_frame"),
        translation_delta_m: z
          .tuple([z.number(), z.number(), z.number()])
          .describe("位移增量，单位米，不是最终位置"),
        rotation_delta_xyzw: z
          .tuple([z.number(), z.number(), z.number(), z.number()])
          .describe("旋转增量，保持朝向为 [0,0,0,1]，不是当前朝向"),
      }),
      async (input, post) => {
        const model = await robotModel();
        await post("/api/motion/mode", { mode: "manual" });
        return post(
          "/api/motion/request",
          tcpRequest(
            model,
            true,
            input.frame,
            input.translation_delta_m,
            input.rotation_delta_xyzw,
          ),
        );
      },
    ),
    gripper: define(
      "gripper",
      "独立执行夹爪驱动关节目标（弧度），等待控制器终态。驱动关节角不是两指总开角。闭合后的保持力仍由执行层持续调节。",
      z.object({ actuator_key: z.string(), position_rad: z.number() }),
      async (input, post) =>
        post("/api/motion/actuator", {
          ...input,
          model_revision: (await robotModel()).model_revision,
        }),
    ),
    set_grip_force: define(
      "set_grip_force",
      "设置现有持续力度反馈目标 0–100；这是相对反馈目标，不是固定 mW，也不是达到后停止。",
      z.object({ target: z.number() }),
      async (input, post) =>
        post("/api/arm-execution/config", {
          action: "apply",
          fields: { gripper_strength_percent: String(input.target) },
        }),
    ),
    request_result: define(
      "request_result",
      "按准确请求编号查询实际状态；只查询，不重发。",
      z.object({ request_id: z.string() }),
      async (input) =>
        gateway(`/api/requests/${encodeURIComponent(input.request_id)}`),
    ),
    cancel_task: define(
      "cancel_task",
      "取消指定机器人请求并等待原请求终态，不自动松爪或回工作位。",
      z.object({ request_id: z.string() }),
      async (input, post) => {
        await post("/api/motion/cancel", {
          action: "cancel",
          target_request_id: input.request_id,
        });
        return waitRequest(input.request_id);
      },
    ),
  };
}
