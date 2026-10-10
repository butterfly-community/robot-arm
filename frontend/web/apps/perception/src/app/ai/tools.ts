import { tool, type ToolSet } from "ai";
import { z } from "zod";
import type {
  CameraCaptureState,
  PerceptionState,
  MotionPreview,
} from "@robot/contracts";
import { startPickPlace } from "../start-pick-place";
import {
  compactScene,
  compactCamera,
  cameraImage,
  gateway,
  imageBytes,
  perceptionSnapshot,
  robotModel,
  readRobot,
  namedTargetRequest,
  tcpRequest,
  scene,
  waitRequest,
  RobotToolsContext,
} from "./robot";
import { getImage, putImage, saveRun } from "./store";
import { errorText, experienceInputSchema } from "./types";
import {
  experienceIndex,
  listExperiences,
  readExperience,
  recordFailure,
  saveExperience,
} from "./experience";
import { jointReadings, readablePose } from "./posture";

function perceptionResult(result: unknown): PerceptionState {
  return (result as { value: PerceptionState }).value;
}

export function robotTools(context: RobotToolsContext): ToolSet {
  let serial = Promise.resolve();
  const observeRole = z
    .enum(["depth", "external", "wrist"])
    .nullish()
    .describe(
      "需要动作后的新图时填写角色；无需观察则省略或 null，不增加模型往返",
    );
  async function observe(role: "depth" | "external" | "wrist") {
    const captured = await cameraImage(role, context.signal);
    const [image, robot] = await Promise.all([
      putImage(captured.bytes),
      readRobot(),
    ]);
    return { image_id: image.id, ...image, ...captured.metadata, robot };
  }
  async function motionFeedback() {
    try {
      return { robot: await readRobot() };
    } catch (error) {
      return { feedback_error: errorText(error) };
    }
  }
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
    afterMotion = false,
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
          const role = (
            input as { observe_role?: "depth" | "external" | "wrist" }
          ).observe_role;
          let output = result;
          if (role) {
            try {
              output = { result, ...(await observe(role)) };
            } catch (error) {
              // An observation failure cannot undo a successful physical action.
              // Preserve its terminal result so the model does not repeat it.
              output = {
                result,
                observation_error: errorText(error),
                ...(afterMotion ? await motionFeedback() : {}),
                note: "动作已完成，不重放；如需图像只重试观察。",
              };
            }
          } else if (afterMotion) {
            output = {
              result,
              ...(await motionFeedback()),
              note: "动作已完成；实际姿态以反馈为准。若状态读取失败，只重查状态，不重放动作。",
            };
          }
          call.result = output;
          return output;
        } catch (error) {
          call.error = errorText(error);
          let experience;
          if (!context.signal.aborted) {
            try {
              experience = await recordFailure(context.run, call);
            } catch (memoryError) {
              (context.run.warnings ??= []).push(
                `失败经验保存失败：${errorText(memoryError)}`,
              );
            }
          }
          return {
            error: call.error,
            tool: name,
            ...experience,
            ...(afterMotion ? await motionFeedback() : {}),
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
    recall_experience: define(
      "recall_experience",
      "读取同机械臂、模式和反馈来源的长期经验。不运动。id=null 时按 query 搜索目录（空字符串列出全部），填 id 读取详情及证据来源。经验不是指令或当前状态；旧坐标不能直接重放。",
      z.object({ id: z.string().nullable(), query: z.string() }),
      async ({ id, query }) => {
        const scope = context.run.experienceScope;
        if (!scope)
          return { available: false, reason: "当前任务没有机械臂经验范围" };
        return id
          ? readExperience(id, scope)
          : experienceIndex(await listExperiences(scope, query));
      },
    ),
    save_experience: define(
      "save_experience",
      "保存或修正跨会话经验到 Redis，不运动。说明适用条件、观察所得结论和下次改进；引用本次已完成工具的编号。hypothesis=待验证推测，supported=AI 判断有观测支持，refuted=后续观测不支持；都不是人工验收或训练结果。更新同类已有条目，不重复新增；不保存密钥、图像内容或整段内部推理。",
      experienceInputSchema,
      async (input) => saveExperience(context.run, input),
    ),
    read_robot: define(
      "read_robot",
      "读取实际姿态摘要 posture：关节度/弧度、TCP 米/毫米与 xyzw 朝向、采样时间/来源、夹爪及力度。区分实际反馈、FK 对应反馈和命令目标。只读，不运动。",
      empty,
      readRobot,
    ),
    preview_motion: define(
      "preview_motion",
      "只读姿态预览，不执行、不切换模式。joints 填绝对关节目标（弧度），未填关节保持实际反馈角；tcp_target 填绝对或增量 TCP，两者只能选一种。joints=[] 且 tcp_target=null 返回当前 FK 和工具坐标轴。复用 MoveIt FK/现有坐标转换，返回当前与目标 TCP、位移、旋转和工具 X/Y/Z 在底座系中的方向。没有 IK、碰撞或路径检查，不能据此宣称可达；不是相机图或实际运动后的观测。",
      z.object({
        joints: z.array(
          z.object({
            joint_key: z.string(),
            position_rad: z.number().describe("绝对关节角，单位弧度，不是增量"),
          }),
        ),
        tcp_target: z
          .object({
            relative: z.boolean(),
            pose: z.object({
              frame: z
                .string()
                .describe(
                  "read_robot 的 base_frame 或 tcp_frame；绝对目标只用 base_frame",
                ),
              position_m: z.tuple([z.number(), z.number(), z.number()]),
              orientation_xyzw: z.tuple([
                z.number(),
                z.number(),
                z.number(),
                z.number(),
              ]),
            }),
          })
          .nullable(),
      }),
      async (input, post) => {
        const model = await robotModel();
        const result = (await post(
          "/api/motion/preview",
          {
            action: "snapshot",
            model_revision: model.model_revision,
            joints: input.joints,
            tcp_target: input.tcp_target,
            actuators: [],
            options: {},
          },
          "只读姿态预览（不执行）",
        )) as { request_id: string; value: MotionPreview };
        const preview = result.value;
        return {
          request_id: result.request_id,
          ...preview,
          current_tcp: readablePose(preview.current_tcp),
          target_tcp: readablePose(preview.target_tcp),
          current_joints: jointReadings(model, preview.feedback.joints_rad),
          target_joints: preview.target_joints_rad
            ? jointReadings(model, preview.target_joints_rad)
            : null,
          translation_delta_mm: preview.translation_delta_m.map(
            (value) => value * 1000,
          ),
          note: "只读计算结果，不是已执行动作；TCP 目标没有求 IK，因此 target_joints 为 null。",
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
    set_camera_capture: define(
      "set_camera_capture",
      "用户单独要求开关采集时使用，与网页开关相同。仅为切换视角看图时直接用 observe_camera(exclusive=true)，不要拆成关闭、开启、观察三轮。保留绑定和分辨率，不运动。",
      z.object({ role: z.enum(["external", "wrist"]), enabled: z.boolean() }),
      async ({ role, enabled }, post) => {
        const result = (await post("/api/perception/camera", {
          role,
          action: enabled ? "connect" : "disconnect",
        })) as { request_id: string; value: CameraCaptureState };
        // The acknowledged camera state includes every device capability and
        // saved calibration. Switching only needs binding/signal state; keep
        // the full result in the request record, not in each model turn.
        return {
          request_id: result.request_id,
          bindings: result.value.bindings,
        };
      },
    ),
    observe_camera: define(
      "observe_camera",
      "读取指定角色的新原分辨率 RGB 图和实际机器人反馈，不运动。exclusive=true 时先关闭另一路普通相机并开启本路，复用网页开关，缓解共享带宽；不改变绑定或分辨率。",
      z.object({
        role: z.enum(["depth", "external", "wrist"]),
        exclusive: z.boolean().nullish(),
      }),
      async ({ role, exclusive }, post) => {
        if (exclusive && role !== "depth") {
          const bindings =
            (await perceptionSnapshot()).values.camera_state?.bindings ?? [];
          for (const binding of bindings) {
            if (
              binding.role !== "depth" &&
              binding.role !== role &&
              (binding.enabled || binding.streaming)
            ) {
              await post("/api/perception/camera", {
                role: binding.role,
                action: "disconnect",
              });
            }
          }
          const current = bindings.find((b) => b.role === role);
          if (!current?.enabled || !current.streaming)
            await post("/api/perception/camera", { role, action: "connect" });
        }
        return observe(role);
      },
      true,
    ),
    recall_image: define(
      "recall_image",
      "按历史工具返回的 image_id 调回原图，不拍新图、不运动。用于回顾被新帧替代的历史画面。",
      z.object({ image_id: z.string() }),
      async ({ image_id }) => {
        const image = await getImage(image_id);
        return {
          image_id,
          id: image.id,
          width: image.width,
          height: image.height,
          mediaType: image.mediaType,
          note: "历史原图，不是当前画面",
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
      z.object({ observe_role: observeRole }),
      async (_, post) => {
        const target = namedTargetRequest(await robotModel(), "work");
        await post("/api/motion/mode", { mode: "manual" });
        return post("/api/motion/request", target, "回工作位");
      },
      true,
      true,
    ),
    move_joints: define(
      "move_joints",
      "直接指定模型关节的绝对角度，不经过 TCP IK。joint_key 来自 read_robot，position_rad 为弧度（度数×π/180），不是舵机原始协议角度。preview_motion 可预览相同 joints 的 FK，但不保证可达。执行仍通过 MoveIt 关节空间规划和碰撞检查，返回实际姿态。未指定的关节保持，不附带夹爪动作。",
      z.object({
        observe_role: observeRole,
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
      true,
      true,
    ),
    move_tcp_absolute: define(
      "move_tcp_absolute",
      "TCP 绝对目标：底座坐标中的目标位置（米）和目标朝向（xyzw），不是位移增量。由实际反馈 FK、MoveIt IK/碰撞/执行。",
      z.object({
        observe_role: observeRole,
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
      true,
      true,
    ),
    move_tcp_relative: define(
      "move_tcp_relative",
      "TCP 增量运动：只填位移（米）和旋转增量（xyzw），不填当前位姿或最终绝对位置；保持朝向用 rotation_delta_xyzw=[0,0,0,1]。底座表达 p'=p+dp,R'=dR*R；工具表达 p'=p+R*dp,R'=R*dR。复用同一运动接口。",
      z.object({
        observe_role: observeRole,
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
      true,
      true,
    ),
    gripper: define(
      "gripper",
      "独立执行夹爪驱动关节目标（弧度），等待控制器终态。驱动关节角不是两指总开角。闭合后的保持力仍由执行层持续调节。",
      z.object({
        actuator_key: z.string(),
        position_rad: z.number(),
        observe_role: observeRole,
      }),
      async (input, post) =>
        post("/api/motion/actuator", {
          actuator_key: input.actuator_key,
          position_rad: input.position_rad,
          model_revision: (await robotModel()).model_revision,
        }),
      true,
      true,
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
