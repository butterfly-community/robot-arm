import type {
  ArmState,
  MotionState,
  RobotModelInfo,
  ToolPose,
} from "@robot/contracts";

const degrees = (radians: number) => (radians * 180) / Math.PI;

/** Display conversions only; FK and target transforms stay in the motion node. */
export function readablePose(pose: ToolPose) {
  return {
    frame: pose.frame,
    position_m: pose.position_m,
    position_mm: pose.position_m.map((value) => value * 1000),
    orientation_xyzw: pose.orientation_xyzw,
  };
}

export function jointReadings(model: RobotModelInfo, positions: number[]) {
  return model.joints.map((joint, index) => {
    const value = positions[index];
    const valid = Number.isFinite(value);
    return {
      joint_key: joint.key,
      label: joint.label,
      position_rad: valid ? value : null,
      position_deg: valid ? degrees(value) : null,
      range_rad: [joint.minimum, joint.maximum],
      range_deg: [degrees(joint.minimum), degrees(joint.maximum)],
    };
  });
}

export const coordinateGuide = {
  units:
    "运动接口使用米、弧度、xyzw 四元数；mm 和 deg 仅用于阅读。1 cm=0.01 m，1°=π/180 rad。",
  base_frame:
    "固定在机械臂底座。+Z 向上、-Z 向下；X/Y 依底座坐标轴，不等同于相机画面的左右远近。",
  tcp_frame:
    "原点是模型定义的工具中心点；X/Y/Z 随夹爪朝向旋转。沿工具 +Z 不一定向上。",
  rotations:
    "正转遵循指定坐标轴的右手定则；TCP 旋转以工具中心点为目标参考点。增量四元数 [0,0,0,1] 表示保持朝向。",
  joints:
    "关节目标是模型坐标中的绝对关节角，不是舵机原始协议角度，方向和零位转换由驱动处理。未指定关节保持。改变一个关节会连带移动后续连杆，末端效果取决于整臂构型，不能固定解释成某关节增加就向上。",
  preview:
    "preview_motion 只计算姿态，不运动、不做 IK 或碰撞检查。工具轴向量按 X/Y/Z 顺序给出它们在底座系中的方向；当前姿态为空目标预览，关节改动用绝对角度预览。",
  observations:
    "相机图像和电机反馈不是硬件同步；TCP 的 feedback 是计算该姿态所用的实际角度，不一定与最新反馈序号相同。不能把命令目标或预览当成实际到位。",
};

export function postureSummary(
  model?: RobotModelInfo,
  arm?: ArmState,
  motion?: MotionState,
) {
  if (!model) return { available: false, reason: "机械臂模型尚未就绪" };
  const actual = arm?.model_revision === model.model_revision ? arm : undefined;
  const tcp = motion?.current_tool_pose;
  const tcpMatches = tcp?.arm_state?.model_revision === model.model_revision;
  return {
    available: Boolean(actual),
    model_revision: model.model_revision,
    base_frame: model.base_frame,
    tcp_frame: model.tcp_frame,
    actual: actual
      ? {
          source: actual.feedback_source,
          sequence: actual.sequence,
          sample_time_ns: actual.sample_time_ns,
          joints: jointReadings(model, actual.joints_rad),
          gripper: model.tool_actuators.map((actuator, index) => {
            const value = actual.actuators_rad[index];
            return {
              actuator_key: actuator.key,
              label: actuator.label,
              position_rad: Number.isFinite(value) ? value : null,
              position_deg: Number.isFinite(value) ? degrees(value) : null,
            };
          }),
        }
      : null,
    measured_tcp:
      tcp && tcpMatches
        ? {
            ...readablePose(tcp),
            feedback: {
              source: tcp.arm_state.feedback_source,
              sequence: tcp.arm_state.sequence,
              sample_time_ns: tcp.arm_state.sample_time_ns,
              joints: jointReadings(model, tcp.arm_state.joints_rad),
            },
          }
        : null,
    commanded_tcp_not_measured: motion?.target_tool_pose
      ? readablePose(motion.target_tool_pose)
      : null,
    note: "actual 来自当前反馈，measured_tcp 来自 FK 对应反馈；null 表示不可用，不表示零位。",
  };
}
