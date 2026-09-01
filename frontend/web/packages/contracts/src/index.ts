export const schemaVersion = 3;
export const virtualFeedbackTarget = {
  sourceId: "virtual-feedback",
  capabilityPath: "feedback/virtual",
} as const;

export type Json =
  null | boolean | number | string | Json[] | { [key: string]: Json };
export type Namespace =
  "tracking" | "spatial" | "perception" | "motion" | "arm-execution";
export type RequestAction =
  | "apply"
  | "cancel"
  | "select"
  | "unselect"
  | "connect"
  | "disconnect"
  | "discover"
  | "refresh";

export interface Snapshot {
  schema_version: number;
  sequence: number;
  namespace: Namespace;
  values: Record<string, Json>;
}

export interface JointMetadata {
  key: string;
  label: string;
  unit: string;
  minimum: number;
  maximum: number;
}

export interface ActuatorMetadata extends JointMetadata {
  visualization_joint_key?: string;
}

export interface RobotModelInfo {
  schema_version: number;
  model_id: string;
  model_revision: string;
  display_name: string;
  base_frame: string;
  tcp_frame: string;
  gripper_asset_id?: string;
  joints: JointMetadata[];
  tool_actuators: ActuatorMetadata[];
  named_targets: Array<{
    key: string;
    label: string;
    joint_positions_rad: Record<string, number>;
    actuator_positions_rad: Record<string, number>;
  }>;
  motion_options: Array<FieldSchema>;
  diagnostics: Array<FieldSchema>;
  visualization: {
    manifest_hash: string;
    root_path: string;
    files: string[];
    link_materials: Record<
      string,
      {
        color_rgb: [number, number, number];
        metalness: number;
        roughness: number;
      }
    >;
  };
}

export interface FieldSchema {
  key: string;
  label: string;
  unit: string;
  minimum?: number | null;
  maximum?: number | null;
  required: boolean;
}

export interface ArmState {
  model_revision: string;
  feedback_source: "software" | "hardware";
  joints_rad: number[];
  actuators_rad: number[];
}

export interface ArmCommand {
  schema_version: number;
  sequence: number;
  controller_time_ns: number;
  model_revision: string;
  joints_rad: number[];
  actuators_rad: number[];
}

export interface ToolPose {
  frame: string;
  position_m: [number, number, number];
  orientation_xyzw: [number, number, number, number];
}

export interface MotionState {
  control_mode: "relative" | "manual" | "perception";
  current_tool_pose?: ToolPose | null;
  target_tool_pose?: ToolPose | null;
}

export interface ParameterValue {
  actuator_key: string;
  field_key: string;
  value?: number | null;
  unit: string;
  read_time_ns: number;
  original_error?: string | null;
}

export interface ExecutionInfo {
  connection_fields: Array<{
    key: string;
    label: string;
    field_type: string;
    required: boolean;
    options: string[];
  }>;
  actuator_labels: string[];
}

export interface PoseFlags {
  position_valid: boolean;
  position_tracked: boolean;
  orientation_valid: boolean;
  orientation_tracked: boolean;
}

export interface AbsolutePoseFrame {
  schema_version: number;
  sequence: number;
  source_time_ns: number;
  received_time_ns: number;
  position_source_id?: string | null;
  orientation_source_id?: string | null;
  position_source_capable: boolean;
  orientation_source_capable: boolean;
  reference_space: string;
  position_m: [number, number, number];
  orientation_xyzw: [number, number, number, number];
  flags: PoseFlags;
}

export interface BooleanActionSample {
  is_active: boolean;
  changed_since_last_sync: boolean;
  value: boolean;
}

export interface FloatActionSample {
  is_active: boolean;
  changed_since_last_sync: boolean;
  value: number;
}

export interface ActuatorActions {
  primary_tool_open: BooleanActionSample;
  primary_tool: FloatActionSample;
}

export interface ControlInputFrame {
  schema_version: number;
  sequence: number;
  source_time_ns: number;
  received_time_ns: number;
  start_stop: BooleanActionSample;
  emergency_stop: BooleanActionSample;
  actuator_actions: ActuatorActions;
  move_forward_back: FloatActionSample;
  move_left_right: FloatActionSample;
  move_up_down: FloatActionSample;
  front_pitch: FloatActionSample;
  horizontal_arc: FloatActionSample;
  tool_pitch: FloatActionSample;
  tool_yaw: FloatActionSample;
  tool_roll: FloatActionSample;
  tool_axis_translation: FloatActionSample;
  tool_helical_motion: FloatActionSample;
}

export type InputSimulationItem =
  | "move_forward_back"
  | "move_left_right"
  | "move_up_down"
  | "tool_pitch"
  | "tool_yaw"
  | "tool_roll"
  | "front_pitch"
  | "horizontal_arc"
  | "primary_tool_open"
  | "primary_tool"
  | "start_stop"
  | "emergency_stop"
  | "primary_tool_feedback"
  | "tool_axis_translation"
  | "tool_helical_motion";

export type PerceptionSourceKind = "camera" | "generated_test_scene";

export interface DepthCameraSourceInfo {
  source_id: string;
  display_name: string;
  color_topic: string;
  depth_topic: string;
  depth_info_topic: string;
}

export interface ImageFrameInfo {
  width: number;
  height: number;
  encoding: string;
  frame_id: string;
}

export interface DepthCameraCalibration {
  schema_version: number;
  sequence: number;
  source_time_ns: number;
  source_id: string;
  parent_frame_id: string;
  frame_id: string;
  translation_m: [number, number, number];
  orientation_xyzw: [number, number, number, number];
  width: number;
  height: number;
  distortion_model: string;
  distortion: number[];
  camera_matrix: [
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
  ];
  projection_matrix: [
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
    number,
  ];
}

export interface PerceptionInstanceSummary {
  instance_id: string;
  label: string;
  confidence: number;
  bounding_box_xyxy: [number, number, number, number];
  position_m?: [number, number, number] | null;
  size_m?: [number, number, number] | null;
  grasp_candidate_count: number;
}

export interface PerceptionState {
  schema_version: number;
  enabled: boolean;
  source_kind?: PerceptionSourceKind | null;
  source_id?: string | null;
  compute_service_url: string;
  model: string;
  classes: string[];
  placement_labels: string[];
  available_sources: DepthCameraSourceInfo[];
  color_frame?: ImageFrameInfo | null;
  depth_frame?: ImageFrameInfo | null;
  camera_calibration?: DepthCameraCalibration | null;
  depth_scale_m: number;
  instances: PerceptionInstanceSummary[];
  point_count?: number | null;
  last_frame_time_ns?: number | null;
  last_scene_sequence?: number | null;
  calibrated: boolean;
  original_error?: string | null;
}

export interface ScenePose {
  position_m: [number, number, number];
  orientation_xyzw: [number, number, number, number];
}

export interface SceneObject {
  object_id: string;
  label: string;
  pose: ScenePose;
  size_m: [number, number, number];
  confidence: number;
  grasp_candidates: ScenePose[];
}

export interface PlacementRegion {
  region_id: string;
  label: string;
  pose: ScenePose;
  size_m: [number, number, number];
  source_object_id?: string | null;
}

export interface WorldScene {
  schema_version: number;
  sequence: number;
  sample_time_ns: number;
  frame_id: string;
  objects: SceneObject[];
  placement_regions: PlacementRegion[];
  obstacles: Array<{
    obstacle_id: string;
    pose: ScenePose;
    size_m: [number, number, number];
  }>;
}

export interface ManipulationTaskState {
  schema_version: number;
  request_id: string;
  object_id?: string | null;
  placement_region_id?: string | null;
  pick_position_m?: [number, number, number] | null;
  place_position_m?: [number, number, number] | null;
  state:
    "idle" | "planning" | "executing" | "succeeded" | "failed" | "cancelled";
  stage?: string | null;
  solution_count?: number | null;
  selected_cost?: number | null;
  original_error?: string | null;
}

export interface CalibrationBoard {
  pattern: string;
  dictionary: string;
  squares_x: number;
  squares_y: number;
  square_size_m: number;
  marker_size_m: number;
  measured_width_m: number;
  measured_height_m: number;
}

export interface CalibrationResult {
  schema_version: number;
  camera_source_id: string;
  robot_model_revision: string;
  calibration_tool_id: string;
  board: CalibrationBoard;
  camera_in_base: ScenePose;
  board_in_calibration_tool: ScenePose;
  solver: string;
  solved_at_ns: number;
  sample_count: number;
  translation_residuals_m: number[];
  rotation_residuals_rad: number[];
}

export interface CalibrationSessionState {
  schema_version: number;
  active: boolean;
  board?: CalibrationBoard | null;
  camera_source_id?: string | null;
  robot_model_revision?: string | null;
  calibration_tool_id?: string | null;
  observations: Array<Record<string, unknown>>;
  solved_result?: CalibrationResult | null;
  original_error?: string | null;
}

export const actionGroupOrder = [
  "tcp",
  "arc",
  "gripper",
  "control",
  "feedback",
  "compound",
] as const;

export const actionGroupLabels: Record<
  (typeof actionGroupOrder)[number],
  string
> = {
  tcp: "TCP 基础自由度",
  arc: "圆弧复合",
  gripper: "夹爪动作",
  control: "控制动作",
  feedback: "力度反馈",
  compound: "其他复合",
};

export const inputActionCatalog = [
  {
    key: "move_forward_back",
    group: "tcp",
    label: "纵向平移",
    actionType: "float",
    domains: ["position"],
    directions: ["后退", "前进"],
    semantics: "正向前进，负向后退",
    motion: "TCP 沿机器人前后方向直线平移",
    invariant: "左右位置、高度和工具朝向不变",
    reference: "机器人底座",
    prepare: true,
  },
  {
    key: "move_left_right",
    group: "tcp",
    label: "横向平移",
    actionType: "float",
    domains: ["position"],
    directions: ["右移", "左移"],
    semantics: "正向左移，负向右移",
    motion: "TCP 沿机器人左右方向直线平移",
    invariant: "前后位置、高度和工具朝向不变",
    reference: "机器人底座",
    prepare: true,
  },
  {
    key: "move_up_down",
    group: "tcp",
    label: "垂直平移",
    actionType: "float",
    domains: ["position"],
    directions: ["下移", "上移"],
    semantics: "正向上移，负向下移",
    motion: "TCP 沿竖直方向直线平移",
    invariant: "前后位置、左右位置和工具朝向不变",
    reference: "机器人底座",
    prepare: true,
  },
  {
    key: "tool_pitch",
    group: "tcp",
    label: "定点垂直旋转",
    actionType: "float",
    domains: ["orientation"],
    directions: ["往下", "抬起"],
    semantics: "正向抬起，负向往下",
    motion: "工具以 TCP 为中心在竖直平面转动",
    invariant: "TCP 三个位置分量不变",
    reference: "最终 TCP",
    prepare: true,
  },
  {
    key: "tool_yaw",
    group: "tcp",
    label: "定点水平旋转",
    actionType: "float",
    domains: ["orientation"],
    directions: ["向右", "向左"],
    semantics: "正向向左，负向向右",
    motion: "工具以 TCP 为中心在水平面转动",
    invariant: "TCP 三个位置分量不变",
    reference: "最终 TCP",
    prepare: true,
  },
  {
    key: "tool_roll",
    group: "tcp",
    label: "轴向旋转",
    actionType: "float",
    domains: ["orientation"],
    directions: ["顺时针", "逆时针"],
    semantics: "从机械臂后部朝尖端观察，正向逆时针、负向顺时针",
    motion: "工具以 TCP 为中心绕自身前后轴旋转",
    invariant: "TCP 三个位置分量不变",
    reference: "工具自身轴",
    prepare: true,
  },
  {
    key: "front_pitch",
    group: "arc",
    label: "垂直圆弧",
    actionType: "float",
    domains: ["orientation"],
    directions: ["前部往下", "前部抬起"],
    semantics: "正向抬起，负向往下",
    motion: "TCP 绕工具后部枢轴在竖直平面走圆弧",
    components: ["TCP 垂直/纵向平移", "工具垂直姿态"],
    invariant: "后部枢轴位置不变；不是整体上下平移",
    reference: "工具后部枢轴",
    prepare: true,
  },
  {
    key: "horizontal_arc",
    group: "arc",
    label: "水平圆弧",
    actionType: "float",
    domains: ["orientation"],
    directions: ["右旋", "左旋"],
    semantics: "正向左旋，负向右旋",
    motion: "TCP 绕工具后部枢轴在水平面走圆弧",
    components: ["TCP 水平面平移", "工具水平姿态"],
    invariant: "后部枢轴和 TCP 高度不变；不是自身轴旋转",
    reference: "工具后部枢轴",
    prepare: true,
  },
  {
    key: "primary_tool_open",
    group: "gripper",
    label: "夹爪张开",
    actionType: "boolean",
    domains: [],
    semantics: "按下触发",
    motion: "夹爪打开到当前机械臂模型声明的张开位置",
    invariant: "机械臂关节目标不变",
    reference: "夹爪执行器",
    prepare: true,
  },
  {
    key: "primary_tool",
    group: "gripper",
    label: "夹爪开合",
    actionType: "float",
    domains: [],
    directions: ["张开", "闭合"],
    semantics: "0 为张开，1 为闭合",
    motion: "输入 0→1 对应模型声明的张开位置→闭合位置",
    invariant: "机械臂关节目标不变",
    reference: "夹爪执行器",
    prepare: true,
  },
  {
    key: "start_stop",
    group: "control",
    label: "接管控制",
    actionType: "boolean",
    domains: [],
    semantics: "每次按下切换开始或结束",
    motion: "开始时建立本轮相对原点，结束后停止相对控制",
    invariant: "按键本身不产生位移",
    reference: "控制过程",
    prepare: true,
  },
  {
    key: "emergency_stop",
    group: "control",
    label: "急停",
    actionType: "boolean",
    domains: [],
    semantics: "按下触发",
    motion: "结束当前控制过程",
    invariant: "不产生新的相对目标；重新接管后可继续",
    reference: "控制过程",
    prepare: true,
  },
  {
    key: "tool_axis_translation",
    group: "compound",
    label: "工具轴向平移",
    actionType: "float",
    domains: ["position"],
    directions: ["向后", "向前"],
    semantics: "正向由工具后部向尖端，负向相反",
    motion: "TCP 沿当前工具前后轴直线平移",
    components: ["工具轴方向平移"],
    invariant: "工具朝向及垂直于工具轴的位移不变",
    reference: "最终工具轴",
    prepare: true,
  },
  {
    key: "tool_helical_motion",
    group: "compound",
    label: "工具轴向螺旋",
    actionType: "float",
    domains: ["position", "orientation"],
    directions: ["后退并顺时针", "前进并逆时针"],
    semantics: "正向前进并逆时针旋转，负向反向返回",
    motion: "TCP 沿当前工具轴平移并同时绕该轴旋转",
    components: ["工具轴方向平移", "工具自身轴旋转"],
    invariant: "垂直于工具轴的位移不变",
    reference: "最终工具轴",
    prepare: true,
  },
] as const;

export const feedbackActionCatalog = [
  {
    key: "primary_tool",
    item: "primary_tool_feedback",
    group: "feedback",
    label: "夹爪力度反馈",
    semantics: "输出 0–100 的力度值",
    motion: "不驱动机械臂，只发送到当前反馈目标",
    invariant: "输入设备与反馈设备相互独立",
    reference: "所选反馈能力",
    prepare: false,
  },
] as const;

export interface InputSimulationState {
  schema_version: number;
  active: boolean;
  item?: InputSimulationItem | null;
  phase?: string | null;
  elapsed_s?: number | null;
}

export interface ActionFeedback {
  schema_version: number;
  sequence: number;
  sample_time_ns: number;
  action: string;
  strength_percent: number;
}

export interface TransformedControlFrame {
  schema_version: number;
  sequence: number;
  source_time_ns: number;
  transformed_time_ns: number;
  control_session_id?: number | null;
  active: boolean;
  translation_m: [number, number, number];
  front_pitch_rad: number;
  horizontal_arc_rad: number;
  tool_pitch_rad: number;
  tool_yaw_rad: number;
  tool_roll_rad: number;
  tool_axis_translation_m: number;
  tool_helical_translation_m: number;
  tool_helical_roll_rad: number;
  actuator_actions: ActuatorActions;
}
