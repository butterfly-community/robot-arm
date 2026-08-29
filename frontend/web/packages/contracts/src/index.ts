export const schemaVersion = 2;
export const virtualFeedbackTarget = {
  sourceId: "virtual-feedback",
  capabilityPath: "feedback/virtual",
} as const;

export type Json =
  null | boolean | number | string | Json[] | { [key: string]: Json };
export type Namespace = "tracking" | "spatial" | "motion" | "arm-execution";
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
  control_mode: "relative" | "manual";
  collision_checking: boolean;
  self_collision_tolerance_m: number;
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

export interface ControlInputFrame {
  schema_version: number;
  sequence: number;
  source_time_ns: number;
  received_time_ns: number;
  control_active: BooleanActionSample;
  confirm_origin: BooleanActionSample;
  primary_tool_open: BooleanActionSample;
  primary_tool: FloatActionSample;
  move_forward_back: FloatActionSample;
  move_left_right: FloatActionSample;
  move_up_down: FloatActionSample;
  front_pitch: FloatActionSample;
  horizontal_arc: FloatActionSample;
}

export interface ActionFeedback {
  schema_version: number;
  sequence: number;
  sample_time_ns: number;
  action: string;
  strength_percent: number;
}

export interface RelativeToolMotion {
  schema_version: number;
  sequence: number;
  source_time_ns: number;
  transformed_time_ns: number;
  control_session_id?: number | null;
  active: boolean;
  translation_m: [number, number, number];
  front_pitch_rad: number;
  horizontal_arc_rad: number;
  primary_tool_open: boolean;
  primary_tool_value: number;
}
