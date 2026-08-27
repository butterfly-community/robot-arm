export const schemaVersion = 1;

export type Json =
  | null
  | boolean
  | number
  | string
  | Json[]
  | { [key: string]: Json };
export type Namespace = "tracking" | "spatial" | "motion" | "arm-execution";
export type RequestAction =
  | "apply"
  | "cancel"
  | "select"
  | "unselect"
  | "connect"
  | "disconnect"
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
    positions_rad: Record<string, number>;
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
