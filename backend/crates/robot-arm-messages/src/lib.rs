use std::{collections::BTreeMap, sync::Arc};

use arrow::{
    array::{Array, ArrayRef, StringArray, StructArray, UInt32Array},
    datatypes::{DataType, Field, Fields},
};
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use thiserror::Error;

pub const SCHEMA_VERSION: u32 = 2;
pub const DEFAULT_ACTION_TRANSLATION_M_PER_S: f64 = 0.01;
pub const DEFAULT_ACTION_ARC_RAD_PER_S: f64 = 0.10;

#[derive(Debug, Error)]
pub enum ArrowCodecError {
    #[error("expected one non-null UTF-8 JSON value")]
    InvalidShape,
    #[error("expected a UTF-8 Arrow array")]
    InvalidArrowType,
    #[error("message schema version {actual} is not supported; expected {expected}")]
    UnsupportedVersion { actual: u32, expected: u32 },
    #[error("JSON encode failed: {0}")]
    Encode(#[from] serde_json::Error),
}

pub fn to_arrow<T: Serialize>(value: &T) -> Result<ArrayRef, ArrowCodecError> {
    let json = serde_json::to_string(value)?;
    let fields = Fields::from(vec![
        Field::new("schema_version", DataType::UInt32, false),
        Field::new("payload_json", DataType::Utf8, false),
    ]);
    Ok(Arc::new(StructArray::new(
        fields,
        vec![
            Arc::new(UInt32Array::from(vec![SCHEMA_VERSION])) as ArrayRef,
            Arc::new(StringArray::from(vec![json])) as ArrayRef,
        ],
        None,
    )))
}

pub fn from_arrow<T: DeserializeOwned>(array: &dyn Array) -> Result<T, ArrowCodecError> {
    let structure = array
        .as_any()
        .downcast_ref::<StructArray>()
        .ok_or(ArrowCodecError::InvalidArrowType)?;
    if structure.len() != 1 || structure.is_null(0) {
        return Err(ArrowCodecError::InvalidShape);
    }
    let versions = structure
        .column_by_name("schema_version")
        .and_then(|value| value.as_any().downcast_ref::<UInt32Array>())
        .ok_or(ArrowCodecError::InvalidArrowType)?;
    let strings = structure
        .column_by_name("payload_json")
        .and_then(|value| value.as_any().downcast_ref::<StringArray>())
        .ok_or(ArrowCodecError::InvalidArrowType)?;
    if versions.is_null(0) || strings.is_null(0) {
        return Err(ArrowCodecError::InvalidShape);
    }
    let actual = versions.value(0);
    if actual != SCHEMA_VERSION {
        return Err(ArrowCodecError::UnsupportedVersion {
            actual,
            expected: SCHEMA_VERSION,
        });
    }
    Ok(serde_json::from_str(strings.value(0))?)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RequestAction {
    Apply,
    Cancel,
    Select,
    Unselect,
    Connect,
    Disconnect,
    Discover,
    Refresh,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RequestResult<T> {
    pub schema_version: u32,
    pub request_id: String,
    pub acknowledged_action: RequestAction,
    pub value: Option<T>,
    pub original_error: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ServiceState {
    pub schema_version: u32,
    pub build_version: String,
    pub config_version: u64,
    pub running: bool,
    pub has_input: bool,
    pub has_output: bool,
    pub last_error: Option<String>,
    pub updated_at_ns: i64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ServiceReadiness {
    pub service_id: String,
    pub reported_ready: bool,
    pub ready: bool,
    pub blocked_by: Vec<String>,
    pub state: Option<ServiceState>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SystemReadiness {
    pub schema_version: u32,
    pub ready: bool,
    pub services: Vec<ServiceReadiness>,
    pub updated_at_ns: i64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct InputDriverInfo {
    pub driver_id: String,
    pub display_name: String,
    pub version: String,
    pub original_error: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct InputSourceInfo {
    pub source_id: String,
    pub driver_id: String,
    pub device_id: String,
    pub display_name: String,
    pub vendor_id: Option<String>,
    pub product_id: Option<String>,
    pub serial: Option<String>,
    pub position_capable: bool,
    pub orientation_capable: bool,
    pub action_capable: bool,
    pub active: bool,
    pub available_components: Vec<InputComponentInfo>,
    pub available_feedback_capabilities: Vec<InputFeedbackCapabilityInfo>,
    pub original_error: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct InputComponentInfo {
    pub path: String,
    pub action_type: ActionType,
    pub localized_name: Option<String>,
    pub definition_source: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct InputFeedbackCapabilityInfo {
    pub path: String,
    pub localized_name: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ActionType {
    Boolean,
    Float,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct InputBindingState {
    pub action: String,
    pub action_type: ActionType,
    pub source_id: Option<String>,
    pub invert: bool,
    pub configured_components: Vec<String>,
    pub active: bool,
    pub value: f64,
    pub applicable: bool,
    pub original_error: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SelectedInputSourceState {
    pub driver_id: String,
    pub device_id: String,
    pub source_id: String,
    pub active: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct InputStreamDiagnostics {
    pub source_id: String,
    pub received_frames: u64,
    pub last_source_time_ns: Option<i64>,
    pub last_received_time_ns: Option<i64>,
    pub observed_rate_hz: Option<f64>,
    pub observed_jitter_ms: Option<f64>,
    pub sequence_gaps: u64,
    pub duplicate_reports: u64,
    pub source_error_count: u64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct InputDiscoveryState {
    pub schema_version: u32,
    pub drivers: Vec<InputDriverInfo>,
    pub sources: Vec<InputSourceInfo>,
    pub position_source_id: Option<String>,
    pub position_source: Option<SelectedInputSourceState>,
    pub orientation_source_id: Option<String>,
    pub orientation_source: Option<SelectedInputSourceState>,
    pub bindings: Vec<InputBindingState>,
    pub feedback_bindings: Vec<ActionFeedbackBindingState>,
    pub live_component_values: BTreeMap<String, BTreeMap<String, f64>>,
    pub virtual_feedback: Option<ActionFeedback>,
    pub diagnostics: Vec<InputStreamDiagnostics>,
    pub simulation: InputSimulationState,
    pub service: ServiceState,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct InputSimulationState {
    pub schema_version: u32,
    pub active: bool,
    pub phase: Option<String>,
    pub elapsed_s: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct InputSimulationRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub enabled: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SelectPoseSourceRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub action: RequestAction,
    pub driver_id: String,
    pub device_id: String,
    pub source_id: String,
    pub component: PoseComponent,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PoseComponent {
    Position,
    Orientation,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ActionBinding {
    pub action: String,
    pub action_type: ActionType,
    pub source_id: String,
    pub component_paths: Vec<String>,
    pub invert: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ActionFeedbackBinding {
    pub action: String,
    pub source_id: String,
    pub capability_path: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ActionFeedbackBindingState {
    pub action: String,
    pub source_id: Option<String>,
    pub capability_path: Option<String>,
    pub applicable: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ApplyInputBindingsRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub bindings: Vec<ActionBinding>,
    pub feedback_bindings: Vec<ActionFeedbackBinding>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PoseFlags {
    pub position_valid: bool,
    pub position_tracked: bool,
    pub orientation_valid: bool,
    pub orientation_tracked: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AbsolutePoseFrame {
    pub schema_version: u32,
    pub sequence: u64,
    pub source_time_ns: i64,
    pub received_time_ns: i64,
    pub position_source_id: Option<String>,
    pub orientation_source_id: Option<String>,
    pub position_source_capable: bool,
    pub orientation_source_capable: bool,
    pub reference_space: String,
    pub position_m: [f64; 3],
    pub orientation_xyzw: [f64; 4],
    pub flags: PoseFlags,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BooleanActionSample {
    pub is_active: bool,
    pub changed_since_last_sync: bool,
    pub value: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FloatActionSample {
    pub is_active: bool,
    pub changed_since_last_sync: bool,
    pub value: f64,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ActuatorActions {
    pub primary_tool_open: BooleanActionSample,
    pub primary_tool: FloatActionSample,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ControlInputFrame {
    pub schema_version: u32,
    pub sequence: u64,
    pub source_time_ns: i64,
    pub received_time_ns: i64,
    pub start_stop: BooleanActionSample,
    pub emergency_stop: BooleanActionSample,
    pub actuator_actions: ActuatorActions,
    pub move_forward_back: FloatActionSample,
    pub move_left_right: FloatActionSample,
    pub move_up_down: FloatActionSample,
    pub front_pitch: FloatActionSample,
    pub horizontal_arc: FloatActionSample,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpatialComponentSwitches {
    pub translation: bool,
    pub front_pitch: bool,
    pub horizontal_arc: bool,
}

impl Default for SpatialComponentSwitches {
    fn default() -> Self {
        Self {
            translation: true,
            front_pitch: true,
            horizontal_arc: true,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpatialConfigState {
    pub schema_version: u32,
    pub config_version: u64,
    pub position_source_id: Option<String>,
    pub orientation_source_id: Option<String>,
    pub base_from_tracking_axes: [[f64; 3]; 3],
    pub translation_scale: f64,
    pub action_translation_m_per_s: Option<f64>,
    pub action_arc_rad_per_s: Option<f64>,
    pub switches: SpatialComponentSwitches,
    pub control_session_id: Option<u64>,
}

impl Default for SpatialConfigState {
    fn default() -> Self {
        Self {
            schema_version: SCHEMA_VERSION,
            config_version: 1,
            position_source_id: None,
            orientation_source_id: None,
            base_from_tracking_axes: [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            translation_scale: 0.5,
            action_translation_m_per_s: Some(DEFAULT_ACTION_TRANSLATION_M_PER_S),
            action_arc_rad_per_s: Some(DEFAULT_ACTION_ARC_RAD_PER_S),
            switches: SpatialComponentSwitches::default(),
            control_session_id: None,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TransformedControlFrame {
    pub schema_version: u32,
    pub sequence: u64,
    pub source_time_ns: i64,
    pub transformed_time_ns: i64,
    pub control_session_id: Option<u64>,
    pub active: bool,
    pub translation_m: [f64; 3],
    pub front_pitch_rad: f64,
    pub horizontal_arc_rad: f64,
    pub actuator_actions: ActuatorActions,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SpatialConfigPatch {
    pub base_from_tracking_axes: Option<[[f64; 3]; 3]>,
    pub translation_scale: Option<f64>,
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        with = "serde_with::rust::double_option"
    )]
    pub action_translation_m_per_s: Option<Option<f64>>,
    #[serde(
        default,
        skip_serializing_if = "Option::is_none",
        with = "serde_with::rust::double_option"
    )]
    pub action_arc_rad_per_s: Option<Option<f64>>,
    pub switches: Option<SpatialComponentSwitches>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct UpdateSpatialConfigRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub patch: SpatialConfigPatch,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NumericFieldSchema {
    pub key: String,
    pub label: String,
    pub unit: String,
    pub minimum: Option<f64>,
    pub maximum: Option<f64>,
    pub required: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointMetadata {
    pub key: String,
    pub label: String,
    pub unit: String,
    pub minimum: f64,
    pub maximum: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ActuatorMetadata {
    pub key: String,
    pub label: String,
    pub unit: String,
    pub minimum: f64,
    pub maximum: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub visualization_joint_key: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct NamedMotionTarget {
    pub key: String,
    pub label: String,
    pub joint_positions_rad: BTreeMap<String, f64>,
    pub actuator_positions_rad: BTreeMap<String, f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VisualizationManifest {
    pub manifest_hash: String,
    pub root_path: String,
    pub files: Vec<String>,
    pub link_materials: BTreeMap<String, LinkMaterial>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinkMaterial {
    pub color_rgb: [f64; 3],
    pub metalness: f64,
    pub roughness: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RobotModelInfo {
    pub schema_version: u32,
    pub model_id: String,
    pub model_revision: String,
    pub display_name: String,
    pub base_frame: String,
    pub tcp_frame: String,
    pub joints: Vec<JointMetadata>,
    pub tool_actuators: Vec<ActuatorMetadata>,
    pub named_targets: Vec<NamedMotionTarget>,
    pub motion_options: Vec<NumericFieldSchema>,
    pub diagnostics: Vec<NumericFieldSchema>,
    pub visualization: VisualizationManifest,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointPosition {
    pub joint_key: String,
    pub position_rad: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ActuatorPosition {
    pub actuator_key: String,
    pub position_rad: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub model_revision: String,
    pub joints: Vec<JointPosition>,
    pub actuators: Vec<ActuatorPosition>,
    pub options: BTreeMap<String, f64>,
    pub action: RequestAction,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RequestState {
    Idle,
    Planning,
    Executing,
    Succeeded,
    Failed,
    Cancelled,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionStatus {
    pub schema_version: u32,
    pub request_id: String,
    pub acknowledged_action: String,
    pub state: RequestState,
    pub backend_name: String,
    pub result_code: Option<String>,
    pub result_message: Option<String>,
    pub trajectory_points: Option<u64>,
    pub planned_duration_s: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ToolActuatorRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub model_revision: String,
    pub actuator_key: String,
    pub position_rad: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ToolActuatorStatus {
    pub schema_version: u32,
    pub request_id: String,
    pub actuator_key: String,
    pub state: RequestState,
    pub result_code: Option<String>,
    pub result_message: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlMode {
    Relative,
    Manual,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SetControlModeRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub mode: ControlMode,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ToolPose {
    pub frame: String,
    pub position_m: [f64; 3],
    pub orientation_xyzw: [f64; 4],
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiagnosticValue {
    pub key: String,
    pub value: serde_json::Value,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionState {
    pub schema_version: u32,
    pub control_mode: ControlMode,
    pub current_tool_pose: Option<ToolPose>,
    pub target_tool_pose: Option<ToolPose>,
    pub control_session_id: Option<u64>,
    pub latest_motion: Option<MotionStatus>,
    pub latest_actuator: Option<ToolActuatorStatus>,
    pub diagnostics: Vec<DiagnosticValue>,
    pub service: ServiceState,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ModelAssetRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub model_revision: String,
    pub manifest_hash: String,
    pub relative_path: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ModelAssetResponse {
    pub schema_version: u32,
    pub request_id: String,
    pub model_revision: String,
    pub manifest_hash: String,
    pub relative_path: String,
    pub mime_type: Option<String>,
    pub content_hash: Option<String>,
    pub content: Option<Vec<u8>>,
    pub original_error: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArmCommand {
    pub schema_version: u32,
    pub sequence: u64,
    pub controller_time_ns: i64,
    pub model_revision: String,
    pub joints_rad: Vec<f64>,
    pub actuators_rad: Vec<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FeedbackSource {
    Software,
    Hardware,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArmState {
    pub schema_version: u32,
    pub sequence: u64,
    pub sample_time_ns: i64,
    pub model_revision: String,
    pub joints_rad: Vec<f64>,
    pub actuators_rad: Vec<f64>,
    pub feedback_source: FeedbackSource,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ActuatorTelemetry {
    pub actuator_key: String,
    pub voltage_mv: u16,
    pub current_ma: u16,
    pub power_mw: u16,
    pub command_power_limit_mw: Option<u16>,
    pub temperature_raw: u16,
    pub status: u8,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArmTelemetry {
    pub schema_version: u32,
    pub sequence: u64,
    pub sample_time_ns: i64,
    pub model_revision: String,
    pub actuators: Vec<ActuatorTelemetry>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ActionFeedback {
    pub schema_version: u32,
    pub sequence: u64,
    pub sample_time_ns: i64,
    pub action: String,
    pub strength_percent: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ConnectionFieldSchema {
    pub key: String,
    pub label: String,
    pub field_type: String,
    pub required: bool,
    pub options: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExecutionInfo {
    pub schema_version: u32,
    pub adapter_id: String,
    pub adapter_revision: String,
    pub model_revision: String,
    pub connection_fields: Vec<ConnectionFieldSchema>,
    pub actuator_labels: Vec<String>,
    pub parameter_fields: Vec<NumericFieldSchema>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExecutionEndpoint {
    pub key: String,
    pub label: String,
    pub properties: BTreeMap<String, String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ExecutionRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub action: RequestAction,
    pub fields: BTreeMap<String, String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ParameterValue {
    pub actuator_key: String,
    pub field_key: String,
    pub value: Option<f64>,
    pub unit: String,
    pub read_time_ns: i64,
    pub original_error: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ExecutionTransportState {
    pub schema_version: u32,
    pub discovered_endpoints: Vec<ExecutionEndpoint>,
    pub selected_endpoint: Option<String>,
    pub connected: bool,
    pub last_error: Option<String>,
    pub last_command: Option<ArmCommand>,
    pub feedback_summary: Option<String>,
    pub parameter_error: Option<String>,
    pub parameter_values: Vec<ParameterValue>,
    pub latest_request_id: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RecordingManifest {
    pub schema_version: u32,
    pub build_versions: BTreeMap<String, String>,
    pub message_schema_version: u32,
    pub model_id: Option<String>,
    pub model_hash: Option<String>,
    pub config_versions: BTreeMap<String, u64>,
    pub coordinate_convention: String,
    pub units: BTreeMap<String, String>,
    pub started_at_ns: i64,
    pub ended_at_ns: i64,
    pub sources: Vec<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arrow_json_round_trip_preserves_dynamic_joint_shape() {
        let state = ArmState {
            schema_version: SCHEMA_VERSION,
            sequence: 7,
            sample_time_ns: 10,
            model_revision: "fixture-4-axis".into(),
            joints_rad: vec![0.1, 0.2, 0.3, 0.4],
            actuators_rad: vec![],
            feedback_source: FeedbackSource::Software,
        };
        let encoded = to_arrow(&state).unwrap();
        let decoded: ArmState = from_arrow(encoded.as_ref()).unwrap();
        assert_eq!(decoded, state);
    }

    #[test]
    fn arrow_envelope_rejects_a_different_schema_version() {
        let fields = Fields::from(vec![
            Field::new("schema_version", DataType::UInt32, false),
            Field::new("payload_json", DataType::Utf8, false),
        ]);
        let array = StructArray::new(
            fields,
            vec![
                Arc::new(UInt32Array::from(vec![SCHEMA_VERSION + 1])) as ArrayRef,
                Arc::new(StringArray::from(vec!["{}"])) as ArrayRef,
            ],
            None,
        );
        assert!(matches!(
            from_arrow::<ServiceState>(&array),
            Err(ArrowCodecError::UnsupportedVersion { .. })
        ));
    }

    #[test]
    fn pose_contract_has_no_action_or_robot_fields() {
        let pose = AbsolutePoseFrame {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            source_time_ns: 2,
            received_time_ns: 3,
            position_source_id: Some("runtime/left".into()),
            orientation_source_id: Some("runtime/right".into()),
            position_source_capable: true,
            orientation_source_capable: true,
            reference_space: "local".into(),
            position_m: [1.0, 2.0, 3.0],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
            flags: PoseFlags::default(),
        };
        let json = serde_json::to_value(pose).unwrap();
        assert!(json.get("start_stop").is_none());
        assert!(json.get("joints_rad").is_none());
        assert!(json.get("translation_scale").is_none());
    }

    #[test]
    fn action_feedback_is_device_independent_and_uses_percent_strength() {
        let feedback = ActionFeedback {
            schema_version: SCHEMA_VERSION,
            sequence: 1,
            sample_time_ns: 2,
            action: "primary_tool".into(),
            strength_percent: 75.0,
        };
        let json = serde_json::to_value(feedback).unwrap();
        assert_eq!(json["strength_percent"], 75.0);
        assert!(json.get("device_id").is_none());
        assert!(json.get("vendor_id").is_none());
        assert!(json.get("model_revision").is_none());
    }

    #[test]
    fn spatial_patch_distinguishes_missing_fields_from_explicit_null() {
        let missing: SpatialConfigPatch = serde_json::from_str("{}").unwrap();
        assert_eq!(missing.action_translation_m_per_s, None);
        assert_eq!(missing.action_arc_rad_per_s, None);

        let cleared: SpatialConfigPatch = serde_json::from_str(
            r#"{
                "action_translation_m_per_s": null,
                "action_arc_rad_per_s": null
            }"#,
        )
        .unwrap();
        assert_eq!(cleared.action_translation_m_per_s, Some(None));
        assert_eq!(cleared.action_arc_rad_per_s, Some(None));
    }

    #[test]
    fn generic_model_supports_multiple_actuators_and_arbitrary_joint_count() {
        let model = RobotModelInfo {
            schema_version: SCHEMA_VERSION,
            model_id: "fixture".into(),
            model_revision: "fixture-v1".into(),
            display_name: "Fixture".into(),
            base_frame: "base".into(),
            tcp_frame: "tool".into(),
            joints: (0..4)
                .map(|index| JointMetadata {
                    key: format!("axis-{index}"),
                    label: format!("Axis {index}"),
                    unit: "rad".into(),
                    minimum: -1.0,
                    maximum: 1.0,
                })
                .collect(),
            tool_actuators: vec![
                ActuatorMetadata {
                    key: "tool-a".into(),
                    label: "Tool A".into(),
                    unit: "rad".into(),
                    minimum: 0.0,
                    maximum: 1.0,
                    visualization_joint_key: Some("tool_joint_a".into()),
                },
                ActuatorMetadata {
                    key: "tool-b".into(),
                    label: "Tool B".into(),
                    unit: "rad".into(),
                    minimum: -1.0,
                    maximum: 1.0,
                    visualization_joint_key: Some("tool_joint_b".into()),
                },
            ],
            named_targets: vec![NamedMotionTarget {
                key: "ready".into(),
                label: "Ready".into(),
                joint_positions_rad: BTreeMap::from([("axis-0".into(), 0.25)]),
                actuator_positions_rad: BTreeMap::from([("tool-a".into(), 0.0)]),
            }],
            motion_options: vec![],
            diagnostics: vec![],
            visualization: VisualizationManifest {
                manifest_hash: "none".into(),
                root_path: "model.urdf".into(),
                files: vec!["model.urdf".into()],
                link_materials: BTreeMap::from([(
                    "fixture-link".into(),
                    LinkMaterial {
                        color_rgb: [0.2, 0.4, 0.6],
                        metalness: 0.3,
                        roughness: 0.7,
                    },
                )]),
            },
        };
        let encoded = to_arrow(&model).unwrap();
        let decoded: RobotModelInfo = from_arrow(encoded.as_ref()).unwrap();
        assert_eq!(decoded.joints.len(), 4);
        assert_eq!(decoded.tool_actuators.len(), 2);
        assert_eq!(decoded.named_targets[0].joint_positions_rad["axis-0"], 0.25);
        assert_eq!(
            decoded.named_targets[0].actuator_positions_rad["tool-a"],
            0.0
        );
        assert_eq!(
            decoded.visualization.link_materials["fixture-link"].color_rgb,
            [0.2, 0.4, 0.6]
        );
    }

    #[test]
    fn recording_manifest_round_trip_keeps_replay_provenance() {
        let manifest = RecordingManifest {
            schema_version: SCHEMA_VERSION,
            build_versions: BTreeMap::from([("source".into(), "1.0.0".into())]),
            message_schema_version: SCHEMA_VERSION,
            model_id: Some("fixture".into()),
            model_hash: Some("sha256:fixture".into()),
            config_versions: BTreeMap::from([("spatial".into(), 4)]),
            coordinate_convention: "+X forward, +Y left, +Z up".into(),
            units: BTreeMap::from([("translation".into(), "m".into())]),
            started_at_ns: 10,
            ended_at_ns: 20,
            sources: vec!["source/absolute_pose".into()],
        };
        let decoded: RecordingManifest = from_arrow(to_arrow(&manifest).unwrap().as_ref()).unwrap();
        assert_eq!(decoded, manifest);
    }
}
