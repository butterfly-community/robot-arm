use std::{collections::BTreeMap, sync::Arc};

use arrow::{
    array::{Array, ArrayRef, BinaryArray, StringArray, StructArray, UInt32Array},
    datatypes::{DataType, Field, Fields},
};
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use thiserror::Error;

pub const SCHEMA_VERSION: u32 = 3;
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
    #[error("camera frame payload is invalid: {0}")]
    InvalidCameraFrame(String),
}

const CAMERA_FRAME_METADATA_FIELD: &str = "metadata_json";
const CAMERA_COLOR_DATA_FIELD: &str = "color_data";
const CAMERA_DEPTH_DATA_FIELD: &str = "depth_data";

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

/// A device-independent stream profile reported by a camera driver.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CameraStreamProfile {
    pub key: String,
    pub stream: CameraStreamKind,
    pub width: u32,
    pub height: u32,
    pub frames_per_second: u32,
    pub pixel_format: String,
    #[serde(default)]
    pub is_default: bool,
    #[serde(default = "default_true")]
    pub available: bool,
    #[serde(default)]
    pub unavailable_reason: Option<String>,
}

const fn default_true() -> bool {
    true
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CameraStreamKind {
    Color,
    Depth,
}

/// A driver-owned setting exposed without leaking the vendor SDK into downstream nodes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CameraDriverParameterKind {
    Boolean,
    Integer,
    Number,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraDriverParameterInfo {
    pub key: String,
    pub display_name: String,
    pub sensor_name: String,
    pub kind: CameraDriverParameterKind,
    pub current_value: f64,
    pub default_value: f64,
    pub minimum: f64,
    pub maximum: f64,
    pub step: f64,
    pub read_only: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraDriverExtensionInfo {
    pub namespace: String,
    pub display_name: String,
    pub parameters: Vec<CameraDriverParameterInfo>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraDriverParameterValue {
    pub namespace: String,
    pub key: String,
    pub value: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraSourceConfiguration {
    pub source_id: String,
    pub color_profile_key: String,
    pub depth_profile_key: String,
    pub output_frames_per_second: f64,
    pub driver_parameters: Vec<CameraDriverParameterValue>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraIntrinsics {
    pub width: u32,
    pub height: u32,
    pub focal_length_px: [f64; 2],
    pub principal_point_px: [f64; 2],
    pub distortion_model: String,
    pub distortion: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraImagePlane {
    pub width: u32,
    pub height: u32,
    pub stride_bytes: u32,
    pub pixel_format: String,
    pub frame_id: String,
    pub data: Vec<u8>,
}

impl CameraImagePlane {
    fn metadata(&self) -> CameraImagePlaneMetadata {
        CameraImagePlaneMetadata {
            width: self.width,
            height: self.height,
            stride_bytes: self.stride_bytes,
            pixel_format: self.pixel_format.clone(),
            frame_id: self.frame_id.clone(),
        }
    }

    fn validate(&self, label: &str) -> Result<(), ArrowCodecError> {
        if self.width == 0 || self.height == 0 || self.stride_bytes == 0 {
            return Err(ArrowCodecError::InvalidCameraFrame(format!(
                "{label} dimensions and stride must be positive"
            )));
        }
        let expected = usize::try_from(self.stride_bytes)
            .ok()
            .and_then(|stride| stride.checked_mul(self.height as usize))
            .ok_or_else(|| {
                ArrowCodecError::InvalidCameraFrame(format!(
                    "{label} dimensions overflow the host address space"
                ))
            })?;
        if self.data.len() != expected {
            return Err(ArrowCodecError::InvalidCameraFrame(format!(
                "{label} payload has {} bytes, expected {expected}",
                self.data.len()
            )));
        }
        Ok(())
    }

    fn validate_format(
        &self,
        label: &str,
        formats: &[(&str, usize)],
    ) -> Result<(), ArrowCodecError> {
        let bytes_per_pixel = formats
            .iter()
            .find_map(|(format, bytes)| (self.pixel_format == *format).then_some(*bytes))
            .ok_or_else(|| {
                ArrowCodecError::InvalidCameraFrame(format!(
                    "{label} pixel format {} is not supported",
                    self.pixel_format
                ))
            })?;
        let packed_stride = (self.width as usize)
            .checked_mul(bytes_per_pixel)
            .ok_or_else(|| {
                ArrowCodecError::InvalidCameraFrame(format!(
                    "{label} packed row size overflows the host address space"
                ))
            })?;
        if (self.stride_bytes as usize) < packed_stride {
            return Err(ArrowCodecError::InvalidCameraFrame(format!(
                "{label} stride is smaller than its packed pixel row"
            )));
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraFrameBundle {
    pub schema_version: u32,
    pub sequence: u64,
    pub source_id: String,
    pub device_time_ns: i64,
    pub device_time_domain: String,
    pub received_time_ns: i64,
    pub color: CameraImagePlane,
    /// Depth registered to the color image plane by the camera driver adapter.
    pub aligned_depth: CameraImagePlane,
    /// Intrinsics of the shared color/aligned-depth image plane.
    pub intrinsics: CameraIntrinsics,
    pub depth_scale_m: f64,
    /// Camera pose calibration snapshot used with this exact RGB-D frame.
    pub calibration: Option<DepthCameraCalibration>,
}

/// Latest color frame for live Web display, independent of RGB-D output throttling.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraRawVideoFrame {
    pub schema_version: u32,
    pub sequence: u64,
    pub source_id: String,
    pub received_time_ns: i64,
    pub color: CameraImagePlane,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct CameraImagePlaneMetadata {
    width: u32,
    height: u32,
    stride_bytes: u32,
    pixel_format: String,
    frame_id: String,
}

impl CameraImagePlaneMetadata {
    fn with_data(self, data: Vec<u8>) -> CameraImagePlane {
        CameraImagePlane {
            width: self.width,
            height: self.height,
            stride_bytes: self.stride_bytes,
            pixel_format: self.pixel_format,
            frame_id: self.frame_id,
            data,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct CameraFrameBundleMetadata {
    schema_version: u32,
    sequence: u64,
    source_id: String,
    device_time_ns: i64,
    device_time_domain: String,
    received_time_ns: i64,
    color: CameraImagePlaneMetadata,
    aligned_depth: CameraImagePlaneMetadata,
    intrinsics: CameraIntrinsics,
    depth_scale_m: f64,
    calibration: Option<DepthCameraCalibration>,
}

impl CameraFrameBundleMetadata {
    fn from_bundle(bundle: &CameraFrameBundle) -> Self {
        Self {
            schema_version: bundle.schema_version,
            sequence: bundle.sequence,
            source_id: bundle.source_id.clone(),
            device_time_ns: bundle.device_time_ns,
            device_time_domain: bundle.device_time_domain.clone(),
            received_time_ns: bundle.received_time_ns,
            color: bundle.color.metadata(),
            aligned_depth: bundle.aligned_depth.metadata(),
            intrinsics: bundle.intrinsics.clone(),
            depth_scale_m: bundle.depth_scale_m,
            calibration: bundle.calibration.clone(),
        }
    }

    fn with_data(self, color_data: Vec<u8>, depth_data: Vec<u8>) -> CameraFrameBundle {
        CameraFrameBundle {
            schema_version: self.schema_version,
            sequence: self.sequence,
            source_id: self.source_id,
            device_time_ns: self.device_time_ns,
            device_time_domain: self.device_time_domain,
            received_time_ns: self.received_time_ns,
            color: self.color.with_data(color_data),
            aligned_depth: self.aligned_depth.with_data(depth_data),
            intrinsics: self.intrinsics,
            depth_scale_m: self.depth_scale_m,
            calibration: self.calibration,
        }
    }
}

/// Encode RGB-D data without serializing the image buffers through JSON/Base64.
pub fn camera_frame_to_arrow(bundle: &CameraFrameBundle) -> Result<ArrayRef, ArrowCodecError> {
    if bundle.schema_version != SCHEMA_VERSION {
        return Err(ArrowCodecError::UnsupportedVersion {
            actual: bundle.schema_version,
            expected: SCHEMA_VERSION,
        });
    }
    validate_camera_frame(bundle)?;
    let metadata = serde_json::to_string(&CameraFrameBundleMetadata::from_bundle(bundle))?;
    let fields = Fields::from(vec![
        Field::new("schema_version", DataType::UInt32, false),
        Field::new(CAMERA_FRAME_METADATA_FIELD, DataType::Utf8, false),
        Field::new(CAMERA_COLOR_DATA_FIELD, DataType::Binary, false),
        Field::new(CAMERA_DEPTH_DATA_FIELD, DataType::Binary, false),
    ]);
    Ok(Arc::new(StructArray::new(
        fields,
        vec![
            Arc::new(UInt32Array::from(vec![SCHEMA_VERSION])) as ArrayRef,
            Arc::new(StringArray::from(vec![metadata])) as ArrayRef,
            Arc::new(BinaryArray::from_vec(vec![bundle.color.data.as_slice()])) as ArrayRef,
            Arc::new(BinaryArray::from_vec(vec![
                bundle.aligned_depth.data.as_slice(),
            ])) as ArrayRef,
        ],
        None,
    )))
}

pub fn camera_frame_from_arrow(array: &dyn Array) -> Result<CameraFrameBundle, ArrowCodecError> {
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
    let metadata = structure
        .column_by_name(CAMERA_FRAME_METADATA_FIELD)
        .and_then(|value| value.as_any().downcast_ref::<StringArray>())
        .ok_or(ArrowCodecError::InvalidArrowType)?;
    let colors = structure
        .column_by_name(CAMERA_COLOR_DATA_FIELD)
        .and_then(|value| value.as_any().downcast_ref::<BinaryArray>())
        .ok_or(ArrowCodecError::InvalidArrowType)?;
    let depths = structure
        .column_by_name(CAMERA_DEPTH_DATA_FIELD)
        .and_then(|value| value.as_any().downcast_ref::<BinaryArray>())
        .ok_or(ArrowCodecError::InvalidArrowType)?;
    if versions.is_null(0) || metadata.is_null(0) || colors.is_null(0) || depths.is_null(0) {
        return Err(ArrowCodecError::InvalidShape);
    }
    let actual = versions.value(0);
    if actual != SCHEMA_VERSION {
        return Err(ArrowCodecError::UnsupportedVersion {
            actual,
            expected: SCHEMA_VERSION,
        });
    }
    let metadata: CameraFrameBundleMetadata = serde_json::from_str(metadata.value(0))?;
    if metadata.schema_version != actual {
        return Err(ArrowCodecError::UnsupportedVersion {
            actual: metadata.schema_version,
            expected: actual,
        });
    }
    let bundle = metadata.with_data(colors.value(0).to_vec(), depths.value(0).to_vec());
    validate_camera_frame(&bundle)?;
    Ok(bundle)
}

fn validate_camera_frame(bundle: &CameraFrameBundle) -> Result<(), ArrowCodecError> {
    bundle.color.validate("color")?;
    bundle.color.validate_format(
        "color",
        &[
            ("rgb8", 3),
            ("bgr8", 3),
            ("rgba8", 4),
            ("bgra8", 4),
            ("y8", 1),
        ],
    )?;
    bundle.aligned_depth.validate("aligned_depth")?;
    bundle
        .aligned_depth
        .validate_format("aligned_depth", &[("z16le", 2), ("z16be", 2)])?;
    if bundle.color.width != bundle.aligned_depth.width
        || bundle.color.height != bundle.aligned_depth.height
        || bundle.color.frame_id != bundle.aligned_depth.frame_id
        || bundle.intrinsics.width != bundle.color.width
        || bundle.intrinsics.height != bundle.color.height
    {
        return Err(ArrowCodecError::InvalidShape);
    }
    if !bundle.depth_scale_m.is_finite() || bundle.depth_scale_m <= 0.0 {
        return Err(ArrowCodecError::InvalidCameraFrame(
            "depth scale must be finite and positive".into(),
        ));
    }
    Ok(())
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
    Snapshot,
    Reset,
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
    pub custom_name: Option<String>,
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ActionDomains {
    pub position: bool,
    pub orientation: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InputActionSpec {
    pub key: &'static str,
    pub label: &'static str,
    pub action_type: ActionType,
    pub domains: ActionDomains,
}

const CONTROL_DOMAINS: ActionDomains = ActionDomains {
    position: false,
    orientation: false,
};
const POSITION_DOMAIN: ActionDomains = ActionDomains {
    position: true,
    orientation: false,
};
const ORIENTATION_DOMAIN: ActionDomains = ActionDomains {
    position: false,
    orientation: true,
};
const POSE_DOMAINS: ActionDomains = ActionDomains {
    position: true,
    orientation: true,
};

pub const INPUT_ACTIONS: [InputActionSpec; 14] = [
    input_action(
        "start_stop",
        "接管控制",
        ActionType::Boolean,
        CONTROL_DOMAINS,
    ),
    input_action(
        "emergency_stop",
        "急停",
        ActionType::Boolean,
        CONTROL_DOMAINS,
    ),
    input_action(
        "primary_tool_open",
        "夹爪张开",
        ActionType::Boolean,
        CONTROL_DOMAINS,
    ),
    input_action(
        "primary_tool",
        "夹爪开合",
        ActionType::Float,
        CONTROL_DOMAINS,
    ),
    input_action(
        "move_forward_back",
        "纵向平移",
        ActionType::Float,
        POSITION_DOMAIN,
    ),
    input_action(
        "move_left_right",
        "横向平移",
        ActionType::Float,
        POSITION_DOMAIN,
    ),
    input_action(
        "move_up_down",
        "垂直平移",
        ActionType::Float,
        POSITION_DOMAIN,
    ),
    input_action(
        "front_pitch",
        "垂直圆弧",
        ActionType::Float,
        ORIENTATION_DOMAIN,
    ),
    input_action(
        "horizontal_arc",
        "水平圆弧",
        ActionType::Float,
        ORIENTATION_DOMAIN,
    ),
    input_action(
        "tool_pitch",
        "定点垂直旋转",
        ActionType::Float,
        ORIENTATION_DOMAIN,
    ),
    input_action(
        "tool_yaw",
        "定点水平旋转",
        ActionType::Float,
        ORIENTATION_DOMAIN,
    ),
    input_action(
        "tool_roll",
        "轴向旋转",
        ActionType::Float,
        ORIENTATION_DOMAIN,
    ),
    input_action(
        "tool_axis_translation",
        "工具轴向平移",
        ActionType::Float,
        POSITION_DOMAIN,
    ),
    input_action(
        "tool_helical_motion",
        "工具轴向螺旋",
        ActionType::Float,
        POSE_DOMAINS,
    ),
];

const fn input_action(
    key: &'static str,
    label: &'static str,
    action_type: ActionType,
    domains: ActionDomains,
) -> InputActionSpec {
    InputActionSpec {
        key,
        label,
        action_type,
        domains,
    }
}

pub fn input_action_spec(key: &str) -> Option<&'static InputActionSpec> {
    INPUT_ACTIONS.iter().find(|action| action.key == key)
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

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DepthCameraCalibration {
    pub schema_version: u32,
    pub sequence: u64,
    pub source_time_ns: i64,
    pub source_id: String,
    pub parent_frame_id: String,
    pub frame_id: String,
    pub translation_m: [f64; 3],
    pub orientation_xyzw: [f64; 4],
    pub width: u32,
    pub height: u32,
    pub distortion_model: String,
    pub distortion: Vec<f64>,
    pub camera_matrix: [f64; 9],
    pub projection_matrix: [f64; 12],
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CameraSourceInfo {
    pub source_id: String,
    pub driver_id: String,
    pub display_name: String,
    pub device_model: Option<String>,
    pub serial_number: Option<String>,
    pub firmware_version: Option<String>,
    pub connection_type: Option<String>,
    pub physical_port: Option<String>,
    pub sensors: Vec<String>,
    pub profiles: Vec<CameraStreamProfile>,
    #[serde(default)]
    pub driver_extensions: Vec<CameraDriverExtensionInfo>,
    pub available: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub action: RequestAction,
    pub source_id: Option<String>,
    pub color_profile_key: Option<String>,
    pub depth_profile_key: Option<String>,
    #[serde(default)]
    pub output_frames_per_second: Option<f64>,
    #[serde(default)]
    pub driver_parameters: Option<Vec<CameraDriverParameterValue>>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CameraCaptureState {
    pub schema_version: u32,
    pub available_sources: Vec<CameraSourceInfo>,
    pub selected_source_id: Option<String>,
    pub selected_color_profile_key: Option<String>,
    pub selected_depth_profile_key: Option<String>,
    pub output_frames_per_second: Option<f64>,
    pub configurations: Vec<CameraSourceConfiguration>,
    pub streaming: bool,
    pub last_sequence: Option<u64>,
    pub last_frame_time_ns: Option<i64>,
    pub measured_frames_per_second: Option<f64>,
    pub measured_output_frames_per_second: Option<f64>,
    pub dropped_frame_count: u64,
    pub skipped_output_frame_count: u64,
    pub original_error: Option<String>,
    pub service: ServiceState,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImageFrameInfo {
    pub width: u32,
    pub height: u32,
    pub encoding: String,
    pub frame_id: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PerceptionInstanceSummary {
    pub instance_id: String,
    pub label: String,
    pub confidence: f64,
    pub bounding_box_xyxy: [f64; 4],
    pub position_m: Option<[f64; 3]>,
    pub size_m: Option<[f64; 3]>,
    pub grasp_candidate_count: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PerceptionState {
    pub schema_version: u32,
    pub enabled: bool,
    pub source_id: Option<String>,
    pub compute_service_url: String,
    pub model: String,
    pub classes: Vec<String>,
    #[serde(default)]
    pub placement_labels: Vec<String>,
    pub color_frame: Option<ImageFrameInfo>,
    pub depth_frame: Option<ImageFrameInfo>,
    pub camera_calibration: Option<DepthCameraCalibration>,
    pub depth_scale_m: Option<f64>,
    #[serde(default)]
    pub instances: Vec<PerceptionInstanceSummary>,
    pub point_count: Option<u64>,
    pub last_frame_time_ns: Option<i64>,
    pub last_scene_sequence: Option<u64>,
    pub task_request_id: Option<String>,
    pub task_state: RequestState,
    pub calibrated: bool,
    pub original_error: Option<String>,
    pub service: ServiceState,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PerceptionAssetRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub asset_key: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PerceptionAssetResponse {
    pub schema_version: u32,
    pub request_id: String,
    pub asset_key: String,
    pub mime_type: Option<String>,
    pub content: Option<Vec<u8>>,
    pub original_error: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PerceptionRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub action: RequestAction,
    pub source_id: Option<String>,
    pub classes: Option<Vec<String>>,
    #[serde(default)]
    pub placement_labels: Option<Vec<String>>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AlignedDepthFrame {
    pub schema_version: u32,
    pub sequence: u64,
    pub source_time_ns: i64,
    pub source_id: String,
    pub frame_id: String,
    pub width: u32,
    pub height: u32,
    pub depth_scale_m: f64,
    pub depth: Vec<u16>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose3 {
    pub position_m: [f64; 3],
    pub orientation_xyzw: [f64; 4],
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DetectedInstance2D {
    pub instance_id: String,
    pub label: String,
    pub confidence: f64,
    pub bounding_box_xyxy: [f64; 4],
    pub mask_width: u32,
    pub mask_height: u32,
    pub mask_png: Vec<u8>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneObject {
    pub object_id: String,
    pub label: String,
    pub pose: Pose3,
    pub size_m: [f64; 3],
    pub confidence: f64,
    pub grasp_candidates: Vec<Pose3>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlacementRegion {
    pub region_id: String,
    pub label: String,
    pub pose: Pose3,
    pub size_m: [f64; 3],
    pub source_object_id: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneObstacle {
    pub obstacle_id: String,
    pub pose: Pose3,
    pub size_m: [f64; 3],
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WorldScene {
    pub schema_version: u32,
    pub sequence: u64,
    pub sample_time_ns: i64,
    pub frame_id: String,
    pub objects: Vec<SceneObject>,
    pub placement_regions: Vec<PlacementRegion>,
    pub obstacles: Vec<SceneObstacle>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CalibrationBoard {
    pub pattern: String,
    pub dictionary: String,
    pub squares_x: u32,
    pub squares_y: u32,
    pub square_size_m: f64,
    pub marker_size_m: f64,
    pub measured_width_m: f64,
    pub measured_height_m: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CalibrationObservation {
    pub sample_id: String,
    pub sample_time_ns: i64,
    pub camera_frame_id: String,
    pub board_in_camera: Pose3,
    pub tcp_in_base: Pose3,
    pub joint_feedback_rad: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CalibrationResult {
    pub schema_version: u32,
    pub camera_source_id: String,
    pub robot_model_revision: String,
    pub calibration_tool_id: String,
    pub board: CalibrationBoard,
    pub camera_in_base: Pose3,
    pub board_in_calibration_tool: Pose3,
    pub solver: String,
    pub solved_at_ns: i64,
    pub sample_count: u32,
    pub translation_residuals_m: Vec<f64>,
    pub rotation_residuals_rad: Vec<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationAction {
    Start,
    Apply,
    Cancel,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CalibrationPhase {
    #[default]
    Idle,
    Preparing,
    Moving,
    Detecting,
    Solving,
    AwaitingConfirmation,
    Applied,
    Failed,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CalibrationRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub action: CalibrationAction,
    pub board: Option<CalibrationBoard>,
    pub camera_source_id: Option<String>,
    pub robot_model_revision: Option<String>,
    pub calibration_tool_id: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CalibrationSessionState {
    pub schema_version: u32,
    pub active: bool,
    #[serde(default)]
    pub phase: CalibrationPhase,
    #[serde(default)]
    pub run_id: Option<String>,
    #[serde(default)]
    pub current_target_index: Option<u32>,
    #[serde(default)]
    pub target_count: u32,
    #[serde(default)]
    pub current_target_key: Option<String>,
    #[serde(default)]
    pub motion_request_id: Option<String>,
    #[serde(default)]
    pub stage_message: Option<String>,
    pub board: Option<CalibrationBoard>,
    pub camera_source_id: Option<String>,
    pub robot_model_revision: Option<String>,
    pub calibration_tool_id: Option<String>,
    pub observations: Vec<CalibrationObservation>,
    pub solved_result: Option<CalibrationResult>,
    pub original_error: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PickPlaceRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub object_id: String,
    pub placement_region_id: String,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ManipulationTaskState {
    pub schema_version: u32,
    pub request_id: String,
    pub object_id: Option<String>,
    pub placement_region_id: Option<String>,
    pub pick_position_m: Option<[f64; 3]>,
    pub place_position_m: Option<[f64; 3]>,
    pub state: RequestState,
    pub stage: Option<String>,
    pub solution_count: Option<u32>,
    pub selected_cost: Option<f64>,
    pub original_error: Option<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct InputSimulationState {
    pub schema_version: u32,
    pub active: bool,
    pub item: Option<InputSimulationItem>,
    pub phase: Option<String>,
    pub elapsed_s: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct InputSimulationRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub enabled: bool,
    pub item: Option<InputSimulationItem>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InputSimulationItem {
    MoveForwardBack,
    MoveLeftRight,
    MoveUpDown,
    ToolPitch,
    ToolYaw,
    ToolRoll,
    FrontPitch,
    HorizontalArc,
    PrimaryToolOpen,
    PrimaryTool,
    StartStop,
    EmergencyStop,
    PrimaryToolFeedback,
    ToolAxisTranslation,
    ToolHelicalMotion,
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

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RenameInputSourceRequest {
    pub schema_version: u32,
    pub request_id: String,
    pub source_id: String,
    pub custom_name: Option<String>,
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
    pub tool_pitch: FloatActionSample,
    pub tool_yaw: FloatActionSample,
    pub tool_roll: FloatActionSample,
    pub tool_axis_translation: FloatActionSample,
    pub tool_helical_motion: FloatActionSample,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpatialComponentSwitches {
    pub translation: bool,
    pub front_pitch: bool,
    pub horizontal_arc: bool,
    #[serde(default = "enabled")]
    pub tool_pitch: bool,
    #[serde(default = "enabled")]
    pub tool_yaw: bool,
    #[serde(default = "enabled")]
    pub tool_roll: bool,
    #[serde(default = "enabled")]
    pub tool_axis_translation: bool,
    #[serde(default = "enabled")]
    pub tool_helical_motion: bool,
}

const fn enabled() -> bool {
    true
}

impl Default for SpatialComponentSwitches {
    fn default() -> Self {
        Self {
            translation: true,
            front_pitch: true,
            horizontal_arc: true,
            tool_pitch: true,
            tool_yaw: true,
            tool_roll: true,
            tool_axis_translation: true,
            tool_helical_motion: true,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VerticalOrientationMapping {
    #[default]
    FrontPitch,
    ToolPitch,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HorizontalOrientationMapping {
    #[default]
    HorizontalArc,
    ToolYaw,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct OrientationActionMapping {
    pub vertical: VerticalOrientationMapping,
    pub horizontal: HorizontalOrientationMapping,
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
    pub orientation_mapping: OrientationActionMapping,
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
            orientation_mapping: OrientationActionMapping::default(),
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
    pub tool_pitch_rad: f64,
    pub tool_yaw_rad: f64,
    pub tool_roll_rad: f64,
    pub tool_axis_translation_m: f64,
    pub tool_helical_translation_m: f64,
    pub tool_helical_roll_rad: f64,
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
    pub orientation_mapping: Option<OrientationActionMapping>,
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
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gripper_asset_id: Option<String>,
    pub joints: Vec<JointMetadata>,
    pub tool_actuators: Vec<ActuatorMetadata>,
    pub named_targets: Vec<NamedMotionTarget>,
    #[serde(default)]
    pub calibration_targets: Vec<NamedMotionTarget>,
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
    Perception,
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
    pub feedback_interval_ms: u64,
    pub last_error: Option<String>,
    pub last_command: Option<ArmCommand>,
    pub feedback_summary: Option<String>,
    pub parameter_error: Option<String>,
    pub parameter_values: Vec<ParameterValue>,
    pub latest_request_id: Option<String>,
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
    fn complete_control_contract_round_trips_all_tool_actions_and_simulation_item() {
        let control = ControlInputFrame {
            schema_version: SCHEMA_VERSION,
            tool_pitch: FloatActionSample {
                is_active: true,
                changed_since_last_sync: true,
                value: 0.1,
            },
            tool_yaw: FloatActionSample {
                value: 0.2,
                ..Default::default()
            },
            tool_roll: FloatActionSample {
                value: 0.3,
                ..Default::default()
            },
            tool_axis_translation: FloatActionSample {
                value: 0.4,
                ..Default::default()
            },
            tool_helical_motion: FloatActionSample {
                value: 0.5,
                ..Default::default()
            },
            ..Default::default()
        };
        let encoded = to_arrow(&control).unwrap();
        let decoded: ControlInputFrame = from_arrow(encoded.as_ref()).unwrap();
        assert_eq!(decoded, control);

        let request = InputSimulationRequest {
            schema_version: SCHEMA_VERSION,
            request_id: "fixture".into(),
            enabled: true,
            item: Some(InputSimulationItem::ToolHelicalMotion),
        };
        let encoded = to_arrow(&request).unwrap();
        let decoded: InputSimulationRequest = from_arrow(encoded.as_ref()).unwrap();
        assert_eq!(decoded, request);
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
            gripper_asset_id: None,
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
            calibration_targets: vec![],
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
    fn perception_scene_calibration_and_pick_place_contracts_round_trip() {
        let pose = Pose3 {
            position_m: [0.1, 0.2, 0.3],
            orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
        };
        let scene = WorldScene {
            schema_version: SCHEMA_VERSION,
            sequence: 4,
            sample_time_ns: 5,
            frame_id: "base_link".into(),
            objects: vec![SceneObject {
                object_id: "red-cube-0".into(),
                label: "red cube".into(),
                pose: pose.clone(),
                size_m: [0.04; 3],
                confidence: 0.9,
                grasp_candidates: vec![pose.clone()],
            }],
            placement_regions: vec![PlacementRegion {
                region_id: "basket-interior".into(),
                label: "gray storage bin interior".into(),
                pose: pose.clone(),
                size_m: [0.15, 0.12, 0.08],
                source_object_id: Some("basket-1".into()),
            }],
            obstacles: vec![],
        };
        let decoded: WorldScene = from_arrow(to_arrow(&scene).unwrap().as_ref()).unwrap();
        assert_eq!(decoded, scene);

        let calibration = CalibrationResult {
            schema_version: SCHEMA_VERSION,
            camera_source_id: "camera-1".into(),
            robot_model_revision: "arm-v1".into(),
            calibration_tool_id: "charuco-test-tool".into(),
            board: CalibrationBoard {
                pattern: "charuco".into(),
                dictionary: "DICT_4X4_50".into(),
                squares_x: 5,
                squares_y: 5,
                square_size_m: 0.015,
                marker_size_m: 0.011,
                measured_width_m: 0.075,
                measured_height_m: 0.075,
            },
            camera_in_base: pose.clone(),
            board_in_calibration_tool: pose,
            solver: "opencv-calibrateRobotWorldHandEye".into(),
            solved_at_ns: 6,
            sample_count: 7,
            translation_residuals_m: vec![0.001],
            rotation_residuals_rad: vec![0.01],
        };
        let decoded: CalibrationResult =
            from_arrow(to_arrow(&calibration).unwrap().as_ref()).unwrap();
        assert_eq!(decoded, calibration);

        let camera_request = PerceptionRequest {
            schema_version: SCHEMA_VERSION,
            request_id: "reset-camera-1".into(),
            action: RequestAction::Reset,
            source_id: Some("simulation:pick-place-scene".into()),
            classes: None,
            placement_labels: None,
        };
        let decoded: PerceptionRequest =
            from_arrow(to_arrow(&camera_request).unwrap().as_ref()).unwrap();
        assert_eq!(decoded, camera_request);

        let request = PickPlaceRequest {
            schema_version: SCHEMA_VERSION,
            request_id: "pick-place-1".into(),
            object_id: "red-cube-0".into(),
            placement_region_id: "basket-interior".into(),
        };
        let decoded: PickPlaceRequest = from_arrow(to_arrow(&request).unwrap().as_ref()).unwrap();
        assert_eq!(decoded, request);
    }

    fn camera_frame_fixture() -> CameraFrameBundle {
        CameraFrameBundle {
            schema_version: SCHEMA_VERSION,
            sequence: 7,
            source_id: "simulation:camera".into(),
            device_time_ns: 11,
            device_time_domain: "simulation".into(),
            received_time_ns: 12,
            color: CameraImagePlane {
                width: 2,
                height: 2,
                stride_bytes: 6,
                pixel_format: "rgb8".into(),
                frame_id: "camera_color_optical_frame".into(),
                data: vec![0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11],
            },
            aligned_depth: CameraImagePlane {
                width: 2,
                height: 2,
                stride_bytes: 4,
                pixel_format: "z16le".into(),
                frame_id: "camera_color_optical_frame".into(),
                data: vec![1, 0, 2, 0, 3, 0, 4, 0],
            },
            intrinsics: CameraIntrinsics {
                width: 2,
                height: 2,
                focal_length_px: [2.0, 2.0],
                principal_point_px: [0.5, 0.5],
                distortion_model: "none".into(),
                distortion: vec![0.0; 5],
            },
            depth_scale_m: 0.001,
            calibration: None,
        }
    }

    #[test]
    fn camera_frame_uses_binary_buffers_and_round_trips() {
        let expected = camera_frame_fixture();
        let encoded = camera_frame_to_arrow(&expected).unwrap();
        let structure = encoded
            .as_any()
            .downcast_ref::<StructArray>()
            .expect("camera frame is an Arrow struct");
        assert_eq!(
            structure
                .column_by_name(CAMERA_COLOR_DATA_FIELD)
                .unwrap()
                .data_type(),
            &DataType::Binary
        );
        let metadata = structure
            .column_by_name(CAMERA_FRAME_METADATA_FIELD)
            .unwrap()
            .as_any()
            .downcast_ref::<StringArray>()
            .unwrap()
            .value(0);
        assert!(!metadata.contains("\"data\""));
        assert_eq!(camera_frame_from_arrow(encoded.as_ref()).unwrap(), expected);
    }

    #[test]
    fn camera_frame_rejects_payload_length_mismatch() {
        let mut frame = camera_frame_fixture();
        frame.aligned_depth.data.pop();
        assert!(matches!(
            camera_frame_to_arrow(&frame),
            Err(ArrowCodecError::InvalidCameraFrame(_))
        ));
    }

    #[test]
    fn camera_frame_rejects_inconsistent_pixel_metadata() {
        let mut frame = camera_frame_fixture();
        frame.color.pixel_format = "unknown".into();
        assert!(matches!(
            camera_frame_to_arrow(&frame),
            Err(ArrowCodecError::InvalidCameraFrame(_))
        ));

        let mut frame = camera_frame_fixture();
        frame.color.stride_bytes = frame.color.width * 2;
        frame.color.data = vec![0; (frame.color.stride_bytes * frame.color.height) as usize];
        assert!(matches!(
            camera_frame_to_arrow(&frame),
            Err(ArrowCodecError::InvalidCameraFrame(_))
        ));

        let mut frame = camera_frame_fixture();
        frame.depth_scale_m = 0.0;
        assert!(matches!(
            camera_frame_to_arrow(&frame),
            Err(ArrowCodecError::InvalidCameraFrame(_))
        ));
    }

    #[test]
    fn camera_frame_rejects_a_different_schema_before_transport() {
        let mut frame = camera_frame_fixture();
        frame.schema_version = SCHEMA_VERSION + 1;
        assert!(matches!(
            camera_frame_to_arrow(&frame),
            Err(ArrowCodecError::UnsupportedVersion { .. })
        ));
    }

    #[test]
    fn camera_frame_preserves_row_padding() {
        let mut frame = camera_frame_fixture();
        frame.color.stride_bytes = 8;
        frame.color.data = (0..16).collect();
        frame.aligned_depth.stride_bytes = 6;
        frame.aligned_depth.data = (0..12).collect();
        let encoded = camera_frame_to_arrow(&frame).unwrap();
        assert_eq!(camera_frame_from_arrow(encoded.as_ref()).unwrap(), frame);
    }

    #[test]
    fn camera_frame_round_trips_a_full_resolution_payload() {
        let mut frame = camera_frame_fixture();
        frame.color.width = 1_280;
        frame.color.height = 720;
        frame.color.stride_bytes = 1_280 * 3;
        frame.color.data = vec![17; (frame.color.stride_bytes * frame.color.height) as usize];
        frame.aligned_depth.width = 1_280;
        frame.aligned_depth.height = 720;
        frame.aligned_depth.stride_bytes = 1_280 * 2;
        frame.aligned_depth.data =
            vec![23; (frame.aligned_depth.stride_bytes * frame.aligned_depth.height) as usize];
        frame.intrinsics.width = 1_280;
        frame.intrinsics.height = 720;
        let encoded = camera_frame_to_arrow(&frame).unwrap();
        assert_eq!(camera_frame_from_arrow(encoded.as_ref()).unwrap(), frame);
    }
}
