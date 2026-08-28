use std::{
    path::{Path, PathBuf},
    time::{SystemTime, UNIX_EPOCH},
};

use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result, eyre};
use json_config_store::{load_or_default, save};
use robot_arm_messages::{
    AbsolutePoseFrame, ConfirmOriginRequest, ControlInputFrame, RequestAction, RequestResult,
    SCHEMA_VERSION, ServiceState, SpatialComponentSwitches, SpatialConfigState,
    UpdateSpatialConfigRequest, from_arrow, to_arrow,
};
use serde::{Deserialize, Serialize};
use spatial_core::SpatialTransform;

const CONFIG_SCHEMA_VERSION: u32 = 1;

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let config_path = std::env::var_os("SPATIAL_CONFIG")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/config/spatial-transform.json"));
    let mut transform = SpatialTransform::new(load_config(&config_path)?);
    let mut service = service_state(transform.config().config_version);

    send_json(&mut node, "config_state", transform.config())?;
    send_json(&mut node, "service_state", &service)?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, data, .. } => match id.as_str() {
                "absolute_pose" => {
                    let frame: AbsolutePoseFrame =
                        from_arrow(data.as_array()).context("decode absolute_pose")?;
                    if let Some(output) = transform.handle_pose(frame, now_ns()) {
                        send_json(&mut node, "relative_motion", &output)?;
                        service.has_input = true;
                        service.has_output = true;
                    }
                }
                "control_input" => {
                    let frame: ControlInputFrame =
                        from_arrow(data.as_array()).context("decode control_input")?;
                    let output = transform.handle_control(frame, now_ns());
                    send_json(&mut node, "relative_motion", &output)?;
                    send_json(&mut node, "config_state", transform.config())?;
                    service.has_input = true;
                    service.has_output = true;
                }
                "update_config" => {
                    let request: UpdateSpatialConfigRequest =
                        from_arrow(data.as_array()).context("decode update_config")?;
                    let mut next = transform.clone();
                    next.apply_config(request.patch);
                    let error = save_config(&config_path, next.config())
                        .err()
                        .map(|value| format!("保存空间配置失败：{value}"));
                    if error.is_none() {
                        transform = next;
                    }
                    service.config_version = transform.config().config_version;
                    service.last_error = error.clone();
                    send_json(&mut node, "config_state", transform.config())?;
                    send_json(
                        &mut node,
                        "request_result",
                        &RequestResult {
                            schema_version: robot_arm_messages::SCHEMA_VERSION,
                            request_id: request.request_id,
                            acknowledged_action: RequestAction::Apply,
                            value: Some(transform.config().clone()),
                            original_error: error,
                        },
                    )?;
                }
                "confirm_origin" => {
                    let request: ConfirmOriginRequest =
                        from_arrow(data.as_array()).context("decode confirm_origin")?;
                    let mut next = transform.clone();
                    let confirmed = next.confirm_origin();
                    let error = if confirmed {
                        save_config(&config_path, next.config())
                            .err()
                            .map(|value| format!("保存空间配置失败：{value}"))
                    } else {
                        Some("当前没有可用于确认原点的绝对位姿".to_owned())
                    };
                    if error.is_none() {
                        transform = next;
                    }
                    service.config_version = transform.config().config_version;
                    service.last_error = error.clone();
                    send_json(&mut node, "config_state", transform.config())?;
                    send_json(
                        &mut node,
                        "request_result",
                        &RequestResult {
                            schema_version: robot_arm_messages::SCHEMA_VERSION,
                            request_id: request.request_id,
                            acknowledged_action: RequestAction::Apply,
                            value: error.is_none().then(|| transform.config().clone()),
                            original_error: error,
                        },
                    )?;
                }
                "snapshot" => send_json(&mut node, "config_state", transform.config())?,
                _ => {}
            },
            Event::Stop(_) => break,
            _ => {}
        }
        service.updated_at_ns = now_ns();
        send_json(&mut node, "service_state", &service)?;
    }
    Ok(())
}

fn service_state(config_version: u64) -> ServiceState {
    ServiceState {
        schema_version: SCHEMA_VERSION,
        build_version: env!("CARGO_PKG_VERSION").into(),
        config_version,
        running: true,
        has_input: false,
        has_output: true,
        last_error: None,
        updated_at_ns: now_ns(),
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct SpatialConfig {
    schema_version: u32,
    config_version: u64,
    base_from_tracking_axes: [[f64; 3]; 3],
    translation_scale: f64,
    action_translation_m_per_s: Option<f64>,
    action_arc_rad_per_s: Option<f64>,
    origin_position_m: Option<[f64; 3]>,
    switches: SpatialComponentSwitches,
}

impl Default for SpatialConfig {
    fn default() -> Self {
        Self::from(&SpatialConfigState::default())
    }
}

impl From<&SpatialConfigState> for SpatialConfig {
    fn from(config: &SpatialConfigState) -> Self {
        Self {
            schema_version: CONFIG_SCHEMA_VERSION,
            config_version: config.config_version,
            base_from_tracking_axes: config.base_from_tracking_axes,
            translation_scale: config.translation_scale,
            action_translation_m_per_s: config.action_translation_m_per_s,
            action_arc_rad_per_s: config.action_arc_rad_per_s,
            origin_position_m: config.origin_position_m,
            switches: config.switches,
        }
    }
}

fn load_config(path: &Path) -> Result<SpatialConfigState> {
    let config: SpatialConfig = load_or_default(path)?;
    if config.schema_version != CONFIG_SCHEMA_VERSION {
        return Err(eyre!("不支持的空间配置版本 {}", config.schema_version));
    }
    Ok(SpatialConfigState {
        schema_version: SCHEMA_VERSION,
        config_version: config.config_version,
        position_source_id: None,
        orientation_source_id: None,
        base_from_tracking_axes: config.base_from_tracking_axes,
        translation_scale: config.translation_scale,
        action_translation_m_per_s: config.action_translation_m_per_s,
        action_arc_rad_per_s: config.action_arc_rad_per_s,
        origin_position_m: config.origin_position_m,
        switches: config.switches,
        control_session_id: None,
    })
}

fn save_config(path: &Path, config: &SpatialConfigState) -> Result<()> {
    save(path, &SpatialConfig::from(config))
}

fn send_json<T: serde::Serialize>(node: &mut DoraNode, output: &str, value: &T) -> Result<()> {
    node.send_output(
        DataId::from(output.to_owned()),
        MetadataParameters::default(),
        to_arrow(value)?,
    )?;
    Ok(())
}

fn now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_nanos() as i64)
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn persisted_config_keeps_origin_and_excludes_runtime_session() {
        let path = std::env::temp_dir().join(format!(
            "spatial-transform-{}-{}.json",
            std::process::id(),
            now_ns()
        ));
        let configured = SpatialConfigState {
            translation_scale: 0.25,
            origin_position_m: Some([1.0, 2.0, 3.0]),
            control_session_id: Some(9),
            position_source_id: Some("position-source".into()),
            orientation_source_id: Some("orientation-source".into()),
            ..Default::default()
        };
        save_config(&path, &configured).unwrap();
        let stored = std::fs::read_to_string(&path).unwrap();
        assert!(!stored.contains("control_session_id"));
        assert!(!stored.contains("position_source_id"));
        assert!(!stored.contains("orientation_source_id"));

        let loaded = load_config(&path).unwrap();
        assert_eq!(loaded.translation_scale, 0.25);
        assert_eq!(loaded.origin_position_m, Some([1.0, 2.0, 3.0]));
        assert_eq!(loaded.control_session_id, None);
        assert_eq!(loaded.position_source_id, None);
        assert_eq!(loaded.orientation_source_id, None);
        std::fs::remove_file(path).unwrap();
    }
}
