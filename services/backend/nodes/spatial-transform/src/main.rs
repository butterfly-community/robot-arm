use std::{
    fs,
    path::{Path, PathBuf},
    time::{SystemTime, UNIX_EPOCH},
};

use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result};
use robot_arm_messages::{
    AbsolutePoseFrame, ConfirmOriginRequest, ControlInputFrame, RequestAction, RequestResult,
    SCHEMA_VERSION, ServiceState, SpatialConfigState, UpdateSpatialConfigRequest, from_arrow,
    to_arrow,
};
use spatial_core::SpatialTransform;

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let config_path = std::env::var_os("SPATIAL_CONFIG")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("/config/spatial-transform.json"));
    let mut transform = SpatialTransform::new(load_config(&config_path));
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
                    transform.apply_config(request.patch);
                    let error = save_config(&config_path, transform.config())
                        .err()
                        .map(|value| format!("保存空间配置失败：{value}"));
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
                    let confirmed = transform.confirm_origin();
                    service.last_error =
                        (!confirmed).then(|| "当前没有可用于确认原点的绝对位姿".to_owned());
                    send_json(&mut node, "config_state", transform.config())?;
                    send_json(
                        &mut node,
                        "request_result",
                        &RequestResult {
                            schema_version: robot_arm_messages::SCHEMA_VERSION,
                            request_id: request.request_id,
                            acknowledged_action: RequestAction::Apply,
                            value: confirmed.then(|| transform.config().clone()),
                            original_error: (!confirmed)
                                .then(|| "当前没有可用于确认原点的绝对位姿".to_owned()),
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

fn load_config(path: &Path) -> SpatialConfigState {
    let mut config: SpatialConfigState = fs::read(path)
        .ok()
        .and_then(|bytes| serde_json::from_slice(&bytes).ok())
        .unwrap_or_default();
    config.origin_position_m = None;
    config.control_session_id = None;
    config.selected_source_id = None;
    config.source_has_absolute_pose = false;
    config
}

fn save_config(path: &Path, config: &SpatialConfigState) -> Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let mut stored = config.clone();
    stored.origin_position_m = None;
    stored.control_session_id = None;
    stored.selected_source_id = None;
    stored.source_has_absolute_pose = false;
    fs::write(path, serde_json::to_vec_pretty(&stored)?)?;
    Ok(())
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
    fn persisted_config_excludes_process_only_origin_and_session() {
        let path = std::env::temp_dir().join(format!(
            "spatial-transform-{}-{}.json",
            std::process::id(),
            now_ns()
        ));
        let configured = SpatialConfigState {
            translation_scale: 0.25,
            origin_position_m: Some([1.0, 2.0, 3.0]),
            control_session_id: Some(9),
            selected_source_id: Some("runtime-source".into()),
            source_has_absolute_pose: true,
            ..Default::default()
        };
        save_config(&path, &configured).unwrap();
        let loaded = load_config(&path);
        assert_eq!(loaded.translation_scale, 0.25);
        assert_eq!(loaded.origin_position_m, None);
        assert_eq!(loaded.control_session_id, None);
        assert_eq!(loaded.selected_source_id, None);
        assert!(!loaded.source_has_absolute_pose);
        std::fs::remove_file(path).unwrap();
    }
}
