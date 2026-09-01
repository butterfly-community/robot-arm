use std::{
    collections::BTreeMap,
    net::{IpAddr, Ipv4Addr, SocketAddr},
    sync::{
        Arc, Mutex,
        atomic::{AtomicU64, Ordering},
        mpsc,
    },
    time::Duration,
};

use axum::{
    Json, Router,
    extract::{
        Path, State, WebSocketUpgrade,
        ws::{Message, WebSocket},
    },
    http::{HeaderValue, StatusCode, header},
    response::{IntoResponse, Response},
    routing::{get, patch, post},
};
use dora_node_api::{DoraNode, Event, MetadataParameters, dora_core::config::DataId};
use eyre::{Context, Result};
use robot_arm_messages::{ExecutionRequest, SCHEMA_VERSION, from_arrow, to_arrow};
use serde_json::{Value, json};
use tokio::sync::{oneshot, watch};

const DEFAULT_PORT: u16 = 8080;

#[derive(Clone)]
struct AppState {
    snapshots: Arc<Mutex<BTreeMap<String, BTreeMap<String, Value>>>>,
    sequences: Arc<Mutex<BTreeMap<String, u64>>>,
    channels: Arc<BTreeMap<String, watch::Sender<String>>>,
    requests: mpsc::Sender<OutgoingRequest>,
    request_sequence: Arc<AtomicU64>,
}

struct OutgoingRequest {
    output: String,
    request_id: String,
    body: Value,
    response: Option<oneshot::Sender<Value>>,
}

fn main() -> Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    let (request_tx, request_rx) = mpsc::channel();
    let state = AppState::new(request_tx);
    let server_state = state.clone();
    let (shutdown_tx, shutdown_rx) = tokio::sync::watch::channel(false);
    let server = std::thread::Builder::new()
        .name("web-gateway-http".into())
        .spawn(move || {
            let runtime = tokio::runtime::Runtime::new().expect("create HTTP runtime");
            runtime
                .block_on(serve(server_state, shutdown_rx))
                .expect("run HTTP server");
        })?;

    let mut pending = BTreeMap::<String, oneshot::Sender<Value>>::new();
    loop {
        while let Ok(request) = request_rx.try_recv() {
            let output = request.output.clone();
            let request_id = request.request_id.clone();
            let result = node.send_output(
                DataId::from(output),
                MetadataParameters::default(),
                to_arrow(&request.body)?,
            );
            if let Some(response) = request.response {
                if let Err(error) = result {
                    let _ = response.send(json!({
                        "schema_version": SCHEMA_VERSION,
                        "request_id": request_id,
                        "original_error": error.to_string()
                    }));
                } else {
                    pending.insert(request_id, response);
                }
            }
        }

        let Some(event) = events.recv_timeout(Duration::from_millis(10)) else {
            continue;
        };
        match event {
            Event::Input { id, data, .. } => {
                let value: Value =
                    from_arrow(data.as_array()).with_context(|| format!("decode {id}"))?;
                let input = id.as_str();
                if (input.ends_with("request_result")
                    || input == "model_asset_response"
                    || input == "perception_asset_response"
                    || input == "actuator_status")
                    && let Some(request_id) = value.get("request_id").and_then(Value::as_str)
                    && let Some(sender) = pending.remove(request_id)
                {
                    let _ = sender.send(value.clone());
                }
                for namespace in mirrored_namespaces(input) {
                    state.update(namespace, input, value.clone());
                }
                if let Some(namespace) = namespace_for_input(input) {
                    state.update(namespace, input, value);
                }
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    let _ = shutdown_tx.send(true);
    drop(state);
    let _ = server.join();
    Ok(())
}

impl AppState {
    fn new(requests: mpsc::Sender<OutgoingRequest>) -> Self {
        let channels = [
            "tracking",
            "spatial",
            "perception",
            "motion",
            "arm-execution",
        ]
        .into_iter()
        .map(|name| {
            let (sender, _) = watch::channel(String::new());
            (name.to_owned(), sender)
        })
        .collect();
        Self {
            snapshots: Arc::new(Mutex::new(BTreeMap::new())),
            sequences: Arc::new(Mutex::new(BTreeMap::new())),
            channels: Arc::new(channels),
            requests,
            request_sequence: Arc::new(AtomicU64::new(1)),
        }
    }

    fn update(&self, namespace: &str, key: &str, value: Value) {
        {
            let mut states = self.snapshots.lock().expect("gateway snapshot lock");
            states
                .entry(namespace.into())
                .or_default()
                .insert(key.into(), value);
        }
        *self
            .sequences
            .lock()
            .expect("gateway sequence lock")
            .entry(namespace.into())
            .or_default() += 1;
        let snapshot = self.snapshot(namespace);
        if let Some(channel) = self.channels.get(namespace) {
            channel.send_replace(snapshot.to_string());
        }
    }

    fn snapshot(&self, namespace: &str) -> Value {
        let values = self
            .snapshots
            .lock()
            .expect("gateway snapshot lock")
            .get(namespace)
            .map(|value| Value::Object(value.clone().into_iter().collect()))
            .unwrap_or_else(|| json!({}));
        let sequence = self
            .sequences
            .lock()
            .expect("gateway sequence lock")
            .get(namespace)
            .copied()
            .unwrap_or_default();
        json!({
            "schema_version": SCHEMA_VERSION,
            "sequence": sequence,
            "namespace": namespace,
            "values": values
        })
    }
}

fn namespace_for_input(input: &str) -> Option<&'static str> {
    match input {
        "system_readiness" => Some("system"),
        "discovery_state"
        | "absolute_pose"
        | "control_input"
        | "pose_source_request_result"
        | "bindings_request_result"
        | "source_name_request_result"
        | "simulation_request_result" => Some("tracking"),
        "spatial_pose"
        | "spatial_config_state"
        | "spatial_service_state"
        | "transformed_control"
        | "spatial_request_result" => Some("spatial"),
        "robot_model_info"
        | "motion_state"
        | "motion_status"
        | "motion_request_result"
        | "actuator_status"
        | "mode_request_result" => Some("motion"),
        "perception_state"
        | "world_scene"
        | "perception_request_result"
        | "calibration_state"
        | "calibration_request_result"
        | "manipulation_state"
        | "manipulation_request_result" => Some("perception"),
        "execution_info"
        | "transport_state"
        | "execution_service_state"
        | "arm_state"
        | "execution_request_result" => Some("arm-execution"),
        _ => None,
    }
}

fn mirrored_namespaces(input: &str) -> &'static [&'static str] {
    match input {
        "arm_state" => &["motion", "perception"],
        "robot_model_info" | "motion_state" => &["arm-execution", "perception"],
        "manipulation_state" => &["arm-execution"],
        _ => &[],
    }
}

async fn serve(state: AppState, mut shutdown: tokio::sync::watch::Receiver<bool>) -> Result<()> {
    let app = Router::new()
        .route("/api/system/readiness", get(snapshot_system))
        .route("/api/tracking/state", get(snapshot_tracking))
        .route("/api/tracking/pose-source", post(request_pose_source))
        .route("/api/tracking/bindings", post(request_bindings))
        .route("/api/tracking/source-name", post(request_source_name))
        .route("/api/tracking/simulation", post(request_simulation))
        .route("/api/tracking/snapshot", post(request_tracking_snapshot))
        .route("/api/spatial/state", get(snapshot_spatial))
        .route("/api/spatial/config", patch(request_spatial_config))
        .route("/api/spatial/snapshot", post(request_spatial_snapshot))
        .route("/api/perception/state", get(snapshot_perception))
        .route("/api/perception/request", post(request_perception))
        .route("/api/perception/calibration", post(request_calibration))
        .route("/api/perception/pick-place", post(request_pick_place))
        .route(
            "/api/perception/snapshot",
            post(request_perception_snapshot),
        )
        .route("/api/perception/assets/{key}", get(perception_asset))
        .route("/api/motion/state", get(snapshot_motion))
        .route("/api/motion/mode", post(request_mode))
        .route(
            "/api/motion/prepare-relative",
            post(request_prepare_relative),
        )
        .route("/api/motion/request", post(request_motion))
        .route("/api/motion/cancel", post(request_motion))
        .route("/api/motion/actuator", post(request_actuator))
        .route("/api/motion/snapshot", post(request_motion_snapshot))
        .route("/api/motion/assets/{*path}", get(model_asset))
        .route("/api/arm-execution/state", get(snapshot_execution))
        .route("/api/arm-execution/connect", post(request_execution))
        .route("/api/arm-execution/disconnect", post(request_execution))
        .route("/api/arm-execution/endpoints", post(request_execution))
        .route("/api/arm-execution/config", post(request_execution))
        .route("/api/arm-execution/parameters", post(request_execution))
        .route(
            "/api/arm-execution/snapshot",
            post(request_execution_snapshot),
        )
        .route("/ws/tracking", get(ws_tracking))
        .route("/ws/spatial", get(ws_spatial))
        .route("/ws/perception", get(ws_perception))
        .route("/ws/motion", get(ws_motion))
        .route("/ws/arm-execution", get(ws_execution))
        .with_state(state);
    let address = SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), DEFAULT_PORT);
    let listener = tokio::net::TcpListener::bind(address).await?;
    axum::serve(listener, app)
        .with_graceful_shutdown(async move {
            while !*shutdown.borrow() && shutdown.changed().await.is_ok() {}
        })
        .await?;
    Ok(())
}

macro_rules! snapshot_handler {
    ($name:ident, $namespace:literal) => {
        async fn $name(State(state): State<AppState>) -> Json<Value> {
            Json(state.snapshot($namespace))
        }
    };
}
snapshot_handler!(snapshot_system, "system");
snapshot_handler!(snapshot_tracking, "tracking");
snapshot_handler!(snapshot_spatial, "spatial");
snapshot_handler!(snapshot_perception, "perception");
snapshot_handler!(snapshot_motion, "motion");
snapshot_handler!(snapshot_execution, "arm-execution");

async fn forward(state: AppState, output: &str, body: Value) -> Response {
    let Some(request_id) = body
        .get("request_id")
        .and_then(Value::as_str)
        .map(str::to_owned)
    else {
        return (
            StatusCode::BAD_REQUEST,
            Json(json!({"original_error":"request_id is required"})),
        )
            .into_response();
    };
    let (sender, receiver) = oneshot::channel();
    if state
        .requests
        .send(OutgoingRequest {
            output: output.into(),
            request_id,
            body,
            response: Some(sender),
        })
        .is_err()
    {
        return (
            StatusCode::SERVICE_UNAVAILABLE,
            Json(json!({"original_error":"Dora gateway is stopping"})),
        )
            .into_response();
    }
    match receiver.await {
        Ok(result) => Json(result).into_response(),
        Err(error) => (
            StatusCode::SERVICE_UNAVAILABLE,
            Json(json!({"original_error":error.to_string()})),
        )
            .into_response(),
    }
}

macro_rules! request_handler {
    ($name:ident, $output:literal) => {
        async fn $name(State(state): State<AppState>, Json(body): Json<Value>) -> Response {
            forward(state, $output, body).await
        }
    };
}
request_handler!(request_pose_source, "select_pose_source_request");
request_handler!(request_bindings, "apply_bindings_request");
request_handler!(request_source_name, "rename_input_source_request");
request_handler!(request_simulation, "set_simulation_request");
request_handler!(request_perception, "perception_request");
request_handler!(request_calibration, "calibration_request");
request_handler!(request_spatial_config, "update_spatial_config_request");
request_handler!(request_mode, "set_control_mode_request");
request_handler!(request_prepare_relative, "prepare_relative_request");
request_handler!(request_motion, "motion_request");
request_handler!(request_actuator, "tool_actuator_request");

async fn request_pick_place(State(state): State<AppState>, Json(body): Json<Value>) -> Response {
    let Some(request_id) = body
        .get("request_id")
        .and_then(Value::as_str)
        .map(str::to_owned)
    else {
        return (
            StatusCode::BAD_REQUEST,
            Json(json!({"original_error":"request_id is required"})),
        )
            .into_response();
    };
    if state
        .requests
        .send(OutgoingRequest {
            output: "pick_place_request".into(),
            request_id: request_id.clone(),
            body,
            response: None,
        })
        .is_err()
    {
        return (
            StatusCode::SERVICE_UNAVAILABLE,
            Json(json!({"original_error":"Dora gateway is stopping"})),
        )
            .into_response();
    }
    (
        StatusCode::ACCEPTED,
        Json(json!({
            "schema_version": SCHEMA_VERSION,
            "request_id": request_id,
            "accepted": true
        })),
    )
        .into_response()
}

async fn request_execution(State(state): State<AppState>, Json(body): Json<Value>) -> Response {
    if let Err(error) = serde_json::from_value::<ExecutionRequest>(body.clone()) {
        return (
            StatusCode::BAD_REQUEST,
            Json(json!({"original_error": error.to_string()})),
        )
            .into_response();
    }
    forward(state, "execution_request", body).await
}

async fn fire(state: AppState, output: &str) -> Response {
    let body = json!({"schema_version":SCHEMA_VERSION,"request_id":format!("snapshot-{output}")});
    match state.requests.send(OutgoingRequest {
        output: output.into(),
        request_id: format!("snapshot-{output}"),
        body,
        response: None,
    }) {
        Ok(()) => StatusCode::ACCEPTED.into_response(),
        Err(_) => StatusCode::SERVICE_UNAVAILABLE.into_response(),
    }
}
async fn request_tracking_snapshot(State(s): State<AppState>) -> Response {
    fire(s, "tracking_snapshot").await
}
async fn request_spatial_snapshot(State(s): State<AppState>) -> Response {
    fire(s, "spatial_snapshot").await
}
async fn request_motion_snapshot(State(s): State<AppState>) -> Response {
    fire(s, "motion_snapshot").await
}
async fn request_perception_snapshot(State(s): State<AppState>) -> Response {
    fire(s, "perception_snapshot").await
}
async fn request_execution_snapshot(State(s): State<AppState>) -> Response {
    fire(s, "execution_snapshot").await
}

async fn model_asset(State(state): State<AppState>, Path(path): Path<String>) -> Response {
    let model = state
        .snapshot("motion")
        .pointer("/values/robot_model_info")
        .cloned()
        .unwrap_or(Value::Null);
    let request_id = format!(
        "asset-{}-{path}",
        state.request_sequence.fetch_add(1, Ordering::Relaxed)
    );
    let body = json!({
        "schema_version": SCHEMA_VERSION,
        "request_id": request_id,
        "model_revision": model.get("model_revision").and_then(Value::as_str).unwrap_or(""),
        "manifest_hash": model.pointer("/visualization/manifest_hash").and_then(Value::as_str).unwrap_or(""),
        "relative_path": path,
    });
    let response = forward_value(state, "model_asset_request", body).await;
    binary_response(response)
}

async fn perception_asset(State(state): State<AppState>, Path(key): Path<String>) -> Response {
    let request_id = format!(
        "perception-asset-{}-{key}",
        state.request_sequence.fetch_add(1, Ordering::Relaxed)
    );
    let response = forward_value(
        state,
        "perception_asset_request",
        json!({
            "schema_version": SCHEMA_VERSION,
            "request_id": request_id,
            "asset_key": key,
        }),
    )
    .await;
    binary_response(response)
}

fn binary_response(response: Value) -> Response {
    let Some(content) = response.get("content").and_then(Value::as_array) else {
        return (StatusCode::NOT_FOUND, Json(response)).into_response();
    };
    let bytes = content
        .iter()
        .filter_map(Value::as_u64)
        .map(|value| value as u8)
        .collect::<Vec<_>>();
    let mime = response
        .get("mime_type")
        .and_then(Value::as_str)
        .unwrap_or("application/octet-stream");
    let mut result = bytes.into_response();
    if let Ok(value) = HeaderValue::from_str(mime) {
        result.headers_mut().insert(header::CONTENT_TYPE, value);
    }
    result
}

async fn forward_value(state: AppState, output: &str, body: Value) -> Value {
    let request_id = body
        .get("request_id")
        .and_then(Value::as_str)
        .unwrap_or_default()
        .to_owned();
    let (sender, receiver) = oneshot::channel();
    if state
        .requests
        .send(OutgoingRequest {
            output: output.into(),
            request_id,
            body,
            response: Some(sender),
        })
        .is_err()
    {
        return json!({"original_error":"Dora gateway is stopping"});
    }
    receiver
        .await
        .unwrap_or_else(|error| json!({"original_error":error.to_string()}))
}

macro_rules! ws_handler {
    ($name:ident, $namespace:literal) => {
        async fn $name(ws: WebSocketUpgrade, State(state): State<AppState>) -> Response {
            ws.on_upgrade(move |socket| websocket(socket, state, $namespace))
        }
    };
}
ws_handler!(ws_tracking, "tracking");
ws_handler!(ws_spatial, "spatial");
ws_handler!(ws_perception, "perception");
ws_handler!(ws_motion, "motion");
ws_handler!(ws_execution, "arm-execution");

async fn websocket(mut socket: WebSocket, state: AppState, namespace: &'static str) {
    let Some(channel) = state.channels.get(namespace) else {
        return;
    };
    let mut receiver = channel.subscribe();
    if socket
        .send(Message::Text(state.snapshot(namespace).to_string().into()))
        .await
        .is_err()
    {
        return;
    }
    let mut ready = false;
    loop {
        tokio::select! {
            changed = receiver.changed(), if ready => {
                if changed.is_err() { break; }
                let update = receiver.borrow_and_update().clone();
                if socket.send(Message::Text(update.into())).await.is_err() { break; }
                ready = false;
            },
            incoming = socket.recv() => match incoming {
                Some(Ok(Message::Close(_))) | Some(Err(_)) | None => break,
                Some(Ok(Message::Text(message))) if message.as_str() == "next" => ready = true,
                _ => {}
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn every_state_input_has_one_web_namespace() {
        for input in [
            "system_readiness",
            "discovery_state",
            "absolute_pose",
            "control_input",
            "spatial_pose",
            "spatial_config_state",
            "transformed_control",
            "robot_model_info",
            "motion_state",
            "arm_state",
            "execution_info",
            "transport_state",
        ] {
            assert!(namespace_for_input(input).is_some(), "{input}");
        }
    }
    #[test]
    fn generic_gateway_snapshot_does_not_require_a_robot_shape() {
        let (sender, _) = mpsc::channel();
        let state = AppState::new(sender);
        state.update(
            "motion",
            "robot_model_info",
            json!({"joints":[{"key":"a"},{"key":"b"},{"key":"c"}]}),
        );
        assert_eq!(
            state.snapshot("motion")["values"]["robot_model_info"]["joints"]
                .as_array()
                .unwrap()
                .len(),
            3
        );
    }
    #[test]
    fn model_feedback_and_target_state_are_available_to_both_robot_pages() {
        assert_eq!(mirrored_namespaces("arm_state"), ["motion", "perception"]);
        assert_eq!(
            mirrored_namespaces("robot_model_info"),
            ["arm-execution", "perception"]
        );
        assert_eq!(
            mirrored_namespaces("motion_state"),
            ["arm-execution", "perception"]
        );
        assert_eq!(mirrored_namespaces("manipulation_state"), ["arm-execution"]);
        assert!(mirrored_namespaces("world_scene").is_empty());
        assert!(mirrored_namespaces("absolute_pose").is_empty());
    }
}
