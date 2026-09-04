use std::{net::SocketAddr, sync::Arc};

use axum::{
    Router,
    extract::{State, WebSocketUpgrade, ws::Message},
    http::StatusCode,
    response::{IntoResponse, Response},
    routing::get,
};
use eyre::{Context, Result};
use robot_arm_messages::CameraRawVideoFrame;
use serde_json::json;

#[derive(Clone)]
struct VideoState {
    frames: tokio::sync::watch::Receiver<Option<Arc<CameraRawVideoFrame>>>,
}

pub(crate) async fn bind() -> Result<tokio::net::TcpListener> {
    let port = std::env::var("CAMERA_VIDEO_PORT")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(8081);
    tokio::net::TcpListener::bind(SocketAddr::from(([0, 0, 0, 0], port)))
        .await
        .with_context(|| format!("监听相机视频端口 {port}"))
}

pub(crate) async fn serve(
    listener: tokio::net::TcpListener,
    frames: tokio::sync::watch::Receiver<Option<Arc<CameraRawVideoFrame>>>,
    mut shutdown: tokio::sync::watch::Receiver<bool>,
) -> Result<()> {
    let app = Router::new()
        .route("/health", get(|| async { StatusCode::NO_CONTENT }))
        .route("/ws", get(video_upgrade))
        .with_state(VideoState { frames });
    axum::serve(listener, app)
        .with_graceful_shutdown(async move {
            while !*shutdown.borrow() && shutdown.changed().await.is_ok() {}
        })
        .await?;
    Ok(())
}

async fn video_upgrade(ws: WebSocketUpgrade, State(state): State<VideoState>) -> Response {
    ws.on_upgrade(move |socket| video_socket(socket, state.frames))
        .into_response()
}

async fn video_socket(
    mut socket: axum::extract::ws::WebSocket,
    mut frames: tokio::sync::watch::Receiver<Option<Arc<CameraRawVideoFrame>>>,
) {
    loop {
        let frame = frames.borrow_and_update().clone();
        if let Some(frame) = frame {
            let metadata = json!({
                "schema_version": frame.schema_version,
                "sequence": frame.sequence,
                "source_id": frame.source_id,
                "received_time_ns": frame.received_time_ns,
                "width": frame.color.width,
                "height": frame.color.height,
                "stride_bytes": frame.color.stride_bytes,
                "pixel_format": frame.color.pixel_format,
                "frame_id": frame.color.frame_id,
            });
            if socket
                .send(Message::Text(metadata.to_string().into()))
                .await
                .is_err()
                || socket
                    .send(Message::Binary(frame.color.data.clone().into()))
                    .await
                    .is_err()
            {
                return;
            }
        }
        if frames.changed().await.is_err() {
            return;
        }
    }
}
