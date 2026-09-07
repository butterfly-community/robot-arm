use std::{net::SocketAddr, sync::Arc};

use axum::{
    Router,
    extract::{State, WebSocketUpgrade, ws::Message},
    http::StatusCode,
    response::{IntoResponse, Response},
    routing::get,
};
use eyre::{Context, Result};
use futures::{SinkExt, StreamExt};
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
    socket: axum::extract::ws::WebSocket,
    mut frames: tokio::sync::watch::Receiver<Option<Arc<CameraRawVideoFrame>>>,
) {
    let (mut sender, mut receiver) = socket.split();
    let send_frames = async {
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
                if sender
                    .send(Message::Text(metadata.to_string().into()))
                    .await
                    .is_err()
                    || sender
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
    };
    // Read even while there are no frames or a slow viewer blocks writes.
    // Axum/tungstenite handles Ping/Pong and queues the Close response.
    tokio::select! {
        _ = send_frames => {}
        _ = async {
            while let Some(Ok(message)) = receiver.next().await {
                if matches!(message, Message::Close(_)) {
                    break;
                }
            }
        } => {}
    }
    // Flush the library's queued Close before dropping the transport.
    let _ = sender.close().await;
}

#[cfg(test)]
mod tests {
    use super::*;
    use robot_arm_messages::{CameraImagePlane, SCHEMA_VERSION};
    use tokio_tungstenite::{connect_async, tungstenite::Message as ClientMessage};

    #[tokio::test]
    async fn ping_and_close_work_with_and_without_video_frames() {
        for streaming in [false, true] {
            let frame = streaming.then(|| {
                Arc::new(CameraRawVideoFrame {
                    schema_version: SCHEMA_VERSION,
                    sequence: 1,
                    source_id: "simulation:test".into(),
                    received_time_ns: 1,
                    color: CameraImagePlane {
                        width: 1,
                        height: 1,
                        stride_bytes: 3,
                        pixel_format: "rgb8".into(),
                        frame_id: "optical".into(),
                        data: vec![255, 0, 0],
                    },
                })
            });
            let (_frames, frames) = tokio::sync::watch::channel(frame);
            let (shutdown, stopped) = tokio::sync::watch::channel(false);
            let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
            let address = listener.local_addr().unwrap();
            let server = tokio::spawn(serve(listener, frames, stopped));
            tokio::time::timeout(std::time::Duration::from_secs(3), async {
                let (mut client, _) = connect_async(format!("ws://{address}/ws")).await.unwrap();
                if streaming {
                    let metadata = client.next().await.unwrap().unwrap();
                    let metadata: serde_json::Value =
                        serde_json::from_str(metadata.to_text().unwrap()).unwrap();
                    assert_eq!(metadata["pixel_format"], "rgb8");
                    assert_eq!(
                        client.next().await.unwrap().unwrap(),
                        ClientMessage::Binary(vec![255, 0, 0].into())
                    );
                }
                let ping = b"preview".to_vec();
                client
                    .send(ClientMessage::Ping(ping.clone().into()))
                    .await
                    .unwrap();
                assert_eq!(
                    client.next().await.unwrap().unwrap(),
                    ClientMessage::Pong(ping.into())
                );
                client.send(ClientMessage::Close(None)).await.unwrap();
                assert!(matches!(
                    client.next().await,
                    Some(Ok(ClientMessage::Close(_)))
                ));
            })
            .await
            .expect("Ping/Close must not wait for the next camera frame");
            shutdown.send(true).unwrap();
            server.await.unwrap().unwrap();
        }
    }
}
