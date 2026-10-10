use std::{collections::BTreeMap, net::SocketAddr, sync::Arc};

use axum::{
    Router,
    extract::{Query, State, WebSocketUpgrade, ws::Message},
    http::{StatusCode, header},
    response::{IntoResponse, Response},
    routing::get,
};
use eyre::{Context, Result};
use futures::{SinkExt, StreamExt};
use robot_arm_messages::{CameraRawVideoFrame, CameraRole};
use serde::Deserialize;
use serde_json::json;

#[derive(Clone)]
struct VideoState {
    frames: VideoStreams,
}

pub(crate) type VideoStreams =
    BTreeMap<CameraRole, tokio::sync::watch::Receiver<Option<Arc<CameraRawVideoFrame>>>>;
#[derive(Default, Deserialize)]
struct View {
    #[serde(default)]
    role: CameraRole,
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
    frames: VideoStreams,
    mut shutdown: tokio::sync::watch::Receiver<bool>,
) -> Result<()> {
    let app = Router::new()
        .route("/health", get(|| async { StatusCode::NO_CONTENT }))
        .route("/ws", get(video_upgrade))
        .route("/snapshot", get(snapshot))
        .with_state(VideoState { frames });
    axum::serve(listener, app)
        .with_graceful_shutdown(async move {
            while !*shutdown.borrow() && shutdown.changed().await.is_ok() {}
        })
        .await?;
    Ok(())
}

async fn video_upgrade(
    ws: WebSocketUpgrade,
    State(state): State<VideoState>,
    Query(view): Query<View>,
) -> Response {
    let Some(frames) = state.frames.get(&view.role).cloned() else {
        return (StatusCode::NOT_FOUND, "相机角色未配置").into_response();
    };
    ws.on_upgrade(move |socket| video_socket(socket, frames))
        .into_response()
}

async fn snapshot(State(state): State<VideoState>, Query(view): Query<View>) -> Response {
    let Some(mut frames) = state.frames.get(&view.role).cloned() else {
        return (StatusCode::NOT_FOUND, "相机角色未配置").into_response();
    };
    if frames.borrow_and_update().is_none() {
        return (StatusCode::SERVICE_UNAVAILABLE, "相机未绑定或没有图像信号").into_response();
    }
    // A new frame after this request, not a stale preview or perception cache.
    if frames.changed().await.is_err() {
        return (StatusCode::SERVICE_UNAVAILABLE, "相机已停止").into_response();
    }
    let Some(frame) = frames.borrow_and_update().clone() else {
        return (StatusCode::SERVICE_UNAVAILABLE, "相机信号中断").into_response();
    };
    let metadata = serde_json::json!({"role": view.role, "source_id":frame.source_id,"sequence":frame.sequence,"received_time_ns":frame.received_time_ns,"width":frame.color.width,"height":frame.color.height,"pixel_format":frame.color.pixel_format});
    let encoded = tokio::task::spawn_blocking(move || -> Result<Vec<u8>> {
        let rgb = image::RgbImage::from_raw(
            frame.color.width,
            frame.color.height,
            frame.color.packed_rgb()?,
        )
        .ok_or_else(|| eyre::eyre!("无效 RGB 图像"))?;
        let mut out = std::io::Cursor::new(Vec::new());
        rgb.write_to(&mut out, image::ImageFormat::Png)?;
        Ok(out.into_inner())
    })
    .await;
    match encoded {
        Ok(Ok(bytes)) => (
            [
                (header::CONTENT_TYPE.as_str(), "image/png"),
                (header::CACHE_CONTROL.as_str(), "no-store"),
                ("x-camera-frame", &metadata.to_string()),
            ],
            bytes,
        )
            .into_response(),
        error => (
            StatusCode::INTERNAL_SERVER_ERROR,
            format!("相机图像编码失败：{error:?}"),
        )
            .into_response(),
    }
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
            } else if sender
                .send(Message::Text(
                    json!({"signal":false,"message":"相机信号中断或未绑定"})
                        .to_string()
                        .into(),
                ))
                .await
                .is_err()
            {
                return;
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
    async fn role_snapshot_waits_for_a_new_frame_and_keeps_pixels_with_metadata() {
        let frame = |sequence, source: &str, color| {
            Arc::new(CameraRawVideoFrame {
                schema_version: SCHEMA_VERSION,
                sequence,
                source_id: source.into(),
                received_time_ns: sequence as i64,
                color: CameraImagePlane {
                    width: 1,
                    height: 1,
                    stride_bytes: 3,
                    pixel_format: "rgb8".into(),
                    frame_id: "optical".into(),
                    data: color,
                },
            })
        };
        let (external, e) =
            tokio::sync::watch::channel(Some(frame(1, "external-source", vec![255, 0, 0])));
        let (_wrist, w) =
            tokio::sync::watch::channel(Some(frame(7, "wrist-source", vec![0, 0, 255])));
        let state = VideoState {
            frames: BTreeMap::from([(CameraRole::External, e), (CameraRole::Wrist, w)]),
        };
        let pending = snapshot(
            State(state.clone()),
            Query(View {
                role: CameraRole::External,
            }),
        );
        tokio::pin!(pending);
        // Starting the read cannot return the frame from before the request.
        assert!(
            tokio::time::timeout(std::time::Duration::from_millis(10), &mut pending)
                .await
                .is_err()
        );
        external
            .send(Some(frame(2, "external-source", vec![0, 255, 0])))
            .unwrap();
        let response = pending.await;
        assert_eq!(response.status(), StatusCode::OK);
        let metadata: serde_json::Value =
            serde_json::from_str(response.headers()["x-camera-frame"].to_str().unwrap()).unwrap();
        assert_eq!(metadata["sequence"], 2);
        assert_eq!(metadata["source_id"], "external-source");
        let bytes = axum::body::to_bytes(response.into_body(), usize::MAX)
            .await
            .unwrap();
        assert_eq!(
            image::load_from_memory(&bytes)
                .unwrap()
                .into_rgb8()
                .into_raw(),
            vec![0, 255, 0]
        );
        external.send(None).unwrap();
        assert_eq!(
            snapshot(
                State(state),
                Query(View {
                    role: CameraRole::External
                })
            )
            .await
            .status(),
            StatusCode::SERVICE_UNAVAILABLE
        );
    }

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
            let server = tokio::spawn(serve(
                listener,
                BTreeMap::from([(CameraRole::Depth, frames)]),
                stopped,
            ));
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
                } else {
                    let status = client.next().await.unwrap().unwrap();
                    assert!(status.to_text().unwrap().contains("signal"));
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
