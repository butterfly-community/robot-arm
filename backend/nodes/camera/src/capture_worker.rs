use std::{sync::Arc, time::Duration};

use robot_arm_messages::{
    CameraDriverParameterValue, CameraFrameBundle, CameraSourceInfo, CameraStreamProfile,
    RequestAction, ToolPose,
};
use tokio::sync::mpsc as async_mpsc;

use crate::{
    drivers::{CameraPoll, Drivers},
    encoding_worker::{EncodingWorker, Job},
};

pub(crate) enum Command {
    CloseColor {
        request_id: String,
        action: RequestAction,
    },
    OpenColor {
        request_id: String,
        action: RequestAction,
        source_id: String,
        color: CameraStreamProfile,
    },
    Discover {
        request_id: String,
        action: RequestAction,
    },
    Open {
        request_id: String,
        action: RequestAction,
        source_id: String,
        color: CameraStreamProfile,
        depth: CameraStreamProfile,
        driver_parameters: Vec<CameraDriverParameterValue>,
        output_frames_per_second: f64,
        capture_frames_per_second: f64,
    },
    Stop,
    Reset {
        request_id: String,
        action: RequestAction,
        source_id: String,
    },
    SetCalibrationActive(bool),
    Shutdown,
}

pub(crate) enum Completion {
    Closed {
        request_id: String,
        action: RequestAction,
    },
    Discovered {
        request_id: String,
        action: RequestAction,
        sources: Vec<CameraSourceInfo>,
        errors: Vec<String>,
    },
    Opened {
        request_id: String,
        action: RequestAction,
        source_id: String,
        result: Result<(), String>,
    },
    Reset {
        request_id: String,
        action: RequestAction,
        source_id: String,
        result: Result<(), String>,
    },
}

#[derive(Clone)]
pub(crate) enum Output {
    Capture {
        sequence: u64,
        received_time_ns: i64,
        frame: Box<CameraFrameBundle>,
        captured_count: u64,
        skipped_count: u64,
    },
    Failed(String),
}

pub(crate) struct Channels {
    pub(crate) commands: async_mpsc::Sender<Command>,
    pub(crate) tool_pose: tokio::sync::watch::Sender<Option<ToolPose>>,
    pub(crate) completions: async_mpsc::Receiver<Completion>,
    pub(crate) outputs: tokio::sync::watch::Receiver<Option<Output>>,
    pub(crate) video: tokio::sync::watch::Receiver<Option<Arc<camera_media::VideoFrame>>>,
}

pub(crate) fn spawn(runtime: &tokio::runtime::Runtime) -> Channels {
    let (commands, mut command_receiver) = async_mpsc::channel(8);
    // Pose is replaceable state, not a command. Slow capture must never queue
    // old poses ahead of user controls or fill their acknowledgement queue.
    let (tool_pose, mut pose_receiver) = tokio::sync::watch::channel(None::<ToolPose>);
    let (completion_sender, completions) = async_mpsc::channel(8);
    let (output_sender, outputs) = tokio::sync::watch::channel(None);
    let (video_sender, video_outputs) = tokio::sync::watch::channel(None);
    runtime.spawn_blocking(move || {
        let mut drivers = Drivers::new().map_err(|error| format!("{error:#}"));
        let mut stream = None;
        let mut encoder: Option<EncodingWorker> = None;
        let mut output_rate = None;
        let mut last_output_at = None;
        let mut captured_count = 0;
        let mut skipped_count = 0;
        loop {
            if command_receiver.is_closed() {
                return;
            }
            if pose_receiver.has_changed().unwrap_or(false)
                && let Some(pose) = pose_receiver.borrow_and_update().clone()
                && let Ok(drivers) = drivers.as_mut()
            {
                drivers.update_tool_pose(pose);
            }
            while let Ok(command) = command_receiver.try_recv() {
                match command {
                    Command::CloseColor { request_id, action } => {
                        encoder = None;
                        // Acknowledge only after STREAMOFF/close releases USB bandwidth.
                        drop(stream.take());
                        output_rate = None;
                        output_sender.send_replace(None);
                        video_sender.send_replace(None);
                        let _ = completion_sender
                            .blocking_send(Completion::Closed { request_id, action });
                    }
                    Command::OpenColor {
                        request_id,
                        action,
                        source_id,
                        color,
                    } => {
                        encoder = None;
                        drop(stream.take());
                        output_sender.send_replace(None);
                        video_sender.send_replace(None);
                        let result = drivers.as_mut().map_err(|e| e.clone()).and_then(|d| {
                            d.open_color(&source_id, &color)
                                .map_err(|e| format!("{e:#}"))
                        });
                        output_rate = None;
                        let result = result.map(|opened| {
                            stream = Some(opened);
                            // Color-only bindings publish live video, not RGB-D.
                            output_rate = Some((0.0, f64::from(color.frames_per_second)));
                            last_output_at = None;
                        });
                        let _ = completion_sender.blocking_send(Completion::Opened {
                            request_id,
                            action,
                            source_id,
                            result,
                        });
                    }
                    Command::Discover { request_id, action } => {
                        let (sources, errors) = match drivers.as_mut() {
                            Ok(drivers) => drivers.discover(),
                            Err(error) => (vec![], vec![error.clone()]),
                        };
                        let _ = completion_sender.blocking_send(Completion::Discovered {
                            request_id,
                            action,
                            sources,
                            errors,
                        });
                    }
                    Command::Open {
                        request_id,
                        action,
                        source_id,
                        color,
                        depth,
                        driver_parameters,
                        output_frames_per_second,
                        capture_frames_per_second,
                    } => {
                        encoder = None;
                        drop(stream.take());
                        output_sender.send_replace(None);
                        video_sender.send_replace(None);
                        let opened_source_id = source_id.clone();
                        let result =
                            drivers
                                .as_mut()
                                .map_err(|error| error.clone())
                                .and_then(|drivers| {
                                    drivers
                                        .open(&source_id, &color, &depth, &driver_parameters)
                                        .map_err(|error| format!("{error:#}"))
                                });
                        match result {
                            Ok(opened) => {
                                stream = Some(opened);
                                output_rate =
                                    Some((output_frames_per_second, capture_frames_per_second));
                                last_output_at = None;
                                captured_count = 0;
                                skipped_count = 0;
                                let _ = completion_sender.blocking_send(Completion::Opened {
                                    request_id,
                                    action,
                                    source_id: opened_source_id,
                                    result: Ok(()),
                                });
                            }
                            Err(error) => {
                                stream = None;
                                output_rate = None;
                                let _ = completion_sender.blocking_send(Completion::Opened {
                                    request_id,
                                    action,
                                    source_id: opened_source_id,
                                    result: Err(error),
                                });
                            }
                        }
                    }
                    Command::Stop => {
                        encoder = None;
                        output_sender.send_replace(None);
                        video_sender.send_replace(None);
                        stream = None;
                        output_rate = None;
                        last_output_at = None;
                        captured_count = 0;
                        skipped_count = 0;
                    }
                    Command::Reset {
                        request_id,
                        action,
                        source_id,
                    } => {
                        encoder = None;
                        output_sender.send_replace(None);
                        video_sender.send_replace(None);
                        stream = None;
                        output_rate = None;
                        last_output_at = None;
                        captured_count = 0;
                        skipped_count = 0;
                        let result =
                            drivers
                                .as_mut()
                                .map_err(|error| error.clone())
                                .and_then(|drivers| {
                                    drivers
                                        .reset(&source_id)
                                        .map_err(|error| format!("{error:#}"))
                                });
                        let _ = completion_sender.blocking_send(Completion::Reset {
                            request_id,
                            action,
                            source_id,
                            result,
                        });
                    }
                    Command::SetCalibrationActive(active) => {
                        if let Ok(drivers) = drivers.as_mut() {
                            drivers.set_calibration_active(active);
                        }
                    }
                    Command::Shutdown => return,
                }
            }
            let Some(active_stream) = stream.as_mut() else {
                std::thread::sleep(Duration::from_millis(2));
                continue;
            };
            let (output_frames_per_second, capture_frames_per_second) =
                output_rate.expect("active stream has output rate");
            let materialize = output_frames_per_second > 0.0
                && super::output_due(
                    last_output_at.map(|last: std::time::Instant| last.elapsed()),
                    output_frames_per_second,
                    capture_frames_per_second,
                );
            match active_stream.poll_frame(materialize).and_then(|poll| {
                let CameraPoll::Captured {
                    video_frame, frame, ..
                } = poll
                else {
                    return Ok(None);
                };
                if encoder.is_none() {
                    encoder = Some(EncodingWorker::spawn(
                        video_sender.clone(),
                        output_sender.clone(),
                    )?);
                }
                captured_count += 1;
                if frame.is_some() {
                    // Sampling belongs to acquisition, not encode completion.
                    last_output_at = Some(std::time::Instant::now());
                } else {
                    skipped_count += 1;
                }
                encoder.as_ref().unwrap().submit(Job {
                    video: *video_frame,
                    sample: frame,
                    fps: capture_frames_per_second,
                    captured_count,
                    skipped_count,
                })?;
                Ok(Some(()))
            }) {
                Ok(None) => std::thread::sleep(Duration::from_millis(1)),
                Ok(Some(())) => {}
                Err(error) => {
                    encoder = None;
                    video_sender.send_replace(None);
                    stream = None;
                    output_rate = None;
                    let _ = output_sender.send(Some(Output::Failed(format!("{error:#}"))));
                }
            }
        }
    });
    Channels {
        commands,
        tool_pose,
        completions,
        outputs,
        video: video_outputs,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn close_color_acknowledges_after_clearing_video() {
        let runtime = tokio::runtime::Runtime::new().unwrap();
        let mut channels = spawn(&runtime);
        let capacity = channels.commands.capacity();
        for i in 0..1000 {
            channels
                .tool_pose
                .send(Some(ToolPose {
                    frame: "base_link".into(),
                    position_m: [i as f64, 0.0, 0.0],
                    orientation_xyzw: [0.0, 0.0, 0.0, 1.0],
                }))
                .unwrap();
        }
        assert_eq!(channels.commands.capacity(), capacity);
        assert_eq!(
            channels.tool_pose.borrow().as_ref().unwrap().position_m[0],
            999.0
        );
        channels
            .commands
            .try_send(Command::CloseColor {
                request_id: "close-wrist".into(),
                action: RequestAction::Disconnect,
            })
            .unwrap();
        let completion = runtime.block_on(async {
            tokio::time::timeout(Duration::from_secs(5), channels.completions.recv())
                .await
                .unwrap()
                .unwrap()
        });
        assert!(
            matches!(completion, Completion::Closed { request_id, action: RequestAction::Disconnect } if request_id == "close-wrist")
        );
        assert!(channels.video.borrow().is_none());
        drop(channels);
        runtime.shutdown_background();
    }

    #[test]
    fn worker_exits_when_its_owner_drops_the_command_channel() {
        let runtime = tokio::runtime::Runtime::new().unwrap();
        let Channels {
            commands,
            mut outputs,
            ..
        } = spawn(&runtime);
        drop(commands);
        let result = runtime.block_on(async {
            tokio::time::timeout(Duration::from_secs(5), outputs.changed()).await
        });
        // The timeout is only a regression deadline, not a device policy.
        runtime.shutdown_background();
        assert!(result.unwrap().is_err());
    }
}
