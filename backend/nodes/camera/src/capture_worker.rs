use std::{sync::Arc, time::Duration};

use robot_arm_messages::{
    CameraDriverParameterValue, CameraFrameBundle, CameraRawVideoFrame, CameraSourceInfo,
    CameraStreamProfile, RequestAction, ToolPose,
};
use tokio::sync::mpsc as async_mpsc;

use crate::drivers::{CameraPoll, Drivers};

pub(crate) enum Command {
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
    UpdateToolPose(ToolPose),
    Shutdown,
}

pub(crate) enum Completion {
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
    pub(crate) completions: async_mpsc::Receiver<Completion>,
    pub(crate) outputs: tokio::sync::watch::Receiver<Option<Output>>,
    pub(crate) video: tokio::sync::watch::Receiver<Option<Arc<CameraRawVideoFrame>>>,
}

pub(crate) fn spawn(runtime: &tokio::runtime::Runtime) -> Channels {
    let (commands, mut command_receiver) = async_mpsc::channel(8);
    let (completion_sender, completions) = async_mpsc::channel(8);
    let (output_sender, outputs) = tokio::sync::watch::channel(None);
    let (video_sender, video_outputs) = tokio::sync::watch::channel(None);
    runtime.spawn_blocking(move || {
        let mut drivers = Drivers::new().map_err(|error| format!("{error:#}"));
        let mut stream = None;
        let mut output_rate = None;
        let mut last_output_at = None;
        let mut captured_count = 0;
        let mut skipped_count = 0;
        loop {
            while let Ok(command) = command_receiver.try_recv() {
                match command {
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
                    Command::UpdateToolPose(pose) => {
                        if let Ok(drivers) = drivers.as_mut() {
                            drivers.update_tool_pose(pose);
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
            let materialize = super::output_due(
                last_output_at.map(|last: std::time::Instant| last.elapsed()),
                output_frames_per_second,
                capture_frames_per_second,
            );
            match active_stream.poll_frame(materialize) {
                Ok(CameraPoll::Pending) => std::thread::sleep(Duration::from_millis(1)),
                Ok(CameraPoll::Captured {
                    sequence,
                    received_time_ns,
                    video_frame,
                    frame,
                }) => {
                    captured_count += 1;
                    let _ = video_sender.send(Some(Arc::from(video_frame)));
                    if let Some(frame) = frame {
                        last_output_at = Some(std::time::Instant::now());
                        let _ = output_sender.send(Some(Output::Capture {
                            sequence,
                            received_time_ns,
                            frame,
                            captured_count,
                            skipped_count,
                        }));
                    } else {
                        skipped_count += 1;
                    }
                }
                Err(error) => {
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
        completions,
        outputs,
        video: video_outputs,
    }
}
