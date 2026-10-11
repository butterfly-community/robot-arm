//! One bounded pipeline per camera: device/decode and encode overlap, while
//! every accepted frame stays ordered. Decoded pixels stay inside FFmpeg;
//! MJPEG frames retain YUV planes instead of round-tripping through RGB.
use std::sync::{Arc, mpsc};

use eyre::{Result, eyre};
use robot_arm_messages::CameraFrameBundle;
use tokio::sync::watch;

use crate::capture_worker::Output;

pub(crate) struct Job {
    pub video: camera_media::DecodedFrame,
    pub sample: Option<Box<CameraFrameBundle>>,
    pub fps: f64,
    pub captured_count: u64,
    pub skipped_count: u64,
}

pub(crate) struct EncodingWorker {
    sender: Option<mpsc::SyncSender<Job>>,
    failure: mpsc::Receiver<String>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl EncodingWorker {
    pub fn spawn(
        video_sender: watch::Sender<Option<Arc<camera_media::VideoFrame>>>,
        output_sender: watch::Sender<Option<Output>>,
    ) -> Result<Self> {
        // One queued frame allows decode(N+1) to overlap encode(N), without
        // building a growing backlog of uncompressed pixel buffers.
        let (sender, receiver) = mpsc::sync_channel::<Job>(1);
        let (failure_sender, failure) = mpsc::channel();
        let thread = std::thread::Builder::new()
            .name("camera-encode".into())
            .spawn(move || {
                let mut encoder: Option<camera_media::Encoder> = None;
                for mut job in receiver {
                    let encoded = (|| -> Result<_> {
                        if encoder
                            .as_ref()
                            .is_none_or(|e| !e.matches(&job.video.source.color, job.fps))
                        {
                            encoder = Some(camera_media::Encoder::new(
                                job.video.source.color.width,
                                job.video.source.color.height,
                                job.fps,
                            )?);
                        }
                        encoder
                            .as_mut()
                            .unwrap()
                            .encode_decoded(job.video, job.sample.is_some())
                    })();
                    match encoded {
                        Ok(video) => {
                            if let Some(sample) = job.sample.as_mut() {
                                sample.color = video.latest.color.clone();
                            }
                            video_sender.send_replace(Some(Arc::new(video)));
                            if let Some(frame) = job.sample {
                                output_sender.send_replace(Some(Output::Capture {
                                    sequence: frame.sequence,
                                    received_time_ns: frame.received_time_ns,
                                    frame,
                                    captured_count: job.captured_count,
                                    skipped_count: job.skipped_count,
                                }));
                            }
                        }
                        Err(error) => {
                            let error = format!("{error:#}");
                            let _ = failure_sender.send(error.clone());
                            video_sender.send_replace(None);
                            output_sender.send_replace(Some(Output::Failed(error)));
                            break;
                        }
                    }
                }
            })?;
        Ok(Self {
            sender: Some(sender),
            failure,
            thread: Some(thread),
        })
    }

    pub fn submit(&self, job: Job) -> Result<()> {
        self.sender.as_ref().unwrap().send(job).map_err(|_| {
            eyre!(
                self.failure
                    .try_recv()
                    .unwrap_or_else(|_| "相机编码线程已退出".into())
            )
        })
    }
}

impl Drop for EncodingWorker {
    fn drop(&mut self) {
        // Drain and join before the capture owner clears its published video,
        // acknowledges close, or opens another source. No stale publication.
        self.sender.take();
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use robot_arm_messages::{CameraImagePlane, CameraRawVideoFrame, SCHEMA_VERSION};

    fn job(sequence: u64) -> Job {
        Job {
            video: camera_media::NativeDecoder::default()
                .decode(CameraRawVideoFrame {
                    schema_version: SCHEMA_VERSION,
                    sequence,
                    source_id: "test".into(),
                    device_time_ns: sequence as i64 * 16_666_667,
                    received_time_ns: sequence as i64 * 16_666_667,
                    color: CameraImagePlane {
                        width: 64,
                        height: 48,
                        stride_bytes: 64 * 3,
                        pixel_format: "rgb8".into(),
                        frame_id: "optical".into(),
                        data: [200, 40, 20].repeat(64 * 48),
                    },
                })
                .unwrap(),
            sample: None,
            fps: 60.0,
            captured_count: sequence,
            skipped_count: sequence,
        }
    }

    #[test]
    fn close_drains_frames_in_order_before_returning() {
        let (video_sender, video) = watch::channel(None);
        let (output_sender, _) = watch::channel(None);
        let worker = EncodingWorker::spawn(video_sender, output_sender).unwrap();
        for sequence in 1..=12 {
            worker.submit(job(sequence)).unwrap();
        }
        drop(worker);
        let frame = video.borrow().clone().unwrap();
        assert_eq!(frame.latest.sequence, 12);
        assert_eq!(
            frame.gop.iter().map(|f| f.sequence).collect::<Vec<_>>(),
            (1..=12).collect::<Vec<_>>()
        );
        assert_eq!(
            camera_media::decode(frame.gop.iter().map(|f| &f.color))
                .unwrap()
                .len(),
            64 * 48 * 3
        );
        assert!(
            video.has_changed().is_err(),
            "encoder must release publisher on close"
        );
    }

    #[test]
    fn rgbd_sample_keeps_its_own_sequence_and_capture_time() {
        let (video_sender, video) = watch::channel(None);
        let (output_sender, output) = watch::channel(None);
        let worker = EncodingWorker::spawn(video_sender, output_sender).unwrap();
        let mut captured = job(100);
        captured.sample = Some(Box::new(CameraFrameBundle {
            schema_version: SCHEMA_VERSION,
            sequence: 7,
            source_id: "test".into(),
            device_time_ns: 700,
            device_time_domain: "test-depth-clock".into(),
            received_time_ns: 800,
            color: captured.video.source.color.clone(),
            aligned_depth: CameraImagePlane {
                width: 64,
                height: 48,
                stride_bytes: 128,
                pixel_format: "z16le".into(),
                frame_id: "optical".into(),
                data: [232, 3].repeat(64 * 48),
            },
            intrinsics: robot_arm_messages::CameraIntrinsics {
                width: 64,
                height: 48,
                focal_length_px: [64., 64.],
                principal_point_px: [32., 24.],
                distortion_model: "none".into(),
                distortion: vec![],
            },
            depth_scale_m: 0.001,
            calibration: None,
        }));
        worker.submit(captured).unwrap();
        drop(worker);
        let video = video.borrow().clone().unwrap();
        assert_eq!(video.latest.sequence, 100);
        let Some(Output::Capture {
            sequence,
            received_time_ns,
            frame,
            ..
        }) = output.borrow().clone()
        else {
            panic!("RGB-D sample was not published");
        };
        assert_eq!(sequence, frame.sequence);
        assert_eq!(sequence, 7);
        assert_eq!(received_time_ns, frame.received_time_ns);
        assert_eq!(received_time_ns, 800);
        assert_eq!(frame.device_time_ns, 700);
        assert_eq!(frame.color, video.latest.color);
        assert_eq!(frame.aligned_depth.data, [232, 3].repeat(64 * 48));
        assert!(camera_media::decode_image(&frame.color).is_ok());
    }

    #[test]
    fn encoding_error_clears_video_and_preserves_original_cause() {
        let (video_sender, video) = watch::channel(None);
        let (output_sender, output) = watch::channel(None);
        let worker = EncodingWorker::spawn(video_sender, output_sender).unwrap();
        worker.submit(job(1)).unwrap();
        let mut invalid = job(2);
        invalid.video.source.color.height = 32;
        worker.submit(invalid).unwrap();
        drop(worker);
        assert!(video.borrow().is_none());
        assert!(
            matches!(output.borrow().as_ref(), Some(Output::Failed(error)) if error.contains("尺寸"))
        );
    }
}
