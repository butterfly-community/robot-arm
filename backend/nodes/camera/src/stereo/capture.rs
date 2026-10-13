//! Stereo RGB-D adapter: reuses bound streams, emits the normal frame contract.
use super::pairing::Pair;
use super::*;
pub fn sources() -> Vec<robot_arm_messages::CameraSourceInfo> {
    let Ok(s) = shared() else { return vec![] };
    [Rig::External, Rig::Wrist]
        .into_iter()
        .filter_map(|rig| {
            let saved = s.saved.read().unwrap().get(&rig).cloned()?;
            let available = rig.roles().iter().enumerate().all(|(i, role)| {
                s.videos[role].borrow().as_ref().is_some_and(|frame| {
                    frame.latest.source_id == saved.sources[i]
                        && frame.latest.color.width == saved.optics.width
                        && frame.latest.color.height == saved.optics.height
                })
            });
            let profiles = [
                ("color", robot_arm_messages::CameraStreamKind::Color, "rgb8"),
                (
                    "depth",
                    robot_arm_messages::CameraStreamKind::Depth,
                    "z16le",
                ),
            ]
            .into_iter()
            .map(
                |(kind, stream, format)| robot_arm_messages::CameraStreamProfile {
                    key: format!(
                        "{kind}:{}x{}:{format}:{}",
                        saved.optics.width, saved.optics.height, saved.capture_fps
                    ),
                    stream,
                    width: saved.optics.width,
                    height: saved.optics.height,
                    frames_per_second: saved.capture_fps,
                    pixel_format: format.into(),
                    is_default: true,
                    available,
                    unavailable_reason: (!available)
                        .then(|| "请开启与保存标定一致的左右相机及分辨率".into()),
                },
            )
            .collect();
            Some(robot_arm_messages::CameraSourceInfo {
                source_id: rig.source().into(),
                driver_id: "stereo".into(),
                display_name: match rig {
                    Rig::External => "外部双目深度",
                    Rig::Wrist => "腕部双目深度",
                }
                .into(),
                device_model: Some("OpenCV StereoSGBM".into()),
                serial_number: None,
                firmware_version: None,
                connection_type: Some("复用已绑定左右相机".into()),
                physical_port: None,
                sensors: vec!["color".into(), "depth".into()],
                profiles,
                driver_extensions: vec![],
                available,
            })
        })
        .collect()
}

pub struct Stream {
    shared: Arc<Shared>,
    rig: Rig,
    saved: Saved,
    last_pair: Option<[i64; 2]>,
    pending_pair: Option<Pair>,
    sequence: u64,
    next: std::time::Instant,
}
impl Stream {
    pub fn open(source: &str) -> Result<Self> {
        let rig = match source {
            "stereo:external" => Rig::External,
            "stereo:wrist" => Rig::Wrist,
            _ => return Err(eyre!("未知双目来源")),
        };
        let shared = shared()?;
        let saved = shared
            .saved
            .read()
            .unwrap()
            .get(&rig)
            .cloned()
            .ok_or_else(|| eyre!("请先完成并应用双目标定"))?;
        let p = pair(&shared, rig)?;
        p.validate(&saved)?;
        Ok(Self {
            shared,
            rig,
            saved,
            last_pair: None,
            pending_pair: None,
            sequence: 0,
            next: std::time::Instant::now(),
        })
    }
}
#[cfg(test)]
impl Stream {
    pub(super) fn test_stream(shared: Arc<Shared>, rig: Rig, saved: Saved) -> Self {
        Self {
            shared,
            rig,
            saved,
            last_pair: None,
            pending_pair: None,
            sequence: 0,
            next: std::time::Instant::now(),
        }
    }
    pub(super) fn test_next_poll(&mut self) {
        self.next = std::time::Instant::now();
    }
}
impl crate::drivers::CameraStream for Stream {
    fn poll_frame(&mut self, materialize: bool) -> Result<crate::drivers::CameraPoll> {
        if self.next > std::time::Instant::now() {
            return Ok(crate::drivers::CameraPoll::Pending);
        }
        self.next = std::time::Instant::now()
            + std::time::Duration::from_secs_f64(1.0 / self.saved.capture_fps as f64);
        self.saved = self
            .shared
            .saved
            .read()
            .unwrap()
            .get(&self.rig)
            .cloned()
            .ok_or_else(|| eyre!("双目标定不存在"))?;
        // Check the live bindings even while a pair waits for joint feedback.
        let current = pair(&self.shared, self.rig)?;
        current.validate(&self.saved)?;
        let p = self.pending_pair.take().unwrap_or(current);
        p.validate(&self.saved)?;
        if self
            .last_pair
            .is_some_and(|last| last[0] == p.times[0] || last[1] == p.times[1])
        {
            return Ok(crate::drivers::CameraPoll::Pending);
        }
        if materialize && self.rig == Rig::Wrist && self.saved.camera_pose.is_some() {
            let history = self.shared.feedback_history.lock().unwrap();
            if history
                .back()
                .is_none_or(|v| v.arm_state.sample_time_ns < p.times[0])
            {
                // Image and feedback travel independently. Wait for this image's
                // bracketing feedback rather than disconnecting or chasing newer images.
                self.pending_pair = Some(p);
                return Ok(crate::drivers::CameraPoll::Pending);
            }
            if history
                .front()
                .is_none_or(|v| v.arm_state.sample_time_ns > p.times[0])
            {
                // A startup image older than the feedback history cannot be paired.
                return Ok(crate::drivers::CameraPoll::Pending);
            }
        }
        // Resolve feedback before the expensive matcher can age it out of history.
        let left_pose = if materialize {
            camera_in_base(
                &self.shared,
                self.rig,
                &self.saved,
                p.times[0],
                &iso_pose(&nalgebra::Isometry3::identity()),
            )?
        } else {
            None
        };
        self.last_pair = Some(p.times);
        self.sequence += 1;
        let left = p.left()?;
        if !materialize {
            let raw = robot_arm_messages::CameraRawVideoFrame {
                schema_version: SCHEMA_VERSION,
                sequence: self.sequence,
                source_id: self.rig.source().into(),
                device_time_ns: p.times[0],
                received_time_ns: p.times[0],
                color: CameraImagePlane {
                    width: p.width,
                    height: p.height,
                    stride_bytes: p.width * 3,
                    pixel_format: "rgb8".into(),
                    frame_id: format!("{}_rectified_optical", self.rig.source().replace(':', "_")),
                    data: self
                        .shared
                        .with_matcher(self.rig, &self.saved.optics, |m| m.rectify_color(&left))?,
                },
            };
            return Ok(crate::drivers::CameraPoll::Captured {
                video_frame: Box::new(camera_media::NativeDecoder::default().decode(raw)?),
                frame: None,
            });
        }
        let right = p.right()?;
        let depth = self
            .shared
            .with_matcher(self.rig, &self.saved.optics, |m| m.compute(&left, &right))?;
        let time = p.times[0];
        let frame_id = format!("{}_rectified_optical", self.rig.source().replace(':', "_"));
        let color = CameraImagePlane {
            width: p.width,
            height: p.height,
            stride_bytes: p.width * 3,
            pixel_format: "rgb8".into(),
            frame_id: frame_id.clone(),
            data: depth.rgb,
        };
        let camera_pose =
            left_pose.map(|pose| iso_pose(&(pose * pose_iso(&depth.rectified_in_left))));
        let k = depth.k;
        let calibration = camera_pose.map(|pose| DepthCameraCalibration {
            schema_version: SCHEMA_VERSION,
            sequence: self.sequence,
            source_time_ns: time,
            source_id: self.rig.source().into(),
            parent_frame_id: "base_link".into(),
            frame_id: frame_id.clone(),
            translation_m: pose.position_m,
            orientation_xyzw: pose.orientation_xyzw,
            width: p.width,
            height: p.height,
            distortion_model: "plumb_bob".into(),
            distortion: vec![0.0; 5],
            camera_matrix: k,
            projection_matrix: [
                k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0,
            ],
        });
        let bundle = Box::new(CameraFrameBundle {
            schema_version: SCHEMA_VERSION,
            sequence: self.sequence,
            source_id: self.rig.source().into(),
            device_time_ns: time,
            device_time_domain: "host_receipt_paired".into(),
            received_time_ns: time,
            color: color.clone(),
            aligned_depth: CameraImagePlane {
                width: p.width,
                height: p.height,
                stride_bytes: p.width * 2,
                pixel_format: "z16le".into(),
                frame_id,
                data: depth
                    .z_m
                    .iter()
                    .flat_map(|z| {
                        let mm = (*z * 1000.0).round();
                        let v = if mm.is_finite() && mm >= 1.0 && mm <= u16::MAX as f32 {
                            mm as u16
                        } else {
                            0
                        };
                        v.to_le_bytes()
                    })
                    .collect(),
            },
            intrinsics: CameraIntrinsics {
                width: p.width,
                height: p.height,
                focal_length_px: [k[0], k[4]],
                principal_point_px: [k[2], k[5]],
                distortion_model: "plumb_bob".into(),
                distortion: vec![0.0; 5],
            },
            depth_scale_m: 0.001,
            calibration,
        });
        let video = robot_arm_messages::CameraRawVideoFrame {
            schema_version: SCHEMA_VERSION,
            sequence: self.sequence,
            source_id: self.rig.source().into(),
            device_time_ns: time,
            received_time_ns: time,
            color,
        };
        Ok(crate::drivers::CameraPoll::Captured {
            video_frame: Box::new(camera_media::NativeDecoder::default().decode(video)?),
            frame: Some(bundle),
        })
    }
}
