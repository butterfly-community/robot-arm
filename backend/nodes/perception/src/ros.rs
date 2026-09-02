use std::{
    collections::BTreeSet,
    process::Command,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
        mpsc::Sender,
    },
    thread,
    time::Duration,
};

use eyre::{Context, Result};
use futures::{StreamExt, executor::block_on};
use r2r::{Context as RosContext, Node, Publisher, QosProfile};
use robot_arm_messages::DepthCameraSourceInfo;
use robot_arm_messages::{DepthCameraCalibration, DepthPointCloudFrame, WorldScene};
use serde_json::Value;

const RECTIFICATION_MATRIX: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

#[derive(Debug)]
pub enum RosEvent {
    Color(String, r2r::sensor_msgs::msg::Image),
    Depth(String, r2r::sensor_msgs::msg::Image),
    DepthInfo(String, r2r::sensor_msgs::msg::CameraInfo),
}

pub struct RosInterface {
    node: Arc<Mutex<Node>>,
    event_sender: Sender<RosEvent>,
    subscribed_sources: Arc<Mutex<BTreeSet<String>>>,
    color_publisher: Publisher<r2r::sensor_msgs::msg::Image>,
    color_info_publisher: Publisher<r2r::sensor_msgs::msg::CameraInfo>,
    depth_publisher: Publisher<r2r::sensor_msgs::msg::Image>,
    depth_info_publisher: Publisher<r2r::sensor_msgs::msg::CameraInfo>,
    cloud_publisher: Publisher<r2r::sensor_msgs::msg::PointCloud2>,
    marker_publisher: Publisher<r2r::visualization_msgs::msg::MarkerArray>,
    segmentation_publisher: Publisher<r2r::sensor_msgs::msg::Image>,
    calibration_debug_publisher: Publisher<r2r::sensor_msgs::msg::Image>,
    static_tf_publisher: Publisher<r2r::tf2_msgs::msg::TFMessage>,
}

impl RosInterface {
    pub fn start(event_sender: Sender<RosEvent>, stop: Arc<AtomicBool>) -> Result<Self> {
        let context = RosContext::create().context("初始化 ROS 2 perception context")?;
        let mut node =
            Node::create(context, "perception_node", "").context("创建 ROS 2 perception 节点")?;
        let color_publisher =
            node.create_publisher("/perception/color/image_raw", QosProfile::sensor_data())?;
        let color_info_publisher =
            node.create_publisher("/perception/color/camera_info", QosProfile::sensor_data())?;
        let depth_publisher =
            node.create_publisher("/perception/depth/image_raw", QosProfile::sensor_data())?;
        let depth_info_publisher =
            node.create_publisher("/perception/depth/camera_info", QosProfile::sensor_data())?;
        let cloud_publisher =
            node.create_publisher("/perception/depth/points", QosProfile::sensor_data())?;
        let marker_publisher =
            node.create_publisher("/perception/debug/markers", QosProfile::default())?;
        let segmentation_publisher =
            node.create_publisher("/perception/debug/segmentation", QosProfile::sensor_data())?;
        let calibration_debug_publisher =
            node.create_publisher("/perception/debug/calibration", QosProfile::sensor_data())?;
        let static_tf_publisher =
            node.create_publisher("/tf_static", QosProfile::default().transient_local())?;
        let node = Arc::new(Mutex::new(node));
        let spin_node = Arc::clone(&node);
        thread::spawn(move || {
            while !stop.load(Ordering::Relaxed) {
                {
                    spin_node
                        .lock()
                        .expect("ROS perception node mutex poisoned")
                        .spin_once(Duration::from_millis(10));
                }
                thread::yield_now();
            }
        });
        Ok(Self {
            node,
            event_sender,
            subscribed_sources: Arc::new(Mutex::new(BTreeSet::new())),
            color_publisher,
            color_info_publisher,
            depth_publisher,
            depth_info_publisher,
            cloud_publisher,
            marker_publisher,
            segmentation_publisher,
            calibration_debug_publisher,
            static_tf_publisher,
        })
    }

    pub fn discover_cameras(&self) -> Result<Vec<DepthCameraSourceInfo>> {
        const DEPTH_SUFFIX: &str = "/aligned_depth_to_color/image_raw";
        let topics = self
            .node
            .lock()
            .expect("ROS perception node mutex poisoned")
            .get_topic_names_and_types()?;
        let names = topics.keys().cloned().collect::<BTreeSet<_>>();
        let mut sources = Vec::new();
        for depth_topic in names.iter().filter(|name| name.ends_with(DEPTH_SUFFIX)) {
            let source_id = depth_topic.trim_end_matches(DEPTH_SUFFIX).to_owned();
            let source = camera_source(source_id);
            if names.contains(&source.color_stream) && names.contains(&source.camera_info_stream) {
                sources.push(source);
            }
        }
        let device_info = enumerate_realsense_device_info();
        let single_source_and_device = sources.len() == 1 && device_info.len() == 1;
        for source in &mut sources {
            let matching = device_info
                .iter()
                .find(|info| {
                    string_field(info, "serial_number")
                        .is_some_and(|serial| source.source_id.contains(&serial))
                })
                .or_else(|| single_source_and_device.then(|| &device_info[0]));
            if let Some(info) = matching {
                apply_realsense_device_info(source, info);
            }
        }
        Ok(sources)
    }

    pub fn select_camera(&self, source: &DepthCameraSourceInfo) -> Result<()> {
        let mut selected = self
            .subscribed_sources
            .lock()
            .expect("ROS camera subscription mutex poisoned");
        if !selected.insert(source.source_id.clone()) {
            return Ok(());
        }
        let mut node = self
            .node
            .lock()
            .expect("ROS perception node mutex poisoned");
        let color = node.subscribe(&source.color_stream, QosProfile::sensor_data())?;
        let depth = node.subscribe(&source.depth_stream, QosProfile::sensor_data())?;
        let depth_info = node.subscribe(&source.camera_info_stream, QosProfile::sensor_data())?;
        drop(node);
        let id = source.source_id.clone();
        forward_stream(color, self.event_sender.clone(), {
            let id = id.clone();
            move |message| RosEvent::Color(id.clone(), message)
        });
        forward_stream(depth, self.event_sender.clone(), {
            let id = id.clone();
            move |message| RosEvent::Depth(id.clone(), message)
        });
        forward_stream(depth_info, self.event_sender.clone(), move |message| {
            RosEvent::DepthInfo(id.clone(), message)
        });
        Ok(())
    }

    pub fn select_camera_id(&self, source_id: &str) -> Result<()> {
        self.select_camera(&camera_source(source_id.to_owned()))
    }

    pub fn camera_source_info(source_id: &str) -> DepthCameraSourceInfo {
        camera_source(source_id.to_owned())
    }

    pub fn publish_color(&self, message: r2r::sensor_msgs::msg::Image) -> Result<()> {
        self.color_publisher.publish(&message)?;
        Ok(())
    }

    pub fn publish_depth(&self, message: r2r::sensor_msgs::msg::Image) -> Result<()> {
        self.depth_publisher.publish(&message)?;
        Ok(())
    }

    pub fn publish_segmentation(&self, message: r2r::sensor_msgs::msg::Image) -> Result<()> {
        self.segmentation_publisher.publish(&message)?;
        Ok(())
    }

    pub fn publish_calibration_debug(&self, message: r2r::sensor_msgs::msg::Image) -> Result<()> {
        self.calibration_debug_publisher.publish(&message)?;
        Ok(())
    }

    pub fn publish_cloud(&self, cloud: DepthPointCloudFrame) -> Result<()> {
        let mut data = Vec::with_capacity(cloud.points_xyz_m.len() * 12);
        for point in cloud.points_xyz_m {
            for coordinate in point {
                data.extend_from_slice(&coordinate.to_le_bytes());
            }
        }
        self.cloud_publisher
            .publish(&r2r::sensor_msgs::msg::PointCloud2 {
                header: r2r::std_msgs::msg::Header {
                    stamp: ros_time(cloud.source_time_ns),
                    frame_id: cloud.frame_id,
                },
                height: cloud.height,
                width: cloud.width,
                fields: ["x", "y", "z"]
                    .into_iter()
                    .enumerate()
                    .map(|(index, name)| r2r::sensor_msgs::msg::PointField {
                        name: name.into(),
                        offset: (index * 4) as u32,
                        datatype: 7,
                        count: 1,
                    })
                    .collect(),
                is_bigendian: false,
                point_step: 12,
                row_step: cloud.width * 12,
                data,
                is_dense: false,
            })?;
        Ok(())
    }

    pub fn cloud_subscription_count(&self) -> Result<usize> {
        Ok(self
            .cloud_publisher
            .get_inter_process_subscription_count()?)
    }

    pub fn publish_calibration(&self, calibration: &DepthCameraCalibration) -> Result<()> {
        let stamp = ros_time(calibration.source_time_ns);
        let info = r2r::sensor_msgs::msg::CameraInfo {
            header: r2r::std_msgs::msg::Header {
                stamp: stamp.clone(),
                frame_id: calibration.frame_id.clone(),
            },
            height: calibration.height,
            width: calibration.width,
            distortion_model: calibration.distortion_model.clone(),
            d: calibration.distortion.clone(),
            k: calibration.camera_matrix.to_vec(),
            r: RECTIFICATION_MATRIX.to_vec(),
            p: calibration.projection_matrix.to_vec(),
            ..Default::default()
        };
        self.depth_info_publisher.publish(&info)?;
        self.color_info_publisher.publish(&info)?;
        let [x, y, z] = calibration.translation_m;
        let [qx, qy, qz, qw] = calibration.orientation_xyzw;
        self.static_tf_publisher
            .publish(&r2r::tf2_msgs::msg::TFMessage {
                transforms: vec![r2r::geometry_msgs::msg::TransformStamped {
                    header: r2r::std_msgs::msg::Header {
                        stamp,
                        frame_id: calibration.parent_frame_id.clone(),
                    },
                    child_frame_id: calibration.frame_id.clone(),
                    transform: r2r::geometry_msgs::msg::Transform {
                        translation: r2r::geometry_msgs::msg::Vector3 { x, y, z },
                        rotation: r2r::geometry_msgs::msg::Quaternion {
                            x: qx,
                            y: qy,
                            z: qz,
                            w: qw,
                        },
                    },
                }],
            })?;
        Ok(())
    }

    pub fn publish_markers(&self, scene: &WorldScene) -> Result<()> {
        let mut markers = Vec::new();
        for (index, object) in scene.objects.iter().enumerate() {
            let [x, y, z] = object.pose.position_m;
            let [qx, qy, qz, qw] = object.pose.orientation_xyzw;
            let [sx, sy, sz] = object.size_m;
            markers.push(r2r::visualization_msgs::msg::Marker {
                header: r2r::std_msgs::msg::Header {
                    stamp: ros_time(scene.sample_time_ns),
                    frame_id: scene.frame_id.clone(),
                },
                ns: "perception_objects".into(),
                id: index as i32,
                type_: 1,
                action: 0,
                pose: r2r::geometry_msgs::msg::Pose {
                    position: r2r::geometry_msgs::msg::Point { x, y, z },
                    orientation: r2r::geometry_msgs::msg::Quaternion {
                        x: qx,
                        y: qy,
                        z: qz,
                        w: qw,
                    },
                },
                scale: r2r::geometry_msgs::msg::Vector3 {
                    x: sx,
                    y: sy,
                    z: sz,
                },
                color: r2r::std_msgs::msg::ColorRGBA {
                    r: 0.2,
                    g: 0.7,
                    b: 1.0,
                    a: 0.65,
                },
                ..Default::default()
            });
            markers.push(r2r::visualization_msgs::msg::Marker {
                header: r2r::std_msgs::msg::Header {
                    stamp: ros_time(scene.sample_time_ns),
                    frame_id: scene.frame_id.clone(),
                },
                ns: "perception_labels".into(),
                id: index as i32,
                type_: 9,
                action: 0,
                pose: r2r::geometry_msgs::msg::Pose {
                    position: r2r::geometry_msgs::msg::Point {
                        x,
                        y,
                        z: z + sz / 2.0,
                    },
                    orientation: r2r::geometry_msgs::msg::Quaternion {
                        w: 1.0,
                        ..Default::default()
                    },
                },
                scale: r2r::geometry_msgs::msg::Vector3 {
                    z: 0.02,
                    ..Default::default()
                },
                color: r2r::std_msgs::msg::ColorRGBA {
                    r: 0.9,
                    g: 0.95,
                    b: 1.0,
                    a: 1.0,
                },
                text: object.label.clone(),
                ..Default::default()
            });
        }
        for (index, region) in scene.placement_regions.iter().enumerate() {
            let [x, y, z] = region.pose.position_m;
            let [sx, sy, _] = region.size_m;
            markers.push(r2r::visualization_msgs::msg::Marker {
                header: r2r::std_msgs::msg::Header {
                    stamp: ros_time(scene.sample_time_ns),
                    frame_id: scene.frame_id.clone(),
                },
                ns: "perception_placement_regions".into(),
                id: index as i32,
                type_: 1,
                action: 0,
                pose: r2r::geometry_msgs::msg::Pose {
                    position: r2r::geometry_msgs::msg::Point { x, y, z },
                    orientation: r2r::geometry_msgs::msg::Quaternion {
                        w: 1.0,
                        ..Default::default()
                    },
                },
                scale: r2r::geometry_msgs::msg::Vector3 {
                    x: sx,
                    y: sy,
                    z: 0.002,
                },
                color: r2r::std_msgs::msg::ColorRGBA {
                    r: 0.2,
                    g: 1.0,
                    b: 0.45,
                    a: 0.55,
                },
                ..Default::default()
            });
        }
        self.marker_publisher
            .publish(&r2r::visualization_msgs::msg::MarkerArray { markers })?;
        Ok(())
    }

    pub fn clear_markers(&self) -> Result<()> {
        self.marker_publisher
            .publish(&r2r::visualization_msgs::msg::MarkerArray {
                markers: vec![r2r::visualization_msgs::msg::Marker {
                    action: 3,
                    ..Default::default()
                }],
            })?;
        Ok(())
    }
}

fn camera_source(source_id: String) -> DepthCameraSourceInfo {
    let display_name = source_id.trim_start_matches('/').replace('/', " / ");
    DepthCameraSourceInfo {
        driver_id: "ros2".into(),
        device_model: None,
        serial_number: None,
        firmware_version: None,
        connection_type: None,
        physical_port: None,
        sensors: Vec::new(),
        color_stream: format!("{source_id}/color/image_raw"),
        depth_stream: format!("{source_id}/aligned_depth_to_color/image_raw"),
        camera_info_stream: format!("{source_id}/aligned_depth_to_color/camera_info"),
        depth_scale_m: 0.001,
        calibrated: false,
        source_id,
        display_name,
    }
}

fn enumerate_realsense_device_info() -> Vec<Value> {
    let Ok(output) = Command::new("rs-enumerate-devices").arg("-c").output() else {
        return Vec::new();
    };
    if !output.status.success() {
        return Vec::new();
    }
    parse_realsense_device_info(&String::from_utf8_lossy(&output.stdout))
}

fn parse_realsense_device_info(output: &str) -> Vec<Value> {
    let mut devices = Vec::new();
    let mut current = None::<serde_json::Map<String, Value>>;
    for line in output.lines() {
        if line.trim() == "Device info:" {
            if let Some(device) = current.take() {
                devices.push(Value::Object(device));
            }
            current = Some(serde_json::Map::new());
            continue;
        }
        let Some(device) = current.as_mut() else {
            continue;
        };
        let Some((label, value)) = line.split_once(':') else {
            continue;
        };
        let key = match label.trim() {
            "Name" => "device_name",
            "Serial Number" => "serial_number",
            "Firmware Version" => "firmware_version",
            "Physical Port" => "physical_port",
            "Usb Type Descriptor" => "usb_type_descriptor",
            _ => continue,
        };
        device.insert(key.into(), Value::String(value.trim().to_owned()));
    }
    if let Some(device) = current {
        devices.push(Value::Object(device));
    }
    devices
}

fn apply_realsense_device_info(source: &mut DepthCameraSourceInfo, info: &Value) {
    let raw_model = string_field(info, "device_name");
    source.device_model = raw_model.as_deref().map(realsense_model_name);
    source.serial_number = string_field(info, "serial_number");
    source.firmware_version = string_field(info, "firmware_version");
    source.connection_type =
        string_field(info, "usb_type_descriptor").map(|version| format!("USB {version}"));
    source.physical_port = string_field(info, "physical_port");
    source.sensors = vec!["color".into(), "depth".into()];
    if let Some(model) = source.device_model.as_deref() {
        source.display_name = match source.serial_number.as_deref() {
            Some(serial) => format!("{model} · {serial}"),
            None => model.to_owned(),
        };
    }
}

fn string_field(value: &Value, field: &str) -> Option<String> {
    value[field]
        .as_str()
        .map(str::trim)
        .filter(|value| !value.is_empty())
        .map(str::to_owned)
}

fn realsense_model_name(raw: &str) -> String {
    if raw.starts_with("RealSense ") {
        return raw.to_owned();
    }
    let model = raw.strip_prefix("realsense_").unwrap_or(raw);
    format!("RealSense {}", model.replace('_', " ").to_uppercase())
}

fn forward_stream<T: Send + 'static>(
    mut stream: impl futures::Stream<Item = T> + Unpin + Send + 'static,
    sender: Sender<RosEvent>,
    wrap: impl Fn(T) -> RosEvent + Send + 'static,
) {
    thread::spawn(move || {
        block_on(async move {
            while let Some(message) = stream.next().await {
                let _ = sender.send(wrap(message));
            }
        });
    });
}

pub fn ros_time(time_ns: i64) -> r2r::builtin_interfaces::msg::Time {
    r2r::builtin_interfaces::msg::Time {
        sec: time_ns.div_euclid(1_000_000_000) as i32,
        nanosec: time_ns.rem_euclid(1_000_000_000) as u32,
    }
}

pub fn time_ns(time: &r2r::builtin_interfaces::msg::Time) -> i64 {
    i64::from(time.sec) * 1_000_000_000 + i64::from(time.nanosec)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn realsense_device_info_populates_generic_camera_metadata() {
        let mut source = camera_source("/camera/camera".into());
        apply_realsense_device_info(
            &mut source,
            &serde_json::json!({
                "device_name": "realsense_d415",
                "serial_number": "924322061032",
                "firmware_version": "5.12.7.100",
                "usb_type_descriptor": "2.1",
                "sensors": "depth_module,rgb_camera",
                "physical_port": "/sys/devices/example/video0",
            }),
        );

        assert_eq!(source.display_name, "RealSense D415 · 924322061032");
        assert_eq!(source.device_model.as_deref(), Some("RealSense D415"));
        assert_eq!(source.connection_type.as_deref(), Some("USB 2.1"));
        assert_eq!(source.sensors, ["color", "depth"]);
    }

    #[test]
    fn parses_compact_device_fields_without_stream_profile_assumptions() {
        let devices = parse_realsense_device_info(
            "Device info:\n\
                 Name : RealSense D415\n\
                 Serial Number : 924322061032\n\
                 Firmware Version : 5.12.7.100\n\
                 Physical Port : /sys/devices/example/video0\n\
                 Usb Type Descriptor : 2.1\n\n\
             Stream Profiles supported by Stereo Module\n\
                 Depth 1280x720 Z16 @ 6 Hz\n",
        );

        assert_eq!(devices.len(), 1);
        assert_eq!(
            string_field(&devices[0], "device_name").as_deref(),
            Some("RealSense D415")
        );
        assert_eq!(
            string_field(&devices[0], "serial_number").as_deref(),
            Some("924322061032")
        );
        assert!(devices[0].get("width").is_none());
        assert!(devices[0].get("fps").is_none());
    }
}
