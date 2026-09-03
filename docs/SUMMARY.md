# 当前系统设计

## 服务边界

| 服务或节点 | 拥有的职责 | 明确不负责 |
| --- | --- | --- |
| `controller-input-node` | NOLO、SDL3、模拟输入的能力发现、绑定、命名和反馈路由 | 空间积分、机械臂运动学 |
| `spatial-transform-node` | 输入换基、接管原点、相对位姿与无绝对来源分量的积分 | 设备枚举、MoveIt |
| `camera-capture-node` | 相机刷新、能力与厂商扩展、profile/上送频率保存、打开/关闭、同步 RGB-D 帧发布 | 对齐、点云、标定、AI、ROS |
| `perception-node` | 深度对齐、外参标定、计算服务编排、结构化三维场景与抓取候选 | 相机 SDK、轨迹规划、硬件执行 |
| `perception-compute` | YOLOE 提示词识别/分割与 GraspGenX 抓取姿态推理 | 相机、Dora、ROS、动作编排 |
| `stararm-102-motion-node` | 控制模式、普通规划、Servo、MTC 抓放与结构化场景到 MoveIt 的映射 | 串口协议、相机原始数据 |
| `stararm-102-execution-node` | FashionStar 总线、执行反馈、模拟执行和型号元数据 | IK、感知、目标语义 |
| `service-status-node` | 按依赖图聚合服务就绪状态 | 业务恢复策略 |
| `web-gateway-node` | HTTP/WebSocket 与 Dora 请求、状态、按需资源转发 | 设备逻辑、图像计算、配置持久化 |

`perception-core`、`spatial-core`、`realsense-camera` 等是库而不是额外服务。capture 与
perception 部署在同一个 `perception` Dora machine/Compose 容器中，既保持进程边界，也不增加
部署服务。计算服务可部署到远端，但只和 `perception-node` 交互。

## 唯一数据流

控制链路：

`设备适配器 → controller-input → spatial-transform → motion/MoveIt → ArmCommand → execution`

感知链路：

`相机适配器 → camera-capture → CameraFrameBundle → perception → WorldScene → motion/MoveIt`

真实设备和模拟只在第一层适配器不同。下游没有模拟专用消息、备用 topic、双写或失败时自动
回退。没有选择相机是合法状态：capture 不发布假空帧，perception 清除旧场景有效性，手动和
相对控制仍可使用。

## 深度相机

相机硬件 SDK 不属于节点业务代码。`realsense-camera` crate 封装 librealsense context、设备与
profile 枚举、pipeline、frameset、厂商元数据和 FFI；`camera-capture-node` 内的 RealSense 文件
只是把 crate 接到统一 `CameraDriver`/`CameraStream` 接口。资源通过 Rust 所有权和 `Drop`
释放，不额外维护一套显式关闭状态机。

枚举只响应网页“刷新相机”。来源描述包含稳定 ID、驱动、型号、序列号、固件、USB/物理端口、
传感器和驱动实际报告的 profile。RealSense 稳定 ID 使用序列号，profile 和驱动参数键不含运行时
枚举索引。选择只允许当前可用能力；profile、上送频率和用户修改的驱动扩展参数按来源保存，
当前来源与 streaming 状态不保存。保存过的离线来源和暂时缺失的 profile 仍由后端导出，网页
置灰并说明原因，不会因一次枚举变化丢失配置。相机型号、分辨率、格式和 FPS 均不写死。
通用分辨率、格式与采集 FPS 直接进入统一契约；Intel 独有 sensor option 由 `librealsense2`
命名空间包装，simulation 不伪造该扩展，厂商字段不进入 perception、ROS 或 motion。

一次 frameset 只发布一个专用 Arrow `CameraFrameBundle`：元数据使用 JSON 字段，彩色与深度平面
是 Arrow Binary buffer，不使用 Base64 或数值 JSON 数组。bundle 同时包含宽高、stride、格式、
光学 frame、设备/主机时间、内参、畸变、深度到彩色外参和设备读取的深度比例。队列使用
`queue_size: 1` 和 `drop_oldest`。采集节点持续排空设备帧；可配置的上送频率只控制最新完整帧束
进入 Dora 的速率，不改变设备采集 profile。设备缺帧和主动略过分别报告。

`simulation:pick-place-scene` 与 `simulation:depth-grid` 是正式相机适配器，声明自己的 RGB-D
profile、标定元数据和确定性资产。前者还根据正式模拟执行反馈的 TCP 位姿渲染 ChArUco 板，
自动标定因此经过和真机相同的帧与运动消息链路。

## 感知、标定与规划

`perception-node` 持续缓存最新原子 RGB-D 帧；只有用户点击“运行一次感知”时，才按 bundle 的
两路内参、畸变与深度到彩色外参完成对齐并调用计算服务。保存提示词、刷新图像和相机持续来帧
都不会运行 YOLOE 或 GraspGenX。实例掩码与深度生成设备无关的 `SceneObject`、`PlacementRegion` 和
`SceneObstacle`；实例点云仅发送给 GraspGenX。计算服务按请求的夹爪资产 ID 工作，不读取机械臂
型号，也不向场景硬编码方块、筐或抓取姿态。

原始图像、点云和相机参数不进入 ROS。motion 只把 `WorldScene` 的结构化几何映射到唯一
PlanningScene；不再运行 ROS 相机驱动、`cv_bridge`、`PointCloudOctomapUpdater`、OctoMap
清理接口或感知 Marker 双写。MoveIt 继续负责机械臂自身、刚性地面及结构化场景的碰撞检查。

自动外参标定从 `RobotModelInfo.calibration_targets` 读取型号声明的姿态，通过现有手动关节
`MotionRequest` FIFO 逐项执行。每项运动成功后稳定等待 10 秒，再使用新 RGB 帧检测 ChArUco，
记录同一时刻的正式 `ArmState` 和 FK TCP 位姿。OpenCV 5 的 ChArUco、PnP 与
Robot-World/Hand-Eye SHAH 求解由 Rust `opencv` crate 调用；结果只有经网页确认后才按来源保存。

## 运动与执行

motion 维护一个顺序工作 FIFO。普通关节运动、标定姿态与抓放不会并行；上一个动作未结束时
下一个等待。抓放使用 MoveIt Task Constructor 标准 stages，输入只有通用对象、放置区域、抓取
候选与型号适配器声明的规划组/TCP/工具信息。所有 FK、IK、attach、可视化与执行统一使用
`tcp_link`。

execution 把唯一 `ArmCommand` 映射为真机总线或软件反馈。选择串口失败不会偷偷切到模拟；
未选择串口才是明确的软件执行模式。StarArm-102 的角度方向、舵机限制、命名位、夹爪联动与
厂家补丁集中在型号目录，见 [StarArm-102 型号适配](STARARM-102.md)。

## 配置与界面

需要持久化的节点复用 `json-config-store`，各自拥有一个 JSON 文件；不存在中央配置服务或前端
配置副本。相机 profile 属于 capture，外参和 AI 提示词属于 perception，绑定属于 input，串口和
反馈周期属于 execution。

感知页按“感知任务、相机来源与配置、相机外参标定、相机参数、采集数据、AI 模型与结果”组织。
相机与模型是不同所有者，但在一个页面中协作。彩色、深度、标定和叠加图只在用户请求快照时
通过网关返回；大 RGB-D 帧不会持续经过浏览器。Three.js 机械臂继续以 `ArmState` 显示真实/软件
反馈，以最后命令显示半透明目标，不由相机链路替代。

## 部署

Compose 只映射 Web `8765` 与 RViz/noVNC `6080`，其余服务在私有网络中。统一 `down`/`up -d`
重启全栈；Dora 容器启用最小 init 负责转发信号和回收子进程。镜像规则见
[Docker 与服务镜像](DOCKER.md)，逐方法实现见 [后端方法与依赖](BACKEND.md)。
