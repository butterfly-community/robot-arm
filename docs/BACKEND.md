# 后端方法与依赖

本文只记录实现边界和关键方法。全局数据流见 [当前系统设计](SUMMARY.md)，StarArm-102 的数值
和补丁见 [型号适配](STARARM-102.md)。

## `controller-input-node`

`ControllerInput::load()` 通过 `json-config-store` 读取设备名称、Action 与反馈绑定；
`commit_config()` 先写盘再替换内存配置。`drain()` 合并驱动事件，`tick()` 只把新样本转成统一
绝对位姿和 Action。`combined_pose_frame()` 允许位置和姿态来自不同设备；
`evaluate_actions()` 将按钮、连续轴或正负按钮对转成设备无关动作。

NOLO 协议解析在 `nolo-cv1` crate，SDL3 依据运行时 `has_axis()`、`has_button()`、sensor 与
haptic 能力发布组件，两者的 IMU 都使用 `fusion-ahrs`。位置和连续轴过滤复用
`one_euro_filter`。模拟输入声明同一 Action，不另建下游测试路径。

## `spatial-transform-node`

节点只处理配置和 Dora I/O，数学集中在 `spatial-core::SpatialTransform`。
`update_pose()` 接收组合绝对位姿，`handle_control()` 接收 Action，`current_output()` 按当前接管
原点完成换基、相对位姿和缺少绝对来源分量的积分。矩阵与四元数使用 `nalgebra`；夹爪和控制
Action 只透明传递。

## `realsense-camera` crate

该 crate 是 RealSense 硬件边界。`discover()` 读取设备、传感器、所有彩色/深度 profile 和 sensor
option；`open()` 应用用户选择后建立 pipeline。`poll_frame(materialize)` 始终排空同步 frameset，
但只有 `materialize=true` 时才调用 librealsense `Align` 并复制彩色和已对齐深度。输出尺寸、
stride、格式、内参、时间和 `depth_units()` 全部来自实际帧，不假定型号、分辨率或 FPS。

来源键使用序列号；profile 键只含 stream、宽高、格式和 FPS；option 键使用 sensor 名称与 option
编号。SDK 类型与指针只存在于该 crate。`docs-only` feature 供无 SDK 环境检查，运行镜像启用
`runtime`。

## `camera-node`

节点统一管理硬件与 simulation 适配器、持久配置、采集和标定。Dora 循环只处理请求、状态和
发布；Tokio `spawn_blocking` 长期任务拥有驱动、pipeline 与非 `Send` 的 Align，通过有界命令通道
和 latest-value 通道通信。设备 60 FPS、上层 1 FPS 时仍排空 60 FPS，但约每秒只 Align、复制并
发布一次。

- `refresh()` 只在按钮请求时发现设备，并把已保存但暂时离线的来源/profile 标成不可用。
- `select()` 校验驱动刚报告的 profile、上送 FPS 与厂商扩展参数，再按稳定来源身份保存。
- `start()`、`stop_capture()` 与 `reset()` 都在同一采集任务内操作驱动，不维护第二套硬件状态。
- `tick()` 读取最新采集结果、绑定该帧的外参快照并发布专用 Arrow Binary bundle。
- 标定状态机消费机械臂反馈和型号声明的姿态，进程内调用 `camera-calibration` 对 OpenCV 5 的
  `CharucoDetector`、`solvePnP` 与 `calibrateRobotWorldHandEye` 包装。不存在标定子进程或
  JSON/Base64 IPC。

配置保存每台相机的 profile、上送 FPS、实际修改的驱动参数和已确认外参；运行选择与 streaming
不保存。simulation 的预置外参使用相同查询与逐帧发布逻辑，重置恢复预置；真实来源重置后未标定。

## `robot-arm-messages`

小消息使用共享 JSON Arrow codec。`CameraFrameBundle` 用专用 codec，把元数据与两个 Arrow Binary
图像 buffer 分开。彩色和深度必须同尺寸、同 frame id，且共享内参尺寸一致。bundle 原子携带已
对齐 RGB-D、深度比例、两个时钟和本帧外参快照；未标定时快照为 `None`。

## `scene-node`

节点缓存 `camera-node` 发布的最新原子帧，只在用户点击时启动一次感知任务。`reqwest::Client`
异步调用 YOLOE/GraspGenX；深度解码、掩码融合、场景重建和预览编码在 Tokio `spawn_blocking`
中执行，因此 Dora 循环仍可响应快照和状态。公开 `task_state` 驱动网页按钮锁定，同类任务不排队。

`scene-core` 负责已对齐深度与实例掩码的领域融合、坐标变换及 `WorldScene` 组织。scene 不发现
相机、不保存内外参、不驱动机械臂，也没有畸变/深度注册算法。提示词与放置区域角色由用户配置，
不写死测试类别。新帧不会自动触发模型，刷新静态预览也不会触发模型。

## `perception-compute`

FastAPI lifespan 只加载一次 `YoloeBackend` 与 `GraspGenXBackend`。`/v1/segment` 解码彩色图，
按请求调用 YOLOE 提示词识别/分割并返回类别、置信度、二维框和 PNG mask；`/v1/grasps` 接收一个
实例点云和 `gripper_asset_id`，返回该资产 TCP 的 SE(3) 候选、分数与分支。

CPU/CUDA 只改变运行设备，不改变接口。服务不连接相机、Dora、ROS 或 MoveIt，不读取类别名称
推断抓放规则。夹爪资产在基础镜像中从设备清单与最终补丁 URDF 自动生成，请求 ID 决定使用哪套
资产。

## `stararm-102-motion-node`

relative、manual、calibration 和 perception 请求进入一个顺序 `WorkItem` FIFO；唯一 ROS worker
依次同步控制器、暂停 Servo、规划/执行并恢复 Servo。普通运动使用 MoveGroup，抓放使用型号 MTC
组件。motion 只把 `WorldScene` 的结构化对象、放置区域和显式障碍映射到 PlanningScene，不订阅
相机、图像、PointCloud2 或 OctoMap。

所有 FK、IK、抓放、attach 和可视化统一使用模型声明的 `tcp_link`。MTC 采用标准
`GeneratePose`、`ComputeIK`、`MoveRelative`、`MoveTo`、`Connect` 与
`ModifyPlanningScene` stages；Rust 层不手写 IK 或 stage 状态机。

## `stararm-102-execution-node`

`configure_endpoint()` 保存用户串口选择并显式连接/断开；未选串口时同一 `ArmCommand` 产生软件
反馈，选择串口但连接失败时不会回退。`StarArmBus::encode_command()` 按模型映射总线指令；
Monitor 读取失败先在同一串口重试一次，仍失败才进入重连逻辑。

`primary_tool_feedback()` 只在设备边界把实际功率映射为 0–100 通用反馈。稳定 0 是有效样本，
不代表“没有反馈”；软件模式不伪造真机力度。

## `web-gateway-node`

Gateway 只保存最近一份小状态和按 request ID 配对的结果。相机及标定请求直接转给
`camera-node`，感知请求转给 `scene-node`；按需图像资源使用二进制 HTTP 响应。实时彩色视频由
`camera-node` 内置的 latest-value WebSocket 直接提供，反向代理只转发连接，不缓存帧。原始 RGB-D
frame 不进入 Gateway，Gateway 也不解析或保存任何服务配置。

## Next.js 抓放场景编排

`web-perception` 的服务端 Route Handler 使用 AI SDK 将自然语言先约束为开放词汇提示词和放置
角色，运行既有 `perception/request` 后，再根据实际 `WorldScene` 约束选择对象与放置区域 ID。
任务解析为每个用户指代生成从具体描述到常见视觉类别的少量英文同义提示词，避免把单一语言
翻译误当成模型固定词表；这些词仍全部由当前指令产生，不包含场景硬编码。
本地校验实例、抓取候选和区域都存在后，才调用既有 `motion/mode` 与 `perception/pick-place`。
它不是 Dora 节点，不新增消息，也不复制 scene、MTC、碰撞或执行逻辑；浏览器只收到编排结果，
接触不到 API 密钥。
