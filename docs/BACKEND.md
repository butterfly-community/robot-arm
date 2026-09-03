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

该 crate 是 RealSense 硬件驱动边界。`RealSenseDriver::discover()` 从 librealsense context 读取
设备、传感器、所有可用彩色/深度 profile，以及各 sensor 实际支持的 option。option 以
`librealsense2` 命名空间包装为设备扩展，报告当前值、默认值、范围、步长和只读性；公共契约与
网页不依赖 `Rs2Option`。`open()` 先应用用户修改的可写 option，再只启用请求中的驱动已报告
profile 并创建 pipeline。`RealSenseStream::next_frameset()` 从同一 frameset 取得两帧，读取实际
stride、格式、设备时间、两路内参、畸变、深度到彩色外参和 `depth_units()`。

来源键使用 RealSense 序列号；持久 profile 键只包含 stream、宽高、格式和 FPS；option 键使用
sensor 名称与 option 编号。三者都不含 SDK 枚举索引、USB 端口或进程内句柄。驱动仍导出当前
Rust 路径不能打开的 profile，并通过 `available` 与原因让上层置灰，而不是悄悄过滤能力。

librealsense/realsense-rust 类型、C 数据指针和唯一 `unsafe` 均封闭在这里。帧数据在句柄有效期内
复制到设备无关 `CameraFrameBundle`；pipeline 由 Rust 所有权在 stream drop 时释放。节点、感知、
网关和前端都不依赖厂商类型。`docs-only` feature 只供无 SDK 的编译检查，运行镜像启用
`runtime`。

## `camera-capture-node`

节点的本地 `CameraDriver`/`CameraStream` trait 只定义发现、打开、取完整 frameset 和可选重置。
硬件适配文件只是委托 `realsense-camera` crate；simulation 适配器实现同一 trait。

- `refresh()` 只在明确请求时聚合驱动发现结果，不在 tick 扫描 USB；保存过但当前离线的来源与
  暂时消失的 profile 通过最后一次能力快照保留为不可用项。
- `select()` 校验来源、profile、上送频率和驱动扩展均属于最近一次发现结果，并按来源保存。
- `start()` 创建 stream，重置运行统计；disconnect/unselect/drop 直接释放 stream。
- `tick()` 始终非阻塞排空设备产生的完整 frameset，只按配置的上送频率发布最新 Arrow bundle；
  设备序号缺口与主动略过中间帧分别计数。
- `state()` 报告来源、每来源保存配置、运行选择、streaming、最后 sequence/时间、采集/上送实测
  FPS、设备缺帧、主动略过和原始错误。

配置行为参考输入采集节点：以驱动提供的稳定来源身份为键，由后端保存并原子替换；运行时枚举
索引不进入文件。与手柄不同，相机保存的是两路 profile、上送 FPS 和用户实际修改的厂商参数，
当前选中来源和 streaming 不保存。重启后仍默认未选择，重新选择同一设备时恢复该设备的配置。

节点不执行图像变换。设备采集 FPS 来自所选 profile；上送 FPS 可配置为不高于彩色/深度两路
共同采集频率。例如设备以 60 FPS 采集而上层只需 1 FPS 时，capture 仍持续取帧以避免设备队列
反压，但只发送每个周期最新的完整 RGB-D 帧束。该节流不触发 YOLOE/GraspGenX；两个模型仍只
响应网页“运行一次感知”。

## `robot-arm-messages`

普通小消息使用共享 JSON Arrow codec。`CameraFrameBundle` 使用专用
`camera_frame_to_arrow()`/`camera_frame_from_arrow()`：元数据保留结构化 schema，彩色和深度数据
各用一个 Binary buffer。解码先核对 schema、宽高、stride、格式最小字节数和 buffer 长度；损坏
或截断数据返回明确错误。

相机时间分为设备时间及其时钟域、主机接收时间。`camera_in_base` 不属于 bundle：这是标定结果；
`depth_to_color` 属于设备内在参数，二者不会混用。

## `perception-node`

节点只接受 `CameraFrameBundle` 与 camera state。相机 streaming 变化决定当前来源；取消选择会
清除缓存图像和旧 `WorldScene`，不会发布伪造空帧。

`handle_camera_frame()` 拒绝非当前来源和回退 sequence，只保存最新原始帧与快照，不调用模型。
用户点击“运行一次感知”后，`process_latest_scene()` 对当时最新的一份原子 RGB-D 帧依次执行：

1. `align_depth_to_color()` 使用 bundle 的两路内参、畸变和外参对齐深度；
2. `segment()` 调用唯一计算服务获取提示词实例分割；
3. `perception-core` 反投影实例深度并转换到 `base_link`；
4. `attach_grasp_candidates()` 只把实例点云与夹爪资产 ID 发给 GraspGenX；
5. 发布一份 `WorldScene`，保存按需 Web 调试资源。

保存提示词只更新配置；相机持续来帧也不会自动运行 YOLOE 或 GraspGenX。这样模型调用次数完全
由显式请求决定，刷新图像也只更新预览。

对齐实现覆盖 librealsense 的 Brown-Conrady、Modified/Inverse Brown-Conrady、F-Theta 与
Kannala-Brandt 语义；零畸变的 `plumb_bob` 用于通用模拟/ROS 兼容元数据。未知且非零的畸变模型
直接报错，不悄悄当针孔模型。公式与命名按 librealsense 官方 projection/deprojection 实现核对。

`start_automatic_calibration()` 校验当前相机、机械臂版本、标定板和型号姿态，然后通过正式
control-mode 与 `MotionRequest` 输出开始流程。`advance_automatic_calibration()` 等待每个运动的
正式状态，成功后稳定等待 10 秒；`try_automatic_calibration_capture()` 只在新帧中检测成功后
记录同期 `ArmState`、FK TCP 和板位姿。`solve_calibration()` 调用 Rust/OpenCV 5 helper；
`apply_solved_calibration()` 才持久化结果。

配置保存模型启用状态、提示词、放置区标签和按相机来源隔离的已应用外参。运行中的标定会话不
持久化；重启不会恢复半个工作流。simulation 的预置外参来自数据资产，重置后恢复该真值；真实
来源重置后回到未标定。

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

Gateway 只保存最近一份小状态和按 request ID 配对的结果。相机请求直接转给 capture，感知与标定
请求转给 perception；按需图像资源使用二进制 HTTP 响应。原始持续 RGB-D frame 不进入 Gateway，
Gateway 也不解析或保存任何服务配置。
