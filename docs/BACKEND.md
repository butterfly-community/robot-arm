# 后端方法与依赖

本文只记录实现边界和关键方法。全局数据流见 [当前系统设计](SUMMARY.md)，StarArm-102 的数值
和补丁见 [型号适配](STARARM-102.md)。

## `controller-input-node`

`ControllerInput::load()` 通过 `json-config-store` 读取设备名称、Action 与反馈绑定；
`commit_config()` 先写盘再替换内存配置。`drain()` 合并驱动事件，`tick()` 只把新样本转成统一
绝对位姿和 Action。`combined_pose_frame()` 允许位置和姿态来自不同设备；
`evaluate_actions()` 将按钮、连续轴或正负按钮对转成设备无关动作。

演示期间 `user_config()` 始终指向演示前的用户配置；`apply_user_config()` 更新该配置，
同时保留正在运行的临时演示绑定。改名/保存绑定不会把生成式演示配置写入文件，停止演示也不会撤销用户刚保存的修改。

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
每帧复制彩色供独立实时视频使用；只有 `materialize=true` 时才调用 librealsense `Align` 并复制已对齐深度。输出尺寸、
stride、格式、内参、时间和 `depth_units()` 全部来自实际帧，不假定型号、分辨率或 FPS。

来源键使用序列号；profile 键只含 stream、宽高、格式和 FPS；option 键使用 sensor 名称与 option
编号。SDK 类型与指针只存在于该 crate。`docs-only` feature 供无 SDK 环境检查，运行镜像启用
`runtime`。

## `camera-node`

节点统一管理硬件与 simulation 适配器、持久配置、采集和标定。Dora 循环只处理请求、状态和
发布；Tokio `spawn_blocking` 长期任务拥有驱动、pipeline 与非 `Send` 的 Align，通过有界命令通道
和 latest-value 通道通信。设备 60 FPS、上层 1 FPS 时仍排空 60 FPS 并提供彩色视频，但约每秒只 Align 并
发布一次 RGB-D。命令通道关闭时退出任务；停用或重开 pipeline 会清空旧帧，旧的打开结果不能覆盖新请求。

- `refresh()` 只在按钮请求时发现设备，并把已保存但暂时离线的来源/profile 标成不可用。
- `select()` 校验驱动刚报告的 profile、上送 FPS 与厂商扩展参数，再按稳定来源身份保存。
- `start()`、`stop_capture()` 与 `reset()` 都在同一采集任务内操作驱动，不维护第二套硬件状态。
- `tick()` 读取最新采集结果、绑定该帧的外参快照并发布专用 Arrow Binary bundle。
- 标定状态机消费机械臂反馈和型号声明的姿态，进程内调用 `camera-calibration` 对 OpenCV 5 的
  `CharucoDetector`、`solvePnP` 与 `calibrateRobotWorldHandEye` 包装。不存在标定子进程或
  JSON/Base64 IPC。

`detect_sample()` 和 `solve_calibration()` 在现有 Tokio 的阻塞任务池运行；Dora 循环通过
`poll_calibration_work()` 接收结果。会话取消后旧任务结果不再应用，不新增进程或服务。
自动流程先等运动节点确认进入手动模式，再逐个提交姿态；成功后等 10 秒并等下一帧采样。
会话进度只在内存中，只有确认应用的外参由 `apply_solved_calibration()` 先落盘再替换配置。

配置保存每台相机的 profile、上送 FPS、实际修改的驱动参数和已确认外参；运行选择与 streaming
不保存。simulation 的预置外参使用相同查询与逐帧发布逻辑，重置恢复预置；真实来源重置后未标定。

### 彩色通道约定

业务链路统一发布 **RGB8**，不是把原生 BGR buffer 改个名字。原生 profile 仍如实报告设备格式；
采集 worker 在驱动输出边界用 OpenCV `cvtColor` 将 BGR/BGRA/RGBA/灰度转换为 RGB8，原生 RGB8
直接通过。行填充按真实 stride 处理，深度仍为 Z16，不参与颜色转换。

| 边界 | 约定与实现 |
| --- | --- |
| camera → Arrow → scene / 视频 | RGB8；`CameraImagePlane::packed_rgb` 只移除行填充，不交换通道 |
| PNG、PIL、YOLOE API | RGB；传 PIL 图给官方 predictor，不另转 BGR NumPy |
| OpenCV 标定 | PNG 用 `IMREAD_COLOR_BGR` 解码；`CharucoDetector` 与 PNG 编码按 BGR 使用，结果 PNG 回到普通图像链路 |
| 浏览器 Canvas | RGB888 通过 `@thi.ng/pixel` 转 Canvas RGBA，不手写设备格式转换 |

OpenCV 5 支持 `IMREAD_COLOR_RGB`，但没有“所有算法切换 RGB”的全局开关。
三通道 ChArUco 检测内部按 BGR 转灰度，因此不能直接把 RGB Mat 当作 BGR 使用。
检测沿用官方默认的ArUco标记角点设置。ChArUco检测后再调用官方 `cornerSubPix`，
局部半窗口7×7、零区(-1,-1)、最多100次迭代/精度1e-4 px；PnP与手眼算法不变。
这些精修参数来自同一PNG/真值对照，结果见 [模拟标定验收](REVIEW.md)，不是额外的观测拒绝条件。
检测前的 OpenCV 高斯预滤波（σ=0.8 像素）由同图、同真值对照确定，用于减小像素采样相位
对梯度定位的影响；不改变几何或深度，诊断叠加仍绘制在原图上，不是新增检测通过门限。
需要直接处理 RGB 的新算法应明确使用 `COLOR_RGB2GRAY` 等对应参数；不要在全链路来回换色。
YOLOE 的官方 PIL loader 内部转 BGR，predictor 再转 RGB tensor，这是库内部契约，不应在调用前补一次转换。
ChArUco/PnP 接受 forward Brown/rational 或零畸变针孔输入，调用时同时传入模型。
非零 inverse/modified Brown、鱼眼投影不能直接当成相同系数，需要驱动提供校正彩色图及对应内参。
鱼眼即使系数全零也不是针孔投影，不能省略这个模型区别。
原始相机分辨率不等于模型张量输入尺寸：不再强制 `imgsz=max(image.size)`，由模型默认预处理
处理输入；`retina_masks=True` 保证输出掩码回到原图尺寸。相同提示词的实测中，强制 1920
输入没有检测结果，模型默认输入则识别到两项；相机图像、内参和标定仍保留 1920×1080。
见 [Ultralytics Predict 参数](https://docs.ultralytics.com/modes/predict/#inference-arguments)。
GraspGenX 当前官方场景推理使用目标/环境 XYZ；所用 sampler 的颜色张量为零，不把展示用点云颜色当作模型输入。

回归测试包含红/绿/蓝/非对称颜色、非整通道行填充、五种原生格式、OpenCV PNG 往返、
官方 Ultralytics loader/predictor 的最终 RGB tensor 和浏览器 RGB888 显示。
依据：[OpenCV 5 图像编解码接口](https://github.com/opencv/opencv/blob/5.0.0/modules/imgcodecs/include/opencv2/imgcodecs.hpp)、
[ChArUco 检测实现](https://github.com/opencv/opencv/blob/5.0.0/modules/objdetect/src/aruco/charuco_detector.cpp)。

## `robot-arm-messages`

小消息使用共享 JSON Arrow codec。`CameraFrameBundle` 用专用 codec，把元数据与两个 Arrow Binary
图像 buffer 分开。彩色和深度必须同尺寸、同 frame id，且共享内参尺寸一致。bundle 原子携带已
对齐 RGB-D、深度比例、两个时钟和本帧外参快照；未标定时快照为 `None`。

## `scene-node`

节点缓存 `camera-node` 发布的最新原子帧，只在用户点击时启动一次感知任务。`reqwest::Client`
异步调用 YOLOE/GraspGenX；深度解码、掩码融合、场景重建和预览编码在 Tokio `spawn_blocking`
中执行，因此 Dora 循环仍可响应状态。手动刷新静态预览的 PNG 编码仍在显式快照请求内执行。
公开 `task_state` 驱动网页按钮锁定，同类任务不排队。运行一次感知会持久化启用状态。
任务保存启动时的场景序号；相机、标定、模型或提示词变化都会使旧结果失效，不能靠同一来源 ID 判断新旧。

`scene-core` 负责已对齐深度与实例掩码的领域融合、坐标变换及 `WorldScene` 组织。scene 不发现
相机、不保存内外参、不驱动机械臂，也不手写畸变/深度注册算法。反投影调用 OpenCV 5
`undistortPoints`，支持 Brown-Conrady / rational 和 Kannala-Brandt / equidistant；
后者即使系数为零也调用 fisheye 版本，非鱼眼的零畸变才按针孔反投影。
非零畸变的其他模型明确返回不支持，不再忽略系数产生错误点云。深度注册仍由采集层 SDK 完成。
提示词与放置区域角色由用户配置，
不写死测试类别。新帧不会自动触发模型，刷新静态预览也不会触发模型。

## `perception-compute`

FastAPI lifespan 只加载一次 `YoloeBackend` 与 `GraspGenXBackend`。`/v1/segment` 解码彩色图，
按请求调用 YOLOE 提示词识别/分割并返回类别、置信度、二维框和 PNG mask；`/v1/grasps` 接收一个
实例点云、排除该实例的环境点云和 `gripper_asset_id`，返回该资产 TCP 的 SE(3) 候选、分数与分支。
YOLOE 的提示词更新和推理共用一把实例锁，避免并发请求混用类别；GraspGenX 也串行访问共享 sampler。

CPU/CUDA 只改变运行设备，不改变接口。服务不连接相机、Dora、ROS 或 MoveIt，不读取类别名称
推断抓放规则。夹爪资产在计算基础镜像中从设备清单与最终补丁 URDF 自动生成，`gripper_asset_id` 决定使用哪套
资产。

## `stararm-102-motion-node`

离散的 manual、calibration、准备相对控制及 perception 请求进入一个顺序 `WorkItem` FIFO；唯一 ROS worker
依次暂停 Servo、规划/执行并恢复 Servo。连续 relative 输入不进 FIFO，只有相对模式且队列空闲时才发送 Servo 位姿和输入夹爪动作。
普通运动由 MoveGroup 规划，并把未改写的关节轨迹交给现有 ros2_control 的 `FollowJointTrajectory` action；抓放使用型号 MTC
组件。motion 只从 `WorldScene` 提取目标中心、放置中心和抓取候选；MTC 场景只加入刚性地平面
和可附着的目标中心参考点，不订阅或转换相机、图像、PointCloud2、OctoMap 和结构化包围体。

所有 FK、IK、抓放、attach 和可视化统一使用模型声明的 `tcp_link`。MTC 采用标准
`GeneratePose`、`ComputeIK`、`MoveRelative`、`MoveTo`、`Connect` 与
`ModifyPlanningScene` stages；Rust 层不手写 IK 或 stage 状态机。

普通运动取消通过保留的控制器 ROS goal handle 调用 cancel；不再经过当前 MoveIt 2.15.0
会阻塞取消回调的 `ExecuteTrajectory` 中转，也不并存备用执行路径。HTTP 的取消确认不代表已经停止，
原请求收到 ROS 终态后才发布 `cancelled`。新请求被拒绝时只回应它自己的 request ID，不覆盖正在执行的状态。
抓放请求携带选中实例所属的 `scene_sequence`，进入队列前核对当前场景；HTTP 202 表示节点已接收排队，
完成/失败仍由 `manipulation_state` 表达。单独夹爪请求也使用统一 RequestResult 返回拒绝原因。

普通运动返回的成功码 `1` 仍表示整个 MoveIt 规划/控制器执行成功；控制器原生成功码为 `0`。
控制器失败不得套用 MoveIt 错误码名称。MTC 的执行及错误码由 MTC 自己返回。

## `stararm-102-execution-node`

`configure_endpoint()` 保存用户串口选择并显式连接/断开；未选串口时同一 `ArmCommand` 产生软件
反馈，选择串口但连接失败时不会回退。`StarArmBus::encode_command()` 按模型映射总线指令；
Monitor 读取失败先在同一串口重试一次，仍失败才进入重连逻辑。
读取/重连失败仍发布连接状态；断连后的最后硬件读数可以保留显示，但不再作为当前力度反馈发出。

`primary_tool_feedback()` 只在设备边界把实际功率映射为 0–100 通用反馈。稳定 0 是有效样本，
不代表“没有反馈”；软件模式不伪造真机力度。

## `web-gateway-node`

Gateway 只保存最近一份小状态和按 request ID 配对的结果。相机及标定请求直接转给
`camera-node`，感知请求转给 `scene-node`；按需图像资源使用二进制 HTTP 响应。实时彩色视频由
`camera-node` 内置的 latest-value WebSocket 直接提供，反向代理只转发连接，不缓存帧。原始 RGB-D
frame 不进入 Gateway，Gateway 也不解析或保存任何服务配置。

HTTP 入站复用 `robot-arm-messages` 请求类型校验；非法载荷在网关返回 400，不送到节点使其退出。
相同尚未完成的 request ID 不重复转发；退出先释放所有 pending 响应，再等待 HTTP 结束。
感知快照同时通知拥有原始图像的 camera 和拥有叠加图的 scene。模型配置请求不再携带已迁移到 camera 的来源选择字段。

## 公共配置存储

`json-config-store::save()` 先序列化完整 JSON，再使用 `atomic-write-file` 在同一目录写入并提交原子替换。
写入失败不截断原文件；各节点仍各自拥有配置，不增加配置服务、消息中转或前端副本。
参见 [AtomicWriteFile](https://docs.rs/atomic-write-file/latest/atomic_write_file/struct.AtomicWriteFile.html)。

## Next.js 抓放场景编排

`web-perception` 的服务端 Route Handler 使用 AI SDK 将自然语言先约束为开放词汇提示词和放置
角色，运行既有 `perception/request` 后，再根据实际 `WorldScene` 约束选择对象与放置区域 ID。
任务解析为每个用户指代生成从具体描述到常见视觉类别的少量英文同义提示词，避免把单一语言
翻译误当成模型固定词表；这些词仍全部由当前指令产生，不包含场景硬编码。
本地校验实例、抓取候选和区域都存在后，才调用既有 `motion/mode` 与 `perception/pick-place`。
它不是 Dora 节点，不新增消息，也不复制 scene、MTC、碰撞或执行逻辑；浏览器只收到编排结果，
接触不到 API 密钥。
