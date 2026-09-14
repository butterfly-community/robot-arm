# 架构与调用链

本文记录当前实现边界和关键方法（方法表为源码导航，随后为契约说明）；StarArm-102 的数值和补丁见 [型号适配](STARARM-102.md)。

控制输入经过 `controller-input → spatial-transform → motion → execution`；相机经过
`camera → scene → WorldScene → motion`。scene 按显式请求分别调用分割和抓取模型。
真实设备与模拟只在驱动适配层不同；同一消息契约下没有失败回退的第二条业务链路。
`service-status` 只聚合就绪状态，`web-gateway` 只转发请求、状态和按需资源。
相机未选择是合法状态，不妨碍独立的手动/相对控制。

## `controller-input-node`

源码：[输入节点](../backend/nodes/controller-input/src/main.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `ControllerInput::load` / `commit_config` | 设备绑定配置 → 内存状态；保存先落盘 |
| `drain` / `tick` | 驱动事件 → 统一位姿与 Action |
| `combined_pose_frame` / `evaluate_actions` | 多设备组件 → 组合位姿、连续轴及按钮动作 |
| `apply_user_config` | 用户编辑 → 持久配置；不把临时演示绑定当用户配置 |

演示期间 `user_config()` 始终指向演示前的用户配置；`apply_user_config()` 更新该配置，
同时保留正在运行的临时演示绑定。改名/保存绑定不会把生成式演示配置写入文件，停止演示也不会撤销用户刚保存的修改。

NOLO 协议解析在 `nolo-cv1` crate，SDL3 依据运行时 `has_axis()`、`has_button()`、sensor 与
haptic 能力发布组件，两者的 IMU 都使用 `fusion-ahrs`。位置和连续轴过滤复用
`one_euro_filter`。模拟输入声明同一 Action，不另建下游测试路径。

## `spatial-transform-node`

源码：[空间核心](../backend/crates/spatial-core/src/lib.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `update_pose` | 输入源绝对位姿 → 最新空间状态 |
| `handle_control` | Action → 接管/积分状态 |
| `current_output` | 接管原点与当前输入 → 统一相对控制帧；不调用 ROS 或硬件 |

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

硬件诊断使用 `camera-node` 的 `realsense-smoke` Cargo example，直接调用此 crate；
不再引入整个节点驱动模块，也不重复编译模拟资产测试。使用相机所属构建环境执行
`cargo run --release --locked -p camera-node --example realsense-smoke --features realsense-runtime`；
会实际占用相机，须先停用该相机的采集，不能与服务争用设备。

## `camera-node`

源码：[节点与标定状态机](../backend/nodes/camera/src/main.rs)、[采集 worker](../backend/nodes/camera/src/capture_worker.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `capture_worker::spawn` | 命令/最新值通道 → 驱动长期任务；事件循环不执行 SDK 阻塞操作 |
| `refresh` / `select` | 发现/用户选择 → 来源和 profile；不按设备枚举索引存配置 |
| `start` / `stop_capture` / `reset` | 明确请求 → 同一 worker 的采集状态 |
| `tick` / `finish_capture_command` | worker 结果 → 状态、RGB-D 原子 bundle |
| `start_automatic_calibration` / `advance_automatic_calibration` | 型号姿态与实际反馈 → 采样、求解、回工作位 |
| `poll_calibration_work` / `apply_solved_calibration` | 当前会话结果 → 待确认状态 / 已落盘外参 |

节点统一管理硬件与 simulation 适配器、持久配置、采集和标定。Dora 循环只处理请求、状态和
发布；Tokio `spawn_blocking` 长期任务拥有驱动、pipeline 与非 `Send` 的 Align，通过有界命令通道
和 latest-value 通道通信。设备 60 FPS、上层 1 FPS 时仍排空 60 FPS 并提供彩色视频，但约每秒只 Align 并
发布一次 RGB-D。命令通道关闭时退出任务；停用或重开 pipeline 会清空旧帧，旧的打开结果不能覆盖新请求。

- `refresh()` 由启动发现和网页刷新共用，发现设备，并把已保存但暂时离线的来源/profile 标成不可用。
- `select()` 校验驱动刚报告的 profile、上送 FPS 与厂商扩展参数，再按稳定来源身份保存。
- `start()`、`stop_capture()` 与 `reset()` 都在同一采集任务内操作驱动，不维护第二套硬件状态。
- `tick()` 读取最新采集结果、绑定该帧的外参快照并发布专用 Arrow Binary bundle。
- 标定状态机消费机械臂反馈和型号声明的姿态，进程内调用 `camera-calibration` 对 OpenCV 5 的
  `CharucoDetector`、`solvePnP` 与 `calibrateRobotWorldHandEye` 包装。不存在标定子进程或
  JSON/Base64 IPC。

`capture_calibration_observation()` 和 `solve_calibration()` 在现有 Tokio 的阻塞任务池运行；Dora 循环通过
`poll_calibration_work()` 接收结果。会话取消后旧任务结果不再应用，不新增进程或服务。
自动流程先等运动节点确认进入手动模式，再逐个提交姿态；成功后等 10 秒并等下一帧采样。
九组求解完成后仍处于自动流程：以独立请求返回型号声明的工作位，保持夹爪状态；
只有匹配的回位请求成功才进入等待确认应用，失败则明确显示失败，不能提前应用或复用开始时的回位结果。
会话进度只在内存中，只有确认应用的外参由 `apply_solved_calibration()` 先落盘再替换配置。
`camera_state.saved_calibrations` 独立发布已落盘的完整结果，不依赖当前会话、相机连接或新图像。
网页显示上次标定的时间、样本数、外参和拟合残差；已有结果时禁用开始按钮，使用确认按钮旁的
“重新标定”启动同一流程。重新标定或取消不会删除旧外参，只有确认应用才替换。
相机节点启动时通过现有异步采集 worker 自动发现来源（与网页刷新共用），不自动选择或打开设备。

标定采样先固定一张新图，等待主机接收该图之后采集的电机反馈及其 FK 结果，然后在同一任务中
识别该图并保存观测。`current_tool_pose.arm_state` 是该 TCP 的原始 FK 输入，含反馈来源、批次、
时间和关节角；相机不再单独订阅另一份 `arm_state` 来拼接样本。运动节点的 FK 输入不裁剪，
只有规划/控制器同步仍使用 `planning_state()`。观测同时保存图像主机接收时间、图像序号与
反馈序号，时间差可复查；相机硬件时间与主机时间不能直接相减。没有硬件同步触发，因此这只是
软件时序配对，不保证曝光和每个电机读数严格同时，也不以已到位 10 秒替代来源一致性。

相机同时消费执行节点的 `transport_state`：连接中断立即结束活动标定并保留原始错误、
已采样观测和旧外参，不再无限等待新反馈。已知串口未连接时，新标定请求在发运动前失败。
同一状态经网关发送到感知页，感知及执行页顶部显示断连原因和重新连接说明；
网页 WebSocket 正常不代表串口正常。恢复连接不自动续跑标定，也不恢复旧夹持调节命令。

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
检测沿用官方默认的 ArUco/ChArUco 角点设置，不额外调用 `cornerSubPix`；PnP 与手眼算法不变。
检测使用原始图像，不做高斯预模糊，不重新引入已撤销的二次角点精修。
需要直接处理 RGB 的新算法应明确使用 `COLOR_RGB2GRAY` 等对应参数；不要在全链路来回换色。
YOLOE 的官方 PIL loader 内部转 BGR，predictor 再转 RGB tensor，这是库内部契约，不应在调用前补一次转换。
ChArUco/PnP 接受 forward Brown/rational 或零畸变针孔输入，调用时同时传入模型。
非零 inverse/modified Brown、鱼眼投影不能直接当成相同系数，需要驱动提供校正彩色图及对应内参。
鱼眼即使系数全零也不是针孔投影，不能省略这个模型区别。
原始相机分辨率不等于模型张量输入尺寸：不再强制 `imgsz=max(image.size)`，由模型默认预处理
处理输入；`retina_masks=True` 保证输出掩码回到原图尺寸。相机图像、内参和标定保留各自实际分辨率，不跟随模型张量尺寸变化。
见 [Ultralytics Predict 参数](https://docs.ultralytics.com/modes/predict/#inference-arguments)。
GraspGenX 当前官方场景推理使用目标/环境 XYZ；所用 sampler 的颜色张量为零，不把展示用点云颜色当作模型输入。

回归测试包含红/绿/蓝/非对称颜色、非整通道行填充、五种原生格式、OpenCV PNG 往返、
官方 Ultralytics loader/predictor 的最终 RGB tensor 和浏览器 RGB888 显示。
依据：[OpenCV 5 图像编解码接口](https://github.com/opencv/opencv/blob/5.0.0/modules/imgcodecs/include/opencv2/imgcodecs.hpp)、
[ChArUco 检测实现](https://github.com/opencv/opencv/blob/5.0.0/modules/objdetect/src/aruco/charuco_detector.cpp)。

## `robot-arm-messages`

源码：[消息与 Arrow codec](../backend/crates/robot-arm-messages/src/lib.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `to_arrow` / `from_arrow` | 小消息 ↔ JSON Arrow |
| `camera_frame_from_arrow` | RGB-D bundle → 已校验的图像平面与同帧元数据 |
| `world_scene_to_arrow` | 场景 → 小元数据与二进制 XYZ；不把点云展开为网页 JSON |
| `scene_pick_place_to_arrow` | 请求 + 场景快照 → 单条绑定消息；没有第二个场景订阅竞态 |

小消息使用共享 JSON Arrow codec。`CameraFrameBundle` 用专用 codec，把元数据与两个 Arrow Binary
图像 buffer 分开。彩色和深度必须同尺寸、同 frame id，且共享内参尺寸一致。bundle 原子携带已
对齐 RGB-D、深度比例、两个时钟和本帧外参快照；未标定时快照为 `None`。

## `scene-node`

源码：[阶段编排](../backend/nodes/scene/src/main.rs)、[组合分割](../backend/nodes/scene/src/segmentation.rs)、[三维重建](../backend/crates/scene-core/src/lib.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `apply_request` / `start_scene_task` | 显式网页请求 → 一个异步阶段；不自动串联全部模型 |
| `next_observation_frame` / `feedback_after_frame` | 新帧与实际反馈 → 同次输入及末端自过滤依据 |
| `segment_frame` / `segment` | 冻结 RGB、模型/提示 → 二维实例；不定位、不抓取 |
| `edit_segmented` / `rebuild_segmented` | 模型层与手动层 → 原图、框信息、mask；不生成染色图 |
| `reconstruct_frame` | 冻结帧深度 + mask + 外参 → 场景与实例点云；不重跑分割 |
| `attach_grasp_candidates` | 指定实例点云 + 环境 + 实际夹爪 → 该实例候选 |
| `finish_scene_task` / `publish_scene` | 同序号阶段结果 → 状态、响应及场景；旧输入结果不覆盖新输入 |
| `clear_scene` / `clear_output` | 标注变化 / 来源配置变化 → 对应下游结果失效 |
| `snapshot_previews` / `send_asset` | 明确刷新 / 资源查询 → 预览 PNG / 已缓存二进制；不触发模型 |

节点缓存 `camera-node` 发布的最新原子帧，只在显式请求时启动对应阶段。`reqwest::Client`
异步调用 YOLOE/GraspGenX；深度解码、掩码融合、场景重建和预览编码在 Tokio `spawn_blocking`
中执行，因此 Dora 循环仍可响应状态。手动刷新静态预览的 PNG 编码仍在显式快照请求内执行。
公开 `task_state` 和 `task_action` 驱动网页按钮状态，同类任务不排队。
`refresh`（网页“运行分割”）**只调用分割模型**，返回类别、二维框、冻结原图及原始掩膜资源，
不要求标定或机械臂反馈、不解码深度、不构造点云、不调用 GraspGenX。
`reconstruct`（“三维定位”）消费 `input_sequence = last_segmentation_sequence`，
使用该分割保留的同一原子帧深度和外参生成场景，不调用任何 AI 模型。
`generate_grasps`（网页“启动”按需调用）消费 `input_sequence = last_scene_sequence` 和 `object_id`，
只为选中的一个实例调用 GraspGenX，不重新拍摄、分割或定位。生成后的场景有新序号。
新分割或配置失效会清除旧三维结果；上游序号不匹配明确报错，不偷偷重跑上游。
分割期间并发记录该帧之后的实际 TCP/夹爪反馈供后续自过滤；反馈缺失不会阻塞分割，
仅在请求抓取候选时报告缺失。掩膜以 `instances[].mask_asset_key` 通过现有资源接口读取。
任务保存启动时的场景序号；相机、标定、模型或提示词变化都会使旧结果失效，不能靠同一来源 ID 判断新旧。

`scene-core` 负责已对齐深度与实例掩码的领域融合、坐标变换及 `WorldScene` 组织。scene 不发现
相机、不保存内外参、不驱动机械臂，也不手写畸变/深度注册算法。反投影调用 OpenCV 5
`undistortPoints`，支持 Brown-Conrady / rational 和 Kannala-Brandt / equidistant；
后者即使系数为零也调用 fisheye 版本，非鱼眼的零畸变才按针孔反投影。
非零畸变的其他模型明确返回不支持，不再忽略系数产生错误点云。深度注册仍由采集层 SDK 完成。
提示词模式的文字与放置区域角色由用户配置，不写死测试类别。自动分割模式不使用这些
提示词配置；三维定位后实际识别出的实例可作为抓取或放置目标，由用户或 AI 选择。
模型 ID 持久保存在场景配置；切换模型不丢弃已保存的文字配置。
新帧不会自动触发模型，刷新静态预览也不会触发模型。
分割完成只替换分割资源，不删除独立生成的深度快照；切换相机或配置时才清空整套图像资源。

### 辅助标注与组合分割

感知页仅分“相机与标定”和“AI 与任务”两组：第一行左侧相机来源/配置/深度预览，右侧外参标定，
内参与当前外参在标定卡片中折叠查看。第二行统一“AI”大栏目内部双栏：左侧自然语言、抓放场景和
高级设置，右侧分割与统一结果图；窄屏上下排列。不再保留独立采集数据、AI 模型与结果或结构化三维场景栏目。
“自动分割”“提示词分割”“手动分割”是三个并列折叠区，折叠状态由浏览器保存。
两种模型始终可用，没有互斥启用开关。先“载入新分割帧”，任选一种或依次运行多种；所有结果使用
同一冻结 RGB-D 帧。运行同一模型仅替换该模型结果，手动框选命名后“应用手动标注”仅替换手动层。
手动应用成功即释放已提交草稿，回显采用服务数据，不以 JSON 字段顺序或浮点尾数完全相同作为保存确认；
失败保留草稿。三维定位仍要求已有帧、已应用标注和相机标定，按钮旁显示具体置灰原因。
每个区域可清除本来源结果。“分割结果”默认折叠，使用同帧原始彩色图展示全部来源，
SVG 仅提供像素对齐的框和可点击标题；不染色，不增加掩膜高亮层，也不在浏览器重做掩膜融合或编解码。
图下合并场景坐标系和物体/放置区/显式障碍数量，场景序号不对应当前结果时显示尚未定位，不展示旧场景统计。
实例位置、尺寸、候选及关联放置区合并到标题详情，不再重复展示三维实例表。
点击标题使用原生 dialog 展示来源、置信度、二维框、三维数据与移除操作；新分割序号或删除会关闭旧详情。
手动编辑图只显示自己的已保存标注/待应用草稿，模型结果只在统一结果图显示；原独立叠加图入口已合并。
折叠不移除结果，载入新帧则清空旧结果。
提示词与视觉示例配置在提示词折叠区；“高级设置”中的“AI 默认分割模型”只指定 AI 任务默认值，
不决定这三个分割入口是否可用。保存模型配置仍会使旧逐帧结果失效。
每条实例通过 `segmentation_source` 保留具体模型 ID 或 `manual`，网页显示模型名或“手动标注”。

沿用 `PerceptionRequest.action = refresh`，可选 `segmentation_edit`：

- `capture`：取得新帧，不调用模型，清空之前的分割和三维结果。
- `model`：仅对冻结帧运行请求指定的模型；替换同一模型的上一层结果，保留其他模型和手动层。
  此请求的提示词不覆盖已保存配置。
- `manual`：以 `regions: [{id, label, bounding_box_xyxy}]` 替换当前手动层，坐标为原图像素，
  与网页缩放无关；空数组清除手动层。
- `remove`：按 `instance_ids` 移除任意来源的结果。

除 `capture` 外均要求 `input_sequence = last_segmentation_sequence`，旧页面的编辑明确拒绝，
不会将标注套到另一帧。每次成功修改都会更新结果序号并清除旧三维结果与候选；失败保留原结果。
模型 ID 命名空间与 `manual:` 命名空间隔离，即使同名目标也不会覆盖其他来源。
不自动合并重叠掩膜或替用户决定哪个结果正确；重复对象可在列表手动移除。

手动框的全部像素生成矩形 PNG 掩膜，**不是模型推断的精细轮廓**，也不表示 100% 识别置信度。
框选须尽量贴近目标；框内背景和其他物体也会参与后续定位。三种来源统一交给现有 `scene-core`
定位，不新增手动三维/抓取链路。手动和自动模型识别出的对象都可作为下游放置对象。
统一结果下方“三维定位”只定位当前结果，不生成抓取候选或执行运动；定位仍需有效深度与相机外参。
AI 内的“抓放场景”单独折叠，仅包含抓取目标、放置区域和“启动”。启动时若选定物体没有候选，
先调用 `generate_grasps`，使用响应中的新场景序号切换感知控制并提交抓放；已有候选则直接复用。
候选生成或模式切换失败不提交运动，不重新拍摄、分割或定位；没有额外的候选生成按钮。
HTTP 接收确认只是 accepted；在收到同请求的运动反馈前持续显示等待并禁止重复启动，
之后随规划/执行状态更新，只在同请求终态结束等待。

`segmentation_frame` 描述冻结图的尺寸，`segmentation-color.png` 单独持有对应 RGB，刷新普通预览
不改变它。冻结帧同时保留当时深度、外参和实际工具反馈；仅载入帧时不等待模型或机械臂连接，
不会在稍后保存标注时读取另一时刻的机械臂姿态。新帧、相机/标定/模型配置改变或服务重启会清空
这些逐帧结果；已应用标注在普通浏览器刷新后仍从服务回显，未应用草稿不视为已保存。

`PerceptionRequest.prompt` 可显式指定 `{"kind":"text"}` 或
`{"kind":"visual","reference_image_base64":"…","bboxes":[[x1,y1,x2,y2]],"class_ids":[0]}`。
类别仍由 `classes` 定义，视觉例子按从 0 开始的 class ID 对应；参考图内容与例子持久保存在场景配置，
不引用临时文件。仅发布 `visual_prompt_active` 状态，不向网页状态流重复发送参考图。
只有这个带图像的 `/api/perception/request` 路由解除 nginx 1 MiB / Axum 2 MiB 的默认文本请求体限制；
其他控制接口保留原有行为。真实 1280×720 参考 PNG 的 base64 请求约 1.43 MB，已复现旧入口 HTTP 413。
修改文字类别默认清除视觉提示；请求中显式提供 `prompt` 可覆盖。单纯刷新保留当前提示方式。

实例深度采用 Otsu 相邻差分类与四连通区域，边阈值不能小于 Z16 的一个量化刻度。
真实 3 mm 亚克力片的连续表面主要是 0/1 刻度差，旧 Otsu=0 会分裂为等深窄条，
造成三维中心跳动。下限来自数据编码分辨率，不是毫米级噪声门限或物体尺寸，不改变深度值。

## `perception-compute`

源码：[模型边界](../backend/services/perception-compute/src/perception_compute/app.py)、[自过滤](../backend/services/perception-compute/src/perception_compute/gripper_self_filter.py)、[碰撞筛选](../backend/services/perception-compute/src/perception_compute/scene_collision.py)、[候选代表](../backend/services/perception-compute/src/perception_compute/grasp_selection.py)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `create_app` / `default_backends` / `default_grasp_backend` | lifespan → 常驻模型；请求不重复初始化 |
| `segment` / `_segment` | PIL RGB、模型与提示 → 官方分割结果；实例锁隔离提示更新 |
| `GraspGenXBackend.infer` | 目标/环境 XYZ、夹爪资产与观测 → 原始评分候选 |
| `GripperSelfFilter.filter` | 实际 TCP/关节、环境点 → 去掉已观测夹爪自体点 |
| `filter_colliding_grasps` | 官方取样几何与场景 → 精确最近距离碰撞筛选 |
| `representative_grasps` | 筛选后的姿态、模型分与 CAD 角点 → 最多 100 个原始代表 |
| `build-description.py` | 正式型号 URDF/清单 → 构建期夹爪资产；脚本由计算服务维护，不依赖 tools |

`GRASPGENX_NUM_GRASPS` 透传官方 `num_grasps`，默认 200；Compose 从 `.env` 读取。
它控制同一模型的候选探索数量，不修改分数门限、候选位姿、关节范围或碰撞规则。
本机已验收配置为 500，保存在持久 `.env`；不改变源码的官方默认值。性能优化已按用户要求收敛。
该参数控制 GraspMoE 的 diffusion 分支采样，不等于最终返回数量。正式模型调用使用
`grasp_threshold=-1.0` 和 `topk_num_grasps=-1` 保留评分候选，不再按示例 0.7 分截断；
分数继续用于完整方案排名。海绵实物重复验证见 [验收边界](REVIEW.md)，不等于保证低分候选能夹住物体。
不能把采样数当作有效候选数或成功证明。环境筛选保留官方场景流程的取样与距离判据，
使用 SciPy `cKDTree.query(eps=0)` 计算最近距离，避免 CPU 上构造全距离矩阵。

环境筛选后，模型服务最多返回 **100 个原始候选**（用户指定上限，不是原始采样数）。
`grasp_selection.py` 用已依赖的 SciPy average-linkage 聚类合并邻近位姿，每组取模型分最高的
原始候选。距离是实际夹爪 CAD 包围盒八角点经位姿变换后的位移，统一计量平移与旋转，
不添加任意角度门限或角度/距离权重。保留原始位姿、分数、来源；不足 100 个不补造候选。
网页、手动与 AI 路径、MoveIt 共用该结果，没有 MoveIt 专属的另一套截断。
这不保证 100 个都有 IK 解；完整路径与碰撞仍由 MoveIt 验证，既有夹取深度优化保持不变。

FastAPI lifespan 加载提示词 `yoloe-26x-seg.pt`、自动分割 `yoloe-26x-seg-pf.pt` 和
共用的 `GraspGenXBackend`。`/v1/model` 返回模型目录和 `prompt_free` 能力；网页据此将模型分别放入
自动与提示词两个独立折叠区，只有提示词区显示文字/视觉提示配置。`/v1/segment` 按请求 `model` 选择模型，
解码同一 RGB 图并返回模型 ID、类别、置信度、二维框和 PNG mask；`/v1/grasps` 接收一个
实例点云、排除该实例的环境点云和 `gripper_asset_id`，返回该资产 TCP 的 SE(3) 候选、分数与分支。
YOLOE 的提示词更新和推理共用一把实例锁，避免并发请求混用类别；GraspGenX 也串行访问共享 sampler。
`/v1/grasps` 同时接收 `observed_gripper`：实际 TCP 位姿、夹爪关节映射和反馈时间。
scene-node 从 motion_state 配对新图像之后的真实 FK/电机反馈；计算侧以现有夹爪资产和
FCL 从环境点中剔除末端自体点，目标点不变。显式 `null` 表示该输入没有观测机器人
（例如官方静态测试场景），不是生产故障时跳过自过滤的降级。设备 `self-filter.json`
与 ROS 共用原有网格自过滤距离，不添加新的抓取碰撞门限。
文字与视觉提示使用同一 YOLOE 检查点、同一 `/v1/segment` 返回契约。
视觉提示直接调用官方 `refer_image`/`YOLOEVPSegPredictor`，框属于保存的参考图，
新帧掩膜由模型生成，官方 object0/object1 输出按 class ID 映射调用方名称。
视觉输入尺寸取原图/参考图最长边，文字模式保留库默认；两种均不覆盖默认置信度。
没有用示例框代替分割掩膜，也没有新增模拟/真机分支或外部算法服务。

自动分割使用官方 prompt-free 权重的内置词表和 `result.names`，不调用 `set_classes`，
不传入文字或视觉提示，也不覆盖默认阈值/输入尺寸。它不是任意物体都能识别的保证；
也不是普通 YOLO26 的固定 COCO 80 类模型。两种模型共用后续深度融合、抓取与运动链路。
参见 [YOLOE 官方用法](https://docs.ultralytics.com/models/yoloe/)。

CPU/CUDA 只改变运行设备，不改变接口。服务不连接相机、Dora、ROS 或 MoveIt，不读取类别名称
推断抓放规则。夹爪资产在计算基础镜像中从设备清单与最终补丁 URDF 自动生成，`gripper_asset_id` 决定使用哪套
资产。

## `stararm-102-motion-node`

源码：[队列与状态](../backend/devices/stararm-102/nodes/motion/src/main.rs)、[ROS 桥](../backend/devices/stararm-102/nodes/motion/src/ros.rs)、[MTC 工厂与执行](../backend/devices/stararm-102/ros2/mtc/src/pick_place_server.cpp)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `planning_state` | 实际反馈 → 控制器同步用状态；不修改用于标定 FK 的原始反馈 |
| `RosInterface::request_current_pose` | 带来源/时间的实际反馈 → 官方 FK 与同一输入快照 |
| `publish_pose` | 相对控制目标 → 带当前 ROS 时间戳的 Servo 消息 |
| `run_motion` / `run_manipulation` / `work_loop` | 已接收任务 → 唯一顺序 ROS worker |
| `plan_and_execute` | 普通关节/TCP 目标 → MoveGroup 规划 → 原生控制器执行 |
| `create_task` / `apply_scene` | 绑定场景 → 本次 MTC 阶段与官方 Octomap 快照 |
| `allowed_grasp_depth_poses` / `rank_complete_grasps` | 候选 → 指尖过滤、深度变体、完整解排名；不执行失败 IK |
| MTC `execute` / `cleanup_scene` | 首条完整解 → 同场景单候选精修 → 一次执行 → 清理 |

离散的 manual、calibration、准备相对控制及 perception 请求进入一个顺序 `WorkItem` FIFO；唯一 ROS worker
依次暂停 Servo、规划/执行并恢复 Servo。连续 relative 输入不进 FIFO，只有相对模式且队列空闲时才发送 Servo 位姿和输入夹爪动作。
普通运动由 MoveGroup 规划，并把未改写的关节轨迹交给现有 ros2_control 的 `FollowJointTrajectory` action；抓放使用型号 MTC
组件。抓放请求先经过持有当前场景的 scene 节点，由其把请求与 `WorldScene`（含二进制点云）
封装为同一条 `ScenePickPlaceRequest` 消息交给 motion；motion 不再订阅另一份独立场景缓存。
这样候选生成完成后的请求不会超前于大体积点云消息，真正过期的场景序号仍拒绝并显示请求/当前序号。
motion 同时接收执行节点的连接状态：选定真机端点但断连时不受理/启动抓放；任务期间已收到的
执行断连错误保留到终态，不被随后 MTC 成功或重连覆盖。MTC 成功本身仍不证明实物夹住。
motion 从绑定快照提取目标包围体、放置中心、抓取候选及同帧点云，使用原生
`PickPlace` action 传递给 MTC。目标成为可附着对象，完整点云进入官方
`PointCloudOctomapUpdater`；不启动 ROS 相机驱动，不把非目标实例包围盒当作实心障碍。
一次任务只建立一次环境快照，结束后清理。更新确认、世界对象过滤和占用地图由官方插件处理，
运输体积由原有 MTC attach/detach 管理。

模型、IK 插件与三类 PipelinePlanner 在服务启动时初始化并复用；每次任务只创建阶段、
冻结起始状态及场景，粗搜索和深度精修共用这些资源。执行和场景清理使用同一串行锁。
地图输入按型号全部碰撞体的保守球形可接触包络裁剪，包含持物相对 TCP 的最大延伸与体素
对角线，不改变关节范围，不按当前姿态猜可达空间；持物长度不与已含夹指的半径重复相加。
已有网格表面距离过滤用 TBB 并行处理互不重叠的点，判据及原始点不变；官方射线/概率树仍保留。
服务日志分别报告场景准备、地图更新/同步、完整方案搜索和所选候选精修，不能只用 OMPL
路径求解时间代表整个 MoveIt 阶段。详细对照入口见 `tools/diagnostics/map-benchmark`。

抓放使用用户选择的同次分割、三维定位和候选快照，只提交一次 MTC Action。
不执行观察位前缀，不在任务中重新调用感知或重新绑定实例。唯一完整任务保留候选排名、
所选候选的有限深度/平面起点精修、环境碰撞、持物回工作位与释放返回。
精修仍使用同一冻结起始状态和地图，不是第二轮观测后的独立抓放规划。
候选按原有成本排序后逐个送入 MTC；粗搜索取得首条完整方案即可进入选中候选精修，
不再等待二十条完整方案。场景建图仍使用官方插件，不把这部分耗时排除在 MoveIt 总时间之外。

`WorldScene` 的 JSON 仅含点云 frame、外参和尺寸；Arrow 的 `point_cloud_xyz` 二进制列保存光学
XYZ float32 LE，避免几百万浮点数展开为 JSON。网页只读元数据。motion 的 Rust 构建/测试须在
自己的 ROS 构建镜像内进行，使用同次构建生成的 `stararm_102_mtc` 类型，不再用 doc-only 消息替身。

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

源码：[执行与串口调度](../backend/devices/stararm-102/nodes/execution/src/main.rs)、[夹持反馈控制](../backend/devices/stararm-102/nodes/execution/src/gripper_feedback.rs)、[UART 协议库](../backend/crates/fashionstar-uart/src/lib.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `configure_endpoint` | 用户选择 → 保存并连接/断开；有端点但断连不回退软件反馈 |
| `encode_command` / `StarArmBus::write` | 完整目标 → 型号总线编码及唯一待发目标 |
| `flush_motion` | 最新目标 → 串口同步写；不能插入未完成回包事务 |
| `poll_hardware` | Monitor / 参数事务 → 实际角度、遥测和连接状态 |
| `GripperFeedbackController::request` / `observe` | 明确夹持意图 + 新负载反馈 → 持续功率调节；不锁存夹持角 |
| `primary_tool_feedback` | 设备遥测 → 统一负载百分比，不是牛顿力 |

`configure_endpoint()` 保存用户串口选择并显式连接/断开；未选串口时同一 `ArmCommand` 产生软件
反馈，选择串口但连接失败时不会回退。模块级 `encode_command()` 按模型映射总线指令，
`StarArmBus::write()` 编码并覆盖唯一的待发送完整目标；已有 tick 在没有回包事务和参数写入时，
由 `flush_motion()` 用同一同步协议发送编码发生变化的舵机。等待期间不积压历史轨迹点，
已接收目标和实际串口发送缓存分开；断线丢弃待发送目标，重新连接不重放旧动作。
同一舵机线上的 Monitor、寄存器和维护事务串行，异步只是不阻塞事件循环；
驱动库也禁止在等待回包时插入普通同步运动写入。回读频率仍由已有配置控制。
Monitor 读取失败先在同一串口重试一次，仍失败才进入重连逻辑。
读取/重连失败仍发布连接状态；断连后的最后硬件读数可以保留显示，但不再作为当前力度反馈发出。

`primary_tool_feedback()` 只在设备边界把实际功率映射为 0–100 通用反馈。稳定 0 是有效样本，
不代表“没有反馈”；软件模式不伪造真机力度。

`GripperFeedbackController::request()` 按原生 UART 位置编码识别闭合/释放意图，避免浮点噪声
误判张开。`poll_hardware()` 每次获取新 Monitor 后调用 `observe()`，按实际负载误差调整
原生功率上限，持续到显式张开、卸力或断开；不锁存保持角，不把目标百分比换成固定功率。
型号刻度、用户配置和真实效果见 [夹持反馈](STARARM-102.md#参数归属与当前值)，不在此重复维护数值。

## `web-gateway-node`

源码：[网关](../backend/nodes/web-gateway/src/main.rs)、[就绪状态聚合](../backend/nodes/service-status/src/main.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `validate_request` / `forward_value` | HTTP 请求 → 校验、request ID 配对与唯一节点输出 |
| `request_pick_place` | 抓放提交 → 场景所有者；HTTP 202 只表示已受理 |
| `AppState::update` / `snapshot` / `websocket` | 节点小状态 → 页面快照与实时流 |
| `perception_asset` / `model_asset` / `binary_response` | 按需资源 → 所属节点 → 二进制 HTTP |
| service-status `evaluate` | 服务报告及依赖表 → 就绪状态；不控制节点重启 |

Gateway 只保存最近一份小状态和按 request ID 配对的结果。相机及标定请求直接转给
`camera-node`，感知请求转给 `scene-node`；按需图像资源使用二进制 HTTP 响应。实时彩色视频由
`camera-node` 内置的 latest-value WebSocket 直接提供，反向代理只转发连接，不缓存帧。原始 RGB-D
frame 不进入 Gateway，Gateway 也不解析或保存任何服务配置。

HTTP 入站复用 `robot-arm-messages` 请求类型校验；非法载荷在网关返回 400，不送到节点使其退出。
相同尚未完成的 request ID 不重复转发；退出先释放所有 pending 响应，再等待 HTTP 结束。
感知快照同时通知采集所有者 camera 和冻结分割图/深度预览所有者 scene。模型配置请求不携带 camera 所有的来源选择字段。

## 公共配置存储

`json-config-store::save()` 先序列化完整 JSON，再使用 `atomic-write-file` 在同一目录写入并提交原子替换。
写入失败不截断原文件；各节点仍各自拥有配置，不增加配置服务、消息中转或前端副本。
参见 [AtomicWriteFile](https://docs.rs/atomic-write-file/latest/atomic_write_file/struct.AtomicWriteFile.html)。

## Next.js 抓放场景编排

源码：[AI 编排](../frontend/web/apps/perception/src/app/instruction-flow.ts)、[共享启动方法](../frontend/web/apps/perception/src/app/start-pick-place.ts)、[服务端接口](../frontend/web/apps/perception/src/app/api/instruction/route.ts)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| Route `POST` / `planner` | 自然语言 → 服务端 AI 调用；密钥不进入浏览器 |
| `executeInstruction` / `compactScene` | 提示配置、分割和定位 → 供 AI 选择的真实实例 ID |
| `startPickPlace` | 已选对象/区域 → 按需候选 → 感知模式 → 抓放提交；AI 与手动共用 |
| `startPickPlace` 的进度回调 | 阶段与 request ID → 网页状态 / AI 返回任务 ID |

`web-perception` 的服务端 Route Handler 先查询已保存的模型。提示词模式使用 AI SDK 将自然语言
转换为开放词汇提示词和放置角色；自动模式保持该模型、跳过提示词配置。
依次请求分割、三维定位后，再根据实际 `WorldScene` 选择对象与放置区域 ID；
选定后显式请求该对象的抓取候选。AI 不改变底层阶段边界。
任务解析为每个用户指代生成从具体描述到常见视觉类别的少量英文同义提示词，避免把单一语言
翻译误当成模型固定词表；用户明确指定的匹配词原样保留，不再扩写。这些词不包含场景硬编码。
选择后直接复用 `startPickPlace`：统一校验实例/区域、按需生成候选、使用候选响应的新场景序号提交。
不再在候选返回后另读一次可能滞后的场景快照，也不维护 AI 专属的模式切换/提交代码。
它不是 Dora 节点，不新增消息，也不复制 scene、MTC、碰撞或执行逻辑；浏览器只收到编排结果，
接触不到 API 密钥。

## 网页与配置归属

五个 Next.js 页面共用契约、UI 和 URDF 渲染器。实际机械臂跟随 `ArmState`；运动页半透明模型
来自尚未执行的编辑目标，执行页半透明模型来自最后命令，都不代表已通过规划。关节滑块只编辑，
点击“执行目标”才运动；默认位/工作位按钮仍点击即执行。“恢复当前姿态”不发送运动。
方向盘复用输入绑定、空间变换和 Servo；释放/失焦发送停止，不是硬件急停。
Servo 使用节点 ROS 时钟填写位姿消息时间戳，不能发送零时间戳。

网页未编辑字段跟随实时快照，草稿不被覆盖；保存失败保留草稿，保存确认后恢复跟随。
折叠状态仅由浏览器记忆。设备配置由各节点的 `json-config-store` 持久保存：相机 profile/外参属于
camera，模型和提示属于 scene，输入绑定属于 controller-input，串口/反馈周期/夹持目标属于 execution。
没有前端或中央配置副本。相机选择和运行中任务不持久化。

## 后续模块化边界（仅计划）

MTC 仍为 C++ 原生实现，Rust 负责消息和编排。可进一步把任务工厂、场景生命周期和排名拆成
直接可测模块，让测试不再通过包含整个服务 `.cpp` 并重命名 `main` 来访问实现。
重构应保持同输入完整规划结果、碰撞、力度和阶段顺序，先做软件回归再按授权真机验证；
不为换语言增加 FFI 或备用路径。这不是当前验收的未完成条件，本轮不实施该重构。
