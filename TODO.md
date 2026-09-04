# 相机与场景链路收敛 TODO

> 状态：适用事项已全部完成。实现、测试、方法级审查、迁移清理和整栈验收已经收敛；真实手眼
> 标定仍按既定安排等待 J4 恢复和用户提供九组姿态，它不是本次架构迁移的未完成分支。

## 最终边界

唯一生产链路：

`相机驱动适配器 → camera-node → 已对齐 RGB-D → scene-node → perception-compute → WorldScene → motion`

- `camera-node` 拥有相机发现、稳定身份、profile、驱动参数、采集、帧同步、深度对齐、配置、
  内外参标定和持久化。
- `scene-node` 缓存最新原子 RGB-D，只在用户请求时编排模型调用，发布预览、实例、抓取候选和
  结构化场景。
- `perception-compute` 只运行 YOLOE/GraspGenX；不接触相机、运动或网页状态。
- `motion` 只消费 `WorldScene` 和抓取候选；原始图像、深度和厂商字段不进入 ROS/MoveIt。
- simulation 是 `camera-node` 的普通驱动适配器，和真机使用同一帧、配置、标定及下游契约。

## 已完成

- [x] 节点、Cargo package、二进制、Dora id、配置和内部 crate 一次性改为最终名称；同步 Docker、
  Compose、dataflow、service-status、gateway、网页契约和测试。
- [x] 配置只使用 `/config/camera.json` 与 `/config/scene.json`；移除旧文件、旧入口、兼容读取、
  空壳 crate 和重复配置来源。
- [x] `CameraFrameBundle` 收窄为彩色图、对齐到彩色平面的深度图、共享内参、深度比例及该帧的
  标定快照；RGB-D 数据用 Arrow `BinaryArray` 传递。
- [x] 帧契约检查像素格式、尺寸、stride、载荷长度、共享 frame id、共享像素平面和深度比例。
- [x] RealSense 适配器使用 `realsense-rust`/librealsense `Align`；不再手写畸变、反投影、外参
  重投影或 Z-buffer。
- [x] 驱动在 Tokio 管理的长期 `spawn_blocking` worker 中独占同步 SDK；有界命令/完成 channel
  和 latest-value watch channel 防止积压，Dora 事件线程不轮询硬件。
- [x] 高频 frameset 持续排空；只有达到上送周期的帧执行 Align 与载荷物化。设备缺帧和主动略过
  分开统计，profile FPS 与上送 FPS 独立。
- [x] 相机配置、预置外参、标定会话、标定资产和持久化归入 `camera-node`；OpenCV 算法提取为
  进程内 `camera-calibration` Rust 库，删除子进程和 Base64/JSON stdin/stdout 链路。
- [x] 标定继续直接使用 OpenCV 5 `CharucoDetector`、`solvePnP` 和
  `calibrateRobotWorldHandEye`；板参数保持 `5×5 / 15 mm / 11 mm / DICT_4X4_50`。
- [x] `scene-node` 使用异步 `reqwest::Client`；PNG、掩码、场景融合等 CPU 工作进入
  `spawn_blocking`。同类模型任务只允许一个在执行，配置或相机改变时丢弃过期结果。
- [x] YOLOE/GraspGenX 只由手动感知请求触发；提示词和实例角色保持开放，不写死方块、筐或
  其他测试场景。
- [x] 删除下游深度对齐、全幅/障碍点云 ROS 消息与 MoveIt 感知依赖；保留的针孔投影只用于
  实例掩码内点云，在 `scene-core` 集中实现并有边界测试。
- [x] 网页相机操作、标定、任务状态和按钮锁定均由后端状态驱动；未选择相机是正常默认状态，
  保存配置不会持久化运行态选择。
- [x] `camera-node` 从同一 pipeline 的每个 frameset 分流最新彩色帧，在节点内提供 WebSocket；
  浏览器 Canvas 使用 `@thi.ng/pixel` 按驱动声明的像素格式显示。该流不经过 Dora/Gateway，不新增
  节点、第二次采集或深度数据副本，也不受感知 RGB-D 上送频率限制。
- [x] 抓放作为感知页内的可折叠应用栏目，而不是感知服务的全部能力；AI 自然语言入口突出展示，
  模型、手动感知、对象/区域选择和 MTC 详情默认折叠并记忆状态。
- [x] 自然语言编排放在 Next.js 服务端并使用 AI SDK；先生成通用开放词汇提示词，运行同一感知
  请求，再从实际 `WorldScene` 选择有效实例，最后调用同一 MTC 抓放接口。未新增 Rust 节点、消息、
  坐标生成或备用运动路径。
- [x] 每个自然语言实体生成从具体描述到常见视觉类别的少量英文同义提示词；放置目标的同义词
  共享同一角色。真实链路验证不会因单一翻译词未命中而提前进入运动。
- [x] FastiCode 地址、`gpt-5.6-sol` 和鉴权只由根目录未跟踪 `.env` 注入 `web-perception`；浏览器、
  镜像和 Git 均不包含密钥，仓库只保留 `.env.example`。
- [x] 后端基础镜像先源码安装 OpenCV 5，再安装 ROS；在任何 Rust 构建前设置 `OpenCV_DIR`、
  `LD_LIBRARY_PATH`、`PKG_CONFIG_PATH` 和 `OPENCV_DISABLE_PROBES=pkg_config`。
- [x] ROS 依赖收窄到实际使用的 MoveIt/Servo/MTC/RViz/控制器包，移除不再使用的感知包。

## 方法级裁决

- [x] 深度注册调用厂商 SDK `Align`，没有保留备用算法或第二条 raw-depth 路径。
- [x] ChArUco、PnP、手眼标定调用 OpenCV；矩阵变换调用 `nalgebra`；模型前后处理调用官方
  Ultralytics/GraspGenX 路径。
- [x] 保留 `scene-core` 的短小针孔公式：OpenCV 5 当前精简构建不含 contrib `rgbd`，为这一处
  标准投影加入 contrib 会增加基础镜像、FFI 和维护面；实现有尺寸、内参和坐标测试。
- [x] 模型输入和结果继续使用按需 PNG/JSON；实时彩色预览独立使用节点内 latest-value
  WebSocket。当前可用 profile 限定为统一帧契约支持的未压缩格式，页面使用像素库，不引入
  FFmpeg/GStreamer、录像协议或第二个服务；后续只有实测带宽/CPU 不满足时才迁移编码。
- [x] 删除未使用依赖、不可达采集结果分支、旧点云函数/测试、旧标定资产、空目录和迁移说明。

## 验收

- [x] Rust 全 workspace runtime-feature 测试 138 项通过；Clippy `-D warnings` 与
  `cargo machete` 通过。
- [x] OpenCV 编译期/运行期版本测试通过；最终 `camera-node` 的全部 OpenCV 动态库来自
  `/opt/opencv5/lib/*.so.500`。
- [x] Python 模型服务 2 项测试与 Ruff 通过；前端格式、ESLint、TypeScript、18 项单测和 5 个
  Next.js 16.3.4 生产构建通过；peer 依赖无冲突，工具链主版本与其插件支持范围一致。
- [x] Playwright 27 项通过、1 项按设计跳过；Compose 软件端到端链路通过。
- [x] `gpt-5.6-sol` 实际完成“中文指令 → 多提示词 → YOLOE/GraspGenX → 真实场景 ID 校验 →
  MTC 软件抓放”，最终状态 `succeeded / pick and place complete`；运动执行端确认未连接真机。
- [x] 两个 simulation 来源均走唯一相机链路；方块/置物筐完成模型和抓取候选测试，点云网格完成
  帧、标定和预览测试。
- [x] D415 `924322061032` 真机完成 profile、驱动参数、采集、SDK 对齐、节流和预览测试；最终
  实测 `61.97 FPS` 采集、`1.00 FPS` 上送，对齐后彩色/深度均为 `424×240` 和同一 frame id。
- [x] 整栈重启后相机保持默认未选择，D415 与 simulation 的保存配置仍可按稳定 id 恢复。
- [x] 完成格式、过时关键词、旧配置、重复路径、空壳、Docker 配置和 `git diff --check` 审查。

## 后续独立验收

- [ ] J4 恢复后，由用户提供九组最终姿态，完成 D415 的真实 ChArUco 手眼标定并确认应用结果。
