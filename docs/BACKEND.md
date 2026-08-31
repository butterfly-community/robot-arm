# 后端方法与依赖

本文记录最终实现，不记录迁移过程。系统总览见 [当前系统设计](SUMMARY.md)，TCP 维护约束见
[StarArm-102 末端坐标](TCP.md)。

## `controller-input-node`

`ControllerInput::load()` 使用公共 `json-config-store` 读取 Action 绑定、反馈绑定与设备名称；
`commit_config()` 写盘成功后才替换运行配置。`drain()` 合并驱动事件，`tick()` 只把新的
设备样本转成统一绝对位姿和 Action。`combined_pose_frame()` 允许空间和姿态来自不同设备；
`evaluate_actions()`、`binding_value()` 把按钮、连续轴或正负按钮对转为设备无关 Action。

生产适配层只有 NOLO CV1 HID 与 SDL3：

- NOLO 加解密和报告解析位于 `nolo-cv1` crate；协议中已确认的按键、触摸板、扳机和位置
  映射到通用组件名。
- SDL3 使用运行时 `has_axis()`、`has_button()`、sensor 与 haptic 能力，只发布硬件真实
  声明的组件。
- 两类 IMU 都进入 `fusion-ahrs`；NOLO 位置和通用连续轴使用 `one_euro_filter`。
- 模拟源只声明同一业务 Action，经同一绑定、空间转换、MoveIt 和 execution 链路。

`live_component_values` 只供输入测试显示，不参与第二次 Action 计算。反馈由
`apply_feedback()` 按用户选择的目标路由；网页虚拟反馈和物理 haptic 使用同一
`ActionFeedback`。本节点不含相机、机械臂参数或运动学。

手写边界：NOLO 协议适配、组件命名、绑定求值、Dora I/O。库：
`hidapi`、SDL3、`fusion-ahrs`、`one_euro_filter`、`nalgebra`、`serde`、
`json-config-store`。

## `spatial-transform-node`

节点外壳只负责配置和 Dora I/O，数学集中在 `spatial-core::SpatialTransform`。
`update_pose()` 接收组合绝对位姿；`handle_control()` 接收 Action。`current_output()` 以
本次接管的设备位置和姿态为原点，按配置完成换基、相对位姿与无绝对来源分量的速度积分。

TCP 三轴平移、三轴定点旋转、两种枢轴圆弧、工具轴向平移和螺旋都有独立语义。夹爪和控制
Action 只透明传递，不在此解释型号。矩阵与四元数使用 `nalgebra`；可清除 JSON 字段使用
`serde_with`，没有自写解析器。

## `perception-compute-service`

`create_app()` 创建 FastAPI；lifespan 只加载一次 `YoloeBackend`。`/health` 和
`/v1/model` 报告实际模型、设备与许可；`/v1/segment` 解码 RGB，调用
`YOLOE-26s-seg`，返回类别、置信度、边界框和 PNG 实例掩码。

类别在请求中提供，修改类别时调用 Ultralytics `set_classes()`；CPU/CUDA 只改变
`PERCEPTION_DEVICE`，接口与后续链路不变。服务不读取深度、不连接 ROS/Dora/MoveIt，也不
推断抓取动作。

手写边界：HTTP DTO 和 Ultralytics 结果归一化。库：FastAPI、Pydantic、Ultralytics、
PyTorch、NumPy、Pillow、Uvicorn。许可为 Ultralytics AGPL-3.0 或企业许可。

## `perception-node`

`PerceptionNode::apply_request()` 持久化启用状态、来源、模型类别和计算服务地址。每次应用
配置或停止时先清除旧 Marker，再由当前来源发布新场景，避免切换来源后
遗留占据数据。`tick()` 处理 ROS 帧或只发布一次确定性场景。

### 相机和测试输入

`RosInterface::start()` 订阅 ROS 主线的彩色图、彩色 CameraInfo、对齐深度图和对应
CameraInfo。真实场景只在彩色、对齐深度、内参和已应用外参同时存在时进入
`process_camera_scene()`。测试来源复用后续全部处理：

- `generated:pick-place-scene` 把固定 RGB 送入真实计算服务，用返回掩码填充确定性深度；
- `generated:depth-grid` 生成 497 点测试云。

`segment()` 是计算服务的唯一客户端。`decode_depth()` 接受 ROS `16UC1`；
`camera_calibration()` 把设备内参与持久化外参组合成唯一标定事实。

### 三维场景

`perception-core::world_scene_from_aligned_depth()` 解码实例掩码、按内参反投影、应用
`camera → base_link` 变换，并生成 `SceneObject`、`PlacementRegion` 与
`SceneObstacle`。`aligned_obstacle_point_cloud()` 排除已经结构化的实例，使排障点云只
表达未结构化的背景深度。

`publish_scene()` 只发布一份 Dora `WorldScene` 和解释性 Marker。`RosInterface` 发布
标准 Image、CameraInfo、PointCloud2、MarkerArray 和 TF。深度点云用于感知与 RViz
排障，不进入 MoveIt 规划场景。

### 标定

`update_calibration()` 提供 start/capture/solve/apply/cancel 的线性会话。
`capture_calibration_observation()` 同时记录 ChArUco 观测、真机关节反馈和 FK TCP 位姿。
Rust `perception-calibration` 工具通过 `opencv` crate 调用 OpenCV 5 的 ChArUco、PnP 和
`calibrateRobotWorldHandEye`。该工具负责图像解码、角点检测、位姿求解、标定和调试图；
`perception-node` 只负责会话、DTO、样本持久化和 ROS 发布。没有 Python/C++ 标定桥、自制
标定求解器或残差通过门限。

### 模拟与测试代码边界

生产几何只在 `perception-core`，其中不导出测试源 API。确定性 RGB-D、497 点云和测试相机
参数集中在 `perception-node/src/test_source.rs`；输入动作回放集中在
`controller-input-node/src/simulation.rs`；离线资产生成和计算服务探针集中在
`tools/perception/`。这些代码只生成统一输入契约，生成后的消息仍进入同一感知、空间、MoveIt
和 execution 链路，不实现测试专用业务流程。

## `manipulation-core`

`plan_pick_place()` 只按 ID 从 `WorldScene` 取可抓物和放置区，生成以打开夹爪开始的八步线性计划；
`cartesian_target()` 只根据物体与放置区几何计算接近/到达位置：抓取点在物体顶面，放置点在
放置区顶面上方半个物体高度。库不含 StarArm 轴数、
关节角、串口或 MoveIt 类型，也没有状态机框架。

## `stararm-102-motion-node`

`core::target_pose()` 把空间节点的设备无关增量组合成 StarArm-102 的 TCP 目标；旋转和
四元数使用 `nalgebra`。`tool_position_rad()` 在此型号边界把 `primary_tool` 映射到
90°→0° 夹爪行程。

普通请求进入一个 FIFO，唯一 action worker 顺序执行“同步控制器、暂停 Servo、规划、执行、
恢复 Servo”。前一个动作未结束时后一个只等待；没有状态机框架、并行规划器、固定队列上限
或模拟/真机分支。

`ros.rs` 通过 `r2r` 使用标准 Servo、MoveGroup、ExecuteTrajectory、FK、状态有效性和
PlanningScene 接口。型号节点向 MoveIt 提交 TCP 位置目标，不预先求 IK，也不附加末端姿态；
MoveIt 在同一次规划中选择 IK 解与轨迹。`WorldScene` 只用于选择抓取目标、放置区和计算 TCP
目标，不发布成 MoveIt CollisionObject、AttachedCollisionObject 或 OctoMap。规划场景中唯一
的世界碰撞体是上表面位于 `base_link` Z=0 的刚性地面，普通规划与 Servo 均检查机械臂自身
碰撞和地面碰撞。厂家网格未替换；只把底座 visual/collision 的模型原点上移 3.5 mm，使网格
最低点与 Z=0 重合。Servo 作为从属 PlanningSceneMonitor 订阅同一个
`/monitored_planning_scene`，不维护独立场景来源。

### 已弃用：构建期单凸包碰撞网格

曾为定位感知场景启用后规划耗时从亚秒级上升到 10 秒以上的问题，在 motion 镜像构建阶段用
OpenSCAD 对厂家每个 STL 执行一次整体 `hull()`，再将结果替换为 URDF collision 网格。该方案
证明了高面数机械臂网格与环境对象反复碰撞查询是主要性能来源，但单一凸包会填平凹槽和夹爪
开口，不能作为可信的最终碰撞模型；构建时生成还会隐藏模型差异并增加工具依赖。因此相关
Docker stage、OpenSCAD 脚本、URDF 补丁和生成目录均已删除，当前没有启用该方案。

以后若重新启用环境碰撞，应重新制作并提交可审查的正式低面数碰撞资产，保留必要凹形结构，
在 RViz 叠加核对 visual/collision 后再做规划和抓放回归；不得直接恢复构建期整体凸包。

起点自碰撞恢复只在普通规划失败且 MoveIt 实测起点碰撞时，对同一次重试临时允许该 link 对；
不写全局 ACM，不保留解锁状态，也没有 MoveIt 源码补丁。所有 FK、IK 和业务目标均使用
`tcp_link`，不得添加 `link6` 补偿。

## `stararm-102-execution-node`

`StarArmExecution::load()` 加载串口选择与反馈周期；`configure_endpoint()` 先保存用户
选择再连接/断开。未选串口时同一 `ArmCommand` 产生软件反馈；选中串口但连接失败时冻结最后
状态，不隐式切换软件模式。

`StarArmBus::encode_command()` 把 J1–J6 与夹爪绝对角编码成 FashionStar 命令。
`read_sorted_monitors()` 读取 ID 0–6；`state_from_monitors()` 与
`telemetry_from_monitors()` 产生位置及遥测。Monitor 失败在同一串口重试一次，仍失败才
重连。串口只在网页主动 discover 时枚举。

`primary_tool_feedback()` 在设备边界把夹爪 Monitor 功率映射成 0–100 通用反馈。稳定 0 是
有效样本，不代表没有反馈；软件模式不伪造真机力度。厂家帧由 `fashionstar-uart` crate，
串口由 `serialport` 库处理。

`stararm-102-model` 从最终 URDF 生成模型信息和资源 manifest；execution 是唯一模型资源
发布者。motion 和 Web 不维护第二份机械臂参数。

## 网关、状态与网页

`service-status-node` 从 JSON 读取服务和依赖，只聚合主动 `ServiceState`。没有业务节点
硬编码、超时门限或恢复分支。`web-gateway-node` 使用 Axum 转发 HTTP/WebSocket 与 Dora
消息，不解释轴数、设备或感知语义。

共享网页客户端按浏览器动画帧合并实时快照。只有采集页“采集频率”文字每秒重算一次；后端
采样、输入测试、位姿、关节反馈和感知不受这个显示节拍限制。

## ROS 与 RViz 接口

| 用途 | ROS 接口 |
| --- | --- |
| 真实相机输入 | `/camera/camera/color/*`、`/camera/camera/aligned_depth_to_color/*` |
| 标准感知输出 | `/perception/color/*`、`/perception/depth/*` |
| 分割/标定调试 | `/perception/debug/segmentation`、`/perception/debug/calibration` |
| 场景说明 | `/perception/debug/markers` |
| MoveIt 自身碰撞状态 | `/get_planning_scene` |
| 世界与坐标 | `/monitored_planning_scene`、`/tf`、`/tf_static` |

RViz2、Openbox 和 KasmVNC 位于 motion 容器，通过 `192.168.100.10:6080` 暴露一个浏览器
入口。项目使用现成的 KasmVNC 显示与输入协议，不维护第二套远程桌面或网页规划器。

## 方法级依赖结论

| 能力 | 采用库 | 保留手写内容 |
| --- | --- | --- |
| 消息与配置 | Dora、Arrow、Serde、json-config-store | 业务 DTO 和请求关联 |
| 设备输入 | SDL3、hidapi、fusion-ahrs、one_euro_filter | NOLO 报告适配、Action 绑定 |
| 几何 | nalgebra、image | 场景语义与型号工具几何 |
| 识别分割 | Ultralytics YOLOE、PyTorch | HTTP DTO 归一化 |
| 标定 | `opencv` crate、OpenCV 5 | 会话、样本和持久化 |
| ROS/规划 | r2r、MoveIt、Servo、ros2_control | ROS JSON 边界与业务步骤 |
| 串口 | serialport、fashionstar-uart | 舵机 ID 与型号命令映射 |
| Web | Axum、Next.js 16、React、Three.js、Radix | 页面业务组合 |

项目没有自制 IK、碰撞检测、轨迹插值、标定数学、实例分割、VNC、状态机框架或数据库。
