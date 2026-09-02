# 后端方法与依赖

本文记录最终实现。系统总览见 [当前系统设计](SUMMARY.md)，型号参数与 TCP 约束见
[StarArm-102 型号适配](STARARM-102.md)。

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

`create_app()` 创建 FastAPI；lifespan 只加载一次 `YoloeBackend` 和 `GraspGenXBackend`。`/health` 和
`/v1/model` 报告实际模型、设备与许可；`/v1/segment` 解码 RGB，调用
`YOLOE-26s-seg`，返回类别、置信度、边界框和 PNG 实例掩码。

`/v1/grasps` 接收单个实例点云和 `gripper_asset_id`，调用 GraspGenX，返回该资产声明的工具
中心在输入点云坐标系中的候选 SE(3)、置信度与分支。`GraspGenXBackend.infer()` 用上游
`run_planner_on_object()` 和上游夹爪扫描体；推理锁只保护 PyTorch/采样器的共享模型状态，
固定种子使相同点云可重复，不过滤或手写候选姿态。

类别在请求中提供，修改类别时调用 Ultralytics `set_classes()`；CPU/CUDA 只改变
`PERCEPTION_DEVICE`，接口与后续链路不变。服务不读取深度、不连接 ROS/Dora/MoveIt，也不
编排抓放动作。

后端基础镜像从设备清单和已应用型号补丁的最终 URDF 自动生成全部夹爪资产；工程镜像直接
继承 `/grippers/<asset-id>`，不再次下载、生成或搬运。清单可声明多套资产，请求 ID 决定
sampler，计算服务不判断机械臂型号、厂家或夹爪关节名。

手写边界：HTTP DTO、Ultralytics 结果归一化、GraspGenX TCP 变换。库：FastAPI、Pydantic、
Ultralytics、GraspGenX、PyTorch、NumPy、Pillow、Uvicorn。许可分别遵循 Ultralytics 与
GraspGenX 仓库声明。

## `perception-node`

`PerceptionNode::apply_request()` 持久化启用状态、来源、模型类别及按 `source_id` 隔离的相机配置。每次应用
配置或停止时先清除旧 Marker，再由当前来源发布新场景，避免切换来源后遗留占据数据。
`apply_request()` 的 `Refresh` 分支只响应网页主动刷新；它从 ROS topic/type 图枚举成组的彩色
图、对齐深度和对齐后的 CameraInfo，并与 `simulation` 驱动声明的相机合并。`Reset` 只删除
当前来源的深度比例与标定保存项；模拟相机恢复编译期配置，真实相机回到无保存标定的状态。

### 相机和测试输入

感知容器内的首个设备适配器启动 ROS `realsense2_camera`；它只负责把 RealSense 系列相机
发布为标准 ROS 接口。`RosInterface::discover_cameras()` 不依赖相机型号，而是从 ROS 图发现
任意符合 RGB-D 组合契约的来源；`select_camera()` 切换动态订阅，来源切换后旧帧会按来源 ID
丢弃。真实场景只在彩色、对齐深度、内参和已应用外参同时存在时进入
`process_camera_scene()`。`simulation` 适配器也只生成这三种标准帧与预设外参，然后进入同一个
`handle_ros_event()` 和 `process_camera_scene()`：

- `simulation:pick-place-scene` 把固定 RGB 与确定性深度送入真实计算服务；
- `simulation:depth-grid` 在标准深度帧中提供 497 个有效像素。

`segment()` 和 `attach_grasp_candidates()` 是计算服务的两个能力调用，共用一个 HTTP 服务边界。抓取
资产 ID 来自通用 `RobotModelInfo.gripper_asset_id`，感知配置不保存机械臂型号或夹爪几何；
模型没有声明该能力时仍发布识别和结构化场景，只不请求抓取候选。
`decode_depth()` 接受统一 `16UC1`；`camera_calibration()` 把设备内参与当前来源持久化外参组合成
唯一标定事实。参数与标定只由后端 `json-config-store` 读写，网页没有配置副本。

### 三维场景

`perception-core::world_scene_and_instance_clouds_from_aligned_depth()` 解码实例掩码、按内参反投影、应用
`camera → base_link` 变换，并生成 `SceneObject`、`PlacementRegion` 与
`SceneObstacle`，同时保留每个实例在 `base_link` 中的点云供 GraspGenX 使用。
`aligned_obstacle_point_cloud()` 按实例二维范围排除已经作为结构化碰撞物体表达的区域，并按
共享 OctoMap 体素尺寸覆盖量化后仍会重复占据的边缘。放置区域的来源对象不在排除集合中，
因此容器由真实深度表面表达，而不是由实心外包围盒表达；代码不读取类别名称。

`publish_scene()` 只发布一份 Dora `WorldScene` 和解释性 Marker。`RosInterface` 发布
标准 Image、CameraInfo、PointCloud2、MarkerArray 和 TF。`PointCloud2` 经 MoveIt 官方
`occupancy_map_monitor/PointCloudOctomapUpdater` 进入唯一 PlanningScene；感知节点不手写
OctoMap、碰撞检测或第二套 ROS 转换。

### 标定

`update_calibration()` 提供 start/capture/solve/apply/cancel 的线性会话。
`capture_calibration_observation()` 同时记录 ChArUco 观测、真机关节反馈和 FK TCP 位姿。
Rust `perception-calibration` 工具通过 `opencv` crate 调用 OpenCV 5 的 ChArUco、PnP 和
`calibrateRobotWorldHandEye`。该工具负责图像解码、角点检测、位姿求解、标定和调试图；
`perception-node` 只负责会话、DTO、样本持久化和 ROS 发布。没有 Python/C++ 标定桥、自制
标定求解器或残差通过门限。

### 模拟与测试代码边界

生产几何只在 `perception-core`，其中不导出测试源 API。确定性 RGB-D、497 点深度帧和测试相机
参数集中在 `perception-node/src/simulation.rs`；输入动作回放集中在
`controller-input-node/src/simulation.rs`；编译进测试源的固定图片位于同节点 `test-assets/`。输入模拟
仍经过 spatial、motion 和 execution，RGB-D 测试输入仍经过 perception、motion 和 execution；
两者都不实现测试专用的下游业务流程。

## `stararm-102-motion-node`

`core::target_pose()` 把空间节点的设备无关增量组合成 StarArm-102 的 TCP 目标；旋转和
四元数使用 `nalgebra`。`tool_position_rad()` 在型号边界把通用 `primary_tool` 映射到模型声明的
夹爪行程，具体定义见 [StarArm-102 型号适配](STARARM-102.md)。

普通关节运动和抓放请求进入同一个 `WorkItem` FIFO。唯一 ROS worker 顺序执行“同步控制器、
暂停 Servo、规划/执行、恢复 Servo”；前一个动作未结束时后一个只等待，没有通用状态机框架、
并行规划器、固定队列上限或模拟/真机分支。

`ros.rs` 通过 `r2r` 使用标准 Servo、MoveGroup、ExecuteTrajectory、FK、状态有效性、
PlanningScene 和型号 MTC Action。普通关节请求仍由 MoveGroup 执行；抓放只把选中的结构化
几何映射到强类型 MTC goal，不含手写 IK 或 stage 推进。被抓对象和放置区域的来源对象按
契约 ID 排除，其他结构化对象及显式障碍按 ID 去重后映射为紧凑 AABB；代码不读取类别名称。
`stararm_102_mtc` 使用标准
`GeneratePose`、`GeneratePlacePose`、`ComputeIK`、`MoveRelative`、`MoveTo`、`Connect`
和 `ModifyPlanningScene`，整条方案规划成功后通过官方 `ExecuteTaskSolution` capability 执行。
障碍点云由同一 `move_group` 的官方 Occupancy Map Monitor 消费；抓放临时允许已附着对象与
支撑表面接触，离开支撑后恢复碰撞。厂家网格未替换；底座 visual/collision 最低点与 Z=0
刚性地面重合。Servo 订阅同一 `/monitored_planning_scene`，不维护独立场景来源。

ROS 绑定固定到官方 `r2r 0.9.6` 标签提交；该版本已声明支持 ROS 2 Lyrical，但尚未发布到
crates.io，因此 workspace 只在一处声明官方 Git 提交，perception 与 motion 共用同一依赖。

起点自碰撞恢复只在普通规划失败且 MoveIt 实测起点碰撞时，对同一次重试临时允许该 link 对；
不写全局 ACM，不保留解锁状态，也没有 MoveIt 源码补丁。型号坐标约束只在
[StarArm-102 型号适配](STARARM-102.md) 维护。

## `stararm-102-execution-node`

`StarArmExecution::load()` 加载串口选择与反馈周期；`configure_endpoint()` 先保存用户
选择再连接/断开。未选串口时同一 `ArmCommand` 产生软件反馈；选中串口但连接失败时冻结最后
状态，不隐式切换软件模式。

`StarArmBus::encode_command()` 把模型关节与夹爪驱动关节绝对角编码成 FashionStar 命令。
`read_sorted_monitors()` 按模型舵机顺序读取；`state_from_monitors()` 与
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
消息，不解释轴数、设备或感知语义。WebSocket 每次发送最新快照后等待客户端确认已消费，再从
`watch` channel 取得当时最新值；高频反馈不会在代理和浏览器之间累积旧快照，也没有固定刷新
频率或丢弃业务消息的数值门限。

共享网页客户端按浏览器动画帧合并实时快照。只有控制绑定页“采集频率”文字每秒重算一次；后端
采样、输入测试、位姿、关节反馈和感知不受这个显示节拍限制。

网页按职责拆为控制绑定、空间转换、场景感知、机械臂运动和机械臂执行五个 Next.js 服务。
感知页使用独立 `/api/perception/*` 与 `/ws/perception`，图片资源按需读取，不塞入实时状态；
运动页只保留相对、手动和感知三种模式及型号运动接口。

## ROS 与 RViz 接口

| 用途 | ROS 接口 |
| --- | --- |
| 真实相机输入 | `{所选来源}/color/*`、`{所选来源}/aligned_depth_to_color/*` |
| 标准感知输出 | `/perception/color/*`、`/perception/depth/*` |
| 分割/标定调试 | `/perception/debug/segmentation`、`/perception/debug/calibration` |
| 场景说明 | `/perception/debug/markers` |
| MoveIt 自身碰撞状态 | `/get_planning_scene` |
| 世界与坐标 | `/monitored_planning_scene`、`/tf`、`/tf_static` |

RViz2、Openbox、TigerVNC 和 noVNC 位于 motion 容器，通过 `192.168.100.10:6080` 暴露一个
浏览器入口。项目直接使用 Ubuntu 26.04 官方包，不维护远程桌面协议或第二套网页规划器。

## 方法级依赖结论

| 能力 | 采用库 | 保留手写内容 |
| --- | --- | --- |
| 消息与配置 | Dora、Arrow、Serde、json-config-store | 业务 DTO 和请求关联 |
| 设备输入 | SDL3、hidapi、fusion-ahrs、one_euro_filter | NOLO 报告适配、Action 绑定 |
| 几何 | nalgebra、image | 场景语义与型号工具几何 |
| 识别与抓取候选 | Ultralytics YOLOE、GraspGenX、PyTorch | HTTP DTO 与 TCP 结果归一化 |
| 标定 | `opencv` crate、OpenCV 5 | 会话、样本和持久化 |
| ROS/规划 | r2r、MoveIt、Servo、ros2_control | ROS JSON 边界与业务步骤 |
| 串口 | serialport、fashionstar-uart | 舵机 ID 与型号命令映射 |
| Web | Axum、Next.js 16、React、Three.js、Radix | 页面业务组合 |

项目没有自制 IK、碰撞检测、轨迹插值、标定数学、实例分割、VNC、状态机框架或数据库。
