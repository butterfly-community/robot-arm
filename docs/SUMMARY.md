# 当前系统设计

## 唯一运行链路

```text
输入设备 / 模拟输入
  → controller-input-node
  → spatial-transform-node
  → stararm-102-motion-node
  → MoveIt / Servo / ros2_control
  → ArmCommand
  → stararm-102-execution-node
  → ArmState / ActionFeedback

ROS 深度相机 / 确定性 RGB-D 测试源
  → perception-node
  → perception-compute-service（RGB 实例分割、实例点云抓取姿态）
  → perception-node（深度几何、标定、抓取候选、WorldScene、ROS 感知话题）
  → MoveIt PointCloudOctomapUpdater / stararm-102-motion-node（环境占据、抓放目标与规划）
  → 同一 ArmCommand / execution 链路
```

模拟和真机、测试 RGB-D 和真实相机都只在各自节点的输入或驱动适配层不同。空间转换、感知
结构化、运动学、规划和执行均没有备用业务路径。

模拟和测试实现也与生产核心物理分开：输入回放在
`backend/nodes/controller-input/src/simulation.rs`，感知测试源和它编译期使用的固定验证图片在
`backend/nodes/perception/src/simulation.rs` 与同节点的 `test-assets/`。
`perception-core`、空间核心、运动学和执行节点不包含测试数据生成逻辑。

## 服务边界

| 服务 | 负责 | 不负责 |
| --- | --- | --- |
| `controller-input-node` | NOLO HID、SDL3、模拟输入，能力发现，输入/反馈绑定，IMU 融合 | 空间积分、相机、运动学、机械臂参数 |
| `spatial-transform-node` | 绝对位姿换基、Action 积分、设备无关 TCP 增量 | 设备驱动、IK、串口 |
| `perception-node` | ROS 相机接入、RGB-D 对齐消费、深度反投影、标定、场景结构化、ROS 点云/图像/Marker/TF | 模型推理、机械臂轨迹 |
| `perception-compute-service` | YOLOE-26s-seg 开放词汇实例分割、按请求资产 ID 生成 GraspGenX 点云抓取候选；CPU/CUDA 使用同一 HTTP 契约 | 设备型号、深度、标定、MoveIt、Dora |
| `stararm-102-motion-node` | StarArm-102 TCP 数学、MoveIt/Servo、MTC 抓放桥接 | 相机采集、设备输入、串口 |
| `stararm-102-execution-node` | 软件反馈与 FashionStar UART 的同一执行契约、模型资源、真机遥测 | IK、目标位姿解释 |
| `service-status-node` | 根据配置聚合节点主动状态与依赖 | 业务探活特例、恢复策略 |
| `web-gateway-node` | HTTP/WebSocket 与 Dora 消息转发 | 设备或机械臂语义 |

`perception-core`、`spatial-core` 是纯库，不是额外服务。计算服务可以
远程部署，但外部只和 `perception-node` 交互。

StarArm 的 visual 与 collision 均使用厂家模型。MoveIt 保留机械臂自身碰撞检查；感知节点将
对齐后的未结构化障碍点云发布给官方 Occupancy Map Monitor，抓放任务同时把选中物体和其他
结构化障碍的紧凑盒几何交给 MTC。已结构化任务物体从点云中剔除，不会在 PlanningScene 中
重复表达。

## 感知与抓放

感知容器内的相机适配层负责启动首个 RealSense ROS 驱动；感知核心只使用 ROS 主线已经发布的
彩色图、对齐到彩色的深度图和 CameraInfo。网页主动刷新时，节点从 ROS topic/type 图发现所有
满足以下组合契约的来源，连接多个已发布来源时可按来源 ID 选择：

相机驱动是 `perception-node` 的内部设备适配层，不另设相机采集服务；型号差异止于该适配层，
结构化感知、远程计算和运动节点只接收统一契约。

| 输入 | ROS topic |
| --- | --- |
| 彩色图 | `{source}/color/image_raw` |
| 对齐深度 / 对齐后的内参 | `{source}/aligned_depth_to_color/image_raw`、`{source}/aligned_depth_to_color/camera_info` |

`perception-node` 只在所选来源具备相机外参后把帧转换到 `base_link`。未启用感知时不发布
占位场景；每次应用感知配置时会清除上一来源的 Marker，再发布当前来源；计算
服务失败时保留原始错误，不切换模型或伪造结果。

仓库提供两个由 `simulation` 驱动声明的确定性相机。它们和 ROS 相机一样枚举为
`DepthCameraSourceInfo`，发布同一组 RGB、对齐深度、CameraInfo 和标定输入，后续不再分流：

- `simulation:pick-place-scene`：固定 RGB 与确定性深度经真实 YOLOE 分割，生成
  红色立方体、灰色置物筐和筐内放置区；实例点云继续送入真实 GraspGenX 生成抓取候选，
  用于完整抓放验收。
- `simulation:depth-grid`：标准 RGB-D 帧内的 497 个有效深度像素，用于 RViz 与 MoveIt
  OctoMap 输入验收。

感知服务按 `source_id` 在自己的 JSON 中保存深度比例和外参。网页不保存相机配置。重置只删除
当前来源的保存项：`simulation` 随即恢复仓库预设，真实来源回到未标定状态，其他相机不受影响。

抓放按 ID 选择 `SceneObject` 和 `PlacementRegion`，不读取类别名称。Rust motion 从放置区域的
`source_object_id` 和物体几何推导放置高度；被抓物体不在点云中重复表达，放置区域来源对象
保留为实际深度表面且不再加入实心 AABB，其余结构化对象与显式障碍按 ID 去重后进入任务场景。
Rust motion 只做场景映射、唯一 FIFO 和
Action 状态转发；型号 MTC 组件用标准 stage 一次构造并选择完整任务解，负责候选位姿、IK、
接近、工具动作、attach/detach、搬运、回撤和返回。执行仍经 MoveIt、ros2_control 和唯一
`ArmCommand`。网页与 RViz 显示任务状态、候选、失败 stage 和选中轨迹。

每个机械臂适配器提供自己的规划组、TCP、工具关节、命名位与 GraspGenX 资产清单；构建时从
最终 URDF 自动生成资产，运行请求只携带通用资产 ID。感知计算和抓放契约不读取型号参数。
当前适配见 [StarArm-102 型号适配](STARARM-102.md)。

## 标定

标定由 `perception-node` 管理，使用 OpenCV 的 ChArUco 检测、PnP 与
`calibrateRobotWorldHandEye`，由 Rust `opencv` crate 调用 OpenCV 5，不使用 Python/C++
标定桥，也不手写标定数学。每个样本保存同期真机关节反馈对应的 TCP 位姿与板在相机中的
观测。求解结果同时包含：

- 相机到 `base_link` 的外参；
- 标定板到专用测试爪的固定变换；
- 每个样本的平移和旋转残差。

应用结果后，真实相机帧、点云、`WorldScene` 与 TF 使用同一份外参。标定板变换只参与标定，
不得作为正常抓放的 TCP 补偿。

## 机械臂适配

机械臂独有的 URDF 补丁、TCP、关节方向、命名位、工具映射和规划参数都归属型号 model、
motion、MTC 与 execution 边界，不进入输入、空间、感知或网页共享契约。当前实现集中记录于
[StarArm-102 型号适配](STARARM-102.md)。

## 输入与空间语义

输入节点按设备运行时声明的能力发布组件，不按型号猜测。NOLO CV1 和 SDL3 IMU 共用
`fusion-ahrs`；连续轴使用 `one_euro_filter`。位置来源、姿态来源、每个 Action 输入和每个
反馈目标都可独立选择设备。模拟输入也先声明同一套 Action，再经过空间、motion 和 execution。

空间节点在每次接管时建立新原点。无绝对位置/姿态来源的分量可由按钮或轴积分；选定绝对来源
的分量不再叠加对应 Action。满输入平移默认 1 cm/s，角向动作默认 0.10 rad/s；这些是用户已
指定的动作比例，不是保护门限。

## 启动和入口

```bash
docker compose up -d
docker compose down
```

- Web：`http://192.168.100.10:8765`
- RViz/noVNC：`http://192.168.100.10:6080`

Compose 使用私有 bridge network，只映射这两个入口。配置持久化于
`backend/config/runtime/*.json`。串口只在执行页点击“刷新串口”时枚举。

## 常用验收

```bash
cargo test --manifest-path backend/Cargo.toml --workspace
pnpm --dir frontend format:check
pnpm --dir frontend lint
pnpm --dir frontend typecheck
pnpm --dir frontend test
pnpm --dir frontend build
docker compose config --quiet
docker compose up -d
node tests/integration/software-flow.mjs
```

诊断和记录工具位于 `tools/`；运行缓存和中间产物不提交。
