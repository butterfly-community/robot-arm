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
  → stararm-102-motion-node（抓放目标与 MoveIt 规划）
  → 同一 ArmCommand / execution 链路
```

模拟和真机、测试 RGB-D 和真实相机都只在各自节点的输入或驱动适配层不同。空间转换、感知
结构化、运动学、规划和执行均没有备用业务路径。

模拟和测试实现也与生产核心物理分开：输入回放在
`backend/nodes/controller-input/src/simulation.rs`，感知测试源在
`backend/nodes/perception/src/test_source.rs`，离线生成器与探针在 `tools/perception/`。
`perception-core`、空间核心、运动学和执行节点不包含测试数据生成逻辑。

## 服务边界

| 服务 | 负责 | 不负责 |
| --- | --- | --- |
| `controller-input-node` | NOLO HID、SDL3、模拟输入，能力发现，输入/反馈绑定，IMU 融合 | 空间积分、相机、运动学、机械臂参数 |
| `spatial-transform-node` | 绝对位姿换基、Action 积分、设备无关 TCP 增量 | 设备驱动、IK、串口 |
| `perception-node` | ROS 相机接入、RGB-D 对齐消费、深度反投影、标定、场景结构化、ROS 点云/图像/Marker/TF | 模型推理、机械臂轨迹 |
| `perception-compute-service` | YOLOE-26s-seg 开放词汇实例分割、GraspGenX 点云抓取候选；CPU/CUDA 使用同一 HTTP 契约 | 深度、标定、MoveIt、Dora |
| `stararm-102-motion-node` | StarArm-102 TCP 数学、MoveIt/Servo、MTC 抓放桥接 | 相机采集、设备输入、串口 |
| `stararm-102-execution-node` | 软件反馈与 FashionStar UART 的同一执行契约、模型资源、真机遥测 | IK、目标位姿解释 |
| `service-status-node` | 根据配置聚合节点主动状态与依赖 | 业务探活特例、恢复策略 |
| `web-gateway-node` | HTTP/WebSocket 与 Dora 消息转发 | 设备或机械臂语义 |

`perception-core`、`spatial-core` 是纯库，不是额外服务。计算服务可以
远程部署，但外部只和 `perception-node` 交互。

StarArm 的 visual 与 collision 均使用厂家模型。MoveIt 保留机械臂自身碰撞检查；感知点云、
原始点云和分割掩码不写入 PlanningScene；抓放只把选中物体和结构化障碍的紧凑盒几何交给
MTC 的任务场景。

## 感知与抓放

真实相机使用 ROS 主线已经发布的彩色图、对齐到彩色的深度图和 CameraInfo。当前订阅接口为：

| 输入 | ROS topic |
| --- | --- |
| 彩色图 / 内参 | `/camera/camera/color/image_raw`、`/camera/camera/color/camera_info` |
| 对齐深度 / 内参 | `/camera/camera/aligned_depth_to_color/image_raw`、`/camera/camera/aligned_depth_to_color/camera_info` |

`perception-node` 只在已应用相机外参后把真实帧转换到 `base_link`。未启用感知时不发布
占位场景；每次应用感知配置时会清除上一来源的 Marker，再发布当前来源；计算
服务失败时保留原始错误，不切换模型或伪造结果。

仓库提供两个走正式链路的确定性来源：

- `generated:pick-place-scene`：固定 RGB 资产经真实 YOLOE 分割，再组合确定性深度，生成
  红色立方体、灰色置物筐和筐内放置区；实例点云继续送入真实 GraspGenX 生成抓取候选，
  用于完整抓放验收。
- `generated:depth-grid`：497 点测试云，用于 RViz 点云输入验收，不进入 MoveIt
  规划场景。

抓放按 ID 选择 `SceneObject` 和 `PlacementRegion`。Rust motion 只做场景映射、唯一 FIFO 和
Action 状态转发；同一容器内的 StarArm-102 MTC 组件用标准 stage 一次构造并选择完整任务解，
负责候选抓取/放置位姿、IK、接近、夹爪、attach/detach、搬运、回撤和返回工作位。执行仍经
MoveIt、ros2_control 和唯一 `ArmCommand`。任务开始先原地打开夹爪，放置后保持打开完成回撤，
最后回到工作位再闭合。网页显示 MTC 状态、stage、候选数和选中代价；RViz 的 Motion Planning
Tasks 面板显示候选、失败 stage 和选中轨迹。

GraspGenX 的 StarArm-102 夹爪扫描体和 URDF 由 `tools/graspgenx` 从固定厂商提交应用项目模型
patch 后导出。生产场景不硬编码抓取姿态；相同点云以可配置推理种子产生可复现候选。

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

## TCP 与机械臂模型

StarArm-102 的业务末端只有 `tcp_link`。它在 URDF 中通过零变换固定到结构链接 `link6`，
当前位置是两侧夹爪尖端中心。MoveIt group、FK/IK 和抓放目标全部使用
`stararm_102_model::TCP_FRAME`。73.13 mm 只用于夹爪转轴圆弧演示，不是工具偏移。完整维护
规则见 [StarArm-102 末端坐标](TCP.md)。

厂家源码固定在提交 `5979b346eb3a417840b29b76740754e4005d071a`。J3 轴向、关节范围、夹爪
范围、TCP 和 ROS 2 打包修正只存在于 `backend/patches/star-arm-102-fl-moveit-model.patch`；
构建和 GraspGenX 导出都先对干净提交应用该 patch，不维护运行时模型覆盖。

默认位为 J1–J6 全 0°；工作位为 J3=60°、J4=60°，其余关节为 0°。两者都携带夹爪闭合目标。夹爪
`primary_tool=0` 表示张开 90°，`1` 表示闭合 0°。J1–J6 与工具执行器在模型中分栏，但经
同一命令、同一执行节点完成。

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
- RViz/KasmVNC：`http://192.168.100.10:6080`

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

测试工具和生成物位于 `tools/`。过程缓存和生成的 RGB-D artifacts 不提交。
