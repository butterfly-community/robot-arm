# StarArm-102 型号适配

本文件集中维护 StarArm-102-FL 独有的模型、坐标、命名位、夹爪、规划与验收事实。通用服务
架构不依赖这里的关节数、尺寸或舵机协议。

型号源码、ROS 包、补丁和镜像入口集中在
[`backend/devices/stararm-102`](../backend/devices/stararm-102/README.md)。感知节点通过通用
`RobotModelInfo.gripper_asset_id` 获取夹爪资产 ID，不保存或判断本型号名称。

## 模型来源与补丁

厂家源码固定在提交 `5979b346eb3a417840b29b76740754e4005d071a`。构建时始终先应用
`backend/devices/stararm-102/patches/` 中的型号补丁；不直接修改厂商仓库，也不维护运行时
模型覆盖。J3 轴向、关节范围、夹爪范围、TCP 和 ROS 2 打包修正位于 `model.patch`，动力学
与 ROS topic 修正分别位于 `dynamics.patch` 和 `topic-io.patch`。

`stararm-102-model` 从最终 URDF 生成模型信息与资源 manifest；execution 是模型资源和
FashionStar UART 映射的唯一所有者。GraspGenX 只在设备目录保存不可推导的清单；计算镜像
构建时从同一份已打补丁模型自动生成夹爪 URDF、网格、扫描体和配置，不读取预生成资产。

## 唯一 TCP

`tcp_link` 是该型号唯一工具中心点，位于两侧夹爪尖端之间。它通过零位移、零旋转固定关节
连接到结构链接 `link6`。两者当前数值位置相同，但语义不同：

| 坐标系 | 含义 | 是否用于业务目标 |
| --- | --- | --- |
| `link6` | 第六轴末端结构链接 | 否 |
| `tcp_link` | 夹爪尖端中心、MoveIt 规划末端 | 是 |
| `link7_left` / `link7_right` | 两侧活动夹指 | 否 |

厂家夹指转轴位于 `link6` 的 `z=-73.13 mm`，夹指网格延伸到 `z≈0`。因此新增 `tcp_link`
只是显式命名厂家模型已有的尖端中心，没有引入工具偏移。MoveIt group tip、FK、IK、抓放目标、
附着物体、模型元数据与标定观测都使用 `stararm_102_model::TCP_FRAME`。

`ARC_PIVOT_TO_TCP_M = [0, 0, 0.07313]` 只表示夹爪转轴到尖端中心的圆弧演示几何，不是
`link6 → tcp_link` 变换，不能用于 FK、IK、抓放、附着或相机标定。

## 关节、命名位与夹爪

| 名词 | 定义 | 单位 |
| --- | --- | --- |
| J3/J4 关节角 | 机械臂第三、第四关节各自的角度 | ° 或 rad |
| 夹爪驱动关节角 | 舵机直接驱动的 `joint7_left` 命令角；模型中的 `joint7_right` 由齿轮反向联动 | ° 或 rad |
| 两指总开角 | 左右夹指之间的实际夹角；按 2:1 齿轮联动为夹爪驱动关节角的两倍 | ° |
| 夹爪开口宽度 | 两夹指之间的线性距离 | mm |

- 默认位：J1–J6 全 0°，夹爪闭合。
- 工作位：J3=60°、J4=60°，其余 J1–J6 为 0°，夹爪闭合。
- `primary_tool=0` 表示夹爪驱动关节角 60°，对应两指总开角 120°；
  `primary_tool=1` 表示夹爪驱动关节角 0°，对应两指闭合。
- 夹爪机械模型允许夹爪驱动关节角达到 90°，对应两指总开角 180°；这是 URDF 与
  底层驱动的机械极限，不是应用工作开度。手动控制、功能绑定、MTC、GraspGenX 与 Web
  公布的工作开度统一为夹爪驱动关节角 0–60°，对应两指总开角 0–120°。
- `夹爪开口宽度` 专指两夹指之间以 mm 表示的线性距离，不得与上述两种角度混用。
- J1–J6 与夹爪在模型信息中分栏，但经同一 `ArmCommand` 和 execution 节点执行。
- `StarArmBus` 只在型号边界把 J1–J6 与夹爪驱动关节绝对角编码成 FashionStar 命令并解析 ID 0–6
  的反馈；软件和真机使用同一状态契约。

## MoveIt 与 MTC

`stararm-102-motion-node` 负责该型号的 TCP 数学、MoveIt/Servo 以及 MTC Action 桥接。
`stararm_102_mtc` 只封装 `arm` 规划组、`tcp_link`、夹爪关节和工作位，使用标准 MTC stage
构造完整抓放任务。抓取候选来自 GraspGenX，生产场景不硬编码型号专用抓取姿态。

任务开始原地打开夹爪，完成接近、抓取、attach、搬运、放置、detach 和回撤，回到工作位后
闭合夹爪。任务对象使用结构化几何，背景及放置容器的实际深度表面经官方 OctoMap 进入同一
PlanningScene；目标实例不会同时以点云和 `CollisionObject` 重复出现。

型号模型或笛卡尔目标修改后必须核对：

1. 最终 URDF 存在 `link6 → tcp_joint → tcp_link`，MoveIt group tip 是 `tcp_link`。
2. 模型元数据、FK 和 IK 都引用同一个 `TCP_FRAME`。
3. 平移只改变 TCP 位置，定点旋转不改变 TCP 位置，圆弧只使用 `ARC_PIVOT_TO_TCP_M`。
4. 感知抓取位姿可直接作为 TCP 目标，不出现额外 73.13 mm 加减。
5. 仿真与真机继续接收同一关节命令，execution 不解释 TCP。

## 20 轮抓放验收

2026-09-01 使用软件反馈执行确定性场景：固定 RGB 经真实 YOLOE 分割，深度反投影生成实例
点云，GraspGenX 生成候选，再进入完整 MTC 任务。连续 20 次均完成规划、抓取、放置、回撤和
返回工作位，每轮选中一个完整任务解。

| 项目 | 结果 |
| --- | ---: |
| 成功 | 20 / 20 |
| 单轮耗时（最小 / 平均 / 最大） | 8.232 / 8.290 / 8.337 s |
| 选中代价（最小 / 平均 / 最大） | 29.084 / 29.278 / 32.936 |
| 工作位最大终点误差 | 0.006° |
| 全部关节最大相邻反馈变化 | 2.027° |
| J6 单轮范围（最小 / 平均 / 最大） | 128.557° / 128.564° / 128.574° |

J6 的范围在 20 轮间只相差约 0.017°，没有单帧跳到 90°。这验证正式软件链路，不把实体
舵机既有的 3–5° 机械偏差变成软件门限。原始数据位于
`tools/diagnostics/results/pick-place-mtc-repeatability.json`，执行工具位于
`tools/diagnostics/pick-place-repeatability.mjs`。

## 禁止重复处理

- 不得在 motion、perception、Web 或 execution 再实现 `link6` 与 TCP 的位置换算。
- 不得添加业务层腕部补偿，或把标定板到测试爪的变换当成正常夹爪 TCP 偏移。
- 不得让模拟、真机、手动控制和抓放使用不同末端坐标系。
- 真实测量需要修正 TCP 时只修改 URDF 的 `tcp_joint`；下游仍只使用 `tcp_link`。
