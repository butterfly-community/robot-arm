# StarArm-102 末端坐标

## 唯一定义

`tcp_link` 是系统唯一的工具中心点（TCP），位于两侧夹爪尖端之间。它通过零位移、零旋转的
固定关节连接到结构链接 `link6`。二者当前数值位置相同，但语义不同：

| 坐标系 | 含义 | 是否用于业务目标 |
| --- | --- | --- |
| `link6` | 第六轴末端结构链接 | 否 |
| `tcp_link` | 夹爪尖端中心、MoveIt 规划末端 | 是 |
| `link7_left` / `link7_right` | 两侧活动夹指 | 否 |

厂家 URDF 中两侧夹指转轴位于 `link6` 的 `z=-73.13 mm`，夹指网格从转轴延伸到 `z≈0`。
因此原有 `link6` 原点确实位于夹爪尖端中心附近；新增 `tcp_link` 只把这个事实显式命名，
没有引入工具偏移。

## 怎么用

- MoveIt 的 `arm` group 以 `tcp_link` 为 `tip_link`。
- FK 请求、IK 请求、网页模型元数据、感知生成的抓取/放置目标和附着物体都使用
  `stararm_102_model::TCP_FRAME`，不得直接写 `link6`。
- 目标位置就是期望夹爪尖端中心到达的 `base_link` 坐标，不需要先换算成腕部位置。
- 目标姿态就是夹爪尖端中心坐标系的姿态，不需要增加工具姿态补偿。
- 抓取后，碰撞物附着到 `tcp_link`；`link6` 和两侧夹指只作为允许接触的结构链接。
- 新的机械臂型号必须在自己的 URDF 中提供明确的 TCP 固定坐标系，并在型号模型 crate
  中只导出这一坐标系。

## 73.13 mm 的用途

`ARC_PIVOT_TO_TCP_M = [0, 0, 0.07313]` 表示夹爪转轴到尖端中心的几何向量，只用于生成
“尖端走圆弧”的演示轨迹。它不是 `link6 -> tcp_link` 变换，也不能用于 FK、IK、抓放、
碰撞物附着或相机标定。

## 禁止事项

- 不得在 motion、perception、Web 或 execution 中再次实现 `link6` 与 TCP 的位置换算。
- 不得添加 `tip_pose()`、`wrist_pose()` 一类业务层补偿函数。
- 不得把标定板到测试爪的变换当作正常夹爪 TCP 偏移。
- 不得让模拟、真机、手动控制和抓放使用不同末端坐标系。
- 如果以后真实测量发现 TCP 需要修正，只修改 URDF 中 `tcp_joint` 的固定变换；下游代码
  仍只使用 `tcp_link`，不能再增加补偿路径。

## 修改后的核对清单

涉及机械臂模型或笛卡尔目标的修改至少应核对：

1. 最终 URDF 中存在 `link6 -> tcp_joint -> tcp_link`，且 MoveIt group tip 是 `tcp_link`。
2. `RobotModelInfo.tcp_frame`、FK 的 link 和 IK 的 link 都来自同一个 `TCP_FRAME`。
3. 平移目标只改变 `tcp_link` 位置；定点旋转不改变 `tcp_link` 位置；圆弧只使用
   `ARC_PIVOT_TO_TCP_M` 构造轨迹。
4. 感知给出的抓取点可以直接作为 IK 目标，不出现额外 73.13 mm 加减。
5. 仿真与真机继续接收同一份关节命令，不在 execution 层解释 TCP。
