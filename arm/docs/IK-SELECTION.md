# IK 与实时笛卡尔伺服选型

## 结论

- 仿真主路径已经采用 **ROS 2 Jazzy + MoveIt Servo** 管理 IK/Jacobian、奇异、关节限位、
  碰撞、平滑和停止；旧 `SimulationController` 已删除，不再维护第二套自写运行时 IK。
- Rust NOLO 进程只生成带时间戳的目标 TCP 位姿，通过 Unix socket 与薄 `rclpy` 桥接器
  对接标准 Servo Pose API；当前 TCP 由官方 `robot_state_publisher`/TF2 根据厂家 URDF
  计算并随反馈返回，Rust 不保存关节链，也不实现 FK。
- 真实机械臂复用同一个 Servo 上游，用标准 `JointTrajectoryController` 和官方
  `JointStateTopicSystem` 替换 `GenericSystem` 输出端。新品 FL 的零位、
  方向、软限位、速度/加速度和急停未完成实机验收，因此真实后端代码不得直接放行。

这能把设备采集和机器人运动学分开：手柄链路负责“用户想让 TCP 怎样运动”，MoveIt
负责“当前机械臂是否能安全地这样运动”。浏览器不计算 IK，也不直接访问串口；机械臂页
只负责输出选择、标准回零请求和状态显示，实时关节状态来自对应 ROS 后端。

## 已核对的方案

### MoveIt Servo：双输出软件已接入，真实后端待现场验收

[MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
原生接收 Pose、Twist 或 JointJog 命令，并提供关节位置/速度限制、奇异检查、碰撞检查、
输入平滑和陈旧命令停止。这些正是通电示教需要由成熟框架统一处理的功能，比在本项目
逐项补写 IK 和安全边界更合适。

两条容器链路复用厂家 `stararm102_description`、SRDF、KDL、碰撞网格和
`JointTrajectoryController`；仿真使用 `mock_components/GenericSystem`，真机使用官方
`JointStateTopicSystem` 加受限 SDK 薄适配。本项目只保存必要补丁、Servo 参数、启动文件
和 IPC 桥接器；厂家完整源码保持在 `~/Develop/temp`。

### `openrr/k`：不替换当前 6R 求解器

核对 [openrr/k](https://github.com/openrr/k) 提交
`244364955475f84715680d51fa95bfa050bacb60`：它有纯 Rust URDF、FK、Jacobian 和迭代 IK，
但不提供 MoveIt Servo 已统一处理的碰撞、命令超时、控制器接入和完整安全状态。当前
项目不再保留自写 IK 作为后备路径，因此没有理由再维护一套纯 Rust 运行时分支。它可
用于离线交叉验证，不作为本项目的最终实物伺服层。

### `rs-opw-kinematics`：机械臂几何不适用

核对 [rs-opw-kinematics](https://github.com/bourumir-wyngs/rs-opw-kinematics) 提交
`fb51d85bd7a4504587e982387bd73e0059e23c0d`。它是成熟的纯 Rust OPW 解析解，但要求特定
的平行基座和球形腕部。用其官方 URDF 参数提取器检查当前候选模型时，提取器因关节偏移
不满足 OPW 结构而拒绝；当前模型的腕部三轴也不共点。因此不能使用。

### IK-Geo：保留作模型确认后的解析解研究

核对 [rpiRobotics/ik-geo](https://github.com/rpiRobotics/ik-geo) 提交
`a3a1675e1f01ad6f8f15f2cc787fa01472082a11`。当前候选链的 J2/J3/J4 平行且 J5/J6 相交，
落在其 `three_parallel_two_intersecting` 类别内；算法本身覆盖一般 6R，并有对应解析
分解。不过官方 Rust 包仍为 `linear-subproblem-solutions-rust 0.1.0`，没有 URDF 导入和
完整机器人模型层，需要人工推导 POE 参数。现阶段接入会把自写几何风险转移到参数转换，
所以只考虑在 FL 实际模型确认后用于离线解析解交叉验证，不作为 P3 主路径。

## 当前接口边界

```text
NOLO USB + Fusion + 位置滤波
             │
             ▼
相对手柄位姿（平移 × 0.2、姿态不缩放、latest-value、带时间戳）
             │
             ▼
MoveIt Servo（厂家模型、IK/Jacobian、限位、奇异、碰撞、平滑、超时停止）
             │
             ├── GenericSystem + /joint_states + TF2 TCP → Rust/网页（仿真）
             └── JointStateTopicSystem + SDK 薄适配 + 反馈/急停（P3 待现场验收）
```

硬件急停独立于以上软件链。MoveIt Servo 的输出仍需经过真实反馈新鲜度检查和驱动层
保守限制；仿真通过不会替代通电安全验收。运行与补丁说明见
[`../ros2/README.md`](../ros2/README.md)。
