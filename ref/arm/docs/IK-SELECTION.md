# IK 与实时笛卡尔伺服选型

## 结论

- 主路径采用 **ROS 2 Jazzy + MoveIt Servo** 管理 IK/Jacobian、奇异、关节限位、
  碰撞、平滑和停止；旧 `SimulationController` 已删除，不再维护第二套自写运行时 IK。
- 当前六轴 IK 插件使用 **TRAC-IK**，替换厂家默认 KDL。它直接实现 MoveIt
  `KinematicsBase`，不需要为每版 URDF 重新生成解析代码；保持 Jazzy 2.0.2 插件默认
  `Distance` 模式和现有 `5 ms` timeout，不添加另一套求解门限。
- Rust NOLO 进程只生成带时间戳的目标末端位姿，通过 Unix socket 与薄 `rclpy` 桥接器
  对接标准 Servo Pose API；当前末端直接使用厂家 URDF 的 `link6`，由官方
  `robot_state_publisher`/TF2 计算并随反馈返回。Rust 不保存关节链，也不实现 FK；在夹爪
  TCP 完成实测前不添加额外工具坐标系。
- 软件反馈和真实机械臂复用标准 `JointTrajectoryController` 与同一个官方
  `JointStateTopicSystem`；是否连接串口只由 Rust `ArmJointIo` 决定。新品 FL 的实际动态
  表现仍需接机测量。

这能把设备采集和机器人运动学分开：手柄链路负责“用户想让 TCP 怎样运动”，MoveIt
负责“当前机械臂如何实现这个目标”。浏览器不计算 IK，也不直接访问串口；机械臂页
负责手柄/手动切换、普通关节请求、串口选择和统一反馈显示。

## 已核对的方案

### MoveIt Servo：统一路径已接入并完成真机定性验收

[MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
原生接收 Pose、Twist 或 JointJog 命令，并提供关节位置/速度限制、奇异检查、碰撞检查、
输入平滑和陈旧命令停止。这些正是通电示教需要由成熟框架统一处理的功能，比在本项目
逐项补写 IK 和安全边界更合适。

唯一容器链路复用厂家 `stararm102_description`、SRDF、TRAC-IK、碰撞网格、
`JointTrajectoryController` 和官方 `JointStateTopicSystem`。本项目只保存必要补丁、Servo 参数、启动文件
和 IPC 桥接器；厂家完整源码保持在 `~/Develop/temp`。

### TRAC-IK：当前六轴求解器

[MoveIt 的 TRAC-IK 文档](https://moveit.picknik.ai/main/doc/how_to_guides/trac_ik/trac_ik_tutorial.html)
将其定义为 KDL 的直接替代。TRAC-IK 同时运行带随机跳出的 KDL 改进算法和 SQP 优化算法，
比 KDL 的单次牛顿迭代更能处理关节范围和局部极小值。Jazzy 已提供
`trac_ik_kinematics_plugin` 二进制包，因此当前阶段不维护外部求解器源码。

这项替换改善 Pose IK 和 MoveGroup 的收敛，不改变 MoveIt Servo 根据机器人雅可比条件数
生成的奇异状态。状态 1/3 是否出现仍由姿态和 URDF 几何决定，不能用更换 IK 插件掩盖。

容器实测已经确认 Servo 和 move_group 都加载
`trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`；`/compute_ik` 返回成功码 `1` 和一组六轴解。
同一轮标准模拟仍能采集到状态 1/3，证明求解器替换有效，但 1/3 不是 KDL 求解失败。
日志中的 `kdl_parser` 只负责解析 URDF 运动树，不是 IK 插件回退。

### IKFast：模型稳定后才考虑

[MoveIt IKFast](https://moveit.picknik.ai/main/doc/examples/ikfast/ikfast_tutorial.html) 可以为六轴链
生成速度很快的解析 C++ 插件，但生成结果与具体 URDF 几何绑定。厂家模型刚更新过 J3 几何，
其他轴和末端仍在确认，此时生成并维护 IKFast 只会增加一份需要随模型重做的代码。

### pick_ik 和 EAIK：当前不接入

`pick_ik` 的全局模式包含进化搜索和梯度优化，适合复杂目标和冗余机械臂，但不是当前实时六轴
路径最薄的替换。EAIK 能从 URDF 分析部分 6R 结构，但目前没有可直接安装的 MoveIt 2
`KinematicsBase` 插件，需要额外适配层；两者当前都不进入运行链。

### `openrr/k`：不替换当前 6R 求解器

核对 [openrr/k](https://github.com/openrr/k) 提交
`244364955475f84715680d51fa95bfa050bacb60`：它有纯 Rust URDF、FK、Jacobian 和迭代 IK，
但不提供 MoveIt Servo 已统一处理的碰撞、命令超时、控制器接入和完整安全状态。当前
项目不再保留自写 IK 作为后备路径，因此没有理由再维护一套纯 Rust 运行时分支。它可
用于离线交叉验证，不作为本项目的最终实物伺服层。

### `rs-opw-kinematics`：机械臂几何不适用

核对 [rs-opw-kinematics](https://github.com/bourumir-wyngs/rs-opw-kinematics) 提交
`fb51d85bd7a4504587e982387bd73e0059e23c0d`。它是成熟的纯 Rust OPW 解析解，但要求特定
的平行基座和球形腕部。用其官方 URDF 参数提取器检查当前修正模型时，提取器因关节偏移
不满足 OPW 结构而拒绝；当前模型的腕部三轴也不共点。因此不能使用。

### IK-Geo：保留作模型确认后的解析解研究

核对 [rpiRobotics/ik-geo](https://github.com/rpiRobotics/ik-geo) 提交
`a3a1675e1f01ad6f8f15f2cc787fa01472082a11`。当前模型链的 J2/J3/J4 平行且 J5/J6 相交，
落在其 `three_parallel_two_intersecting` 类别内；算法本身覆盖一般 6R，并有对应解析
分解。不过官方 Rust 包仍为 `linear-subproblem-solutions-rust 0.1.0`，没有 URDF 导入和
完整机器人模型层，需要人工推导 POE 参数。现阶段接入会把自写几何风险转移到参数转换，
所以只考虑在 FL 实际模型确认后用于离线解析解交叉验证，不作为 P3 主路径。

## 当前接口边界

```text
NOLO USB + Fusion + 位置滤波
             │
             ▼
相对手柄位姿（平移 × 0.5、姿态不缩放、latest-value、带时间戳）
             │
             ▼
MoveIt Servo（厂家模型、IK/Jacobian、限位、奇异、碰撞、平滑、超时停止）
             │
             ▼
arm/hand controllers -> JointStateTopicSystem -> Rust ArmJointIo
                                                    │
                                  软件反馈或 ID 0–6 串口 Monitor
                                                    │
                                                    ▼
                                      /joint_states + TF2 TCP → Rust/网页
```

真机已经完成串口反馈和运动的定性实测，定量精度、外参和长期稳定性仍待测量；MoveIt、
ros2_control 和厂家驱动自身的行为不由项目重复实现。运行与
补丁说明见
[`../ros2/README.md`](../ros2/README.md)。
