# MoveIt Servo 双输出链路

本目录保存 Star Arm 102-FL 的 ROS 2 集成。仿真和真机共用同一套 NOLO 坐标映射、目标
TCP、MoveIt Servo IK、限位、奇异、碰撞和平滑；网页只选择最终输出端，服务每次启动
默认选择**仿真**。

```text
NOLO USB Rust server
  -> 目标 TCP Pose
  -> MoveIt Servo
     ├─ 仿真：JointTrajectoryController -> GenericSystem
     └─ 真机：JointTrajectoryController -> JointStateTopicSystem
                -> CommandSafetyGate -> fashionstar_uart_sdk -> Goal_Position
  -> /joint_states -> robot_state_publisher / TF2
  -> Rust 状态 -> 网页数字孪生
```

MoveIt 不访问串口。MoveIt Servo 和回零都把 J1–J6 轨迹交给标准
`JointTrajectoryController`；ros-controls 官方 `JointStateTopicSystem` 把当前单帧位置目标
发给本项目 `hardware_node`。该节点完成最后一层检查和总线写入，只使用官方 SDK
的打开端口、ping、
位置读取和同步位置写入，没有校准、设置原点、重置多圈角或力矩接口。已审计的
`lerobot_motor_starai 0.0.6/0.0.7` 的 `connect()` 会改变力矩并广播 `ResetLoop(0xFF)`，
因此本项目明确不调用该连接路径。

厂家完整源码仍位于 `~/Develop/temp/Star-Arm-102`。两个启动脚本仅在容器临时副本中应用
[`../patches/`](../patches/) 的新品 FL 模型补丁，不修改外部仓库。

## 默认仿真

先启动 NOLO 服务，再启动 MoveIt 仿真：

```bash
cargo run --release --manifest-path vr-xr/src/nolo-usb-server/Cargo.toml -- \
  --host=192.168.100.10

arm/ros2/run-moveit-simulation.sh
```

打开 `http://192.168.100.10:8765/arm-simulator/`。顶部“仿真 / 真机”按钮只切换 Rust
目标路由，不负责启动 ROS 或访问设备。默认以及每次重启 NOLO 服务后均为“仿真”。切换
输出会清除 TCP 接管原点；必须先松开、再按 Squeeze 才能向新输出发送有效目标。

仿真使用 `ROS_DOMAIN_ID=42` 和 `/ipc/moveit-servo.sock`，真机使用
`ROS_DOMAIN_ID=43` 和 `/ipc/moveit-servo-hardware.sock`，因此可以同时在线而不会互相
订阅 ROS topic。未选中的链路只收到 `enabled=false`，但继续返回独立反馈用于网页显示。

仿真容器的 `GenericSystem` 从 J1–J7 全零启动。网页停止虚拟 NOLO 不会自动移动机械臂。
需要主动回零时，在机械臂页面使用“专用回零”：MoveIt/OMPL 保持正常碰撞检查，目标为
J1–J6 全零，官方 TOTG 生成时间参数；仿真和真机都由标准
`ExecuteTrajectory -> JointTrajectoryController` 执行。真机必须先预览并再次明确确认。
ros-controls 官方 `JointStateTopicSystem` 连接 ros2_control 和 Python SDK 薄适配器；
Python 不实现 `FollowJointTrajectory`、轨迹插值或路径规划。该动作独立于示教，
不修改舵机原点、多圈状态或力矩，仍属于 P3 现场验收项。

该 topic 硬件接口来自 ros-controls 官方
[`topic_based_hardware_interfaces`](https://github.com/ros-controls/topic_based_hardware_interfaces)，
本次按 `main@8c84b35bc14b871a021761b92b4604332d3e7087` 审核接口，并在 Jazzy 二进制包
`joint-state-topic-hardware-interface 1.1.0-1` 上完成无设备构建和控制器冒烟测试；项目
不复制或维护它的实现。

如需重建整个仿真进程，也可以手工执行：

```bash
docker restart stararm102-moveit-simulation
```

厂家 MoveIt 文件原先未提供加速度上限，Jazzy TOTG 因而无法生成可执行轨迹。本项目的
[`star-arm-102-fl-moveit-dynamics.patch`](../patches/star-arm-102-fl-moveit-dynamics.patch)
将 J1–J7 速度统一写为 `3.14 rad/s`，并为 J1–J6 设置 `30 rad/s²`。加速度依据
[RA8-U25H-M 高级速度控制](https://fashionstar.com.hk/wiki/zh/uart-servo/protocols/uart-rs485-protocol/#cmd-12)
给出的 `200°/s`、`100 ms` 示例换算值 `34.91 rad/s²` 向下取整；回零请求使用 `0.1`
缩放，因此规划上限为 `0.314 rad/s`、`3 rad/s²`。这是待实机验证的保守软件限制，不是
对整机动态性能的重新标定。

项目没有 Kinect、深度相机或点云传感器，启动时不加载厂家 `sensors_3d.yaml` 的可选
更新器。MoveIt 仍使用 URDF/SRDF 与碰撞网格完成机器人自碰撞检查；没有外部传感器就
不能感知临时进入工作区的人和物体。Jazzy 的 MoveGroup 即使收到空传感器配置仍可能
打印 `No 3D sensor plugin(s) defined for octomap updates`；这是上游已报告的空占据栅格
监视器诊断，不表示本项目配置了 Kinect，判断启动是否成功应以规划器、碰撞监视器和
`/move_action`、`/execute_trajectory` 是否就绪为准。

## 真机输出代码与启动边界

真机镜像只需构建一次：

```bash
docker build -t stararm102-moveit-hardware:jazzy \
  -f arm/ros2/Dockerfile.hardware arm/ros2
```

只有完成 TODO 中的只读身份、零位、方向、软限位和急停验收后，才可显式启动：

```bash
STAR_ARM_102_PORT=/dev/serial/by-id/<UC-01稳定名称> \
  arm/ros2/run-moveit-hardware.sh
```

脚本拒绝 `/dev/ttyUSB*` 作为用户输入，只把稳定名称解析后的单个设备映射到容器。启动
真机 ROS 进程本身不会运动；还必须在网页选择“真机”，松开 Squeeze，再重新按下。网页
滑块始终只改变浏览器模型，不会生成命令。

真机输出端执行以下边界检查：

- 上电后只 ping J1–J6（ID 0–5），并完整读取它们的 `Present_Position`；未启用的真机
  夹爪不会阻断六轴主线；
- 只接受包含 J1–J6、反馈在 100 ms 内且处于新品 FL 产品行程内的有限目标；
- 轨迹插值、连续性、速度和加速度由 MoveIt 与 `JointTrajectoryController` 负责，不在
  单帧总线适配器中用猜测门限重复判定；
- 反馈缺失、越界或读写错误立即停止新写入；
- 故障恢复或输出切换后必须观察到 Squeeze 松开，再允许重新接管；
- 退出时使用 `disconnect(disable_torque=False)`，避免软件退出导致负载突然失去保持力。

这些软件条件不能替代独立硬件急停。当前代码尚未在这台实物上完成分级通电验收，不能
因为单元测试或仿真通过就跳过 P3.1/P3.2/P3.5/P3.6。

真机夹爪有意保持禁用：现有资料只确认了模型 `joint7_left` 0°～90° 和舵机多圈范围，
还没有确认电流/功率、保护标志、接触阈值和安全开口映射。仿真 Trigger 命令不会被真机
桥转发，待 P3.4 闭环保护完成后再接入。

## 接口与故障规则

仿真 IPC 保持 schema v2 和原路径兼容。`GET /api/status` 同时提供
`latestArmSimulation`、`latestArmHardware` 与 `armOutputBackend`；
`POST /api/arm-output/simulation|hardware` 只选择输出。两份快照都包含关节角、TF2
`base_link -> tool0`、期望 TCP、Servo 状态和反馈年龄。

Rust 不维护 FK/IK。IPC 断开、反馈超过 100 ms、序号重复/倒退、非法 TF、Servo 停止、
碰撞、奇异或关节边界均沿用现有 fail-closed 规则。平移比例仍为 `0.2`，即手柄移动
5 cm、目标 TCP 移动 1 cm；姿态角保持 1:1。

NOLO 服务不会覆盖已经存在的两个 IPC socket。进程异常退出后，必须先确认没有所属进程，
再只删除确认陈旧的具体 socket，不能用递归或通配清理状态目录。
