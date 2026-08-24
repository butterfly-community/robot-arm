# MoveIt Servo 仿真链路

本目录保存本项目必须维护的 ROS 2 集成代码；厂家完整源码继续位于
`~/Develop/temp/Star-Arm-102`，不会复制进当前仓库。

当前链路只启动厂家 `mock_components/GenericSystem`，不会打开串口或加载厂家真实驱动：

```text
NOLO USB Rust server
  -> 坐标映射（平移 × 0.2，姿态不缩放）
  -> Unix socket JSON
  -> servo_ipc_bridge (rclpy，仅做消息转换)
  -> MoveIt Servo Pose API
  -> arm_controller
  -> GenericSystem /joint_states
  -> robot_state_publisher / TF2 (base_link -> tool0)
  -> Unix socket JSON
  -> 现有网页仿真
```

厂家 Humble 模型包已经在官方 Jazzy MoveIt 镜像中验证可构建。本项目在容器内的临时
副本上应用 [`../patches/`](../patches/) 中的补丁，外部 Git clone 保持干净：

- 按新品 102-FL 产品表覆盖六个旋转关节的模型和 `ros2_control` 范围；
- 添加与当前候选模型一致的 `tool0`；
- 将 MoveIt `arm` 组定义为 `base_link -> tool0` 链；
- 将 ROS `joint6` 最大速度从 `13.14` 覆盖为 `3.14 rad/s`。

## 启动

先启动 NOLO USB server，使其创建默认 IPC socket；再启动 MoveIt 仿真：

```bash
cargo run --release --manifest-path vr-xr/src/nolo-usb-server/Cargo.toml -- \
  --host=192.168.100.10

arm/ros2/run-moveit-simulation.sh
```

第二条命令使用官方 `moveit/moveit2:jazzy-release` 镜像，在容器临时目录中构建三个 ROS
包，然后以前台方式运行。`Ctrl+C` 只停止 MoveIt 仿真容器。若厂家源码不在默认目录，
可设置任务专用环境变量 `STAR_ARM_102_SOURCE=/绝对路径`。

网页仍访问 `http://192.168.100.10:8765/arm-simulator/`。只有 IPC 连接、Servo 状态和
`/joint_states` 反馈均有效时，Rust 才把仿真状态标记为可用；IPC 断开或反馈超过 100 ms
立即停止发布目标并显示故障。Servo 状态缺失、停止或超出 `-1..=6`，以及反馈序号重复或
倒退同样不会被当作正常状态；恢复后必须先松开 Squeeze，再重新按下建立接管原点。

IPC schema v2 的每份反馈同时包含 `/joint_states` 和 TF2 的 `base_link -> tool0`。当前
TCP 及新的 Squeeze 接管原点完全采用该 TF2 位姿；Rust 不维护 URDF 关节链或 FK。

平移比例在 Rust 的版本化设备配置中固定为 `0.2`：手柄移动 5 cm，目标 TCP 移动
1 cm。ROS 桥不再做第二次缩放；姿态旋转角度保持 1:1，由 Servo 的角速度限制约束。

NOLO 服务不会覆盖已经存在的 Servo IPC socket，避免第二个实例破坏正在运行的连接。
若进程崩溃后遗留 socket，必须先确认没有 `nolo-usb-server` 进程，再显式删除该单个
socket 后重启；不要把“路径存在”直接当作陈旧文件。

## 真实机械臂边界

真实 102-FL 后端保留为 TODO。以后只替换 `GenericSystem` 输出端并复用同一 Servo 输入，
但在确认稳定设备名、零位、方向、软限位、速度/加速度、反馈冻结检测和硬件急停之前，
不得启动厂家 `robo_driver`。厂家旧驱动构造阶段会复位多圈角，不属于当前仿真路径。
