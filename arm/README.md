# 机械臂本体

本目录保存机械臂本体的当前资料、后续驱动与测试。VR/XR 手柄采集仍独立维护在
[`../vr-xr/`](../vr-xr/)。

当前实物是可独立执行命令的 Star Arm 102-FL 新品从臂。上游资料提炼和接入边界见
[Star Arm 102-FL 新品关键参数](docs/STAR-ARM-102-FL.md)。

当前阶段已经完成设备描述、严格只读探针、NOLO 到 TCP 的坐标映射、官方 MoveIt Servo
双输出、TF2 反馈和网页数字孪生。真机 J1–J6 输出代码已经实现但尚未进行实物通电验收；
夹爪已接入与仿真相同的标准控制通道，真机启动前会配置并校验 ID 6 的厂家模式二参数。
服务默认仿真；下一阶段只处理实测参数、硬件急停、夹爪保护配置和分级验收。

当前代码分为三条严格隔离的链路：

- [`tools/stararm102_fl_readonly_probe.py`](tools/stararm102_fl_readonly_probe.py)：只执行
  ID 0–6 `ping` 和 `Present_Position` 读取，关闭时不释放力矩。
- [`src/stararm102-control/`](src/stararm102-control/)：完全不包含串口或电机写入的 Rust
  坐标映射、MoveIt IPC 契约和仿真反馈状态库；示教平移按 1:5 缩放，运行时不再执行
  或保存 FK/IK。`moveit_interface` 只定义 IPC 帧名、默认状态和已确认的 0～90° 夹爪
  模型端点，当前 TCP 统一来自 ROS TF2。
- [`ros2/`](ros2/)：复用厂家模型和官方 MoveIt Servo；仿真输出到 `GenericSystem`，真机
  输出经标准 `JointTrajectoryController`、官方 `JointStateTopicSystem` 和受限
  FashionStar SDK 薄适配层写 J1–J6 与夹爪 ID 6。两路独立运行，网页显式选择且默认
  仿真。

Trigger 在两种后端都通过 ROS `hand_controller` 命令 `joint7_left`；仿真中的
`joint7_right` 由 URDF mimic 反向联动，真机只写厂家指定的 ID 6。真机同时读取位置、
功率、电流、温度和原始状态字节，但不猜测接触门限。真机启动器先读取完整公开参数；
仅当不匹配时，把 ID 6 的堵转失锁保护关闭、B 设为 `2000 mW`、A 设为 `4000 mW`，保留
其他字段并回读校验。首次夹持仍须完成低风险实物验收。

三维数字孪生位于
[`../vr-xr/src/controller-viewer/public/arm-simulator/`](../vr-xr/src/controller-viewer/public/arm-simulator/)；
它加载厂家 URDF/STL，并读取当前所选后端快照；网页滑块不连接机械臂总线。服务启动后访问
`http://<服务地址>:8765/arm-simulator/`。

版本化设备配置、厂家 MoveIt 模型补丁、坐标映射和 Servo 边界见
[Star Arm 102-FL 接入与双输出](docs/STAR-ARM-102-FL-INTEGRATION.md)。
方案比较和 MoveIt Servo 接口边界见
[IK 与实时笛卡尔伺服选型](docs/IK-SELECTION.md)。

运行离线测试：

```bash
cargo test --all-targets --manifest-path arm/src/stararm102-control/Cargo.toml
python3 -m unittest discover -s arm/tests -v
```
