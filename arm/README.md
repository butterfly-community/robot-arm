# 机械臂本体

本目录保存机械臂本体的当前资料、后续驱动与测试。VR/XR 手柄采集仍独立维护在
[`../vr-xr/`](../vr-xr/)。

当前实物是可独立执行命令的 Star Arm 102-FL 新品从臂。上游资料提炼和接入边界见
[Star Arm 102-FL 新品关键参数](docs/STAR-ARM-102-FL.md)。

当前阶段已经完成设备描述、严格只读探针、NOLO 到 TCP 的坐标映射、官方 MoveIt Servo
双输出、TF2 反馈和网页数字孪生。真机 J1–J6 输出代码已经实现但尚未进行实物通电验收；
服务默认仿真，夹爪保持禁用。下一阶段只处理实测参数、硬件急停、夹爪闭环和分级验收。

当前代码分为三条严格隔离的链路：

- [`tools/stararm102_fl_readonly_probe.py`](tools/stararm102_fl_readonly_probe.py)：只执行
  ID 0–6 `ping` 和 `Present_Position` 读取，关闭时不释放力矩。
- [`src/stararm102-control/`](src/stararm102-control/)：完全不包含串口或电机写入的 Rust
  坐标映射、MoveIt IPC 契约和仿真反馈状态库；示教平移按 1:5 缩放，运行时不再执行
  或保存 FK/IK。`moveit_interface` 只定义 IPC 帧名、默认状态和已确认的 0～90° 夹爪
  模型端点，当前 TCP 统一来自 ROS TF2。
- [`ros2/`](ros2/)：复用厂家模型和官方 MoveIt Servo；仿真输出到 `GenericSystem`，真机
  输出经标准 `JointTrajectoryController`、官方 `JointStateTopicSystem` 和受限
  FashionStar SDK 薄适配层写 J1–J6。两路独立运行，网页显式选择且默认仿真。

Trigger/J7 当前只是仿真状态。真机夹爪不能持续命令到固定完全闭合角：接入时必须读取
位置、电流/功率和保护标志，慢速闭合并在接触、堵转或超时后停止继续收紧。舵机自身
保护只能作为最后防线，不能替代软件限位、状态监测和低风险实机验收。

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
