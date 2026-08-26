# 机械臂本体

本目录保存机械臂本体的当前资料、后续驱动与测试。VR/XR 手柄采集仍独立维护在
[`../vr-xr/`](../vr-xr/)。

当前实物是可独立执行命令的 Star Arm 102-FL 新品从臂。上游资料提炼和接入边界见
[Star Arm 102-FL 新品关键参数](docs/STAR-ARM-102-FL.md)。

当前阶段已经完成设备描述、严格只读探针、NOLO 到 TCP 的坐标映射、官方 MoveIt Servo、
统一七关节反馈和网页控制。真机串口协议已与厂家 Python SDK 1.3.12 做双向伪串口交叉验收，
并已完成实物串口连接、Monitor 反馈、J4 方向、夹爪和圆弧运动的现场定性验证；定量精度、
外参和长期稳定性仍待继续测量。

当前代码按四个边界分工：

- [`tools/stararm102_fl_readonly_probe.py`](tools/stararm102_fl_readonly_probe.py)：只执行
  ID 0–6 `ping` 和 `Present_Position` 读取，关闭时不释放力矩。
- [`src/fashionstar-uart/`](src/fashionstar-uart/)：厂家公开 UART 帧、Monitor 和同步位置命令
  的独立 Rust 协议库；不包含机械臂关节顺序、弧度换算或夹爪正负号。
- [`src/stararm102-control/`](src/stararm102-control/)：完全不包含串口或电机写入的 Rust
  坐标映射、MoveIt IPC 契约和反馈状态库；示教平移按 1:2 缩放，运行时不再执行
  或保存 FK/IK。`moveit_interface` 只定义 IPC 坐标系和已确认的 1～90° 夹爪控制模型端点，
  当前末端统一使用厂家 `link6` 并来自 ROS TF2；夹爪 TCP 未实测前不添加额外工具坐标系。
- [`ros2/`](ros2/)：复用厂家模型和官方 MoveIt Servo；唯一 `JointStateTopicSystem`
  与 Rust `ArmJointIo` 交换七关节设定值和反馈。Rust 未连接串口时使用软件反馈，运行时
  连接后写 J1–J6 与夹爪 ID 6 并读取 Monitor。

Menu 和网页夹爪目标都通过 ROS `hand_controller` 命令 `joint7_left`；模型中的
`joint7_right` 由 URDF mimic 反向联动，真机只写厂家指定的 ID 6。UART 库完整解码
Monitor 帧以便协议校验；生产 `ArmJointIo` 只消费 ID 和位置，不把功率、电流、温度、状态
或多圈值写入控制状态，也不读取或改写堵转、原点或力矩配置。厂家推荐参数只作为资料记录，
不进入启动链。

三维数字孪生位于
[`../vr-xr/src/controller-viewer/public/arm-simulator/`](../vr-xr/src/controller-viewer/public/arm-simulator/)；
Docker 构建时从应用补丁后的厂家 description 包生成它使用的 URDF/STL，源码不保存模型副本或
第二套关节范围。它读取统一反馈，网页滑块松开后通过普通 HTTP 控制接口提交目标。服务启动后访问
`http://<服务地址>:8765/arm-simulator/`。

版本化设备配置、厂家 MoveIt 模型补丁、坐标映射和 Servo 边界见
[Star Arm 102-FL 统一接入](docs/STAR-ARM-102-FL-INTEGRATION.md)。
方案比较和 MoveIt Servo 接口边界见
[IK 与实时笛卡尔伺服选型](docs/IK-SELECTION.md)。

运行离线测试：

```bash
cargo test --all-targets --manifest-path arm/src/fashionstar-uart/Cargo.toml
cargo test --all-targets --manifest-path arm/src/stararm102-control/Cargo.toml
python3 -m unittest discover -s arm/tests -v
```

UART 库与厂家 Python SDK 的 `socat` 交叉命令及依赖参数见
[`src/fashionstar-uart/README.md`](src/fashionstar-uart/README.md)。
