# Star Arm 102-FL MoveIt Servo

这里提供两条使用同一 MoveIt 模型和 IPC 协议的 ROS 2 Jazzy 链路：

```text
NOLO/Rust -> PoseStamped -> MoveIt Servo -> JointTrajectoryController
                                      ├─ 仿真：GenericSystem
                                      └─ 真机：JointStateTopicSystem -> FashionStar SDK -> ID 0–6
```

Rust 不计算 FK/IK；当前 TCP 来自 `/joint_states` 和 TF2。MoveIt 负责运动学、规划、碰撞、
奇异状态和标准关节约束，`JointTrajectoryController` 负责轨迹执行。项目硬件节点只在 ROS
话题与厂家 SDK 的 `Monitor` / `Goal_Position` 之间转换单位，不再增加授权、反馈时效、
产品行程、松开重按或故障闭锁层。

真机启动不会写入舵机保护、原点、多圈或力矩参数。夹爪功率、电流、温度和原始状态只作
反馈显示。

## 依赖

厂家公开仓库放在工程外：

```bash
git clone https://github.com/servodevelop/Star-Arm-102 \
  ~/Develop/temp/Star-Arm-102
```

也可以通过 `STAR_ARM_102_SOURCE=/绝对路径/Star-Arm-102` 指定现有仓库。启动脚本只把
所需 ROS 包复制到容器临时工作区，再应用项目补丁，不修改厂家仓库。

## 启动仿真

```bash
arm/ros2/run-moveit-simulation.sh
```

脚本使用 ROS domain 42，并创建 `vr-xr/state/moveit-servo.sock`。联合启动 NOLO 服务和
MoveIt 仿真时，也可以在工程根目录运行：

```bash
./manage-simulation-services.sh start
```

## 启动真机

先构建一次镜像：

```bash
docker build -t stararm102-moveit-hardware:jazzy \
  -f arm/ros2/Dockerfile.hardware arm/ros2
```

再传入实际串口；`/dev/ttyUSB0`、`/dev/ttyACM0` 和 `/dev/serial/by-id/...` 都可以：

```bash
STAR_ARM_102_PORT=/dev/ttyUSB0 arm/ros2/run-moveit-hardware.sh
```

真机使用 ROS domain 43 和 `vr-xr/state/moveit-servo-hardware.sock`，可以与仿真同时运行。
网页默认选择仿真，切换输出时控制器会在当前手柄和当前 TCP 重新锚定，不要求松开再按。

## 专用回零

网页“专用回零”对仿真和真机使用同一流程：

1. 以当前 J1–J6 反馈向 MoveGroup 请求全零目标；目标容差采用用户明确的 `±1°`。
2. 如果标准规划因起始自碰撞失败，通过 `GetStateValidity` 取得当前自碰撞对，并只在本次
   MoveGroup 请求的 planning-scene diff 中临时放行这些对后重新规划。
3. 官方 TOTG 根据动力学补丁中的已确认速度和 `30 rad/s²` 加速度生成轨迹时间。
4. 暂停 Servo，以 `ExecuteTrajectory -> JointTrajectoryController` 执行规划结果。
5. 新反馈在全零 `±1°` 内即报告成功，然后恢复 Servo。

这仍是同一条 MoveIt 规划路径，不存在 Python 直写关节角或第二套回零轨迹。点击一次即
规划并执行；“停止 / 取消”只取消当前回零操作。停止手柄模拟不会联动回零。

## 项目覆盖

- `star-arm-102-fl-moveit-model.patch`：FL 关节范围、`tool0`、SRDF arm 链和 Jazzy mimic。
- `star-arm-102-fl-moveit-dynamics.patch`：统一速度，并按用户此前根据官方
  `200°/s、100 ms` 示例换算后明确选定的 `30 rad/s²` 为 J1–J6 补齐加速度，使官方
  TOTG 可以工作。
- `star-arm-102-fl-topic-hardware.patch`：真机把 ros2_control SystemInterface 换成官方
  `JointStateTopicSystem`。
- `config/servo.yaml`：只覆盖当前机器人名称、控制器输出、厂家控制器不接受速度字段、
  已实测的折叠位姿自碰撞距离，以及用户要求移除的有限奇异硬停；其他参数使用 MoveIt
  2.12.4 默认值。

除上述有来源且由用户明确选定的动力学值外，项目不复制 MoveIt、控制器或厂家驱动已有门限。

## 验证

```bash
bash -n arm/ros2/run-moveit-simulation.sh arm/ros2/run-moveit-hardware.sh
python3 -m unittest discover -s arm/tests -v
cargo test --all-targets --manifest-path arm/src/stararm102-control/Cargo.toml
```

完整启动步骤见
[启动 NOLO 手柄控制 MoveIt 仿真](../../vr-xr/docs/START-HANDLE-MOVEIT-SIMULATION.md)。
