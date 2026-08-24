# Star Arm 102-FL 接入与双输出

本文记录已经实现的 P1 设备边界、P2 末端相对示教仿真，以及尚待实机验收的 P3 真机
输出代码。服务默认且每次重启后都选择仿真；只有单独启动真机 ROS、网页显式选择真机并
松开再按 Squeeze，才可能写入 J1–J6。

## 阶段基线

P1/P2 已完成并冻结：项目已具备严格只读设备探针、版本化设备资料、NOLO 相对位姿到
机械臂 TCP 的映射、官方 MoveIt Servo 约束求解、TF2 当前位姿反馈、确定性虚拟输入和
只读网页数字孪生。Rust 不再维护 FK/IK，ROS 桥不再承担坐标缩放或运动学计算。

该结论只表示仿真链路和真机软件输出端已经具备，不表示新品 FL 已经允许通电运动。真实
输出仍必须完成 [TODO P3](../../vr-xr/docs/TODO.md#下一步p3-通电机械臂安全接入) 的
实物基线、安全参数、硬件急停和逐级放行。

## P1：设备配置与只读探针

版本化设备配置位于
[`../config/stararm102-fl.v1.json`](../config/stararm102-fl.v1.json)。其中保存：

- 新品 FL 的 6R + 1 夹爪、舵机 ID 0–6、产品表型号、1 Mbaud 和 degrees 反馈单位；
- 上游 FL 插件的逻辑方向，以及以该方向表达的产品表行程；
- 模型版本、ROS 基座/TCP 帧名、坐标映射和示教比例；
- 尚未从实物确认的稳定设备名、零偏和固件明确为 `null`，不使用猜测值。

生产关节总线直接使用上游 FL 包底层的官方 `fashionstar_uart_sdk==1.3.12`。已审计的
`lerobot_motor_starai 0.0.6/0.0.7` 在 `connect()` 内改变力矩并广播
`ResetLoop(0xFF)`，因此不能作为无状态连接入口。当前也不采用旧 ROS2 驱动：旧模型没有
标注新品 FL，启动/退出路径不满足本项目安全要求。项目只调用官方 SDK 已有的端口、ping、
位置同步读写，不重写串口协议。

严格只读探针是
[`../tools/stararm102_fl_readonly_probe.py`](../tools/stararm102_fl_readonly_probe.py)。它不实例化
上游 `Stararm102FL` 或 `StaraiMotorsBus.connect()`，因为两条路径都会改变设备状态。
探针直接打开官方 SDK 端口，能力面只包含：

1. 打开串口；
2. 依次 `ping` ID 0–6；
3. 使用 `Present_Position`、`normalize=False` 连续读取完整七关节反馈；
4. 不发送力矩命令地关闭串口。

任何 ID 缺失、反馈键缺失、`None` 或非有限值都会使整份结果 `valid=false`；不复用缓存，
也不调用 `Set_Origin`、`Reset_Multi_Turn`、`Goal_Position`、参数写入或力矩切换。

在安装了上游依赖的独立 Python 环境中，以 UC-01 稳定设备名运行：

```bash
python3 -m pip install -r arm/tools/requirements-readonly-probe.txt
python3 arm/tools/stararm102_fl_readonly_probe.py \
  --port=/dev/serial/by-id/<UC-01稳定名称>
```

`/dev/ttyUSB*` 默认被拒绝；只允许在首次发现时显式添加 `--allow-unstable-device`，输出仍
标记为不稳定。探针是验证工具，不会把结果自动写回配置，避免一次偶然枚举覆盖生产身份。

## MoveIt 模型边界

运行时只使用厂家 `Star-Arm-102` ROS 包的 URDF/SRDF/STL，并通过本仓库补丁修正新品 FL
关节范围、`tool0`、Jazzy mimic、动力学参数和 topic hardware interface；不再维护第二套
无网格 URDF。Rust 配置只读取运行时需要的关节名/方向、接口帧、默认模型角、夹爪模型
端点和坐标映射。舵机型号、产品行程及串口资料仍留在 JSON 供审计，但不复制进控制算法。
夹爪只命令主动 `joint7_left`，`joint7_right` 由 URDF mimic 反向联动。

厂家几何尚未在这台新品 FL 上完成尺寸和碰撞体测量验收；通电前仍需核对零位、轴方向、
尺寸、TCP、软限位和碰撞体。

## P2：MoveIt Servo 相对末端示教仿真

Rust 库 [`../src/stararm102-control/`](../src/stararm102-control/) 完全不依赖串口，只负责
坐标映射、目标 TCP、IPC 契约、反馈校验和网页快照。NOLO 服务以 100 Hz 通过 Unix
socket 发送唯一最新目标；ROS 侧薄桥接器把目标转换为标准 `PoseStamped`，MoveIt Servo
完成 IK、限位、奇异、碰撞和平滑，厂家 `GenericSystem` 返回 `/joint_states`，官方
`robot_state_publisher`/TF2 返回 `base_link -> tool0`。Rust 不保存关节链、不计算 FK/IK；
旧自写 `SimulationController` 已删除。

接管规则：

- 右侧 Squeeze 新按下时记录**当前仿真 TCP**，此时相对输入为零，所以目标无跳变；
- 按住 Squeeze 时目标为“接管 TCP + 手柄相对位姿”；Trigger 只控制夹爪开合；
- 松开 Squeeze 后停止并清除接管原点；下次接管从当时机械臂位置重新开始；
- MoveIt 的不可达目标、奇异、碰撞和关节边界状态显示为 `constrained`，后续命令继续
  流动，以允许离开约束；Rust 不维护另一套猜测工作空间；
- 上游意图故障、IPC 断开、反馈超过 100 ms、反馈序号未递增、反馈非法，以及 Servo
  状态缺失、停止或未知都进入 `faulted`；反馈恢复时若 Squeeze 仍按住不会自动接管，
  必须先松开再重新按下并建立新 TCP 原点。

默认坐标约定是机械臂常用的 `base_link`：前 `+X`、左 `+Y`、上 `+Z`。NOLO 跟踪轴
右 `+X`、上 `+Y`、前 `-Z` 映射为：

```text
robot +X = NOLO -Z   （前）
robot +Y = NOLO -X   （左）
robot +Z = NOLO +Y   （上）
```

平移比例为 `0.2`，即手柄移动 5 cm，机械臂目标 TCP 移动 1 cm。位置轴和手柄自身
姿态轴是两个不同的设备坐标系，不能复用同一个矩阵。
姿态按已经确认的手柄人体动作映射为：

```text
手柄头部抬起  （原始 -X）→ TCP 绕 -Y
手柄向左侧倾  （原始 +Y）→ TCP 绕 -X
手柄向左转向  （原始 +Z）→ TCP 绕 +Z
```

两组矩阵分别保存在 `robot_from_nolo_position_axes` 和
`robot_from_controller_orientation_axes`。二者都必须是正规正交矩阵；姿态矩阵通过
`R * q_relative * R^-1` 变换旋转轴，保留全部三个旋转自由度。参数不散落于 NOLO 采集
代码。

空间平移不会直接改变 TCP 姿态，只有手柄自身旋转才改变夹爪朝向。默认姿态下执行横向
平移时，MoveIt 可能主要旋转 J1 并保持末端朝向；在 1:5 比例下手柄 10 cm 只产生 TCP
2 cm 位移，因此应使用 TF2 当前/目标 TCP 判断结果，而不能用某个关节是否明显转动判断
坐标映射。

当前 ROS 仿真复用厂家 URDF/SRDF、KDL、碰撞 STL、`ros2_control` 和
`JointTrajectoryController`。项目补丁按新品产品表覆盖模型和 mock hardware 关节范围，
添加 `tool0`，把 `arm` 定义为 `base_link -> tool0`，并通过统一动力学补丁把 J1–J7
速度写为 `3.14 rad/s`、J1–J6 加速度写为 `30 rad/s²`。后者由 RA8-U25H-M 文档的
`200°/s`、`100 ms` 梯形加速示例换算后向下取整，仍需在 P3 实机验收。Servo 使用官方
Butterworth 平滑、0.08 m/s 线速度、0.8 rad/s 角速度、关节余量、奇异和碰撞检查；
Servo 参数只保存在 ROS 配置中，不再在 Rust 设备描述内重复维护。这些仍是仿真参数，
不自动放行实物。

专用回零复用同一个 MoveGroup/OMPL、碰撞模型和官方 TOTG，目标为严格 J1–J6 全零；
没有关闭碰撞，也没有第二套自写路径规划。仿真和真机轨迹都由标准
`ExecuteTrajectory -> JointTrajectoryController` 执行。真机先预览再确认，官方
`JointStateTopicSystem` 将 ros2_control 的单帧位置目标传给厂家 Python SDK 薄适配器。
Python 不实现 `FollowJointTrajectory`、轨迹插值或路径规划。
系统没有 3D 传感器，因此不加载 Kinect/点云更新器；自碰撞模型仍有效，但无法感知外部
临时障碍物。

现场通过仿真模型确认：J1–J7 全部 `0°` 是唯一默认姿态，也是示教启动姿态。配置不再
区分回零姿态与工作姿态，也不会在开始示教前自动展开。J1–J6 以逻辑角和换算后的模型角
全零初始化；J7 以闭合 `0°` 初始化，松开 Trigger 后显示张开 `90°` 目标，运动时间由
ROS 控制器负责。

`GET /api/status` 的 `latestArmSimulation` 和 `latestArmHardware` 分别给出
`idle/active/constrained/faulted`、
六轴逻辑关节角、
对应 URDF 模型关节角、夹爪角度及开合命令、TF2 当前 TCP、期望 TCP、Servo 状态、
反馈年龄和停止原因；`armOutputBackend` 给出当前唯一有效输出，默认是 `simulation`。
`backend=moveit_servo_simulation|moveit_servo_hardware` 明确区分两条 ROS 反馈。
夹爪模型张开 `90°`、闭合 `0°`；命令只面向 `joint7_left`，`joint7_right` 自动反向
联动。模型角与厂家 FL 插件的舵机逻辑多圈量
`[-270°, 0°] / direction=-6` 分开保存，不再用传动量推导模型角；网页运动继续复用
ROS 控制器的速度/加速度限制。仿真字段包含 `simulation_only=true`，真机字段为 `false`。
网页 J1–J7 滑块始终只是本地模型操作，不会写入任一后端。

仿真中的 Trigger 是两态显示，不是可直接复用的真机夹持算法。当前真机桥明确忽略夹爪
命令。真实 RA8-U35H-M 夹爪
需要读取位置、电流/功率和保护状态，使用受限速度闭合，并在接触、堵转或超时后停止
继续收紧；在验证保护阈值和开口映射前，不得持续命令完全闭合角。

## P3 真机输出软件边界

真机 ROS 链路继续采用 MoveIt Servo 求解，并与回零共用标准
`JointTrajectoryController`。ros-controls 官方 `JointStateTopicSystem` 提供 ros2_control
SystemInterface，本项目 `hardware_node` 只负责 topic 与官方
`fashionstar_uart_sdk==1.3.12` 之间的单帧状态/目标转发。它读取 `Present_Position`、写入
`Goal_Position`；MoveIt 本身不访问串口。节点只有 ping、位置读写和
`disconnect(disable_torque=false)` 能力，不调用 `Set_Origin`、`Reset_Multi_Turn`、
校准或力矩切换。单帧 `Goal_Position` 编码沿用已审计的
`lerobot_motor_starai 0.0.7` 默认值（运动时间 350 ms、加/减速时间各 50 ms），不是本项目
另行调出的动态参数；实际跟随效果仍需 P3 低速验收。

仿真和真机使用独立 Unix socket 与 ROS domain，可以同时运行。Rust 服务启动时总是选择
仿真，未选中的后端持续反馈但只收到 `enabled=false`。网页切换会使两条控制器都进入
重新接管状态，必须观察到 Squeeze 松开后才能建立新的当前 TCP 原点。

真机边界检查 J1–J6 完整反馈、100 ms 时效、有限值和新品 FL 产品行程；轨迹插值、速度、
加速度和连续性由 MoveIt 与 `JointTrajectoryController` 负责，不再用猜测的每包跳变门限
重复判定。任何边界失败停止新写入并要求释放后重接管。它是软件保护，不替代
硬件急停。当前代码没有经过这台实物的单关节和整臂通电验收，夹爪也保持禁用。

## 已自动验证

- 配置 schema、关节名唯一性、运行时字段有限性和坐标矩阵正交性；
- TF2 TCP 和坐标映射有限、非法目标四元数、反馈顺序/方向和关节范围校验；
- 接管无跳变、重新接管、位置三轴映射和手柄三种人体旋转映射；
- IPC 缺失、超时、乱序/重复序号、非法反馈、Servo 状态缺失/未知，以及故障后必须松开
  Squeeze 才能重新接管；
- 只读探针的唯一总线调用集合，以及反馈缺失不复用缓存；
- HTTP 双后端状态、默认仿真、显式切换及 `simulation_only` 契约；
- 真机适配层只对 J1–J6 使用 ping、`Present_Position`、`Goal_Position` 和非失能关闭；
  反馈缺失、超时、非有限值或产品行程越界均 fail closed；未启用的真机夹爪不会阻断六轴
  主线启动；
- 官方 Jazzy MoveIt 容器中完成三个 ROS 包构建、模型/KDL/碰撞/控制器/Servo 启动，
  以及 Rust↔ROS/TF2 端到端反馈验证（三轮现场检查反馈年龄约 0–9 ms）。

启动方法见 [`../ros2/README.md`](../ros2/README.md)。真实机械臂的实测参数、硬件急停、
夹爪闭环和分级通电验收仍属于 P3，不因真机输出代码或仿真通过而自动放行。
