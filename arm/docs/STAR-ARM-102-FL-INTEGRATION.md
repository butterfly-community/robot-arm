# Star Arm 102-FL 接入与仿真

本文记录已经实现的 P1 设备边界和 P2 末端相对示教仿真。当前实现不会驱动真实机械臂；
所有输出都是仿真关节目标和诊断快照。

## P1：设备配置与只读探针

版本化设备配置位于
[`../config/stararm102-fl.v1.json`](../config/stararm102-fl.v1.json)。其中保存：

- 新品 FL 的 6R + 1 夹爪、舵机 ID 0–6、产品表型号、1 Mbaud 和 degrees 反馈单位；
- 上游 FL 插件的逻辑方向，以及以该方向表达的产品表行程；
- 模型版本、候选关节链、仿真工作空间、坐标映射和控制约束；
- 尚未从实物确认的稳定设备名、零偏和固件明确为 `null`，不使用猜测值。

生产关节总线选择上游已经用于新品 FL 的 `lerobot_motor_starai>=0.0.6`。当前不采用旧
ROS2 驱动作为生产接口：旧模型没有标注新品 FL，启动/退出路径也不满足本项目安全要求。
项目不会根据历史示例自行重写 Fashion Star 串口协议。

严格只读探针是
[`../tools/stararm102_fl_readonly_probe.py`](../tools/stararm102_fl_readonly_probe.py)。它不实例化
上游 `Stararm102FL`，因为后者的 `configure()` 会释放力矩并执行 `Reset_Multi_Turn`。
探针直接打开同一个上游 MotorBus，但能力面只包含：

1. 打开串口；
2. 依次 `ping` ID 0–6；
3. 使用 `Present_Position`、`normalize=False` 连续读取完整七关节反馈；
4. `disconnect(disable_torque=False)` 关闭串口。

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

## 候选模型边界

[`../models/stararm102-fl-sim-v1.urdf`](../models/stararm102-fl-sim-v1.urdf) 是无网格的
版本化 6R 仿真模型：

- 关节变换来自审核提交中的旧 Star Arm 102 ROS2 几何链；
- 关节范围采用新品 FL 产品范围，并按当前 FL 插件逻辑符号表达；
- `base_link`、六关节轴和 `tool0` 与 JSON 配置逐项一致；
- Rust API 的 `joints_rad` 是 FL 驱动使用的逻辑角；只读 FK 显示会按厂家
  `model_angle = logical_angle / direction` 转换到 URDF 关节角。仿真快照另外发布
  `model_joints_rad`，网页不得把逻辑角直接写入 URDF；
- 没有 transmission、`ros2_control` 或执行器接口，不能被误用来驱动实物；
- 力矩、惯量和实机速度没有可靠新品来源，因此没有伪造这些参数。

该模型足以完成算法仿真，但仍是“候选几何”，不是经过测量验收的新品 FL URDF。实机
通电前仍需核对零位、轴方向、尺寸、TCP、软限位和碰撞体。

## P2：MoveIt Servo 相对末端示教仿真

Rust 库 [`../src/stararm102-control/`](../src/stararm102-control/) 完全不依赖串口，只负责
坐标映射、目标 TCP、IPC 契约、反馈校验和网页快照。NOLO 服务以 100 Hz 通过 Unix
socket 发送唯一最新目标；ROS 侧薄桥接器把目标转换为标准 `PoseStamped`，MoveIt Servo
完成 IK、限位、奇异、碰撞和平滑，厂家 `GenericSystem` 返回 `/joint_states`。旧自写
`SimulationController` 已删除，网页不再走第二套 IK。

接管规则：

- 右侧 Squeeze 新按下时记录**当前仿真 TCP**，此时相对输入为零，所以目标无跳变；
- 按住 Squeeze 时目标为“接管 TCP + 手柄相对位姿”；Trigger 只控制夹爪开合；
- 松开 Squeeze 后停止并清除接管原点；下次接管从当时机械臂位置重新开始；
- Rust 候选工作空间越界时不发布该目标；MoveIt 的奇异、碰撞和关节边界状态显示为
  `constrained`，后续命令继续流动，以允许离开约束；
- 上游意图故障、IPC 断开、反馈超过 100 ms 或反馈非法进入 `faulted`。

默认坐标约定是机械臂常用的 `base_link`：前 `+X`、左 `+Y`、上 `+Z`。NOLO 跟踪轴
右 `+X`、上 `+Y`、前 `-Z` 映射为：

```text
robot +X = NOLO -Z   （前）
robot +Y = NOLO -X   （左）
robot +Z = NOLO +Y   （上）
```

平移默认 1:1。位置轴和手柄自身姿态轴是两个不同的设备坐标系，不能复用同一个矩阵。
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

当前 ROS 仿真复用厂家 URDF/SRDF、KDL、碰撞 STL、`ros2_control` 和
`JointTrajectoryController`。项目补丁按新品产品表覆盖模型和 mock hardware 关节范围，
添加 `tool0`，把 `arm` 定义为 `base_link -> tool0`，并按已确认要求把 ROS `joint6`
速度覆盖为 `3.14 rad/s`。Servo 使用官方 Butterworth 平滑、0.08 m/s 线速度、0.8 rad/s
角速度、关节余量、奇异和碰撞检查；Servo 参数只保存在 ROS 配置中，不再在 Rust
设备描述内重复维护。这些仍是仿真参数，不自动放行实物。

现场通过仿真模型确认：J1–J7 全部 `0°` 是唯一默认姿态，也是示教启动姿态。配置不再
区分回零姿态与工作姿态，也不会在开始示教前自动展开。J1–J6 以逻辑角和换算后的模型角
全零初始化；J7 以闭合 `0°` 初始化，松开 Trigger 后才向张开 `45°` 限速运动。

`GET /api/status` 的 `latestArmSimulation` 给出 `idle/active/constrained/faulted`、
六轴逻辑关节角、
对应 URDF 模型关节角、夹爪角度及开合命令、关节速度、当前/期望 TCP、Servo 状态、
反馈年龄和停止原因。`backend=moveit_servo` 明确表示数据来自 ROS 仿真反馈。
夹爪张开 `45°`、闭合 `0°` 由厂家 FL 的 `[-270°, 0°] / direction=-6` 配置推导，
并复用现有关节速度/加速度限制。字段始终包含
`simulation_only=true`；它不是电机命令接口。

## 已自动验证

- 配置 schema、ID 0–6、未知字段保持 `null` 和坐标矩阵正交性；
- FK 和坐标映射有限、非法目标四元数、反馈顺序/方向和关节范围校验；
- 接管无跳变、重新接管、位置三轴映射和手柄三种人体旋转映射；
- IPC 缺失、超时、非法反馈和 Servo 奇异/碰撞/关节边界状态；
- 只读探针的唯一总线调用集合，以及反馈缺失不复用缓存；
- HTTP 状态字段、`simulation_only` 契约；
- 官方 Jazzy MoveIt 容器中完成三个 ROS 包构建、模型/KDL/碰撞/控制器/Servo 启动，
  以及 Rust↔ROS 端到端反馈验证（现场检查反馈年龄约 6 ms）。

启动方法见 [`../ros2/README.md`](../ros2/README.md)。真实机械臂输出端、反馈冻结检测、
急停和通电验收属于 P3，不因仿真通过而自动放行。
