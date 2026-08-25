# Star Arm 102-FL 接入与双输出

## 当前结论

当前主线只有一套运动学：NOLO 生成相对 TCP 位姿，MoveIt Servo 解算关节目标，
`JointTrajectoryController` 执行。仿真和真机只在 ros2_control 的末端接口不同：

```text
NOLO -> Rust IPC -> MoveIt Servo -> arm/hand controllers
                                  ├─ GenericSystem（仿真）
                                  └─ JointStateTopicSystem -> SDK（真机）
```

旧 Rust IK、Python 关节直写回零、真机授权门、反馈超时闭锁、关节范围二次校验、夹爪参数
配置器和真机二次确认均已删除。项目不再重复 MoveIt、控制器或厂家驱动已有约束。

## 设备描述

[`../config/stararm102-fl.v1.json`](../config/stararm102-fl.v1.json) 保存 FL 型号、ID、产品表
角度、坐标方向、MoveIt 帧和 1:2 平移比例。Rust 运行时只读取实际用于坐标转换的字段；
产品元数据不在运行时再次形成拒绝条件。

厂家 ROS 包在容器临时副本中应用两份补丁：

- [`../patches/star-arm-102-fl-moveit-model.patch`](../patches/star-arm-102-fl-moveit-model.patch)
  修正 FL 模型范围、mimic、`tool0` 和 SRDF arm 链。
- [`../patches/star-arm-102-fl-topic-hardware.patch`](../patches/star-arm-102-fl-topic-hardware.patch)
  把真机 ros2_control 接口换为官方 `JointStateTopicSystem`。
- [`../patches/star-arm-102-fl-moveit-dynamics.patch`](../patches/star-arm-102-fl-moveit-dynamics.patch)
  统一速度，并为 J1–J6 提供用户此前根据官方 `200°/s、100 ms` 示例换算后明确选定的
  `30 rad/s²` 加速度。

MoveGroup 回零由 OMPL 规划路径并由官方 TOTG 使用上述动力学值生成轨迹时间；桥接器不再
维护第二套时间参数化。

## 只读探针

探针只执行 ping 和位置读取，可使用任意实际设备路径：

```bash
python3 arm/tools/stararm102_fl_readonly_probe.py \
  --port=/dev/ttyUSB0
```

输出中的 `stable_device` 仅说明输入是否为 `/dev/serial/by-id/...`，不影响是否运行。

## 实时示教

- Squeeze 按住时生成相对位姿，松开时停止发布目标。
- 平移乘以 `0.5`，即手柄移动 2 cm 对应 TCP 1 cm；姿态为 1:1。
- 启动、输出切换或故障恢复后，如果 Squeeze 仍按住，系统在当前手柄位姿和当前 TCP 自动
  重新锚定并继续，不要求松开重按。
- MoveIt Servo 状态 1–6作为状态展示，但后续恢复命令继续流动；有限奇异硬停已按用户
  要求移除。
- 序号新鲜度和反馈年龄保留为诊断字段，不再作为项目输出门限。

## 专用回零

回零由 MoveGroup/OMPL 规划并通过标准 ExecuteTrajectory 执行。起始状态自碰撞时，桥接器
从 MoveIt 读取实际碰撞对，并仅在本次规划请求中放行这些起始对后强制重新规划；不是第二
条路径，也不修改全局碰撞矩阵。仿真和真机点击一次都自动规划并执行，反馈在用户明确的
全零 `±1°` 内即成功。

## 真机适配

`hardware_node` 打开用户传入的串口，ping ID 0–6，读取 `Monitor`，并把 ros2_control 的
J1–J6 与 `joint7_left` 位置目标编码为厂家 `Goal_Position`。结构长度和非有限数值仍作为
API/编码错误报告；产品行程、时效、授权、松开重按和故障闭锁不在这里重复实现。一次读写
失败会发布错误状态，后续回调继续重试。

夹爪反馈包含位置、功率、电流、温度和原始状态。启动代码不修改保护、原点、多圈或力矩
参数，也不解释厂家状态位语义。

启动、配置和验证命令统一见 [`../ros2/README.md`](../ros2/README.md)；尚待实测项目见
[`../../vr-xr/docs/TODO.md`](../../vr-xr/docs/TODO.md)。
