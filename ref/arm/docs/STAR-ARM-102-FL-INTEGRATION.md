# Star Arm 102-FL 统一接入

## 当前控制链

ROS 只看到一套机械臂、controller 和 `JointStateTopicSystem`：

```text
手柄 / 网页普通目标 -> MoveIt Servo / MoveGroup -> arm_controller ┐
网页 / 手柄夹爪角  -> hand_controller                            ├-> joint_commands
Rust ArmJointIo     <- joint_states <- 统一 ArmState              ┘
```

`ArmJointIo` 没有串口时把七关节 controller 设定值作为软件反馈；用户在运行时连接串口后，
同一设定值编码到 ID 0–6，Monitor 实测值覆盖同一个 `ArmState`。ROS、HTTP 运动接口和页面
不判断模拟/真机 backend。

旧 Rust IK、Python 串口、双 ROS 配置、专用起始位、反馈超时闭锁、关节范围二次校验、夹爪
布尔命令和真机二次确认均已删除。项目不重复 MoveIt、controller 或厂家驱动已有约束。

## 模型与补丁

[`../config/stararm102-fl.v1.json`](../config/stararm102-fl.v1.json) 保存型号、坐标映射，并
明确使用厂家已有的 `link6` 作为当前末端控制 link。手柄采集层已有 1:2 平移缩放，不属于
本次统一执行层迁移。

容器临时副本依次应用：

- [`../patches/star-arm-102-fl-moveit-model.patch`](../patches/star-arm-102-fl-moveit-model.patch)：
  把厂家新版误导出的 ROS 1 `catkin` description 外壳适配为 ROS 2 `ament_cmake`，并把
  SRDF arm 链明确为 `base_link` 到 `link6`；不修改厂家 URDF 关节位置范围，也不添加未经
  实测的额外 TCP link；六轴 IK 插件由厂家 KDL 改为 Jazzy 二进制提供的 TRAC-IK。
- [`../patches/star-arm-102-fl-moveit-dynamics.patch`](../patches/star-arm-102-fl-moveit-dynamics.patch)：
  保留用户此前依据官方 `200°/s、100 ms` 示例确认并取整的 J1–J6 `30 rad/s²`。
- [`../patches/star-arm-102-fl-topic-io.patch`](../patches/star-arm-102-fl-topic-io.patch)：
  统一使用 ros-controls 官方 `JointStateTopicSystem`，命令和状态都包含夹爪主关节；其中
  J5 和夹爪主关节的 command interface 范围按同一厂家 URDF 修正为 `[-2.27, 2.27]`，
  消除厂家两份配置之间的冲突；其他关节范围原本一致，不改。

MoveIt 和网页镜像分别在各自的厂家临时副本上应用这三个补丁。网页随后通过
`vr-xr/tools/prepare-controller-viewer.ts` 从补丁后的 `stararm102_description` 生成最终
URDF 和 meshes；仓库不再维护网页模型副本或重复的关节参数。

## 普通运动与起始位

网页手动 J1–J6 和起始位都提交完整六轴 `MotionRequest`。MoveGroup/OMPL 规划并通过标准
`ExecuteTrajectory` 执行；起始位目标为 `[0°, 0°, -3°, 0°, 0°, 0°]`，保持夹爪当前角度。
MoveIt 返回普通轨迹执行成功即完成，不为起始位增加反馈容差二次判断。

起始状态确有模型自碰撞且初次规划失败时，bridge 读取本次真实接触对，只在本次请求的 ACM
中放行这些对，然后重试相同 MoveGroup 路径。该逻辑适用于任意六轴目标，不是起始位旁路。

## 运行时串口

Rust 通过独立 `fashionstar-uart` crate 使用厂家公开协议的 1 Mbps 帧格式：连接时 Ping
ID 0–6，并对每个 ID 执行一次只读内部 PID 参数查询，再同步读取 Monitor，不先发送位置。
查询结果通过 `serialState.internal_parameters` 交给网页模型标签；断开串口时保留最后一次
读数，下次连接重新读取。参数查询失败只显示 `parameter_error` 和“参数未读取”，不阻止串口
连接，也不触发重连。协议库完整解码 Monitor 帧，生产适配器只把 ID 和位置写入 `ArmState`。
连接不检查软件或真机起始位；真机 J1–J6 与夹爪反馈读取成功后直接替换当前统一状态。
连接后同步位置命令使用厂家 `Python_SDK/stararm102_ro.py` 连续 follower 循环的原值
`100 ms / 50 ms / 50 ms / power 0`，不再沿用此前 LeRobot 单帧适配器的 350 ms 默认值。
串口边界按最终编码的 0.1° 整数协议命令精确去重：命令未变化时不重复发送，避免每个
10 ms 控制周期重新启动舵机插值；任一编码值变化时立即发送。这里没有另加角度门限。

### 真机未到位与圆弧抖动记录（暂不处理）

2026-08-26 在 `J1=0°、J2=0°、J3=-20°、J4=0°、J5=0°、J6=0°` 测试位执行固定的
`前部抬起 8° → 前部往下 8° → 返回`。MoveIt/TCP 目标连续且回程重走同一条圆弧，串口
没有报错，Monitor 电流和功率也未显示过载；真机反馈则出现停顿后分段追赶：

| 阶段 | 关节 / 舵机 | 目标 | Monitor 实际 | 实际减目标 |
| --- | --- | ---: | ---: | ---: |
| 到达测试位 | J3 / ID 2 | -20.0° | -18.7° | +1.3° |
| 到达测试位 | J4 / ID 3 | 0.0° | -1.7° | -1.7° |
| 抬起最高点 | J4 / ID 3 | +13.1° | +10.9° | -2.2° |
| 下行中段 | J3 / ID 2 | -14.5° | -12.4° | +2.1° |
| 返回末段 | J3 / ID 2 | -18.7° | -17.3° | +1.4° |

下行时 J3 和 J4 分别停住并在不同时间开始追赶，因此末端表现为抖动下降。厂家反馈建议
调整 PID；当前决定是只记录问题，暂不修改 PID、保持 PID、死区或控制代码。

同日使用只读内部参数查询得到：

| ID | 关节 | Kp | Ki | Kd | 保持 Kp | 保持 Ki | 保持 Kd | 死区 |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | J1 | 400 | 0 | 2300 | 400 | 0 | 2300 | 3 |
| 1 | J2 | 800 | 0 | 50 | 800 | 0 | 50 | 3 |
| 2 | J3 | 800 | 0 | 50 | 800 | 0 | 50 | 3 |
| 3 | J4 | 200 | 0 | 1100 | 200 | 0 | 1100 | 3 |
| 4 | J5 | 200 | 0 | 20 | 200 | 0 | 20 | 3 |
| 5 | J6 | 200 | 0 | 1100 | 200 | 0 | 1100 | 3 |
| 6 | 夹爪 | 200 | 0 | 20 | 200 | 0 | 20 | 3 |

上述七个舵机的运行偏置、保持偏置和方向字段均为 `0`。这些值是现场读数，不代表本项目
建议的新参数。

断开串口保留最后七关节反馈并回到软件模式，不移动、不返回起始位、不重启 ROS。已经完成的
真实 USB 定性验收和剩余定量现场项目见
[`../../vr-xr/docs/TODO.md`](../../vr-xr/docs/TODO.md)。

只读探针仍可单独用于设备识别：

```bash
python3 arm/tools/stararm102_fl_readonly_probe.py --port=/dev/ttyUSB0
```

启动和验证命令见 [`../ros2/README.md`](../ros2/README.md)。
