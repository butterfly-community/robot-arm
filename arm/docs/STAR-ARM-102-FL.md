# Star Arm 102-FL 新品从臂关键参数

本文只整理当前项目持有的 **Star Arm 102-FL 新品从臂**。它是可独立接收命令并执行
运动的 Follower，不需要配套 LD/HD Leader 才能运行。

## 资料范围

- 上游仓库：<https://github.com/servodevelop/Star-Arm-102>
- 本地只读参考：`~/Develop/temp/Star-Arm-102`
- 审核提交：`5979b346eb3a417840b29b76740754e4005d071a`
- 提交时间：2026-07-30 11:09:45 +08:00

本文采用两级来源：

1. 根目录 `README.md` 规格表中明确标为 **Star Arm 102-FL** 的一列，用于硬件规格。
2. 新版 LeRobot 插件中明确命名为 `Stararm102FL` / `stararm102_fl` 的配置和实现，用于
   当前软件接口。

以下内容不再混入 FL 参数表：

- `Hardware/` 中明确属于 102-LD 的 BOM、CAD 和舵机配置；
- 旧 `lerobot-robot-stararm102` 中把所有电机写成 `ra8-u25` 的通用 follower；
- `Python_SDK/stararm102_ro*.py` 中的 LD/HD Leader→Follower 关节复制参数；
- 没有明确标注新品 FL 版本的旧 ROS2/URDF/MoveIt 数值。

这些历史内容只用于理解接口形态，不作为新品 FL 参数来源。

## 新品 FL 整机规格

以下只抄录上游当前产品表的 FL 列：

| 项目 | Star Arm 102-FL |
| --- | --- |
| 构型 | 6 个主动旋转关节 + 1 个夹爪执行器 |
| 臂展 | 420 mm |
| 建议最大负载 | 500 g |
| 标称重复精度 | ±0.5 mm |
| 整机重量 | 791 g |
| 编码器 | 12-bit magnetic encoder |
| 推荐工作温度 | 0–40 ℃ |
| 标称供电 | 12 V、10 A |
| 规格表电源接口 | XT30 |
| 通信集线器 | UC-01 |
| 通信方式 | UART |
| 上游支持 | LeRobot、ROS 2、MoveIt、Gazebo |

审核提交 `5979b346` 刚把 FL 建议负载从旧文档的 300 g 更新为 500 g；本文只保留当前
新品规格 500 g，不沿用旧值。

上游同一张表的 FL 配件栏写了“5.5×2.1 mm DC 电源转接线”，与 FL 电源规格栏的 XT30
冲突。这是 FL 自身资料的内部冲突，不是混入了主臂参数。接电前必须以新品实物接口、
铭牌和随附 12 V/10 A 电源为准。

### 新品 FL 舵机型号

以下同样只来自产品表 FL 列：

| 舵机 ID | FL 关节 | 新品规格表型号 |
| ---: | --- | --- |
| 0 | 关节 1 | RA8-U35H-M |
| 1 | 关节 2 | RX8-U50H-M |
| 2 | 关节 3 | RX8-U50H-M |
| 3 | 关节 4 | RA8-U35H-M |
| 4 | 关节 5 | RA8-U27H-M-C005 |
| 5 | 关节 6 | RA8-U35H-M-C047 |
| 6 | 夹爪 | RA8-U35H-M |

规格表原文把 ID 6 同时描述为“关节 7 / 夹爪关节”；本文统一称为夹爪，不引入额外的
第 8 个电机。

### 产品表物理行程

| 舵机 ID | 关节 | FL 产品表范围 |
| ---: | --- | --- |
| 0 | 关节 1 | ±110° |
| 1 | 关节 2 | 0°…180° |
| 2 | 关节 3 | 0°…270° |
| 3 | 关节 4 | ±90° |
| 4 | 关节 5 | ±65° |
| 5 | 关节 6 | ±150° |
| 6 | 夹爪 | 0°…90° |

旋转夹爪只通过主动关节 `joint7_left` 控制，范围为 `0°…90°`；`joint7_right` 是
`joint7_left` 的反向联动关节，使用 `multiplier=-1` 自动同步，不对应第二个舵机，也
不得暴露独立运动命令。

这些是产品描述，不自动等于以某个零位和正方向表示的软件命令区间。最终软限位仍应从
本机标定和机械止挡测量得到。

## 当前明确标注 FL 的软件接口

新版插件注册的机器人类型是 `stararm102_fl`，核心类是 `Stararm102FL`。它只需要
从臂自己的串口，可独立完成连接、读取和发送动作。

### 基础配置

| 项目 | 新版 `Stararm102FLConfig` |
| --- | --- |
| 串口 | 必填，例如 `/dev/ttyUSB1` |
| 波特率 | 1,000,000 baud |
| 舵机 ID | 0–6 |
| 命令单位 | degrees |
| Python | ≥3.10 |
| LeRobot | ≥0.4 |
| Fashion Star SDK | `fashionstar_uart_sdk >=1.3.12` |
| 电机总线包 | `lerobot_motor_starai >=0.0.6` |

上述是厂家 FL 插件的依赖关系，不是本项目运行时依赖。审计 `lerobot_motor_starai` 0.0.6
和 0.0.7 后确认其 `connect()` 会释放力矩并广播 `ResetLoop(0xFF)`。本项目的独立 Python
只读探针使用 `fashionstar_uart_sdk==1.3.12` 执行 ping 和 `Present_Position`；生产 Rust
使用独立 `fashionstar-uart` 库实现厂家公开帧格式，以 `Monitor` 取得七轴位置并发送同步
位置命令，不加载 Python SDK。该库已通过官方 `Packet`/`PortHandler` 的双向 `socat`
交叉测试；`ArmJointIo` 只消费 Monitor 的 ID 和位置，不把其他负载字段加入控制状态。

Linux 的 `/dev/ttyUSB1` 只是示例，也可以使用 `/dev/serial/by-id/...` 等稳定名称。同一
串口只能由一个进程占用。

### FL 逻辑关节配置

这是新版 FL 插件的逻辑坐标配置，不是新品产品表的物理角度定义：

| 逻辑名称 | ID | 原始反馈→逻辑值系数 | 插件裁剪范围 |
| --- | ---: | ---: | --- |
| `shoulder_pan` | 0 | −1 | −105°…105° |
| `shoulder_lift` | 1 | −1 | −180°…1° |
| `elbow_flex` | 2 | +1 | −270°…1° |
| `wrist_flex` | 3 | +1 | −90°…90° |
| `wrist_yaw` | 4 | +1 | −65°…65° |
| `wrist_roll` | 5 | −1 | −180°…180° |
| `gripper` | 6 | −6 | −270°…0° |

关节 1、2、6 的符号和夹爪的 `−6` 比例属于软件坐标映射。插件范围与产品表在关节 1、
关节 6 和夹爪上并不完全一致，因此当前只能作为上游软件默认值，不能直接当作本机安全
限位。夹爪尤其需要实测“逻辑角—舵机角—开口毫米”的关系。

### 夹爪执行器保护边界

厂家明确建议夹爪使用功率保护模式二：关闭堵转失锁保护，功率达到保护值 A 后由舵机
自动降到较低的堵转功率上限 B 保持夹持，且要求 `B < A`。厂家 Star Arm 参数脚本为
ID 6 给出 `stall_protection=0`、`stall_power_limit=2000 mW`、
`power_protection=4000 mW`、`current_protection=6000 mA` 的模板，但这不能证明当前实物
已配置为这些数值。

当前代码统一用 `hand_controller` 控制仿真与真机，只向真机 ID 6 发送位置目标。UART
协议库完整解码 Monitor 帧用于协议校验；启动链只消费 ID 和位置，不把其他字段写入控制
状态，也不读取或改写上述保护参数。厂家依据见
[功率保护参数说明](https://fashionstar.com.hk/wiki/zh/documents/servo/setting-protection-parameters/)，
项目参数模板见上游 `Python_SDK/set-param.py` 的 ID 6 配置。

### FL 连接、标定和读写行为

当前 `Stararm102FL` 的实际行为是：

- `lerobot_motor_starai` 的总线 `connect()` 打开串口并依次 ping 后，会释放力矩并广播
  `ResetLoop(0xFF)`；这一步早于 `Stararm102FL.configure()`，同样不是只读操作。
- 没有标定文件且 `calibrate=True` 时，释放力矩，要求人工摆到零姿态并闭合夹爪，随后
  执行 `Set_Origin`、`Reset_Multi_Turn` 并记录全行程。
- 即使使用 `connect(calibrate=False)`，后续 `configure()` 仍会释放全部关节力矩并执行
  `Reset_Multi_Turn`。因此上游的 observation 检查脚本并不是完全无状态的只读探针。
- `get_observation()` 使用 `Present_Position` 同步读取七个关节，并转换为上述逻辑坐标。
- 读取异常或任一电机返回空值时，代码会复用上一份缓存位置，但没有同时输出
  `valid=false`。该行为不能直接进入本项目的安全控制判据。
- `send_action()` 对每个逻辑目标执行范围裁剪，再通过 `Goal_Position` 同步写入。
- `disconnect()` 默认释放力矩。

FL 插件提供的是**关节角目标接口**，没有把 6D 末端位姿自动转换为关节角。本项目现用
MoveIt Servo 完成仿真与真机软件链路的运动学和约束，不调用上游 `send_action()`；
真机输出改由标准控制器和 Rust 串口薄适配层接收关节目标。当前已经完成实物串口连接、
Monitor 反馈、J4 方向、夹爪和圆弧运动的现场定性验收；定量精度、外参和长期动态仍待测量。

## ROS 资产的采用边界

根目录产品表说明 FL 支持 ROS2、MoveIt 和 Gazebo。厂家后续提供的 description 更新修正了
关节几何并同步替换了 9 个 STL，但没有对应的新 Git 提交，而且把 description 包外壳重新
导出成了 ROS 1 `catkin`。项目只在容器临时副本中将该外壳适配为 ROS 2 `ament_cmake`。

新版 URDF 的位置范围仍与 FL 产品表、当前 FL LeRobot 配置不同：

- URDF 的 joint1 约为 ±130°，产品表为 ±110°。
- URDF 的 joint2、joint3、joint4 均为约 ±90°；其中 joint3 总范围约 180°，产品表为 270°。
- URDF 的 joint5、joint6 和两个夹爪关节均为约 ±130°，与对应产品表范围不同。
- 上游 MoveIt 覆盖配置曾把 ROS `joint6`（第六旋转关节、舵机 ID 5，不是舵机 ID 6
  的夹爪）最大速度写成 `13.14 rad/s`。动力学补丁将 J1–J7 统一为 `3.14 rad/s`，并使用
  用户此前根据官方 `200°/s、100 ms` 示例换算后明确选定的 J1–J6 `30 rad/s²`；真实动态
  仍可另行测量核对。
- ROS2 驱动默认启动会向零位运动，退出路径没有实现可靠停止。

按用户确认，当前统一 ROS 路径以厂家新版 URDF 的位置范围为唯一来源；项目不按产品表增加
覆盖，也不在 Rust 配置中保存第二套位置限位。厂家 ros2_control 中 J5 和夹爪主关节与
URDF 不一致，topic I/O 补丁将这两项同步为 URDF 的 `[-2.27, 2.27]`。其余补丁负责 ROS 2
包装、把 arm 链结束在厂家已有的 `link6`、Jazzy mimic 控制边界和
`JointStateTopicSystem`；不添加未经实测的 TCP link。上游惯量、effort、几何和动力学数值
仍需与实物对照。

同理，历史 `Python_SDK/stararm102_ro.py` 虽然会向 follower 发送同步关节命令，但它的
目标来自 LD/HD 主臂角度复制。本文不再从中提取滤波窗口、刷新频率、裸数据包结构或
角度映射。

## 当前接入结论

可以直接用于设计的事实：

- 设备是 Star Arm 102-FL 新品从臂，6R + 1 夹爪，ID 为 0–6。
- 新品规格为 420 mm 臂展、500 g 建议负载、±0.5 mm 标称重复精度、12 V/10 A。
- 当前明确的独立软件入口是新版 LeRobot `Stararm102FL`，通过单独串口以 1 Mbaud 读写
  七个关节。
- MoveIt Servo、软件反馈与运行时串口链路已经完成，并已连接真机执行定性运动测试；J4
  串口方向、PID 读数和未到位/抖动现象已经记录。外参、定量误差与长期稳定性仍需现场测量。

后续仍需补充或定量核对：

1. 机身标签确实为 Star Arm 102-FL 新品，而非旧版同名 follower。
2. 实物电源接口、额定输入、UC-01 USB 身份和稳定串口名称。
3. ID 0–6 的实际舵机型号；当前位置反馈链已经现场验证。
4. 每个关节的机械零位、正方向和硬止挡；只有用户明确要求时才据此修改软件范围。
5. 夹爪逻辑比例与实际开口毫米。
6. 新品 FL 对应的 URDF、`base_link`、工具中心点和逆运动学模型。
7. 夹爪堵转、功率和电流保护的实际配置与状态反馈。
8. 通信超时、程序退出、USB 断开、释放力矩和断电时的真实行为。

这些测量用于修正模型和可复现问题，不作为项目新增软件门限的依据，除非用户明确要求。

当前只读探针、版本化模型和统一控制实现见
[Star Arm 102-FL 统一接入](STAR-ARM-102-FL-INTEGRATION.md)。

## 采用的上游文件

- [新品 FL 产品规格列](https://github.com/servodevelop/Star-Arm-102/blob/5979b346eb3a417840b29b76740754e4005d071a/README.md)
- [新版 FL 配置](https://github.com/servodevelop/Star-Arm-102/blob/5979b346eb3a417840b29b76740754e4005d071a/Lerobot/lerobot-stararm102/lerobot_teleoperator_stararm102/config_stararm102_fl.py)
- [新版 FL 连接、标定和读写实现](https://github.com/servodevelop/Star-Arm-102/blob/5979b346eb3a417840b29b76740754e4005d071a/Lerobot/lerobot-stararm102/lerobot_teleoperator_stararm102/stararm102_fl.py)
- [新版插件依赖](https://github.com/servodevelop/Star-Arm-102/blob/5979b346eb3a417840b29b76740754e4005d071a/Lerobot/lerobot-stararm102/pyproject.toml)
