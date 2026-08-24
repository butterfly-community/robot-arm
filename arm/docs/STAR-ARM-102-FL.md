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

这些历史内容最多只能帮助理解接口形态，不能用于识别新品 FL 的硬件、零位或安全
限位。

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

Linux 的 `/dev/ttyUSB1` 只是示例，生产程序应根据 UC-01 的 USB 属性建立稳定 udev
名称。同一串口只能由一个进程占用。

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

### FL 连接、标定和读写行为

当前 `Stararm102FL` 的实际行为是：

- 连接后依次 ping ID 0–6。
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
MoveIt Servo 完成仿真运动学和约束，不把手柄 XYZ、四元数或 Servo 输出直接写给
`send_action()`，也尚未连接真实总线。

## ROS 资产的采用边界

根目录产品表说明 FL 支持 ROS2、MoveIt 和 Gazebo，但仓库里的 ROS2 模型没有在文件中
标注“新品 FL 硬件版本”，而且数值与当前 FL 产品表、当前 FL LeRobot 配置存在冲突：

- URDF 的 joint1 约为 ±130°，产品表为 ±110°。
- URDF 的 joint3 只有约 180°，产品表为 270°。
- URDF 的 joint4 上限约 126°，产品表为 90°。
- URDF 的 joint6 为 ±180°，产品表为 ±150°。
- 上游 MoveIt 覆盖配置曾把 ROS `joint6`（第六旋转关节、舵机 ID 5，不是舵机 ID 6
  的夹爪）最大速度写成 `13.14 rad/s`。本项目通过
  [`../patches/star-arm-102-moveit-joint6-velocity.patch`](../patches/star-arm-102-moveit-joint6-velocity.patch)
  将其明确覆盖为与其余旋转关节一致的 `3.14 rad/s`；舵机 ID 6 的夹爪不受该补丁影响。
  上游仍关闭了所有关节的加速度限制，真实硬件接入前需另外补充并验证保守加速度限制。
- ROS2 驱动默认启动会向零位运动，退出路径没有实现可靠停止。

当前仿真复用其几何链、网格、SRDF、KDL、控制器配置和 `GenericSystem`，并在容器临时
副本上通过项目补丁按新品产品表覆盖关节范围、添加 `tool0`、修正 Jazzy mimic 配置和
J6 速度。上游惯量、effort 和真实驱动仍不视为新品 FL 已验证参数；补丁也只标记为仿真
候选，必须先与实物尺寸、零位和末端坐标核对才能进入真实后端。

同理，历史 `Python_SDK/stararm102_ro.py` 虽然会向 follower 发送同步关节命令，但它的
目标来自 LD/HD 主臂角度复制。本文不再从中提取滤波窗口、刷新频率、裸数据包结构或
角度映射。

## 当前接入结论

可以直接用于设计的事实：

- 设备是 Star Arm 102-FL 新品从臂，6R + 1 夹爪，ID 为 0–6。
- 新品规格为 420 mm 臂展、500 g 建议负载、±0.5 mm 标称重复精度、12 V/10 A。
- 当前明确的独立软件入口是新版 LeRobot `Stararm102FL`，通过单独串口以 1 Mbaud 读写
  七个关节。
- MoveIt Servo 仿真链路已经完成；真实关节输出、反馈冻结、急停和通电安全控制仍属于
  后续阶段。

接入通电实物前必须确认：

1. 机身标签确实为 Star Arm 102-FL 新品，而非旧版同名 follower。
2. 实物电源接口、额定输入、UC-01 USB 身份和稳定串口名称。
3. ID 0–6 的实际舵机型号与当前位置反馈。
4. 每个关节的机械零位、正方向、硬止挡和保守软限位。
5. 夹爪逻辑比例与实际开口毫米。
6. 新品 FL 对应的 URDF、`base_link`、工具中心点和逆运动学模型。
7. 通信超时、程序退出、USB 断开、释放力矩和断电时的真实行为。

上述信息确认前，只允许电机失能检查或受控低速单关节点动，不放行 NOLO 驱动的整臂
闭环运动。

当前只读探针、版本化候选模型和仿真控制实现见
[Star Arm 102-FL 接入与仿真](STAR-ARM-102-FL-INTEGRATION.md)。

## 采用的上游文件

- [新品 FL 产品规格列](https://github.com/servodevelop/Star-Arm-102/blob/5979b346eb3a417840b29b76740754e4005d071a/README.md)
- [新版 FL 配置](https://github.com/servodevelop/Star-Arm-102/blob/5979b346eb3a417840b29b76740754e4005d071a/Lerobot/lerobot-stararm102/lerobot_teleoperator_stararm102/config_stararm102_fl.py)
- [新版 FL 连接、标定和读写实现](https://github.com/servodevelop/Star-Arm-102/blob/5979b346eb3a417840b29b76740754e4005d071a/Lerobot/lerobot-stararm102/lerobot_teleoperator_stararm102/stararm102_fl.py)
- [新版插件依赖](https://github.com/servodevelop/Star-Arm-102/blob/5979b346eb3a417840b29b76740754e4005d071a/Lerobot/lerobot-stararm102/pyproject.toml)
