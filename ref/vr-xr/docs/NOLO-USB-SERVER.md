# NOLO CV1 Rust USB 后端

这是当前默认数据后端。它通过 HIDAPI/libusb 独占读取 NOLO CV1，兼容现场 USB ID
`0483:5750` 和生产版 `28e9:028a`，不经过 OpenHMD、Monado 或 OpenXR。

后端会解密 64 字节新版报告，把 Controller 0、Controller 1 和 Head Marker
分别发布给网页，并为三者维护独立的 Rust `fusion-ahrs` 状态。两只手柄还分别维护
独立的三轴 One Euro 位置滤波状态。

## 能力与限制

输出包括：

- 基站跟踪空间中的两只手柄和 Head Marker 位置；
- 三个设备各自的 `[x, y, z, w]` 融合四元数；
- Menu、Trigger 和 Squeeze 按键；
- 采样序号、采样质量和 Fusion 状态诊断数据；
- Controller 0 的 Trigger 相对示教接管和 Menu 夹爪意图；它们只作为后续控制层输入，
  不访问机械臂。

后端只在对应设备采样序号发生变化时更新 Fusion 和发布新姿态。重复的旧负载不会再次
积分；实际 USB 读取错误或设备未打开时，已发布来源转为不可用。

状态字段的准确含义：

- `sample_sequence`：当前设备采样序号；手柄取 `report[24]`，Head Marker 取
  `report[59]`。
- `unchanged_ms`：当前设备的 `sample_sequence` 多久没有变化。
- `communication_fresh`：当前 USB 会话已经收到过该设备的新样本；实际断开时为 false。
- `hmd_unchanged_ms`：`report[59]` HMD/USB 中继采样序号多久没有变化。
- `hmd_relay_online`：当前 USB 会话已经收到过 HMD 新样本；实际断开时为 false。
- `samples_received`、`samples_missed`、`duplicate_reports`：序号连续性累计统计；序号按
  `u8` 回绕处理。
- `measured_rate_hz`、`sample_jitter_ms`：按最近两个新样本的间隔计算。
- `simulated`：该帧是否来自内置虚拟 64 字节报告源；真实 HID 始终为 `false`。

这些字段只描述通信/采样。它们不能判断基站是否开机，不能判断光学位置是否
有效。没有协议依据的 `pose_usable` 和 `optical_tracking_valid`，以及迁移期的
`source_online` 和 `flags` 均已移除。当前 API 不再伪装 OpenXR tracking flags。`time_ns`
是后端进程启动后的单调时间，不是设备硬件时间戳。

手柄输出两种位置字段：

- `position`：NOLO 报告换算出的原始光学标记位置。
- `filtered_position`：对当前 `position` 做 One Euro 滤波后的结果；手柄为三元素
  数组，头部和离线状态为 `null`。

`position` 始终保留未平滑数据，便于协议诊断、回放和定量比较。姿态变化不会再通过
猜测的旋转中心偏移制造空间位移。One Euro 只处理位置，姿态已经由 Fusion AHRS 融合，
不重复滤波。
滤波器直接依赖官方
[`casiez/OneEuroFilter`](https://github.com/casiez/OneEuroFilter) 的 Rust 实现，并固定
到提交 `d78925584245597f2aa9c4c01a802eb0f0b77fb9`。默认采用其参考示例参数
`min_cutoff=1.0 Hz`、`beta=0.1`、`d_cutoff=1.0 Hz`；名义采样率采用 NOLO 单手柄
`120 Hz`。这些参数没有基于主观手感进行二次调优。

USB 重连、设备序号停止、完整位姿标定、非有限输入或非递增时间戳都会清空对应手柄的滤波历史。
下一段有效跟踪的第一帧直接穿透，以免旧位置在恢复后造成拖尾。网页只在诊断区并列
显示原始和滤波位置；手柄空间渲染及页面标定仍使用原始 `position`。机械臂相对意图
消费 `filtered_position`，原始位置只用于显示和有限值校验。

姿态融合没有已确认的磁力计或绝对旋转观测，三路 yaw 都可能漂移。USB 端点总计
约 240 包/秒：两个 Controller 报告各约 120 包/秒；HMD 字段和 `report[59]`
出现在每包中，因此 Head Marker 融合按约 240 Hz 更新。

两种 Controller 报告内的 HMD 序号相位不同。后端分别跟踪两条约 120 Hz 的
`report[59]` 流，再合并其新样本和统计；禁止直接对相邻异类报告的字节 59 做差。

## Compose 运行与源码调试

先停止所有会争用同一个 HID 的其他程序。

完整服务直接在工程根目录运行：

```bash
docker compose up -d
docker compose down
```

下面的 Cargo 命令只用于单独调试 Rust 后端，不是完整服务的运行入口。

构建：

```bash
cd ~/Develop/job/my/embedded/robot-arm/vr-xr/src/nolo-usb-server
cargo build --release
```

本机运行：

```bash
./target/release/nolo-usb-server
```

指定局域网地址和端口：

```bash
./target/release/nolo-usb-server --host=192.168.100.10 --port=8765
```

可选参数：

```text
--host=IP
--port=PORT
--static-dir=PATH
--gyro-calibration-file=PATH
--servo-ipc=PATH
```

控制器位置固定使用 `casiez/OneEuroFilter` Rust 示例的上游参数，不提供本地调参入口。

零偏文件默认是 `vr-xr/state/gyro-bias-v1.json`，可用 `--gyro-calibration-file` 改写。
文件按 Controller 0、Controller 1、Head Marker 三个来源分别保存；运行时文件由程序
原子替换，`state/` 已忽略生成内容，不进入 Git。

默认静态目录为相邻的
`controller-viewer/public/`。服务没有登录认证；远程使用时只绑定
受信任局域网中的具体地址，不要绑定 `0.0.0.0`。

## HTTP 接口

- `/`：实时网页。
- `/arm-simulator/`：独立的 Star Arm 102-FL 三维反馈与控制页。
- `/events`：`status` 和 `pose` Server-Sent Events。
- `/api/status`：当前状态和三个设备各自最新帧 `latestFrames`；`latestTeleopIntent` 是
  Controller 0 的最新相对示教意图；`latestArmState` 是唯一机械臂快照，`motionStatus`、
  `serialState` 和 `controlMode` 分别给出普通运动、运行时串口和手柄/手动控制状态；
  `teleopComponents` 给出空间位置移动、前部抬起/前部往下、左旋/右旋三个采集开关；
  `simulationRequested` 和
  `simulationActive` 分别表示网页切换请求与虚拟报告线程实际状态。
- `POST /api/gyro-calibration/0|1|2`：开始对应设备的显式陀螺仪零偏标定。
- `POST /api/pose-calibration/0|1|2`：重新初始化对应设备的 Fusion；已有零偏时直接复用，
  没有零偏时同时开始首次零偏标定。
- `POST /api/simulation/start`：切到手柄控制，暂停真实 HID 读取并直接启动确定性虚拟 USB 报告；
  不提交 J1–J6 或夹爪运动。
- `POST /api/simulation/stop`：只停止虚拟报告并重新尝试连接真实 HID；不联动返回起始位，起始位
  由机械臂页起始位按钮提交普通六轴 `[0°, 0°, -3°, 0°, 0°, 0°]` 目标。
- `POST /api/arm-control-mode`：`{"mode":"teleop|manual"}`，切换命令来源且不产生运动。
- `POST /api/arm-teleop-components`：提交
  `{"position":true|false,"pitch":true|false,"turn":true|false}`。
  三个字段分别控制空间位置移动、前部抬起/前部往下、左旋/右旋是否进入机械臂目标；状态保存在
  当前 Rust 后端会话中，刷新网页不会丢失，服务重启后三项默认勾选。
- `POST /api/arm-motion`：提交完整的 J1–J6 弧度目标；`[0°, 0°, -3°, 0°, 0°, 0°]` 就是
  普通起始位运动。
- `POST /api/arm-motion/cancel`：取消当前普通运动请求。
- `POST /api/arm-gripper`：提交单个夹爪绝对弧度，不触发六轴规划。
- `GET /api/arm-serial`、`POST /api/arm-serial/connect`、
  `POST /api/arm-serial/disconnect`：查询、按用户端口连接和断开 Rust 串口。
  每次连接会只读查询 ID 0–6 的内部 PID、保持 PID、偏置、方向和死区，并通过
  `serialState.internal_parameters` 返回；查询失败只写入 `parameter_error`，不阻止连接。

`pose` 帧以 `source_id=0/1/2` 区分 Controller 0、Controller 1 和 Head Marker；
`sample_rate_hz` 给出名义采样率，`sample_sequence` 和 `communication_fresh` 对三种设备
具有统一语义。网页的“手柄 1 / 手柄 2 / 头部”按钮只切换显示，不会停止其他设备采集。

## 虚拟 USB 报告模拟

网页顶部“启动模拟”使用和真实设备相同的加密 64 字节报告边界。模拟器先构造
Controller 0/1 与 HMD 的位置、IMU、按键和两个采样序号，调用生产报告编码器，再把
结果交回生产 `decode_report`；解码之后继续走相同的 Fusion、One Euro、采样新鲜度、
Trigger Teleop 和 SSE 路径。它不会绕过采集链路直接写网页 JSON，也不会读取或覆盖
真实设备的陀螺仪零偏文件。虚拟输入使用独立的无持久化 Fusion 状态，并直接使用模拟器
生成的陀螺仪数据，不应用手工零偏或 Fusion Offset 的自适应零偏补偿；真实 USB 的补偿
路径不变。虚拟 Fusion 的 `delta_time` 来自生成报告自身的固定时间轴，不受 Docker 调度和
报告循环唤醒抖动影响；真实 USB 仍使用实际样本接收时间。

启动模拟只切到手柄控制，暂停真实 HID，然后直接产生虚拟报告并让网页切到 Controller 0；
不要求起始位也不提交机械臂运动。前 6.25 秒虚拟 Squeeze 保持
按下，位置和 IMU 保持静止，网页按原有流程从第二秒开始完成标定状态并记录原点、零姿态
和朝向基站的人体前方；这段流程不会把计算结果作为虚拟陀螺仪零偏。虚拟源保持静止到
第 10 秒，随后自动按住 Trigger，在
3 秒内连续上升 10 cm 并
直接进入循环。之后每个方向使用 3 秒平滑过渡，相反方向负责回到工作状态，30 秒一轮
并持续循环：

1. 从工作高度再向上 5 cm，到达相对原始零点 `+15 cm`；随后向下 5 cm，返回
   `+10 cm` 工作高度，不回到零点；
2. 向左 2 cm 后向右 2 cm、向前 5 cm 后向后 5 cm，分别回到工作中心；
3. 手柄左旋 8° 后右旋 8° 返回、前部抬起 8° 后前部往下 8° 返回；回程反向重走去程，
   分别回到工作姿态。

虚拟手柄姿态直接使用 8° 峰值角；进入机械臂后使用厂家 URDF 中夹爪后部安装位置到
`link6` 的 7.313 cm 几何生成末端圆弧目标。
模拟平移进入机械臂链路
后仍会应用 1:2 比例，因此手柄左右移动 2 cm 对应目标 TCP 1 cm，其他方向移动 5 cm 对应
目标 TCP 2.5 cm；姿态不应用该比例。
Controller 1 和 Head Marker 在该循环中保持静止，但仍按各自名义频率输出。

人体坐标原点和姿态参考保存在当前 Rust 后端会话的 `humanReferences` 中。普通 Squeeze 标定
完成时页面通过 `/api/human-reference/{source_id}` 把最终参考交给后端；虚拟 NOLO 则在会话
开始时直接登记生成器的确定参考。刷新页面后从 `/api/status` 恢复各来源对应的参考；之后
切换手柄或头部时使用该来源的最新帧，不从 Teleop 接管锚点反推，也不使用 `localStorage`、
`sessionStorage` 或其他网页存储。

同一生成器也提供独立程序 `nolo-cv1-simulator`，默认向标准输出写出一次完整的
43 秒数据（10 秒标定与接管准备、3 秒预抬升 10 cm 和一轮动作），
格式是连续拼接的加密 64 字节报告：

```bash
cargo run --release --manifest-path vr-xr/src/nolo-usb-server/Cargo.toml \
  --bin nolo-cv1-simulator -- \
  --output=/tmp/nolo-cv1-sim.bin --reports=11760
```

添加 `--realtime` 会按 240 报告/秒实时输出。输出文件必须不存在，程序拒绝覆盖已有
采集。这个程序模拟的是 NOLO HID 报告协议，不在操作系统中创建 USB 枚举项；后者需要
root、USB gadget 控制器或专用硬件，且不会提高当前采集到机械臂仿真的覆盖率。

## Controller 0 相对示教意图

后端在 USB 读取线程中直接把 Controller 0 新样本交给独立的 `TeleopIntent` 状态机，
不经过网页 SSE。状态机输出保存在进程内 Tokio `watch` latest-value 通道中，同一时刻
只保留最新值；`/api/status.latestTeleopIntent` 只是诊断快照，不是机械臂控制接口。

状态语义：

- `idle`：Trigger 未按下，没有活动意图。
- `active`：Trigger 保持按下，输出相对位置、相对姿态和 Menu 夹爪开合命令，
  其余层可直接由状态判断是否接管。
- `faulted`：USB、标定或输入数据使当前意图失效；下一份有效样本会在当前位姿重新锚定。

接管时记录当前 `filtered_position` 和 Fusion 四元数。`relative_position` 是当前滤波位置
减接管位置，仍处于 NOLO 跟踪空间轴；人体到机械臂坐标映射留给后续运动学层。
`relative_orientation` 使用 `[x,y,z,w]`，计算为
`inverse(q_start) * q_current`，并统一四元数符号，等价的 `q` / `-q` 不会产生跳变。

意图只保留状态、采样序号和时间、接管状态、相对位姿、夹爪意图及停止原因。原始/滤波
位置和当前四元数已经存在于 `latestFrames`，不在意图中复制。

下列情况会使意图进入 `faulted` 并清除相对输出：

- USB 读取中断、长度异常或解码失败；其他 HID 报告类型由解码器忽略，不会误伤正在
  工作的 Controller 0；
- Fusion 初始化或陀螺仪标定正在进行；
- 原始/滤波位置非有限、滤波位置缺失或四元数不可用；

后端启动时如果 Trigger 已经按住，会立即以当前有效位姿建立零相对锚点。活动期间
`gripper_pressed=true` 表示 Menu 按下，`false` 表示 Menu 松开；idle/faulted
时该字段为 `null`。状态机不打开 Star Arm 串口、不运行
逆运动学，也不直接产生关节目标。

## Star Arm 102-FL 统一控制

NOLO 进程只在 `moveit-servo.sock` 上提供一套 100 Hz latest-value IPC。ROS 始终运行同一个
`JointStateTopicSystem`、`arm_controller`、`hand_controller`、MoveIt Servo 和 MoveGroup。
运动学、限位、奇异、碰撞、平滑、路径规划和时间参数化都由 MoveIt 完成；动力学补丁保留
此前依据官方资料确认的 J1–J6 `30 rad/s²`。IPC schema v5 明确区分 Rust 发往 ROS 的
手柄目标、普通六轴运动、夹爪绝对角和统一 `ArmState`，以及 ROS 返回的七关节控制器设定值、
TF2 TCP 和运动状态。Rust 不保存 URDF 关节链，也不计算 FK；反馈年龄只用于展示。

Trigger 新接管时记录当前 TCP；相对输入零对应当前末端，重新接管不会跳回旧目标。
默认坐标映射为 NOLO 前 `-Z` → 机械臂前 `+X`、NOLO 右 `+X` → 机械臂右 `-Y`、
NOLO 上 `+Y` → 机械臂上 `+Z`。手柄自身姿态不复用位置矩阵，而是按实机动作单独映射：
空间移动平移整只夹爪并保持姿态。姿态手势保持夹爪后部安装基准点不动，并将位置和姿态作为同一个
六自由度末端目标交给 MoveIt：前部抬起（原始 `-X`）使尖端沿竖直圆弧向上移动，前部往下则沿同一圆弧向下返回；
左旋（原始 `+Z`）使尖端从机械臂上方向下看时沿水平圆弧逆时针移动，右旋则顺时针移动。
这些语义描述夹爪尖端的位置移动，不描述为改变指向。手柄原始 `Y` 轴的自身滚转不进入机械臂目标，
也不保留独立的左转向/右转向路径。
三组输入可在机械臂页用三个复选框独立控制。相对四元数转换成接管时手柄局部坐标系中的旋转向量，
不使用欧拉角。切换时以当前 TCP 重新锚定，复选框本身不产生位移。
平移坐标矩阵由机械臂版本化配置提供并接受正交性检查；姿态直接使用上述两个明确轴，不再保留额外映射矩阵。

平移缩放由版本化配置的 `translation_scale=0.5` 唯一控制：

```text
robot_delta = robot_from_nolo_position × controller_delta × 0.5
```

因此手柄移动 2 cm，机械臂目标 TCP 移动 1 cm。该比例只作用于相对位置；相对四元数
保持完整旋转角度，MoveIt Servo 的线速度和角速度限制也仍分别生效。

唯一 `ArmSnapshot` 包含固定 J1–J6 顺序的 `joints_rad`、夹爪绝对角 `gripper_rad`、
`software|serial` 反馈来源、TF2 当前/期望 TCP、Servo 状态和停止原因。IPC 不再发送关节
名称，也不保存逻辑角/模型角两套可逆副本。IPC 缺失、反馈非法、Servo 停止或状态未知时不发布目标；
有效反馈恢复后，即使 Trigger 一直按住也会在最新 TCP 重新锚定。
MoveIt 奇异、碰撞或关节边界显示为 `constrained`；状态码 `1/3/4` 的减速提示只出现在
Servo 状态中，不写入停止原因，状态码 `2/5/6` 的停止才给出对应停止原因。后续目标继续处理，
以允许移回可行区域。Servo 不使用有限奇异点硬停值。Menu 输入按机械臂配置转换成夹爪绝对角，ROS
桥不再二次解释开合状态，直接通过
`hand_controller/joint7_left` 发送 `JointTrajectory`。`JointStateTopicSystem` 把 J1–J6
与夹爪设定值发给 Rust `ArmJointIo`：未连接串口时，它们直接成为软件反馈；连接后同一组
设定值写入 ID 0–6，并由 Monitor 实测位置覆盖同一个 `ArmState`。独立 UART 库完整解码
Monitor 帧用于协议校验；生产适配器只把 ID 和位置写入当前状态，功率、电流、温度、状态及
多圈值不进入控制路径。本机实测确认 J4/ID 3 的串口正方向与模型相反，因此只在真机串口
边界对 J4 命令和反馈同时取反；纯软件反馈、MoveIt、URDF 和网页模型不变。
位置命令使用厂家连续 follower 循环的 `100 ms / 50 ms / 50 ms / power 0`，并按最终 0.1° 整数协议包精确去重；
同一包不在每个 10 ms 控制周期重复发送，以免反复重启舵机插值。任一编码值变化即发送，
不另加角度门限。

网页按用户给出的端口运行时连接。Rust 先 Ping 并 Monitor 读取 ID 0–6，不发送运动；连接不判断
软件或真机的起始位，读取成功后直接使用真机 J1–J6 与夹爪反馈。
断开后保留最后实测反馈并继续软件模式，不返回起始位、不重启 ROS。适配器不调用校准、原点、
多圈重置或力矩切换。运行中串口读写失败保留最后真机反馈，随后释放旧句柄并立即重开同一
端口；成功后继续读写，失败则显示实际错误并回到未连接状态，不增加重试次数门限或另一套
重连状态机。

统一起始位和示教启动姿态是 J1–J6 `[0°, 0°, -3°, 0°, 0°, 0°]`、夹爪 `+1°`；没有单独的起始位姿态、
工作姿态或自动展开阶段。该模型角与 FL 插件的
舵机逻辑多圈量 `[-270°, 0°] / direction=-6` 独立，不做直接除法换算。

网页“启动模拟”只切到手柄控制并直接开始虚拟 NOLO 输入，不发送机械臂或夹爪命令。
“停止模拟”也只停止虚拟输入。单独的起始位按钮会把 J1–J6 的
`[0°, 0°, -3°, 0°, 0°, 0°]` 提交为普通 `MotionRequest`，保持夹爪当前角度。所有六轴目标都使用
MoveGroup/OMPL、官方 TOTG 和标准 `ExecuteTrajectory -> JointTrajectoryController`；
执行前暂停 Servo 对同一 controller 的连续输出，结束后恢复。MoveIt 返回普通轨迹执行成功即完成，
不为起始位增加反馈容差二次判断。若起始状态已经自碰撞，标准规划失败后会通过 MoveIt 状态有效性服务取得
当前碰撞对，读取完整 ACM，并仅在本次 MoveGroup 请求中临时放行这些碰撞对后强制重新
规划；该重试适用于任意六轴目标，不存在控制器直发或起始位旁路。项目没有 3D 传感器，所以只能检查模型自碰撞，
不能感知工作区中的临时外部障碍物。

`/arm-simulator/` 每 33 ms 读取一次统一快照，用 `joints_rad` 的 J1–J6 驱动从厂家 URDF
构建的关节树，
并显示当前和目标 TCP。页面使用厂家 STL 表现本体，并按 `gripper_rad` 驱动
J7 左夹爪及 URDF mimic 右夹爪。页面可选择手柄/手动控制，并按用户输入连接 Rust 串口；
切换模式本身不产生运动。七个滑块直接读取已加载 URDF 的范围，拖动只显示目标数值，
不覆盖持续显示反馈的 Three.js 主模型；松开 J1–J6 时提交完整六轴目标，松开夹爪时只提交
绝对夹爪角。
网页源码不保存厂家 URDF、网格或第二份关节参数。Docker 构建先在厂家包临时副本上应用
与 MoveIt 相同的三个补丁，再由 `tools/prepare-controller-viewer.ts` 把其中的 description
URDF 和 meshes 写入最终 `/srv/controller-viewer/arm-simulator/models`；七个滑块运行时
直接读取该 URDF 的关节范围。

设备配置、厂家 MoveIt 模型补丁、具体限制和 P1 只读探针见
[Star Arm 102-FL 接入](../../arm/docs/STAR-ARM-102-FL-INTEGRATION.md)。
ROS 启动和补丁见 [MoveIt Servo 统一链路](../../arm/ros2/README.md)，方案边界见
[IK 与实时笛卡尔伺服选型](../../arm/docs/IK-SELECTION.md)。

内部解码、Fusion、`/api/status` 快照和 SSE 都按设备新样本更新。机械臂控制使用本机
`watch` + Unix socket latest-value 边界，不经过网页 SSE。

收到 Ctrl-C 或 SIGTERM 时，后端会通知现有 SSE 流结束，再退出进程并释放 USB。

手柄网页从 jsDelivr 加载固定的 `three@0.180.0`。机械臂页从 esm.sh 加载同版本 Three.js、
`OrbitControls` 和固定的 `urdf-loader@0.13.1`；加载器的 Three.js peer 也固定为 0.180.0，
避免生成第二份不同版本实例。工程不保存这些第三方源码；浏览器必须能访问
`cdn.jsdelivr.net` 和 `esm.sh`。

## 陀螺仪零偏标定

本节只适用于真实 USB 设备。虚拟报告保持相同的标定交互与诊断状态，但不读取、保存或应用
陀螺仪零偏，也不运行 Offset 自适应零偏补偿，避免把已知的模拟慢速旋转误判为传感器漂移。

每个设备第一次成功完成的手工陀螺仪零偏会立即写入
`vr-xr/state/gyro-bias-v1.json`。后端重启或 USB 重连时读取文件，并把对应 Fusion 状态
直接恢复为“零偏已完成”。因此重复长按 Squeeze 只使用最近的有效样本记录网页原点、
零姿态和人体前方，不重启 Fusion、不清空 Offset，也不重新采集零偏。

若某个设备在文件中没有有效条目，它第一次长按 Squeeze 时仍按原流程执行：第一秒用于
摆稳，随后重新初始化 Fusion，并要求连续静止 3 秒来计算零偏；成功后才记录原点并将
零偏写入文件。文件存在但缺少该来源、schema 不兼容、数值非有限或超过 Fusion 官方
静止门限时，都不会冒充有效零偏。

网页诊断区仍可明确要求覆盖当前设备的已保存零偏，也可以直接调用 HTTP 接口。点击后
将设备平稳放在桌面并保持静止。后端直接使用 `OffsetSettings::default()` 的持续时间
和角速度门限；任何超过 Fusion 默认静止门限的样本都会清零连续计时。重新标定成功后
原子覆盖文件中的对应来源。

运行时 Fusion Offset 继续使用 `OffsetSettings::default()`，用于补偿保存零偏后的残余
慢漂；Offset 的瞬时内部状态不写文件。显式零偏完成或从文件加载时会先清空 Offset，
保证切换瞬间只应用一份零偏；随后 Offset 才从零开始估计残差。网页会显示已加载/测得
的零偏、进度、加速度拒绝和恢复状态。

AHRS 使用 `Ahrs::new()`，全部算法参数由当前 `fusion-ahrs` 版本的
`AhrsSettings::default()` 提供，本工程不复制或覆盖默认值。NOLO 没有磁力计，因此
更新接口使用 `update_no_magnetometer()`。

后端不再叠加自定义快速收敛或动态增益。快速姿态动作后会按照 Fusion 的标准重力反馈自然收敛；
无磁力计时，绕重力方向的姿态不会由加速度计纠正。

## 采样前确认设备状态

NOLO CV1 手柄具有省电休眠/关机机制。手柄之前开过机，或者 HMD 中继仍持续收到
对应报告类型，都不能证明采样时手柄仍活跃；报告可能重复旧负载。

进行协议分析、位姿验收或示教采集前必须：

1. 分别确认基站、Head Marker 和目标手柄当前已开机。
2. 按键并移动每只待测手柄，使其退出省电状态。
3. 单独移动 Controller 0、Controller 1，确认对应位置或 IMU 原始负载确实变化。
4. 在采集记录中写明每个设备的开关机、唤醒和动作确认状态。

缺少任一项时，数据只能标记为排查样本，不能用于解释未知字节。

## 未知字段探针

`nolo-unknown-probe` 会同时读取两个 Controller 报告，统计 IMU 负载变化，以及
尚未命名的字节 22、23、31–36、43–48、55–58、60–63。运行前必须停止网页后端和
其他 USB 读取程序，并按上一节确认两只手柄状态。

```bash
cargo build --release --manifest-path vr-xr/src/nolo-usb-server/Cargo.toml \
  --bin nolo-unknown-probe
./vr-xr/src/nolo-usb-server/target/release/nolo-unknown-probe 12
```

参数是采样秒数，默认 10 秒。探针只打印统计结果，不保存原始抓包，也不修改设备。
协议结论统一维护在 [NOLO-CV1-PROTOCOL.md](NOLO-CV1-PROTOCOL.md)。

## 测试

```bash
cargo test --all-targets --manifest-path vr-xr/src/nolo-usb-server/Cargo.toml
cargo clippy --all-targets --manifest-path vr-xr/src/nolo-usb-server/Cargo.toml -- -D warnings
deno test --allow-read=vr-xr,/tmp --allow-write=/tmp vr-xr/tests
```

测试不访问 USB；实机采样必须单独执行探针或启动后端。
