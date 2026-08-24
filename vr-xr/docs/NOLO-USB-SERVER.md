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
- Menu、Trigger、Home、Squeeze、Touchpad 按键和触摸板坐标；
- 原始加速度计、陀螺仪和采样序号诊断数据。
- Controller 0 的 Squeeze 相对示教接管和 Trigger 夹爪意图；它们只作为后续控制层输入，
  不访问机械臂。

后端只在对应设备采样序号发生变化时更新 Fusion 和发布新姿态。重复的旧负载不会再次
积分；USB 完全静默超过 200 ms 时，所有已发布来源都会转为不可用。

状态字段的准确含义：

- `sample_sequence`：当前设备采样序号；手柄取 `report[24]`，Head Marker 取
  `report[59]`。
- `unchanged_ms`：当前设备的 `sample_sequence` 多久没有变化。
- `communication_fresh`：上述时长不超过 200 ms；`source_online` 是兼容别名。
- `sample_changed`：这一帧是否来自新的设备采样序号。
- `pose_usable`：本帧同时满足通信新鲜且是新样本；这仍不表示光学跟踪有效。
- `optical_tracking_valid`：固定为 `null`，直至协议中找到经过验证的光学有效标志。
- `controller_sequence`：手柄帧为 `report[24]`，头部帧为 `null`。
- `hmd_unchanged_ms`：`report[59]` HMD/USB 中继采样序号多久没有变化。
- `hmd_relay_online`：上述时长不超过 200 ms。
- `sequence_delta`、`samples_received`、`samples_missed`、`duplicate_reports`：序号连续性
  累计统计；序号按 `u8` 回绕处理。
- `measured_rate_hz`、`sample_jitter_ms`：按新样本间隔计算的指数移动统计。
- `flags`：仅为旧 API 兼容保留，固定为 `0`，禁止解释为 OpenXR tracking flags。
- `simulated`：该帧是否来自内置虚拟 64 字节报告源；真实 HID 始终为 `false`。

这些字段只判断通信/采样新鲜度。它们不能判断基站是否开机，不能判断光学位置是否
有效，也不是 OpenXR runtime 返回的真实 tracking flags。`time_ns`
是后端进程启动后的单调时间，不是设备硬件时间戳。

手柄同时输出三种位置字段：

- `marker_position`：NOLO 报告中的原始光学标记位置。
- `grip_position`：根据历史 NOLO SDK 旋转中心偏移
  `[0, -0.0045, 0.0755] m` 计算的实验性握持点位置。
- `position`：当前主位置；默认等于 `marker_position`。
- `filtered_position`：对当前 `position` 做 One Euro 滤波后的结果；手柄为三元素
  数组，头部和离线状态为 `null`。

`grip_position` 的旋转方向和符号仍需实机验证。默认设置不会改变现有空间位置；只有
显式使用 `--controller-position=grip` 才会让 `position` 采用估算握持点。

`position`、`marker_position` 和 `grip_position` 始终保留未平滑数据，便于协议诊断、
回放和定量比较。One Euro 只处理位置，姿态已经由 Fusion AHRS 融合，不重复滤波。
滤波器直接依赖官方
[`casiez/OneEuroFilter`](https://github.com/casiez/OneEuroFilter) 的 Rust 实现，并固定
到提交 `d78925584245597f2aa9c4c01a802eb0f0b77fb9`。默认采用其参考示例参数
`min_cutoff=1.0 Hz`、`beta=0.1`、`d_cutoff=1.0 Hz`；名义采样率采用 NOLO 单手柄
`120 Hz`。这些参数没有基于主观手感进行二次调优。

USB 重连、设备序号停止、完整位姿标定、非有限输入或非递增时间戳都会清空对应手柄的滤波历史。
下一段有效跟踪的第一帧直接穿透，以免旧位置在恢复后造成拖尾。网页只在诊断区并列
显示原始和滤波位置；手柄空间渲染及页面标定仍使用原始 `position`。机械臂相对意图
已经消费 `filtered_position`，同时保留原始位置供冻结、跳变和安全判据使用。

姿态融合没有已确认的磁力计或绝对旋转观测，三路 yaw 都可能漂移。USB 端点总计
约 240 包/秒：两个 Controller 报告各约 120 包/秒；HMD 字段和 `report[59]`
出现在每包中，因此 Head Marker 融合按约 240 Hz 更新。

两种 Controller 报告内的 HMD 序号相位不同。后端分别跟踪两条约 120 Hz 的
`report[59]` 流，再合并其新样本和统计；禁止直接对相邻异类报告的字节 59 做差。

## 构建与运行

先停止所有会争用同一个 HID 的其他程序。

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
--controller-position=marker|grip
--gyro-calibration-file=PATH
--position-filter-min-cutoff=HZ
--position-filter-beta=VALUE
--position-filter-derivative-cutoff=HZ
```

三个位置滤波参数用于有测量依据时覆盖上游参考值：截止频率必须大于零，`beta` 必须
非负，所有值必须有限。无可靠测量时保持默认值。

零偏文件默认是 `vr-xr/state/gyro-bias-v1.json`，可用 `--gyro-calibration-file` 改写。
文件按 Controller 0、Controller 1、Head Marker 三个来源分别保存；运行时文件由程序
原子替换，`state/` 已忽略生成内容，不进入 Git。

默认静态目录为相邻的
`controller-viewer/public/`。服务没有登录认证；远程使用时只绑定
受信任局域网中的具体地址，不要绑定 `0.0.0.0`。

## HTTP 接口

- `/`：实时网页。
- `/arm-simulator/`：独立的 Star Arm 102-FL 只读三维仿真页；不改变 `/` 查看器。
- `/events`：`status` 和 `pose` Server-Sent Events。
- `/api/status`：当前状态、最后收到的一帧 `latestFrame`，以及三个设备各自最新帧
  `latestFrames`；`latestTeleopIntent` 是 Controller 0 的最新相对示教意图，
  `latestArmSimulation` 是 Star Arm 102-FL 最新仿真输出；`simulationRequested` 和
  `simulationActive` 分别表示网页切换请求与虚拟报告线程实际状态。
- `POST /api/gyro-calibration/0|1|2`：开始对应设备的显式陀螺仪零偏标定。
- `POST /api/pose-calibration/0|1|2`：仅当该设备没有已加载或已完成的零偏时，初始化
  Fusion 并开始首次零偏标定；已有零偏时为空操作。
- `POST /api/simulation/start`：暂停真实 HID 读取并启动确定性的虚拟 USB 报告循环。
- `POST /api/simulation/stop`：只停止虚拟报告并重新尝试连接真实 HID。网页随后提示手工
  执行 `docker restart stararm102-moveit-simulation`，完成后由用户点击确认。

`pose` 帧以 `source_id=0/1/2` 区分 Controller 0、Controller 1 和 Head Marker；
`sample_rate_hz` 给出名义采样率，`sample_sequence` 和 `source_online` 对三种设备具有
统一语义。网页的“手柄 1 / 手柄 2 / 头部”按钮只切换显示，不会停止其他设备采集。

## 虚拟 USB 报告模拟

网页顶部“启动模拟”使用和真实设备相同的加密 64 字节报告边界。模拟器先构造
Controller 0/1 与 HMD 的位置、IMU、按键和两个采样序号，调用生产报告编码器，再把
结果交回生产 `decode_report`；解码之后继续走相同的 Fusion、One Euro、采样新鲜度、
Squeeze Teleop 和 SSE 路径。它不会绕过采集链路直接写网页 JSON，也不会读取或覆盖
真实设备的陀螺仪零偏文件。

启动模拟会临时释放真实 HID，并让网页切到 Controller 0。前 6.25 秒虚拟 Menu 保持
按下，位置和 IMU 保持静止，网页按原有流程从第二秒开始完成 Fusion 零偏并记录原点、
零姿态和朝向基站的人体前方。虚拟源保持静止到第 10 秒，确保 Fusion 标定和标定故障
后的 Squeeze 松开门槛都已完成；随后自动按住 Squeeze，在 3 秒内连续上升 20 cm 并
直接进入循环。之后每个方向使用 3 秒平滑过渡，相反方向负责回到工作状态，36 秒一轮
并持续循环：

1. 从工作高度再向上 10 cm，到达相对原始零点 `+30 cm`；随后向下 10 cm，返回
   `+20 cm` 工作高度，不回到零点；
2. 向左 10 cm 后向右 10 cm、向前 10 cm 后向后 10 cm，分别回到工作中心；
3. 手柄头部抬起 3 cm 后下压 3 cm、向右侧倾 3 cm 后向左侧倾 3 cm，分别回到工作
   姿态；
4. 手柄向左旋转 5 cm 后向右旋转 5 cm，回到工作姿态。

姿态的“cm”不是角度单位。它按网页模型头尾标记间距 20.5 cm 计算：抬起和侧倾 3 cm
等价峰值角约 8.415°，左右旋转 5 cm 等价峰值角约 14.117°。模拟平移进入机械臂链路
后仍会应用 1:5 比例，因此手柄空间移动 10 cm 对应目标 TCP 2 cm；姿态不应用该比例。
Controller 1 和 Head Marker 在该循环中保持静止，但仍按各自名义频率输出。

同一生成器也提供独立程序 `nolo-cv1-simulator`，默认向标准输出写出一次完整的
49 秒数据（10 秒标定与接管准备、3 秒预抬升 20 cm 和一轮动作），
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

- `idle`：没有活动意图，等待观察到 Squeeze 松开后的新按下沿。
- `active`：Squeeze 保持按下，输出相对位置、相对姿态和 Trigger 夹爪开合命令，
  `enabled=true`。
- `faulted`：采样、USB 或标定使当前意图失效。恢复后仍保持锁定；必须先松开 Squeeze，
  再重新按下才能建立新原点。Trigger 不会解除故障或启动接管。

接管时记录当前 `filtered_position` 和 Fusion 四元数。`relative_position` 是当前滤波位置
减接管位置，仍处于 NOLO 跟踪空间轴；人体到机械臂坐标映射留给后续运动学层。
`relative_orientation` 使用 `[x,y,z,w]`，计算为
`inverse(q_start) * q_current`，并统一四元数符号，等价的 `q` / `-q` 不会产生跳变。

意图同时保留 `raw_position`、`filtered_position`、当前四元数、采样序号和单调 USB 接收
时间。`sample_valid` 只表示采集数据满足当前结构和新鲜度要求。协议未知的
`optical_tracking_valid=null` 会原样传递，绝不改写成光学跟踪有效。

下列情况会使意图进入 `faulted` 并清除相对输出：

- Controller 0 通信超时、没有新样本或 `pose_usable=false`；
- USB 读取中断、长度异常或解码失败；其他 HID 报告类型由解码器忽略，不会误伤正在
  工作的 Controller 0，是否断流仍以其自身采样序号为准；
- Fusion 初始化或陀螺仪标定正在进行；
- 原始/滤波位置非有限、滤波位置缺失或四元数不可用；
- 将来协议若明确发布 `optical_tracking_valid=false`。

后端启动时不会把已经按住的 Squeeze 当成新按下沿；必须先观察到一次有效松开。活动
期间 `gripper_closed=true` 表示 Trigger 按下，`false` 表示 Trigger 松开；idle/faulted
时该字段为 `null`，避免夹爪命令越过接管边界。状态机不打开 Star Arm 串口、不运行
逆运动学，也不直接产生关节目标。

## Star Arm 102-FL 仿真输出

NOLO 进程在本机 Unix socket 上提供 100 Hz latest-value IPC。它读取唯一最新
`TeleopIntent`，用 `stararm102-control` 完成坐标映射和目标 TCP，然后发送给 ROS 侧
`servo_ipc_bridge`。桥接器只转换 JSON、`PoseStamped`、`JointState`、`ServoStatus` 和
TF2 位姿；
运动学、限位、奇异、碰撞和平滑全部由 MoveIt Servo 完成。厂家 `GenericSystem` 的
`/joint_states` 与官方 `robot_state_publisher` 生成的 `base_link -> tool0` 经同一 socket
返回。Rust 校验 IPC schema v2、六关节名称/长度/有限值、TF2 TCP、FL 方向和范围后生成
`/api/status.latestArmSimulation`；Rust 不保存 URDF 关节链，也不计算 FK。
每条反馈还必须具有严格递增的 `sequence`；重复或倒退会断开该 IPC 会话，防止缓存或
重放数据刷新反馈新鲜度。

Squeeze 新接管时记录当前仿真 TCP；相对输入零对应当前末端，重新接管不会跳回旧目标。
默认坐标映射为 NOLO 前 `-Z` → 机械臂前 `+X`、NOLO 右 `+X` → 机械臂右 `-Y`、
NOLO 上 `+Y` → 机械臂上 `+Z`。手柄自身姿态不复用位置矩阵，而是按实机动作单独映射：
头部抬起（原始 `-X`）→ TCP `-Y`，向左侧倾（原始 `+Y`）→ TCP `-X`，向左转向
（原始 `+Z`）→ TCP `+Z`。两组矩阵都由机械臂版本化配置提供并分别接受正交性检查。

平移缩放由版本化配置的 `translation_scale=0.2` 唯一控制：

```text
robot_delta = robot_from_nolo_position × controller_delta × 0.2
```

因此手柄移动 5 cm，机械臂目标 TCP 移动 1 cm。该比例只作用于相对位置；相对四元数
保持完整旋转角度，MoveIt Servo 的线速度和角速度限制也仍分别生效。

输出包含六轴逻辑角 `joints_rad`、厂家 URDF 模型角 `model_joints_rad`、关节速度、
TF2 当前 TCP、期望 TCP、Servo 状态/说明、反馈年龄和停止原因。`backend=moveit_servo`；IPC 缺失、
反馈超过 100 ms、反馈非法、Servo 状态缺失/停止/未知时不发布目标并进入 `faulted`。
上述故障恢复时即使 Squeeze 一直按住也不会自动恢复输出，必须先松开再重新按下。
候选工作空间越界、MoveIt 奇异、碰撞或关节边界显示为 `constrained`；越界目标只丢弃
本帧，约束状态的后续目标继续处理，以允许移回安全区域。上游意图故障也要求先松开
Squeeze 再接管。Trigger 在 active 状态产生两态夹爪意图：Rust 快照让网页 J7 在张开
`90°` 与闭合 `0°` 之间限速移动，ROS 桥同时向仿真的 `hand_controller/joint7_left`
发送对应 `JointTrajectory`。两路都只作用于 `GenericSystem` 仿真，不打开串口、不发送
硬件夹爪命令；`simulation_only` 永远为 `true`。

现场确认的唯一默认姿态和示教启动姿态都是 J1–J7 全部 `0°`。仿真启动时六轴逻辑角、
URDF 模型角和闭合夹爪角均为零；没有单独的回零姿态、工作姿态或自动展开阶段。松开
Trigger 后，J7 才会按限速规则从闭合 `0°` 向张开 `90°` 运动。该模型角与 FL 插件的
舵机逻辑多圈量 `[-270°, 0°] / direction=-6` 独立，不做直接除法换算。

这里的“仿真启动”是 ROS `GenericSystem` 进程启动，不是网页“启动模拟”按钮。网页按钮
只切换真实/虚拟 NOLO 输入；停止后机械臂保持当前位置，不发送回零轨迹。需要重新从
J1–J7 全零开始时，重新启动 ROS 仿真进程。

`/arm-simulator/` 每 33 ms 读取一次该快照，用 `model_joints_rad` 的 J1–J6 驱动从厂家
URDF 构建的关节树，
并显示当前和目标 TCP。页面使用厂家 STL 表现本体，并按 `gripper_rad` 驱动
J7 左夹爪及 URDF mimic 右夹爪。页面只发出 `GET /api/status`，没有串口、运动执行、
零点、参数或力矩写入能力。J1–J7 行提供模型角滑块；拖动后只在当前浏览器暂停实时
模型跟随并调整 Three.js 关节，不修改后端快照。点击“恢复实时”即可重新跟随后端。
模型资产来源和固定上游提交记录在
`src/controller-viewer/public/arm-simulator/models/README.md`。

设备配置、候选 URDF、具体限制和 P1 只读探针见
[Star Arm 102-FL 接入与仿真](../../arm/docs/STAR-ARM-102-FL-INTEGRATION.md)。
ROS 启动和补丁见 [MoveIt Servo 仿真链路](../../arm/ros2/README.md)，方案边界见
[IK 与实时笛卡尔伺服选型](../../arm/docs/IK-SELECTION.md)。

内部解码、Fusion 和 `/api/status` 快照保持设备原生更新率。面向查看器的 SSE 对每个
来源限制为最高约 60 Hz，离线/恢复转换会立即发送，以免网页 JSON 序列化和网络传输
积压。机械臂控制不得使用 SSE；当前仿真使用本机 `watch` + Unix socket latest-value
边界，未来真实控制也必须保持有界语义，不能改为无界队列。

收到 Ctrl-C 或 SIGTERM 时，后端会通知现有 SSE 流结束，再退出进程并释放 USB。

网页 Three.js 固定从 jsDelivr 的 `three@0.180.0` 加载；机械臂仿真页同时使用同版本
`OrbitControls` 和 `STLLoader` 的 jsDelivr ESM 构建。工程不保存 Three.js 源码副本；
浏览器必须能访问 `cdn.jsdelivr.net`。

## 陀螺仪零偏标定

每个设备第一次成功完成的手工陀螺仪零偏会立即写入
`vr-xr/state/gyro-bias-v1.json`。后端重启或 USB 重连时读取文件，并把对应 Fusion 状态
直接恢复为“零偏已完成”。因此重复长按 Menu 只使用最后约 1.2 秒样本记录网页原点、
零姿态和人体前方，不重启 Fusion、不清空 Offset，也不重新采集零偏。

若某个设备在文件中没有有效条目，它第一次长按 Menu 时仍按原流程执行：第一秒用于
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

后端不再叠加自定义快速收敛或动态增益。快速动作后俯仰/侧倾会按照 Fusion 的标准
重力反馈自然收敛；无磁力计时 yaw 不会由加速度计纠正。

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
cargo build --release --bin nolo-unknown-probe
./target/release/nolo-unknown-probe 12
```

参数是采样秒数，默认 10 秒。探针只打印统计结果，不保存原始抓包，也不修改设备。
协议结论统一维护在 [NOLO-CV1-PROTOCOL.md](NOLO-CV1-PROTOCOL.md)。

## 测试

```bash
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
cd ../..
deno test --allow-read tests
```

测试不访问 USB；实机采样必须单独执行探针或启动后端。
