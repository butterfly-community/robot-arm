# 实施探针与验收事实

本文只记录实际观察结果，不用设计意图冒充运行事实。当前更新日期为 2026-08-28。

## 工具和工程边界

- 宿主机可用 Rust/Cargo 1.97.1、Docker 29.7.2、Docker Compose 5.5.0、Node.js
  24.12.0、pnpm 10.28.0 和 Python 3.13.5；宿主机没有 `dora`、`ros2` 或 `colcon`。
- Dora 和 ROS 2 均在构建镜像时安装，容器启动不编译代码。
- `backend/` 独立保存 Rust workspace、ROS 2 包、型号补丁和后端 Dockerfile；
  `frontend/` 独立保存 pnpm workspace、四个 Next.js 应用、共享包、入口配置和前端
  Dockerfile。`services/` 根目录只保存部署、跨端测试、记录工具和文档。
- 前端四个运行服务共用同一构建镜像，依赖只安装一次、四个应用一次构建；每个容器通过
  `WEB_APP` 选择自己的 Next.js server。

## Dora 与 Compose 生命周期

- 当前使用 Dora networked 模式：一个 coordinator，六个带不同 `machine-id` 的 daemon；
  daemon 根据同一份 `dataflow.yml` 启动节点子进程。`dataflow` 容器只提交并附着图。
- 业务节点是独立进程，daemon 容器是它们的部署边界；项目不再增加另一套消息总线、注册器、
  Unix socket 或进程管理脚本。
- 用户生命周期只有整套 `docker compose up -d` 和整套 `docker compose down`。重启就是先
  `down` 再 `up -d`，不维护单服务停止、重启或状态接续。
- Compose 对六个 Dora daemon 使用本地监听端口健康检查，`dataflow` 等全部 daemon 注册就绪后
  只提交一次图。业务节点异常不会由 `dataflow` 容器局部重启成一张残缺图；恢复仍只执行用户
  规定的整套 `down`、整套 `up -d`，没有自定义恢复状态机。
- 干净启动探针最终只出现一个 `robot-arm-services` Running dataflow；整套 `down` 后项目容器
  列表为空。连续两轮结果记录在最终验收提交中。

## OpenXR 与输入设备

- 主机只读 USB 探针看到 `0483:5750`、产品字符串 `NOLO HMD`、序列号
  `56E1634B3234`，以及 `1a86:7523` 串口设备。USB 清单与 OpenXR 输入源保持为两个来源，
  不按顺序或名称自动关联；本次由用户在页面确认 `1-8.1` 是所选 Runtime 输入端点对应的
  主机设备。
- `openxr-runtime` 是独立镜像：以本地 Monado、OpenHMD 和
  `xioTechnologies/Fusion` 源码构建，只在 Runtime 驱动层应用项目维护的 NOLO CV1 数据解析
  与 IMU 补丁。`openxr-source-node`、空间节点、消息和网页均不读取厂商协议或执行第二遍
  Fusion。
- 镜像内实际 Runtime 是 Monado 25.1.0。Monado 枚举到 Left
  `NOLO CV1: Controller 1 (OpenHMD)`、Right `NOLO CV1: Controller 0 (OpenHMD)`；OpenXR
  标准接口为左右端点都返回 `/interaction_profiles/khr/simple_controller`，两者的 grip pose
  bound source、位置和姿态均有效。
- 镜像内 libmonado API 版本为 1.7。其 role/device 接口能补充左右端点的 Runtime-native
  名称、序列号和位置/姿态能力，但没有输入 component 或 binding-profile 枚举接口；后两项
  继续来自 OpenXR interaction-profile 定义、`xrEnumerateBoundSourcesForAction` 和
  `xrGetInputSourceLocalizedName`，没有把不存在的私有能力写进节点。
- Runtime 镜像从 Monado 同一版本的 `bindings.json` 构建 40 个 profile 的页面目录；建立
  OpenXR 会话时只为标准 simple-controller 和用户明确选定的 profile 调用 suggest。早期逐个
  探测全部 profile 会产生未启用扩展和空绑定错误，已经删除；最终运行日志没有这些探测错误。
- 2026-08-27 的现场探针确认 Runtime 能枚举两个真实输入端点并连续输出约 `100 Hz`；观察到
  `received_frames=11196`、`sequence_gaps=0`、`runtime_error_count=0`，位置与姿态的
  valid/tracked 标志均为 true。用户随后确认当次准备复测移动时设备实际处于关机状态，因此
  这组事实只证明 USB → patched OpenHMD → Monado → OpenXR → Dora → 网关能取得有效静态数据；
  实体移动导致位姿变化仍列为后续现场验收，不冒充完成。
- 无渲染采集会话使用 `XR_MND_headless`，Action 通过 `xrSyncActions` 更新，姿态定位时间由
  `XR_KHR_convert_timespec_time` 取得。早期实现错误调用渲染用的 `xrWaitFrame/xrBeginFrame`
  并触发 `XR_ERROR_CALL_ORDER_INVALID`；移除无关渲染帧循环后错误计数保持为 0。
- 后续空间语义回归使用 `tests/fixtures/openxr-synthetic-cycle.json` 的确定性合成样本，覆盖
  六向 2 cm、两种圆弧正反 8° 和返回基准。合成数据不读取真实设备零偏，也不替代上述一次
  真机采集证据。
- 页面保留旧版模拟数据入口。`openxr-source-node` 直接生成 100 Hz 确定性输入：启动抬升
  10 cm，随后每段 3 秒依次执行上/下 5 cm、左/右 2 cm、前/后 5 cm、左旋/右旋 8°、前部
  抬起/往下 8°；动作经过正常空间、MoveIt 和 execution 链路。停止只发布 inactive 输入，
  不发送起始位或夹爪命令。单元、Compose 闭环和 Playwright 均覆盖该入口。
- 现场应用 boolean 和 boolean-pair 绑定时，早期实现会在重建 OpenXR 会话后触发
  `openxr-source-node` SIGSEGV。根因是 Rust 结构字段按声明顺序析构，而父级 Instance/ActionSet
  先于 Session/Space/Action 子句柄析构。字段改为明确的子句柄到父句柄顺序后，同一 Runtime
  连续完成 boolean 绑定、两按钮方向绑定和恢复空绑定；bound source、本地化名称和 active
  均来自 OpenXR，节点及其余容器 restart count 都保持 0。Compose 集成测试保留这条实际会话
  重建回归，不用 mock 替代。

## 串口与执行节点

- `/dev/ttyUSB0` 在主机和 execution 容器内均可见；Compose 使用 `/dev`、只读 udev/sysfs 和
  `c 188:* rwm`，没有 host network。
- 新 execution 链路已做一次只读真机连接：Ping ID 0–6、读取 63 个参数值、Monitor 和主动断开
  全部完成，反馈来源切到 `hardware`；连接过程没有发送位置命令。
- 同一新链路随后执行单关节 `+5° → 返回本轮基准`：J1、J2、J4、J5、J6 的 MoveIt 结果均为
  success/code 1，UART 无错误，Monitor 的被测关节方向与命令同号；J3 的正向 `+5°` 在规划阶段
  原样返回 `99999（未检测到起始自碰撞）`，因此没有发送该步串口命令，也没有自动重试或换向。
  J4 返回后 Monitor 停在约 `1.8°`，这是此前已决定暂不处理的实体到位偏差，节点没有用软件
  门限掩盖它。
- 夹爪完成 `+5° → 返回` 请求，ID 6 映射和传输均无串口错误；Monitor 只从约 `0.9°` 到
  `1.5°` 后回到约 `0.9°`，实测未达到命令目标，同样按原始反馈记录而未加入补偿或成功门限。
- 测试后主动断开，保留最后一帧 `hardware` 反馈；没有自动发送起始位或夹爪命令。
- 真机连接不比较软件位置、实测位置或起始位，也没有角度差门限。J1–J6 在 UART 边界同号；
  只有夹爪按已验证的机械传动方向换算。
- 厂家 Python SDK 1.3.12 与 Rust 协议实现的双向伪串口交叉测试已覆盖 Ping、Monitor、同步位置、
  内部参数、分片、噪声、损坏帧恢复和舍入；最终收敛阶段重复三轮。
- 一次真机复测首次到达测试位后，J1 请求在执行前被 MoveIt 默认
  `trajectory_execution.allowed_start_tolerance=0.01 rad` 以 J2 实时抖动为由返回
  `-4 / CONTROL_FAILED`，execution 没有收到峰值命令。MoveIt 源码明确规定该参数为 `0` 时跳过
  验证；型号 motion launch 现设为 `0.0`，撤销平台默认门限。两轮运行态探针均读回 `0.0`，没有
  用另一个容差、重试状态机或执行旁路替代。
- 来源切换审查随后发现 ros2_control 的 command interface 仍保留切换前的软件值：若直接激活
  controller，会把旧值作为保持命令写回真机。motion 节点现在等待 arm/hand action server
  就绪后，在首次反馈和 `software → hardware` 真机接入时停用并重新启用两个 controller；同步
  期间持续以最新反馈更新控制器基准，且只有相对控制、普通轨迹或末端执行器请求才允许输出。
  `hardware → software` 只发生在断开后的首条命令回显，继续当前用户轨迹而不重复同步。运行态逐项读回
  两个 controller 的 `set_last_command_interface_value_as_state_on_activation=false`，没有增加
  姿态差或时间门限。
- 第一次验证上述输出门控时，串口连接已保持命令序号不变，但 execution 把包含 63 项参数的完整
  `transport_state` 与 Monitor 一起按 10 ms 发布，网关离散请求被无界重复状态排在后面，最终
  出现 HTTP 504。修正后 Monitor 仍按 10 ms 读取，只在协议 0.1° 整数反馈发生精确变化时发布
  `ArmState`；串口发现和完整传输状态按已有 1 s 扫描节拍发布，网关将其声明为 latest-value。
  这是重复消除和消息调度修正，没有增加角度阈值、重试次数或运动限制。
- 最终复测连接前后的最后命令 sequence 都是 `303`；连接只执行 Ping、63 项参数和 Monitor，等待
  3 秒也没有产生保持、起始位或夹爪命令。首帧真机反馈为
  `[0.3,-0.1,-18.6,1.2,0.4,-0.1]°`，J3 没有跳向软件初始值 `-3°`。
- 普通 `MotionRequest` 到 `[0,0,-20,0,0,0]°` 返回 success/code 1、2 个轨迹点、规划时长
  `0.057078519 s`；Monitor 为 `[0.3,-0.1,-18.6,1.2,0.4,-0.1]°`。J1 `+10°` 返回
  success/code 1、3 个轨迹点、`0.150242887 s`，下发命令严格为
  `[10,0,-20,0,0,0]°`，采样峰值 J1 `10.0°`、末帧 `9.7°`。返回测试位同样为
  success/code 1、3 个轨迹点、`0.150242887 s`，最终 Monitor
  `[0.3,-0.1,-18.6,1.2,0.4,-0.1]°`；三步 UART 原始错误均为空，连接、运动和断开均未再出现
  504 或 `Operation timed out`。
- 最终主动断开后运行状态为 `connected=false`，保留最后一帧 `hardware` 反馈，没有自动发送
  起始位、夹爪或保持命令。
- 真机复测后的软件闭环首次暴露一个来源切换问题：断开后保留的最后反馈仍标为 `hardware`，
  下一条用户命令产生的首帧 `software` 回显被 motion 节点误当成新的外部状态，导致 controller
  再次同步并截断正在执行的轨迹；MoveIt 虽返回成功，J1 实际只到约 `0.72°`。来源同步现只在
  首帧反馈和新取得 `hardware` 反馈时发生，`hardware → software` 继续当前用户轨迹。对应单元
  测试覆盖三个来源转换，完整 Compose 软件闭环再次验证 J1 `+5°` 收敛到目标；没有加入角度
  容差、延迟或重试分支。

## 纯软件闭环

- 四个页面、四组 API、四个 WebSocket 首帧、模型 manifest、URDF/mesh 读取均通过 Compose
  内部入口验证；外部只绑定 `192.168.100.10:8765`。
- 软件模式下按运行时 `RobotModelInfo` 提交 J1 `+5°`，MoveIt 规划与执行成功，唯一
  `ArmState` 收敛到同一目标；夹爪 `+5°` 走同一 ROS controller output → Dora → execution
  链路；普通 `start` 目标只恢复 J1–J6 并保持夹爪值。
- ROS 的 arm 与 hand controller 会把未控制的接口发布为 NaN。motion 节点首次输出以最新权威
  `ArmState` 补齐，随后继承上一条控制目标中另一控制器负责的字段；这避免交错到达的手臂末帧
  用尚在插值中的反馈覆盖夹爪终点。不补零、不增加角度门限；该竞态已有 Python 单元测试和
  完整 Compose 闭环测试。
- 冷启动时 hand controller 晚于 arm controller 可用。`MotionState.service.has_output` 现在取
  ROS 客户端/订阅者的实际就绪状态；尚未就绪的 actuator 请求直接返回
  `controller_unavailable`，不再虚报成功或维护等待队列。
- 2026-08-28 的浏览器回归发现显式请求错误会被下一帧 WebSocket 快照立即清除。前端现将
  状态连接错误与用户操作错误分开保存；实时快照只能清除连接错误，串口字段等原始操作错误
  保持可见。另一次局部 Web 容器重建证明入口代理会保留旧容器地址，因此验收和用户生命周期
  统一只用完整 `down`/`up -d`，不再把局部重建当成受支持流程。
- 浏览器级 Action 绑定验收进一步发现共享 `Field` 把多个控件放进同一 `<label>`：其中一个
  select 被禁用时，同组 component option 也被浏览器判为不可操作。共享组件改为语义正确的
  `<fieldset>/<legend>` 后，真实 Runtime component 能从页面选择、应用、回显 bound source
  并恢复原配置；四个页面仍共用同一组件。

## 方法级依赖审查

本审查的“库”不局限于标准库：逐个方法同时检查标准库、质量和测试情况合格的 npm 包、PyPI
包、Rust crates 以及已采用框架的现成功能。只有不存在满足当前契约的成熟实现时才保留自写逻辑。

- Python motion core 原先自写四元数乘法、归一化、共轭、向量旋转和轴角转换。镜像加入经过
  自身测试的 `transforms3d 0.4.2` 后，通用数学全部改用 `axangle2quat`、`qmult` 和
  `quat2mat`；项目只保留 StarArm 安装基准与 TCP 圆弧公式，原有圆弧测试逐项通过。
- Rust 空间变换已经使用 `nalgebra`；OpenXR 会话/Action 使用 `openxr`；串口打开使用
  `serialport`；HTTP/WebSocket 使用 Axum/Tokio；配置和契约使用 Serde；三维页面使用 Three.js
  和 URDF Loader；共享样式组合使用 shadcn/ui 采用的 Radix Slot、CVA、clsx 和
  tailwind-merge。没有保留同职责的自写基础设施。
- 实测核对 `dora-arrow-convert 1.0.0-rc.5`：它只直接转换标量和基础类型数组，不能表达本项目
  的嵌套动态契约、显式 schema version、request result，也不能直接供 Python 节点使用。因此
  当前 `robot-arm-messages` 的两段 Arrow Struct + Serde JSON 编解码仍是必要的跨语言边界，
  不是重复实现 Dora 已有结构体转换。
- FashionStar crate 的帧头、长度、校验、Monitor、内部参数和同步位置格式是厂商 UART 协议本身；
  `serialport` 不提供这些业务帧，厂家 Python SDK 也不能作为 Rust 生产依赖。该实现继续由厂家
  SDK 双向交叉测试约束。服务 readiness、配置归属、动态元数据和请求状态都是本项目契约逻辑，
  通用库不能在不复制职责的情况下替代。
- 空间核心原有手写三维向量相减和矩阵乘法已改为现有 `nalgebra::Vector3/Matrix3`；
  `cargo-machete 0.9.2` 没有发现未使用的 Rust 依赖，`jscpd 5.0.16` 没有发现前端生产 TypeScript
  重复片段。各节点的四行系统时间读取仍直接使用标准库；提取包装不会减少业务实现，反而会
  增加跨节点耦合。
- 页面运行在 `http://192.168.100.10`，不属于浏览器 secure context，直接使用
  `crypto.randomUUID()` 会在真实页面抛错。请求 ID 现在使用经过测试的 npm
  `nanoid/non-secure 5.1.16`；此前自写的伪 UUID fallback 已删除。类名合并和 WebSocket 传输也
  分别使用已有库或浏览器 API，没有引入第二套工具。

## 依赖公告审查

- `pnpm audit --audit-level high` 和针对镜像中 `dora-rs 1.0.0rc5`、`pyarrow 25.0.1`、
  `transforms3d 0.4.2` 的 `pip-audit 2.9.0` 均无已知漏洞。审查中把 Next.js 升至 `15.5.24`、
  Vitest 升至 `3.2.7`、PostCSS 锁定 `8.5.26`，并把 nanoid 从存在公告的 `5.1.6` 升至
  `5.1.16`；升级后重新完成两轮前端构建和浏览器测试。
- `cargo-audit 0.22.2` 报告 Dora 固定的 Zenoh 1.9.0 间接依赖 `lz4_flex 0.10.0`
  （RUSTSEC-2026-0041）。实际 Cargo feature 图没有 `transport_compression`，Zenoh 的压缩和
  解压调用均受该 feature 的 `cfg` 控制，因此漏洞函数没有编译进本项目路径。Dora rc5 要求
  Zenoh `~1.9`，无法直接升级到已经换用新版 lz4 的 Zenoh；为不可达代码 fork 整个 Zenoh 会
  增加维护面，本轮不制造该分支。其余 bincode、paste、rustls-pemfile 是同一 Dora/Zenoh 链的
  transitive unmaintained 警告，没有项目直接依赖。

## 现场范围边界

- OpenXR 实时采集、端点选用和 simple-controller profile 已通过真实设备验证。按用户决定，
  真实数据链路确认正常后，空间动作样本由确定性生成器构造；本轮不再把实体按键移动样本或 USB
  热插拔扩张成完成条件。生成样本覆盖原有全部六向平移、俯仰和水平圆弧，并经过正常服务链路。
- 真机运动测试不要求速度、加速度、起始位、姿态差、Servo 持续时间或其他前置条件，唯一额外
  操作范围是每次运动不超过 `10°`。该范围只用于验收操作，不实现为生产运行时门限。

## 2026-08-28 收敛记录

- 第一批全量闭环暴露三项真实问题：HTTP 页面操作错误被实时快照清除；局部重建 Web 容器后
  入口代理仍持有旧地址；模拟输入刚结束就紧接普通规划会混合两个独立验收场景。修正分别是
  分离连接/操作错误、生命周期只使用整套 `down/up`、把模拟链路放在软件闭环末尾。没有增加
  运行时门限、等待状态机或重试次数。
- 普通运动取消审查发现取消请求能返回、被取消原请求却会悬挂。motion 节点现在用各自 request
  ID 同时发布原请求 `cancelled` 结果和取消请求确认；集成测试实际并发验证
  `planning → executing → succeeded` 以及另一路 `planning/executing → cancelled`。
- ROS hand controller 的最终软件反馈可能与请求值相差约 `10^-7 rad`。逐位相等的测试断言已
  删除，生产代码没有因此增加成功门限、补偿或重试；起始位保持夹爪的验收直接检查夹爪没有
  被重置回请求前值。
- 最终代码连续两轮都从容器列表为空的 `docker compose down` 状态执行全镜像构建和
  `up -d`，随后通过 43 个 Rust 测试、严格 Clippy、14 个 Python/MoveIt 测试、MoveIt 运行参数
  探针、厂家 SDK 双向 `64 × 2` 轮交叉测试、Ruff、前端 Prettier/ESLint/TypeScript/Vitest、四个
  Next.js 生产构建、npm 漏洞审计、Compose 软件闭环和浏览器回归。输入端点在线时 16 个
  Playwright 测试全通过；设备休眠、Monado 明确只报告 `Simulated HMD` 时，实体 Runtime
  绑定 1 项按实际前置条件跳过，纯软件模拟测试仍始终执行。两轮之间没有生产代码修改；
  最后才执行上述真机复测。结束时 14 个服务容器均运行、业务 readiness 为 true、restart count
  为 0，模拟输入 inactive，串口已主动断开。
- 两轮最终软件验收后又按同一路径执行一次真机终验。连接前后最后命令 sequence 均为 `194`，
  3 秒只读阶段得到 63 项参数和 `[0.3,-0.1,-18.6,1.2,0.4,-0.1]°`，没有自动位置命令。测试位、
  J1 `+10°`、返回三次 MoveIt 结果均为 success/code 1，分别为 2/3/3 个轨迹点，规划时长
  `0.057078519/0.150242887/0.120116840 s`，采样到的 J1 命令峰值为 `10.0°`。动作结果返回时
  Monitor 分别为 `0.3°/5.6°/5.5°`，随后仍继续趋近，主动断开时最后反馈为
  `[0.3,-0.1,-18.6,1.2,0.0,-0.1]°`；UART 原始错误始终为空。这保留了实体响应滞后和不到位的
  原始事实，没有新增反馈成功门限、补偿或自动重试。
