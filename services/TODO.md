# Dora 服务化迁移 TODO

本文是 `services/` 新工程的权威实施和验收清单。用户已在 2026-08-27 明确授权实施、构建、
启动服务以及第 10 节的真实设备操作。当前处于实现与循环验收阶段，必须先完成相应纯软件
验收，再按第 10 节单步复测。

状态更新（2026-08-28）：新工程的生产链路已实现，当前正在执行最终循环验收。勾选项必须有
代码、自动化或 `docs/PROBES.md` 中的现场证据；合成样本只能证明确定性软件链路，不能冒充
实体输入或真机观察。当前范围中的未勾选项全部收敛后才提交最终实现。

`../ref/` 是迁移前实现的只读参考，用于核对已经验证过的算法、协议、页面行为和测试；新工程
不得从 `ref/` 建立构建、运行或代码依赖。全部工作遵守仓库根目录 `AGENTS.md`，尤其禁止擅自
增加门限、保护、限制、拒绝条件或复杂状态机。

## 1. 已确认范围

### 1.1 已完成的目录调整

- [x] 新建与 `ref/` 同级的 `services/`。
- [x] 将迁移前的 `arm/`、`vr-xr/`、Compose 和 Docker 构建上下文移入 `ref/`。
- [x] 根目录保留 `.git`、`.gitignore`、`AGENTS.md`、`.agents/` 和 `.codex/`。

### 1.2 不再讨论的架构决定

- 所有输入设备只通过 Monado/OpenXR 接入；应用层不按厂商、型号或设备组合分支。
- 采集服务分别发布主机硬件清单、OpenXR Runtime/system 信息、Runtime 输入端点、原始位姿和
  设备无关的功能 Action；用户选用实际输入端点后，服务才分配会话内 `source_id`。
- `openxr-source-node` 按当前 interaction profile 让用户把功能 Action 绑定到具体 component，
  并发布语义化 Action 值；通用代码不出现设备型号或实体按键名。
- 原始位姿和功能 Action 是两个独立输出；采集节点不做空间转换、倍率、动作分量开关、机械臂
  运动学或 Fusion。
- `spatial-transform-node` 是唯一的空间坐标、相对基准和姿态语义转换位置。
- `stararm-102-motion-node` 是唯一的 MoveIt/IK、规划、Servo 和轨迹插值位置。
- 软件模拟与 UART 真机统一在 `stararm-102-execution-node`，不再拆两个 executor。
- 全系统只有一个权威 `ArmState`，由 `stararm-102-execution-node` 发布。
- 是否选择并连接串口只是运行时配置，不是启动模式；未连接就是纯软件反馈。
- 起始位只是普通关节目标，不建立专用回零路径或反馈角度门限。
- 网页拆为四个独立 Next.js 服务，不再提供统一控制台。
- `web-entry` 是职责名，具体反向代理产品可替换，不进入其他服务的接口或逻辑。
- 所有生产服务只由 `services/compose.yaml` 构建和管理，不恢复管理脚本或 systemd。
- 除 `stararm-102-motion-node` 和 `stararm-102-execution-node` 外，消息、Dora 节点、网关和
  Web 服务均不得写死机械臂型号、关节数量、关节名、起始位、末端执行器数量、舵机 ID、模型
  范围或串口协议参数；它们只消费两个型号节点发布的元数据和动态数组。

### 1.3 功能完整性和自主收益评估

TODO 不是封闭需求列表。实现时既不能因为某项没有写到就默认删除，也不能看到可能有用的功能
就无边界扩张。每个阶段必须执行下面的盘点：

1. [x] 读取对应的 `ref/` 源码、页面、文档和测试，列出所有用户可见操作、状态、错误、配置、
       自动恢复行为和诊断信息。
2. [x] 为每项旧能力记录新归属：`原样迁移`、`由新架构替代`、`明确删除` 或 `后续`；后两种必须
       写出理由，不能以“新 TODO 没写”为理由。
3. [x] 检查新架构自然产生的高收益功能，包括设备发现、有效配置回显、状态可观测性、记录回放、
       整套服务确定性重启和错误定位；有真实调用方、能直接验收且不制造重复职责的功能应加入实现。
4. [x] 对候选功能说明用户收益、归属服务、数据来源、输出、失败表现、测试方法和新增复杂度；
       无法说明其中任一项的功能不得直接进入代码。
5. [x] 每轮实现后重新盘点调用方和页面流程，发现遗漏立即补回 TODO、实现和测试，而不是等到
       最终验收才处理。

### 1.4 循环验收和停止条件

- [x] 每一轮都完整执行“盘点 → 复现或新增测试 → 实现/修正 → 相关测试 → 全量回归 → 复杂度、
      重复、无意义代码、最佳实践、迁移遗留和文档审查”。
- [x] 任一测试或审查发现新问题，都必须开始下一轮；下一轮仍执行全量回归，不能只跑最后失败项。
- [x] 不设置最大轮数。只有所有当前可执行验收通过，并且连续两轮从干净状态开始的全量测试与
      全面审查都没有产生新的修改，才视为当前阶段收敛。
- [x] 最终状态还要单独进行一轮只读复核，逐项检查本文、`AGENTS.md`、功能迁移表、实际服务、
      页面和测试证据；复核发现问题则重新进入循环。
- [x] 真机项目按第 10 节在纯软件阶段通过后执行；尚未执行的项目不能伪装成已通过，也不阻止
      已明确划分的纯软件阶段收敛。

### 1.5 TODO 条目的可实施标准

本文中的每个功能条目都必须让实现者不依赖猜测回答下面六个问题；不能回答的条目先补充计划，
不得直接编码：

1. [x] **数据从哪里来**：写明系统调用、Dora 消息、ROS 接口、配置文件或用户操作，不能只写
       “读取设备”“处理状态”。
2. [x] **用户做什么**：写明页面上能看到的字段、按钮或选择项，以及操作发生在哪个页面，不能
       让用户填写内部 ID 或理解 Runtime 内部结构。
3. [x] **服务做什么**：写明所属服务、状态变化、持久化位置和是否会产生机械臂命令，不能只写
       “支持”“适配”“同步”。
4. [x] **输出是什么**：写明消息类型、关键字段、单位和权威状态来源，不能用一个笼统的
       `status` 同时表示配置、命令和反馈。
5. [x] **失败时发生什么**：保留第三方原始错误并说明页面状态；不得自行发明重试次数、超时、
       姿态差值或其他门限。
6. [x] **怎样证明完成**：至少有一条可重复执行的测试，核对输入、输出和用户可见结果；“能用”
       “正常显示”“完成适配”不算验收证据。

跨服务功能还必须在第 6.6 节写成完整用户流程。协议字段和内部结构只有在被某个用户流程、
服务输入输出或自动化测试实际使用时才保留。

### 1.6 必须由探针决定、当前不得假定的事项

| 未知项 | 探针要回答的问题 | 决策写回位置 |
| --- | --- | --- |
| Dora 与 Compose 进程边界 | 谁启动节点、整套 `down/up` 后是否有残留或重复 dataflow | 第 3、7 节和 `compose.yaml` |
| 当前 Monado 可见设备 | 主机发现的设备中，哪些实际成为 OpenXR system/input source，各自有什么 profile、Action 和 pose | 第 5.1、6.1 节及输入 fixture |
| 主机硬件与 Runtime 输入源的关系 | Runtime/Monado 是否给出可验证的 serial 或明确关系；没有时页面只能并列还是需要用户人工关联 | 第 4.1、6.1 节 |
| OpenXR session 内重新应用绑定 | 当前依赖版本能否直接更新；若必须重建，实际丢失和恢复哪些状态 | 第 6.1 节及绑定测试 |
| Monado 容器访问方式 | 所需设备节点、udev/sysfs、Runtime JSON、IPC socket/volume 和权限分别是什么 | 第 7 节和 Compose 探针记录 |
| 圆弧使用的安装基准几何 | `ref/` 中当前参数来自哪个模型/实测，网页模型、TF 和 MoveIt 是否一致 | 第 6.3 节和圆弧 fixture |

探针只记录事实，不顺便实现业务功能。结论与当前计划不一致时先修改 TODO 并由用户核对，再
进入依赖该结论的阶段。

### 1.7 已解决的迁移冲突

- [x] 旧链路的绝对位置 `0.5` 缩放迁到 `spatial-transform-node`，字段为可配置
      `translation_scale`，初始值 `0.5`；`openxr-source-node` 始终原样发布 Runtime 位姿，任何
      其他节点不得再次缩放。
- [x] 根 `AGENTS.md` 已由用户清理；当前只保留通用协作约束，不再迁移具体设备、实体按键或
      长按标定规则。
- [x] 功能 Action 与原始按键/轴 component 的对应关系归 `openxr-source-node`；三个空间动作
      分量开关归 `spatial-transform-node`。两类配置分别由所属节点保存和回显，前端不保存副本。
- [x] J4 方向已在 StarArm URDF 中修正，现有驱动也已改为 J1–J6 同号传递；新节点不得
      在 UART、Dora、ROS 或网页中再对 J4 取反或补偿。

## 2. 目标数据流

```text
Monado/OpenXR
  └─ openxr-source-node
       ├─ absolute_pose ─┐
       ├─ control_input ─┴─> spatial-transform-node ─> RelativeToolMotion ─┐
       └─ discovery/bindings ───────────────────────────────────────────┐ │
                                                                       │ v
ArmState ───────────────────────────────────> stararm-102-motion-node   │
       ^                                                       │        │
       │                                                       v        │
       │                                                  ArmCommand    │
       │                                                       │        │
       └────────────────────── stararm-102-execution-node <────┘        │
                                ├─ 未连接执行端点：软件反馈              │
                                └─ 已连接执行端点：真机反馈              │
                                                                        │
各业务节点的请求/状态（包括上面的 discovery/bindings）<─────────────────┘

各业务节点 <─ Dora ─> web-gateway-node <─ HTTP/WebSocket ─> 四个 Web 服务
                                                               ^
                                                               │
                                  web-entry：唯一入口和路径转发 ─┘
```

规则：

- 数据只沿上述职责边界流动，不增加跨层旁路。
- 命令目标、软件反馈和真机实测反馈保持可区分，不复用字段冒充彼此。
- 同一坐标变换只在空间节点应用一次；关节运动学方向只由型号节点加载的 URDF 表达，
  execution 节点不再改关节符号；硬件编码只在对应 execution 节点处理一次。
- 高频流使用 latest-value 语义；序号和时间只用于排序、记录和回放，不派生超时门限。

## 3. 服务清单与边界

| 逻辑服务/节点 | 职责 | 明确不负责 |
| --- | --- | --- |
| `openxr-runtime` | 在容器内提供 Monado/OpenXR Runtime 和设备驱动 | 业务 Action、空间转换 |
| `openxr-source-node` | 读取主机硬件清单和 Runtime 输入源，处理设备无关的功能 Action 绑定，发布原始绝对位姿与 Action 值 | 空间转换、倍率、动作分量开关、机械臂语义 |
| `spatial-transform-node` | 处理原点、倍率、相对基准、三个动作分量开关、空间与姿态转换 | OpenXR component 绑定、机械臂型号、关节、ROS、URDF、IK、UART |
| `stararm-102-motion-node` | MoveIt、Servo、关节规划、轨迹插值 | 输入设备、串口、页面状态 |
| `stararm-102-execution-node` | 软件反馈、UART、Monitor、唯一 `ArmState` | IK、规划、空间转换 |
| `web-gateway-node` | Dora 与 HTTP/WebSocket 协议适配 | 页面托管、运动计算、状态所有权 |
| `web-tracking` | 展示主机/Runtime 信息，选用输入源，编辑并核对 Action 绑定，查看原始输入 | 空间转换和机械臂控制 |
| `web-spatial` | 确认位置原点，编辑轴映射/输入倍率/三个分量开关，查看转换结果 | 设备枚举、IK 和硬件传输 |
| `web-motion` | 按 motion 元数据动态渲染关节、命名目标、执行选项和后端诊断，提交/取消目标 | 固定轴数、固定规划后端、输入映射、硬件传输和三维反馈模型 |
| `web-arm-execution` | 按执行元数据动态渲染模型、反馈、末端执行器、连接方式和参数 | 固定轴数、输入采集、坐标转换和 IK |
| `web-entry` | 唯一外部端口、路径和 WebSocket 转发 | 状态、鉴权、业务逻辑 |

`stararm-102-motion-node` 是 Dora 节点和逻辑服务名，ROS 2 包名使用
`stararm_102_motion_node`；最终容器是否与 Dora 节点一一对应由第 7 节部署探针确认，名称不能
掩盖真实进程边界。机械臂相关两个节点明确绑定 StarArm-102 构型，其余通用节点和网页契约
不得写死输入设备或机械臂的厂商、型号、固定编号、轴数、关节名、执行器名或实体按键名称。

设备无关的含义是：通用服务可以显示运行时元数据中的 `StarArm-102`、J1–J6、夹爪、串口和
舵机参数，但源码、类型、页面布局和测试不能据此假定六轴或一个夹爪。把执行节点换成另一种
关节数量、命名目标、末端执行器或传输方式的实现时，通用服务无需修改或重新编译。

### 3.1 已知旧能力迁移表

这张表是初始盘点，不是完整性的上限。实施过程中发现其他现有能力时必须立即补表。

| 旧能力 | 新归属 | 必须保留或替代的结果 |
| --- | --- | --- |
| 输入设备状态、绝对位置和姿态 | `openxr-source-node` | 改用 OpenXR 原始位姿、跟踪标志和时间，不再自行 Fusion |
| 实体设备选择 | `openxr-source-node` + `web-tracking` | 分别展示主机硬件、Runtime system 和 Runtime 输入源；用户选用 Runtime 输入源，不直接填写抽象 ID，也不把“选用”冒充 USB 打开操作 |
| 功能 Action 与按键/轴映射 | `openxr-source-node` + `web-tracking` | 按当前 interaction profile 配置，显示 component path、类型、Runtime bound source 和实时语义值；功能名不包含设备或机械臂型号 |
| 旧 IMU Fusion、零偏和长按标定 | 删除/由 OpenXR 替代 | 使用 Runtime 姿态；只保留一次触发的位置原点确认，不迁移旧 Fusion 状态机 |
| 旧虚拟输入模拟 | `openxr-source-node` + `web-tracking` | 保留页面启动/停止和确定性动作循环；模拟帧走与实体输入相同的空间、运动、执行链路，停止只结束输入，不联动起始位或夹爪 |
| 采样率、抖动、序号和断流诊断 | `openxr-source-node` + `web-tracking` | 从时间和序号显示观察值，只用于诊断，不生成控制门限 |
| 原点和接管基准 | `spatial-transform-node` | 原点一次触发确认；每轮接管重新建立位置与姿态基准 |
| 三个空间动作分量开关和 `0.5` 平移缩放 | `spatial-transform-node` + `web-spatial` | 空间节点保存并实时显示；缩放只应用一次，刷新页面不丢失 |
| 手柄/手动控制模式 | `stararm-102-motion-node` + `web-motion` | 切换本身不运动；切回相对控制时从最新反馈重建基准 |
| 连续末端控制 | `stararm-102-motion-node` | MoveIt Servo 继续输出到同一个手臂控制器 |
| 普通关节运动和取消 | `stararm-102-motion-node` | 型号节点发布关节元数据；通用请求按模型 revision 和关节 key 提交，保留 request ID 与完整结果 |
| 起始位 | `stararm-102-motion-node` | 型号节点发布命名目标 `start`，网页按名称提交普通关节请求，不写死关节数值或专用路径 |
| 起始自碰撞规划重试 | `stararm-102-motion-node` | 只把本次检测到的接触对放入本次 ACM，重试同一个普通规划请求 |
| Servo 与普通轨迹协调 | `stararm-102-motion-node` | 普通轨迹期间暂停 Servo 写同一控制器，结束后从最新反馈重建基准 |
| Servo 状态码和停止原因 | `stararm-102-motion-node` + `web-motion` | 原样显示 MoveIt 状态；减速、停止和无新命令状态不混淆、不锁存 |
| Servo code 6 时冻结异常控制器输出 | `stararm-102-motion-node` | 保留用户已要求的“关节限位时停在最新反馈、释放接管后重新接管”的行为；只依据 MoveIt code 6，不新增角度或次数门限 |
| 厂家模型、项目补丁和模型生成 | motion 镜像 + `web-arm-execution` 镜像 | 构建时从同一厂家版本生成 MoveIt 与网页模型，不保存两份模型参数 |
| `base_link -> link6`、TRAC-IK 和动力学配置 | `stararm-102-motion-node` | 保留已验证配置和 `30 rad/s²`，不创建第二个 TCP 或 IK 路径 |
| 100 Hz ROS 控制循环与旧 10 ms IPC tick | `stararm-102-motion-node` | 保留 controller manager 100 Hz；由 ROS 控制循环产生 Dora 命令，删除 Rust 侧重复 10 ms 调度器 |
| 独立末端执行器绝对位置请求 | `stararm-102-motion-node` | 型号节点声明 `gripper` actuator 并交给 `hand_controller`；通用契约不假定只有一个夹爪或六个手臂关节 |
| 软件/真机统一执行 | `stararm-102-execution-node` | 同一 `ArmCommand`、同一 `ArmState`，仅反馈来源不同 |
| 串口发现、连接、断开和重连 | `stararm-102-execution-node` + `web-arm-execution` | 执行节点发布传输表单和状态元数据，通用页面据此显示真实设备信息和错误 |
| ID 0–6 Monitor 与内部参数 | `stararm-102-execution-node` + `web-arm-execution` | 具体 ID 和字段只由执行节点产生，通用页面按动态标签/值显示，参数读取失败不伪装成连接失败 |
| J4 方向与夹爪硬件换算 | 两个 StarArm 型号节点 | J4 方向只由已修正 URDF 表达，命令和反馈端到端同号；只有夹爪传动/编码在 execution 边界换算 |
| 三维机械臂、反馈和目标预览 | `web-arm-execution` | 反馈模型与目标预览明确区分，模型颜色和观察位置保持 |
| 动态关节与末端执行器滑块 | 两个型号节点 + 对应 Web | 页面按元数据生成，从最新反馈初始化；拖动预览，松开提交；数量、名称、范围均不写死 |
| 关节标签和实测执行器参数 | `stararm-102-execution-node` + `web-arm-execution` | 页面按节点发布的标签和字段渲染；参数来自本次真机读取，不写死字段或 ID |
| 记录和重复回放 | Dora 记录工具 + 测试夹具 | 可复现输入、空间输出、运动命令和反馈，用于跨轮回归 |
| 示教点、轨迹保存和回放 | 后续功能 | 当前先保证消息含模型/配置版本、时间和坐标语义；本轮不增加示教业务或页面，但不得封死扩展 |
| 真机未到位、抖动和 PID 调整 | 后续现场工作 | 保留 Monitor 与参数读取、记录问题；本轮不自动改 PID、死区或控制参数 |
| 旧并排测试页和来源切换页 | 四个 Web 服务 + 共享导航 | 各页面独立显示本服务状态；跨链路验收由自动化测试和共享导航完成，不复制旧统一控制台 |
| 浏览器运行时 CDN 脚本 | 各 Next.js 镜像 | Three.js、URDF loader 等固定为 workspace 依赖并在镜像构建时打包；运行页面不依赖公网 CDN |
| 旧直接 USB 独占和 USB reset 流程 | `openxr-runtime` + Compose | 设备驱动与恢复归 Runtime；采集节点只显示 Runtime 原始错误，不保留第二条直读或复位协议 |
| Unix Socket、HTTP/SSE 单体桥接 | Dora + `web-gateway-node` | 迁移完成后删除旧 IPC 和单体状态聚合，不保留兼容旁路 |
| 服务状态和错误诊断 | 每个业务服务 + 对应网页 | 显示真实来源、当前配置、最近请求和原始错误，不生成猜测性限制 |
| 旧文档中的具体设备与实体按键约束 | 已清理 + 新配置 | 根 `AGENTS.md` 已清理；新代码只保存设备无关功能 Action 与用户绑定，不恢复旧名称 |

## 4. 工程目录

```text
services/
├── compose.yaml
├── dataflow.yml
├── backend/
│   ├── Cargo.toml
│   ├── crates/
│   ├── nodes/
│   ├── ros2/stararm_102_motion_node/
│   ├── patches/
│   └── Dockerfile.*
├── frontend/
│   ├── package.json
│   ├── Dockerfile.web
│   └── web/
│       ├── apps/
│       │   ├── tracking/
│       │   ├── spatial/
│       │   ├── motion/
│       │   └── arm-execution/
│       ├── entry/
│       └── packages/
├── tests/
│   ├── fixtures/
│   ├── integration/
│   └── replay/
├── tools/
└── docs/
```

- [x] 建立 Cargo workspace、Dora dataflow、自定义 Arrow 类型和消息 crate。
- [x] 每个 Rust 节点是独立二进制 crate；共享 Rust crate 只保存契约和无业务含义的编解码。
- [x] MoveIt ROS 2 包可以使用 Python/rclpy，但对外只暴露 Dora 输入输出。
- [x] Web 使用 pnpm workspace、Next.js、TypeScript、App Router 和 shadcn/ui。
- [x] 暂不增加 Turborepo；只有出现第二个真实调用方才提取新的共享包。
- [x] `config/` 只保存用户选择、Action 绑定、坐标映射和机械臂模型引用等实际运行配置；每项
      配置写明所有者、单位、默认来源和修改后如何生效。
- [x] `tests/fixtures` 保存合成数据，`tests/replay` 保存经过脱敏且有来源说明的记录数据；测试
      产物和现场日志不混入生产配置。

### 4.1 用户配置清单

配置项只能由下表中的所属服务解释和保存；网页提交后必须回读服务端生效值。字段名可在契约
阶段调整，但不能把不同含义合成一个不透明 JSON：

| 用户实际配置 | 所属服务 | 页面操作 | 生效方式 |
| --- | --- | --- | --- |
| 选用的 Runtime 输入源 | `openxr-source-node` | 在 Runtime 输入源卡片点击“使用此输入源”或“停止使用” | 更新采集选择；不打开/关闭 USB，不发送机械臂命令 |
| 人工确认的主机硬件关联 | `openxr-source-node` | 从主机硬件清单选择一项与当前 Runtime 输入源关联，或清除关联 | 只改善设备识别和重连显示，不改变 OpenXR 数据来源 |
| 功能 Action 与 component 映射 | `openxr-source-node` | `/tracking/` 为每个功能从当前 profile 的兼容 component 中选择一个轴或一对按键 | 采集节点应用 OpenXR binding，并回显 configured path、bound source、active 和语义值 |
| 功能 Action 方向 | `openxr-source-node` | 在方向 Action 旁切换正方向 | 改变发布的归一化 Action 符号，不修改 Runtime 原始值或空间坐标矩阵 |
| 位置原点 | `spatial-transform-node` | 点击“确认当前位置为原点”或触发绑定的同名 Action | 立即保存当前绝对位置；节点重启后需要重新确认 |
| 空间坐标轴映射 | `spatial-transform-node` | 页面先显示已验证的 OpenXR→机械臂默认映射，也允许分别修改前/左/上对应方向及正负 | 下一帧转换生效；页面同时显示转换前后数值 |
| 绝对位姿平移倍率 | `spatial-transform-node` | 输入无量纲 `translation_scale`，初始值为已确认的 `0.5` | 相对绝对位置在坐标轴映射后乘一次；不影响姿态和 action-only 速度 |
| 三个空间动作分量开关 | `spatial-transform-node` | 勾选空间移动、前后俯仰圆弧、左右旋水平圆弧 | 对应输出归零或恢复，不重建控制会话 |
| 无绝对位姿设备的移动倍率 | `spatial-transform-node` | 分别输入平移速度和圆弧角速度，页面标明 m/s、rad/s | 只用于按键/轴积分；未填写时该设备仍显示输入，但不生成对应位移 |
| 控制模式 | `stararm-102-motion-node` | 选择相对控制或手动关节控制 | 只切换命令来源；切换动作本身不运动 |
| 关节目标 | `stararm-102-motion-node` | `/motion/` 按 `RobotModelInfo.joints` 动态生成滑块，拖动时预览，松开提交 | 生成带 request ID 和 model revision 的 `MotionRequest`，通用页面不假定轴数 |
| 末端执行器目标 | `stararm-102-motion-node` | `/arm-execution/` 按 `RobotModelInfo.tool_actuators` 动态生成滑块 | 生成带 actuator key 的请求；StarArm 节点把 `gripper` 交给 hand controller |
| 执行连接参数 | `stararm-102-execution-node` | 页面按 `ExecutionInfo.connection_fields` 生成选择/输入控件 | 当前节点发布串口路径字段；通用页面不写死串口或设备路径 |

- [x] 每个配置 schema 都声明字段类型、单位、是否必填、初始来源、修改请求、成功返回、失败返回、
      持久化位置和节点重启后的行为，并为这些行为建立契约测试。
- [x] 配置页面不显示需要用户理解的 `source_id`、Dora topic、ROS topic、舵机编码或容器路径；
      诊断视图可以显示这些值，但不能要求用户用它们完成普通操作。

## 5. 共享消息契约

所有消息显式声明版本和单位：距离为米，角度为弧度，四元数为 `[x,y,z,w]`。通用消息不固定
关节数量、名称、顺序或末端执行器数量；型号节点用 `RobotModelInfo` 发布稳定 key、显示名、
顺序、单位、范围和 model revision，高频数组携带该 revision。舵机 ID 和 UART 编码不得进入
通用控制消息。

### 5.1 设备发现和绑定

- [x] `HostDeviceInfo` 表示主机只读硬件清单：连接类型、USB 总线/端口路径、VID、PID、
      manufacturer、product、serial、设备节点和当前是否存在；取不到的字段明确为空，不猜测。
- [x] `OpenXrRuntimeInfo` 表示 Runtime 名称/版本、OpenXR 版本、system ID、system name、
      vendor ID，以及系统报告的位置和姿态跟踪能力。
- [x] `OpenXrInputSource` 表示 Runtime 输入端点：会话内 `source_id`、top-level user path、
      当前 interaction profile、Runtime 本地化名称、可选的 Runtime-native 设备名/序列号、
      位姿能力、Action 能力和当前 active 状态。
- [x] `InputBindingState` 对每个设备无关的功能 Action 发布 configured component path、解析后的
      bound source、Runtime 本地化名称、OpenXR Action 类型、active 和最近语义值；未绑定、
      当前 profile 不适用和设备未激活必须能区分。
- [x] `InputStreamDiagnostics` 按端点发布已接收帧数、最近源/接收时间、观测采样率、观测间隔
      抖动、序号缺口和 Runtime/USB 原始错误计数；这些是观察值，不参与接管或停止判断。
- [x] `InputDiscoveryState` 聚合上述信息、已确认的物理设备关联、当前选择和绑定生效结果，
      供网页一次取得完整快照并订阅后续变化。
- [x] `source_id` 只是在当前 Runtime 会话内路由位姿和 Action 的内部标识。用户实际选择的是
      页面展示的物理信息和 OpenXR 输入端点；持久配置保存可重新识别的 Runtime、user path 和
      interaction profile 条件，不保存一个含义不明的裸数字。
- [x] OpenXR 核心没有暴露每个控制器 USB VID/PID/serial 的接口。物理 USB 信息与 OpenXR
      输入源分别保存；只有 Monado 或用户配置提供可核对依据时才记录关联及其来源，否则页面
      明确显示“未关联”，不得按顺序或名称猜配。
- [x] OpenXR 核心也没有“枚举所有物理控制器”的通用调用。标准输入端点列表由本应用 Action
      Set 使用的 top-level user paths、当前 interaction profiles 和 bound sources 组成；Monado
      设备列表作为带来源标记的补充。页面必须区分“主机发现”“Runtime-native 发现”和
      “OpenXR 当前可绑定端点”。

### 5.2 采集和空间

- [x] `AbsolutePoseFrame` 包含 schema version、序号、源时间、接收时间、`source_id`、user
      path、reference space、位置、姿态，以及 OpenXR 原始 position/orientation
      valid/tracked 标志；不包含 Action、相对基准或机械臂语义。
- [x] `ControlInputFrame` 包含 schema version、序号、时间、`source_id`，以及每个功能 Action 的
      `is_active`、`changed_since_last_sync` 和语义值；不携带实体按键名。
- [x] 功能 Action 固定为 `control_active`、`confirm_origin`、`primary_tool_active`、
      `move_forward_back`、`move_left_right`、`move_up_down`、`front_pitch` 和
      `horizontal_arc`。它们描述输入意图，不包含机械臂型号、关节数或具体末端执行器。
- [x] 三个开关类 Action 使用 boolean；五个方向 Action 由采集节点输出 `[-1,1]` 标量。绑定可
      使用一个 float 轴或 negative/positive 两个 boolean component，并包含用户配置的方向
      反转；采集节点不乘距离、角度、速度、`translation_scale` 或死区。
- [x] `SpatialConfigState` 包含所选 Runtime 输入源、该输入源是否有绝对位姿、坐标轴映射、
      `translation_scale`、action-only 平移/圆弧倍率、已确认原点、三个分量开关和当前控制会话；
      网页修改后返回服务端实际生效值。
- [x] `RelativeToolMotion` 包含 schema version、序号、时间、`control_session_id`、接管状态、
      三维相对平移、`front_pitch_rad`、`horizontal_arc_rad` 和 `primary_tool_active`；不保留
      特定夹爪字段、左右转向或末端绕自身前后轴滚转字段。
- [x] `RelativeToolMotion` 符号固定：平移 `+X/+Y/+Z` 分别是前/左/上，负值分别是后/右/下；
      `front_pitch_rad>0` 是前部抬起，`<0` 是前部往下；从机械臂上方向下看，
      `horizontal_arc_rad>0` 是尖端沿水平圆弧逆时针左旋，`<0` 是顺时针右旋。页面、fixture、
      Rust、Python 和 TypeScript 使用同一约定。

### 5.3 运动和执行

- [x] `RobotModelInfo` 由 motion 节点发布：model ID/revision、显示名、base/TCP frame、按命令顺序
      排列的关节 metadata、末端执行器 metadata、命名关节目标、可选 motion options schema，
      backend diagnostics schema，以及通用 Web 可加载的可视化 manifest。每个 metadata 项
      包含稳定 key、显示名、单位和模型提供的范围；数量不得固定。
- [x] `ModelAssetRequest/Response` 按 model revision、manifest hash 和相对路径读取 URDF/mesh，
      返回 MIME、内容哈希和字节；型号 motion 节点只允许访问自己构建进镜像的 manifest 文件，
      网关只做流式转发和 HTTP 缓存头转换，不理解机器人型号或文件结构。
- [x] `MotionRequest` 包含 request ID、model revision、`{joint_key, position_rad}` 列表、按
      metadata 填写的可选 motion options 或取消 action；motion 节点检查 key/revision/options
      后补成该模型要求的完整目标。命名目标先由网页解析为同一种请求，不建立专用协议。
- [x] `MotionStatus` 使用同一 request ID，包含 acknowledged action、
      `idle/planning/executing/succeeded/failed/cancelled`、后端名称、原始结果 code/message、
      可选轨迹点数和计划时长；通用契约不定义 MoveIt 专属错误码。
- [x] `ToolActuatorRequest` 包含 request ID、model revision、actuator key 和绝对位置；它不携带
      开/合布尔命令，也不假定只有一个 actuator。
- [x] `ToolActuatorStatus` 使用同一 request ID 和 actuator key 发布
      acknowledged/executing/succeeded/failed/cancelled 及型号节点的原始结果；实测位置仍只
      来自 `ArmState`。
- [x] `MotionState` 包含控制模式、可选当前/目标 tool pose、当前控制会话、最新普通运动状态、
      最新 actuator 请求状态，以及按 `RobotModelInfo` diagnostics schema 发布的 backend 字段；
      后端专属码、消息和诊断量均通过 schema 描述，不进入通用固定字段。
- [x] `ArmCommand` 包含 schema version、序号、控制器时间、model revision、按
      `RobotModelInfo` 顺序的关节设定数组和 actuator 设定数组；不携带串口编码、舵机 ID 或
      固定数组长度。
- [x] `ArmState` 包含 schema version、序号、采样时间、model revision、同顺序的关节/actuator
      反馈数组和 `software/hardware` 来源；命令目标和目标预览不能写入反馈字段。
- [x] `ExecutionInfo` 由 execution 节点发布：adapter ID/revision、支持的 model revision、连接
      字段 schema、执行器标签和参数字段 schema；通用页面据此生成连接表单和参数表。
- [x] `ExecutionTransportState` 包含已发现连接端点、当前选择、connected、当前原始错误、最近
      命令摘要、反馈摘要、参数读取结果和参数错误；它不复制 `ArmState`，也不派生运动权限。
- [x] 执行器参数通用契约只规定字段 key、显示名、值、单位、读取时间和错误；具体型号节点决定
      参数字段和标签。没有读到就显示缺失，不使用历史值冒充本次读数。

### 5.4 请求、状态和测试规则

- [x] `SelectInputSourceRequest` 携带 request ID、Runtime identity、top-level user path 和当前
      interaction profile；停止使用是同一请求类型的明确 action。结果返回完整选用状态，不能
      只返回 true/false。
- [x] `ApplyInputBindingsRequest` 携带 request ID、目标 interaction profile，以及每个功能
      Action 的 component path、类型、可选方向反转或 boolean pair；结果逐项返回已接受配置、
      Runtime bound source、active 和原始错误，不能因部分成功而把整页显示成成功。
- [x] `UpdateSpatialConfigRequest` 携带 request ID 和本次修改的坐标轴矩阵、
      `translation_scale`、action-only 倍率或三个开关，并返回完整 `SpatialConfigState`。
- [x] `ConfirmOriginRequest` 只表达立即保存当前位置，并返回完整 `SpatialConfigState`；页面据此
      显示服务端最终值。
- [x] `SetControlModeRequest`、`MotionRequest`/cancel、`ToolActuatorRequest`、执行连接/断开和
      参数刷新请求都带 request ID；响应包含 acknowledged action、最终状态和原始错误，不依赖
      网页按钮状态猜测是否生效。
- [x] 每个服务发布自身 `ServiceState`：构建版本、配置版本、运行状态、当前输入/输出是否存在、
      最近一次真实错误和更新时间；不从“多久没消息”自行推导新的停止门限。
- [x] 每份 Dora 记录配套 `RecordingManifest`，保存构建版本、消息 schema、机械臂模型 ID/哈希、
      各服务配置版本、坐标与单位约定、记录起止时间和数据来源，供回归及后续示教数据复用。
- [x] 高频位姿、相对运动、控制器命令和反馈使用 latest-value；离散请求使用请求/响应或
      action 语义，不能混进可能被覆盖的高频通道。
- [x] 高频消息同时携带来源时间、当前节点接收时间和单调序号；节点保留原来源时间，只新增本层
      时间，不覆盖上游字段。记录回放分别计算采集、空间、ROS 控制输出和执行反馈的实际频率、
      丢帧与逐段延迟，仅作为链路对齐证据，不由观测值产生新的控制门限。
- [x] motion 节点直接把其控制器输出发布为 latest-value `ArmCommand`，下游不再建立第二个固定
      频率重采样器；通用 Dora 契约只传 model revision 和动态数组。具体更新率与硬件编码去重
      由两个型号节点声明和测试。
- [x] Monitor 原始结果只在执行节点内部使用，不增加跨节点 `HardwareFeedback` 通道。
- [x] 每种消息提供 Rust/TypeScript/Python 所需表示、Arrow 往返、字段顺序、单位、缺失字段、
      版本不匹配和 request ID 对齐测试。

## 6. 各服务实施清单

### 6.1 `openxr-source-node`

- [x] 建立无下游依赖的只读探针，先输出主机设备、Runtime、system、interaction profile、
      bound source、位姿能力和 Action 状态，再决定正式节点实现。
- [x] 通过 udev/sysfs 读取主机硬件清单。USB 设备显示总线、端口路径、VID、PID、manufacturer、
      product、serial、接口和相关设备节点；非 USB 输入仍以 Monado/OpenXR 端点显示，不在采集
      节点旁接 evdev、SDL 或另一套平台设备枚举。
- [x] 主机硬件扫描只读取识别信息和热插拔事件，不从 hidraw、evdev、SDL 或厂商 USB 协议读取
      控制数据，因此不会形成绕开 OpenXR 的第二条输入链。
- [x] 使用 `xrGetInstanceProperties` 显示 Runtime 名称和版本，使用 `xrGetSystemProperties`
      显示 system name、vendor ID 及系统位置/姿态跟踪能力；这些是系统级信息，不能冒充单个
      控制器的 USB 信息。
- [x] 当前 Runtime 为 Monado 时，通过镜像内 libmonado 1.7 的只读 role/device API 读取该版本
      实际提供的设备名称、序列号和位置/姿态能力；其他 Runtime 没有这些信息时保持为空，不让
      通用下游依赖 Monado 私有字段。libmonado 1.7 不提供输入 component 或 binding profile
      枚举，这两项仍以 OpenXR interaction profile 定义和 bound-source 查询为权威来源。
- [x] 为 Action Set 涉及的 top-level user path 调用 `xrGetCurrentInteractionProfile`，显示
      当前 interaction profile；在 profile change 事件后立即重新读取并发布新状态。
- [x] 依据构建时同步的 interaction profile 定义生成候选 top-level user paths；只把
      `xrGetCurrentInteractionProfile`、bound sources 或 Monado 设备信息实际确认的候选标为
      当前可用，未确认候选不伪装成已连接设备。
- [x] 页面可选项只来自 Runtime 实际公开的输入端点。主机即使发现某个游戏手柄，如果当前
      Monado/OpenXR 没有为它公开 interaction profile、Action 或位姿，页面也只能显示“主机已
      发现、Runtime 未提供”，不能仅凭 USB 型号承诺可以控制；补齐该设备应修改 Runtime 驱动或
      profile 支持，不得在采集节点旁接厂商协议。
- [x] 为第 5.2 节的设备无关功能创建对应 boolean/float OpenXR Action；用户从当前 interaction
      profile 实际提供的兼容 component 中选择绑定，不在代码里按产品型号或实体按键分支。
- [x] 对每个功能 Action 调用 `xrEnumerateBoundSourcesForAction`，再用
      `xrGetInputSourceLocalizedName` 取得 Runtime 返回的实际输入名称；网页同时显示 configured
      component path、类型、Runtime bound source、active 和语义值。
- [x] 设备选择页面先展示主机硬件清单，再展示 Runtime system 和可用 OpenXR 输入端点；用户
      在实际端点点击“使用此输入源”后，服务端保存选择，并返回 Runtime 当前是否 active、位姿
      能力和 Action 能力。OpenXR 已由 Runtime 管理物理连接，本操作不能伪装成打开 USB。
- [x] “停止使用”只取消当前选择和 Action 激活，并发布 unselected 状态；不停止 Runtime、不
      断开物理 USB、不影响其他 Runtime 端点，也不向空间或机械臂节点发送运动。
- [x] 节点为已选用的 Runtime 端点分配会话内 `source_id`，下游只用它关联数据帧；网页不得把
      裸 `source_id` 当作设备型号或让用户手工填写。
- [x] 物理硬件与 OpenXR 输入端点的关联优先使用 Runtime/Monado 明确暴露的信息，其次使用
      用户确认的关联；无法确认时保留两张清单并显示未关联，不按数组顺序、左右角色或相似名称
      猜测。
- [x] 自动关联只接受两侧实际返回的同一非空序列号或 Runtime 明确提供的设备关系；VID/PID、
      product 名称和 user path 只能帮助展示，不能单独证明是同一物理设备。
- [x] 绑定编辑器根据 Action 类型只展示兼容 component：boolean 功能绑定 boolean component，
      方向功能可绑定一个 float component 或一对 boolean component；保存前显示组合后的
      `[-1,1]` 实际值，不能只显示“已保存”。
- [x] 应用绑定时执行 `xrSuggestInteractionProfileBindings` 并附加 Action Set；若依赖版本要求
      重建 OpenXR session，则只重建采集会话并返回明确结果，不重启其他服务。
- [x] 应用后重新枚举 bound sources；configured path、Runtime 实际解析源、active 和语义值一起
      发布，未绑定、当前 profile 不适用和设备未激活分别显示。
- [x] 用户确认的输入选择、物理关联和功能 Action 绑定写入采集服务配置 volume；节点重启后重新读取
      并只匹配同一 Runtime/user path/interaction profile 条件，找不到时显示未连接，不自动
      改选其他端点。
- [x] 同一已选端点重新出现时恢复选择和对应 profile 的绑定配置，并如实发布 Runtime
      当前 Action 值，不强制改成 `control_active=false`。空间节点在来源中断时清除旧基准；
      恢复时如果 Action 当前为 active，从当前位姿建立新会话基准。profile 变化时只加载该
      profile 已存在的用户绑定，没有配置的 Action 明确显示未绑定。
- [x] 绝对位姿 Action 只发布 Runtime 返回的位置、姿态、时间和 valid/tracked 标志；功能 Action
      单独发布，二者不能合并成一个带机械臂含义的状态。
- [x] 有位姿的端点发布绝对跟踪；没有位姿但有按键/轴的端点仍发布同一种
      `ControlInputFrame`。下游契约不因端点类型改变。
- [x] 不做第二遍 Fusion，不默认迁移旧 One Euro 滤波。若记录数据证明 Runtime 输出存在具体
      问题，先保存复现数据和影响，再由用户确认是否加入处理。
- [x] 主机热插拔、Runtime 暂不可用、interaction profile 变化、所选端点消失和重新出现都更新
      `InputDiscoveryState`；只报告实际状态，不添加计数门限或自动切换到另一个设备。
- [x] 将发现快照、绝对位姿和功能 Action 帧接入 Dora 记录输出；测试可在不连接真实设备时
      原样回放，回放帧明确标记来源并使用独立测试配置，不污染现场选择和绑定。
- [x] 保留旧页面的确定性模拟数据入口：采集节点生成独立 simulation source 和 100 Hz 标准
      OpenXR 契约帧，完整动作循环为启动抬升 10 cm、上/下 5 cm、左/右 2 cm、前/后 5 cm、
      左旋/右旋 8°、前部抬起/往下 8°。模拟帧走正常空间、运动和 execution 链路；停止只发布
      inactive 输入，不联动起始位、夹爪或串口连接，也不读取实体输入零偏。

### 6.2 `spatial-transform-node`

- [x] 只订阅已选用端点的 `AbsolutePoseFrame`、`ControlInputFrame` 和空间配置，不读取 USB、
      OpenXR API、ROS、URDF 或 UART。
- [x] 发布 `SpatialConfigState`，完整回显当前输入端点、是否有绝对位姿、轴映射、
      `translation_scale`、action-only 平移/圆弧倍率、原点、控制会话和
      三个分量开关；网页首次打开先读取该状态，不能用前端默认值覆盖服务端。
- [x] `confirm_origin` 可来自已配置的功能 Action 或网页单次请求；每次上升沿/点击只保存当前
      绝对位置，不等待、不累计样本、不重置姿态，也不产生机械臂命令。
- [x] 原点保存在空间节点进程中，页面刷新或切换 Web 服务不丢失；空间节点重启后原点为空，
      页面如实显示“未确认”，由用户再次单次确认，浏览器不能补写旧值。
- [x] `control_active` 上升沿生成新的 `control_session_id`，同时保存当前输入位置和姿态作为
      本轮零相对量；第一帧输出应为零平移、零圆弧。
- [x] 接管期间每帧都相对于本轮基准计算，不把上一轮残余量带入新会话；取消接管只发布 inactive
      并清除本轮基准，不发送起始位、夹爪或其他运动。
- [x] 绝对位置路径按“原点 → 本轮基准 → `base_from_tracking_axes`”的固定顺序计算；每一步的
      输入和输出都能在记录测试中单独核对。
- [x] `base_from_tracking_axes` 默认沿用已验证矩阵
      `[[0,0,-1],[-1,0,0],[0,1,0]]`：OpenXR 的前方 `-Z` 映射机械臂 `+X`，OpenXR 左方
      `-X` 映射机械臂 `+Y`，OpenXR 上方 `+Y` 映射机械臂 `+Z`。这是坐标约定，不是设备型号
      分支；用户修改后只保存一份矩阵并由页面回显。
- [x] 轴映射后的绝对位置相对量只在此乘一次 `translation_scale`；初始值为已确认的 `0.5`，
      页面允许修改并显示输入相对量、缩放值和最终输出。姿态、action-only 速度和下游节点不再
      应用该倍率。
- [x] 绝对姿态相对于本轮姿态基准计算，再按手柄自身局部轴分解 `front_pitch` 和
      `horizontal_arc`；不使用欧拉角跨轴拼接，不采集夹爪自身前后轴滚转。
- [x] 位姿输入沿用已验收的旋转向量分解：对相对四元数取局部 scaled axis，
      `front_pitch_rad=-local_rotation.x`、`horizontal_arc_rad=local_rotation.z`，忽略局部 Y
      分量；用单轴和组合四元数 fixture 固定符号及交叉轴行为。action-only 输入直接使用绑定后
      的两个归一化 Action，不再做四元数分解。
- [x] 三个开关分别控制：空间位置移动、前部抬起/前部往下、左旋/右旋。关闭某项只把对应输出
      归零，其他已开启分量继续工作，不重建会话。
- [x] 上、下、左、右、前、后只输出整体平移；前部抬起/前部往下只输出竖直圆弧角；左旋/
      右旋只输出水平圆弧角。名称、符号和语义以本文第 5.2 节的唯一定义为准。
- [x] 接管期间把 `primary_tool_active` 当前值传入 `RelativeToolMotion`；接管开始时只建立基准，
      不能因读取到初始布尔值就自动提交动作，只有实际状态变化才交给型号 motion 节点解释。
- [x] 对只有 Action 的输入端点，在 `control_active=true` 时按时间积分归一化轴/按键值：空间
      三轴乘用户填写的平移速度（m/s），两个圆弧轴乘用户填写的角速度（rad/s），再写入同一
      `RelativeToolMotion` 字段。缺少相应倍率时页面显示“该分量尚未配置倍率”，原始 Action
      仍可查看，但该分量不产生位移；不得写入猜测默认值。
- [x] 当所选端点消失，或 OpenXR 明确报告当前所需位置/姿态无效时，发布对应事实和 inactive
      状态，不沿用旧样本继续累计；恢复后下一次接管重新建立基准。
- [x] 为轴映射、原点、每轮基准、四元数相对旋转、三个开关、两类输入端点和刷新恢复分别建立
      单元测试与记录回放测试。

### 6.3 `stararm-102-motion-node`

- [x] 从 `ref/arm/ros2` 迁移一套 ROS 2 launch、controller 配置、MoveGroup、MoveIt Servo、
      `JointTrajectoryController`、`hand_controller` 和 `JointStateTopicSystem`；不保留第二套
      真机/模拟 ROS 配置。
- [x] 镜像构建时取得厂家 StarArm-102 模型并应用当前已验证补丁；生产仓库只维护补丁和配置，
      不复制完整厂家源码、URDF、SRDF 或 mesh 树。
- [x] 模型构建补丁逐项保留并测试，而不是用“沿用旧补丁”带过：把厂家误导出的 catkin
      description 改为 ament 安装；SRDF arm group 明确为 `base_link -> link6`；IK 从 KDL 改为
      TRAC-IK；厂家新版 URDF 的 J4 axis 修正为 `0 0 -1`；ros2_control 使用
      `JointStateTopicSystem` 和唯一 command/state topics；J1–J6 与
      `joint7_left` 初始值使用 `[0°,0°,-3°,0°,0°,0°]`、`1°`；J5 和 `joint7_left` command
      interface 与同一 URDF 范围一致；`joint7_right` 只保留 URDF mimic，不作为第二个主动控制
      关节；动力学补丁保持 J1–J6 速度配置和 `30 rad/s²`。
- [x] 构建后检查厂家新版 URDF 的 `joint3 xyz="0.12162 -0.0021 0"` 和九个 mesh，而不是在项目
      里保存另一份几何副本；MoveIt 与 Web 镜像从同一厂家提交/归档哈希和同一补丁集生成产物。
- [x] arm 链固定使用厂家 `base_link -> link6`，网页和 TF 使用同一模型；在夹爪 TCP 未完成
      实测前不添加另一个虚构工具 link。
- [x] 六轴 IK 保留 TRAC-IK、当前已验证的 `Distance` 模式和厂家/现有配置中的 5 ms timeout；
      J1–J6 `30 rad/s²` 保留此前官方资料确认后的数值，不重新猜测或删除。
- [x] 保留现有 `controller_manager.update_rate=100 Hz`、Servo
      `self_collision_proximity_threshold=0.002 m` 和 `hard_stop_singularity_threshold=.inf`；前者
      来自现有控制循环，后两项分别对应已复现的超小机械臂 2.464 mm 起始间隙和有限奇异硬停
      无法退出问题。迁移测试核对原配置与来源，不再叠加第二套阈值。
- [x] 100 Hz ROS 控制器输出直接发布为 latest-value `ArmCommand`，删除旧 Rust 10 ms ticker；
      execution 节点消费最新完整命令，不在通用层重采样。
- [x] 发布 `RobotModelInfo`，明确本型号的六个主动关节、一个 `gripper` actuator、命名目标
      `start`、model revision、范围、显示标签、MoveIt velocity/acceleration scaling options 和
      Servo 原始状态/停止原因 diagnostics schema 及可视化 manifest；这些具体参数不得复制到
      通用 Web、网关或共享消息源码。
- [x] 接收 `RelativeToolMotion`、最新 `ArmState`、`MotionRequest`、`ToolActuatorRequest` 和
      控制模式请求；发布 `RobotModelInfo`、`ArmCommand`、`MotionState` 和 `MotionStatus`。
- [x] 收到新的 `control_session_id` 时，使用最新 `ArmState` 经 TF2 得到当前 `link6` 位姿，
      作为机械臂本轮基准；不能使用网页目标或上一轮缓存代替反馈。
- [x] 将三维平移叠加到本轮 TCP 基准。使用已确认的夹爪后部安装基准到 TCP 几何关系，把
      `front_pitch_rad` 和 `horizontal_arc_rad` 转为同时包含位置和姿态的六自由度 Pose 目标。
- [x] 当前安装基准到 `link6` 使用已验收配置向量 `[0,0,0.07313] m`，实现阶段从 `ref/`、厂家
      模型和回放 fixture 三方核对后只保留一处。若基准位姿为 `(p0,q0)`、本轮平移为 `t`、局部
      前后俯仰为 `pitch`、基座水平旋转为 `turn`、上述向量为 `r`，目标使用
      `q1=Rot(base_Z,turn)*q0*Rot(tool_X,pitch)`、
      `p1=p0+t+R(q1)r-R(q0)r`；不得把圆弧退化成纯平移或原地改变姿态。
- [x] 左旋/右旋保持夹爪后部安装基准不动，尖端沿同一水平圆弧正反向移动；前部抬起/前部往下
      保持同一基准不动，尖端沿同一竖直圆弧正反向移动。
- [x] 连续 Pose 目标只交给 MoveIt Servo；普通完整 J1–J6 目标只交给 MoveGroup。两者最终写
      同一个 `arm_controller`，夹爪绝对角只写同一 ROS 实例的 `hand_controller`。
- [x] 控制模式明确为相对控制和手动控制。切换模式本身不发布运动或夹爪命令；进入手动模式时
      页面从最新反馈初始化，切回相对控制时要求下一次接管建立新基准。
- [x] 本型号把 `primary_tool_active` 映射到 `gripper` actuator：true 为闭合 `1°`、false 为打开
      `90°`。接管开始只记录当前值；后续实际 false→true/true→false 变化才分别提交对应的
      `ToolActuatorRequest`。
- [x] 普通关节请求处于 planning/executing 时暂停 Servo 向手臂控制器写入；普通请求成功、
      失败或取消后，以最新反馈重建下一轮相对控制基准。夹爪请求保持独立，不因此暂停。
- [x] `MotionStatus` 完整保留 request ID、已确认动作、规划/执行/成功/失败/取消、MoveIt 原始
      错误、轨迹点数和时长，网页不根据按钮状态自行推断结果。
- [x] 多个普通请求的接受、抢占和取消沿用 MoveGroup action 的实际语义；项目不再额外建立
      请求队列、自动取消、拒绝门槛或恢复状态机。
- [x] 起始位按钮提交普通 `MotionRequest` 目标 `[0°,0°,-3°,0°,0°,0°]`，保持当前夹爪角度；
      MoveIt 普通执行结果就是结果，不增加反馈角度二次检查。
- [x] 普通规划因当前模型真实起始自碰撞失败时，读取本次接触对，只在本次 planning scene 的
      ACM 放行这些对，然后重试同一个 MoveGroup 请求；该逻辑适用于任意普通目标，不是起始位
      旁路，也不能直接写关节位置。
- [x] Servo 状态码保持 MoveIt 原义：1、3、4 显示减速/约束状态但不伪装成停止原因；2、5、6
      分别显示接近奇异、碰撞和关节限位停止；暂停后尚无新命令的 -1 只显示原始状态，不闭锁
      下一次有效接管。
- [x] 接管期间收到 code 6 时，用最新 `ArmState` 发布一次保持命令并忽略后续 Servo 控制器输出；
      用户释放接管后结束冻结，下一次接管重新建立基准，且只有 code 6 已消失才恢复 Servo 输出。
      普通 MoveGroup 运动和独立夹爪请求不经过此冻结；不增加角度、持续时间或次数判断。
- [x] 从 code 2/5/6 或接管释放恢复时不要求重启服务；下一次有效控制使用最新 `ArmState`
      重建基准。不得通过修改 MoveIt 门限来掩盖状态。
- [x] ROS 控制器输出的 J1–J6 与夹爪设定值组成 `ArmCommand`；该节点不编码串口、不读取舵机
      ID，也不判断当前是软件反馈还是真机反馈。
- [x] 首次收到 `ArmState` 或从软件反馈接入真机反馈时，以该反馈停用再启用 arm/hand controller，
      使 ros2_control 的状态与唯一权威反馈同步；同步期间不转发控制器输出。真机断开后保留最后
      一帧 `hardware` 是既定语义，下一条用户命令产生的首帧 `software` 是本节点命令回显，不能
      再次同步并截断正在执行的轨迹。连接本身只更新反馈，只有相对运动、普通轨迹或末端执行器
      请求这一实际用户意图才重新允许 `ArmCommand`，不能把 controller 激活时的保持值当成用户命令。
- [x] 关节范围只取自构建后实际加载的 URDF/ros2_control 配置，网页通过同一模型取得范围，
      不在 Rust、Python 或 TypeScript 再维护一套手写限制。

### 6.4 `stararm-102-execution-node`

- [x] 发布 `ExecutionInfo`，声明所支持的 StarArm model revision、七个执行器显示标签、串口连接
      字段、Monitor 字段和内部参数字段；通用网关和页面只按该 schema 渲染。
- [x] 从 `ref/arm/src/fashionstar-uart` 提炼厂家公开协议库到新工程，不建立对 `ref/` 的生产依赖；
      协议 crate 不知道 J1–J6、URDF、MoveIt 或网页。
- [x] 通过 udev/sysfs 枚举可用串口及其路径、USB VID/PID、manufacturer、product 和 serial；
      网页提供发现列表，同时保留用户输入明确路径的能力。占用或权限问题只在实际打开后作为
      `last_open_error` 返回，不靠扫描猜测。
- [x] 未选择或未连接串口时节点仍然运行。收到 `ArmCommand` 后直接把控制器 J1–J6 和夹爪
      设定值发布为 `software` `ArmState`，不重复插值、不模拟电机延迟、不启动另一个 executor。
- [x] 完全没有收到命令或反馈时才以 `[0°,0°,-3°,0°,0°,0°]` 和夹爪 `1°` 建立初始软件
      状态；它们只用于初始显示，不是连接条件或运动门限。
- [x] 连接请求先释放本节点已有串口句柄，再打开用户选择的路径；只执行 Ping ID 0–6、内部
      参数读取和 Monitor 读取，不先发送位置命令。
- [x] Ping/Monitor 成功后，把实测 J1–J6 和夹爪作为同一个 `ArmState` 的 `hardware` 来源；
      不检查软件姿态、真机姿态、起始位或二者差值。
- [x] 真机连接后同一 `ArmCommand` 编码为 ID 0–6 同步位置命令。继续使用已验证的 1 Mbps
      协议及厂家 follower 路径参数 `100 ms / 50 ms / 50 ms / power 0`，不得用新猜测值覆盖。
- [x] 按最终 0.1° 整数编码后的完整七舵机命令精确去重：编码值完全相同时不重复发送，任一值
      变化就发送；这不是角度门限，不能再叠加变化量限制。
- [x] Monitor 持续更新 `hardware` `ArmState`。10 ms 读取保持既有反馈节拍；协议返回的 0.1°
      整数位置完全相同时只更新节点内采样序号，任一关节或夹爪值变化就发布新状态。这是精确重复
      消除而非角度门限。串口发现和完整传输状态复用系统已有的 1 s 发现节拍，网关按 latest-value
      消费，避免把包含 63 项参数的重复状态积压在离散请求之前。命令目标、最后发送命令和
      Monitor 实测分别保存，页面可以同时查看但不能互相冒充。
- [x] J1–J6 命令和 Monitor 反馈在 execution 节点中保持同号，J4 不再取反或增加补偿；
      J4 运动学方向只由已修正的 URDF 表达。夹爪的模型角、传动方向和 ID 6 编码
      只在 execution 边界换算；通用 `ArmCommand`、`ArmState`、ROS 和网页保持模型方向。
- [x] 每次连接读取 ID 0–6 的内部参数并带上舵机 ID、J 序号/夹爪和型号发布。参数读取失败只
      写入 `parameter_error` 并显示“本次未读取”，不把旧参数当新值，也不阻止已经成功的连接。
- [x] 运行中串口读写出现 `Operation timed out`、设备消失或其他实际 I/O 错误时，立即释放旧
      句柄并尝试重新打开同一 selected port；重开结果直接更新 `ExecutionTransportState`，不增加重试次数
      门限、姿态门槛或机械臂自动运动。
- [x] 用户断开后停止 UART 读写，保留最后一次真机反馈；下一条 `ArmCommand` 从该状态继续
      `software` 反馈，不返回起始位、不重启 ROS。
- [x] 网页刷新、输入设备变化、串口连接/断开/重连都不能自动发送起始位、夹爪或保持命令。
- [x] 保留厂家 Python SDK 双向伪串口交叉测试，并覆盖 Ping、Monitor、同步位置、内部参数、
      半单位舍入、负角度、分片、乱序、噪声、损坏帧重同步、端口重连和七舵机映射。

### 6.5 Web 网关、四个网页服务和 Web 入口

- [x] `web-gateway-node` 为 tracking、spatial、motion 和 arm-execution 分别提供带命名空间的
      HTTP 请求和 WebSocket 状态流；每个请求保留 Dora request ID、结果和原始错误。
- [x] 网关最小 HTTP 面明确为：`GET /api/tracking/state`、`POST /api/tracking/source`、
      `POST /api/tracking/bindings`、`POST /api/tracking/simulation`；`GET /api/spatial/state`、
      `PATCH /api/spatial/config`、
      `POST /api/spatial/origin`；`GET /api/motion/state`、`POST /api/motion/mode`、
      `POST /api/motion/request`、`POST /api/motion/cancel`；
      `GET /api/motion/assets/{path}`；`GET /api/arm-execution/state`、
      `POST /api/arm-execution/actuator`、`POST /api/arm-execution/connect`、
      `POST /api/arm-execution/disconnect` 和 `POST /api/arm-execution/parameters`。具体 JSON
      使用第 5 节唯一契约；当前 execution metadata 让页面呈现串口，但通用路由不写死传输类型。
- [x] WebSocket 固定为 `/ws/tracking`、`/ws/spatial`、`/ws/motion` 和
      `/ws/arm-execution`；连接后第一条业务消息是该命名空间完整快照，后续才是带 schema
      version 和序号的更新。WebSocket 断开只改变页面连接提示，不改变业务节点状态。
- [x] 网关启动或 WebSocket 重连时先读取各业务节点最新快照，再转发增量；浏览器不需要等待
      下一帧才能知道当前选择、原点、运动、机械臂或执行连接状态。
- [x] 网关只转换 Dora 与 HTTP/WebSocket，不托管页面、不计算运动、不复制权威状态，也不在
      一个业务节点离线时伪造其默认值。
- [x] 四个 Next.js 应用分别作为 `web-tracking`、`web-spatial`、`web-motion`、
      `web-arm-execution` Compose 服务运行，可独立构建、停止和查看日志。
- [x] `web-tracking` 分成四块：主机硬件清单、OpenXR Runtime/system 信息、Runtime 输入端点、
      当前选用项与 Action 绑定。每条记录显示数据来源和缺失字段，不把 USB 信息与 OpenXR 信息
      合并成虚假设备。
- [x] `web-tracking` 的操作流程是“查看主机硬件与 Runtime 信息 → 在 Runtime 输入端点点击使用
      → 查看当前 interaction profile 和能力 → 为每个功能选择兼容 component → 应用绑定 →
      查看 Runtime 最终 bound sources → 查看实时位姿/Action”。任一步失败都保留当前真实状态，
      并在发生失败的步骤旁显示原始错误。
- [x] `web-tracking` 提供绝对位置、姿态、valid/tracked、时间、当前 Action 值和连接变化视图，
      便于确认输入本身，不显示机械臂目标。
- [x] `web-spatial` 首次加载读取 `SpatialConfigState`，显示当前端点、原点是否已确认、一次确认
      原点按钮、输入方式、轴映射结果、三个分量开关、control session 和最新相对输出。
- [x] `web-spatial` 同时显示输入绝对位姿和转换后相对量，明确单位和坐标名称，使上下左右前后、
      前部抬起/往下、左旋/右旋可以逐项核对，但不计算 IK。
- [x] `web-motion` 显示相对/手动控制模式、可选当前/目标 tool pose、普通运动 request ID、状态、
      轨迹信息，并按 `RobotModelInfo` diagnostics schema 显示后端原始字段。当前 StarArm 页面会
      因运行时元数据显示 MoveIt/Servo 原始码和停止原因，但通用源码不写死这些字段。
- [x] `web-motion` 按 `RobotModelInfo.joints` 动态生成任意数量的手动关节滑块，按
      `named_targets` 生成目标按钮，并按 motion options schema 生成可选执行参数。进入手动模式
      时从同 revision 的最新 `ArmState` 初始化；拖动只更新目标预览，松开提交 joint key/value
      列表；页面不出现硬编码 J1–J6、起始位值或 MoveIt 参数。
- [x] `web-arm-execution` 按 `RobotModelInfo`、`ExecutionInfo` 和唯一 `ArmState` 动态显示关节、
      actuator、反馈来源、最后 `ArmCommand`、连接端点、反馈摘要、连接错误和本次参数字段。
- [x] 通用页面通过 `visualization_manifest` 和 `/api/motion/assets/...` 加载型号 motion 节点提供的
      URDF/mesh；主模型只显示反馈，目标预览使用独立样式。关节/执行器标签和硬件 ID 都取自
      元数据，页面源码不包含 StarArm 模型或固定标签。
- [x] 页面按 `ExecutionInfo.connection_fields` 生成连接控件；当前 StarArm 节点提供串口发现列表
      和路径字段。连接成功后隐藏输入，显示实际端点、命令去向和断开；失败后恢复相同动态控件。
- [x] 页面按 `RobotModelInfo.tool_actuators` 动态生成 actuator 滑块，从最新反馈初始化，拖动只
      预览，松开提交 `ToolActuatorRequest`；是否触发手臂规划由型号 motion 节点定义。
- [x] 应用之间不通过浏览器内存、浏览器存储或彼此私有 API 传递系统状态。
- [x] `packages/ui` 保存共享 shadcn/ui 组件和设计 token；`packages/contracts` 保存唯一的
      TypeScript 契约；`packages/gateway-client` 统一 HTTP/WebSocket 和请求 ID。
- [x] 三维渲染器留在 `web-arm-execution`；机器人资产只由型号 motion 节点提供，通用 Web 包不
      打包或复制任何特定型号 URDF、mesh、关节范围或命名目标。
- [x] 所有页面遵循 shadcn/ui 的组件组合、CSS 变量和可访问性规范，不建立第二套样式系统。
- [x] 四个页面使用共享的跨服务导航链接，但导航不聚合业务状态、不形成统一控制台；当前页面
      故障不影响用户打开其他页面。
- [x] 保持模型颜色、观察平面位置、页面文本选择和复制等已验收行为。
- [x] 目标预览与 `ArmState` 反馈明确区分；刷新后从服务恢复状态，不把权威状态放进浏览器存储。
- [x] `web-entry` 绑定 `192.168.100.10:8765`，其余 Web 服务只使用 Compose 内部端口。
- [x] 页面路径固定为 `/tracking/`、`/spatial/`、`/motion/`、`/arm-execution/`；
      `/api/` 和 `/ws/` 转发给 `web-gateway-node`。
- [x] 每个 Next.js 应用设置匹配路径的 `basePath`，覆盖刷新、跳转、静态资源和生产构建。
- [x] `web-entry` 只做 HTTP/WebSocket 转发；具体实现可替换，不承担鉴权、状态或业务逻辑。
- [x] 浏览器测试分别覆盖四个页面的首次快照、状态增量、刷新、WebSocket 重连、请求成功/失败、
      键盘操作、文本复制和错误可见性；输入 fixture 和 Compose 集成测试覆盖空设备及设备变化，
      一个页面不依赖另一个页面先打开。

### 6.6 必须跑通的完整操作与系统流程

下面的流程不是概念示意。每条都要有浏览器级操作记录、对应 Dora 请求/状态和最终断言；其中
任一步未定义时，先修订消息或服务条目，再进入实现。

1. [x] **查看并选用输入设备**：打开 `/tracking/` 后，页面分别列出主机硬件、Runtime/system
       和 Runtime 输入端点；用户根据型号、USB 基础信息、Runtime 名称、interaction profile、
       本地化名称和能力选中一个 Runtime 端点，点击“使用此输入源”；页面显示 selected/active
       的实际结果。USB 与 Runtime 无法证明关联时保留两条记录，由用户可选地确认关联。
2. [x] **配置功能输入**：页面根据所选端点当前 interaction profile 列出 Runtime 实际支持的
       boolean/float component；用户逐项为接管、确认原点、主末端执行器、三轴移动和两个圆弧
       动作选择 component 及方向；点击应用后，页面逐项显示配置路径、Runtime 最终 bound
       source、active 和实时值。未绑定或 Runtime 拒绝的项目保持未生效，不能显示成成功。
3. [x] **确认原点并接管**：打开 `/spatial/`，先看到服务端当前原点、坐标映射、倍率和三个开关；
       用户点击一次确认当前位置为原点。接管 Action 从 false 变 true 时建立本轮位置/姿态基准，
       第一帧相对量为零；释放后停止输出运动，再次接管使用新基准。
4. [x] **用有绝对位姿的输入源控制**：移动或转动所选跟踪设备；空间页同时显示 Runtime 原始位姿
       与转换后的上、下、左、右、前、后空间移动、前部抬起/前部往下和左旋/右旋；运动节点用
       最新 `ArmState` 建立机械臂基准并输出 `ArmCommand`，具体规划/连续控制后端由该型号节点实现；
       执行页的软件或真机反馈模型随唯一 `ArmState` 更新。
5. [x] **用只有按键/轴的输入源控制**：原始 Action 页面先能看到轴/按键值；用户在空间页填写
       平移速度和圆弧角速度并确认轴映射，接管后才按时间积分为同一相对运动。未填写倍率时只显示
       缺少哪个配置，不产生该分量位移，也不影响其他已配置分量。
6. [x] **手动关节与命名目标**：在 `/motion/` 切到手动控制，页面按 `RobotModelInfo` 为当前型号
       动态生成关节滑块并从同 revision 的最新反馈初始化；拖动只改目标预览，松开提交普通
       `MotionRequest`。点击型号节点提供的 `start` 命名目标也提交同一种请求，页面使用同一
       request ID 显示结果；通用页面不知道其轴数和值，取消不联动输入或 actuator 归位。
7. [x] **末端执行器操作**：`primary_tool_active` 的实际真假变化由型号 motion 节点解释；执行页
       按 metadata 动态生成 actuator 滑块。两条入口都生成 `ToolActuatorRequest/Status`，是否与
       手臂规划独立由型号节点声明，三维模型只按 `ArmState` 反馈更新。
8. [x] **连接与断开真机**：打开 `/arm-execution/`，页面按 `ExecutionInfo` 显示连接字段；当前
       StarArm 节点提供串口清单/路径。连接请求先读取 Ping、参数和 Monitor，不发送位置；成功后
       显示实际端点、命令摘要、反馈和参数，断开后保留最后反馈并继续软件反馈。
9. [x] **刷新、重启和设备变化**：刷新任一网页只重新取得服务端快照；输入端点消失时停止选中源
       的有效输出并显示 Runtime 状态，重新出现后恢复配置并如实传递当前 Action；执行传输 I/O 失败时释放
       旧连接并报告同一端点的重开结果；任何一种变化都不自动发送机械臂或 actuator 命令。
10. [x] **记录与回放回归**：测试工具以明确的记录文件路径启动一次真实或合成输入记录，保存
        manifest、发现快照、原始输入、空间输出、ArmCommand 和 ArmState；测试命令在独立配置中
        回放并逐字段比较输出。本阶段不为它新增生产页面；启动/停止工具本身不改变起始位、控制
        模式或夹爪。

## 7. Docker Compose 运行方式

- [x] 先建立两个最小 Dora 节点的 Compose 部署探针，验证官方 local `dora run` 和 networked
      coordinator/daemon 两种方式中，哪一种能同时满足：同一 dataflow、Compose 一键启动、
      日志可见、单节点/服务可恢复、`down` 后无残留进程或 socket。
- [x] 记录探针的真实进程树、容器边界、Dora 节点启动者、通信端点、重启行为和停止行为；不能
      把 Dora daemon 启动的子进程假称为 Compose 直接管理的服务。
- [x] 优先选择满足当前单机需求的最简单官方模式。若 Dora 官方生命周期无法同时满足“一节点一
      Compose 服务”，必须在开始业务实现前明确报告差异并更新本节，不能自行编写第二套节点
      注册、消息总线或进程管理器绕过 Dora。
- [x] 无论内部选用哪种 Dora 官方模式，`services/compose.yaml` 都是用户唯一入口：Compose
      管理 OpenXR Runtime、Dora 运行基础设施/数据流、四个网页、Web 入口及所需构建、网络、
      volume 和设备映射；用户不需要另开终端执行 `dora up` 或 `dora start`。
- [x] 在 `services/` 中使用 `docker compose up -d` 启动完整系统。
- [x] 在 `services/` 中使用 `docker compose down` 停止完整系统。
- [x] 生命周期只使用整套 `docker compose up -d` 和整套 `docker compose down`；重启就是先
      `down` 再 `up -d`。不维护单服务重启、接续或恢复状态，也不增加包装脚本。
- [x] 不使用 host network；只有 `web-entry` 对外暴露端口。
- [x] `openxr-runtime` 获得实际需要的输入设备和 Runtime IPC 目录；`openxr-source-node` 只连接
     该 Runtime。用现场探针确认 USB 热插拔和容器重启后的设备可见性。
- [x] `stararm-102-execution-node` 获得串口发现所需的只读 udev/sysfs 信息和实际串口设备；
      用假串口和真实设备节点分别验证 Compose 映射，避免再次出现容器内路径不存在。
- [x] 所有内部 HTTP、WebSocket、Dora、ROS 和 Runtime 端点只在 Compose 网络或专用 volume
      中使用；端口、socket 和 volume 的所有者在 Compose 中逐项写明。
- [x] 编译发生在镜像构建阶段，容器启动时不现场编译。
- [x] Compose、镜像和容器均不从 `ref/` 挂载或读取生产代码。
- [x] `docker compose logs <service>` 能看到该服务的启动配置摘要、连接状态、请求 ID 和原始
      错误；日志不输出用户未要求的警报判定或安全结论。

## 8. 实施顺序与阶段出口

每阶段先完成对应测试和审查，再进入下一阶段；迁移完成后删除同一职责的旧生产路径，不长期
维护新旧两套实现。

1. [x] **完整盘点**：读取 `ref/` 的代码、页面、文档和测试，补全第 3.1 节；每项旧能力都有
       新归属、替代或明确删除理由，并列出此次发现的高收益新增功能。
2. [x] **部署探针**：用两个最小 Dora 节点和 Compose 验证第 7 节全部行为，记录并确定唯一
       Dora/Compose 运行拓扑；此阶段不创建业务节点。
3. [x] **设备发现探针**：只读取得主机硬件、OpenXR 标准信息、Monado 可选信息、interaction
       profile 和 bound sources；保存实际快照，证明哪些字段可得、哪些不可得。
4. [x] **契约和骨架**：根据部署与设备探针结论建立 workspace、Arrow 类型、消息 crate、
       dataflow、配置 schema 和 Compose 骨架；所有消息往返、request ID 和配置回显测试通过。
5. [x] **采集和绑定**：实现 Runtime 输入源的选用/停止使用、Action 配置/应用/回显、主机与
       Runtime 设备变化、绝对位姿和控制输入两个独立输出；不连接空间、MoveIt 或 UART。
6. [x] **空间**：用记录数据和合成 Action 实现原点、接管基准、坐标转换、三个开关和两类输入
       路径，逐项验收所有平移与圆弧语义。
7. [x] **运动**：迁移模型构建、MoveIt/Servo、MoveGroup、控制模式、普通运动、夹爪、起始位、
       自碰撞同路径重试和 Servo 状态语义，以合成 `ArmState` 完成闭环。
8. [x] **执行**：先实现软件反馈，再用假串口完成端口发现、连接、命令、Monitor、内部参数、
       J1–J6 同号传递、夹爪编码、断开和运行中重连。
9. [x] **Web**：实现网关、四个 Next.js 服务、共享包和可替换 Web 入口；每个页面单独完成首次
       快照、操作、错误、刷新和断线重连测试。
10. [x] **完整集成**：`docker compose up -d` 启动 OpenXR Runtime、Dora 链、ROS、软件执行、
        四个网页和 Web 入口；完成从输入记录到软件 `ArmState` 和三维模型的全链路。
11. [x] **迁移清理**：切换唯一生产入口，删除 Unix Socket、旧 HTTP 旁路、旧单体职责、重复
        状态、旧页面和无调用方测试替身，再更新第 3.1 节最终去向。
12. [x] **循环收敛**：严格执行第 1.4 节，问题出现多少轮就修正和重跑多少轮；达到连续两轮无
        新修改后再做最终只读复核。
13. [x] **真机验收**：用户已经授权；第 9 节通过后严格按第 10 节执行最后一次单步复测。不得把
        未执行项写成通过，也不能为了预想问题提前增加用户未要求的门限或分支。

## 9. 自动化与纯软件验收

### 9.1 输入和空间

- [x] 使用固定 udev/sysfs fixture 验证 USB、缺少字符串、同型号多设备和热插拔清单；
      未提供字段保持为空，设备移除后状态明确更新。
- [x] 使用 OpenXR/Monado 探针记录验证 Runtime name/version、system name/vendor/tracking、
      Runtime-native device name/serial、user path 和 interaction profile 分别来自正确接口。
- [x] 构造无法关联的 USB 和 OpenXR 输入源，验证页面保持两条独立记录并显示“未关联”，不按
      顺序或名称自动匹配。
- [x] 选用实际 Runtime 输入端点后，验证服务端返回完整选择对象和会话内 `source_id`；停止使用
      后不再把该端点作为有效输出，但主机与 Runtime 清单仍可查看，物理 USB 不受操作影响。
- [x] 对每个功能 Action 验证配置路径、Action 类型、suggest binding、最终 bound source、
      本地化名称、active 状态和当前值；未绑定、profile 不适用和设备未激活可区分。
- [x] 分别把方向 Action 绑定到一个 float 轴和一对 boolean 输入，验证输出范围、正负组合与用户
      配置的方向反转；接管/原点/主末端执行器只允许 boolean component，类型错误返回原始应用错误。
- [x] 修改绑定后验证只重建所需 OpenXR 会话，其他业务服务不重启；interaction profile change
      后绑定状态自动刷新。
- [x] 证明 `AbsolutePoseFrame` 不含 Action 或机械臂语义。
- [x] 合成无绝对位姿和有绝对位姿的设备，证明下游契约不因设备类型变化。
- [x] 重复回放同一记录得到相同输出，模拟数据不使用真实设备零偏。
- [x] 用记录中的来源时间、各层接收时间和序号核对采集→空间→运动→执行链；报告每层实测频率、
      覆盖帧、丢帧和延迟，确认没有第二个 10 ms 重采样器，也不把这些观察值变成停止条件。
- [x] 分别验收上、下、左、右、前、后空间移动。
- [x] 分别验收左旋/右旋水平圆弧和前部抬起/前部往下竖直圆弧。
- [x] 验证三个分量开关只影响对应分量。
- [x] 验证取消接管立即停止；重新接管后位置和姿态都使用新基准。
- [x] 验证确认原点只需一次触发、不重置姿态、不产生机械臂命令，并且页面刷新不清除节点原点；
      空间节点重启后页面如实显示未确认。
- [x] 验证关闭接管、输入端点消失和 OpenXR 明确报告所需位姿无效时不沿用旧样本；恢复后下一轮
      从新基准开始。

### 9.2 运动和执行

- [x] 验证空间控制、按 metadata 提交的手动关节、`start` 命名目标和 actuator 请求最终都生成
      同一种带 model revision 的动态数组 `ArmCommand`。
- [x] 验证相对/手动模式切换本身不运动；手动滑块从反馈初始化、拖动只预览、松开提交；切回
      相对控制后下一次接管使用新基准。
- [x] 用动态关节数量验证 `RobotModelInfo`、`MotionRequest`、`ArmCommand` 和 `ArmState` 往返；
      model revision 或 joint/actuator key 不匹配时返回具体契约错误，不截断、补零或按数组位置
      猜测另一型号。
- [x] 验证普通运动完整经历 request ID 对齐的 planning/executing/result，显式取消返回同一
      request ID；MoveGroup 的实际抢占语义未被第二套队列覆盖。
- [x] 验证普通轨迹期间 Servo 不再写同一手臂控制器，轨迹成功、失败和取消后均从最新反馈恢复；
      独立夹爪请求仍可执行且不触发手臂规划。
- [x] 验证接管开始读取到的 `primary_tool_active` 初始值不产生请求；StarArm motion 节点在之后
      false→true 提交 `gripper=1°`、true→false 提交 `gripper=90°`，每次
      `ToolActuatorRequest/Status` 的 request ID 和 actuator key 对齐，`ArmState` 只在执行节点
      反馈后变化。
- [x] 改变夹爪后执行起始位，确认目标为 `[0°,0°,-3°,0°,0°,0°]` 且夹爪保持原值；起始位和
      任意普通目标使用完全相同的 MoveIt 路径和结果。
- [x] 用 MoveIt 消息构造起始自碰撞接触对，验证仅本次实际接触对进入本次 ACM，然后重试同一个普通请求；没有
      直接写关节、专用起始位或永久允许碰撞路径。
- [x] 注入 Servo -1、1–6 状态，验证页面原样显示 MoveIt code/message；1/3/4 不进入项目冻结，
      只有用户指定的 code 6 冻结当前接管，-1、2、5 不被项目改写或锁存；恢复控制不需要重启服务。
- [x] 接管期间注入 code 6，验证只发布一次最新反馈保持命令，随后 Servo 输出不能进入
      `ArmCommand`；释放接管、清除 code 6 并重新接管后恢复。code 2/5、普通 MoveGroup 和夹爪
      不误走该分支。
- [x] 验证加载的 IK 插件、base frame、TCP link、URDF 关节范围、100 Hz 控制循环、5 ms IK、
      `30 rad/s²`、0.002 m 自碰撞接近值和无限奇异硬停都来自构建后的实际 MoveIt 配置，而不是
      网页或 Rust 中的副本；逐项对应迁移前配置和记录来源。
- [x] 验证软件执行、假串口执行共用同一输入，并且只发布一个 `ArmState`。
- [x] 验证连接假串口后只有 Monitor 更新 `hardware` 反馈。
- [x] 验证断开真机后保留最后实测状态，并从该状态继续软件反馈。
- [x] 验证 J1–J6 命令与 Monitor 反馈端到端同号，特别验证 J4 在 execution 节点没有额外
      取反或补偿；夹爪编码只在 UART 边界换算一次。
- [x] 验证同一七舵机命令按 0.1° 编码完全相同时不重复写入，任一编码值变化时立即写入；没有
      额外角度变化门限。
- [x] 验证连接流程先读不写、参数读取失败不影响连接、运行 I/O 错误释放旧句柄并重开同一端口、
      重连过程不发送起始位或夹爪命令。
- [x] 连续多轮运行厂家 Python SDK 交叉测试和假串口恢复测试，每轮结果一致；任何不一致都进入
      下一轮修正而不是标记为偶发。

### 9.3 Web 和 Compose

- [x] 四个 Web 服务分别通过格式、ESLint、TypeScript、单元测试和 Next.js 生产构建。
- [x] 验证四个应用使用共享 UI、契约和客户端包，没有复制第二套底层实现。
- [x] 用至少两份非 StarArm fixture 验证通用契约、网关和页面：一份关节数不是 6 且没有
      actuator，一份具有多个 actuator 和非串口连接字段；同一 Web 构建必须按 metadata 正确
      生成控件、请求、状态和模型标签，不修改源码或重新编译。
- [x] 在通用节点、`frontend/web/`、共享类型和网关中静态搜索 StarArm 型号、J1–J6、七执行器、起始位
      数组、J4 额外取反/补偿、舵机 ID 和 UART 参数；除协议 fixture 明确测试“不应依赖这些值”外均为失败。
- [x] 验证四个页面路径、静态资源、API 和 WebSocket 不会落到错误服务。
- [x] `web-tracking` 从主机硬件发现一直验收到 Action 实时值；`web-spatial` 从原点确认验收到
      相对输出；`web-motion` 从模式切换验收到运动结果；`web-arm-execution` 从软件反馈验收到
      执行连接和参数显示。每条用户路径都有浏览器级测试。
- [x] 验证网页显示的是服务端实际生效配置和请求结果；请求失败、节点离线或字段缺失时不保留
      看似成功的本地 UI 状态。
- [x] 验证反馈模型、目标预览、命令目标和执行后端实测分别可辨；元数据提供的标签开关、
      执行器编号和本次读取参数不会改变控制状态。
- [x] 验证模型颜色、观察平面位置、全部页面文字选中/复制、键盘操作和基础可访问性。
- [x] 四个网页分别独立构建、独立路由且没有跨页面私有 API 或浏览器状态依赖；用户生命周期按
      已确认规则只支持整套 `down/up -d`，不把局部停止/接续作为受支持操作。
- [x] 页面刷新不清除 OpenXR 节点状态、空间原点或机械臂状态。
- [x] Compose/Dora 探针重复执行整套启动、消息往返、日志查看和整套停止，确认记录的实际
      容器/进程边界与最终 `compose.yaml` 一致。
- [x] `docker compose up -d` 和 `docker compose down` 是完整系统的可用启停路径。
- [x] 从干净 `down` 状态启动后无需额外 `dora`、ROS、构建或清理命令；`down` 后没有遗留业务
      容器、Dora 子进程、ROS 进程、Runtime 进程或项目 socket。

### 9.4 全量质量检查

- [x] Rust：fmt、clippy `-D warnings`、单元测试和集成测试。
- [x] Python/ROS 2：语法、节点、消息和规划路径测试。
- [x] Web：格式、ESLint、TypeScript、单元测试和全部生产构建。
- [x] Dora 和 Compose：类型、dataflow、Compose 配置及完整纯软件闭环。
- [x] 仓库：`git diff --check`。
- [x] 功能完整性：第 3.1 节每一行都有实现位置、测试名称和运行证据；新发现能力已补表。
- [x] 每轮保存失败、原因、修改、相关测试和全量结果摘要；下一轮从 `docker compose down` 后的
      干净状态开始，不能复用上一轮进程状态证明通过。
- [x] 连续两轮全量测试与七类审查均无新修改后，再执行最终只读复核；只读复核发现任何遗漏就
      取消收敛结论并继续循环。

## 10. 真实设备验收

用户已授权在实现完成后执行真实输入设备和真实机械臂操作。本授权不要求现在用 `ref/` 旧链路
驱动设备；真机复测必须等新链路先通过第 9 节纯软件/假串口验收。USB、串口和设备命令按根
`AGENTS.md` 在沙盒外执行。

真机运动测试只有一个额外操作范围：每次运动不超过 `10°`。它只约束本轮验收操作，不进入
生产代码成为运行时门限。真机测试不再要求速度、加速度、起始位、姿态差、Servo 持续时间或
其他前置条件。用户已明确将 `3°–5°` 实测偏差视为正常观察值；它不进入代码，也不作为成功
或失败判定。

### 10.1 启动和只读连接

1. [x] 从 `docker compose down` 状态执行 `docker compose up -d`，保存 Compose 服务、Dora
       dataflow、ROS controller 和两个 StarArm 节点的启动日志；不运行 `ref/` 中旧服务。
2. [x] 先在未连接串口时完成一次手动关节、命名目标预览、末端执行器预览和记录回放，确认软件
       `ArmState`、model revision 和动态页面一致。
3. [x] 在宿主机读取串口设备路径和 udev/sysfs 基础信息，核对 Compose 内同一路径可见；记录
       权限、占用或不存在的原始结果，不执行位置命令。
4. [x] 在 `/arm-execution/` 选择 execution 节点提供的串口字段并连接。连接阶段只执行 Ping
       ID 0–6、内部参数和 Monitor；页面必须先显示七个实测位置、反馈来源和参数，不能因连接
       自动发送起始位、夹爪或保持命令。
5. [x] 保存连接瞬间的软件反馈、第一帧真机反馈和 model/execution revision；不比较起始位，也
       不使用软件/真机角度差作为连接条件。

### 10.2 小幅单步关节与执行器复测

1. [x] 对 StarArm J1–J6 逐个测试：以当前 Monitor 反馈为基准，只提交该关节不超过 `10°` 的普通
       `MotionRequest`，其余关节目标保持当前反馈。等待
       同一 request ID 的 MoveIt 最终结果，记录 `ArmCommand` 和 Monitor，然后提交原基准值。
2. [x] 每个关节回到本轮记录的原基准后才由用户点击下一关节；失败、取消或现场要求停止时保留
       当时真实状态，不自动重试、不切换方向、不返回起始位。
3. [x] 单独核对 J4：网页/URDF/MoveIt/驱动/Monitor 的数值端到端同号，execution 节点不取反也不
       增加补偿；实物运动方向由已修正 URDF 的关节轴表达。
4. [x] 对 `gripper` 从当前 Monitor 反馈做不超过 `10°` 的单步和返回，确认 ID 6 映射、方向、
       请求状态与模型反馈，且不触发 J1–J6 MoveGroup 规划。

### 10.3 最后一次 10° 真机复测

本项按用户最后给出的流程执行，不再扩张成所有空间方向、热插拔或故障注入矩阵；空间平移和
两种圆弧已经由确定性模拟数据经过同一 spatial → motion → execution 链路完成回归。

1. [x] 通过普通 `MotionRequest` 到达测试姿态 `[0°,0°,-20°,0°,0°,0°]`，记录同一 request ID
       的 MoveIt 结果和 Monitor；不建立真机测试旁路。
2. [x] 从测试姿态提交 J1 `+10°`，记录 `ArmCommand` 和 Monitor，再用普通请求返回测试姿态。
       每次运动都不超过用户指定的 `10°`。
3. [x] 主动断开串口，确认保留最后一帧 `hardware` 反馈、UART 停止读写、后续状态可以继续作为
       软件反馈；不自动发送起始位、夹爪或保持命令。
4. [x] 把本次目标、MoveIt 原始结果、轨迹点数、峰值和返回后的 Monitor 写入
       `docs/PROBES.md`。异常时回到纯软件全量回归后再复测，不能用角度门限掩盖原始反馈。

## 11. 最终迁移与复杂度审查

- [x] **最高优先：按方法逐项检查标准库以及质量合格、有测试依据的 npm、PyPI、crates.io 库和
      已采用框架是否已经提供同一能力；已有直接能力时删除自实现并使用库。只有库接口确实不
      满足当前调用方时才保留自实现，并在代码或 `docs/PROBES.md` 记录具体缺口；不得因为习惯、
      猜测性能或假设边界情况重复造轮子。**
- [x] 搜索通用生产代码、配置、页面和日志，清理输入设备的厂商、型号、固定编号和实体按键名。
- [x] 搜索除两个 StarArm 型号节点及其专属 fixture 之外的生产代码、类型、配置和页面，
      清理固定轴数、关节名、起始位、执行器数量、硬件 ID、传输类型和 StarArm 参数。
- [x] 删除 Unix Socket、旧 HTTP 控制旁路、旧单体采集/空间/执行组合和重复状态缓存。
- [x] 确认没有重新拆出独立模拟节点、独立 UART 节点或二者之间的反馈路由。
- [x] 确认生产代码、Compose、Docker 构建和运行时都不依赖 `ref/`。
- [x] 保留必要的协议追溯文档和底层驱动补丁，但不得传播到通用契约。
- [x] 检查坐标转换只存在空间节点、J4 没有任何额外取反/补偿、夹爪换算只存在
      StarArm execution 边界。
- [x] 检查每个新增结构、分支、配置和状态都有当前调用方及测试；没有则删除。
- [x] 检查是否为边界条件加入门限、限制、重试状态机或拒绝条件；未经用户明确要求的全部删除。
- [x] 逐句审查 TODO 和实现文档，删除只有“支持、处理、管理、可用”等结论而没有数据来源、
      用户操作、状态变化、失败表现或验收证据的表述。
- [x] 把实现过程中发现但原 TODO 未列出的高收益功能补入第 3.1 节和对应服务测试；不能以未写
      需求为由静默省略，也不能无调用方地扩展架构。
- [x] 每轮全量测试后都执行复杂度、重复、无意义代码、最佳实践、迁移遗留、文档和禁止新增
      门限七类审查，修正后重新开始完整一轮，直到满足第 1.4 和 9.4 节的收敛条件。
- [x] 任务结束前额外从全局视角重做一次全面审查，重点搜索为了无关紧要的边界条件而增加的
      状态、分支、重试、包装层或大段代码；能够直接返回原始错误的路径不得维护额外状态机。
      同时重新判断是否存在职责相同但更短、更直接、更易验收的实现。
- [x] 最终逐项核对本文和根目录 `AGENTS.md`，并检查实际 Compose 服务、Dora 图、页面路由和
      测试清单；没有可复现证据的项目不得标记完成。

## 12. 实施时必须复核的官方接口

- OpenXR Runtime/system 信息：[`XrSystemProperties`](https://registry.khronos.org/OpenXR/specs/1.1/man/html/XrSystemProperties.html)
  和 [`xrGetInstanceProperties`](https://registry.khronos.org/OpenXR/specs/1.1/man/html/xrGetInstanceProperties.html)。
- OpenXR 当前 profile 和最终绑定：[`xrGetCurrentInteractionProfile`](https://registry.khronos.org/OpenXR/specs/1.1/man/html/xrGetCurrentInteractionProfile.html)、
  [`xrEnumerateBoundSourcesForAction`](https://registry.khronos.org/OpenXR/specs/1.1/man/html/xrEnumerateBoundSourcesForAction.html)
  和 [`xrGetInputSourceLocalizedName`](https://registry.khronos.org/OpenXR/specs/1.1/man/html/xrGetInputSourceLocalizedName.html)。
- Monado 设备只读信息：[`libmonado`](https://monado.pages.freedesktop.org/monado/monado_8c.html)
  的设备数量、名称、序列号和属性接口；使用前核对镜像内实际版本。
- Dora 生命周期和节点启动边界：官方
  [架构说明](https://github.com/dora-rs/dora/blob/main/guide/src/concepts/architecture.md) 与
  [dataflow YAML 说明](https://github.com/dora-rs/dora/blob/main/guide/src/concepts/dataflow-yaml.md)。

上述链接用于实现阶段核对能力边界，不把当前文档理解替代为永久不变的第三方行为；依赖版本
变化时必须用探针和测试重新确认，再更新本 TODO 和实现。
