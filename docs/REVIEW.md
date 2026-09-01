# 最终审查与验收

## 审查结论

当前生产系统只有一条端到端链路。真实设备、模拟输入、真实相机和确定性 RGB-D 只在所属
节点的驱动或输入适配层不同，下游共用正式消息契约。仓库不包含完整第三方源码、历史参考树、
备用服务、兼容入口、测试专用规划路径或运行时模型覆盖。

架构和服务边界只在 [当前系统设计](SUMMARY.md) 维护；逐方法职责、采用的库和手写边界只在
[后端方法与依赖](BACKEND.md) 维护；型号事实只在
[StarArm-102 型号适配](STARARM-102.md) 维护。

| 审查项 | 最终状态 |
| --- | --- |
| 服务与消息 | Dora 节点按职责拆分；感知计算只与 `perception-node` 交互；不存在无消费者 DTO |
| 模拟与真机 | 输入适配后共用 spatial、motion、MoveIt、`ArmCommand` 和 execution |
| 感知与规划 | 原始点云和掩码不进入 MoveIt；MTC 只接收当前任务所需的结构化几何 |
| 坐标 | FK、IK、抓放、附着和模型元数据统一使用 `tcp_link` |
| 型号隔离 | StarArm-102 源码和夹爪清单集中在 `backend/devices/stararm-102`；计算服务只按请求资产 ID 工作 |
| 调度 | motion 只有一个 FIFO 和顺序 worker；没有通用状态机、并行规划器或隐藏恢复服务 |
| 配置 | 各服务只持久化自身 JSON；状态和反馈不写配置；串口只在用户主动刷新时枚举 |
| 前端 | 五个 Next.js 应用共用契约、网关客户端和 UI；业务页不复制后端几何或机械臂参数 |
| 依赖 | Rust 使用 Cargo Machete 核对无未使用 crate；前端包声明与实际 import 对齐 |
| 镜像 | 仅 `backend-base`、`frontend-base` 声明环境；三个应用 Dockerfile 只复制和构建工程，不存在 StarArm 专用 Dockerfile、按服务/语言分层或产物搬运层 |
| 生成物 | 夹爪资产由镜像构建自动生成；仓库只提交语义清单、正式测试资产和最终验收数据 |

没有新增运动门限、队列上限、确认流程、保护分支或失败降级。真实失败保留原始错误，不切换
模拟、不伪造场景，也不以额外规则猜测抓取姿态。

## 验收范围

- Rust：格式、Clippy（warnings 视为错误）、全部 workspace 单元测试和 Cargo Machete。
- Python 计算服务：Ruff、Pytest；运行镜像加载 YOLOE、GraspGenX 和 CPU PyTorch。
- Web：Prettier、ESLint、TypeScript、Vitest、五个 Next.js 生产构建和 Playwright。
- 部署：Compose 配置、全部镜像构建、完整 `down`/`up -d` 冷启动和健康状态。
- 全链路：输入、空间、普通运动、感知、抓放、模型资源、配置持久化和最终状态集成测试。
- 可视化：五个 Web 入口、执行模型、感知资源，以及 RViz/noVNC 的模型、场景与轨迹。

最终冷启动后，七个 Dora daemon、协调器、感知计算和五个 Web 应用持续运行超过 7 分钟，
未出现心跳超时、断联或重部署。Dora 关闭的是可选的跨 daemon 直连优化，跨容器数据继续走其
官方无损 daemon 转发路径；Compose 不再维护静态 IP 或 Zenoh 全连接列表。ROS
`arm_controller`、`hand_controller` 与 `joint_state_broadcaster` 均为 `active`，Servo 参数以
double 载入且进程持续存活。

依赖审查升级到 Node 24 LTS、pnpm 11.25、Next.js 16.3.3 及当时兼容的最新直接依赖。ESLint
保持 9.39.5、TypeScript 保持 6.0.3、Node 类型保持 24.x：分别受当前 Next/ESLint 插件 peer、
typescript-eslint peer 与 Node 24 运行时约束，未用忽略 peer 的方式强行升级。

测试点云保持 497 点并可在 RViz 独立显示，不生成 OctoMap。机械臂、抓取物、放置区域和刚性
地面统一使用 `base_link`；抓放完成后临时任务对象被清理，Servo 与同一 PlanningScene 继续
运行。当前型号 20 轮抓放结果见 [StarArm-102 型号适配](STARARM-102.md)。

最终实际结果为 Rust 106 项、Python 1 项、Vitest 11 项全部通过，Playwright 22 项通过、1 项按
设计跳过；五个前端生产构建、全部 Compose 镜像构建和 `software-flow` 端到端测试均通过。
