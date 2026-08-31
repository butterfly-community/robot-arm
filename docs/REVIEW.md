# 最终审查结果

## 结论

当前实现只有一条端到端业务链路。真实设备、模拟输入、真实相机和确定性测试场景只在所属
节点的输入或驱动适配层不同；它们输出统一契约，之后共用空间转换、感知结构化、MoveIt、
`ArmCommand` 和 execution。没有兼容入口、备用服务或模拟专用运动链路。

感知、标定、抓放、RViz/KasmVNC、统一 TCP 和异步抓放请求均已接入。OpenCV 标定使用 Rust
`opencv` crate 调用 OpenCV 5；Python 只保留在必须承载 Ultralytics/PyTorch 的远程计算服务。

## 代码边界审查

| 范围 | 位置 | 边界 |
| --- | --- | --- |
| 生产感知几何 | `backend/crates/perception-core` | 深度反投影、掩码几何、场景结构化；不导出测试源 |
| 感知测试源 | `backend/nodes/perception/src/test_source.rs` | 生成 RGB-D、测试相机参数和 497 点云；只输出正式输入契约 |
| 输入模拟 | `backend/nodes/controller-input/src/simulation.rs` | 生成设备无关 Action；仍走绑定、空间、motion、execution |
| 离线测试工具 | `tools/perception` | 资产生成、计算服务探针和 ignored artifacts |
| 标定运行时 | `perception-calibration` Rust binary | OpenCV 5 ChArUco、PnP、robot-world/hand-eye 和调试图 |
| 模型计算 | `perception-compute-service` | YOLOE 实例分割；不读取深度、不连接 Dora/ROS/MoveIt |

普通单元测试使用 Rust `#[cfg(test)]`、Python `tests/`、Vitest 和 Playwright 的标准隔离方式，
不会进入发布二进制。测试源虽然可在运行时由用户选择，但其生成逻辑与生产核心模块分开，且
没有复制下游处理。

## 唯一链路与迁移审查

- `controller-input → spatial-transform → stararm-102-motion → MoveIt/Servo/ros2_control →
  ArmCommand → stararm-102-execution → ArmState` 是唯一控制链路。
- `ROS RGB-D/确定性测试源 → perception-node → perception-compute-service → perception-node
  → WorldScene/ROS 感知 → stararm-102-motion` 是唯一感知链路。
- 旧 controller 深度输出、motion 点云桥、Python motion、Python/C++ 标定桥、`model.json`、
  第二份模型目录和直接 `link6` 业务目标均已移除。
- 感知计算可以远程部署，但只与 `perception-node` 交互；CPU/CUDA 不改变协议和下游链路。
- 未启用相机或测试源时不发布空场景；错误直接返回，不切换模型或伪造数据。

## 坐标与 TCP 审查

`tcp_link` 是唯一业务 TCP，位于两侧夹爪尖端中心。MoveIt group tip、FK、IK、目标位姿、
标定观测和模型元数据都引用 `stararm_102_model::TCP_FRAME`。`link6` 只保留为结构
链接，73.13 mm 只用于明确的圆弧演示几何。相机内参、对齐深度、外参和 `base_link`
转换均只处理一次。维护规则见 [StarArm-102 末端坐标](TCP.md)。

## 复杂度与重复审查

- motion 只保留 FIFO 和一个顺序 worker；前一个运动未结束时后一个等待，没有通用状态机、
  并行规划器、队列门限或永久错误状态。
- 抓放由 `manipulation-core` 生成以打开夹爪开始的八步线性计划，型号 motion 负责 MoveIt 实现；没有额外编排
  服务或第二套夹爪命令。
- 网关对长时间抓放返回 `202 Accepted`，最终结果继续从既有 manipulation 状态流返回；没有
  HTTP 超时状态机或重复执行路径。
- 结构化场景和点云用于目标计算与可视化，不进入 MoveIt 环境碰撞场景；唯一世界碰撞体是
  Z=0 刚性地面。
- execution 是模型资源和硬件差异的唯一所有者；网页和 motion 不维护另一份参数。
- 没有为低收益边界条件增加包装层、恢复服务、降级分支或用户未要求的保护门限。

## 方法级库审查

| 能力 | 使用的库/标准实现 | 保留手写部分 |
| --- | --- | --- |
| 消息与配置 | Dora、Arrow、Serde、json-config-store | DTO 与请求关联 |
| 输入设备 | SDL3、hidapi、fusion-ahrs、one_euro_filter | NOLO 报告适配和 Action 绑定 |
| 几何数学 | nalgebra、image | 场景语义、型号工具几何 |
| 标定 | `opencv` crate、OpenCV 5 | 会话、样本和持久化 |
| 识别分割 | Ultralytics YOLOE、PyTorch、FastAPI/Pydantic | HTTP DTO 归一化 |
| ROS 与规划 | r2r、MoveIt、Servo、ros2_control | ROS 消息边界和抓放步骤执行 |
| 串口 | serialport、fashionstar-uart | 舵机 ID 与型号映射 |
| Web/3D | Next.js 16、React、Radix、Three.js、Axum | 页面业务组合 |
| 远程 GUI | RViz2、KasmVNC、Openbox | 项目 RViz 配置和最短启动脚本 |

未自制 IK、碰撞检测、轨迹插值、标定求解、实例分割、VNC、数据库、状态机框架或文件摘要
实现。最终 API 核对采用 OpenCV 5 当前 ChArUco/Calib3d API；依赖锁文件没有可用的兼容更新。
ESLint 10 和 TypeScript 7 没有升级，因为当前 Next 插件和 typescript-eslint 的 peer 约束尚不
兼容；这不是保留历史运行路径。

## 验收结果

- Rust workspace：格式、Clippy（warnings 视为错误）和全部单元/文档测试通过。
- 标定 helper：OpenCV 5 合成 hand-eye 求解与生成 ChArUco 检测自测通过。
- Python 计算服务：Ruff 和 Pytest 通过；运行镜像使用 OpenCV Python 5.0.0.93。
- 前端：Prettier、ESLint、TypeScript、Vitest、四个 Next.js 16.3.3 生产构建通过。
- 浏览器：Playwright 21 项通过。
- Compose：配置、全量镜像构建、冷停止/启动和服务健康检查通过。
- 软件全链路：输入、空间、普通运动、抓放、模型资源、配置和最终状态集成测试通过。
- 环境碰撞：点云、识别物、障碍物和已抓物体均不写入 MoveIt；规划只保留自身碰撞和用户
  要求的 Z=0 刚性地面，抓放仍使用结构化场景计算目标。
- 感知：497 点测试云继续通过标准 PointCloud2 发布，可在 RViz 独立显示，不生成 OctoMap。
- 运行态场景：冷启动后的完整八步抓放通过；流程结束后 `collision_objects` 中只有
  `ground`，`attached_collision_objects` 和 OctoMap 为空，Servo 进程保持存活并检查同一地面。
- 执行可视化：抓放状态携带选中的物体、放置区 ID 和 motion 已计算的两个目标坐标；执行页
  只画点，不镜像整份 `WorldScene`，也不重复计算场景几何。
- 测试场景坐标：机械臂、抓取点和放置点均直接使用 `base_link`；确定性立方体与置物筐的
  底面同为 Z=0，两者均保留 0.04 m 高度；底座模型最低点和渲染地面也统一为 Z=0。抓取目标
  是立方体顶面 Z=0.04 m，放置目标是筐顶加半个立方体高度 Z=0.06 m。
- 规划性能：结构化感知场景不进入碰撞世界；完整抓放无需对识别掩码或点云做碰撞查询。
- TCP：运行时 `link6 → tcp_link` 为零平移、单位旋转，所有业务调用使用 `tcp_link`。

最终实现没有新增用户未要求的运动门限、确认流程、队列限制、拒绝条件或隐藏保护数值。
