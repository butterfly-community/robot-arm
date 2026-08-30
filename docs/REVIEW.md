# Rust 运动链路迁移审查结果

## 最终结论

`stararm-102-motion-node` 的业务实现已经从 ROS Python 包迁移到 Rust。运行时只有一条链路：

```text
controller-input
→ spatial-transform
→ stararm-102-motion（Rust）
→ MoveIt / Servo / ros2_control
→ ArmCommand
→ stararm-102-execution
→ ArmState
```

模拟与真机都消费同一份 `ArmCommand`，并通过同一份 `ArmState` 返回；差异只保留在 execution
节点的驱动适配层。ROS 包只剩标准节点的 launch/config，不再包含 Dora 分发、目标数学、模型
资源、请求生命周期或状态聚合。旧 Python motion、模型目录、MoveIt adapter 和对应测试已删
除，没有兼容入口或备用流程。

模型目录现在由 execution 节点通过共享 `stararm-102-model` crate 发布。该 crate 使用最终 URDF
读取关节范围，并用 `walkdir`、`sha2` 和 `mime_guess` 生成、校验和返回资源；motion 和网页只
消费这一个目录。StarArm-102-FL 的 frame、关节顺序、默认位、测试位、夹爪行程和工具几何只
存在于型号 crate/motion/execution 边界，没有进入采集、空间或前端常量。

## 运动与反馈语义

- 普通运动进入一个 `VecDeque` FIFO，唯一 MoveIt action worker 逐个处理。下一项与当前项冲突
  时只等待，不设置队列上限，也不返回“已有运动”错误。
- 单项流程固定为“同步控制器 → 暂停 Servo → 规划 → 执行 → 恢复 Servo”。没有层级状态机、
  工作流框架、永久 Fault 状态或模拟/真机分支。
- 业务取消只停止继续采纳该请求的结果；已经提交给 MoveIt 的动作自然结束后才执行下一项。
  机械臂急停是物理断电，不由 motion 节点伪造。
- 起始自碰撞仍只在同一次规划失败后查询实际碰撞对，并把这些 link 对加入该次重试的 ACM；
  没有全局解锁状态、恢复服务或第二条规划路径。
- 夹爪位置和力度是两个不同事实。软件反馈可以准确表示夹爪目标角度，但不能证明真实接触
  力；真机力度来自 ID 6 的 Monitor 功率。真机稳定反馈为 `0` 是有效样本，表示扣除已确认的
  空载功率后没有检测到负载，并不表示“没有反馈”。夹爪张开或收起过程中可产生非零真实反
  馈，稳定后是否为零取决于负载；模拟值不得代替该结论。

## 复杂度、重复与迁移审查

- 删除了 1,620 行 Python 生产业务和 389 行 Python 测试。新增 Rust 生产代码约 1,870 行；
  增量来自显式的 r2r topic/service/action 类型边界、模型资源校验和编译期契约，不是状态机层。
  审查后没有为了压缩行数重新引入动态 Python、通用工作流或第二个 ROS adapter。
- 旧实现把运动生命周期分散在大量回调和布尔字段中；新实现只有一个活动请求所有者、一个
  FIFO 和一个顺序 action worker。第二轮审查删除了无意义的可变借用，未发现可安全删除的单
  调用包装、重复 Pose 计算或无消费者状态。
- TCP 目标只在型号 motion 的 `core::target_pose()` 合成；空间节点只输出设备无关增量。
  ros2_control 的 arm/hand 稀疏命令只在 `merge_controller_command()` 合并一次。JSON 中表示
  非本控制器槽位的 `null` 会恢复成 `NaN` 占位并保留七槽形状，不会再丢失夹爪或关节槽位。
- execution 是模型信息和模型资源的唯一发布者；motion 不再响应资源请求，Web Gateway 只转
  发。全局搜索未发现 `model.json`、旧 Python 入口、`transforms3d` 运行依赖或并行模型目录。
- 就绪状态读取 r2r publisher 的实际订阅数，避免 DDS 尚未发现控制器时过早报告可用；这只是
  当前连接事实，没有等待时限、重试上限或新的运动拒绝条件。

## 方法级造轮子审查

| 能力 | 采用实现 | 项目手写边界 |
| --- | --- | --- |
| Dora 消息 | `dora-node-api`、Arrow、`serde` | 请求关联和业务状态发布 |
| ROS 2 | `r2r` 标准 topic/service/action | MoveIt JSON 消息组装与结果映射 |
| 运动学 | MoveIt、Servo、ros2_control | 将设备无关增量组合成型号 TCP 目标 |
| 向量/四元数 | `nalgebra` | 型号工具枢轴与动作语义 |
| URDF | `urdf-rs` | 型号命名目标和显示元数据 |
| 模型资源 | `walkdir`、`sha2`、`mime_guess` | manifest 契约和路径归属 |
| 配置 | `json-config-store`、`serde` | 控制模式字段 |
| 串口 | `serialport`、`fashionstar-uart` | StarArm-102 舵机 ID 与命令映射 |

没有引入状态机 crate、手写线程池、DDS/CDR、IK、碰撞检测、轨迹插值、JSON 配置存储或文件
遍历/摘要实现。r2r action client 必须由拥有 ROS node 的线程持续 spin，因此保留一个标准线程
和 `std::sync::mpsc`；改成 Tokio 或状态机库只会增加 runtime、桥接和取消分支。

## 验收证据

- Rust workspace：格式、Clippy（warnings 视为错误）和 94 项单元测试通过。
- 前端：格式、Lint、TypeScript、Vitest 及 Next.js 16.3.3 的四个生产构建通过。
- 浏览器：Playwright 19 项通过，包含命名目标、独立夹爪命令、页面状态和绑定流程。
- 软件全链路：Compose 完整启动后集成测试通过；普通运动、FIFO、准备相对控制、软件反馈、
  模型资源和配置链路均走正式 API。
- 真机全链路：最终镜像的反馈源为 `hardware`；J1 从 0.3° 命令到 5.3°，反馈 5.0°，实际变化
  4.7°；返回命令成功后反馈 0.5°，相对测试前误差 0.2°。两次 MoveIt 规划/执行均成功，随后保留真机
  连接，没有遗留测试程序或配置。

最终代码没有新增用户未要求的运动门限、确认流程、队列限制、保护分支或超时策略。保留的
数值均为迁移前已确认的模型、驱动、MoveIt/Servo 配置或既有动作参数。
