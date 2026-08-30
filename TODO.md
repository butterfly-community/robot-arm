# Rust 运动编排与 ROS 包瘦身 TODO

## 1. 当前阶段

- [ ] 本文件只定义迁移目标、边界、实施顺序和验收方法。
- [ ] 在用户明确要求开始实现前，不修改生产链路。
- [ ] 实施时只保留一条端到端业务链路，不保留 Python 兼容节点、备用 ROS 接口或新旧实现
      并行运行。

## 2. 任务目标

将当前 `stararm_102_motion_node` 中与 ROS 无关的业务逻辑迁移到 Rust，使 ROS 包最终只负责
启动和配置 MoveIt、MoveIt Servo、ros2_control 及其标准接口。

迁移目标：

1. 用 Rust 的显式类型替代多个布尔值、字符串状态和回调之间的隐式组合。
2. 用一个线性异步过程表达“同步控制器 → 暂停 Servo → 规划 → 执行 → 恢复 Servo”。
3. 保持空间转换、机械臂目标合成、MoveIt 规划和真实执行之间的边界清楚。
4. 删除迁移后的 Python 业务代码、重复数学实现和无消费者状态。
5. 迁移后业务代码应更少、更直，不得为了状态机概念增加代码量和分支数量。

## 3. 固定约束

### 3.1 唯一业务链路

```text
controller-input
→ spatial-transform
→ stararm-102-motion（Rust）
→ MoveIt / Servo / ros2_control
→ ArmCommand
→ stararm-102-execution
→ ArmState
→ stararm-102-motion
```

- [ ] 模拟和真机继续使用同一份 `ArmCommand` 与 `ArmState` 契约。
- [ ] 模拟/真机差异只存在于 `stararm-102-execution` 的驱动适配层。
- [ ] 不增加模拟专用 motion、模拟专用 MoveIt、Python 兼容路径或前端直接运动路径。
- [ ] 最终仍保留 `stararm-102-motion-node` 这一对外节点身份，避免无收益的前后端接口迁移。

### 3.2 禁止新增限制

- [ ] 不新增运动门限、反馈门槛、确认流程、保护距离、超时策略或自动重试次数。
- [ ] 保留 MoveIt、Servo 和 ros2_control 自身返回结果，不在 Rust 重复一套限制。
- [ ] 库中若存在固定超时、队列上限或自动拒绝条件，采用前必须识别，不能把隐含限制当作
      项目设计。

### 3.3 控制复杂度

- [ ] 不引入层级状态机框架、状态机 DSL、Event/Effect 通用框架或工作流引擎。
- [ ] 不为理论上的异常顺序建立没有实际消费者的状态和恢复分支。
- [ ] 可以直接返回 ROS/MoveIt 错误的地方直接返回原始错误。
- [ ] 一个抽象只有在减少重复、隔离外部库或明显改善测试时才能保留。

### 3.4 不在本任务范围

- [ ] 不重新设计网页、动作绑定、生成式测试场景或可视化。
- [ ] 不改变现有动作名称、正负语义、空间缩放、默认位、测试位和夹爪范围。
- [ ] 不调整 MoveIt/Servo/ros2_control 参数、URDF 运动学或串口舵机参数。
- [ ] `controller-input` 和 `spatial-transform` 只在边界审查发现现有职责放错时修改；不得借
      Rust 迁移重写已经正确工作的采集和空间链路。
- [ ] execution 除接收模型目录职责和适配必要契约外，不改变模拟/真机执行行为。

## 4. 节点职责边界

### 4.1 `controller-input`

继续负责设备发现、能力声明、输入绑定、原始按钮/轴/位置/姿态采集、驱动内部必要的姿态融
合和生成式测试输入。

不得负责机械臂 TCP 目标、MoveIt 状态、机械臂型号、关节数或夹爪几何参数。

### 4.2 `spatial-transform`

继续负责所有设备无关的空间处理：

- 原点确认和坐标基准。
- 设备空间位置、姿态向人体/工作空间坐标的转换。
- 输入动作的积分、缩放和正负语义。
- 空间来源与按钮/轴来源的仲裁。
- 输出设备无关的平移、旋转和复合动作增量。

`spatial-transform` 输出的是空间控制增量，不是某台机械臂的最终 TCP Pose。它不得读取或
保存：

- `base_link`、`link6` 等 StarArm-102 frame 名称。
- 当前机械臂 TCP。
- `PIVOT_TO_TCP_M` 等末端结构尺寸。
- URDF 关节、MoveIt group 或规划器参数。

### 4.3 `stararm-102-motion`（Rust）

负责：

- Dora 输入输出和公开请求结果。
- 当前 `ArmState`、活动请求和相对控制会话。
- 将空间节点输出的设备无关增量应用到当前机械臂 TCP。
- 使用 StarArm-102 的 TCP frame、工具枢轴和末端几何合成目标 Pose。
- 普通运动的同步、暂停 Servo、规划、执行、取消和恢复流程。
- 接收 ros2_control 关节目标并合并成完整 `ArmCommand`。
- 夹爪目标和机械臂目标在同一请求生命周期中的编排。
- 现有 motion/actuator 状态和运动模式配置持久化。

不得负责原始手柄姿态、按键映射、零偏、滤波、设备坐标转换、串口、模拟执行，或自行实现
IK、碰撞检测、轨迹规划和轨迹插值。

### 4.4 ROS 包

最终只保留：

- URDF/SRDF、运动学、规划器、控制器和 Servo 配置。
- `robot_state_publisher`、`ros2_control_node`、`move_group`、`servo_node` 和 controller
  spawner 的启动描述。
- 启动 Rust `stararm-102-motion-node` 所需的最薄启动入口。

最终删除 ROS 包中的 Dora 请求分发、状态聚合、JSON 配置、相对控制会话、TCP 目标数学、
普通运动生命周期、模型资源服务、手写线程池和跨回调业务锁。

### 4.5 `stararm-102-execution`

- [ ] 继续负责模拟/真机统一执行和反馈。
- [ ] 接收当前 Python `ModelCatalog` 中与型号相关的模型信息及资源目录职责。
- [ ] 模型信息、命名目标、关节/夹爪描述和可视化资源由 execution 节点发布。
- [ ] motion 和前端消费 execution 发布的同一份模型信息，不维护第二份目录。
- [ ] Web Gateway 只转发模型信息和资源，不解析 URDF 或复制模型参数。

## 5. Rust 运动状态表达

### 5.1 小型显式状态

使用普通 Rust 枚举，不采用状态机框架：

```rust
enum MotionState {
    WaitingForFeedback,
    Ready,
    Running {
        request_id: String,
        phase: MotionPhase,
    },
}

enum MotionPhase {
    Planning,
    Executing,
}
```

- [ ] `Planning` 覆盖控制器同步、暂停 Servo 和 MoveIt 规划，不为页面不需要的内部步骤增加
      公开状态。
- [ ] `Executing` 只在规划成功并取得非空轨迹后进入。
- [ ] 成功、失败和取消是请求结果，不作为会锁住后续控制的永久状态。
- [ ] `manual/relative` 是节点上下文，不与运动阶段组合成更多状态。
- [ ] 相对控制会话只用一个小结构保存 session、anchor 和 target，不建立第二套状态机。

### 5.2 单一线性运动过程

每个普通运动只进入一个异步函数：

```text
确认有 ArmState
→ 必要时同步 ros2_control 当前反馈
→ 暂停 Servo
→ 请求 MoveIt 规划
→ 执行规划轨迹
→ 恢复 Servo
→ 发布最终结果
```

- [ ] 任一步返回错误后仍完成已经需要的 Servo 恢复，然后回到 `Ready`。
- [ ] 第二个普通运动只返回“已有运动正在执行”，不能覆盖活动请求。
- [ ] 取消只取消当前 MoveIt goal，由同一个运动函数完成清理和最终结果。
- [ ] execution 反馈 generation 变化时只标记控制器需要重新同步；motion 不判断
      `hardware/simulation`，也不传播设备类型分支。
- [ ] 不保留永久 `Fault` 状态；下一次请求可以重新进入同一流程。

## 6. 库选型

### 6.1 采用

- [ ] 使用现有 `dora-node-api` 处理 Dora 契约和节点事件。
- [ ] 使用 Dora 官方原生 Rust ROS 2 API/`dora-ros2-bridge` 生成 ROS topic、service 和 action
      类型，避免手写 DDS/CDR 或保留 Python rclpy 业务适配器。
- [ ] 使用现有 `tokio` 执行线性异步 ROS 调用和取消，不自行实现线程池。
- [ ] 使用现有 `nalgebra` 完成必要的 TCP `Isometry3`/四元数组合。
- [ ] 使用现有 `serde`、`thiserror`、`json-config-store` 完成契约、错误和配置。
- [ ] 模型目录迁移使用 `urdf-rs` 解析 URDF；实际需要时再分别使用 `walkdir`、`sha2`、
      `mime_guess`，不手写这些通用能力。

### 6.2 不采用

- [ ] 不使用 `statig`、`rustfsm` 或其他状态机 DSL：当前流程是线性的。
- [ ] 不使用 `dora-moveit2`：它会引入另一套 IK、碰撞、规划和执行实现。
- [ ] 不同时引入 `rclrs` 和 Dora ROS 2 bridge；最终只保留一套 Rust ROS 客户端。
- [ ] 不使用 YAML 动态 action bridge 承担普通运动；其固定超时、并发限制和取消能力不符合
      当前要求。

### 6.3 ROS 2 库探针

正式迁移前做可删除的最小探针，确认当前锁定的 Dora 版本可以：

- [ ] 订阅 `/joint_states`、控制器命令和 Servo 状态。
- [ ] 发布 Servo Pose、关节状态代理和夹爪轨迹。
- [ ] 调用 Servo、controller manager 和 MoveIt planning scene services。
- [ ] 发送、取得结果并取消 `/move_action` 和 `/execute_trajectory` action goal。
- [ ] 生成 `moveit_msgs`、`control_msgs`、`controller_manager_msgs`、`geometry_msgs`、
      `sensor_msgs` 和 `trajectory_msgs` 类型。
- [ ] 与当前 ROS 2 RMW 实现完成发现、请求关联和 action 结果关联。
- [ ] 完成后删除探针代码和产物；生产中不保留备用客户端。

如果库本身无法覆盖必需接口，应先更新本 TODO 和库选择，不能一边保留 Python 路径一边新
增半套 Rust 路径。

## 7. 实施步骤

### 7.1 当前行为基线

- [ ] 列出 Python ROS 节点全部输入、输出、topic、service、action 和公开状态字段。
- [ ] 将普通运动、相对控制、夹爪、控制器同步和模型资源测试按职责分类。
- [ ] 记录默认位、测试位、准备相对控制、取消和规划失败的当前结果。
- [ ] 保留现有 `-7 PREEMPTED` 竞态作为迁移回归测试。

### 7.2 共享契约

- [ ] 在 `robot-arm-messages` 复用现有 Motion/Arm/Control 契约，不复制第二套 DTO。
- [ ] 只在确实缺少关联信息时补充请求 ID、ROS 操作结果或反馈 generation。
- [ ] 公共契约不出现 MoveIt goal handle、具体 ROS 客户端等内部类型。
- [ ] 空间输出继续表达设备无关增量，不改成 StarArm-102 专属 Pose。

### 7.3 Rust motion 节点

- [ ] 在 Rust workspace 增加 `stararm-102-motion-node`。
- [ ] 实现 Dora 事件循环、配置读取、公开状态和服务状态。
- [ ] 实现 `MotionState`、活动请求所有权和单个 `run_motion()`。
- [ ] 将 `motion_core.py` 中属于机械臂目标合成的部分用 `nalgebra` 迁移。
- [ ] 不迁移输入姿态转换、设备零偏或来源仲裁；发现后保留在 spatial。

### 7.4 ROS 2 接口

- [ ] 通过 Dora 原生 Rust ROS 2 API 接入当前所有 topic、service 和 action。
- [ ] 普通运动继续使用 MoveIt `MoveGroup` 规划和 `ExecuteTrajectory` 执行。
- [ ] 起始自碰撞处理继续属于同一次规划：取得碰撞对、构造本次 ACM、重新规划；不建立第二
      个运动入口。
- [ ] 当前 TCP 由 MoveIt/ROS 的实际机器人状态取得，不在 spatial 计算 StarArm FK。
- [ ] 相对控制开始时只建立一次 TCP anchor，后续增量相对该 anchor 合成目标。
- [ ] ros2_control 输出仍合并成一份完整 `ArmCommand` 后发给 execution。

### 7.5 模型目录

- [ ] 将模型信息、命名目标、关节范围、夹爪范围和资源 manifest 移到 execution Rust 节点。
- [ ] 使用 URDF 作为关节和 mimic 信息来源，不维护重复边界数据。
- [ ] motion 和前端消费 execution 发布的同一份模型信息。
- [ ] 删除 Python `model_catalog.py` 及其资源输出路径。

### 7.6 单次切换

- [ ] Docker 构建时把 Rust motion 二进制编译进 ROS motion 镜像。
- [ ] ROS launch 启动标准 ROS 节点和 Rust motion，不再启动 Python Dora motion。
- [ ] 更新 `dataflow.yml`，保持外部 node ID 和 Web API 不变。
- [ ] 同一次修改中删除旧 Python 生产入口，最终不得同时存在新旧 motion。

## 8. 删除与清理

最终确认无消费者后删除：

- [ ] `dora_motion_node.py`、`motion_core.py`、`moveit_backend.py`、`model_catalog.py`。
- [ ] 对应 Python 业务测试、入口点、依赖和缓存。
- [ ] Python 专属线程池、队列、锁和回调状态字段。
- [ ] 已迁移后仍残留在 ROS 包中的配置、Dora DTO 和模型资源逻辑。
- [ ] 文档中“ROS motion 节点承担业务编排”的旧描述。

不得保留 `legacy`、`compat`、`fallback`、`v1` 等迁移路径。

## 9. 验收

### 9.1 状态与流程

- [ ] 没有 `ArmState` 时保持等待，收到有效状态后进入 `Ready`。
- [ ] 同步完成前不会规划，规划完成前不会执行。
- [ ] 第二个请求不会覆盖活动请求 ID、目标或取消句柄。
- [ ] 规划失败、执行失败和取消均恢复 Servo 并允许下一次请求。
- [ ] MoveIt 返回 `PREEMPTED` 时保留原始错误，不把节点永久锁死。
- [ ] 控制器同步中的请求等待同一次同步完成后继续，不启动另一条路径。
- [ ] 相对控制结束后清除 anchor；下一轮使用新的当前 TCP。

### 9.2 空间边界

- [ ] spatial 测试覆盖设备位置、姿态、原点、动作积分和设备无关增量。
- [ ] spatial crate 和配置中不存在 StarArm frame、关节名、URDF 或工具结构尺寸。
- [ ] Rust motion 测试覆盖当前 TCP 加平移/旋转增量得到目标 Pose。
- [ ] 圆弧测试覆盖 StarArm 工具枢轴不变量，结构参数只出现在 StarArm 侧。
- [ ] 同一输入不得在 spatial 和 motion 重复缩放、积分或反转。

### 9.3 ROS 与运行链路

- [ ] ROS 类型生成、Rust 编译、topic、service 和 action 集成测试通过。
- [ ] MoveIt 规划、执行、取消、planning scene 和 Servo 恢复通过。
- [ ] 控制器同步后第一份 `ArmCommand` 从实际 `ArmState` 连续开始。
- [ ] 默认位、测试位和准备相对控制使用同一个 `run_motion()`。
- [ ] 未连接串口时软件反馈沿唯一链路驱动网页模型。
- [ ] 连接串口时同一 `ArmCommand` 驱动真机，网页根据返回的 `ArmState` 更新。
- [ ] Compose `down`、`up -d`、服务就绪和页面连接通过。
- [ ] 最后进行此前授权范围内的真机复测，不为验收新增运行门限。

### 9.4 代码量与复杂度

- [ ] 对比删除的 Python 与新增 Rust 业务代码；若方法、分支或总业务代码反而增加，重新简
      化后才能完成。
- [ ] Rust motion 只有一个普通运动入口和一个活动请求所有者。
- [ ] 不存在状态机框架、重复 ROS adapter、重复 Pose 数学或重复模型目录。
- [ ] 不存在只有一个调用点且没有隔离外部库价值的包装层。
- [ ] 所有新增状态都对应真实阶段或公开结果；删除为推测边界建立的状态。

## 10. 完成前循环审查

首次实现和测试通过后执行以下循环，发现问题后修复并从第一项重新开始：

1. [ ] 功能审查：普通运动、相对控制、夹爪、取消、模型信息和状态发布逐项核对。
2. [ ] 链路审查：确认模拟/真机、空间/姿态和 ROS/Rust 没有并行路径。
3. [ ] 复杂度审查：检查状态、分支、包装、线程、锁和错误恢复是否可以删除。
4. [ ] 重复审查：检查空间转换、目标 Pose、模型参数和请求校验是否处理两次。
5. [ ] 方法级造轮子审查：逐个新增方法确认成熟 crate 是否能更短地完成。
6. [ ] 限制审查：检查新增数值、超时、队列和拒绝条件，删除用户未要求的限制。
7. [ ] 迁移审查：全局搜索旧 Python 入口、旧消息源、旧文档和无消费者代码。
8. [ ] 测试复跑：Rust、ROS、前端、Compose、软件反馈和最终真机测试全部重跑。
9. [ ] 最后一次修改后，再执行一轮没有发现新问题的完整审查。

## 11. 最终交付

- [ ] 更新 `docs/BACKEND.md` 和 `docs/SUMMARY.md`，只描述最终边界和运行方式。
- [ ] 清除探针、过程记录、缓存、测试产物和空目录。
- [ ] 执行格式化、Clippy、Python/ROS 测试、前端测试和 `git diff --check`。
- [ ] 确认没有覆盖其他并行任务的修改。
- [ ] TODO 所有完成项都有测试、代码或最终审查证据后，将最终结论写入正式文档并删除本
      TODO，不保留全勾选的过程计划。
- [ ] 提交一次收敛后的最终实现，不提交中间迁移状态或已完成 TODO。
