# 系统设计

项目只有一条运行链路：

```text
controller-input-node
  → spatial-transform-node
  → stararm-102-motion-node
  → stararm-102-execution-node
  → controller-input-node（设备无关 Action 回馈）
```

`controller-input-node` 同时承载两个硬件输入适配层和一个模拟测试源，但不分裂业务流程：NOLO CV1 使用本地 Rust HID
协议，其他手柄使用 SDL3，并且只根据设备声明的轴、按钮、传感器和振动能力工作；带 IMU 的手柄（包括 PS4）与 NOLO CV1 都交给同一个
`fusion-ahrs` 实现。生产代码不维护手柄型号白名单，不按型号选择行为，也不提供型号专属默认绑定；型号和 USB 信息只用于发现页面展示。没有 IMU 能力时仍发布按键和轴。空间位置与姿态来源分别只列出声明对应能力的设备；例如两个仅有按键和轴的设备不会产生姿态候选，此时仍可把任一设备的按键或轴绑定为俯仰和水平圆弧 Action。只要某一分量已经选择绝对来源，空间节点就不再叠加该分量的 Action。每个 Action 和反馈能力仍独立选择设备，不受位姿来源选择影响。采集节点把组合位姿和跨设备聚合 Action 送入同一个空间转换节点。

空间位置来源、姿态来源、每个功能 Action 的输入设备与组件、每个 Action 回馈的目标设备与能力路径分别保存在 /config/controller-input.json；同一轮控制可以组合 NOLO、多个 SDL3 手柄和模拟输入。模拟数据也是 `controller-input-node` 的测试功能，输出同一消息契约，
不会另起模拟链路。

所有可修改配置都由 Compose 把宿主 `backend/config/runtime/` 挂载到容器 `/config`：采集节点保存
输入源和 Action 绑定，空间节点保存轴映射、比例、Action 速率、分量开关与原点，motion 节点保存
控制模式，execution 节点保存串口选择。文件缺失时生成默认配置；文件存在但损坏时明确失败。配置
更新先写盘再替换内存状态。实时位姿、控制会话、模拟启停、关节反馈、连接状态和错误不会持久化。

型号节点通过 `RobotModelInfo.named_targets` 发布“默认位”和“测试位”；前端遍历元数据生成按钮，
不写死机械臂型号、关节数量或测试位关节值。StarArm-102 的测试位是 J3=-20°。

夹爪使用连续 `primary_tool` 值：`0` 表示张开并对应 90°，`1` 表示闭合并对应 0°，中间值
线性插值。`joint7_left` 是 0°～90° 的主动关节，`joint7_right` 通过齿轮和 URDF `mimic=-1`
反向联动；两侧各运动 90°，不是单侧 180°。这个动作语义只在型号 motion 节点转换一次；模型、
MoveIt、网页、`ArmCommand`、UART 命令和 Monitor 反馈随后都使用同一个主动关节正向绝对角，不存在
第二次符号换算。J1–J6 与夹爪分别放在 `joints` 和 `tool_actuators` 中，只是为了让 IK 关节与工具
执行器保持清楚的模型语义；两者仍由同一
`ArmCommand`、同一 execution 节点和同一反馈状态执行。采集页负责把设备输入绑定为
`primary_tool`，手动夹爪目标与 J1–J6 一起位于运动页，执行页只负责连接、命令/反馈和舵机参数。
StarArm-102 FL 的夹爪舵机是 ID 6、RA8-U35H-M；位置命令为它填写 2000 mW，J1–J6 仍为
0 mW。这个值来自厂家 UART SDK 的功率限制示例；不是运行门限。execution 节点将 Monitor 的实际功率扣除 400 mW 空载区间后，通过设备无关的 action_feedback 输出 0～100 力度百分比；输入节点仅按独立反馈绑定路由到用户选择的能力。SDL3 运行时声明左右扳机反馈或整机振动能力，网页虚拟反馈则始终作为一个可选目标；选中后四个页面共享的小型可拖动圆环显示最新的 0～100 值。代码不假定 primary_tool 必须绑定扳机，也不在能力之间自动回退。`primary_tool_open` 是独立的按下沿 Action，可与连续 `primary_tool` 分别绑定；两者不启动整臂空间接管。

Compose 启动 Dora coordinator、五个 Dora daemon、MoveIt motion 服务、dataflow、四个前端与
统一 Web 入口。输入容器挂载主机 `/dev`、只读 udev/sys 信息，因此能够同时枚举 HID 和
SDL 控制器。SDL3 手柄按运行时声明的轴、按钮、传感器与振动能力接入，NOLO CV1 由同一节点的 HID 适配层接入；
两者输出统一消息。服务配置使用宿主 bind mount，不依赖 Compose 命名卷。

常用验收：

```bash
cargo test --manifest-path backend/Cargo.toml --workspace
pnpm --dir frontend typecheck
pnpm --dir frontend test
docker compose config
docker compose build
docker compose up -d
```

测试回放工具与产物统一位于 `tools/replay/`；原始排障数据在各网页的折叠排障区，默认视图使用
Three.js 和高信息密度指标展示位置、姿态与机械臂状态。
