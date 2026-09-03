# 最终审查与验收

## 结果

相机采集已经收敛为一条链路：

`驱动适配器 → camera-capture-node → CameraFrameBundle → perception-node`

RealSense 与 simulation 只在驱动适配器不同。原始 RGB-D 不经过 ROS，YOLOE/GraspGenX 只由
用户明确请求执行，MoveIt 只接收结构化 `WorldScene`。仓库中已移除 ROS RealSense 驱动、
`cv_bridge`、OctoMap sensor updater、旧 simulation 源和对应补丁/配置，没有保留备用路径。

## 配置与设备身份审查

相机配置参考手柄采集节点的原则，但按相机语义保存：

- RealSense 来源键使用设备序列号，例如 `realsense:924322061032`，不使用枚举索引、USB 口或
  `/dev` 路径；simulation 使用代码定义的稳定来源键。
- profile 键只含 stream、宽高、格式和 FPS，例如 `depth:480x270:z16le:60`，不含 SDK stream
  index。
- librealsense 参数键使用稳定 sensor 名称与 option 编号，不含 sensor 枚举顺序。
- 后端按来源保存 profile、上送 FPS、用户修改的可写参数和最后一次能力快照。设备离线或 profile
  消失时仍返回配置，但标记为不可用；网页置灰而不删除。
- 当前选中相机和 streaming 是运行态，不持久化。系统重启后默认未选择，用户重新选择同一设备时
  恢复其保存配置。

当前仅有的真实相机驱动是 RealSense，而该设备提供硬件序列号。以后加入没有序列号的驱动时，
驱动适配器必须像 SDL3 手柄适配器一样提供平台稳定别名或厂商稳定标识，不能把运行时索引暴露为
持久身份。

## 数据量与调度审查

采集 FPS 由两路所选 profile 决定，上送 FPS 独立配置且不超过共同采集 FPS。capture 持续读取
完整 frameset 以避免 SDK 队列反压，只把最接近每个上送周期的完整 RGB-D 帧束送入 Dora。
队列保持 `queue_size: 1`/`drop_oldest`。设备序列缺口计入 dropped；有意降低上送率计入 skipped，
二者不混淆。没有额外线程池、视频转码、ROS 消息转换或第二套缓冲层。

## 真机验收

RealSense D415 `924322061032` 的实际枚举结果：

| 项目 | 实测结果 |
| --- | --- |
| profile | 62 个；52 个可用，10 个 YUYV 由当前 Rust 路径明确置灰 |
| 驱动参数 | 30 个实际 option；29 个可写 |
| 测试采集 | 彩色 `424×240 BGR8 @ 60 FPS`；深度 `480×270 Z16 @ 60 FPS` |
| 实测采集/上送 | `60.2388 FPS / 0.9875 FPS`（配置上送 1 FPS） |
| 统计 | 设备缺帧 4；主动略过 2308；两类统计独立 |
| 下游 | perception 收到实际宽高、格式、内外参和深度比例；未自动调用模型 |
| 重启 | 统一重启后未选择、未采集；D415 保存配置仍存在并能按稳定键恢复 |

## 自动化验收

- Rust workspace：132 项通过；Clippy `-D warnings` 通过；`cargo machete` 无未使用依赖。
- RealSense runtime feature：5 项通过；camera-capture 与 runtime feature 组合测试通过。
- Python perception-compute：2 项通过；Ruff 通过。
- 前端 Vitest：11 项通过；格式、ESLint、TypeScript 与生产构建通过。
- Playwright：27 项通过、1 项按设计跳过；覆盖相机配置保存、取消选择和浏览器重新加载恢复。
- Compose `software-flow`：重复执行通过，覆盖唯一控制、感知、抓放与相机状态链路。
- Docker 清理：删除 208.9 GB 历史构建缓存和 13.84 GB 已替换的 backend-base r1；保留
  运行中镜像、验证过的 backend-base r2、frontend-base、模型及配置卷。

## 方法级与复杂度审查

- `realsense-camera` 独占 SDK、FFI、profile/option 发现和 frameset 读取；公共消息没有厂商类型。
- `camera-capture-node` 只负责发现、配置、stream 生命周期与采样调度，不进行图像计算或规划。
- `perception-node` 只消费统一 bundle；真实/模拟没有条件分支进入不同业务流程。
- 配置复用 `json-config-store`，由所有者节点持久化；没有中央配置服务和前端配置副本。
- 用户修改的驱动参数按 patch 语义保存，未修改的参数继续使用设备当前值，避免复制整份厂商默认值。
- 离线能力通过最后一次快照表达，避免第二套“历史设备”消息或迁移服务。
- 上送调度是单个纯函数；没有为 60→1 FPS 引入额外队列、定时任务或采集线程。

最终 `git diff --check`、过时关键词、运行时索引、生成缓存和空目录检查均应在提交前为干净结果。
