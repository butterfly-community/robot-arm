# 最终审查与验收

## 收敛结果

相机与场景处理只有一条生产链路：

`驱动适配器 → camera-node → CameraFrameBundle → scene-node → perception-compute → WorldScene → motion`

RealSense 与 simulation 只在驱动适配器不同。原始 RGB-D 不进入 ROS、Gateway 或浏览器状态流；
YOLOE/GraspGenX 只由显式任务触发；MoveIt 只接收结构化场景。旧节点名、旧配置文件、旧点云
topic、OctoMap sensor updater、标定子进程和兼容读取路径均已移除。

## 边界与复杂度审查

- `realsense-camera` 独占 librealsense、profile/option 发现、frameset 和 SDK `Align`；公共契约没有
  厂商类型。
- `camera-node` 拥有驱动生命周期、稳定身份、配置、节流和标定。同步 SDK 位于一个长期
  `spawn_blocking` worker；有界命令通道与 latest-value 通道不会积压视频帧。
- 实时彩色视频从同一 frameset 分流，经节点内 WebSocket 到浏览器；没有第二次采集、额外 node、
  Dora 大帧转发或后端视频编码。
- `camera-calibration` 进程内复用 OpenCV 5 的 ChArUco、PnP 和 hand-eye API；`scene-core` 只保留
  实例掩码内的标准针孔投影。没有手写深度注册、畸变或 IK。
- `scene-node` 只缓存最新原子帧并按请求异步调用计算服务；计算与 PNG 工作进入
  `spawn_blocking`，Dora 事件循环不执行同步模型或图像计算。
- 自然语言抓放位于现有 Next.js 感知应用。AI 只生成开放词汇提示词并从实际 `WorldScene` 选择
  ID；坐标、抓取姿态、碰撞、MTC 与执行仍是手动路径的原接口。
- 配置继续由各所有者节点通过 `json-config-store` 保存；网页没有配置副本，也没有中央配置服务。
- `cargo machete` 未发现无用依赖；迁移关键词、旧配置路径、空壳 crate 和重复服务入口检查无命中。

## 真机相机验收

RealSense D415 `924322061032` 的最终实测：

| 项目 | 结果 |
| --- | --- |
| 设备信息 | `RealSense D415`；固件 `5.12.7.100`；连接类型报告 `2.1` |
| 能力 | 62 个 profile；52 个当前可用；10 个 YUYV profile 明确置灰 |
| 测试 profile | 彩色 `424×240 BGR8 @ 60 FPS`；深度 `480×270 Z16 @ 60 FPS` |
| 采集 / 上送 | `61.97 FPS / 1.00 FPS`；主动略过帧单独计数 |
| SDK 对齐 | 输出彩色和深度均为 `424×240`，共享 `color_optical_frame` |
| 实时预览 | 同一 pipeline 连续收到 5 个 `bgr8` WebSocket 帧，不受 1 FPS 上送限制 |
| 收尾 | 验收后主动断开并取消选择；保存配置仍按稳定序列号保留 |

## AI 与抓放验收

服务端使用 `gpt-5.6-sol` 的结构化输出完成一次实际中文指令验收。模型生成的提示词包括具体描述
和通用视觉类别；YOLOE/GraspGenX 返回实际 `red block-0` 与
`gray container-1-interior`。两者通过本地场景存在性与抓取候选校验后进入既有 MTC 接口，最终为
`succeeded / pick and place complete`。测试前确认 execution 的 `connected=false` 且
`selected_endpoint=null`，因此本轮只执行软件反馈。

## 自动化验收

- Rust workspace runtime feature：138 项通过；Clippy `-D warnings` 通过。
- `camera-node` 编译与运行动态链接 `/opt/opencv5/lib/*.so.500`；OpenCV 5 算法测试通过。
- `cargo machete` 无未使用依赖；`cargo fmt --check` 与 `git diff --check` 通过。
- Python `perception-compute`：2 项测试与 Ruff 通过。
- 前端：Prettier、ESLint、TypeScript、18 项 Vitest 和 5 个 Next.js 16.3.4 生产构建通过；
  `pnpm peers check` 无问题。`@types/node` 保持与 Node 24 运行时一致，ESLint 9 与 TypeScript 6
  分别是当前 React/Import 插件和 `typescript-eslint` 声明支持的最高主版本，没有用忽略 peer
  warning 的方式强升 ESLint 10 或 TypeScript 7。
- Playwright：27 项通过、1 项按设计跳过；桌面和移动视口完成实际截图检查。
- Compose `software-flow` 通过：覆盖 simulation 相机、独立预览、两轮自动标定、YOLOE、
  GraspGenX、WorldScene、MTC 抓放、回工作位及配置恢复；相机重置按公开的“重置 → 选择 → 启用”
  生命周期验证，不依赖停止采集后的旧帧竞态。
- `docker compose config --quiet` 通过；根目录 `.env` 已确认被 Git 忽略且未跟踪。

## 保留的后续硬件事实

J4 恢复并由用户给出九组最终姿态后，再完成真实 D415 ChArUco 手眼标定。当前 simulation 已对同一
九姿态自动标定链路完成两轮可重复验收；未把缺少真实姿态伪装成已完成。
