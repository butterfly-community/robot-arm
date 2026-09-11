# Robot Arm Services

本仓库是一套前后端分离的 Dora 机械臂系统。设备适配、空间变换、相机采集、感知、运动、
执行与 Web 网关各自只有一个明确边界；真实设备和 `simulation` 在适配层之后使用同一消息契约。

- `backend/`：Rust/Dora 节点、公共 crate、感知计算服务，以及 ROS 2/MoveIt 型号适配。
- `frontend/`：五个 Next.js 应用与共享 UI、契约和可视化包。
- `compose.yaml`、`dataflow.yml`：统一部署和唯一数据流。
- `docs/`：当前架构、部署、型号参数与验收边界；不维护排查流水账。

## 启停

在仓库根目录统一启停，不维护单个节点的独立运行状态：

```bash
docker compose up -d --no-build
docker compose down
```

应用代码变化时：

```bash
docker compose build <受影响的服务>
docker compose down
docker compose up -d --no-build --force-recreate
```

Web 入口为 `http://192.168.100.10:8765/`，MoveIt/RViz 的 noVNC 入口为
`http://192.168.100.10:6080/`。

## 镜像

应用镜像固定继承已经验证的基础镜像；标签只在 [镜像清单](docs/DOCKER.md) 维护，避免多份清单失配。

每个服务在自己的 Dockerfile 中声明专属构建和运行依赖：相机拥有 librealsense，相机和场景各自拥有
OpenCV 5，运动拥有 ROS 2/MoveIt/RViz，感知计算拥有 Python、PyTorch、YOLOE、GraspGenX 和
模型。全局后端基础只保留三个以上服务共同使用的 Rust/Dora 工具链及最小运行库；Compose
不会再构建一个包含所有节点与硬件环境的总后端镜像。前端的构建与运行使用同一个经过验证的
Ubuntu 基础镜像。

只有对应依赖变化时才重新构建受影响的基础镜像并发布新标签。具体规则见
[Docker 与服务镜像](docs/DOCKER.md)。

## 相机与感知

深度相机默认未选择；启动时自动发现来源，网页可刷新列表。RealSense 和内置模拟相机都进入：

`驱动 crate/模拟适配器 → camera-node → CameraFrameBundle → scene-node → WorldScene`

`CameraFrameBundle` 原子携带已对齐到彩色平面的 RGB-D、共享平面内参、设备深度比例、时间信息
和逐帧外参快照。原始相机数据不经过 ROS；`camera-node` 负责采集、对齐和标定，`scene-node` 负责编排识别、分割、三维实例和
抓取候选。MoveIt 接收目标体积、放置位姿、候选及同帧完整点云，使用官方 Octomap 插件建立一次
抓放环境快照；MTC 管理目标附着和运输，不持续重发旧点云。相机 profile 与已确认标定由后端文件
保存，当前选择不持久化，因此重启仍回到未选择状态。配置以设备序列号等稳定身份关联，不使用
枚举索引或 USB 口；离线设备及暂时缺失的 profile 仍保留在配置中，并在网页置灰说明。网页可
选择驱动实际报告的分辨率、格式和采集 FPS，并独立设置不高于采集频率的上送 FPS；厂商专属
底层参数通过驱动命名空间扩展展示，不会形成第二条感知链路。采集和 SDK Align 在 Tokio 长期
任务中执行，高采集、低上送时只对待发布 RGB-D 帧做深度对齐；实时彩色视频仍按采集频率复制。

网页各步骤独立触发，不隐式串联：

1. **运行分割**：只输出类别、二维框、掩膜和叠加图，无需标定或机械臂连接。
2. **三维定位**：使用该次分割的同帧深度与外参，不重跑模型。
3. 选择目标后 **生成抓取候选**：只为选中的实例调用 GraspGenX。
4. 选择放置区域后 **执行抓放**：交给现有 MoveIt/MTC。新分割会清空旧三维结果和候选。

感知页顶部把抓放作为一个可折叠应用场景展示。自然语言入口由 Next.js 服务端使用 AI SDK 将
指令转换为开放词汇提示词，并从本次真实 `WorldScene` 中选择对象和放置区域，再调用同一套手动
分割、定位、候选与 MTC 抓放接口；AI 不生成机械臂坐标、姿态或轨迹。模型配置、手动执行与规划详情默认
收起，抓放场景之外的相机、标定和通用场景结果仍保持独立。

首次启用自然语言入口时复制 `.env.example` 为 `.env` 并填写密钥。Compose 只把这些变量注入
`web-perception`，密钥不会进入浏览器或 Git。

## 常用验收

```bash
pnpm --dir frontend format:check
pnpm --dir frontend lint
pnpm --dir frontend typecheck
pnpm --dir frontend test
pnpm --dir frontend build
docker compose config --quiet
```

完整软件抓放测试会改变连接、配置和姿态，不要在连接真机时随手运行；先阅读测试入口中的执行条件。

Rust 测试、Clippy 和模型测试必须在各自服务的构建/运行镜像执行，不能在 Host 用
`cargo test --workspace` 混测 ROS、SDL 和 OpenCV。分组命令见
[测试入口与环境](tests/README.md)。

测试脚本和诊断工具位于 `tests/`、`tools/` 与各节点自己的测试目录。项目中间资源只能放仓库
`temp/`；运行配置位于 `backend/config/runtime/`，网页不保存服务配置副本。
服务停止后 `temp/` 随时可以清空，下次启动/测试不依赖旧临时数据；正式输入和配置不放在其中。

## 文档入口

- [架构与调用链](docs/BACKEND.md)
- [Docker 与服务镜像](docs/DOCKER.md)
- [StarArm-102 型号适配](docs/STARARM-102.md)
- [资产基线与验收边界](docs/REVIEW.md)
- [测试入口与环境](tests/README.md)

真实抓放已由用户验收；海绵通过一批十次实际持物搬运，硬盒不稳定和漏检仍是已知边界，详见验收说明。
`docs/` 只维护当前实现、操作方法和必要限制；历史调查与逐轮记录不再保留在工作树，
需要追溯可从 Git 历史恢复；不可重拍的关键实物证据归档在 `tools/diagnostics/evidence/`。
标定与运行配置保留，不受文档清理影响。
