# Robot Arm Services

本目录是前后端分离的 Dora 服务化实现：

- `backend/`：Rust/Dora 节点、FashionStar UART、ROS 2/MoveIt 型号节点和后端镜像。
- `frontend/`：五个 Next.js 应用、共享 UI/契约/客户端和单一前端镜像。
- 根目录：Compose、Dora dataflow、跨端验收、记录工具和当前设计文档。

## 启停

全部命令都在本目录执行：

```bash
docker compose up -d
docker compose down
```

重启就是依次执行上面的 `down` 和 `up -d`。项目不提供管理脚本，也不维护单服务重启状态。
Compose 停止 dataflow 时会向 Dora attach 会话发送 `SIGINT`，由 Dora 自己停止全部节点，避免下次启动重复部署。
需要重新构建时使用：

```bash
docker compose up -d --build
```

Compose 只维护两个工程基础镜像：`frontend-base` 与 `backend-base`。两者都基于 Ubuntu
26.04 LTS；后端基础镜像统一提供 ROS 2 Lyrical、MoveIt、RealSense SDK/ROS 驱动、
OpenCV 5 源码构建、Rust、Python、Dora、GraspGenX、模型权重和已应用补丁的 StarArm-102
ROS 软件包。ROS 图像桥从与 Lyrical 对齐的 `cv_bridge 4.1.0` 源码链接同一套 OpenCV 5，
不会再引入发行版 OpenCV 4。通用第三方仓库只在后端基础镜像构建中按固定提交处理；StarArm-102 的厂家源码、
补丁、ROS 包和夹爪资产由 `backend/devices/stararm-102/docker/install-base.sh` 作为一个连续
设备模块安装，主机不需要第三方源码目录。模型权重从
`backend/services/perception-compute/models/` 复制进基础镜像，避免每次构建重新下载。各工程
镜像只复制并构建本工程源码，不重复声明系统、厂商或模型依赖，也不拆分构建/运行层或逐个
搬运产物。

页面入口为 `http://192.168.100.10:8765/`，业务路径是 `/tracking/`、`/spatial/`、
`/perception/`、`/motion/` 和 `/arm-execution/`。只有入口服务暴露主机端口。

## 配置

可修改的服务配置保存在宿主 `backend/config/runtime/`，Compose 将这个目录挂载为容器内
`/config`。目录中的 JSON 被 Git 忽略，但 `.gitignore` 本身受版本控制，因此 `down`、重新构建和
`up -d` 都不会丢失配置。控制绑定、空间、感知、motion 和执行节点各自只读写自己的文件；文件不存在时使用
默认值，已有文件损坏时服务直接报告启动错误。后端统一通过 `json-config-store` 加载和保存，网页只提交
配置请求，不把服务参数保存在浏览器中。

`backend/config/service-status.json` 是受版本控制的静态依赖配置。实时姿态、控制会话、模拟状态、
关节反馈、串口连接状态和错误不会写入配置文件。

## 开发检查

```bash
cd backend
cargo fmt --all -- --check
cargo test --workspace --all-targets
cargo clippy --workspace --all-targets -- -D warnings

cd ../frontend
pnpm format:check
pnpm lint
pnpm typecheck
pnpm test
pnpm build
pnpm test:e2e

cd ..
docker compose config --quiet
docker compose build
docker compose --profile test run --rm integration-test
```

架构与行为见 [docs/SUMMARY.md](docs/SUMMARY.md)，后端逐服务、逐方法和依赖边界见
[docs/BACKEND.md](docs/BACKEND.md)，最终审查与验收见 [docs/REVIEW.md](docs/REVIEW.md)，
当前机械臂型号事实见 [docs/STARARM-102.md](docs/STARARM-102.md)，测试用记录/回放工具见
[tools/replay/README.md](tools/replay/README.md)。
