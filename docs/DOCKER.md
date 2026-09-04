# Docker 与服务镜像

## 两个基础镜像

项目只维护两个 Ubuntu 26.04 基础镜像：

- `robot-arm-services-backend-base:2026.09.04-r1`：原生构建依赖、ROS 2 Lyrical、MoveIt/MTC、
  noVNC/TigerVNC、librealsense、OpenCV 5、Python/uv、PyTorch、YOLOE、GraspGenX、Rust、Dora，
  以及打过项目补丁的 StarArm-102 厂家包和构建期生成的夹爪资产。
- `robot-arm-services-frontend-base:2026.09.03-r1`：Node.js 与 pnpm。

后端基础镜像不安装 MoveIt 元包，也不安装 ROS RealSense 驱动或 `cv_bridge`；只保留当前实际
使用的 OMPL、Servo、MTC、RViz 和运动执行组件。`move_group` 自身仍依赖 occupancy-map-monitor
库，但项目不配置 3D sensor updater，也不向 ROS 发布相机原始数据。相机采集直接使用
librealsense。OpenCV 5 在 ROS 前安装到 `/opt/opencv5`；`opencv-rust` 禁用 `pkg_config` 探测并
通过 `OpenCV_DIR` 选择这份 CMake package，避免误用 ROS 的 OpenCV。Python 模型环境使用
`opencv-python` 5。模型权重从仓库
`backend/services/perception-compute/models` 复制进镜像，避免每次联网下载；GraspGenX 源码按
固定提交在镜像内取得，不污染主机。

## 应用镜像

| 镜像 | 内容 |
| --- | --- |
| `robot-arm-services-backend` | 一次复制并编译完整 Rust workspace；运行所有 Dora 后端节点 |
| `robot-arm-services-perception-compute` | 在基础镜像已有虚拟环境中安装本工程计算服务 |
| `robot-arm-services-frontend` | 一次复制、安装并构建五个 Next.js 应用 |
| `nginx:stable-alpine` | 唯一 Web 入口 |

应用 Dockerfile 不安装系统环境、不下载厂家包、不生成夹爪资产，也不拆构建/运行阶段或逐个搬运
二进制。不同 Dora 服务共享同一个后端应用镜像，不按服务复制 Dockerfile。

自然语言抓放的 OpenAI-compatible 地址、模型和密钥属于 `web-perception` 运行配置。Compose 从
仓库根目录 `.env` 读取并注入容器，镜像构建不读取密钥；`.env.example` 只保留无密钥模板。

## 日常构建与启停

应用代码变化只执行：

```bash
docker compose build
docker compose down
docker compose up -d
```

只改挂载配置时可以省略 build。系统统一启停，不维护单服务生命周期；Dora daemon 统一启用
Compose `init`，保证停止信号转发和子进程回收。

## 基础镜像升级

只有下列内容变化才允许全量构建基础镜像：

- `backend/docker/base/Dockerfile` 或 `frontend/docker/base/Dockerfile`；
- 系统、语言工具链、ROS/MoveIt、OpenCV、相机 SDK、AI 运行环境或权重；
- 厂家源码版本、项目补丁、设备 ROS 包或构建期夹爪资产。

后端基础镜像按注释分组处理：通用工具、原生/USB、OpenCV、ROS、MoveIt、VNC、RealSense、
Python/模型、Rust/Dora、StarArm-102。厂家包只在最后一个连续设备模块中 clone、校验、patch、
编译和生成资产一次。OpenCV 的选择环境变量位于 Rust 工具链和所有 Cargo 构建之前。

升级使用新的不可复用标签：

```bash
docker build --progress=plain -f backend/docker/base/Dockerfile \
  -t robot-arm-services-backend-base:<新标签> .
```

验证完成后再一次性修改应用 Dockerfile 的 `FROM`。普通 Compose 构建不引用基础 Dockerfile，
因此上层服务变化不会重复构建基础环境。旧标签用于复现；应用不依赖 `latest`。

## 设备权限与存储

input、camera 和 execution 容器只挂载各自需要的 `/dev`、udev/sysfs 和 cgroup 规则。
RealSense 真机由 `perception` 容器内的 capture 节点直接打开；默认未选择，不会在容器启动时
扫描或占用设备。

Docker 数据根目录属于主机配置，不由本仓库脚本修改。迁移后应使用 `docker info` 核对
`Docker Root Dir`，清理历史镜像前先用只读命令确认引用关系，避免把镜像清理混入应用部署。
