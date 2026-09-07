# Docker 与服务镜像

## 原则

每个服务在自己的目录维护 Dockerfile、专属构建依赖和专属运行依赖。只有三个以上后端服务共同
使用的 Rust/Dora 构建环境与最小运行库进入全局基础镜像。相机、OpenCV、RealSense、ROS、
MoveIt、RViz、Python、AI 模型和设备厂商包都不属于全局基础层。

构建阶段与运行阶段分开；运行镜像只复制成品和实际动态库。Compose 只聚合服务，不维护第二份
依赖安装逻辑，也不存在能够运行所有节点的总后端应用镜像。

## 固定基础标签

| 标签 | 唯一职责 |
| --- | --- |
| `robot-arm-services-backend-base:2026.09.05-r9` | Ubuntu 26.04、Rust 1.97、Dora 和多个 Rust 服务共同使用的原生构建工具 |
| `robot-arm-services-backend-runtime:2026.09.05-r3` | Ubuntu 26.04、Dora 可执行文件、入口脚本和最小 Rust 运行库 |
| `robot-arm-services-stararm-102-base:2026.09.05-r2` | 固定厂商提交、校验、StarArm-102 型号补丁与验证后的厂商资产 |
| `robot-arm-services-perception-compute-base:2026.09.07-r7` | Python 3.11、PyTorch、YOLOE、GraspGenX、仓库模型和构建期夹爪资产；由张开夹指几何测量采样深度 |
| `robot-arm-services-perception-compute-runtime:2026.09.07-r7` | 上述计算环境的运行文件，不含其构建工具 |
| `robot-arm-services-frontend-base:2026.09.03-r1` | Ubuntu 26.04、Node.js 24 与 pnpm 11 |

StarArm-102 基础不是全局后端基础。运动和执行镜像读取同一份已打补丁的型号资产，计算基础镜像
读取它来生成对应 GraspGenX 夹爪描述。该资产层不含 ROS；当前工程 ROS 源码由 motion 自己构建，
因此修改应用代码不需要重建厂商资产层。

## 服务所有权

| 服务 | Dockerfile | 服务独占的环境 |
| --- | --- | --- |
| controller-input | `backend/nodes/controller-input/Dockerfile` | SDL/HID 所需 USB 构建与运行库 |
| camera | `backend/nodes/camera/Dockerfile` | OpenCV 5、librealsense SDK/运行库和相机 USB 权限 |
| perception | `backend/nodes/scene/Dockerfile` | OpenCV 5；不含相机驱动、ROS 或 AI 权重 |
| perception-compute | `backend/services/perception-compute/Dockerfile` | Python 计算运行环境、YOLOE、GraspGenX、模型和夹爪资产 |
| stararm-102-motion | `backend/devices/stararm-102/nodes/motion/Dockerfile` | ROS 2 Lyrical、MoveIt/Servo/MTC、RViz/noVNC、设备 ROS 模型 |
| stararm-102-execution | `backend/devices/stararm-102/nodes/execution/Dockerfile` | StarArm 串口运行库、同源 URDF/网格；不含 ROS |
| spatial-transform | `backend/nodes/spatial-transform/Dockerfile` | 无额外系统环境 |
| service-status | `backend/nodes/service-status/Dockerfile` | 无额外系统环境 |
| web-gateway | `backend/nodes/web-gateway/Dockerfile` | 无额外系统环境 |
| 五个 Next.js 应用 | `frontend/web/Dockerfile` | 同一个前端镜像，由 `WEB_APP` 选择入口 |

相机与场景服务各自声明 OpenCV 5，因为它们是独立服务所有者；两份完全相同的源码构建步骤可由
Docker 内容缓存复用，不因此增加共享基础镜像或隐藏依赖。OpenCV 的
`OpenCV_DIR`、`LD_LIBRARY_PATH`、`PKG_CONFIG_PATH` 和
`OPENCV_DISABLE_PROBES=pkg_config` 都在 Cargo 构建前声明，确保 `opencv-rust` 选择
`/opt/opencv5` 而不是 ROS 的 OpenCV 4。RealSense 仅存在于 camera 镜像；ROS/MoveIt 仅存在于
motion 镜像；AI 环境与模型仅存在于 perception-compute 镜像。

## 构建顺序

只有基础依赖实际变化才构建对应基础 Dockerfile，并发布**新标签**，不覆盖上表已验证标签。
顺序为受影响的公共构建/运行基础 → 型号资产（若变更）→ 计算构建/运行基础（若变更）→ 应用。
未受影响的层直接复用；不能因为应用源码改动重跑全套基础构建。

前端基础镜像仅在 Node 或 pnpm 变化时重建。基础标签更新后，应一次性修改直接引用它的
Dockerfile；不得用 `latest` 隐式漂移。应用代码的日常构建仍只有：

```bash
docker compose build <受影响的服务>
docker compose up -d --no-build --force-recreate
```

系统统一启停，不维护单节点生命周期。需要保留容器定义并整体恢复时使用
`docker compose up -d --no-build --force-recreate`，且不指定单个服务；dataflow 不单独自动重启某个节点。
Rust 构建目标中的测试复用该服务环境，Python 运行测试用计算运行镜像，夹爪生成测试用含官方向导的计算构建镜像。
不要把“精简运行镜像没有构建向导”或“构建镜像未安装显示运行库”错误地补成第二套运行环境。
自然语言 API 的地址、模型和密钥只通过根目录
`.env` 注入 `web-perception`；它们不进入镜像层或浏览器 bundle。

## 设备与存储

服务停止后，根目录 `temp/` 可随时清空；下次构建/启动不依赖其中的历史文件。
Compose 只把 `temp/recordings` 作为录制输出，以及测试 profile 的结果目录；缺失时挂载会创建目录。
正式配置在 `backend/config/runtime`，相机资产在 `backend/nodes/camera/assets`，
模型在计算服务所属目录并打入镜像，均不依赖 `temp/`。不要求服务运行中清理的容错。

controller-input、camera 和 execution 只挂载各自需要的 `/dev`、udev/sysfs 和 cgroup 规则。
相机默认未选择，不会在容器启动时占用设备。RealSense 真机由 camera 服务直接打开，原始 RGB-D
不经过 ROS。

Docker 数据根目录属于主机配置，不由仓库脚本修改。清理历史镜像前先检查 Compose 引用和共享
层；已迁移的数据层不应再次复制回系统盘。
