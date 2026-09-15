# Docker 与服务镜像

构建上下文仅使用根目录、`frontend/` 和 `backend/services/perception-compute/`；
各自的 `.dockerignore` 排除本地 `.env*`、缓存和运行配置。计算基础镜像使用根上下文及
`Dockerfile.base.dockerignore`，该文件会覆盖根规则，必须同步排除这些本地资源。
根上下文与计算基础上下文同时排除独立 `tools/` 及嵌套 `.git`。
正式夹爪资产生成器位于 `backend/services/perception-compute/build-description.py`；设备自己的
`backend/devices/stararm-102/tools/` 仍属于生产构建输入，不能和根目录工具仓库一并排除。
不维护未被构建入口使用的 `backend/.dockerignore`。正式标定配置只由 Compose 挂载，不打进镜像。
规则按 [Docker 构建上下文约定](https://docs.docker.com/build/concepts/context/#dockerignore-files) 生效，
不是 Git 忽略规则的继承。

## 原则

本地 `redis` 服务固定使用 `redis:8.6.3`，仅在 Compose 内网提供请求记录存储，不发布主机端口。
`web-gateway` 通过 `REDIS_URL` 连接，启动依赖 Redis 健康检查。命名卷
`request-history-data` 保存 AOF，`appendfsync everysec`；普通整套重启保留记录，
突然断电可能丢失最后约一秒写入，不能当作恰好一次物理执行保证。
不要使用 `docker compose down -v` 做日常重启。该卷不在 `temp/`，也不代替相机标定/设备配置。
web-perception 同样连接本地 Redis，使用 robot-arm:ai: 独立键空间保存会话、非敏感设置和运行关联。
AI 原图单独保存于 ai-conversation-data 卷的 /data/ai/images；Redis 仅存引用，不存图片/点云。
AI_API_KEY 仅通过服务端环境注入；Responses 使用 Bearer 鉴权。AI_API_BASE_URL、AI_MODEL、
AI_REASONING_EFFORT 是未在网页保存过设置时的默认值。不要把密钥写入网页配置或对话。
持久化语义见 [Redis 官方说明](https://redis.io/docs/latest/operate/oss_and_stack/management/persistence/)。

每个服务在自己的目录维护 Dockerfile、专属构建依赖和专属运行依赖。只有三个以上后端服务共同
使用的 Rust/Dora 构建环境与最小运行库进入全局基础镜像。相机、OpenCV、RealSense、ROS、
MoveIt、RViz、Python、AI 模型和设备厂商包都不属于全局基础层。

构建阶段与运行阶段分开；运行镜像只复制成品和实际动态库。Compose 只聚合服务，不维护第二份
依赖安装逻辑，也不存在能够运行所有节点的总后端应用镜像。

## 固定基础标签

Dora daemon 的公共 Compose 配置声明 `ulimits.memlock: -1`。Zenoh 1.9.0 使用 `mlock`
锁定 POSIX SHM；默认 16 MiB 传输池超过原容器 8 MiB 锁页额度，会报 `OS error 12`，
即使 `/dev/shm` 和主机 RAM 空余很多。只调整容器锁页额度，不关闭共享内存、不增加传输分支；
motion 使用相同锁页配置。只读探针见 [zenoh-shm.rs](../tools/diagnostics/zenoh-shm.rs)。

| 标签 | 唯一职责 |
| --- | --- |
| `robot-arm-services-backend-base:2026.09.05-r9` | Ubuntu 26.04、Rust 1.97、Dora 和多个 Rust 服务共同使用的原生构建工具 |
| `robot-arm-services-backend-runtime:2026.09.05-r3` | Ubuntu 26.04、Dora 可执行文件、入口脚本和最小 Rust 运行库 |
| `robot-arm-services-stararm-102-base:2026.09.13-r1` | 固定厂商提交、校验、StarArm-102 型号补丁与验证后的厂商资产 |
| `robot-arm-services-perception-compute-base:2026.09.07-r7` | Python 3.11、PyTorch、YOLOE、GraspGenX、仓库模型和构建期夹爪资产；由张开夹指几何测量采样深度 |
| `robot-arm-services-perception-compute-runtime:2026.09.07-r7` | 上述计算环境的运行文件，不含其构建工具 |
| `robot-arm-services-frontend-base:2026.09.03-r1` | Ubuntu 26.04、Node.js 24 与 pnpm 11 |

StarArm-102 基础不是全局后端基础。运动和执行镜像读取同一份已打补丁的型号资产，计算基础镜像
读取它来生成对应 GraspGenX 夹爪描述。该资产层不含 ROS；当前工程 ROS 源码由 motion 自己构建，
因此修改应用代码不需要重建厂商资产层。

固定提交和补丁说明见[模型来源](STARARM-102.md#模型来源与补丁)。

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

motion 还安装官方 `moveit-ros-perception`，处理显式抓放请求里的点云快照，不安装 ROS 相机驱动。
该服务仅对同版本 MoveIt 2.15.0 构建已有 `moveit_servo` 边界停止 overlay；
TOTG/moveit_core 与 JTC 均使用官方二进制，不构建自定义 JTC 完成判定。
运行镜像复制 Servo 成品并在入口加载 overlay，
不拷贝研究源码树、不改厂商资产标签。修改这些依赖补丁才重编对应运动依赖层；
核对时必须检查实际加载的共享库及原生回归，不能只看补丁文件存在。
完整 1920×1080 XYZ 消息约 24.9 MB：该服务的 Fast DDS 配置为每个 participant 分配 128 MiB SHM
segment，Compose `/dev/shm` 为 2 GiB，供 MoveGroup、MTC、Rust 桥和 RViz 共用；UDP 仍用于
常规 ROS 发现和外部诊断。配置归 motion，不改全局 DDS 或其他服务。

## 构建顺序

只有基础依赖实际变化才构建对应基础 Dockerfile，并发布**新标签**，不覆盖上表已验证标签。
顺序为受影响的公共构建/运行基础 → 型号资产（若变更）→ 计算构建/运行基础（若变更）→ 应用。
未受影响的层直接复用；不能因为应用源码改动重跑全套基础构建。

前端基础镜像仅在 Node 或 pnpm 变化时重建。基础标签更新后，应一次性修改直接引用它的
Dockerfile；不得用 `latest` 隐式漂移。应用代码的日常构建仍只有：

```bash
docker compose build <受影响的后端服务>
docker compose down
docker compose up -d --no-build --force-recreate
```

五个网页共用一个镜像，Compose 仅在 `web-tracking` 声明构建入口。
任何前端应用或共享包变化，都运行 `docker compose build web-tracking`，再整套重启；
`docker compose build web-perception` 等没有 build 声明的服务不会更新网页镜像。

系统统一启停，不维护单节点生命周期。重启时先整套 `down`，再整体 `up`，不指定单个服务；
dataflow 不单独自动重启某个节点。只读审查或不改变行为的源码整理不要求打断当前真机运行；
需要部署应用变化时再使用上述完整流程。
Rust 构建目标中的测试复用该服务环境，Python 运行测试用计算运行镜像，夹爪生成测试用含官方向导的计算构建镜像。
不要把“精简运行镜像没有构建向导”或“构建镜像未安装显示运行库”错误地补成第二套运行环境。
自然语言 API 的地址、模型和密钥只通过根目录
`.env` 注入 `web-perception`；它们不进入镜像层或浏览器 bundle。

本地感知、候选生成、MoveIt 规划和执行不依赖该外部自然语言入口。
已有镜像和正式配置准备好后，离线验收用 `--no-build --pull never` 整套冷启动，
清空可选 AI 环境并隔离服务出站；方法及恢复命令见
[离线冷启动验收](../tools/diagnostics/offline/README.md)。首次构建下载依赖与离线运行是不同条件。

## 设备与存储

服务停止后，根目录 `temp/` 可随时清空；下次构建/启动不依赖其中的历史文件。
Compose 只把 `temp/recordings` 作为录制输出，以及测试 profile 的结果目录；缺失时挂载会创建目录。
正式配置在 `backend/config/runtime`，相机资产在 `backend/nodes/camera/assets`，
模型在计算服务所属目录并打入镜像，均不依赖 `temp/`。不要求服务运行中清理的容错。
自动分割权重 `yoloe-26x-seg-pf.pt` 由计算服务应用 Dockerfile 从 Ultralytics 官方资产下载，
固定 SHA256 并打入该服务镜像；新增它不要求重建全局基础镜像或其他服务的环境。

controller-input、camera 和 execution 只挂载各自需要的 `/dev`、udev/sysfs 和 cgroup 规则。
相机默认未选择，不会在容器启动时占用设备。RealSense 真机由 camera 服务直接打开，原始 RGB-D
不经过 ROS。

Docker 数据根目录属于主机配置，不由仓库脚本修改。清理历史镜像前先检查 Compose 引用和共享
层；已迁移的数据层不应再次复制回系统盘。
