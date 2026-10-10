# Docker 与服务镜像

本文说明如何启停服务、更新镜像，以及配置和外部设备由谁管理。
日常使用主要关注“使用入口”“数据保存位置”和“构建顺序”；其余章节供维护镜像与排查设备问题时查阅。

## 使用入口

首次配置自然语言能力时，将根目录 `.env.example` 复制为 `.env` 并填写模型服务设置与密钥。
Compose 只向所需服务注入配置，密钥不进入浏览器或 Git。

Web 入口为 `http://192.168.100.10:8765/`，MoveIt/RViz 的 noVNC 入口为
`http://192.168.100.10:6080/`。以下命令在仓库根目录执行，前提是服务镜像已完成构建。

启动整套服务：

```bash
docker compose up -d --no-build
```

关闭整套服务：

```bash
docker compose down
```

重启始终整套关闭、整套启动，不独立重启节点。源码变更的构建方式见下文“构建顺序”。
前端检查和原生服务测试见[测试入口](../tests/README.md)。

## 构建上下文

构建上下文是 Docker 在构建镜像时能够读取的文件范围。本项目只使用根目录、`frontend/` 和
`backend/services/perception-compute/` 作为上下文，各自的 `.dockerignore` 排除本地 `.env*`、缓存和运行配置。
计算基础镜像从根目录读取文件，但使用专属 `Dockerfile.base.dockerignore`；它取代根目录的忽略规则，
因此两处都必须排除本地配置、独立 `tools/` 仓库和嵌套 `.git`。
正式夹爪资产生成器位于 `backend/services/perception-compute/build-description.py`；设备自己的
`backend/devices/stararm-102/tools/` 仍属于生产构建输入，不能和根目录工具仓库一并排除。
正式标定配置只由 Compose 挂载，不打进镜像。
规则按 [Docker 构建上下文约定](https://docs.docker.com/build/concepts/context/#dockerignore-files) 生效，
不是 Git 忽略规则的继承。

## 数据保存位置

普通重启会保留 Docker 命名卷和正式配置目录。**日常重启不要使用 `docker compose down -v`**，
该选项会删除命名卷，包括请求记录和 AI 会话图片。

| 数据 | 保存位置 | 用途 |
| --- | --- | --- |
| 请求记录、AI 对话和非敏感模型设置 | Redis，命名卷 `request-history-data` | 页面刷新或服务重启后查询历史；AI 数据使用 `robot-arm:ai:` 键名前缀 |
| AI 对话图片 | 命名卷 `ai-conversation-data`，容器内 `/data/ai/images` | 对话中的图片引用指向这些文件，不把图片本体放入 Redis |
| 设备绑定、相机标定等正式配置 | `backend/config/runtime` | 由对应节点保存，Compose 挂载到服务，不打入镜像 |
| 模型服务密钥 | 根目录 `.env`，注入服务端环境 | 不进入浏览器、对话记录或 Git |
| 录制与测试过程输出 | 根目录 `temp/` | 服务停止后可清理，不作为下次启动的必需输入 |

本地 `redis` 服务固定使用 `redis:8.6.3`，仅在 Compose 内网提供请求记录存储，不发布主机端口。
`web-gateway` 通过 `REDIS_URL` 连接，启动依赖 Redis 健康检查。命名卷
`request-history-data` 保存 Redis 的追加写入日志（AOF），每秒同步到磁盘一次。
突然断电可能丢失最后约一秒写入；这些记录不能单独保证物理动作恰好执行一次。
`web-perception` 也连接同一个 Redis，保存 AI 会话、设置和工具调用对应的机器人请求编号。

`AI_API_KEY` 仅通过服务端环境注入，Responses 请求使用 Bearer 鉴权。
`AI_API_BASE_URL`、`AI_MODEL`、`AI_REASONING_EFFORT` 是首次使用时的默认值；
网页保存过设置后，以网页值为准。不要把密钥写入网页配置或对话。
数据写入磁盘的方式见 [Redis 官方说明](https://redis.io/docs/latest/operate/oss_and_stack/management/persistence/)。

## 镜像如何拆分

每个服务在自己的目录维护 Dockerfile、专属构建依赖和专属运行依赖。只有三个以上后端服务共同
使用的 Rust/Dora 构建环境与最小运行库进入全局基础镜像。相机、OpenCV、RealSense、ROS、
MoveIt、RViz、Python、AI 模型和设备厂商包都不属于全局基础层。

构建镜像包含编译器和开发依赖；运行镜像只复制编译结果及运行所需的动态库。
Compose 负责组合和启停服务，安装依赖的步骤只维护在各服务的 Dockerfile 中。

## 固定基础标签

| 标签 | 唯一职责 |
| --- | --- |
| `robot-arm-services-backend-base:2026.09.05-r9` | Ubuntu 26.04、Rust 1.97、Dora 和多个 Rust 服务共同使用的原生构建工具 |
| `robot-arm-services-backend-runtime:2026.09.05-r3` | Ubuntu 26.04、Dora 可执行文件、入口脚本和最小 Rust 运行库 |
| `robot-arm-services-stararm-102-base:2026.09.13-r1` | 固定厂商提交、校验、StarArm-102 型号补丁与验证后的厂商资产 |
| `robot-arm-services-perception-compute-base:2026.09.07-r7` | Python 3.11、PyTorch、YOLOE、GraspGenX、仓库模型和构建期夹爪资产；由张开夹指几何测量采样深度 |
| `robot-arm-services-perception-compute-runtime:2026.09.07-r7` | 上述计算环境的运行文件，不含其构建工具 |
| `robot-arm-services-frontend-base:2026.09.03-r1` | Ubuntu 26.04、Node.js 24 与 pnpm 11 |

StarArm-102 基础镜像只提供本型号资产，不是所有后端服务的共同依赖。
运动和执行服务读取同一份已打补丁的模型，计算服务用它生成 GraspGenX 夹爪描述。
该资产镜像不含 ROS；项目的 ROS 代码由 motion 服务自行构建，普通应用改动不需要重建型号资产。

固定提交和补丁说明见[模型来源](STARARM-102.md#模型来源与补丁)。

## 各服务的依赖

| 服务 | Dockerfile | 该服务维护的依赖 |
| --- | --- | --- |
| controller-input | `backend/nodes/controller-input/Dockerfile` | SDL/HID 所需 USB 构建与运行库 |
| camera | `backend/nodes/camera/Dockerfile` | OpenCV 5、librealsense、V4L2 普通摄像头与相机设备权限 |
| perception | `backend/nodes/scene/Dockerfile` | OpenCV 5；不含相机驱动、ROS 或 AI 权重 |
| perception-compute | `backend/services/perception-compute/Dockerfile` | Python 计算运行环境、YOLOE、GraspGenX、模型和夹爪资产 |
| stararm-102-motion | `backend/devices/stararm-102/nodes/motion/Dockerfile` | ROS 2 Lyrical、MoveIt/Servo/MTC、RViz/noVNC、设备 ROS 模型 |
| stararm-102-execution | `backend/devices/stararm-102/nodes/execution/Dockerfile` | StarArm 串口运行库、同源 URDF/网格；不含 ROS |
| spatial-transform | `backend/nodes/spatial-transform/Dockerfile` | 无额外系统环境 |
| service-status | `backend/nodes/service-status/Dockerfile` | 无额外系统环境 |
| web-gateway | `backend/nodes/web-gateway/Dockerfile` | 无额外系统环境 |
| 五个 Next.js 应用 | `frontend/web/Dockerfile` | 同一个前端镜像，由 `WEB_APP` 选择入口 |

相机与场景服务各自在 Dockerfile 中声明 OpenCV 5；相同构建步骤由 Docker 缓存复用。
OpenCV 的 `OpenCV_DIR`、`LD_LIBRARY_PATH`、`PKG_CONFIG_PATH` 和
`OPENCV_DISABLE_PROBES=pkg_config` 都在 Cargo 构建前声明，确保 `opencv-rust` 选择
`/opt/opencv5` 而不是 ROS 的 OpenCV 4。RealSense 仅存在于 camera 镜像；ROS/MoveIt 仅存在于
motion 镜像；AI 环境与模型仅存在于 perception-compute 镜像。

motion 安装官方 `moveit-ros-perception`，把抓放请求中的点云用于碰撞检测，不安装 ROS 相机驱动。
它基于同版本 MoveIt 2.15.0 构建带关节边界停止补丁的 `moveit_servo`，启动时加载该补丁构建产物。
轨迹时间参数化（TOTG）、MoveIt 核心库和关节轨迹控制器（JTC）仍使用官方二进制，
没有自定义的控制器完成判定。只有修改这些依赖补丁时才重编对应依赖层；
验证补丁时需核对实际加载的库并运行回归测试，不能只确认补丁文件存在。

### 点云传输所需的共享内存

共享内存容量和允许锁定的内存额度是两个不同配置，不能互相替代：

- Dora 的 Zenoh 1.9.0 会锁定默认 16 MiB 的共享内存传输池。Compose 给 Dora daemon 和 motion 设置
  `ulimits.memlock: -1`；若仍使用 8 MiB 锁页额度，即使主机内存充足也可能报 `OS error 12`。
- ROS 的 Fast DDS 在 motion 服务内部传递大点云，使用下面的共享内存配置。

Zenoh 的只读检查程序见 [zenoh-shm.rs](../tools/diagnostics/zenoh-shm.rs)。
完整 1920×1080 XYZ 消息约 24.9 MB：该服务的 Fast DDS 配置为每个 participant 分配 128 MiB SHM
segment（每个通信参与者的共享内存区），Compose `/dev/shm` 总容量为 2 GiB，
供 MoveGroup、MTC、Rust 桥和 RViz 共用。UDP 仍用于常规 ROS 发现和外部诊断；这些配置只作用于 motion。

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
Rust 测试在所属服务的构建环境中运行；Python 运行测试使用计算运行镜像；
夹爪资产生成测试使用包含官方生成工具的计算构建镜像。不要为主机测试另装 ROS、OpenCV 等依赖。

本地分割、候选生成、MoveIt 规划和执行不依赖外部大模型服务；自然语言 AI 助手需要连接已配置的模型服务。
已有镜像和正式配置准备好后，离线验收用 `--no-build --pull never` 整套冷启动，
清空可选的外部 AI 服务环境变量，并阻断服务的外网访问；方法及恢复命令见
[离线冷启动验收](../tools/diagnostics/offline/README.md)。首次构建下载依赖与离线运行是不同条件。

## 临时文件与模型文件

服务停止后，根目录 `temp/` 可随时清空；下次构建/启动不依赖其中的历史文件。
Compose 只把 `temp/recordings` 作为录制输出，以及测试 profile 的结果目录；缺失时挂载会创建目录。
正式配置在 `backend/config/runtime`，相机资产在 `backend/nodes/camera/assets`，
模型在计算服务所属目录并打入镜像，均不依赖 `temp/`。不要求服务运行中清理的容错。
自动分割权重 `yoloe-26x-seg-pf.pt` 由计算服务应用 Dockerfile 从 Ultralytics 官方资产下载，
固定 SHA256 并打入该服务镜像；新增它不要求重建全局基础镜像或其他服务的环境。

## 外部设备访问

controller-input、camera 和 stararm-102-execution 统一引用 Compose 的
`hardware-device-rules`，使用 `c *:* rw` 放行字符设备读写，不维护设备号白名单。
覆盖手柄 `event*`/`js*`/`hidraw*`、USB、相机 `video*` 和串口 `ttyUSB*`/`ttyACM*`；
三个服务均保留整个 `/dev` 挂载与只读 udev/sysfs，支持设备枚举和热插拔。
例如 `hidraw` 主设备号由主机动态分配，不能固定为某台机器的编号。
这项授权也覆盖这些容器可见的其他字符设备，但不放行块设备、不启用 `privileged`；
文件权限与主机其他访问控制仍然有效。不直接访问外设的服务不获得这项授权。
依据：[Linux hidraw](https://www.kernel.org/doc/html/latest/hid/hidraw.html)、
[Compose 设备规则](https://docs.docker.com/reference/compose-file/services/#device_cgroup_rules)。
只修改这项 Compose 权限不需要重编镜像；更新配置后整套 `docker compose down`，再执行
`docker compose up -d --no-build`，不要只运行 `restart`，因为它不会更新容器设备规则。
设备文件能被 `ls` 看见不代表可以打开；仍需在网页验证手柄输入、相机画面或串口反馈。

### 手柄热插拔

controller-input 设置 `SDL_JOYSTICK_DISABLE_UDEV=1`、`SDL_HIDAPI_UDEV=0`，让 SDL 自身通过
inotify/轮询发现 evdev 与 HIDAPI 设备。Docker 网络命名空间不能接收主机的 udev 热插拔通知，
仅挂载 `/run/udev` 不会转发通知；默认 udev 发现可能表现为启动前插入可见、启动后插入不可见。
保留 udev 数据挂载用于设备属性读取，不禁用 HIDAPI、传感器或震动，也不增加自定义设备扫描路径。
依据：[SDL Linux 手柄发现实现](https://github.com/libsdl-org/SDL/blob/release-3.4.14/src/joystick/linux/SDL_sysjoystick.c)、
[SDL HIDAPI 发现实现](https://github.com/libsdl-org/SDL/blob/release-3.4.14/src/hidapi/SDL_hidapi.c)。

### 相机绑定与 USB 带宽

深度相机默认未选择；外部/腕部摄像头按保存的采集开关恢复，未绑定或已关闭则不占用设备。
三路预览共用 camera 服务的视频端口，配置保存在正式 runtime 目录，不依赖 temp。
外部/腕部绑定提供独立采集开关；关闭等待驱动释放设备，保留绑定与分辨率，重启保持关闭状态。
共享 USB 带宽不足时可先关闭一路再开启另一路轮流取图，AI 使用相同相机请求，不另建采集节点。
RealSense 真机由 camera 服务直接打开，原始 RGB-D 不经过 ROS。

Docker 自身的数据目录属于主机配置，不由仓库脚本修改。清理旧镜像前需检查 Compose 是否仍在引用，
以及镜像层是否仍被其他服务使用。
