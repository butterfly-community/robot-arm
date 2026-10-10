# 回归测试

测试入口按影响范围选择，不默认运行所有硬件测试。网页功能必须通过真实页面完成选择、
保存、运行、等待与结果核对；单元测试、Mock 和接口检查仅作补充。
控制器终态、视觉确认和人工验收是不同证据，不能互相替代。

Host 只运行 Node/Git/Docker；ROS、OpenCV、SDL、RealSense 和模型测试在所属镜像中运行。
输出统一放根目录 `temp/`，测试自行创建目录，不依赖旧输出。独立 `tools/` 保存工具和可复用夹具，
生产构建不依赖它。服务地址默认 `http://192.168.100.10:8765`；重启始终整套关闭、整套启动。

## 源码与前端检查

在根目录执行：

```bash
pnpm --dir frontend format:check
pnpm --dir frontend lint
pnpm --dir frontend typecheck
pnpm --dir frontend test
node tools/check-doc-links.mjs
node tools/check-git-ignore.mjs
node tools/check-build-context.mjs
node --test tests/tools/*.test.mjs
```

链接检查覆盖主仓库与 tools 的本地路径；构建上下文检查需要 Docker，只用合成输入。
前端单测覆盖业务提交、请求终态、Responses 上下文、图片持久化与 RGB 像素一致性。
消息/场景/运动回归确认抓放使用绑定同次快照的单条请求，不靠独立点云订阅拼接。

## 浏览器回归

在 `frontend` 中执行 `pnpm exec playwright test 文件名.spec.ts`。不要在真机工作期间直接运行
整个 `test:e2e`：部分用例会修改正式配置、断开执行器、调用模型或执行运动。

| 用例 | 范围与副作用 |
| --- | --- |
| `ai-layout`、`layout-spacing` | 五页栏目归属、折叠、桌面/窄屏间距；拦截业务写入 |
| `form-sync`、`request-status`、`pick-place-progress` | 草稿/生效值、失败保留、同请求进度与刷新；拦截写入 |
| `segmentation-composition`、`segmentation-models`、`grasp-selection` | 三种分割组合、原图框选、来源与实例选择；拦截写入，不运行实体抓放 |
| `calibration-persistence` | 已保存外参、重新标定/取消、刷新回显；拦截标定请求，不证明真实标定精度 |
| `input-discovery` | 实际未发现提示与布局；注入故障用例只验证错误显示 |
| `input-gamepad` | 实体手柄发现、编辑/保存/刷新与采样；恢复原名称，不发运动 |
| `camera-bindings` | 网页绑定两个模拟来源、出图、拖动/收起、持久化与解除；不拦截，不发运动 |
| `execution-mode` | 实际切换模拟/真机模式、错误串口、软件关节运动，结束保留模拟器；另含拦截式故障回归 |

手柄测试用 `INPUT_GAMEPAD_NAME` 指定实际 SDL 名称。可选 `INPUT_GAMEPAD_USB_INTERFACE`
启用 USB 解绑/重绑测试（会短暂断开该手柄），脚本核对设备类型并恢复；未提供时明确跳过。
相机绑定测试加 `CAMERA_AI_TEST=1` 可通过已配置模型验证角色取图，须已授权发送图片；
`CAMERA_BINDINGS_KEEP=1` 保留测试绑定。模拟来源不证明实体 UVC 采集或真机抓放。

## 真实页面操作

以下从根目录运行 `node tests/integration/文件名.mjs 参数`。
写配置、标定和运动测试必须串行，不与人工操作争用设备；实体抓放前取下标定板。

| 入口与参数 | 用途与副作用 |
| --- | --- |
| `camera-bindings discover OUTPUT` | 页面刷新并列出设备/profile |
| `camera-bindings bind OUTPUT ROLE SOURCE PROFILE` | 选择、保存并核对新图；ROLE 为 external 或 wrist |
| `camera-bindings on\|off\|unbind OUTPUT ROLE` | 开关采集或解除绑定；关闭保留配置 |
| `camera-bindings cycle OUTPUT` | 已绑定两路轮流切换并核对新帧、刷新持久化；不改分辨率、不运动 |
| `connect-execution ENDPOINT OUTPUT` | 页面连接真实串口并核对反馈，不发运动 |
| `control-capabilities work OUTPUT` | 只点击工作位并等原请求终态，会运动 |
| `control-capabilities motion OUTPUT` | 工作位、TCP、无效参数与空载夹爪；会运动 |
| `control-capabilities history OUTPUT` | 查询先前请求并确认不重发，不运动 |
| `grasp-cancel observe OUTPUT` | 新帧、分割、定位，不抓放 |
| `grasp-cancel queued\|planning\|executing\|complete OUTPUT OBJECT_ID REGION_ID` | 实际抓放/取消并跟踪原请求终态，会运动 |
| `gripper-hold-ui OUTPUT OBJECT_ID REGION_ID` | 实际抓放并临时调整保持力度，结束恢复原配置 |
| `execution-hold OUTPUT` | 全部上力并读取信息，会操作真实电机，不写固件参数 |

`queued` 用例先准备当前候选，再临时降速排队测试，结束恢复；若任务已进入规划，不能计为排队取消通过。
力度测试须核对画面和实际反馈，非零负载不证明抓到物体；异常时取消自身活动任务，不主动松爪或回位。
相机 profile 来自实际枚举，不写死 video 编号；共享 USB 带宽不足时先关闭另一路再开启目标相机。

### 通用 AI

`node tests/integration/general-ai.mjs MODE OUTPUT [参数]` 操作正式通用 AI 页面，不拦截或注入结果。
使用当前网页保存的模型设置；任务和图片会发送给该模型服务。

| MODE | 参数 / 行为 |
| --- | --- |
| `send` | `"任务文本" [IMAGE_PATH]`，实际发送并等同一运行终态 |
| `watch` | `RUN_ID`，仅跟踪已有任务 |
| `inspect`、`reply-layout` | 只读查看状态 / 当前会话段落、间距与刷新 |
| `settings` | 下拉/手输模型与思考强度、保存/刷新，结束恢复原设置 |
| `model` | `MODEL`，从页面保存模型并验证持久化 |
| `config` | `"" [IMAGE_PATH] [EFFORT]`，保存配置并验证模型文本/图像/工具能力 |
| `new`、`history` | 新会话 / `SESSION_ID` 重开历史 |
| `stop` | 点击停止并等终态 |
| `camera` | 选择并启用可用 RealSense，会改变当前相机选择 |

`AI_RELOAD_AFTER_START=1` 实测发送后刷新仍跟踪原运行，不重发。
`AI_WAIT_TIMEOUT_MS` 只设置测试等待时间，不限制后端任务。
浏览器状态在 `temp/general-ai/browser-state.json`，丢失后通过网页历史会话恢复。
输出包含请求终态、相机画面、网页执行反馈及耗时；AI 自报成功不算实际抓放成功。
`timing.json` 的整轮减串行工具时间包含网络等待和本地编排，不是纯模型推理耗时。

图片历史验收：发送附图任务 → 新会话 → 重开原会话 → 不附图追问，核对原图内容及请求记录。
只读完整性检查在 perception 镜像中执行，验证历史图片哈希、解码、尺寸，不改历史或发任务：

```bash
docker compose exec -T -w /workspace/web/apps/perception web-perception \
  node --input-type=module - SESSION_ID < tests/integration/ai-history-audit.mjs
```

## 软件链路与原生模块

| 入口 | 范围与副作用 |
| --- | --- |
| `node tests/integration/repository-audit.mjs` | API/ROS 取消、请求隔离与恢复；要求 software 执行，不替代网页验收 |
| `node tests/integration/mouse-control.mjs` | 鼠标十二方向经绑定、空间、Servo、软件反馈；要求 software，恢复配置 |
| `node tests/integration/software-flow.mjs` | 模拟 RGB-D、1280/1920 标定、三维场景、MTC；会断开真机，成功时恢复配置 |
| `docker compose run --rm integration-test` | Compose 隔离集成测试入口 |

失败后先核对正式配置和设备状态，不能假定恢复步骤已执行。可用 `SERVICES_BASE_URL` 修改地址。
模拟输入经过正式链路，但不证明实物夹持或真机精度。

Rust 使用对应 Dockerfile 的 `build` 目标，设置 `TMPDIR=/workspace/temp`，先创建输出目录。
同时运行 `cargo fmt --all -- --check` 与相同 features 的 `cargo clippy … -- -D warnings`。

| 构建环境 | 测试包与依赖 |
| --- | --- |
| 公共后端 | messages、json-config-store、spatial-core、型号库、fashionstar-uart、nolo-cv1、网关、状态、空间、execution |
| controller-input | `controller-input-node`，实际 SDL/HID 依赖 |
| camera | `camera-node --features realsense-runtime,opencv-runtime`、camera-calibration、uvc-camera、realsense-camera 的 runtime |
| scene | scene-node、scene-core，实际 OpenCV |
| motion | source ROS、MoveIt、型号三个工作区；`stararm-102-motion-node --no-default-features --features ros-runtime` |

Redis 请求持久化测试显式设置 `REDIS_TEST_URL` 并使用 `--ignored`，只操作唯一 test 请求，不操作机器人。
不能用未启用 runtime features 的编译冒充驱动验收。

Python 测试使用计算镜像的 `/opt/compute-venv/bin/python -m pytest -p no:cacheprovider`，
挂载仓库，设置 `PYTHONPATH=/workspace/backend/services/perception-compute/src`、
`TMPDIR=/workspace/temp`、`PYTHONDONTWRITEBYTECODE=1`：

- 运行镜像：计算服务 `tests/` 和 `tools/graspgenx/tests/test_contact_geometry.py`。
- 构建镜像：`tools/graspgenx/tests/test_description.py`，依赖官方资产生成向导。

更多诊断入口见 [tools](../tools/README.md)，当前架构与参数见 [BACKEND](../docs/BACKEND.md)，
硬件精度边界见 [型号说明](../docs/STARARM-102.md)。
