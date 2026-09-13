# 回归测试

生产逻辑在各服务/crate 中，纯函数回归随模块维护；本目录只放跨服务测试和输入夹具。
临时输出统一根目录 `temp/`。软件反馈测试不证明真机夹持、力度或实际定位精度。
服务停止后 `temp/` 可以随时清空；测试自行创建结果目录，正式夹具不在其中。

文档整理后在根目录运行 `node tools/check-doc-links.mjs` 检查本地内联链接目标；
此检查不访问服务、不发控制请求，也不验证外部网页和标题锚点。
检查器本身的回归运行 `node --test tests/tools/check-doc-links.test.mjs`；
Docker 忽略规则运行 `node tools/check-build-context.mjs`，只使用合成输入，不构建基础镜像。

`node --test tests/tools/pick-place-routing.test.mjs` 确认抓放只经场景节点绑定快照后到运动节点，
不恢复独立点云订阅与请求之间的时序竞争。消息 crate 测试覆盖绑定消息的二进制点云往返、
场景后续变化不修改已绑定快照及缺失/损坏点云；运动测试使用相同绑定消息进入正常任务转换。

## 前端

`segmentation-composition.spec.ts` 在 1440/390 像素宽度验证手动框选、中文命名、反向拖动的
1920 像素坐标换算、保存失败保留草稿、三种来源单独/混用、同模型替换、独立清除、来源型号显示、
刷新回显与删除。三个折叠状态独立保存；其他模型运行不丢失手动草稿，AI 默认模型不控制分割区可用性。
结果使用原始彩色图和框线，不染色或高亮；测试实际点击三种来源的图上标题、详情置信度、Escape 关闭、删除后关闭旧详情，
并核对手动编辑图不显示模型标注。覆盖桌面及窄屏，框坐标仍对应原图而非 CSS 缩放尺寸。
保存回显改变 JSON 字段顺序/浮点尾数时，成功确认仍释放手动草稿并启用三维定位；失败则保留草稿。
置灰原因在按钮旁明确显示，不再只有无说明的禁用状态。
三种分割都在顶部 AI 栏目内；“抓放场景”单独折叠，有两个选择框、启动和同请求实时进度。
`pick-place-progress.spec.ts` 覆盖候选等待、模式切换、规划、执行、刷新后继续显示和失败，
并核对桌面/窄屏无横向溢出。它拦截写请求，只证明状态展示，不作为实际抓放成绩。
HTTP 202 与首条运动反馈之间仍计时并禁用重复启动；仅匹配请求的终态结束等待，不使用旧任务状态。
`ai-layout.spec.ts` 核对相机/标定第一行、AI 双栏第二行，深度/内外参/结果折叠项的归属，
以及桌面和窄屏的宽度、间距、展开和刷新保持。旧的独立结果卡片不能重新出现。
`segmentation-models.spec.ts` 从按钮验证启动顺序及新场景序号，所有运动写入均拦截；
`start-pick-place.test.ts` 补充已有候选复用、候选失败、模式失败及失效选择的测试。
所有写请求和相机视频均拦截，不运行真实模型或机械臂；与真实网页链路验收分开报告。
运行：`pnpm --dir frontend exec playwright test tests/browser/segmentation-composition.spec.ts`。
Rust `scene-node` 的 `stage_tests` 补充同帧 RGB、掩膜像素、1280/1920 尺寸、重复标注编号、
过期序号和三种来源经同一深度重建的回归，在 scene 构建镜像运行 `cargo test -p scene-node`。

`grasp-selection.spec.ts` 核对任务已接收的实例选择随快照更新，但不覆盖用户随后编辑的放置选择。
所有 POST 被拦截，不发实际抓放；可单独运行 `pnpm --dir frontend exec playwright test tests/browser/grasp-selection.spec.ts`。

`node --test tests/tools/check-ui-css.test.mjs` 检查共享 CSS 不引用未定义的旧变量；
属于源码检查，不依赖服务。实际控件间距另由下面的浏览器回归验证。

`layout-spacing.spec.ts` 从全部五个真实网页展开/收起栏目，在桌面、平板和窄屏测量实际边框。
拦截业务写请求，不改变硬件或配置；覆盖“错误行底线紧贴深度折叠框”和关节/夹爪框等回归。
此检查包含正常信息行到独立框的距离，不只检查按钮和页面溢出。共享块分组间距为 20 px、
控制框组为 12 px，连续表格行不额外插入分组空白。

`form-sync.spec.ts` 覆盖网页编辑值与已生效配置的区分、保存失败不继续运行、保存后的实时回显、
未保存编辑保留（包括控制绑定），以及切换相机 profile 后低频上送保持、首次深度预览更新。
它拦截所有业务写请求，不触发真机动作。
布局审查从实际网页开始，使用 [网页审查脚本](../tools/diagnostics/audit-web-layout.mjs)；
状态注入只用于故障和交互回归，不能代替真实页面连接、预览与跨窗口同步检查。

`calibration-persistence.spec.ts` 在真实页面拦截标定 POST，覆盖历史数值、刷新保留、已标定按钮锁定、
重新标定/取消保留旧结果及确认后替换；不会控制机械臂。部署后另从未连接状态打开网页，检查来源
自动可选、历史标定可见，再连接实际相机并刷新验证。不要把这项界面回归当作重新完成真机标定。

在 `frontend` 执行 `pnpm format:check && pnpm lint && pnpm typecheck && pnpm test`。
`lint` 覆盖 `web/apps` 和 `web/packages`，不能只检查有独立脚本的应用而遗漏共享库。
服务启动后执行 `pnpm test:e2e`，使用正式页面、模型与软件反馈；部分测试会断开执行器、
运行模型、切换相机和改变模拟姿态，不能和人工操作并行。测试中的拦截只用于浏览器故障回归，
不会进入生产链路。截图在 `temp/playwright/`。

只验证分割模型配置切换可执行
`pnpm --dir frontend exec playwright test tests/browser/segmentation-models.spec.ts`。
该测试截获全部 POST，验证自动/提示词两个入口始终可用、AI 默认模型不隐藏它们、视觉示例保存
不自动运行模型，以及启动按新候选响应序号提交抓放；不操作真机。

## 正式软件调用链

在根目录执行，默认访问 `http://192.168.100.10:8765`：

| 命令 | 验证范围 |
| --- | --- |
| `node tests/integration/repository-audit.mjs` | 非法请求、真实 ROS cancel、请求 ID 隔离、取消后恢复、夹爪模式拒绝、抓放接收确认 |
| `node tests/integration/mouse-control.mjs` | 十二方向经过绑定/空间/Servo/软件反馈，停止和配置恢复 |
| `node tests/integration/software-flow.mjs` | 相机/视频/提示词、两分辨率九姿态自动标定、三维场景、MTC 软件抓放、配置与控制 |

前两项要求执行器未选端点且为 software；第三项会显式断开执行器。
按顺序单独运行，不和真机或其他测试争用状态。前两项 finally 恢复姿态/模式；全链路脚本有
显式配置恢复步骤，若断言提前失败须核对最后状态再继续，不能假定它已复原。
`SERVICES_BASE_URL` 可更改访问地址。

Compose 的 `integration-test` profile 挂载测试、分步请求辅助模块、全部相机资产和 `temp/`：

```bash
docker compose run --rm integration-test
```

## Rust 原生测试

Host 不安装 ROS/OpenCV/SDL/librealsense。以对应服务 Dockerfile 的 `build` 目标生成临时测试镜像，
复用已验证基础标签和 Docker 缓存；不重新发布基础镜像。例：

```bash
docker build -f backend/nodes/camera/Dockerfile --target build -t robot-arm-camera-audit-build .
docker run --rm -v "$PWD":/workspace -w /workspace/backend \
  -e TMPDIR=/workspace/temp -e CARGO_TARGET_DIR=/src/target robot-arm-camera-audit-build \
  bash -c 'mkdir -p /workspace/temp && cargo test --release --locked -p camera-node --features realsense-runtime,opencv-runtime'
```

| 所属环境 | 测试包/准备 |
| --- | --- |
| 公共后端构建镜像 | messages、json-config-store、spatial-core、型号库、fashionstar-uart、nolo-cv1、网关、状态、空间、execution |
| controller-input 的 build 目标 | `controller-input-node`（SDL/HID） |
| camera 的 build 目标 | `camera-node` 的两 runtime features、`camera-calibration`、`realsense-camera --features runtime` |
| scene 的 build 目标 | `scene-node`、`scene-core`（OpenCV） |
| motion 的 build 目标 | source `/opt/ros/lyrical/setup.bash`、`/opt/moveit_ws/install/setup.bash`、`/opt/devices/stararm-102/ros_ws/install/setup.bash`；`stararm-102-motion-node --no-default-features --features ros-runtime` |

同一环境、同一 features 下运行 `cargo clippy … -- -D warnings` 和 `cargo fmt --all -- --check`。
不能用不启用 runtime features 的编译冒充原生驱动验收。没有安装 `cargo-machete` 时不声称其通过。

## Python 模型与资产

挂载仓库到 `/workspace`，设置 `PYTHONPATH=/workspace/backend/services/perception-compute/src`、
`TMPDIR=/workspace/temp`、`PYTHONDONTWRITEBYTECODE=1`，使用镜像内 `/opt/compute-venv/bin/python -m pytest -p no:cacheprovider`：
运行前 `mkdir -p /workspace/temp`，不要依赖上一次测试已创建目录。

- 计算**运行镜像**：`/workspace/backend/services/perception-compute/tests`、`/workspace/tools/graspgenx/tests/test_contact_geometry.py`。
- 计算**构建镜像**：`/workspace/tools/graspgenx/tests/test_description.py`，需要官方生成向导与补丁厂商 URDF。

运行镜像故意不含生成向导，构建镜像不是 GUI 运行环境；不要为混跑测试向两边补入不属于它们的依赖。
当前验收状态与测试范围见 [验收说明](../docs/REVIEW.md)。复测结果写 `temp/`，不依赖历史运行记录。
