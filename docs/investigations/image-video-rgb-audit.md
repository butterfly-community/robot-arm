# 图片与视频 RGB/BGR 链路排查

日期：2026-09-07。排查源码基线：`e9913f5`。排查阶段仅运行现有回归和只读采样，没有修改生产代码、
切换相机、运行模型任务、发出运动指令或重启服务。随后按用户要求修复视频关闭握手，见文末；
修复验收使用模拟相机，未发出机械臂运动指令，未构建基础镜像。

## 结论与约束边界

**已检查的业务链路没有发现 RGB/BGR 颠倒、重复换色或格式声明与像素不符。**
当前模拟源的原始 RGB、运行 WebSocket、网页 PNG 和实际浏览器 Canvas 逐像素一致。
但不能因此宣布所有硬件、格式和 SDK 私有缓冲区都已实测且始终是 RGB。

- 本项目跨节点、模型 API 和视频消息的彩色契约是 **RGB8**。不能把 BGR buffer 改名为 RGB8。
- 相机硬件原生 BGR/BGRA/RGBA/灰度只留在驱动适配范围，发布前由 OpenCV 归一到 RGB8。
- 需要 BGR 的 OpenCV 检测/编码接口只在局部边界使用 BGR，输出普通 PNG 后结束这一语义。
  OpenCV 的纯几何变换不要求 BGR，灰度和 RGBA 操作不应为了统一名词而强行转三通道 BGR。
- **第三方内部例外：Ultralytics 官方 PIL loader 会 RGB→BGR，predictor 再 BGR→RGB tensor。**
  我们没有在应用代码中补转换；最终模型张量已实测为 RGB。因此业务契约符合要求，
  但若把“只有 OpenCV 边界才能转 BGR”理解为连模型库内部也不能存在 BGR，当前实现不满足这个字面约束。
  不应为消除第三方内部的两次转换擅自另写模型预处理链路。
- 深度 Z16、二值掩码、灰度预览、透明度通道和 RGB 颜色顺序是不同概念，不能混为一谈。

## 方法级链路

| 环节 | 实际入口、处理与输出 | 核对结果 |
| --- | --- | --- |
| RealSense 驱动 | `RealSenseStream::poll_frame` → `image_plane` 按 SDK profile 记录格式、width/height/stride，原样复制 buffer | 没有擅自把 native BGR 标成 RGB；SDK Align 对齐深度，不承担颜色归一 |
| 发布前归一 | `CameraPoll::into_rgb` → `normalize_rgb`，匹配 `BGR2RGB`、`BGRA2RGB`、`RGBA2RGB`、`GRAY2RGB`；原生 RGB 直通 | 转换码与实际输入对应；支持非整通道行填充，再次处理 RGB 不换色 |
| 模拟图像 | `SimulationDriver::open` 读取源 PNG，通过 image crate 转 RGB8；`SimulationStream::poll_frame` 发布同一契约 | 不经过 BGR；本轮原生 1920 图像与运行帧完全一致 |
| 模拟标定板 | `warp_board_texture` 灰度→RGBA，OpenCV 透视变换/缩放；`render_charuco` 按 RGBA 及预乘 alpha 混入 RGB | 几何算法不解释红蓝通道；只在标定阶段叠加，不把 RGBA 当 BGRA |
| 原子 RGB-D | `camera_frame_to_arrow/from_arrow` 将元数据、彩色、深度存为独立字段 | 字节不重排；RGB8、stride、尺寸及有效载荷校验，深度单独支持 z16le/z16be |
| 彩色 PNG | camera/scene 的 `color_png` → `packed_rgb` → `RgbImage` → PNG | `packed_rgb` 只去掉行填充；没有 RGB/BGR 翻转 |
| 外参标定 | `camera_calibration::detect`：PNG→`IMREAD_COLOR_BGR`→ChArUco；灰度精修用 `BGR2GRAY`；BGR 标注→`imencode` PNG | 同一 OpenCV 局部边界内首尾匹配；`Scalar(255,0,0)` 在此代表蓝色，不是红色 |
| YOLOE 请求 | scene `segment`：RGB→PNG→Base64；HTTP 解 Base64→PIL；`YoloeBackend::segment` 传 `image.convert("RGB")` | PNG/Base64 是编码和运输，不是通道转换；没有传 RGB NumPy 给要求 BGR NumPy 的入口 |
| YOLOE 内部 | 官方 `LoadPilAndNumpy._single_check` 返回 BGR；`BasePredictor.preprocess` 转 BCHW，再翻通道和归一化 | 实测最终 tensor=原始 RGB/255；这是库内部契约，不在调用前再交换红蓝 |
| 分割掩码 | 模型 mask×255→uint8 单通道 PNG；scene 和 scene-core 用 `into_luma8` 读取 | 掩码不是 RGB 图；尺寸与彩色/深度匹配，不进行红蓝解释 |
| 分割叠加 | `segmentation_debug_image` 用 `Rgb` 调色板画框，按 `(2×原像素+调色板)/3` 混合掩码 | 本轮运行叠加图全部符合原 RGB 或上述绘制结果，没有异常颜色 |
| 深度预览 | `decode_depth` 按 z16le/z16be 显式解 u16；`depth_preview_png` 映射到 L8 灰度 PNG | 网页标出的 z16le 是原始深度编码；下载的 depth.png 是显示用灰度，不是原始 Z16 |
| GraspGenX | 正式服务传目标/环境 XYZ；官方场景加载器使用 PIL 读取 RGB 用于点云显示；所用推理颜色分量为零 | 当前抓取网络不读取真实 RGB；不能把可视化点云颜色当作参与抓取评分的颜色输入 |
| 网页图片 | gateway `binary_response` 原样转发 PNG；Next Image 使用 `unoptimized`，浏览器解码 | 不存在中途 JPEG 压缩或自定义换色；运行 PNG 与源图逐像素一致 |
| 实时视频 | `capture_worker` 在归一后分发；`video_socket` 发送 JSON 元数据+RGB8 二进制；Nginx 只代理 WebSocket | 当前为原始帧，不是 H.264/MJPEG；视频与感知上送频率独立，二者颜色约定一致 |
| Canvas | `rawFrameToRgba` 按 stride 读 RGB888，调用 `@thi.ng/pixel` 转 ABGR32，再用 RGBA 字节视图创建 ImageData | 当前小端浏览器通道顺序正确；实际浏览器全图与源图一致，不把整数 ABGR 命名误解成字节 BGR |
| 3D 标签纹理 | 浏览器 Canvas→Three.js CanvasTexture，声明 SRGBColorSpace；渲染输出同样声明 sRGB | 没有 OpenCV buffer 或 BGR 输入；光照、色调映射与 RGB/BGR 顺序是独立问题 |

对应实现：

- [RealSense 驱动](../../backend/crates/realsense-camera/src/lib.rs)、[发布前归一](../../backend/nodes/camera/src/drivers/mod.rs)、[采集 worker](../../backend/nodes/camera/src/capture_worker.rs)
- [模拟图与标定板](../../backend/nodes/camera/src/drivers/simulation.rs)、[图像消息](../../backend/crates/robot-arm-messages/src/lib.rs)、[OpenCV 标定](../../backend/crates/camera-calibration/src/lib.rs)
- [场景/图片/掩码](../../backend/nodes/scene/src/main.rs)、[三维重建](../../backend/crates/scene-core/src/lib.rs)、[计算服务](../../backend/services/perception-compute/src/perception_compute/app.py)
- [视频服务](../../backend/nodes/camera/src/video_server.rs)、[网关图片响应](../../backend/nodes/web-gateway/src/main.rs)、[浏览器视频](../../frontend/web/apps/perception/src/app/camera-video.tsx)
- [图片组件](../../frontend/web/apps/perception/src/app/page.tsx)、[反向代理](../../frontend/web/entry/nginx.conf)、[姿态纹理](../../frontend/web/packages/visualization/src/index.tsx)、[整臂纹理](../../frontend/web/apps/arm-execution/src/app/robot-viewer.tsx)

## 生成资产和诊断工具

- `generate-pick-place-scene.py`：pyrender 返回 RGBA，取前三通道保存 PIL RGB；SEG 标签使用独立 RGB
  调色板（红=方块、绿=筐、蓝=内区），不是从物体表面颜色猜标签。深度独立保存 Z16。
- `generate-charuco-board.py`：OpenCV 生成单通道板图，PIL 按 L 保存，不涉及 BGR。
- `render-camera-assets.py`：PIL RGB→彩色点云；OpenCV `applyColorMap` 的 BGR 结果立刻
  `COLOR_BGR2RGB` 后交给 PIL/渲染器，边界处理正确。
- `audit-simulation.py`、`audit-calibration-session.py`、`check-runtime-assets.py`：PIL 的 RGB/L
  模式明确；OpenCV Otsu 处理的是单通道数据，不存在 RGB/BGR 转灰参数误用。
- `render-fixture-arm.py`、`render-ranked-grasps.py`：RGB 点颜色传给 pyrender/matplotlib；
  不交换通道。点云几何坐标转换不改变 RGB。
- 官方场景诊断工具读取 `scene_rgb`/`obj["rgb"]` 直接展示；官方 `load_realworld_scene`
  使用 PIL 读 RGB。`trace-official-demo.py` 记录官方返回值，不加通道转换。

## 本轮实测

### 现有回归

| 测试 | 结果 |
| --- | --- |
| 原生 RGB/BGR/RGBA/BGRA/Y8、行填充、重复归一、OpenCV PNG 往返 | 相机及 smoke 两个测试目标各 1 项通过；未访问硬件 |
| Arrow 原子帧、格式契约、行填充、1280/1920 载荷 | 7 项通过 |
| OpenCV Rust 链接版本及标定自测 | 2 项通过 |
| 计算服务（含真实官方 loader/predictor 色块核对） | 5 项通过 |
| 前端 RGB→RGBA（含三原色、行填充、1280/1920、拒绝 native BGR 等格式） | 9 项通过 |

共 25 个测试执行通过，其中相机和 smoke 复用了同一格式测试，不是 25 种独立格式。
相机测试复用现有镜像依赖与 temp 构建缓存、挂载当前源码执行，没有用旧测试镜像源码冒充当前实现。
实际 camera 进程链接 `/opt/opencv5/lib/libopencv_*.so.500`；计算容器为 OpenCV 5.0.0、Ultralytics 8.4.135。

### 正式服务只读采样

选中来源 `simulation:pick-place-scene`；1920×1080；彩色 rgb8，深度 z16le。
不更改当前配置、不点刷新或推理，读取已有 PNG，并直接采集 3 帧视频、打开实际网页读取 Canvas。

| 对照 | 结果 |
| --- | --- |
| 源 PNG 解码 RGB ↔ WebSocket RGB | 2,073,600 像素完全一致；最大通道差 0 |
| 源 PNG 解码 RGB ↔ 网页 color.png | 全部一致；最大通道差 0 |
| 源 PNG 解码 RGB ↔ Chromium Canvas | 全部一致；最大通道差 0 |
| 源 Z16 按当前显示公式归一 ↔ 网页 depth.png | 全部一致，PNG mode=L |
| overlay.png ↔ 原 RGB/调色板/混合公式 | 1,992,462 像素未修改，81,138 像素属于正常叠加；异常颜色 0 |
| calibration.png | 当前无诊断图，返回 404；不宣称完成了当前运行态标定叠加图验收 |

前三路解码后 RGB 字节 SHA-256 均为
`ee9c995b312f4234fc742f94c281881f397e49cda552987c5cf82a636440718d`。
脚本和采样证据在 `temp/rgb-audit-20260907/`：`receive.mjs`、`compare.py`、`video.json`、
`video.rgb`、网页/Canvas PNG、`receive.json`、`comparison.json`；不把生成图片放进文档目录。

## 独立问题与未覆盖边界

1. **视频关闭握手问题，与颜色无关，已修复。** 排查基线的 `video_socket` 只发送，不读取客户端 Close/Ping。
   独立 Node WebSocket 收帧后发送 Close(1000)，3 秒后仍为 `readyState=2`、无 close 事件；
   初次采样脚本完成写报告后也未及时退出，已只终止该诊断进程。修复前证据为 `close-probe-before.json`。
   这不说明帧内容错误；后续修复和同一探针复测见下节。
2. 帧束需要上送时，视频和 RGB-D 各自持有颜色缓冲，各执行一次归一；对非 RGB 原生格式会重复计算。
   不是同一缓冲翻转两次，也不是颜色 bug；不为此次排查引入性能重构。
3. 本轮没有切换真实相机及其各 native profile；这些转换有色块回归和源码检查，不能称为逐 profile 真机验收。
4. YUYV/UYVY/MJPEG/Y16 等不支持的彩色 profile 已声明不可用，不会按 RGB 强行解释。
   设备“支持该格式”不代表当前应用已经支持解码。
5. Canvas ABGR32→RGBA 字节视图依赖当前平台的小端字节序，本轮 Linux Chromium 实测通过，未声称覆盖大端平台。
6. 本轮验证通道顺序、布局、编解码与展示一致性，不代表相机白平衡、ICC/HDR 色彩管理、深度标定或模型识别准确率验收。

后续原则仍见 [后端彩色通道约定](../BACKEND.md#彩色通道约定)：应用边界 RGB，必要的 OpenCV 边界转换局部完成；
不得对已为 RGB 的数据重复换色，不得在官方 PIL 模型入口前额外转 BGR。

## 视频预览关闭修复与验收

- **最小界面改动**：复用浮窗已有“收起/展开”，不新增按钮或控制状态。收起卸载播放器、关闭
  WebSocket；展开重新连接。收起状态显示“预览已收起”，按钮提示明确“不停止相机采集”。
- **服务端根因修复**：`video_socket` 使用现有 futures 的读写拆分和 `tokio::select!`，发送帧的
  同时读取控制消息。Ping/Pong 和 Close 响应交由 Axum/tungstenite 处理，退出前刷新关闭响应。
  无帧时也能关闭；不增加线程、队列、重试、节流或颜色转换。
- **依赖**：只为 camera 显式声明工作区已有的 futures，以及测试使用的 tokio-tungstenite；
  锁文件没有升级或新增第三方包版本。

| 验收 | 结果 |
| --- | --- |
| 相机回归 | 18 项通过；包含有帧/无帧 Ping/Pong 和 Close 握手，RGB 单像素载荷不变。未重跑无关的完整手眼标定 oracle 测试 |
| 前端像素转换 | 9 项通过 |
| 浏览器浮窗与连接生命周期 | 2 项通过：拖动/折叠持久化；连续 3 轮展开收帧、收起关闭连接，收起后相机仍在采集 |
| 原关闭探针复测 | Close(1000) 后收到正常关闭事件；3 秒观察点 `closed=true`、`readyState=3`、`closeCode=1000` |
| 静态检查 | 感知前端 typecheck、lint、Prettier，以及 Rust 格式检查、`git diff --check` 通过 |
| 部署 | 仅构建 camera 和前端应用镜像；基础镜像、OpenCV、RealSense 层复用缓存。按项目约定整体重启后验收 |

浏览器测试首次准备遗漏刷新设备列表，正式选择接口因此拒绝未枚举来源；补齐“刷新→选择→连接”
后通过，未修改生产选择逻辑。关闭探针首次在重启后的无采集状态下未收到帧，恢复模拟采集后复测通过。
测试入口为 [服务端握手测试](../../backend/nodes/camera/src/video_server.rs) 和
[浏览器浮窗测试](../../frontend/tests/browser/main.spec.ts)；修复前后探针结果位于上述 temp 证据目录。
