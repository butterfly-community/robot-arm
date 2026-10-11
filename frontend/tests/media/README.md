# H.264 媒体回归

验证 FFmpeg 编码 → 正式视频 WebSocket → 正式 `CameraVideo` 组件 → 浏览器解码，
并检查同一视频流的 PNG 快照。输入是 Rust 测试生成的三路移动色块：外部 1920×1080，
深度彩色与腕部 1280×720。检查尺寸、RGB 通道及颜色误差、实测 FPS 显示、收起/展开、
晚加入参考帧链与编码器重建后的恢复。

这不是正式页面的绑定、保存、设备采集或抓放验收。夹具没有 Dora、USB、串口或机器人连接，
适用于正式服务不能中断时的补充检查；部署后仍须运行真实页面的 `camera-bindings` 验收。

## 运行

在工程根目录构建测试依赖镜像，不替换生产镜像：

```bash
docker build --target build-deps -t robot-arm-camera-media-build-deps \
  -f backend/nodes/camera/Dockerfile .
```

终端一启动合成视频（持续运行，直到手动停止）：

```bash
docker run --rm --name robot-arm-h264-fixture \
  -p 192.168.100.10:38081:8081 \
  -v "$PWD:/work" -w /work/backend \
  -e CARGO_TARGET_DIR=/work/temp/h264-target \
  robot-arm-camera-media-build-deps \
  cargo test --locked -p camera-node --test video_fixture \
    serve_browser_fixture -- --ignored --nocapture
```

终端二、三分别在 `frontend/` 执行：

```bash
node tests/media/server.mjs
node tests/media/playback.mjs
```

前端依赖和 Playwright Chromium 须按工程常规方式安装。测试页面位于
`http://192.168.100.10:38765/tests/media/index.html`，只代理上述隔离端口，不访问正式服务。
结束后 Ctrl+C 停止测试页面，并执行 `docker stop robot-arm-h264-fixture`。

后端编解码回归在同一构建镜像执行 `cargo test -p camera-media`；相机节点的
`charuco_h264_detection_and_hand_eye_solution_match_oracle` 测试覆盖压缩后的标定板检测与手眼求解，
需启用 `opencv-runtime`。设置 `ROBOT_ARM_TEST_IMAGE_WIDTH=1920` 运行 1080p 版本。
测试源文件保存在本目录和 `backend/nodes/camera/tests/video_fixture.rs`，缓存与输出均位于根目录 `temp/`。

同一 Rust 测试文件的 `benchmark_real_image_codec` 使用 `CAMERA_BENCHMARK_IMAGE` 指定的本地图像，
重复编码 120 帧，报告编码平均耗时和载荷大小。它隔离了 USB 等待时间，静止图像的码率不能代表动态视频。
UVC 的 `device_controls` 使用 `UVC_DIAGNOSTIC_DEVICE` 指定设备。
单独选择 `report_device_controls_without_starting_capture` 只读查询格式、帧率及曝光值；
`fixed_rate_trial` 会临时关闭动态帧率后恢复，详见根目录 `tests/README.md`。
两项都默认忽略，需要显式授权设备访问，在相机镜像中运行。

`capture_stage_benchmark` 独占设备测量采集、MJPEG 解码、H.264 编码，各阶段持续 20 秒。
运行前从正式网页关闭对应摄像头采集，结束后从网页恢复；不与生产采集同时打开同一设备。
记录原值恢复后的复测结果，不将光照变化或编译负载造成的差异误认为编码参数效果。
