# NOLO CV1 Rust USB 后端

这是当前默认数据后端。它通过 HIDAPI/libusb 独占读取 NOLO CV1，兼容现场 USB ID
`0483:5750` 和生产版 `28e9:028a`，不经过 OpenHMD、Monado 或 OpenXR。

后端会解密 64 字节新版报告，把 Controller 0、Controller 1 和 Head Marker
分别发布给网页，并为三者维护独立的 Rust `fusion-ahrs` 状态。

## 能力与限制

输出包括：

- 基站跟踪空间中的两只手柄和 Head Marker 位置；
- 三个设备各自的 `[x, y, z, w]` 融合四元数；
- Menu、Trigger、Home、Squeeze、Touchpad 按键和触摸板坐标；
- 原始加速度计、陀螺仪和采样序号诊断数据。

后端只在对应设备采样序号发生变化时更新 Fusion 和发布新姿态。重复的旧负载不会再次
积分；USB 完全静默超过 200 ms 时，所有已发布来源都会转为不可用。

状态字段的准确含义：

- `sample_sequence`：当前设备采样序号；手柄取 `report[24]`，Head Marker 取
  `report[59]`。
- `unchanged_ms`：当前设备的 `sample_sequence` 多久没有变化。
- `communication_fresh`：上述时长不超过 200 ms；`source_online` 是兼容别名。
- `sample_changed`：这一帧是否来自新的设备采样序号。
- `pose_usable`：本帧同时满足通信新鲜且是新样本；这仍不表示光学跟踪有效。
- `optical_tracking_valid`：固定为 `null`，直至协议中找到经过验证的光学有效标志。
- `controller_sequence`：手柄帧为 `report[24]`，头部帧为 `null`。
- `hmd_unchanged_ms`：`report[59]` HMD/USB 中继采样序号多久没有变化。
- `hmd_relay_online`：上述时长不超过 200 ms。
- `sequence_delta`、`samples_received`、`samples_missed`、`duplicate_reports`：序号连续性
  累计统计；序号按 `u8` 回绕处理。
- `measured_rate_hz`、`sample_jitter_ms`：按新样本间隔计算的指数移动统计。
- `flags`：仅为旧 API 兼容保留，固定为 `0`，禁止解释为 OpenXR tracking flags。

这些字段只判断通信/采样新鲜度。它们不能判断基站是否开机，不能判断光学位置是否
有效，也不是 OpenXR runtime 返回的真实 tracking flags。`time_ns`
是后端进程启动后的单调时间，不是设备硬件时间戳。

手柄同时输出三种位置字段：

- `marker_position`：NOLO 报告中的原始光学标记位置。
- `grip_position`：根据历史 NOLO SDK 旋转中心偏移
  `[0, -0.0045, 0.0755] m` 计算的实验性握持点位置。
- `position`：当前主位置；默认等于 `marker_position`。

`grip_position` 的旋转方向和符号仍需实机验证。默认设置不会改变现有空间位置；只有
显式使用 `--controller-position=grip` 才会让 `position` 采用估算握持点。

姿态融合没有已确认的磁力计或绝对旋转观测，三路 yaw 都可能漂移。USB 端点总计
约 240 包/秒：两个 Controller 报告各约 120 包/秒；HMD 字段和 `report[59]`
出现在每包中，因此 Head Marker 融合按约 240 Hz 更新。

两种 Controller 报告内的 HMD 序号相位不同。后端分别跟踪两条约 120 Hz 的
`report[59]` 流，再合并其新样本和统计；禁止直接对相邻异类报告的字节 59 做差。

## 构建与运行

先停止所有会争用同一个 HID 的其他程序。

构建：

```bash
cd ~/Develop/job/my/embedded/robot-arm/vr-xr/src/nolo-usb-server
cargo build --release
```

本机运行：

```bash
./target/release/nolo-usb-server
```

指定局域网地址和端口：

```bash
./target/release/nolo-usb-server --host=192.168.100.10 --port=8765
```

可选参数：

```text
--host=IP
--port=PORT
--static-dir=PATH
--controller-position=marker|grip
```

默认静态目录为相邻的
`controller-viewer/public/`。服务没有登录认证；远程使用时只绑定
受信任局域网中的具体地址，不要绑定 `0.0.0.0`。

## HTTP 接口

- `/`：实时网页。
- `/events`：`status` 和 `pose` Server-Sent Events。
- `/api/status`：当前状态、最后收到的一帧 `latestFrame`，以及三个设备各自最新帧
  `latestFrames`。
- `POST /api/gyro-calibration/0|1|2`：开始对应设备的显式陀螺仪零偏标定。
- `POST /api/pose-calibration/0|1|2`：重新初始化对应设备的 Fusion，同时开始陀螺仪
  零偏标定；网页 6 秒标定流程会自动调用。

`pose` 帧以 `source_id=0/1/2` 区分 Controller 0、Controller 1 和 Head Marker；
`sample_rate_hz` 给出名义采样率，`sample_sequence` 和 `source_online` 对三种设备具有
统一语义。网页的“手柄 1 / 手柄 2 / 头部”按钮只切换显示，不会停止其他设备采集。

内部解码、Fusion 和 `/api/status` 快照保持设备原生更新率。面向查看器的 SSE 对每个
来源限制为最高约 60 Hz，离线/恢复转换会立即发送，以免网页 JSON 序列化和网络传输
积压。机械臂控制不得使用 SSE；后续应接本机有界 latest-value IPC。

收到 Ctrl-C 或 SIGTERM 时，后端会通知现有 SSE 流结束，再退出进程并释放 USB。

网页 Three.js 固定从
`https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.module.js` 加载，工程不保存
Three.js 源码副本；浏览器必须能访问该 CDN。

## 陀螺仪零偏标定

手柄记录原点和零姿态时，连续长按 Menu 6 秒会自动调用完整位姿标定：第一秒只用于
按键后摆稳，进入第二秒时后端重新进入 Fusion 官方初始化阶段，同时清空 Offset 并
开始连续 3 秒陀螺仪零偏采集；网页只在 Fusion 初始化和零偏采集都成功后，才使用
最后约 1.2 秒样本记录原点与零姿态。按键后未能连续静止满 3 秒时，本次标定失败且
不会覆盖已有网页标定。

网页诊断区可以为当前设备启动标定，也可以直接调用 HTTP 接口。点击后将设备平稳放在
桌面并保持静止。后端直接使用 `OffsetSettings::default()` 的持续时间和角速度门限；
任何超过 Fusion 默认静止门限的样本都会清零连续计时。

标定结果只保存在当前进程内，重启后失效。运行时 Fusion Offset 直接使用
`OffsetSettings::default()`，用于补偿标定后的残余慢漂；网页会显示零偏、进度、
加速度拒绝和恢复状态。

AHRS 使用 `Ahrs::new()`，全部算法参数由当前 `fusion-ahrs` 版本的
`AhrsSettings::default()` 提供，本工程不复制或覆盖默认值。NOLO 没有磁力计，因此
更新接口使用 `update_no_magnetometer()`。

后端不再叠加自定义快速收敛或动态增益。快速动作后俯仰/侧倾会按照 Fusion 的标准
重力反馈自然收敛；无磁力计时 yaw 不会由加速度计纠正。

## 采样前确认设备状态

NOLO CV1 手柄具有省电休眠/关机机制。手柄之前开过机，或者 HMD 中继仍持续收到
对应报告类型，都不能证明采样时手柄仍活跃；报告可能重复旧负载。

进行协议分析、位姿验收或示教采集前必须：

1. 分别确认基站、Head Marker 和目标手柄当前已开机。
2. 按键并移动每只待测手柄，使其退出省电状态。
3. 单独移动 Controller 0、Controller 1，确认对应位置或 IMU 原始负载确实变化。
4. 在采集记录中写明每个设备的开关机、唤醒和动作确认状态。

缺少任一项时，数据只能标记为排查样本，不能用于解释未知字节。

## 未知字段探针

`nolo-unknown-probe` 会同时读取两个 Controller 报告，统计 IMU 负载变化，以及
尚未命名的字节 22、23、31–36、43–48、55–58、60–63。运行前必须停止网页后端和
其他 USB 读取程序，并按上一节确认两只手柄状态。

```bash
cargo build --release --bin nolo-unknown-probe
./target/release/nolo-unknown-probe 12
```

参数是采样秒数，默认 10 秒。探针只打印统计结果，不保存原始抓包，也不修改设备。
协议结论统一维护在 [NOLO-CV1-PROTOCOL.md](NOLO-CV1-PROTOCOL.md)。

## 测试

```bash
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
cd ../..
deno test --allow-read tests
```

测试不访问 USB；实机采样必须单独执行探针或启动后端。
