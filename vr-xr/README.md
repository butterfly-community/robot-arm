# NOLO CV1 机械臂示教子系统

本目录保存 NOLO CV1 手柄示教所需的当前运行程序、网页、协议记录和测试。旧
OpenHMD/Monado 链路已经
冻结到 `reference/openhmd/`，只保留为历史参考，不再维护或提供兼容支持。

## 当前方案

| 项目           | 当前决定                                         |
| -------------- | ------------------------------------------------ |
| 示教设备       | 最终示教仍选一只手柄；查看器可切换两只手柄和头部 |
| 默认数据链路   | Rust `nolo-usb-server` 直接独占读取 USB/HID      |
| 姿态算法       | Rust `fusion-ahrs`，分别融合各设备的陀螺仪和加速度计 |
| 空间位置       | 使用 NOLO 基站给出的设备位置                     |
| Head Marker    | 发布位置和约 240 Hz IMU 融合姿态，同时承担 USB 中继 |
| Controller 1   | 与 Controller 0 一样发布；网页按钮切换观察       |
| OpenHMD/Monado | 历史参考；不属于当前方案，不再维护或兼容         |

当前链路不经过 OpenHMD、Monado 或 OpenXR。历史目录中的旧实现不是当前运行时依赖，
也不进入当前测试与验收。

## 已确认状态与边界

已确认：

- 现场 CV1 的 USB ID 为 `0483:5750`；Rust 后端可以解密新版 64 字节报告。
- 两只手柄的位置、原始 IMU、按键和独立融合四元数，以及 Head Marker
  的位置和融合四元数可以通过 HTTP/SSE 连续输出。
- 两只手柄唤醒后的受控采样确认了报告类型、未知字段行为和采样序号，详见
  [协议说明](docs/NOLO-CV1-PROTOCOL.md)。
- Controller 0 的前后、左右、上下空间关系和自身三轴姿态已经完成实机定性验收；
  快速旋转停止后的姿态表现正常。
- 远程网页的设备切换、空间视图和一次 Menu 长按 6 秒完整标定已经实机通过；第一秒
  摆稳、Fusion 初始化、零偏采集、原点和零姿态记录均按预期工作。

尚未确认或尚未放行：

- 已通过的位姿验收是定性结果；位置误差与抖动、角度尺度、yaw 漂移和长时间稳定性仍需
  使用可靠参考进行定量测量。
- Controller 1 和 Head Marker 尚未完成与 Controller 0 同等级的独立实机验收。
- `communication_fresh`（兼容别名 `source_online`）只表示当前设备采样序号在
  200 ms 内继续变化；`pose_usable` 还要求当前帧是新样本。它们不证明基站已开机，
  也不证明光学位置当前有效。旧 `flags` 已固定为零，不能用作跟踪判断。
- `hmd_relay_online` 只表示 HMD/USB 中继序号继续变化，不能表示基站状态。
- 当前协议没有已确认的磁力计或绝对旋转观测。重力只能修正俯仰和侧倾，yaw
  仍可能漂移。
- 人体坐标标定不是机械臂基坐标外参。
- 真实通电机械臂尚未放行。外参、使能、超时、跟踪失效、跳变、限速、工作空间和急停
  保护均未完成，详见 [TODO](docs/TODO.md)。

## 启动默认网页

先确认：

1. 基站和 Head Marker 已开机并连接。
2. 待看的手柄已按键唤醒；移动后原始位置或 IMU 确实变化。
3. 其他会独占同一个 NOLO HID 的读取程序均已停止。

构建并启动：

```bash
cd ~/Develop/job/my/embedded/robot-arm

cargo build --release --manifest-path vr-xr/src/nolo-usb-server/Cargo.toml

./vr-xr/src/nolo-usb-server/target/release/nolo-usb-server \
  --host=127.0.0.1 --port=8765
```

本机打开 `http://127.0.0.1:8765/`。远程访问时把 `127.0.0.1` 换成机器的具体、
受信任局域网地址。服务没有认证，不要直接绑定 `0.0.0.0` 或暴露到不受信任网络。

后端参数、接口和协议诊断工具见
[Rust 后端说明](docs/NOLO-USB-SERVER.md)。

默认 `position` 是原始光学标记位置。历史 SDK 推导的实验性握持点会同时输出为
`grip_position`，但在完成实机符号验证前不要使用
`--controller-position=grip`。网页诊断区可以在设备平放时执行连续静止 3 秒的
显式陀螺仪零偏标定。

## 网页标定

网页顶部可切换手柄 1（Controller 0）、手柄 2（Controller 1）和头部。两只手柄
各自保存独立的页面内标定；切换不会混用标定。手柄标定会同时记录位置原点、
相对姿态零点和人体前方：

1. 人正对基站，手柄距基站至少 50 cm。
2. 触摸板朝上，手柄头部水平指向基站，尾部朝向胸口。
3. 保持静止并连续按住 Menu 6 秒；第一秒用于按键后摆稳，从第二秒开始并行完成
   Fusion 初始化和陀螺仪零偏标定，页面使用最后约 1.2 秒有效样本记录原点和零姿态。
4. 页面把“标定位置指向基站原点”的水平向量定义为人体前方。

位置输出采用人体坐标：右 `+X`、上 `+Y`、前 `-Z`。姿态输出是相对标定姿态，
不提供绝对 yaw，也不替代机械臂外参标定。

头部没有 Menu 键，页面直接按基站跟踪空间显示位置，姿态零点是 Rust 后端启动时的
Fusion 初始状态。USB 报告总频率约 240 Hz：Controller 0/1 报告交替出现，所以
每只手柄约 120 Hz；每个报告都携带一套新的 HMD 字段，两种报告类型分别维护递增的
HMD 序号流，合并后的头部链路约 240 Hz。网页通过固定版本 jsDelivr CDN 加载
Three.js，远程浏览器需能访问 `cdn.jsdelivr.net`。

## 手柄省电与采样要求

手柄具有省电休眠/关机机制。收到某个 Controller 类型的报告并不能证明手柄仍活跃，
HMD 中继可能重复旧负载。每次协议抓包、位姿验收或示教采集前必须：

1. 分别按键唤醒所有待测手柄。
2. 依次单独移动每只手柄，确认对应位置或 IMU 原始负载变化。
3. 记录基站、Head Marker、Controller 0/1 的开关机和唤醒状态。

未完成以上确认的采样只能用于排查，不能用于给未知字节命名或推翻既有结论。

## 历史参考

旧 OpenHMD、Monado、OpenXR 桥、补丁、构建脚本和相关测试统一归档在
[`reference/openhmd/`](reference/openhmd/)。该目录用于理解历史实现和协议来源，
不是备用运行链路；当前项目不再修复、构建、安装或承诺兼容它。

## 目录

- [`src/`](src/)：运行代码；包含 Rust USB 后端和共用网页。
- [`tests/`](tests/)：当前主线测试，不放生产代码或历史兼容测试。
- [`docs/`](docs/)：TODO、验证记录、协议和运行维护说明。
- [`reference/`](reference/)：已冻结的历史实现；不进入当前构建和维护。

网页与 Rust 测试：

```bash
deno test --allow-read=vr-xr/src/controller-viewer/public \
  vr-xr/tests/controller-viewer-math_test.ts \
  vr-xr/tests/controller-viewer-ui_test.ts

cargo test --all-targets --manifest-path vr-xr/src/nolo-usb-server/Cargo.toml
cargo clippy --all-targets --manifest-path vr-xr/src/nolo-usb-server/Cargo.toml -- -D warnings
```

## USB 恢复

如果控制器采样序号停止或 USB 读取报错：

1. 停止当前 Rust 后端和所有直接访问 NOLO 的其他程序。
2. 确认目标手柄没有进入省电状态。
3. 必要时复位现场 USB 设备：

```bash
sudo usbreset 0483:5750
```

4. 只重新启动一条数据链路。

USB 复位只能恢复通信，不能证明基站光学跟踪有效。
