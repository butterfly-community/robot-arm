# OpenHMD / Monado 历史参考

> 状态：冻结的历史资料。当前项目不再维护、构建、安装或兼容本链路。

本目录保留旧 OpenHMD、Monado、OpenXR 桥、补丁、脚本和测试，仅用于追溯协议来源
与此前实验。下列命令是历史复现记录，不是当前操作手册；当前机械臂示教只使用
[`../../src/nolo-usb-server/`](../../src/nolo-usb-server/) 的 Rust USB 后端。

## 固定版本

| 项目            | 固定值                                                             |
| --------------- | ------------------------------------------------------------------ |
| OpenHMD 源码    | `~/Develop/temp/openhmd-src`                                       |
| OpenHMD 基线    | `85075b0c7e3c723ded2577edb79d00ee11aac339`                         |
| 项目补丁        | `vr-xr/reference/openhmd/patches/openhmd-nolo-cv1.patch`           |
| 补丁 SHA-256    | `01413dca19ef873a11040024553c19e7669b1b79592e40c190dcdec91d8b6c59` |
| OpenHMD 安装    | `~/Develop/temp/openhmd-install`                                   |
| Fusion 源码     | `~/Develop/temp/Fusion`                                            |
| Fusion 提交     | `d69784c8f7058a6545802b8852d26d6fcfd5e119`                         |
| Monado 源码     | `~/Develop/temp/monado-src`                                        |
| Monado 提交     | `01c1f6b23ab73c1459c8f4cfd2eb16d5142214c2`                         |
| Monado 构建目录 | `~/Develop/temp/monado-build`                                      |
| Monado 安装前缀 | `/usr/local`                                                       |

OpenHMD 已停止维护。项目分发依据是固定基线加仓库内单个补丁，不是本机历史分支。
Monado 保持未修改，项目不维护 Monado 补丁。

本机 OpenHMD 开发目录当前位于历史提交 `f6463f6`，但恢复和构建仍应从固定基线应用
项目补丁。系统包 `libopenhmd0`、`libopenhmd-dev` 已移除；安装后的 Monado 通过
RUNPATH 加载独立目录中的 `libopenhmd.so.0`。

## 补丁范围

补丁相对固定基线修改 8 个文件，增加 374 行、删除 41 行：

- 修正新版 NOLO Controller/Head Marker 的 gyro、accel 字段语义。
- 使用 Controller `1024 counts/g`、Head Marker `16384 counts/g`、真实秒时间步，以及
  当时采用但未完成定量确认的陀螺仪近似值 `0.001 rad/s/count`。当前 Rust 主链路已经
  改用 `nolo-teleop` 的 ±2000 °/s 量程解释；本补丁仅作为历史记录保留。
- 增加 `nolo_fusion.c/.h`，接入固定版本 xioTechnologies Fusion AHRS/Bias。
- 修正镜像坐标下角速度轴向量所需的符号关系。
- 为 CMake 增加外部 Fusion 源码路径，并把 Fusion C 源码编入 OpenHMD 共享库。
- 修正独立安装的 pkg-config 路径。
- 增加包解码、单位、时间步、姿态积分和加速度拒绝测试。

补丁不包含 Monado 修改、网页坐标映射或本机历史分支中的额外 Meson 改动。
当前可复现构建只使用 CMake。

## 准备源码

首次准备 OpenHMD：

```bash
git clone https://github.com/OpenHMD/OpenHMD.git ~/Develop/temp/openhmd-src
git -C ~/Develop/temp/openhmd-src checkout --detach \
  85075b0c7e3c723ded2577edb79d00ee11aac339

cd ~/Develop/job/my/embedded/robot-arm
git -C ~/Develop/temp/openhmd-src apply --check \
  "$PWD/vr-xr/reference/openhmd/patches/openhmd-nolo-cv1.patch"
git -C ~/Develop/temp/openhmd-src apply \
  "$PWD/vr-xr/reference/openhmd/patches/openhmd-nolo-cv1.patch"
```

确认已有目录已经完整应用补丁；成功且无输出表示正确，不要再次应用：

```bash
git -C ~/Develop/temp/openhmd-src apply --reverse --check \
  "$PWD/vr-xr/reference/openhmd/patches/openhmd-nolo-cv1.patch"
```

首次准备 Fusion：

```bash
git clone https://github.com/xioTechnologies/Fusion.git ~/Develop/temp/Fusion
git -C ~/Develop/temp/Fusion checkout \
  d69784c8f7058a6545802b8852d26d6fcfd5e119
```

## 检查、构建和安装 OpenHMD

只检查补丁、Fusion 固定提交、干净工作树和 NOLO 单元测试：

```bash
cd ~/Develop/job/my/embedded/robot-arm
sh ./vr-xr/reference/openhmd/tools/build-openhmd.sh --check
```

检查后构建并安装到 `~/Develop/temp/openhmd-install`：

```bash
sh ./vr-xr/reference/openhmd/tools/build-openhmd.sh
```

脚本不使用 `sudo`，不写系统目录。Fusion C 源码直接编入
`libopenhmd.so.0`，无需单独安装。

如果 Monado 已经正确记录该安装位置的 RUNPATH，只更新 OpenHMD
共享库后无需重新编译 Monado，但必须重启 `monado-service` 才会加载新库。

## 首次配置 Monado

只有首次接入维护版 OpenHMD、改变安装位置/ABI、修改头文件或 CMake 缓存指向其他
OpenHMD 时，才重新配置并安装 Monado：

```bash
PKG_CONFIG_PATH=~/Develop/temp/openhmd-install/lib/pkgconfig \
cmake -S ~/Develop/temp/monado-src -B ~/Develop/temp/monado-build \
  '-UOPENHMD_*' '-UPC_OPENHMD_*' '-Upkgcfg_lib_PC_OPENHMD_*' \
  -DOPENHMD_ROOT_DIR=/home/life/Develop/temp/openhmd-install \
  -DCMAKE_BUILD_RPATH=/home/life/Develop/temp/openhmd-install/lib \
  -DCMAKE_INSTALL_RPATH=/home/life/Develop/temp/openhmd-install/lib

cmake --build ~/Develop/temp/monado-build -j2
sudo cmake --install ~/Develop/temp/monado-build
```

验证安装结果：

```bash
ldd /usr/local/bin/monado-service | grep openhmd
readelf -d /usr/local/bin/monado-service | grep -E 'RPATH|RUNPATH'
```

两条结果都必须指向
`/home/life/Develop/temp/openhmd-install/lib`。当前本机已经验证为：

```text
libopenhmd.so.0 => /home/life/Develop/temp/openhmd-install/lib/libopenhmd.so.0
RUNPATH: /home/life/Develop/temp/openhmd-install/lib
```

## 运行与测试

先停止 Rust 后端，再启动 Monado。构建 C 测试工具：

```bash
cd ~/Develop/job/my/embedded/robot-arm
sh ./vr-xr/reference/openhmd/tools/build-tools.sh
monado-service
```

可用工具：

- `build/openhmd-enumerate`：直接枚举 OpenHMD；会独占 HID，不能与 Monado
  同时运行。
- `build/openhmd-pose-monitor`：直接读取 OpenHMD 位姿；会独占 HID。
- `build/nolo-xdev-monitor`：通过 Monado `XR_MNDX_xdev_space` 读取位姿。
- `build/nolo-controller-stream`：输出 Controller 0 OpenXR 位姿和 Menu 的 JSON
  lines。

OpenHMD/Monado 路径没有 Rust 后端的原始控制器/HMD 采样序号。重复位姿时长只用于
诊断，不能可靠区分真正静止和缓存冻结。

## 可选 IMU 诊断

正常运行不写日志。需要排查 OpenHMD 融合行为时：

```bash
NOLO_FUSION_LOG=/tmp/nolo-controller0-imu.csv monado-service
```

停止 Monado 以刷新文件，然后分析：

```bash
deno run --allow-read=/tmp/nolo-controller0-imu.csv \
  vr-xr/reference/openhmd/tests/analyze-nolo-imu.ts /tmp/nolo-controller0-imu.csv 2
```

CSV 只记录 Controller 0，包含 SI 单位
gyro/accel、四元数、加速度拒绝/恢复状态和零偏。

## 修改补丁

需要修改 OpenHMD
时，应从同一固定基线应用当前补丁，在独立分支完成代码、测试和实机
验收，再生成相对同一基线的单个补丁。更新后必须同步：

- `reference/openhmd/tools/build-openhmd.sh` 中的补丁 SHA-256；
- 本页的固定值与补丁范围；
- [历史验证记录](../../docs/CHANGELOG.md)。

只有明确升级并重新验收时才允许修改 Fusion 固定提交。

## 恢复 Debian OpenHMD

恢复系统包并不会自动改变 Monado 已写入的自定义 RUNPATH：

```bash
sudo apt-get install libopenhmd0 libopenhmd-dev
```

若要真正切回系统库，必须用 `/usr` 重新配置、构建并安装 Monado，清空自定义
build/install RPATH，最后用 `ldd` 验证。默认项目方案不使用这条恢复路径。
