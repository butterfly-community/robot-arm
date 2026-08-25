# 启动 NOLO 手柄控制 MoveIt 仿真

本文只启动仿真机械臂，不连接机械臂串口，也不会向真实机械臂发送命令。数据链路为：

```text
NOLO 实体手柄 -> Rust USB 服务 -> MoveIt Servo -> ROS 2 GenericSystem -> 网页数字孪生
```

## 1. 启动前确认

在工程根目录执行：

```bash
cd ~/Develop/job/my/embedded/robot-arm

ip -brief address
lsusb
docker version
test -d ~/Develop/temp/Star-Arm-102/.git
```

应满足：

- 本机网卡具有 `192.168.100.10`；HTTP 服务只绑定这个受信任的局域网地址。
- `lsusb` 能看到 NOLO。当前现场设备的 USB ID 是 `0483:5750`。
- Docker daemon 可用。
- 厂家仓库位于 `~/Develop/temp/Star-Arm-102`。

如果缺少厂家仓库，只需准备一次：

```bash
git clone https://github.com/servodevelop/Star-Arm-102 \
  ~/Develop/temp/Star-Arm-102
```

打开 NOLO 基站和 Head Marker，按键唤醒 Controller 0，并停止其他可能独占读取 NOLO HID
的程序。

## 2. 终端一：启动 NOLO USB 服务

推荐使用工程根目录的统一管理脚本。它在当前终端启动 NOLO 和 MoveIt 仿真并持续显示日志；
保持这个终端运行，按 `Ctrl-C` 即可结束：

```bash
./manage-simulation-services.sh start
```

重启和停止：

```bash
./manage-simulation-services.sh restart
./manage-simulation-services.sh stop
```

`restart` 会重建 `GenericSystem`，仿真关节状态将回到初始值。下面保留双终端手工启动
方法，便于直接观察两个进程的完整日志。

保持进程在前台运行：

```bash
cd ~/Develop/job/my/embedded/robot-arm

cargo run --release \
  --manifest-path vr-xr/src/nolo-usb-server/Cargo.toml -- \
  --host=192.168.100.10 --port=8765
```

首次运行可能需要编译 Rust 依赖。服务启动后可在另一个终端检查：

```bash
curl --fail http://192.168.100.10:8765/api/status
```

服务默认把机械臂输出选为 `simulation`，不会自行切到真机。

## 3. 终端二：启动 MoveIt 仿真

保持 NOLO 服务继续运行，再打开第二个终端：

```bash
cd ~/Develop/job/my/embedded/robot-arm

arm/ros2/run-moveit-simulation.sh
```

脚本会启动名为 `stararm102-moveit-simulation` 的 Docker 容器，在容器内构建 ROS 2
Jazzy 工作区，然后启动 MoveIt Servo 和 `GenericSystem`。首次运行还可能下载
`moveit/moveit2:jazzy-release` 镜像，因此会比之后启动慢。

出现 ROS 2 节点持续运行的日志后，可另开终端检查：

```bash
docker ps --filter name=stararm102-moveit-simulation
ls -l vr-xr/state/moveit-servo.sock
```

## 4. 打开网页并开始控制

推荐直接打开联合测试页：

- 手柄与机械臂并排：<http://192.168.100.10:8765/test-dashboard/>
- 只看机械臂：<http://192.168.100.10:8765/arm-simulator/>
- 只看手柄：<http://192.168.100.10:8765/>

操作顺序：

1. 确认机械臂页面顶部选择的是“仿真”。不要选择“真机”。
2. 在手柄页面选择“手柄 1”，即 Controller 0。
3. 若尚未完成姿态标定，人正对基站，触摸板朝上、手柄头部水平指向基站，保持静止并
   连续按住 Menu 6 秒。
4. 按住手柄右侧的 Squeeze；系统以当时手柄位姿和机械臂当前 TCP 建立相对控制原点。
5. 按住 Squeeze 移动或旋转手柄，仿真机械臂跟随；松开 Squeeze 立即停止接管。
6. 接管期间按下 Trigger 闭合夹爪，松开 Trigger 张开夹爪。

默认平移比例是 `0.5`：手柄平移 2 cm，目标 TCP 平移 1 cm；姿态旋转保持 1:1。
服务启动、输出切换或故障恢复时，如果 Squeeze 已经按住，系统会在当前位姿自动重新
锚定，不要求松开重按。

## 5. 两种“模拟”不要混淆

- 本文启动的是 **MoveIt 仿真机械臂**，输入来自实体 NOLO 手柄。
- 手柄网页顶部的“启动模拟”会暂停真实 HID，改用程序生成的虚拟 NOLO 报告。只有没有
  实体 NOLO、想自动测试整条数据链路时才点击它。

使用实体手柄时，不要点击网页的“启动模拟”。如果已经启用了虚拟 NOLO，点击“停止
模拟”即可：后端只停止虚拟报告并重新连接真实 HID，不联动回零。需要回零时单独点击
机械臂页的“专用回零”。

## 6. 停止服务

先在终端二按 `Ctrl-C` 停止 MoveIt 仿真，再在终端一按 `Ctrl-C` 停止 NOLO 服务。
仿真容器使用 `--rm` 启动，正常停止后会自动删除；运行状态不会写入真实机械臂。

## 7. 常见问题

### 网页打不开

确认主机确实具有 `192.168.100.10`，终端一没有报地址绑定失败，并检查：

```bash
curl --fail http://192.168.100.10:8765/api/status
```

服务没有登录认证，不要改成绑定 `0.0.0.0` 或暴露到不受信任网络。

### NOLO 存在，但页面没有新姿态

确认基站和 Head Marker 已开启，按键唤醒手柄并实际移动它。NOLO HID 只能由一个程序
独占读取；若另一个采集程序正在运行，先停止它，再重启终端一的服务。

### 手柄在动，机械臂不跟随

依次确认：

1. MoveIt 容器仍在运行；
2. `vr-xr/state/moveit-servo.sock` 存在；
3. 网页输出为“仿真”；
4. Controller 0 的通信和姿态标定正常；
5. Squeeze 当前处于按下状态。

可在 `/api/status` 中查看 `latestTeleopIntent`、`latestArmSimulation` 和
`armOutputBackend` 的当前状态与停止原因。

### MoveIt 脚本提示缺少厂家源码

把公开仓库克隆到 `~/Develop/temp/Star-Arm-102`，或显式指定已有源码：

```bash
STAR_ARM_102_SOURCE=/绝对路径/Star-Arm-102 \
  arm/ros2/run-moveit-simulation.sh
```

脚本只在容器的临时副本中应用本项目补丁，不会修改厂家仓库。
