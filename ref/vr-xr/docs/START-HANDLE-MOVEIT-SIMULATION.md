# 启动 NOLO 手柄控制 MoveIt 仿真

本文启动统一机械臂服务但不连接串口，因此只使用软件反馈，不会向真实机械臂发送命令：

```text
NOLO 实体手柄 -> Rust USB 服务 -> MoveIt Servo -> JointStateTopicSystem -> Rust 软件反馈 -> 网页
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

- 本机网卡具有 `192.168.100.10`；Compose 不使用 host 网络，只把 HTTP `8765` 发布到
  这个局域网地址，MoveIt 不发布端口。
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

## 2. 后台启动完整服务

在工程根目录通过 Docker Compose 同时启动 NOLO 和 MoveIt：

```bash
docker compose up -d
```

停止：

```bash
docker compose down
```

完整重启：

```bash
docker compose down
docker compose up -d
```

`docker compose up -d` 返回后两个服务继续在后台运行。
首次构建会在镜像内编译 Rust 和 ROS；之后未修改源码时直接复用构建缓存。冷启动 J1–J6 为
`[0°, 0°, -3°, 0°, 0°, 0°]`，夹爪为 `+1°`。查看两个服务的连续日志：

```bash
docker compose logs --follow nolo-usb-server moveit
```

服务启动后检查：

```bash
curl --fail http://192.168.100.10:8765/api/status
```

服务未连接用户选择的串口时始终使用软件反馈。MoveIt 镜像已经包含应用补丁并编译完成的
ROS 2 Jazzy 工作区，运行容器只启动 MoveIt Servo 和统一 `JointStateTopicSystem`。检查容器：

```bash
docker compose ps
```

Rust 与 ROS 通过 Compose 专用卷中的 `/ipc/moveit-servo.sock` 通信，不再把 socket 留在
宿主机 `vr-xr/state`。

## 3. 打开网页并开始控制

推荐直接打开联合测试页：

- 手柄与机械臂并排：<http://192.168.100.10:8765/test-dashboard/>
- 只看机械臂：<http://192.168.100.10:8765/arm-simulator/>
- 只看手柄：<http://192.168.100.10:8765/>

操作顺序：

1. 确认机械臂页面反馈来源是“ROS 软件反馈”。
2. 在手柄页面选择“手柄 1”，即 Controller 0。
3. 若尚未完成姿态标定，人正对基站，触摸板朝上、手柄头部水平指向基站，保持静止并
   连续按住手柄右侧的 Squeeze 6 秒。
4. 按住 Trigger；系统以当时手柄位姿和机械臂当前 TCP 建立相对控制原点。
5. 按住 Trigger 移动或旋转手柄，仿真机械臂跟随；松开 Trigger 立即停止接管。
6. 接管期间按下 Menu 闭合夹爪，松开 Menu 张开夹爪。

默认平移比例是 `0.5`：手柄平移 2 cm，目标 TCP 平移 1 cm；姿态旋转保持 1:1。
服务启动、输出切换或故障恢复时，如果 Trigger 已经按住，系统会在当前位姿自动重新
锚定，不要求松开重按。

## 4. 两种“模拟”不要混淆

- 本文启动的是 **MoveIt 仿真机械臂**，输入来自实体 NOLO 手柄。
- 手柄网页顶部的“启动模拟”会切到手柄控制，暂停真实 HID 并直接改用程序生成的
  虚拟 NOLO 报告；它不要求起始位，不提交机械臂或夹爪命令。只有没有实体 NOLO、想自动测试
  整条数据链路时才点击它。

使用实体手柄时，不要点击网页的“启动模拟”。如果已经启用了虚拟 NOLO，点击“停止
模拟”即可：后端只停止虚拟报告并重新连接真实 HID，不联动返回起始位。需要时单独点击
机械臂页的“J1–J6 起始位”，它提交普通六轴运动请求，目标为
`[0°, 0°, -3°, 0°, 0°, 0°]`。

## 5. 停止服务

```bash
docker compose down
```

该命令正常结束并删除两个 Compose 容器；Rust 收到结束信号后删除 IPC 卷里的 Unix socket。
即使上一次异常退出留下旧 socket，下次启动也会替换它，因此无需额外清理命令。运行状态
不会写入真实机械臂。

## 6. 常见问题

### 网页打不开

确认主机确实具有 `192.168.100.10`，Compose 日志没有地址绑定失败，并检查：

```bash
curl --fail http://192.168.100.10:8765/api/status
```

服务没有登录认证，不要改成绑定 `0.0.0.0` 或暴露到不受信任网络。

### NOLO 存在，但页面没有新姿态

确认基站和 Head Marker 已开启，按键唤醒手柄并实际移动它。NOLO HID 只能由一个程序
独占读取；若另一个采集程序正在运行，先停止它，再用 Compose 重启服务。

### 手柄在动，机械臂不跟随

依次确认：

1. `docker compose ps` 显示两个服务都在运行；
2. `docker compose logs moveit nolo-usb-server` 没有 IPC 连接错误；
3. 网页反馈来源为“ROS 软件反馈”；
4. Controller 0 的通信和姿态标定正常；
5. Trigger 当前处于按下状态。

可在 `/api/status` 中查看 `latestTeleopIntent`、`latestArmState` 和
`serialState` 的当前状态与停止原因。

### Compose 构建提示缺少厂家源码

把公开仓库克隆到 `~/Develop/temp/Star-Arm-102`，或显式指定已有源码：

```bash
STAR_ARM_102_SOURCE=/绝对路径/Star-Arm-102 \
  docker compose up -d
```

Docker 构建只复制厂家仓库中的两个 ROS 包，再在镜像构建层应用本项目补丁，不会修改厂家
仓库。
