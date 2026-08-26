# Star Arm 102-FL 统一 MoveIt 服务

ROS 只运行一套 `JointStateTopicSystem`、controller、MoveIt Servo、MoveGroup 和 IPC bridge。
是否驱动真机不改变 ROS：Rust `ArmJointIo` 未连接串口时回写软件反馈，连接串口后写入舵机并
用 ID 0–6 Monitor 覆盖同一个反馈状态。

## 启动

厂家仓库按根目录约定放在 `~/Develop/temp/Star-Arm-102`，或用绝对路径环境变量
`STAR_ARM_102_SOURCE` 指定。完整服务统一由根目录 Compose 管理：

```bash
docker compose up -d
docker compose down
```

第一条启动，第二条停止；重启就是依次执行这两条。MoveIt 镜像构建时复制厂家模型、
应用项目补丁并完成 `colcon build`；网页镜像在同样的补丁后厂家副本上运行
`vr-xr/tools/prepare-controller-viewer.ts` 生成 URDF/meshes，Rust 阶段完成 `cargo build --release`。启动容器时
不再编译，也不需要 systemd 或宿主机进程管理脚本。

Compose 不使用 host 网络。HTTP 容器内监听 `8765`，宿主机只发布
`192.168.100.10:8765`；MoveIt 不发布端口。两个服务只通过专用 `servo-ipc` 卷中的
`/ipc/moveit-servo.sock` 通信；`docker compose down` 会正常结束 Rust 服务，由 Rust 删除
socket。若进程异常退出留下 socket，下次启动会直接替换该旧 socket，无需手工进入卷清理。

## 运行时串口

网页输入串口路径并点击连接，或者调用：

```bash
curl -X POST http://192.168.100.10:8765/api/arm-serial/connect \
  -H 'content-type: application/json' \
  -d '{"port":"/dev/ttyUSB0"}'
```

Rust 先 Ping 并 Monitor 读取 ID 0–6，不先发送运动。连接不检查软件或真机起始位；读取成功后
直接把真机 J1–J6 和夹爪写入统一状态。失败时继续软件反馈。

厂家 UART 帧、流式收包、Monitor 和同步位置命令由独立
[`fashionstar-uart`](../src/fashionstar-uart/README.md) crate 负责；机械臂层只映射 J1–J6
顺序、弧度，以及本机实测确认的 J4/ID 3 和夹爪 ID 6 符号。断开不移动、不返回起始位，
并保留最后反馈。运行中串口读写错误保留最后真机反馈，随后释放旧句柄并立即重开同一端口；
重开成功后继续读写，失败则显示实际错误并回到未连接状态，不增加重试次数门限或额外恢复
状态机：

```bash
curl -X POST http://192.168.100.10:8765/api/arm-serial/disconnect
```

## 控制路径

- MoveIt arm 链为 `base_link` 到厂家已有的 `link6`；手柄和反馈都使用该末端 link。当前不
  添加未经实测的额外 TCP 坐标系。
- 六轴 IK 使用 `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`，替换厂家 KDL；使用
  Jazzy 2.0.2 插件默认 `Distance` 模式和厂家已有的 `5 ms` timeout。
- 位置范围以厂家新版 URDF 为唯一来源；补丁只把厂家 ros2_control 中不一致的 J5 和夹爪
  command interface 同步为 URDF 的 `[-2.27, 2.27]`。项目不按产品表覆盖，也不在 Rust
  或网页源码中维护模型副本或第二套位置限位。
- 手柄目标由 MoveIt Servo 写入 `arm_controller`。
- 网页手动 J1–J6 和起始位运动（目标为 `[0°, 0°, -3°, 0°, 0°, 0°]`）都提交普通
  `MotionRequest`，由 MoveGroup 规划并写入同一 controller。
- 夹爪绝对角由 `hand_controller` 独立执行。
- `JointStateTopicSystem` 把七关节设定值发布到 `/stararm102/joint_commands`，Rust 返回的统一
  `ArmState` 发布到 `/stararm102/joint_states`。
- 动力学补丁保留此前依据官方资料确认的 J1–J6 `30 rad/s²`。

## 静态验收

```bash
bash -n arm/ros2/start-moveit.sh
docker compose config --quiet
python3 -m compileall -q arm/ros2/stararm102_teleop_moveit
python3 -m unittest discover -s arm/tests -p 'test_*.py' -v
```
