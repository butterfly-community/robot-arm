# Robot Arm Services

本目录是前后端分离的 Dora 服务化实现：

- `backend/`：Rust/Dora 节点、FashionStar UART、ROS 2/MoveIt 型号节点和后端镜像。
- `frontend/`：四个 Next.js 应用、共享 UI/契约/客户端和单一前端镜像。
- 根目录：Compose、Dora dataflow、跨端验收、记录工具和当前设计文档。

## 启停

全部命令都在本目录执行：

```bash
docker compose up -d
docker compose down
```

重启就是依次执行上面的 `down` 和 `up -d`。项目不提供管理脚本，也不维护单服务重启状态。
Compose 停止 dataflow 时会向 Dora attach 会话发送 `SIGINT`，由 Dora 自己停止全部节点，避免下次启动重复部署。
需要重新构建时使用：

```bash
docker compose up -d --build
```

motion 镜像默认从 `~/Develop/temp/Star-Arm-102` 读取机械臂模型，并从
`~/Develop/temp/moveit2-2.12.4/moveit_ros/moveit_servo` 读取 MoveIt Servo 2.12.4 源码。
路径不同时分别用 `STAR_ARM_102_SOURCE` 和 `MOVEIT_SERVO_SOURCE` 覆盖；后者应直接指向
`moveit_servo` 软件包目录。

页面入口为 `http://192.168.100.10:8765/`，业务路径是 `/tracking/`、`/spatial/`、
`/motion/` 和 `/arm-execution/`。只有入口服务暴露主机端口。

## 配置

可修改的服务配置保存在宿主 `backend/config/runtime/`，Compose 将这个目录挂载为容器内
`/config`。目录中的 JSON 被 Git 忽略，但 `.gitignore` 本身受版本控制，因此 `down`、重新构建和
`up -d` 都不会丢失配置。采集、空间、motion 和执行节点各自只读写自己的文件；文件不存在时使用
默认值，已有文件损坏时服务直接报告启动错误。

`backend/config/service-status.json` 是受版本控制的静态依赖配置。实时姿态、控制会话、模拟状态、
关节反馈、串口连接状态和错误不会写入配置文件。

## 开发检查

```bash
cd backend
cargo fmt --all -- --check
cargo test --workspace --all-targets
cargo clippy --workspace --all-targets -- -D warnings

cd ../frontend
pnpm format:check
pnpm lint
pnpm typecheck
pnpm test
pnpm build
pnpm test:e2e

cd ..
docker compose exec -T stararm-102-motion bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /opt/ros_ws/install/setup.bash && \
   python3 -m unittest discover -s /opt/ros_ws/src/stararm_102_motion_node/test -v'
docker compose --profile test run --rm integration-test
```

架构与行为见 [docs/SUMMARY.md](docs/SUMMARY.md)，后端逐服务、逐方法和依赖边界见
[docs/BACKEND.md](docs/BACKEND.md)，测试用记录/回放工具见
[tools/replay/README.md](tools/replay/README.md)。
