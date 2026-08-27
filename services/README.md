# Robot Arm Services

本目录是前后端分离的 Dora 服务化实现：

- `backend/`：Rust/Dora 节点、FashionStar UART、ROS 2/MoveIt 型号节点和后端镜像。
- `frontend/`：四个 Next.js 应用、共享 UI/契约/客户端和单一前端镜像。
- 根目录：Compose、Dora dataflow、跨端验收、记录工具和实施文档。

## 启停

全部命令都在本目录执行：

```bash
docker compose up -d
docker compose down
```

重启就是依次执行上面的 `down` 和 `up -d`。项目不提供管理脚本，也不维护单服务重启状态。
需要重新构建时使用：

```bash
docker compose up -d --build
```

页面入口为 `http://192.168.100.10:8765/`，业务路径是 `/tracking/`、`/spatial/`、
`/motion/` 和 `/arm-execution/`。只有入口服务暴露主机端口。

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

运行事实、不可执行的现场项目和原因见 [docs/PROBES.md](docs/PROBES.md)，记录/回放见
[docs/RECORDING.md](docs/RECORDING.md)。
