# Dora 记录与回放

记录文件写入 `recordings/`（记录内容由 Git 忽略），每个 `.drec` 必须使用同名
`.manifest.json`。清单从四个服务的真实快照读取构建版本、配置版本和模型哈希，不接受手写副本。

下面的示例只旁路读取正在运行的数据流，不提交控制请求：

```bash
python3 tools/recording_manifest.py start recordings/session.manifest.json \
  --source openxr-source-node/absolute_pose \
  --source openxr-source-node/control_input \
  --source spatial-transform-node/relative_motion \
  --source stararm-102-motion-node/arm_command \
  --source stararm-102-execution-node/arm_state
docker compose exec dataflow dora record --proxy --name robot-arm-services \
  --topics openxr-source-node/absolute_pose,openxr-source-node/control_input,spatial-transform-node/relative_motion,stararm-102-motion-node/arm_command,stararm-102-execution-node/arm_state \
  -o /recordings/session.drec /config/dataflow.yml
python3 tools/recording_manifest.py finish recordings/session.manifest.json
```

第二条命令使用 `Ctrl-C` 结束。若进程或主机异常退出导致 `ended_at_ns` 仍为 `0`，清单明确表示
记录未正常结束，不能当成完整回归样本。

先生成回放图并审查被替换的节点和输出：

```bash
docker compose exec dataflow dora replay /recordings/session.drec \
  --output-yaml /recordings/session.replay.yml
```

输入记录用于独立软件回归时，只记录输入源并在未连接执行端点的干净 Compose 实例回放。回放
不会复用采集节点的零偏或网页状态；空间原点和每轮接管基准仍由记录中的输入动作建立。包含
`ArmCommand` 的全链路记录用于对照，不作为输入再次驱动执行节点。

无需真实设备的确定性空间回归使用
`tests/fixtures/openxr-synthetic-cycle.json`。它是在真实 OpenXR 链路确认消息、单位和姿态标志
正常后独立生成的输入，不复制现场位置或 IMU 零偏；当前覆盖六向 2 cm、竖直和水平圆弧正反
8°以及返回基准，并由 `spatial-core` 单元测试逐字段核对。
