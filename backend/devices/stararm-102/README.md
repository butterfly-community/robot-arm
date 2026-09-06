# StarArm-102 设备域

此目录集中保存 StarArm-102 独有的实现：

- `crates/model`：关节、工具执行器、命名位、模型资源和抓取描述能力；
- `nodes/motion`、`nodes/execution`：该型号的运动与执行服务；
- `ros2`：MoveIt、Servo 和 MTC 适配；
- `patches`：只适用于厂家 StarArm-102 源码的补丁；
- `graspgenx`：从最终型号模型生成夹爪资产所需的最小语义清单；
- `tools/verify-model.py`：构建时分别验证固定厂商提交和补丁后成品的关节轴、范围、夹爪联动
  与 TCP，防止角度定义再次漂移；
- `Dockerfile.base` 与 `docker/prepare-vendor.sh`：固定、校验、应用型号补丁并验证厂商资产；
- `nodes/motion/Dockerfile` 与 `docker/install-ros.sh`：只在运动服务中安装 ROS/MoveIt 并构建
  设备 ROS 包；
- `nodes/execution/Dockerfile`：只构建串口执行服务；
- `docker/motion-entrypoint.sh`、`docker/rviz-index.html`：本型号运动入口和 RViz 页面。

FashionStar 串口帧解析仍是可复用的厂家协议库，保留在
`backend/crates/fashionstar-uart`。通用消息、输入、空间转换、感知、状态和网页服务不得包含
本型号的关节数、命名位、TCP 或夹爪几何。完整型号事实与验收记录见
[`docs/STARARM-102.md`](../../../docs/STARARM-102.md)。
