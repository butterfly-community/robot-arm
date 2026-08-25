# Star Arm 102 可视化资产

本目录的 URDF 几何和 9 个 STL 网格未经几何修改，来自本地上游仓库：

- 仓库：`servodevelop/Star-Arm-102`
- 提交：`5979b346eb3a417840b29b76740754e4005d071a`
- 来源：`ROS2_HUMBLE/src/stararm102_description/{urdf,meshes}`
- 上游 README 声明的许可证：MIT

这里只保留浏览器数字孪生需要的可视化文件；页面不读取碰撞、惯量、传动或硬件控制
数据。保留 URDF 中的 7 个 `<limit>` 已覆盖为 Star Arm 102-FL 产品/模型范围：J1 ±110°、
J2 0°～180°、J3 按 URDF 轴约定为 -270°～0°、J4 ±90°、J5 ±65°、J6 ±150°，主动夹爪
`joint7_left` 为 0°～90°；`joint7_right` 继续作为 `multiplier=-1` 的 mimic 关节。

网页用这些资产显示仿真或真机后端快照。关节滑块只在当前浏览器调整模型；后端选择和
专用回零由页面的独立 HTTP 控制接口完成，资产本身不包含执行器逻辑。
