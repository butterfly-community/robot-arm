# 官方点云与附着物路径验证（2026-09-07）

## 范围与结论

用户要求先验证官方路径，再决定运输碰撞集成。验证使用当前带补丁的型号模型、正式模拟 RGB-D、
归档的实际软件反馈；只启动隔离的官方 MoveGroup 与 RobotStatePublisher，没有运行真机、
执行控制器或新的生产数据链路。**官方机制可用，但尚不能宣称生产运输碰撞已经解决。**

原型生产改动已撤回；生产服务没有重启或重新部署。motion 的本地 latest 镜像已从恢复后的源码
重新构建，避免后续重启误用之前的原型；基础镜像未重建。新增文件只属于离线诊断工具及文档。

## 核对的官方行为

- [x] `sensor_msgs/PointCloud2 → occupancy_map_monitor/PointCloudOctomapUpdater → PlanningScene`。
- [x] 相机光学坐标点云保留正确 frame、时间戳和相机原点；TF 到 `world`，没有把相机伪装在 base 原点。
- [x] 世界目标作为独立 `CollisionObject` 时，官方 updater 排除目标占用，不需要自己删目标点。
- [x] 标准 `AttachedCollisionObject` 从世界对象转为 TCP 附着物；随机械臂移动；释放后回到世界对象。
- [x] 反复送旧观测会重新占用原位置；真实更新后的观测由官方射线更新清除旧占用。
- [x] 官方 `GetStateValidity` 会报告附着物及夹爪与 Octomap 的碰撞。
- [x] RViz/noVNC 显示原始点云、新观测、官方地图、独立机械臂参考模型，已截图检查点云可见。
- [x] 默认资产生成深度与现存正式 PNG 逐像素相同；新增移动物体渲染仅为离线测试，深度确实变化。
- [x] 方法级复查后再次独立运行，5 mm 两轮的九个原生地图快照逐字节一致。

对应官方资料：[感知流水线](https://moveit.picknik.ai/main/doc/examples/perception_pipeline/perception_pipeline_tutorial.html)、
[MTC 抓放教程](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)、
[PlanningSceneMonitor 源码](https://github.com/moveit/moveit2/blob/2.15.0/moveit_ros/planning/planning_scene_monitor/src/planning_scene_monitor.cpp)。
此次运行插件版本为 ROS Lyrical 的 MoveIt 2.15.0。MTC 的规划场景阶段与运行时场景监视器是不同
上下文，不能认为尚未执行的 MTC attach 阶段已经让实时相机过滤器移动了目标。

## 完整输入与实测

每帧 1920×1080，2,073,600 个有效点，XYZ float32 共 24,883,200 字节；没有手动删除地面、
下采样或改物体尺寸。测试 `point_subsample=1`、过滤 padding offset=0/scale=1。
目标碰撞体来自同次识别 OBB，约 37.77×37.74×30.97 mm，不拿 30 mm 真值盒替代感知输出。
筐只在观测中，没有向 MoveIt 创建实心筐盒；真值尺寸仅用于离线统计。

| 核对项目 | 5 mm 地图 | 2 mm 地图 |
| --- | ---: | ---: |
| 初始官方占用叶节点 | 73,019 | 426,109 |
| 初始原方块区域占用叶节点 | 138 | 316 |
| 建立独立目标后该区域占用 | 0 | 15 |
| 附着后原地观察，该区域占用 | 0 | 15 |
| 目标移动后重复旧观测 5 次 | 138 | 316 |
| 改为新观测 5 次 | 138 | 316 |
| 改为新观测 15 次 | 0 | 本轮只测到 5 次及随后一次释放观测 |
| 筐内部检查区域占用 | 0 | 0 |
| 筐沿检查区域占用 | 889 | 1,476 |
| 完整点云单次更新耗时（含诊断传输） | 约 1.5～2.6 s | 约 23～38 s |

这些是指定 ROI 内的叶节点数，不是体积或完整拓扑证明。地图分辨率不是允许穿透值，也不等于
相机的 1 mm 光轴深度量化步长；本轮没有改生产分辨率、碰撞门限或地面高度。
在同一静态测试观测上重复多次只是验证概率更新，不表示相机产生了独立噪声样本。

**旧占用不会一帧消失。** 本次累计旧观测后，新观测第 5 帧仍保留原位置占用，到第 15 帧已清除。
这证明最终清除，不断言恰好必须 15 帧，也不保证真机遮挡情况下必然在该帧数内清除。

## 发现的问题，不能掩盖为已通过

1. **传输配置**：默认 Fast DDS 下 6,120 字节的尺寸隔离探针能进入回调，而完整帧不能。
   单独增大 Docker SHM 总量不够；显式配置 128 MiB SHM segment 后完整帧稳定进入。
   官方 [SHM 文档](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/transport/shared_memory/shared_memory.html)
   明确提醒 segment 小于消息会丢数据。GUI 额外 participant 需要更大总 SHM，本次用 1 GiB。
   尝试同时增大 TCP/UDP socket buffer 曾被主机限制拒绝；最终配置只调整隔离实验实际使用的 SHM，
   未修改主机 sysctl 或生产 DDS 配置。
2. **诊断错误**：最初漏了 `world → base_link` 固定 TF；另一次将 GetPlanningScene 的 full Octomap
   当成 binary 读取，诊断解码器崩溃。已分别补齐 TF、按 `binary` 字段选择原生读法，重新完整测试。
   失败日志不充当成功证据；工具使用 `--ulimit core=0`，避免崩溃产物散落。
3. **目标边缘**：2 mm 图过滤后仍有 15 个占用叶节点，中心为 X=203 mm、Z=17 mm，沿方块侧面分布。
   同次 OBB 的 X 上界约 202.5845 mm。需要继续检查感知包围体与量化点/体素边界的覆盖关系，
   不能以 5 mm 图恰好清干净就宣称所有分辨率都成立，也不能擅自扩大碰撞体或删点。
4. **地面与底座**：2 mm 图报告 `base_link` 接触，所示穿透深度仅约 `4.55e-11 m`，接触 Z 接近 0。
   这是本次浮点/接触边界问题的证据，不是机械臂真实扎入地面若干毫米；需单独收敛。
5. **运输没有通过**：对原记录中抓取到释放之间的 74 个离散姿态作官方查询，5 mm 图 33 个无效，
   2 mm 图 74 个无效（包含上述底座接触，不能都解释为撞筐）。官方返回的筐附近接触分别为
   10 / 6 条，涉及夹爪/附着物，所报最大深度约 2.249 / 2.641 mm。
   GetStateValidity 返回的接触不是穷举，采样也不是连续扫掠；本轮没有重新规划，更没有完成无碰撞执行。

## 新观测是不是必须

**单次完整抓放规划不强制需要抓起后的新点云。** 可以在规划前建立一份一致场景快照，目标作为
独立世界对象，MTC 用 attach/detach 表达抓取、运输、释放。附着物体积参与碰撞检查。
不能持续重复发送旧观测，再期待它自己理解“物体已经抓走”。

如果以后启用持续环境更新，真机由相机产生新的 RGB-D；模拟源也应按相同消息契约提供观测。
不需要针对真机编写“删除方块”分支。本轮移动物体重渲染用于验证官方更新机制，**不是给每次抓放
增加强制刷新、等待新帧或生成动态图片的步骤**。

下一步先讨论并收敛单次快照、目标体积、边缘残留和地面接触，再做正式 MTC 运输规划复测。
不用最高评分候选作为验收条件，按任一候选可达并完整无碰撞执行的标准验收。

## 复现与人工检查

命令、文件职责、GUI 开关及产物说明见 [工具 README](../../../tools/diagnostics/octomap/README.md)。
原始反馈输入来自 `tools/diagnostics/results/grasp-acceptance-20260907.tar.gz`。
本轮原生地图/接触响应/报告与日志保存在 `tools/diagnostics/results/official-octomap-20260907.tar.gz`，
派生图片和 XYZ 可从正式资产及工具重建，不放入 docs/assets。

当前隔离 GUI：`http://192.168.100.10:6081/vnc.html?autoconnect=1&resize=scale`。
默认原始点云按高度着色，并非 RGB；机械臂只是另一个可选显示，不在相机点云中。
GUI 维持人工检查，生产网页/运动服务保持原状态。
