# MoveIt 时间参数化与控制器改动复核

## 最新决定：恢复执行基线

用户授权返回基线并重新验收连续四次。生产 Dockerfile 已移除 JTC overlay 与 TOTG
补丁构建，controller 配置恢复 position 状态/命令；真实编码器反馈与动态力控保留。
下面描述的是隔离对照实验，不是当前生产修复方案。两份补丁移到
`tools/diagnostics/mtc-ranking-test/archived-patches/`，不进入生产构建依赖。
恢复后的镜像仍须整套部署与实物验收，不能沿用历史四次结果宣布本轮完成。

## 结论与部署状态（2026-09-10）

用户要求先证明必要性，不得因抓放失败随意修改经过验证的底层库。
本次不重启服务、不执行真机动作、不部署新 TOTG 镜像。
确认了具体版本的数值缺陷，但不能将其扩大成“MoveIt 规划有问题”，也不能将
数值回归通过当作六次真实抓放通过。当前连续实物验收仍为 0/6。

**必须区分两个问题：**

- 原版 MoveIt TOTG 的速度查询时刻存在偏差，有脱离本项目的最小复现。
- 本次接近后等待不结束，还依赖本项目最近修改的 JTC 完成判定；不能归咎于原版 MoveIt 单独失效。

一行采样时间修正有数学依据；它是否是本项目最合适的集成修复，仍须连同 JTC
改动一起评估。没有证明所有抓取失败均来自该偏差，也没有证明只能修改底层库。

## 同一个程序、原版与修正版共享库对照

官方二进制：`ros-lyrical-moveit-core 2.15.0-1resolute.20260814.181429`。
同版本源码：`0b5a5420630ddce212b69c5b5ddef3928783ce26`。
以 `ldd` 确认分别加载 `/opt/ros/lyrical/lib/libmoveit_trajectory_processing.so.2.15.0`
和 `/opt/moveit_ws/install/moveit_core/lib/libmoveit_trajectory_processing.so.2.15.0`。
同一可执行文件，只改变动态库搜索顺序；没有相机、机械臂、规划节点或控制请求。

测试的单关节路径为 0→1，速度上限 1，加速度上限 30；这些是测试输入，不修改设备配置。

| 查询时刻（秒） | 原版速度 | 位置数值导数 | 修正后速度 |
| --- | ---: | ---: | ---: |
| 0 | 0.03 | 起点静止 | 0 |
| 0.00025 | 0.03 | 0.0075 | 0.0075 |
| 0.0005 | 0.03 | 0.015 | 0.015 |
| 0.012345 | 0.39 | 0.37035 | 0.37035 |
| 0.12 | 1 | 1 | 1 |

再通过官方公共 `TimeOptimalTrajectoryGeneration::computeTimeStamps()` 和
`RobotTrajectory::reverse()` 测试，而非只调用内部采样器。加速度上限依次为
0.1、1、5、30 时，原版起点速度分别为 0.0001、0.001、0.005、0.03；
反转后末点取其相反数。修正版四种输入均为零。原版四种输入的正向末点均接近零。
四项原生回归通过；扩展公共入口测试也通过。

源码根因：先以积分段长度求加速度，随后速度查询仍使用整段长度，而不是请求时刻
距段起点的时长。补丁仅补回 `time_step = time - previous->time_`。
位置函数本来已有此操作。未改路径点、IK、碰撞、候选、控制器容差或硬清零路点。

官方 [CartesianPath](https://github.com/moveit/moveit_task_constructor/blob/ros2/core/src/solvers/cartesian_path.cpp)
执行时间参数化；[MoveRelative](https://github.com/moveit/moveit_task_constructor/blob/ros2/core/src/stages/move_relative.cpp)
反向传播时反转轨迹。这是官方调用语义，不是本项目自行交换起终点。

## JTC 对照：卡住现象包含本项目修改的影响

同一个 `hand_completion_test --audit`，分别加载原版 6.9.0 与补丁库，位置命令模式，
相同输入及生成的默认容差：

| 期望速度 | 实际速度 | 原版完成判定 | 本项目补丁完成判定 |
| ---: | ---: | --- | --- |
| 0 | 0.1 | 通过 | 不通过 |
| 0 | 0 | 通过 | 通过 |
| 0.03 | 0 | 通过 | 不通过 |

原版 position 命令模式未计算该速度误差；本项目补丁让它参与原有完成检查。
因此补丁既能阻止仍在运动时完成，也会让错误的非零末点速度在实际静止时不通过。
原生 `goal_time=0` 表示可能无限等待；这与当前等待现象一致。
参见 [JTC 参数说明](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html)。

保留真实反馈并不等于修改原生控制器完成语义必然是最佳方案。部署决策需要评估这条
完整因果链，不能仅因第二个补丁让第一个补丁的回归通过就宣布业务问题解决。

## 可复现入口

代码仅在 `tools/diagnostics/mtc-ranking-test/`：
`trajectory_sampling_test.cpp`、`hand_completion_test.cpp`；构建见同目录 README。
在运动构建镜像中，加载 ROS 和相应 overlay 后运行：

```bash
cmake --build temp/mtc-totg-test --target trajectory_sampling_test hand_completion_test
ldd temp/mtc-totg-test/trajectory_sampling_test
temp/mtc-totg-test/trajectory_sampling_test
temp/mtc-totg-test/hand_completion_test --audit
# 原版对照必须确认 ldd 加载 /opt/ros/lyrical/lib 下的库：
LD_LIBRARY_PATH=/opt/ros/lyrical/lib:$LD_LIBRARY_PATH ldd temp/mtc-totg-test/trajectory_sampling_test
LD_LIBRARY_PATH=/opt/ros/lyrical/lib:$LD_LIBRARY_PATH temp/mtc-totg-test/trajectory_sampling_test
LD_LIBRARY_PATH=/opt/ros/lyrical/lib:$LD_LIBRARY_PATH temp/mtc-totg-test/hand_completion_test --audit
```

原版采样测试返回 1 是预期复现结果，不是环境失败。测试不创建 ROS 节点；无节点
日志上下文警告与无几何测试模型警告不参与数值断言。测试产物均可重新生成。
