# 架构与调用链

单一业务链路：相机 → 场景/模型 → 绑定观测的抓放请求 → MoveIt/MTC → 执行器。
手动与 AI 共用这条链路；模拟只替换输入适配器，不替换运动学或执行契约。
部署见[Docker](DOCKER.md)，硬件/角度与精度边界见[型号说明](STARARM-102.md)。

## 离散控制与请求结果

网页和 AI 共用 HTTP → 网关 → 原节点；Redis 保存请求记录与 AI 会话元数据，不承载图像或点云。
`forward_value` 在下发前用 Redis `SET NX` 登记 `request_id`，重复编号返回 409，不重放动作；
无法登记时返回 503，尚未下发。`RequestHistory::observe` 将节点反馈送入异步写入队列，
`update_record` 区分受理、规划、执行及终态；后续反馈不覆盖已确认终态。
`GET /api/requests/{request_id}` 查询独立记录，缺失返回 404，存储不可用返回 503。
重启前未完成的记录显示 `unknown`，不能据此判断物理动作已停止或已经成功；系统不会自动重放。
运动/抓放页“请求结果”可以按编号查询，刷新只恢复查询。执行成功不是视觉确认抓持成功。

`MotionRequest.tcp_target` 与 `joints` 二选一；省略时保留关节目标接口。目标为米和
`orientation_xyzw` 四元数，`relative=false` 表示底座系绝对位姿。
相对底座目标：`p'=p+dp, R'=dR·R`；相对工具目标：`p'=p+R·dp, R'=R·dR`。
`resolve_tcp_target` 使用任务开始时的实际电机反馈 FK，随后走官方 `compute_ik`、MoveGroup 和控制器；
浏览器只预览目标坐标轴，不求 IK、不表示可达。未指定 `actuators` 不附带夹爪角度命令。

`run_actuator` 将独立开合放入同一 ROS worker，等待 `hand_controller/follow_joint_trajectory`
终态；受理时为 `executing`，不再把发布命令当成功。保持力仍由执行层持续调节，
目标力度、实际负载、采样时间与调节状态独立展示；控制器完成不证明夹持成功。

`/api/motion/cancel` 可携带 `target_request_id`，对应取消排队或活动任务；省略时仅取消普通运动。
规划取消使用 `Task::preempt()`；执行直接使用官方 `execute_task_solution` Action，
由本节点已有 executor 处理。取消回执之后仍等待原 Action 终态，不能以取消受理确认停止。
不调用安装版 `Task::execute()` 的便利包装：它的取消处理会提前返回，真机取消还复现了 executor 重复注册异常。
取消不自动张开夹爪或回工作位；若任务已附着物体，保留持物碰撞几何，不假定已经释放。
正在进行的单个求解/服务调用可能先返回，取消受理不是瞬时停止。若执行已先成功，保留成功结果。

观测沿用已有元数据：RGB-D 的来源/序号/采集时间、场景绑定的观测、实际关节的
`sequence/sample_time_ns/feedback_source`、TCP 内嵌的同次 FK 反馈和力度采样时间。
请求记录时间是状态写入时间，不是相机曝光或电机采样时间。读取图像和查询结果不触发推理或动作。

## `controller-input-node`

源码：[输入节点](../backend/nodes/controller-input/src/main.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `ControllerInput::load` / `commit_config` | 设备绑定配置 → 内存状态；保存先落盘 |
| `drain` / `tick` | 驱动事件 → 统一位姿与 Action |
| `combined_pose_frame` / `evaluate_actions` | 多设备组件 → 组合位姿、连续轴及按钮动作 |
| `apply_user_config` | 用户编辑 → 持久配置；不把临时演示绑定当用户配置 |

设备差异只在驱动适配层处理，配置与输入绑定由节点持有。

## `spatial-transform-node`

源码：[空间核心](../backend/crates/spatial-core/src/lib.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `update_pose` | 输入源绝对位姿 → 最新空间状态 |
| `handle_control` | Action → 接管/积分状态 |
| `current_output` | 接管原点与当前输入 → 统一相对控制帧；不调用 ROS 或硬件 |

统一处理跟踪坐标、标定与控制绑定；不在执行节点重复 TCP 换算。

## `realsense-camera`


SDK pipeline/Align 由采集 worker 拥有。采集频率与上送频率独立，只对待发布帧对齐；视频仍按采集频率输出。

## `camera-node`

源码：[节点与标定状态机](../backend/nodes/camera/src/main.rs)、[采集 worker](../backend/nodes/camera/src/capture_worker.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `capture_worker::spawn` | 命令/最新值通道 → 驱动长期任务；事件循环不执行 SDK 阻塞操作 |
| `refresh` / `select` | 发现/用户选择 → 来源和 profile；不按设备枚举索引存配置 |
| `start` / `stop_capture` / `reset` | 明确请求 → 同一 worker 的采集状态 |
| `tick` / `finish_capture_command` | worker 结果 → 状态、RGB-D 原子 bundle |
| `start_automatic_calibration` / `advance_automatic_calibration` | 型号姿态与实际反馈 → 采样、求解、回工作位 |
| `poll_calibration_work` / `apply_solved_calibration` | 当前会话结果 → 待确认状态 / 已落盘外参 |

来源启动时发现但不自动选择。profile 与已应用外参持久化，设备离线不丢配置。
采集、SDK Align、标定使用 Tokio 阻塞任务，Dora 循环收取结果。
标定流程为工作位 → 九姿态采图/新电机反馈 FK → 求解 → 回工作位 → 确认应用；
取消或失败不覆盖已有外参。标定、图像、相机参数均由本节点管理。

## `robot-arm-messages`

源码：[消息与 Arrow codec](../backend/crates/robot-arm-messages/src/lib.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `to_arrow` / `from_arrow` | 小消息 ↔ JSON Arrow |
| `camera_frame_from_arrow` | RGB-D bundle → 已校验的图像平面与同帧元数据 |
| `world_scene_to_arrow` | 场景 → 小元数据与二进制 XYZ；不把点云展开为网页 JSON |
| `scene_pick_place_to_arrow` | 请求 + 场景快照 → 单条绑定消息；没有第二个场景订阅竞态 |

CameraFrameBundle 原子携带同帧已对齐 RGB-D、内参、深度比例和外参快照。
WorldScene 的大体积 XYZ 使用 Arrow 二进制列，JSON 只含元数据。

## `scene-node`

源码：[阶段编排](../backend/nodes/scene/src/main.rs)、[组合分割](../backend/nodes/scene/src/segmentation.rs)、[三维重建](../backend/crates/scene-core/src/lib.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `apply_request` / `start_scene_task` | 显式网页请求 → 一个异步阶段；不自动串联全部模型 |
| `next_observation_frame` / `feedback_after_frame` | 新帧与实际反馈 → 同次输入及末端自过滤依据 |
| `segment_frame` / `segment` | 冻结 RGB、模型/提示 → 二维实例；不定位、不抓取 |
| `edit_segmented` / `rebuild_segmented` | 模型层与手动层 → 原图、框信息、mask；不生成染色图 |
| `reconstruct_frame` | 冻结帧深度 + mask + 外参 → 场景与实例点云；不重跑分割 |
| `attach_grasp_candidates` | 指定实例点云 + 环境 + 实际夹爪 → 该实例候选 |
| `finish_scene_task` / `publish_scene` | 同序号阶段结果 → 状态、响应及场景；旧输入结果不覆盖新输入 |
| `clear_scene` / `clear_output` | 标注变化 / 来源配置变化 → 对应下游结果失效 |
| `snapshot_previews` / `send_asset` | 明确刷新 / 资源查询 → 预览 PNG / 已缓存二进制；不触发模型 |

分割、三维定位、抓取候选独立按请求触发。新帧/配置变化使旧结果失效，
每个请求绑定输入序号；失败不伪造结果，也不暗中重跑上游。
自动、提示词和手动分割可组合，标注保留具体模型 ID 或 manual 来源；
统一结果显示冻结原图与框，点击标题看详情，手动编辑只显示手动标注。
手动矩形经同一深度重建链路定位，不建立专用抓取路径。
目标深度采用 Otsu 相邻差和四连通，阈值下限取一个深度量化刻度。

## `perception-compute`

源码：[模型边界](../backend/services/perception-compute/src/perception_compute/app.py)、[自过滤](../backend/services/perception-compute/src/perception_compute/gripper_self_filter.py)、[碰撞筛选](../backend/services/perception-compute/src/perception_compute/scene_collision.py)、[候选代表](../backend/services/perception-compute/src/perception_compute/grasp_selection.py)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `create_app` / `default_backends` / `default_grasp_backend` | lifespan → 常驻模型；请求不重复初始化 |
| `segment` / `_segment` | PIL RGB、模型与提示 → 官方分割结果；实例锁隔离提示更新 |
| `GraspGenXBackend.infer` | 目标/环境 XYZ、夹爪资产与观测 → 原始评分候选 |
| `GripperSelfFilter.filter` | 实际 TCP/关节、环境点 → 去掉已观测夹爪自体点 |
| `filter_colliding_grasps` | 官方取样几何与场景 → 精确最近距离碰撞筛选 |
| `representative_grasps` | 筛选后的姿态、模型分与 CAD 角点 → 最多 100 个原始代表 |
| `build-description.py` | 正式型号 URDF/清单 → 构建期夹爪资产；脚本由计算服务维护，不依赖 tools |

FastAPI lifespan 常驻加载 YOLOE 与 GraspGenX，模型实例串行访问。
/v1/segment 按模型能力使用文字、视觉参考或 prompt-free；没有应用级 0.18 阈值。
视觉提示属于保存的参考图，新帧掩码仍由模型推理，不直接拿示例框充当分割。
/v1/grasps 使用同帧目标/非目标环境点云和夹爪资产；先生成、按环境距离筛选、
再按 CAD 角点位移聚类，返回原始代表位姿与分数。源模型不读取真实 RGB 颜色通道，
RGB 用于上游分割。官方接口细节见[GraspGenX 参数](../tools/graspgenx/PARAMETERS.md)。
计算服务拥有模型与构建期夹爪资产，不直接访问相机、ROS 或执行器。

## `stararm-102-motion-node`

源码：[队列与状态](../backend/devices/stararm-102/nodes/motion/src/main.rs)、[ROS 桥](../backend/devices/stararm-102/nodes/motion/src/ros.rs)、[Action 调度](../backend/devices/stararm-102/ros2/mtc/src/pick_place_server.cpp)、[任务工厂](../backend/devices/stararm-102/ros2/mtc/src/task_factory.hpp)、[场景管理](../backend/devices/stararm-102/ros2/mtc/src/task_scene.hpp)、[候选与排名](../backend/devices/stararm-102/ros2/mtc/src/grasp_candidates.hpp)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `planning_state` | 实际反馈 → 控制器同步用状态；不修改用于标定 FK 的原始反馈 |
| `RosInterface::request_current_pose` | 带来源/时间的实际反馈 → 官方 FK 与同一输入快照 |
| `publish_pose` | 相对控制目标 → 带当前 ROS 时间戳的 Servo 消息 |
| `run_motion` / `run_manipulation` / `work_loop` | 已接收任务 → 唯一顺序 ROS worker |
| `plan_and_execute` | 普通关节/TCP 目标 → MoveGroup 规划 → 原生控制器执行 |
| `create_task` / `TaskScene::apply` | 绑定场景 → 本次 MTC 阶段与官方 Octomap 快照 |
| `allowed_grasp_depth_poses` / `rank_complete_grasps` | 候选 → 指尖过滤、深度变体、完整解排名；不执行失败 IK |
| MTC `execute` / `TaskScene::cleanup` | 首条完整解 → 同场景单候选精修 → 一次执行 → 清理 |

普通运动走 MoveGroup；抓放由 scene 绑定 WorldScene/点云为 ScenePickPlaceRequest 后排队。
只有一个顺序 ROS worker，离散任务期间暂停 Servo，结束恢复；相对控制仅在队列空闲时发送。
Servo 位姿带当前 ROS 时间戳。选定真机端点但断连时明确报错，断连错误不被后续成功覆盖。
取消使用控制器 goal handle，收到真实 ROS 终态后才发布 cancelled。
模型和规划器资源启动时初始化一次；每次只构造 MTC 阶段与任务场景。

## `stararm-102-execution-node`

源码：[执行与串口调度](../backend/devices/stararm-102/nodes/execution/src/main.rs)、[夹持反馈控制](../backend/devices/stararm-102/nodes/execution/src/gripper_feedback.rs)、[UART 协议库](../backend/crates/fashionstar-uart/src/lib.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `configure_endpoint` | 用户选择 → 保存并连接/断开；有端点但断连不回退软件反馈 |
| `encode_command` / `StarArmBus::write` | 完整目标 → 型号总线编码及唯一待发目标 |
| `flush_motion` | 最新目标 → 串口同步写；不能插入未完成回包事务 |
| `poll_hardware` | Monitor / 参数事务 → 实际角度、遥测和连接状态 |
| `GripperFeedbackController::request` / `observe` | 明确夹持意图 + 新负载反馈 → 持续功率调节；不锁存夹持角 |
| `primary_tool_feedback` | 设备遥测 → 统一负载百分比，不是牛顿力 |

同一型号驱动处理真实串口和软件反馈。反馈保留原始电机角，不以命令目标冒充观测。
UART 查询/回包与写入串行，待发送目标只保留最新值；断连清空待发命令，重连不重放。
夹持闭环在运输中持续调节，显式张开才退出；参数语义见[型号说明](STARARM-102.md)。

## `web-gateway-node`

源码：[网关](../backend/nodes/web-gateway/src/main.rs)、[就绪状态聚合](../backend/nodes/service-status/src/main.rs)。

| 方法 | 输入 → 输出 / 副作用 |
| --- | --- |
| `validate_request` / `forward_value` | HTTP 请求 → 校验、request ID 配对与唯一节点输出 |
| `request_pick_place` | 抓放提交 → 场景所有者；HTTP 202 只表示已受理 |
| `AppState::update` / `snapshot` / `websocket` | 节点小状态 → 页面快照与实时流 |
| `perception_asset` / `model_asset` / `binary_response` | 按需资源 → 所属节点 → 二进制 HTTP |
| service-status `evaluate` | 服务报告及依赖表 → 就绪状态；不控制节点重启 |

网关转发契约消息、RequestResult 和实时状态，不保存第二份业务配置。
HTTP 接收成功不是动作完成；网页按同一 request ID 跟踪准备、规划、执行和终态。

## 抓放参数

源码默认与现场持久配置分开：.env 管理计算采样，网页管理提示、环境距离与夹持目标。
以下保留会影响抓法的参数及来源，现场值不在文档另存副本。

| 所在方法 / 参数 | 当前值、来源与实际影响 |
| --- | --- |
| YOLOE `predict` | 部署 Ultralytics 8.4.135；文字/自动模式沿用库输入 640、NMS IoU 0.7、最多 300 检测；应用指定 `retina_masks=True`。1920 原图不等于模型以 1920 推理；视觉提示另取原图/参考图最长边。没有应用级 0.18 置信度覆盖。 |
| `scene-core` 深度区域 | Otsu 相邻深度差、四连通；下限一个深度编码量化刻度，不是额外物理厚度门槛。包围盒来自观测，不是实体表面真值。 |
| GraspMoE 探索 | 源码默认 200，由 `GRASPGENX_NUM_GRASPS` 配置；种子仅初始化时设置。种子不是每请求重置，500 只控制 diffusion 分支，不是最终总数。 |
| OBB 提议 | 官方场景 `dense-topandside`、外法线偏移 -2/0 cm；不追加正向外移档位。采样参考点、真实 TCP 与资产几何不随此修正改变。 |
| OBB 库内默认 | 未覆盖的 GraspMoE 参数为 36 个绕轴角、密集位置步距 1 cm、邻域 20 点/离群距离 14 mm、`advanced` OBB 与 `auto` 跳过规则；这些来自上游，不是应用新增的接近角度/深度门限，但仍影响提议生成。 |
| 夹爪资产 | 本型号清单的张开/中间扫掠盒为 129×15×20 / 71×15×20 mm，中心 Z=60/80 mm；属于模型条件描述，不是关节行程限制。采样深度 66.471 mm 来自张开活动网格前沿，真实 TCP Z=93.38 mm，二者不能互相替换。资产 `standoff=[0,10] mm` 用于上游抓取体积定义，不是控制器再后退 10 mm。 |
| 模型评分 | `grasp_threshold=-1`、`topk=-1` 关闭示例 0.7 截断；保留模型分，不保证低分可抓。 |
| 环境预筛选 | 最多随机 8192 环境点与张开网格 2000 表面点，来自官方示例；是点距离筛选，不是完整连续网格碰撞保证。 |
| 环境距离 | 官方/源码默认 20 mm，实际值由网页 `grasp_collision_distance_m` 持久配置决定；是拒绝近邻候选的距离，不是允许穿透地面 5 mm。 |
| 机器人自过滤 | `self-filter.json` **40 mm** 网格表面包络，模型和 ROS 共用；会同时删除包络内的近邻外物点。它不是 TCP 偏移，也不是环境碰撞膨胀。 |
| 返回候选 | 用户指定最多 100；CAD 角点距离聚类、每组保留原始最高分，不重新生成姿态。 |
| 接近 `MoveRelative` | 不按 `object_size.z` 截断。原生阶段从抓取终点反向求预抓取位置，碰撞/IK 决定实际退离距离。有限搜索线段覆盖模型完整包络直径 `2R`，由三角不等式保证不截掉可达线段，不是新固定距离。它不决定最终夹入深度。 |
| 抓取 `ComputeIK` | 不做应用级 8 解截断，使用接口整数容量上界，仍受原生时间预算控制；设 0 会完全不搜索。部署厂商组默认超时 **0.005 s**，不是整任务 5 s。 |
| 真正加深搜索 | 沿候选 TCP +Z，0 到观测目标在该方向的远端面；步距来自 Octomap 5 mm，包含端点。此处仍有物体相关的几何搜索范围：防止把整个抓取参考点推进物体背后，不是接近段上限。所有变体须完成碰撞/IK/闭合/运输规划。 |
| 平面起点 | 单候选精修另试 XY 对准观测中心，原高度和朝向不变；原始起点保留。模型分未对移动后的提议重算，不能称作模型原始输出。 |
| 指尖过滤 | 用户指定两指尖连线离水平角小于 20°；不是夹爪本体倾角，也不是强制顶部抓取。 |
| 搜索与排名 | 初步 `plan(1)` 首条完整方案；选中候选有限网格 `plan(0)` 精修。成本为 `(1-模型分)+(投影夹持跨度+夹取偏差)/对象对角线`，两项系数均为 1；同质量才比较指尖等高、归一化关节行程。不是全候选全局最优证明。 |
| ROS 场景 | Octomap 5 mm、普通形状 padding 半体素 2.5 mm；目标清点包络每面加一个体素后恢复实际观测尺寸。地面顶面 z=0、水平 2×2 m、厚 1 m；没有固定允许地下穿透量。 |
| 允许接触 | 夹指/目标由正式 ACM 放行；持物脱离允许初始已测支撑接触但不加深，回工作位后恢复检查。不是删除整个地图。 |
| 放置 | 物体中心释放 Z 为 `max(目标 Z, 0.05 m)`，相对场景 Z=0 的下限为 5 cm；不是目标上方偏移或物体底部净空。XY 为目标中心，位置容差继承库 0.1 mm，姿态自由。 |
| 运动 | 速度缩放 0.125、闭合 0.0625，来自用户多次降速；不另设加速度缩放，继承规划器默认。型号最高关节速度仍是用户指定 34 rpm。 |
| 保持力度 | 目标由执行页持久配置决定，是监测功率换算负载刻度，不是牛顿。400 mW 零负载、2000 mW 满刻度/指令上界；30 对应 880 mW 只是调节目标/初始估计，实际输出随新反馈动态积分，不锁角、不固定 880 mW。 |
| 执行反馈 | 默认回读 100 ms；调节平滑复用原有 100 ms 舵机命令时域；1 mW 下界避开厂商 0=最大功率的特殊编码。软件完成仅表示轨迹终态，仍需图像/人工核对物体搬运。 |


## MTC 模块边界

| 模块 | 责任 |
| --- | --- |
| `pick_place_server.cpp` | Action、进度、搜索/精修、一次完整执行与清理 |
| `task_factory.hpp` | 用原生 stage 构造完整抓放任务 |
| `task_scene.hpp` | 点云发布/对应确认、目标过滤与恢复、任务地图清理 |
| `grasp_candidates.hpp` | 几何变体、缓存排序、解属性和完整方案排名 |
| `planner_adapters.hpp` | 原生输出轨迹检查、支撑脱离和自由朝向位置目标 |
| `planning_resources.hpp` | 模型/插件/规划器初始化与复用 |

正常阶段：张开 → 接近 → 闭合并附着 → 持物回工作位 → 运输释放 → 分离 → 回工作位并闭合。
初步 plan(1) 获取首条完整方案，再以同一起点/地图穷尽所选候选的深度与平面起点变体；
没有更优完整解就保留原完整方案。只有一次实际执行，不在中途二次观测或更换目标。
同候选/起点取最深完整解，同深度选较短行程；跨候选比较软质量成本，同质量再比较指尖等高和行程。
这不是全候选全局最优，也不证明物理夹持成功。

夹取偏差是观测中心在最终 TCP 坐标中的 `sqrt(x²+y²+max(z,0)²)`，
不会惩罚已经进入指尖后方的深度；投影跨度取目标 OBB 沿闭合轴的宽度，均不是实际接触真值。
场景只建一次 Octomap 快照，目标为独立可附着体，其他物体不改成实心盒。
机器人网格表面包络与目标相邻体素由形状过滤处理，规划前恢复目标真实观测尺寸。
持物脱离只允许原有支撑接触且不加深，离开后恢复普通碰撞；夹指/目标接触由 ACM 单独允许。
释放约束作用于附着物中心，不是 TCP；TRAC-IK Distance 求自由朝向的位置解，随后原生规划器检查完整路径。

## RGB、图像与视频

全链路使用 RGB；Z16 深度不做颜色转换。设备边界按真实格式/stride 解码，SDK 负责彩深对齐。
OpenCV 灰度计算使用 RGB2GRAY；仅在需要 BGR 的编码/解码边界转换，检测原图不预模糊。
浏览器使用 @thi.ng/pixel 将 RGB888 转 Canvas RGBA。实时彩色视频与低频上送 RGB-D 独立节流，
同一 camera 节点发布，不增加视频节点；网关转发，浏览器浮窗可拖动和收起。

## 网页与配置归属

五个 Next.js 应用共用 UI/契约。关节滑块编辑草稿，点击执行才发运动；预估姿态不是已通过规划的承诺。
AI 栏目包含自动/提示词/手动分割和可折叠抓放场景。分割冻结帧与普通视频/深度预览互不覆盖。
已应用标定回显数值并锁定普通开始按钮，用“重新标定”开启新会话。
配置由节点持久化到 backend/config/runtime，网页保存失败保留草稿；temp 不存正式配置。
输入绑定归 controller-input，相机与标定归 camera，模型提示归 scene，串口/反馈/夹持归 execution。

通用 AI 在 Next.js 服务端，使用 AI SDK 的 OpenAI Responses provider。模型可看图、回答或组合
已有工具，不固定为抓放流程；抓放仍复用手动按钮的 startPickPlace，不复制感知/运动实现。
地址、模型、reasoning effort 在 AI 配置中保存到 Redis；.env 提供初始默认值和仅服务端可见的密钥。
本地分割、候选、规划和执行不依赖外部 AI 服务。

### AI 方法与状态

| 入口/方法 | 职责 |
| --- | --- |
| api/ai → createRun / executeRun | 持久登记运行、调用 Responses、SDK 工具循环；HTTP 返回不终止后台运行 |
| robotTools | 观察、分割、定位、关节/TCP、工作位、夹爪/力度、抓放、查询/取消；只封装已有业务 API |
| RobotToolsContext.command | 下发前记录运行/工具/动作编号；等待原请求终态。重复工具编号只查询，不重发 |
| api/ai/events | 向网页传输持久状态；断开只影响显示，不取消机器人动作 |
| stopRun | 停止后续 AI 调用，取消可取消的原动作并等终态；不自动松爪或回位 |
| store.messages / saveMessages | 保存完整 SDK 消息/工具/reasoning 项；图片外置到持久卷，不存 Base64 到 Redis |

Responses 使用 store:false，本地回传完整上下文，不依赖供应商会话保存。模型内部思维不展示；
网页显示公开文本、工具步骤、原任务状态、实际返回模型与用量。能力验证不会操作机械臂；
模型与思考强度使用可编辑下拉框；模型列表来自当前服务，思考强度提供常见档位并标注实测通过项，
也可手动填写。目录不是能力保证，不支持的档位保留原始错误，不自动替换。
正常任务可由 AI 直接读取当前相机；浏览器上传只用于补充参考图片，不是执行前提。
提交任务即授权任务范围内的已有工具，无额外确认步骤；漏检时可自主调整提示词或基于当前固定帧
框选，再继续三维定位及执行。框选仍标记为 manual，不冒充模型分割；只读、暂停等用户指令仍生效。
同一会话按时间连续显示用户消息和 AI 回复，模型参数及工具记录收在每轮详情中，不拆为独立任务卡片。

同一服务进程串行执行有依赖的工具。整套重启后未结束运行标为中断，不自动续跑物理动作。
AI 工作位按模型 named_targets 中的 work 生成普通运动请求，与网页工作位按钮相同；
prepare-relative 是准备相对控制的全零默认位，不是工作位，二者不能互换。
绝对 TCP 与增量 TCP 工具使用不同参数名，最终都映射到同一个 MotionRequest.tcp_target。
增量平移不是最终位置，保持朝向的增量四元数为 [0,0,0,1]，不是当前 TCP 的四元数。
思考强度同时记录请求值与服务端回显；接口可能接受无效字符串，故仅 HTTP 成功不能证明档位生效。
没有回显时明确标为未确认，不按耗时猜测，也不自动换档位。
场景查询默认只带相机状态摘要；明确查询硬件能力时才返回完整分辨率、FPS 和驱动参数目录，
避免每次抓放都重复向模型发送设备目录。完整场景点云和抓取数组不进入对话上下文，由现有服务处理。
固定帧采集结果同时返回实际可用分割模型和已保存提示词配置，模型无需猜测名称或重复查询目录。
分割/框选直接使用与网页相同的单次 refresh，不先 apply 清除标注帧；工具结果以原请求的终态值为准。
定位、候选请求结束后按返回的场景序号等待对应快照，避免把异步到达的旧空场景当作本次结果。
模型分割和框选由工具读取当前固定帧的结果序号，与网页提交方式一致；不让大模型填写内部版本号。
历史图片缺失会报错，不用新图冒充旧图。会话和图片默认保留供追问及结果核对；
图片为内容哈希共享引用，清理时必须同时考虑会话消息、运行记录和图片元数据，不可仅按文件时间删除。
“本轮回复已结束”仅表示模型本轮回复及工具循环结束；控制器成功、视觉确认和用户验收是不同证据，不能混称实际抓放成功。
