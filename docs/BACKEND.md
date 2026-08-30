# 后端方法与依赖

## controller-input-node

入口 `main()` 只连接 Dora 输入输出。`ControllerInput::load()` 通过公共 `json-config-store` 读取功能 Action、反馈绑定和设备自定义名称；`apply_config()` 只替换运行时状态，`commit_config()` 先写盘成功再替换内存配置。`ControllerInput::drain()` 合并驱动事件；`tick()` 只在收到新样本时生成统一位姿和 Action；`select_pose_source()` 按运行时能力选择空间位置或姿态来源且不持久化；`apply_bindings()` 校验并持久化每个 Action 的输入和反馈目标。测试播放和硬件驱动共用 `replace_driver_sources()`、`accept_sample()`、`combined_pose_frame()` 与 `evaluate_actions()`；模拟数据仍经过统一的来源选择、Action 绑定和消息生成链路，停止后只恢复启动前的内存配置。`combined_pose_frame()` 从两个来源合成位姿；`evaluate_actions()` 与 `binding_value()` 可同时读取多个设备，把按钮、连续轴或方向按钮对转换成功能 Action。能力筛选只约束绝对空间与姿态来源，没有绝对位姿的分量仍可由按键或轴 Action 驱动空间节点积分。

采集节点的统一目录包含十四个输入 Action：TCP 三轴平移与三轴定点旋转、两种枢轴圆弧、
两个夹爪动作、接管与急停、工具轴向平移和工具轴向螺旋。采集页按动作选择演示时仍调用
同一个 `/api/tracking/simulation`；生成源只声明这十四项 Action，不声明虚假的绝对位置或姿态。
十四个输入动作演示开始前都调用 `/api/motion/prepare-relative`：motion 节点依次进入手动模
式、经普通 MoveIt 链路执行包含夹爪闭合的默认位，然后进入相对模式。生成源建立控制基准后
先用同一 `move_up_down` Action 垂直上移 5 cm，再执行所选动作；垂直平移演示只执行这次上
移，不再叠加第二段上下运动，接管与急停也使用该前置阶段。力度反馈测试只使用现有 Action
反馈路由且不移动机械臂。每个连续演示独立完成去程、反向回程和控制结束，随后从同一清理
出口恢复演示前的内存配置；用户主动停止也走同一出口且不归位。
原始按钮和轴始终保留给输入测试。SDL3 已绑定连续轴若原始值绝对值不超过 0.1 且连续 3 秒完全不变，则在内存中用该值作为计算零偏；值变化会重新计时。连续轴按“原始值 → 内存零偏 → One Euro → Action”处理，按钮和方向按钮对不滤波，也不引入死区。功能 Action 绑定、反馈目标和设备自定义名称写入 `/config/controller-input.json`；SDL 持久化来源标识优先使用设备序列号，其次使用 Linux `by-id` / `by-path`，不保存重连时变化的 instance id。空间与姿态来源、动态零偏、滤波状态仍只存在内存，节点重启后重新建立。`live_component_values` 原样携带每个在线来源最近一帧的组件值，采集页用它显示当前按下的按钮或偏离零位的轴，便于确认物理控件名称；它不参与 Action 求值，也不形成测试旁路。

深度相机是同一采集节点的可选能力。`depth_camera_enabled` 与其他输入配置保存在同一个
`controller-input.json`；旧的 schema 2 文件加载后一次性写成 schema 3，默认关闭。状态分别
报告用户开关、驱动发现和实际采集，不把它们压成一个连接标志，也不参与系统 readiness。
未启用或没有驱动事件时不发布空点云、假内参或周期占位消息。采集页通过现有 tracking 请求
链路修改开关，并只接收小型状态摘要；点云不会经过 Web Gateway。

当前尚未接入真实深度相机驱动。确定性“深度测试场景”复用采集节点的 simulation 驱动边
界，以 10 Hz 连续生成平面与障碍物，并生成一次相机内参与 `base_link -> depth_sim_frame`
标定变换；这个频率只描述测试源，不限制真实驱动。它与未来硬
件适配器输出完全相同的设备无关契约。XYZ 数据以米为单位，使用 Arrow 原生的
`LargeList<FixedSizeList<Float32, 3>>`，避免把高带宽点云展开为 JSON 或手写字节协议；内参与
外参是独立的低频消息，并与点云一样携带 source、sequence 和原始采样时间。测试源显式开始和
停止，不用收帧超时猜测状态。真实相机接入时只需在本节点增加一个驱动适配器，提供真实时
间戳、frame、CameraInfo、标定 TF 和点云，不修改下游数据流。

NOLO 适配层的 `run_nolo_driver()` 使用 `hidapi` 枚举和读报告，协议解密/解析集中在
`crates/nolo-cv1`；每只 NOLO 手柄分别维护三轴 One Euro 状态，统一绝对位置使用滤波结果。SDL 通用适配层的 `run_sdl_driver()` 使用 SDL3 的标准 gamepad、sensor、rumble
API，并通过 `has_axis()`、`has_button()` 只发布设备实际声明的标准轴和按钮；路径、类型和中文名称来自同一能力表，网页不维护第二份组件名称映射。NOLO 将协议中已确认的触摸板、扳机、菜单、系统、侧握以及触摸板坐标转换为同样的 `button/*`、`axis/*` 组件，不发布未确认的按键位。生产代码没有手柄型号白名单、型号分支或默认按键映射，型号名称与 USB 信息只作为发现元数据展示。`ImuFusion::update()` 是两类 IMU 输入唯一的姿态融合入口，算法来自 `fusion-ahrs`，项目只做
单位与坐标适配。apply_feedback() 按独立反馈绑定把设备无关的 Action 回馈路由到指定 source_id 和通用能力路径；haptic_intensity() 与 apply_haptic() 只完成 SDL3 归一化强度和反馈 API 的适配，不解释机械臂遥测。SDL3 按运行时实际声明发布 feedback/trigger_left、feedback/trigger_right、feedback/rumble；feedback/virtual 是始终可选的网页目标，选中时把同一 Action 回馈放进采集快照，由共享 Shell 渲染跨页面可拖动圆环。代码不根据型号猜测反馈目标；只有生成式反馈测试在没有已选目标时临时使用网页虚拟反馈，测试结束后恢复原绑定。

手写部分：Dora 编排、消息组装、设备到统一组件的命名、用户绑定求值、NOLO 字节协议适配。
使用库：`hidapi`、`sdl3`、`fusion-ahrs`、`one_euro_filter`、`nalgebra`、`serde`、Dora API 和项目公共的 `json-config-store`。`one_euro_filter` 固定到算法作者仓库当前带参考数据测试的提交；NOLO 位置与通用连续轴共用其上游示例参数。

## spatial-transform-node

节点外壳负责持久化配置和 Dora I/O；磁盘 `SpatialConfig` 只含轴映射、比例、Action 速率和分量
开关，更新时复制、写盘成功后再替换运行配置。全部转换状态在
`crates/spatial-core::SpatialTransform`。
update_pose() 接收已经分别标明空间来源和姿态来源的组合位姿，handle_control() 接收与设备无关的聚合 Action；两者不要求同源。采集节点每个周期先发布位姿、再发布控制帧，空间节点只在控制帧到达时输出一次运动消息，避免同一采样重复进入运动节点。current_output() 以每次接管开始时的位置和姿态为本轮原点；未选择绝对位置时积分底座三轴平移和工具轴向平移，未选择绝对姿态时积分两种圆弧与三个定点旋转。工具轴向螺旋只在位置和姿态都没有绝对来源时原子地积分距离和轴向角度，不能只输出半个动作。绝对姿态的垂直、水平分量分别按配置映射到圆弧或定点旋转，轴向分量映射到工具轴旋转。平移默认满输入 1 cm/s，所有角向动作共用 0.10 rad/s。空间节点不解释执行器动作，只把夹爪连续值和打开按下沿作为统一控制帧的透明载荷传给 motion 节点。矩阵与四元数运算使用 `nalgebra`。可清除的 Action 速率配置由 `serde_with::rust::double_option` 表达，没有自写 JSON 解析分支。

## stararm-102-motion-node

这是 Rust 实现的设备专属运动节点。`main.rs` 的 Dora 事件循环接收空间增量、模型信息、
`ArmState` 和网页请求；`MotionConfig` 通过公共 `json-config-store` 保存控制模式。普通运动进入
同一个 FIFO，前一个 MoveIt action 完成后才取出下一个，不存在并行规划器、固定队列上限或
“已有运动”拒绝路径。活动请求只保存请求 ID、准备相对控制标记和业务取消标记，没有引入
状态机框架或永久故障状态。

`core::target_pose()` 直接消费 `TransformedControlFrame`：先构造两种绕工具后部枢轴的圆弧，
再以得到的 TCP 为中心组合定点俯仰、偏航和轴向旋转，最后沿最终工具轴组合轴向平移与螺旋；
定点旋转不改变 TCP，工具轴动作不退化成底座固定轴。旋转和四元数组合只使用 `nalgebra`。
`tool_position_rad()` 把连续夹爪值线性映射到厂家定义的 90°→0° 行程。
`apply_transformed_control()` 只在这个设备专属节点解释透明传入的夹爪 Action，并且只在控制过程处于启动状态时执行。

`ros.rs` 是唯一 ROS 适配层，使用 `r2r` 的标准 topic、service 和 action 客户端连接 Servo、
MoveGroup、ExecuteTrajectory 与 ros2_control。普通运动只有一个 action worker；它按顺序执行
“同步控制器 → 暂停 Servo → 规划 → 执行 → 恢复 Servo”。业务取消不创建第二个 ROS 路径：
已经提交的 action 自然结束后才开始队列下一项。物理急停是机械臂断电，不由该节点模拟。
ROS Python 包只剩 MoveIt、Servo、ros2_control、robot_state_publisher 的 launch/config，不再
包含 Dora 分发、目标数学、模型服务或运动生命周期代码。

同一 `ros.rs` 也是感知进入 ROS 的唯一边界：Arrow XYZ 转成
`sensor_msgs/msg/PointCloud2` 发布到 `/perception/depth/points`，标定消息转成
`CameraInfo` 和 `/tf_static`。MoveIt 通过项目唯一的 `sensors_3d.yaml` 和标准
`occupancy_map_monitor/PointCloudOctomapUpdater` 消费该点云，自过滤结果发布到
`/perception/depth/points_filtered`，占据数据进入现有 PlanningSceneMonitor。相机明确停止或
用户关闭能力时只调用一次 MoveIt 自带 `/clear_octomap`；普通 CollisionObject 和附着物不清
除。无相机时 updater 继续监听同一 topic，没有消息就是完整语义，不切换 launch 或规划器。
ROS 边界使用 MoveIt 官方 SensorDataQoS；motion 只保留订阅发现前的最新一帧，并在对应标定
已经发布后从既有 tick 发送，不引入收帧超时、重试次数或第二条感知路径。

`ModelCatalog` 位于共享 `stararm-102-model` crate，由 execution 从镜像中实际运行的最终 URDF
读取关节范围并生成模型信息与资源清单；motion 只消费这份模型信息。前端不包含 StarArm-102
关节常量。默认位为
J3=-5°，测试位为 J3=-20°，其余 J1–J6 均为 0°。命名目标同时携带关节和工具执行器位置；
默认位、测试位都把夹爪设为闭合 0°，前端用一个 `MotionRequest` 提交全部目标，motion 节点在
同一轮执行中驱动手臂轨迹与夹爪。
这里的 0° 是工具执行器的物理角度，不是 `primary_tool` 的归一化值。
Servo 保持官方碰撞检查，并在自碰撞距离 1 cm 时开始减速。
普通规划第一次失败时，适配层查询当前状态；仅当起点存在自碰撞时，才在同一个规划请求的
重试中临时允许已检测到的 link 对，从碰撞位置规划退出。该允许矩阵不写入全局规划场景，
执行后不保留状态，也没有碰撞解锁接口或 MoveIt 源码补丁。

## stararm-102-execution-node

`StarArmExecution::load()` 读取串口选择；`configure_endpoint()` 先保存选择，再连接或断开。
未选择真机时，`StarArmExecution` 接受同一个 `ArmCommand` 并发布软件反馈；已经选择真机但
连接中断时冻结最后状态和最后命令，不会隐式切换到软件反馈。连接后把同一命令交给
`StarArmBus`。`encode_command()` 把 J1–J6 和夹爪的统一模型绝对角直接编码为厂家命令，不做
第二次方向换算；只有 ID 6 携带 2000 mW。`read_sorted_monitors()` 一次读取 ID 0–6；
`state_from_monitors()` 生成位置反馈，`telemetry_from_monitors()` 生成电压、电流、功率、温度和状态。
`primary_tool_feedback()` 在此设备专属边界把夹爪 400～2000 mW 映射为通用 `primary_tool` 0～100 力度百分比
Action 回馈；400 mW 来自实测空载 364 mW 后保留的余量。
该字段由每次真机 Monitor 样本生成，稳定的 0 仍是有效反馈，表示扣除空载功率后当前没有
检测到负载，而不是“没有反馈”。夹爪运动中可以产生非零反馈；软件模式只能反馈夹爪位置，
不能伪装成真机力度测量。
串口帧和厂家协议由 `crates/fashionstar-uart` 实现；Monitor 失败先在原串口完整重试一次，仍
失败才执行重开串口和完整初始化。真机反馈周期默认 100 ms，可在执行页修改并持久化；10 ms
的 Dora tick 只负责检查配置周期，不等于每次访问串口。串口枚举使用 `serialport`，只在网页
发送 `discover` 请求时执行，不随定时器或状态快照运行。读取舵机参数继续使用独立的
`refresh` 请求，不会顺带枚举串口。execution 同时持有共享 `ModelCatalog`，发布模型信息并按
manifest 返回资源；motion 与 Web Gateway 均不再维护第二份模型目录。每次接受 `ArmCommand`
后同时发布 transport 和 `ArmState`，因此网页的“最后命令”与软件/真机反馈使用同一事件。

## service-status-node 与 web-gateway-node

状态节点只读取配置中的服务和依赖关系，聚合各节点主动上报的 `ServiceState`；就绪只表示节点正在运行，`has_input` 和 `has_output` 保留为观测字段，不作为通用门槛。没有硬编码业务节点列表、超时门限或探活分支。网关使用 Axum 暴露 HTTP/WebSocket，把请求原样转成 Dora 消息并按
namespace 聚合快照；不解释机械臂轴数、设备类型或运动语义。
共享网页客户端通过浏览器动画帧合并实时快照，避免同一显示帧重复渲染，也不限制位姿、控制、输入测试和关节反馈的更新频率；文本被选中时保留最新待显示快照，避免破坏复制操作。只有采集页的“采集频率”数字在页面本地每秒更新一次，这个显示节拍不会进入后端或其他数据链路。可连接串口列表只在用户点击“刷新串口”后更新。

## 浏览器 RViz 与感知接口

`stararm-102-motion` 容器同时运行 RViz2、Openbox 和 KasmVNC 1.5.0。KasmVNC 使用其现成的
X11、WebSocket、输入转发、远端动态分辨率和 Native Resolution 能力；项目不维护 VNC/编码
协议或第二套网页 3D 规划视图。根入口只把浏览器导向 KasmVNC 自带页面并默认启用其
`enable_hidpi` 参数。唯一入口是 `http://192.168.100.10:6080`，Compose 只映射这
一个 TCP 端口，没有 host network、宿主 DISPLAY 或 X11 socket。该地址位于项目局域网且当前
不启用登录或 TLS，不应直接暴露到公网。

项目 `.rviz` 默认加载 RobotModel、TF、MoveIt MotionPlanning 和规划轨迹，并按“机械臂与
TF / MoveIt 规划 / 深度感知 / 目标与放置区”分组。尚无发布者的彩色图、深度图、原始点
云、过滤点云和 Marker 默认关闭；需要排障时在同一会话启用。PlanningScene 中的 OctoMap 和
CollisionObject 才参与碰撞规划，Marker 只用于解释识别结果。RViz 的 Execute 仅供排障，不
是项目控制入口。

motion 容器内固定的标准接口如下；当前已经发布深度 CameraInfo、TF、原始点云和过滤点
云，其余接口为真实相机与识别节点接入时沿同一链路补齐：

`robot_state_publisher` 与现有 ros2_control 的 100 Hz 关节状态同步发布机器人 TF。MoveIt
自过滤由此能在点云原始采样时间取得各 link 变换；项目没有改写时间戳，也没有增加变换等待
超时或重试分支。

| 用途 | ROS 接口 | 标准消息 |
| --- | --- | --- |
| 彩色图 / 内参 | `/perception/color/image_raw`、`/perception/color/camera_info` | `Image`、`CameraInfo` |
| 深度图 / 内参 | `/perception/depth/image_raw`、`/perception/depth/camera_info` | `Image`、`CameraInfo` |
| 规划点云 / 自过滤点云 | `/perception/depth/points`、`/perception/depth/points_filtered` | `PointCloud2` |
| 识别排障标记 | `/perception/debug/markers` | `MarkerArray` |
| 世界物体 / 附着物 | `/collision_object`、`/attached_collision_object` | `CollisionObject`、`AttachedCollisionObject` |
| 统一规划场景 | `/monitored_planning_scene` | `PlanningScene` |
| 坐标关系 | `/tf`、`/tf_static` | `TFMessage` |

真实相机可以固定安装或腕部安装，差异只体现在标定产物的 parent frame 与变换值；下游始终
消费同一 frame/TF 语义。彩色与深度若使用不同光学坐标系，必须由真实设备标定关系对齐，
不能只改 topic 或 frame 名称。以后识别物体与放置区时，用 CollisionObject 更新同一个
PlanningScene，抓取后用 AttachedCollisionObject 附着到现有 TCP，不重新定义夹爪中心。

## 实现边界

- 输入业务只有一个节点、一份内存状态、一套消息和一条数据流；后端差异止于采集函数。
- 姿态融合只保留 `fusion-ahrs`，没有按手柄型号拆分实现。
- 模拟与真机共享消息、空间转换、运动和执行状态模型；模拟只是输入测试功能。
- 四个可修改服务共享一个宿主 bind mount，但各自拥有独立 JSON；不引入数据库、文件监听器、
  备份层或双写路径。
- 需要持久化的 Rust 节点共用 `json-config-store`；ROS Python 包不保存业务状态。
- 业务层不实现死区、姿态门限、关节范围、起始位检查或设备保护流程。
- 使用 SDL3、hidapi、Fusion、nalgebra、r2r、urdf-rs、serialport、Axum、Dora 和 MoveIt；手写部分都是协议或业务边界，不重复实现这些库已经提供的通用能力。
