# 后端方法与依赖

## controller-input-node

入口 `main()` 只连接 Dora 输入输出。`ControllerInput::load()` 读取持久配置，
`commit_config()` 先写盘再替换内存配置；`ControllerInput::drain()` 合并驱动事件；`tick()` 只在收到
新样本时生成统一位姿和 Action；select_pose_source() 分别配置空间位置与姿态来源，并按运行时对应能力筛选和校验候选；apply_bindings() 保存每个 Action 自己的 source_id、输入组件以及独立的 Action 反馈目标；应用绑定时对已选连续组件采集 3 秒静止值并把均值作为零偏一同持久化。测试播放和硬件驱动共用 replace_driver_sources()、accept_sample()、combined_pose_frame() 与 evaluate_actions()：启动时声明具备绝对空间、绝对姿态、接管按钮和夹爪连续轴的通用六自由度输入源，通过普通来源选择与 Action 绑定依次测试三轴空间移动、偏航/俯仰/横滚姿态和夹爪闭合/张开，并临时把夹爪力度反馈绑定到网页虚拟反馈；停止后移除测试来源并恢复启动前配置。模拟数据不会直接构造最终位姿或 Action，也不模拟某个具体手柄型号。combined_pose_frame() 从两个来源透明合成位姿；evaluate_actions() 与 binding_value() 可同时读取多个设备，把连续轴或一对方向按钮转换成功能 Action；连续值只减去绑定时记录的零偏，不增加死区。能力筛选只约束绝对空间与姿态来源，不约束 Action 或反馈绑定；没有绝对位姿的分量仍可由按键或轴 Action 驱动空间节点积分。输入源是否在线根据当前发现结果计算，不把运行时的 active 状态写盘。

网页启用测试播放前只调用 `/api/motion/prepare-relative`：motion 节点复用普通模式切换和普通 MoveIt 运动链路，依次进入手动模式、执行包含夹爪闭合的默认位、在执行成功后进入相对模式；同步响应返回后，网页才启用测试输入源；运动页把同一接口显示为“准备相对控制”按钮，并与默认位、测试位放在同一按钮组。测试源每次从默认位对应的夹爪闭合值开始，末段先张开再闭合并回到同一状态。停止测试输入只停止输入源，不触发归位。
零偏只进入 Action 求值，按钮不校正，原始输入仍保持原值。`live_component_values` 原样携带每个在线来源最近一帧的组件值，采集页用它显示当前按下的按钮或偏离零位的轴，便于确认物理控件名称；它不参与 Action 求值，也不形成测试旁路。

NOLO 适配层的 `run_nolo_driver()` 使用 `hidapi` 枚举和读报告，协议解密/解析集中在
`crates/nolo-cv1`。SDL 通用适配层的 `run_sdl_driver()` 使用 SDL3 的标准 gamepad、sensor、rumble
API，并通过 `has_axis()`、`has_button()` 只发布设备实际声明的标准轴和按钮；路径、类型和中文名称来自同一能力表，网页不维护第二份组件名称映射。NOLO 将协议中已确认的触摸板、扳机、菜单、系统、侧握以及触摸板坐标转换为同样的 `button/*`、`axis/*` 组件，不发布未确认的按键位。生产代码没有手柄型号白名单、型号分支或默认按键映射，型号名称与 USB 信息只作为发现元数据展示。`ImuFusion::update()` 是两类 IMU 输入唯一的姿态融合入口，算法来自 `fusion-ahrs`，项目只做
单位与坐标适配。apply_feedback() 按独立反馈绑定把设备无关的 Action 回馈路由到指定 source_id 和通用能力路径；haptic_intensity() 与 apply_haptic() 只完成 SDL3 归一化强度和反馈 API 的适配，不解释机械臂遥测。SDL3 按运行时实际声明发布 feedback/trigger_left、feedback/trigger_right、feedback/rumble；feedback/virtual 是始终可选的网页目标，选中时把同一 Action 回馈放进采集快照，由共享 Shell 渲染跨页面可拖动圆环。代码不根据型号猜测目标，也没有未绑定时的默认回退。

手写部分：Dora 编排、消息组装、设备到统一组件的命名、用户绑定求值、NOLO 字节协议适配。
使用库：`hidapi`、`sdl3`、`fusion-ahrs`、`nalgebra`、`serde`、Dora API，以及项目公共的
`json-config-store`。

## spatial-transform-node

节点外壳负责持久化配置和 Dora I/O；磁盘 `SpatialConfig` 只含轴映射、比例、Action 速率、分量
开关和原点，更新时复制、写盘成功后再替换运行配置。全部转换状态机在
`crates/spatial-core::SpatialTransform`。
update_pose() 接收已经分别标明空间来源和姿态来源的组合位姿，handle_control() 接收与设备无关的聚合 Action；两者不要求同源。采集节点每个周期先发布位姿、再发布控制帧，空间节点只在控制帧到达时输出一次运动消息，避免同一采样重复进入运动节点。current_output() 以每次接管时的有效位姿分量为基准输出相对平移、前部圆弧和水平圆弧；未选择绝对位置或姿态来源的分量分别按用户配置速率积分对应 Action；平移默认满输入 1 cm/s；圆弧默认角速度为 0.10 rad/s，选择后则只消费该绝对分量。夹爪连续值和打开按下沿原样穿过，不依赖整臂空间接管。矩阵与四元数运算使用 `nalgebra`。配置补丁的“字段缺失”和“显式 `null` 清除”由
`serde_with::rust::double_option` 表达，没有自写 JSON 解析分支。

## stararm-102-motion-node

这是设备专属运动学节点。Dora 外壳处理请求和状态；`load_motion_config()`、
`save_motion_config()` 使用 Python 标准库 JSON 保存控制模式，写盘成功后才替换内存状态。
MoveIt/Servo 负责规划、逆运动学、控制器同步
和碰撞模型。`motion_core.target_pose()` 仅构造绕机械臂工具后部枢轴的目标；四元数运算使用
`transforms3d`。运动镜像固定 MoveIt Servo 2.12.4，并应用项目补丁把 `self_collision_proximity_threshold=0.002` 同时作为自碰撞网格穿透容差：2 mm 以内不急停，超过才停止；环境碰撞判断保持原样。运动页的“解除碰撞”通过同一个 `MotionRequest` 调用 Servo 官方动态参数服务关闭碰撞检查，按钮随后变为“恢复碰撞检测”，不创建另一条运动链。`MotionState` 明确报告碰撞检查状态和容差。`tool_position_rad()` 把连续夹爪值线性映射到厂家定义的 90°→0° 行程。
`_model_info()` 通过通用 `named_targets` 契约发布默认位和测试位，前端只消费契约，不包含
StarArm-102 关节常量。命名目标同时携带关节和工具执行器位置；默认位、测试位都把夹爪设为闭合 0°，前端用一个 `MotionRequest` 提交全部目标，motion 节点在同一轮执行中驱动手臂轨迹与夹爪。
这里的 0° 是工具执行器的物理角度，不是 `primary_tool` 的归一化值。

## stararm-102-execution-node

`StarArmExecution::load()` 读取串口选择；`configure_endpoint()` 先保存选择，再连接或断开。
`StarArmExecution` 在未连接串口时接受同一个 `ArmCommand` 并发布软件反馈，连接后把同一命令交给
`StarArmBus`。`encode_command()` 把 J1–J6 和夹爪的统一模型绝对角直接编码为厂家命令，不做
第二次方向换算；只有 ID 6 携带 2000 mW。`read_sorted_monitors()` 一次读取 ID 0–6；
`state_from_monitors()` 生成位置反馈，`telemetry_from_monitors()` 生成电压、电流、功率、温度和状态。
`primary_tool_feedback()` 在此设备专属边界把夹爪 400～2000 mW 映射为通用 `primary_tool` 0～100 力度百分比
Action 回馈；400 mW 来自实测空载 364 mW 后保留的余量。
串口帧和厂家协议由 `crates/fashionstar-uart` 实现；串口枚举使用 `serialport`，只在网页发送 `discover` 请求时执行，不随定时器或状态快照运行。读取舵机参数继续使用独立的 `refresh` 请求，不会顺带枚举串口。

## service-status-node 与 web-gateway-node

状态节点只读取配置中的服务和依赖关系，聚合各节点主动上报的 `ServiceState`；就绪只表示节点正在运行，`has_input` 和 `has_output` 保留为观测字段，不作为通用门槛。没有硬编码业务节点列表、超时门限或探活分支。网关使用 Axum 暴露 HTTP/WebSocket，把请求原样转成 Dora 消息并按
namespace 聚合快照；不解释机械臂轴数、设备类型或运动语义。
共享网页客户端通过浏览器动画帧合并实时快照，避免同一显示帧重复渲染，也不限制位姿、控制、输入测试和关节反馈的更新频率；文本被选中时保留最新待显示快照，避免破坏复制操作。只有采集页的“采集频率”数字在页面本地每秒更新一次，这个显示节拍不会进入后端或其他数据链路。可连接串口列表只在用户点击“刷新串口”后更新。

## 实现边界

- 输入业务只有一个节点、一个配置文件、一套消息和一条数据流；后端差异止于采集函数。
- 姿态融合只保留 `fusion-ahrs`，没有按手柄型号拆分实现。
- 模拟与真机共享消息、空间转换、运动和执行状态模型；模拟只是输入测试功能。
- 四个可修改服务共享一个宿主 bind mount，但各自拥有独立 JSON；不引入数据库、文件监听器、
  备份层或双写路径。
- Rust 节点共用 `json-config-store` 的读取、默认值和格式化写盘方法；Python motion 节点直接使用
  标准库 JSON，没有为一个文件引入配置框架。
- 业务层不实现死区、姿态门限、关节范围、起始位检查或设备保护流程。
- 使用 SDL3、hidapi、Fusion、nalgebra、transforms3d、serialport、Axum、Dora 和 MoveIt；手写部分都是协议或业务边界，不重复实现这些库已经提供的通用能力。
