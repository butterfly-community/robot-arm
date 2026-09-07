# 当前系统设计

## 服务边界

| 服务或节点 | 拥有的职责 | 明确不负责 |
| --- | --- | --- |
| `controller-input-node` | NOLO、SDL3、模拟输入的能力发现、绑定、命名和反馈路由 | 空间积分、机械臂运动学 |
| `spatial-transform-node` | 输入换基、接管原点、相对位姿与无绝对来源分量的积分 | 设备枚举、MoveIt |
| `camera-node` | 相机能力与配置、采集、SDK 深度对齐、外参标定、原子 RGB-D 发布 | 点云、AI、ROS |
| `scene-node` | 异步计算服务编排、结构化三维场景与抓取候选 | 相机 SDK/配置、轨迹规划、硬件执行 |
| `perception-compute` | YOLOE 提示词识别/分割与 GraspGenX 抓取姿态推理 | 相机、Dora、ROS、动作编排 |
| `stararm-102-motion-node` | 控制模式、普通规划、Servo、MTC 抓放与结构化场景到 MoveIt 的映射 | 串口协议、相机原始数据 |
| `stararm-102-execution-node` | FashionStar 总线、执行反馈、模拟执行和型号元数据 | IK、感知、目标语义 |
| `service-status-node` | 按依赖图聚合服务就绪状态 | 业务恢复策略 |
| `web-gateway-node` | HTTP/WebSocket 与 Dora 请求、状态、按需资源转发 | 设备逻辑、图像计算、配置持久化 |

`scene-core`、`camera-calibration`、`spatial-core`、`realsense-camera` 等是库而不是额外服务。
camera 与 scene 分别部署在 `camera` 和 `perception` Dora machine/Compose 容器中。计算服务可远程部署，
但只和 `scene-node` 交互。

## 唯一数据流

控制链路：

`设备适配器 → controller-input → spatial-transform → motion/MoveIt → ArmCommand → execution`

感知链路：

`相机适配器 → camera-node → CameraFrameBundle → scene-node → WorldScene → motion/MoveIt`

抓放场景的自然语言入口位于 Next.js 服务端：

`自然语言 → 结构化任务 → 既有感知请求 → 实际 WorldScene 实例选择 → 既有 MTC 抓放请求`

AI 不生成坐标、姿态或轨迹；它只编排网页已有的手动接口。选中的对象和放置区域必须存在于本次
`WorldScene`，且抓取对象必须含计算服务实际返回的抓取候选，否则请求不会进入 motion。

真实设备和模拟只在第一层适配器不同。下游没有模拟专用消息、备用 topic、双写或失败时自动
回退。没有选择相机是合法状态：camera 不发布假空帧，scene 清除旧场景有效性，手动和
相对控制仍可使用。

## 手动目标与机械臂预览

运动页的关节和夹爪滑块只编辑浏览器内存中的目标，松开滑块或使用键盘都不发送运动。
点击“执行目标”才通过现有模式切换和 `/api/motion/request` 提交整组目标；默认位、工作位
仍然点击即执行。草稿不作为机械臂配置持久化，也不进入后端状态。

实体模型跟随 `ArmState`，半透明模型显示编辑目标的 URDF 姿态，不代表路径已通过规划。
编辑后实际反馈不能覆盖草稿；执行成功或失败都保留目标，失败原因在编辑区显示。
“恢复当前姿态”用最新反馈替换编辑值，不发送运动，也不取消已提交的运动；取消仍使用原有按钮。
更换型号不沿用旧型号目标。渲染器由运动页和执行页共用 `@robot/visualization/robot-viewer`，
执行页的半透明模型仍表示最后下发命令，不和运动页的未执行草稿混淆。
模型网格只加载一次，再使用 URDF 库克隆目标机械臂，关节表和 mimic 关系由库维护；
避免并发重复加载同一网格导致预览材质在网格到达前应用、最终仍显示成实体颜色。
首次居中前更新整棵 URDF 的世界矩阵，包围盒使用当前关节姿态而非尚未渲染的局部坐标。

回归入口：在 `frontend` 中运行 `pnpm exec playwright test manual-target.spec.ts`，
使用部署中的正式模型与网格，只在测试浏览器拦截控制请求和反馈，验证不自动执行、失败保留目标、
恢复当前姿态及桌面/窄屏布局，不向机械臂发送运动。
`main.spec.ts` 中的命名姿态、夹爪目标测试走正式软件执行链路，会断开执行器并改变模拟姿态；
不得把这一结果称为真机验收。截图统一输出到根目录 `temp/playwright/`。

## 前端状态与交互约定

控制绑定页的“鼠标方向控制”提供前后左右上下和自身姿态的俯仰、水平转向、轴向旋转。
按住鼠标或 Space/Enter 输出，松开、取消指针、窗口失焦或离开页面时发送停止。
进入相对控制后，以当前 TCP 建立输入会话；不执行回默认位、抬升和往返演示。
平移速度及姿态角速度来自空间配置，不在网页另设一套参数。
它复用 `tracking/simulation` 的生成式输入适配器：`value` 有值时保持该控制分量，未传时仍为原有演示。
输入依次经过绑定求值、空间转换、运动节点和执行节点，停止后恢复临时接管前的绑定，不写配置文件。
释放使用既有输入会话复位动作；这不是电机断电或物理急停。页面关闭的停止请求使用 `keepalive`，
网络断开时仍可能发送失败，不能把浏览器按钮当硬件急停。

方向盘回归：`cd frontend && pnpm exec playwright test mouse-control.spec.ts`，
拦截控制请求验证十二个方向、移出按钮松开、快速按松、失焦及桌面/手机布局。
正式软件链路：根目录运行 `node tests/integration/mouse-control.mjs`，要求执行器未连接、
反馈来源为 `software`；从工作位检查输入、空间目标、Servo 执行反馈与停止，最后恢复原姿态及模式，
并确认临时绑定未写入配置。记录保存到 `temp/mouse-control/software-chain.json`，不代表真机验收。
Servo 位姿命令必须使用节点 ROS 时钟填写 `PoseStamped.header.stamp`，不能只填坐标系；
零时间戳会被 Servo 按过期命令处理，出现“空间目标在变、关节不动”。
本轮验收：十二方向软件链路重复两轮通过；浏览器回归 32 项、前端单元测试 23 项、
输入/消息测试 45 项、运动测试 10 项通过。工作位附近部分方向会触发已有 Servo 关节限位或
奇异位减速，记录保留原始诊断，不将“输入正确”解释为“任何姿态都能向任意方向运动”。

- 顶栏显示当前页面与网关的 WebSocket 连接状态，不再用固定的“服务在线”代表整个系统健康。
  断开时明确提示当前数据是最后快照；真机连接、反馈来源仍以执行节点状态为准。
- 复制排障 JSON 只暂停被选中 JSON 的文本替换，不暂停整页状态、实际姿态或任务进度更新。
  解除选择后恢复最新文本。只有控制绑定的采集频率数字按约定每秒更新，其他数据没有这一限制。
- 普通运动请求等待返回时仍可点击“取消普通运动”。它不是断电急停，不用于取消 MTC 任务。
  规划结果同时显示结果码、轨迹点数和计划时长；半透明目标不等于已经执行到位。
- 感知模式切换失败不继续提交抓放。提示词及可选放置角色允许清空编辑；显式选中的实例消失后
  显示未选择，不继续提交旧编号。串联操作等待期间不接受重复提交。
- 抓放阶段和规划错误不藏在默认折叠的配置中；相机采集帧序号与 AI 场景序号分别展示。
  保存失败保留空间矩阵草稿，输入设备改名失败显示错误并允许重试。
- 机械臂预览显示模型加载进度状态；URDF 或网格资源失败时显示原因和刷新重试提示，不把空场景当作正常预览。

只读与故障回归入口：`cd frontend && pnpm exec playwright test frontend-review.spec.ts manual-target.spec.ts`。
测试拦截浏览器中的控制写请求，不驱动真机或运行 AI 模型；使用部署模型验证目标预览，检查五页的
1920、1280、390 像素布局及断连、规划/保存失败等交互。截图输出到 `temp/playwright/`。
这不代替真机运动、相机流及模型推理的专属验收。

## 深度相机

相机硬件 SDK 不属于节点业务代码。`realsense-camera` crate 封装 librealsense context、设备与
profile 枚举、pipeline、frameset、SDK Align、厂商元数据和 FFI；`camera-node` 内的 RealSense 文件
只是把 crate 接到统一 `CameraDriver`/`CameraStream` 接口。资源通过 Rust 所有权和 `Drop`
释放，不额外维护一套显式关闭状态机。

枚举只响应网页“刷新相机”。来源描述包含稳定 ID、驱动、型号、序列号、固件、USB/物理端口、
传感器和驱动实际报告的 profile。RealSense 稳定 ID 使用序列号，profile 和驱动参数键不含运行时
枚举索引。选择只允许当前可用能力；profile、上送频率和用户修改的驱动扩展参数按来源保存，
当前来源与 streaming 状态不保存。保存过的离线来源和暂时缺失的 profile 仍由后端导出，网页
置灰并说明原因，不会因一次枚举变化丢失配置。相机型号、分辨率、格式和 FPS 均不写死。
通用分辨率、格式与采集 FPS 直接进入统一契约；Intel 独有 sensor option 由 `librealsense2`
命名空间包装，simulation 不伪造该扩展，厂商字段不进入 perception、ROS 或 motion。

一次 frameset 只发布一个专用 Arrow `CameraFrameBundle`：元数据使用 JSON 字段，彩色与深度平面
是 Arrow Binary buffer，不使用 Base64 或数值 JSON 数组。bundle 同时包含宽高、stride、格式、
光学 frame、设备/主机时间、共享像素平面内参、设备读取的深度比例和本帧外参快照。彩色与深度
同尺寸、同 frame id。采集任务持续排空设备帧，但只对到达上送周期的 frameset 执行 Align 和
载荷复制；可配置上送频率不改变设备 profile。

`simulation:pick-place-scene` 与 `simulation:depth-grid` 是正式相机适配器，声明自己的 RGB-D
profile、标定元数据和确定性资产。前者只在自动标定会话有效时，根据正式模拟执行反馈的 TCP
位姿渲染 ChArUco 板；普通感知不出现标定板，自动标定仍经过和真机相同的帧与运动消息链路。
RGB、米制深度和实例真值由同一相机和参数化网格生成，尺寸、姿态、壁、底板和沿均有明确真值。
分割得到的可见表面包围体与材料实体积分别报告，不混用。详细数值和图像见 [最终审查与验收](REVIEW.md)。

## 感知、标定与规划

`scene-node` 持续缓存最新原子 RGB-D 帧；只有用户点击“运行一次感知”时才调用计算服务。保存提示词、刷新图像和相机持续来帧
都不会运行 YOLOE 或 GraspGenX。实例掩码与深度生成设备无关的 `SceneObject`、`PlacementRegion` 和
`SceneObstacle`；目标及排除目标后的环境点云仅发送给 GraspGenX，后者保留地面供官方场景筛选。
计算服务按请求的夹爪资产 ID 工作，不读取机械臂
型号，也不向场景硬编码方块、筐或抓取姿态。

原始图像、点云、相机参数和结构化包围体不进入 ROS。motion 从 `WorldScene` 选择任务目标与
放置位姿，MTC 的 PlanningScene 只加入刚性地平面和可附着的目标中心参考点；不再运行 ROS 相机
驱动、`cv_bridge`、`PointCloudOctomapUpdater`、OctoMap 清理接口或感知 Marker 双写。
MoveIt 继续负责机械臂自身和刚性地面的碰撞检查。

`camera-node` 的自动外参标定从 `RobotModelInfo.calibration_targets` 读取型号声明的姿态，通过现有手动关节
`MotionRequest` FIFO 逐项执行。每项运动成功后稳定等待 10 秒，再使用新 RGB 帧检测 ChArUco，
记录同一时刻的正式 `ArmState` 和 FK TCP 位姿。OpenCV 5 的 ChArUco、PnP 与
Robot-World/Hand-Eye SHAH 求解由 Rust `opencv` crate 调用；结果只有经网页确认后才按来源保存。
camera 与 scene 的服务 Dockerfile 各自通过同一 CMake package 步骤把 `opencv-rust` 固定到
`/opt/opencv5`，并在测试中同时检查编译期和运行期 OpenCV 主版本；全局后端基础镜像不含
OpenCV。

## 运动与执行

motion 维护一个顺序工作 FIFO。普通关节运动、标定姿态与抓放不会并行；上一个动作未结束时
下一个等待。抓放使用 MoveIt Task Constructor 标准 stages，输入只有通用对象、放置区域、抓取
候选与型号适配器声明的规划组/TCP/工具信息。所有 FK、IK、attach、可视化与执行统一使用
`tcp_link`。

execution 把唯一 `ArmCommand` 映射为真机总线或软件反馈。选择串口失败不会偷偷切到模拟；
未选择串口才是明确的软件执行模式。StarArm-102 的角度方向、舵机限制、命名位、夹爪联动与
厂家补丁集中在型号目录，见 [StarArm-102 型号适配](STARARM-102.md)。

## 配置与界面

需要持久化的节点复用 `json-config-store`，各自拥有一个 JSON 文件；不存在中央配置服务或前端
配置副本。相机 profile 与外参属于 camera，AI 提示词属于 scene，绑定属于 input，串口和
反馈周期属于 execution。

感知页按“相机与应用场景、相机外参标定、相机参数、采集数据、AI 模型与结果”组织。抓放作为
一个可折叠应用栏目存在：自然语言 AI 是主入口，提示词模型、手动感知、目标选择和 MTC 详情在
同一栏目的“抓放详细配置”中默认收起；它不代表感知服务的全部能力。相机与模型是不同所有者，但在一个页面中协作。深度、标定和叠加图按请求通过网关返回；实时
彩色视频由 `camera-node` 从同一 pipeline 分流，经节点内 WebSocket 直接交给 Canvas，独立于
感知 RGB-D 上送频率。大 RGB-D 帧和深度不会持续经过浏览器。Three.js 机械臂继续以 `ArmState`
显示真实/软件反馈；执行页的半透明目标来自最后命令，运动页则来自当前编辑目标，不由相机链路替代。

自然语言抓放使用 Next.js Route Handler 和 Vercel AI SDK 的 OpenAI-compatible provider。API
密钥只从容器运行环境读取，不进入浏览器 bundle、Dora 消息或后端应用镜像；Compose 从根目录
未跟踪的 `.env` 注入配置。任务解析为每个指代生成从具体描述到常见视觉类别的少量同义提示词，
再以本次实际场景 ID 完成选择；长推理的代理等待只对该 Route Handler 生效，不改变其他 API 的
时序。

## 部署

Compose 只映射 Web `8765` 与 RViz/noVNC `6080`，其余服务在私有网络中。统一 `down`/`up -d`
重启全栈；Dora 容器启用最小 init 负责转发信号和回收子进程。镜像规则见
[Docker 与服务镜像](DOCKER.md)，逐方法实现见 [后端方法与依赖](BACKEND.md)。
