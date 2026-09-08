# 全仓库现状审查（2026-09-07）

起点：`1c693a0`。依据当前源码、固定镜像和真实消息复核生产服务的边界、方法调用、
请求/状态所有权、异步任务、部署和测试；不把历史文档的勾选复制为本次通过。
运行测试使用软件执行反馈，不打开真机端点。软件轨迹成功不证明实际夹持。

## 调用链与所有权

| 入口 | 实际调用链 | 核对点 |
| --- | --- | --- |
| 手柄/鼠标/演示 | controller-input → spatial-transform → motion Servo → ArmCommand → execution | PoseStamped 使用 ROS 时钟；连续相对输入不进离散 FIFO |
| 手动关节目标 | 网关 → motion FIFO → MoveGroup 规划 → FollowJointTrajectory → ArmCommand → execution | 原样执行规划结果，控制器 goal handle 负责取消 |
| RGB-D/视频 | SDK → camera 统一帧 → scene；视频由 camera 直接给浏览器 | SDK Align；采集 FPS 与上送 FPS 分开；无额外视频节点 |
| 自动标定 | camera → motion 模式确认/关节目标 → 正式反馈 → 等待10秒后的新帧 → OpenCV → 用户应用 | Tokio blocking 检测/求解；旧会话结果不能回写 |
| 显式推理 | scene → compute YOLOE → scene-core 掩码/深度 → compute GraspGenX → WorldScene | RGB；相机、标定、配置变化后丢弃旧推理结果 |
| 手动/AI抓放 | 页面或 Next.js AI → 相同场景序号与实例请求 → motion → MTC → controller → execution | AI不另造坐标/轨迹；202等待节点接收确认 |
| 页面状态 | 节点状态 → 网关 → WebSocket初始全快照及后续更新 | 删除并发初始HTTP快照；断线标明最后数据 |

方法见 [BACKEND](../BACKEND.md)，界面与边界见 [SUMMARY](../SUMMARY.md)。
型号值只在 [STARARM-102](../STARARM-102.md) 维护。本轮未修改角度、速度、限位、TCP、
物体尺寸、地面高度、模型置信度，未增加运动保护门限。

## 已修复问题

| 位置/方法 | 原问题 | 当前处理 |
| --- | --- | --- |
| json-config-store::save | 直接截断文件，失败/并发读取可能得到坏JSON | 先序列化，atomic-write-file同目录原子提交；不自写事务层 |
| controller-input配置 | 演示期间改名/保存绑定可能丢失或保存临时配置 | 用户持久配置与临时演示分清，停止恢复用户配置 |
| mouse-control按松 | 旧失败请求清掉后一次按住状态 | 只清理所属按压请求 |
| camera capture worker | 通道断开空转；旧pipeline/打开结果污染新选择 | 通道关闭退出；重开释放旧流；清旧帧、核对请求与来源 |
| camera自动标定 | 同步检测/求解阻塞；模式切换和采样顺序不完整 | 复用Tokio，等模式确认，稳定等待后取新帧，旧会话无结果所有权 |
| camera标定持久化 | 会话频繁写盘，失败可能先改内存 | 会话不序列化，确认配置原子保存后应用 |
| scene异步推理 | 仅按来源名判断，换标定或A→B→A可接受旧结果 | 复用场景序号作为输入版本，不新增任务管理层 |
| scene预览/几何包装 | PNG编码占Dora循环；恒定成功的Result包装 | 结果编码放已有blocking阶段，去掉无错误包装 |
| scene-core反投影 | 忽略非零畸变 | OpenCV5 undistortPoints，支持Brown/rational、Kannala/equidistant；其他非零模型明确报错 |
| camera-calibration::detect | 不知道畸变模型，将不同系数当同一种PnP输入 | 显式传模型，不兼容的非零模型返回清楚原因 |
| compute YOLOE | set_classes和predict可被并发请求插入 | 同一锁覆盖共享模型的两次调用 |
| motion夹爪输入 | 模式/FIFO判断前发送，干扰手动/感知 | 输入夹爪一并遵守相对模式与离散任务所有权 |
| motion请求状态 | 新请求失败覆盖活动ID，旧完成事件取走新任务 | 拒绝只回复自己的请求，匹配ID后取活动任务，失败继续队列 |
| motion cancel | 本地取消或ROS请求成功，但实际走完 | 保留goal handle，以终态确认；见下节复现 |
| execution轮询失败 | 断开不及时发布；旧硬件力度被当新反馈 | 更新transport；仅连接中的hardware发布真实力度，稳定0仍有效 |
| 网关验证 | 非法JSON到节点后from_arrow?使节点退出 | 复用共享请求类型，非法字段400；取消只校验自己的信封 |
| 网关pending | 重复ID覆盖等待者；退出不释放等待请求 | 转发前拒绝重复ID，清理失效接收者，退出释放pending |
| 快照/夹爪/抓放回执 | 快照没给camera；夹爪错误没回请求；无条件202 | 发给实际所有者，独立请求结果，抓放等节点接收确认 |
| PickPlaceRequest | 旧实例ID可在新场景复用 | 加scene_sequence，先核场景后查实例，UI/AI/tools/tests同步 |

畸变支持不能混称“全部相机均已支持”：当前ChArUco/PnP支持forward Brown/rational或零畸变针孔图。
鱼眼（包括零系数）及非零inverse/modified Brown需要驱动提供校正彩色图与对应内参，不能直接塞入同一distCoeffs。
深度重建支持OpenCV鱼眼，不代表ChArUco也能原样使用鱼眼系数。理想模拟输入为零畸变，未改像素/算法。

## 普通取消：不能只看HTTP成功

固定版本：MoveIt 2.15.0，joint_trajectory_controller 6.9.0。软件反馈J1目标0.8 rad、
速度倍率0.03，规划约7.49秒：

1. 最初补goal.cancel后，HTTP取消3–5 ms确认，但原运动仍跑约7.5秒并succeeded，两次复现。
2. 对照镜像中同版本execute_trajectory_action_capability.cpp：accepted callback同步等待执行，
   同互斥回调组的cancel不能及时处理。
3. 删除普通运动ExecuteTrajectory中转，MoveGroup仍规划，原样joint_trajectory交给现有
   FollowJointTrajectory；控制器处理取消，MTC不变。
4. 同轨迹复测：取消请求约1 ms确认，原请求约199 ms结束为cancelled，下一运动成功。
   最终回归再等待真实软件反馈走过10%行程才取消：停止在0.084190 rad，没有到0.8 rad目标，
   原请求为cancelled，下一运动成功。不是只取消尚未开始的任务或只检查状态名称。

没有新增MoveIt补丁、备用停止事件路径或伪成功。FJT成功码0归一为本系统既有普通运动成功码1，
错误保留控制器码及说明。两种码不可混用。
参考 [控制器action/取消语义](https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
及镜像同版本源码；升级后重跑同一取消回归，不照搬“HTTP成功即停止”。

## 迁移、复杂度与文档收敛

- 删除PerceptionRequest.source_id，相机选择只归camera；补齐camera_snapshot和各请求回执的所有权。
- 删除camera/input/scene单节点on-failure重启，遵守整栈一起重启，不维护单节点恢复状态。
- recordings移至temp/recordings；integration-test挂载实际入口、完整相机资产及temp。
- 删除文档重复的过期镜像标签；依赖命令集中DOCKER，测试方法集中tests/README，历史结果留REVIEW/
  排查记录。README不再建议无ROS/OpenCV的Host跑全工作区。型号角度统一同符号、无偏移契约。
- TODO分清本轮审查与历史未证实夹持，已撤销的3/5 mm地面试验不混入现行方案。
- 保留读取用户旧绑定的一次性配置迁移；它不是第二套控制流程。空ROS包标记和Python __init__
  属加载所需，不按“空文件”误删。
- 核对SDK Align、OpenCV、r2r、YOLOE官方loader/predictor、GraspGenX场景调用和资源边界。
  只修证实问题，不新增框架、全局状态机或缓存层。
- 全局1秒检查：仅控制绑定的采集频率数字保留约定刷新。WS重连退避不是状态节流，
  requestAnimationFrame是绘制调度，不是1秒限流。

## 测试复审暴露的旧假设

- fixture的tv只匹配旧视角。当前默认YOLOE参数下red cube/dark rectangle定位方块与篮子，
  后者可能匹配内区和沿。仅更新资产匹配提示词，测试读取同一字段，不再断言“恰好一个”；
  未改几何、图像、模型阈值或生产类别。
- 软件/浏览器测试仍断言放置区Z+7 cm，当前规则已为max(Z,100 mm)且不固定释放朝向。
  统一测试，不改生产轨迹去迎合旧断言。
- 一次并行构建期间网页模型加载超过默认5秒；相同断言单独复跑通过，未放宽断言。
- Python生成向导测试属于计算构建镜像，YOLOE图像loader测试属于运行镜像。混跑分别缺向导/
  libGL，分回所属环境后通过，不为测试补入生产无关依赖。
- 软件反馈不等于测得力度：旧测试刷新执行快照后等待虚拟力度，是伪造硬件反馈的残留假设。
  改用已有显式力度演示验证绑定，不让生产软件反馈生成假测量。演示期间改名也保留临时反馈配置，
  但不将它写进用户配置。
- 控制器改名测试原来只选SDL，实际接口支持全部来源。改为选择在线通用来源，不引入模拟专属接口。

## 验证记录

命令和环境见 [tests/README](../../tests/README.md)，各模块只计最终一次，不累计重复轮次。

| 检查 | 结果 |
| --- | --- |
| 公共库/消息/空间/执行/型号/网关/状态 | 77项通过 |
| 输入原生SDL/HID | 31项通过 |
| camera / realsense-smoke / calibration / realsense-camera | 20 / 11 / 3 / 5项通过 |
| scene-core / scene / motion原生ROS | 8 / 3 / 10项通过 |
| 上述受影响Rust包Clippy，开启对应runtime features | 通过 |
| Python计算/RGB边界/并发/接触、资产描述 | 9项运行环境+1项构建环境通过；Ruff通过 |
| 前端单元/类型/lint | 23项通过 |
| Dora契约 | 8节点84条声明连接，无悬空输入/单节点restart_policy |
| 取消/隔离/错误回传 | 7组通过；最终版本含运动中取消再次通过 |
| 自动标定/模型/MTC/软件全流程 | 最终完整脚本software-flow: ok；含1920两轮、1280一轮，每轮9姿态，0.1 mm断言不变；FIFO、夹爪、取消及15项演示通过 |
| 浏览器整套、十二方向 | 浏览器46项全部通过，无跳过；十二方向完整软件链路两轮通过 |
| 空temp复测 | 输入/配置/执行48项，相机/标定/投影42项，计算9项通过；未清理用户现有目录 |

服务停止后temp可随时清空的约定已加入AGENTS/TODO。正式配置、图像/深度真值、夹爪资产来源与模型
不依赖历史临时产物；Docker构建上下文排除temp，Compose只挂录制/测试输出。修复标定工具输出目录
依赖，Rust测试默认路径统一项目temp。旧RGB审计临时脚本/恢复包仅是当时过程记录，不再承诺长期存在。
中间输入需先按tools对应命令重新采样/生成，不复制临时数据到生产路径。

收尾删除本轮4个临时原生测试镜像标签及约2.4 GiB Cargo/30 MiB工具缓存，均可重建；
运行镜像、固定基础标签、其他任务的镜像/文件未删除。少量本轮原始测试日志留在temp，
可随时清理；本页保存结论，tests/tools保存重跑方法，不依赖日志文件存在。

基础标签未重发或全量重建，只构建受影响应用；运行栈统一force-recreate。
没有安装cargo-machete，不声称它通过。未验收真机夹取、摩擦、力度或真实相机精度。
当时尚未证明实际夹持，不能由该轮软件测试自动勾掉真机验收事项。

## 2026-09-08 文档与代码清理

后续真机标定及实际放置已有独立证据，见[交换布局与夹持反馈](real-white-acrylic-feedback.md)。
上面的测试表保留为当时记录，不表示当前默认检测已满足旧模拟精度标准；后续按用户最新
1 mm 标准的复核与共享内存修复见[收尾验证](closure-20260908.md)。待办完成后已移除 TODO.md。

- 标定历史从型号参数表和工具使用说明中分离；修正过期主线状态、调用方法名和整套启停示例。
- 执行节点清除保持角旧命名和未使用的写入返回值，消息默认实现改为派生，整理等价条件及格式；
  不修改力度控制算法、运动参数、消息契约或错误恢复行为。
- Rust 所属构建镜像中严格 Clippy 通过，消息 15 项及执行 19 项测试通过；
  前端格式、Lint、类型检查及 23 项测试通过。新增只读本地文档链接检查工具并修正错误相对路径。
- 没有执行真机动作或重启服务；本次测试不作为新一轮带载抓放验收。
