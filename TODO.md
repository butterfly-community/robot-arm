# 相机与场景链路收敛 TODO

> 状态：本次软件主线实现与最终整栈验收完成。30 mm 方块的 20 次实际反馈几何检查通过；
> 放置为 TCP 到所选点上方 7 cm、水平张合后打开并回工作位。
> 当前结果与边界集中在 docs/REVIEW.md；真实手眼标定仍按既定安排等待 J4 恢复和九组姿态。

## 最终边界

唯一生产链路：

`相机驱动适配器 → camera-node → 已对齐 RGB-D → scene-node → perception-compute → WorldScene → motion`

- `camera-node` 拥有相机发现、稳定身份、profile、驱动参数、采集、帧同步、深度对齐、配置、
  内外参标定和持久化。
- `scene-node` 缓存最新原子 RGB-D，只在用户请求时编排模型调用，发布预览、实例、抓取候选和
  结构化场景。
- `perception-compute` 只运行 YOLOE/GraspGenX；不接触相机、运动或网页状态。
- `motion` 只消费 `WorldScene` 和抓取候选；原始图像、深度和厂商字段不进入 ROS/MoveIt。
  MoveIt 的环境碰撞世界只包含刚性地平面；目标以可附着的中心参考点表示，放置使用中心位姿，
  分割包围体、置物区和其他感知几何不参与路径碰撞。
- simulation 是 `camera-node` 的普通驱动适配器，和真机使用同一帧、配置、标定及下游契约。

## 已完成

- [x] 节点、Cargo package、二进制、Dora id、配置和内部 crate 一次性改为最终名称；同步 Docker、
  Compose、dataflow、service-status、gateway、网页契约和测试。
- [x] 配置只使用 `/config/camera.json` 与 `/config/scene.json`；移除旧文件、旧入口、兼容读取、
  空壳 crate 和重复配置来源。
- [x] `CameraFrameBundle` 收窄为彩色图、对齐到彩色平面的深度图、共享内参、深度比例及该帧的
  标定快照；RGB-D 数据用 Arrow `BinaryArray` 传递。
- [x] 帧契约检查像素格式、尺寸、stride、载荷长度、共享 frame id、共享像素平面和深度比例。
- [x] RealSense 适配器使用 `realsense-rust`/librealsense `Align`；不再手写畸变、反投影、外参
  重投影或 Z-buffer。
- [x] 驱动在 Tokio 管理的长期 `spawn_blocking` worker 中独占同步 SDK；有界命令/完成 channel
  和 latest-value watch channel 防止积压，Dora 事件线程不轮询硬件。
- [x] 高频 frameset 持续排空；只有达到上送周期的帧执行 Align 与载荷物化。设备缺帧和主动略过
  分开统计，profile FPS 与上送 FPS 独立。
- [x] 相机配置、预置外参、标定会话、标定资产和持久化归入 `camera-node`；OpenCV 算法提取为
  进程内 `camera-calibration` Rust 库，删除子进程和 Base64/JSON stdin/stdout 链路。
- [x] 标定继续直接使用 OpenCV 5 `CharucoDetector`、`solvePnP` 和
  `calibrateRobotWorldHandEye`；板参数保持 `5×5 / 15 mm / 11 mm / DICT_4X4_50`。
- [x] `scene-node` 使用异步 `reqwest::Client`；PNG、掩码、场景融合等 CPU 工作进入
  `spawn_blocking`。同类模型任务只允许一个在执行，配置或相机改变时丢弃过期结果。
- [x] YOLOE/GraspGenX 只由手动感知请求触发；提示词和实例角色保持开放，不写死方块、筐或
  其他测试场景。
- [x] 删除下游深度对齐、全幅/障碍点云 ROS 消息与 MoveIt 感知依赖；针孔投影在 `scene-core`
  集中实现，实例点云供候选生成，非目标环境点云只供 GraspGenX 官方场景筛选，不进入 ROS。
- [x] 网页相机操作、标定、任务状态和按钮锁定均由后端状态驱动；未选择相机是正常默认状态，
  保存配置不会持久化运行态选择。
- [x] `camera-node` 从同一 pipeline 的每个 frameset 分流最新彩色帧，在节点内提供 WebSocket；
  浏览器 Canvas 使用 `@thi.ng/pixel` 按驱动声明的像素格式显示。该流不经过 Dora/Gateway，不新增
  节点、第二次采集或深度数据副本，也不受感知 RGB-D 上送频率限制。
- [x] 抓放作为感知页内的可折叠应用栏目，而不是感知服务的全部能力；AI 自然语言入口突出展示，
  模型、手动感知、对象/区域选择和 MTC 详情默认折叠并记忆状态。
- [x] 自然语言编排放在 Next.js 服务端并使用 AI SDK；先生成通用开放词汇提示词，运行同一感知
  请求，再从实际 `WorldScene` 选择有效实例，最后调用同一 MTC 抓放接口。未新增 Rust 节点、消息、
  坐标生成或备用运动路径。
- [x] 每个自然语言实体生成从具体描述到常见视觉类别的少量英文同义提示词；放置目标的同义词
  共享同一角色。真实链路验证不会因单一翻译词未命中而提前进入运动。
- [x] FastiCode 地址、`gpt-5.6-sol` 和鉴权只由根目录未跟踪 `.env` 注入 `web-perception`；浏览器、
  镜像和 Git 均不包含密钥，仓库只保留 `.env.example`。
- [x] Docker 按服务所有权完全拆分构建/运行阶段：全局后端基础只保留三个以上服务共用的
  Rust/Dora 环境；camera 独占 RealSense，scene 独占自身 OpenCV 5，motion 独占
  ROS 2/MoveIt/RViz，perception-compute 独占 Python、PyTorch、YOLOE、GraspGenX、模型和
  夹爪计算资产。删除能够运行所有节点的总后端镜像。
- [x] camera 与 scene 都在各自 Cargo 构建前设置 `OpenCV_DIR`、`LD_LIBRARY_PATH`、
  `PKG_CONFIG_PATH` 和 `OPENCV_DISABLE_PROBES=pkg_config`；完全相同的 OpenCV 5 源码步骤
  由 Docker 内容缓存复用，不为复用重新建立隐藏共享层。
- [x] ROS 依赖收窄到运动服务实际使用的 MoveIt/Servo/MTC/RViz/控制器包，移除不再使用的感知包。
- [x] simulation 的 ChArUco 只在自动标定会话 active 时进入同一 RGB-D 帧；普通感知与抓放不再
  因持续存在的标定板污染场景。
- [x] 参数化网格在同一相机投影下生成 RGB、Z16 深度和实例真值：方块按最新要求改为 30 mm 立方体；
  置物筐外部 300 × 400 × 80 mm，壁与底厚 5 mm，连续四边沿向外 15 mm、厚 5 mm。
  两者底面均为 Z=0；实体材料体积、内部容积与 AABB 包络分别计算，禁止混用。
- [x] 彩色视频复用现有 WebSocket 改为可拖动、可收起且记忆状态的浮动窗口；深度图不再随 1 Hz
  状态刷新重建；跨页共享控件的间距和单行省略显示完成回归检查。

## 方法级裁决

- [x] 深度注册调用厂商 SDK `Align`，没有保留备用算法或第二条 raw-depth 路径。
- [x] ChArUco、PnP、手眼标定调用 OpenCV；矩阵变换调用 `nalgebra`；模型前后处理调用官方
  Ultralytics/GraspGenX 路径。
- [x] 保留 `scene-core` 的短小针孔公式：OpenCV 5 当前精简构建不含 contrib `rgbd`，为这一处
  标准投影加入 contrib 会增加基础镜像、FFI 和维护面；实现有尺寸、内参和坐标测试。
- [x] 模型输入和结果继续使用按需 PNG/JSON；实时彩色预览独立使用节点内 latest-value
  WebSocket。当前可用 profile 限定为统一帧契约支持的未压缩格式，页面使用像素库，不引入
  FFmpeg/GStreamer、录像协议或第二个服务；后续只有实测带宽/CPU 不满足时才迁移编码。
- [x] 删除未使用依赖、不可达采集结果分支、旧点云函数/测试、旧标定资产、空目录和迁移说明。

## 验收

- [x] 用户指定的六图抓取验收：两个固定视角 × 张开 / 首次双侧接触 / 闭合命令几何。
  保留真实网格、方块真值、明确的驱动关节角和穿透显示；每组附源资产版本、候选索引、
  TCP 位姿及接触报告。分别给原始候选和任何离线深度对照生成图，最终以实际执行候选
  为准供用户确认。图片不能替代数值核验，闭合穿透不能被画成物理夹持。
  六图旁增加完整机械臂、方块、筐和地平面；使用实际反馈或明确标注的 IK 关节解，
  先验证整臂 FK 与局部 TCP 一致。无 IK 解不伪造整臂姿态，生成后助手也必须看图核对。
- [x] 本次测试代码统一保留在 `tools/` 或 `tests/`，README 写明镜像、输入、输出、命令与
  验证边界；`temp/` 只保留测试产物，不让复验依赖一次性临时脚本。

最终整栈已按当前资产、夹爪与模型设置重跑。二十轮 MTC 成功不等于二十轮夹持成功：
旧脚本把双指同面擦碰计为双侧接触，该批几何结论已撤销；本次二十轮逐一核对首次相对接触面，
原始关节反馈、独立报告和六图均已归档，不包含真实接触力或摩擦动力学证明。

- [x] 源码 RGB/BGR 链路统一：原生格式只在采集边界经 OpenCV 转 RGB8，Arrow/scene/模型
  API/视频保持 RGB；OpenCV 标定明确 BGR 编解码边界。颜色、stride、官方 YOLOE 输入张量和
  浏览器像素测试通过，用法记录在 `docs/BACKEND.md`。运行镜像联验并入最终整栈验收。
- [x] 整套模拟资产重新审核，旧报告不能替代当前版本：固定并记录几何、RGB、深度、
  ChArUco、内外参、模型和夹爪资产的版本/摘要；图像、深度、点云、实体及运行态必须对应。
  当前二进制内嵌资产、1920 运行 RGB/深度预览逐像素核对通过；1280/1920 各五轮、五视角
  验证通过。模型掩码误差、未选中候选可达性与运输碰筐诊断独立保留，详见 `docs/REVIEW.md`，
  不将资产几何正确等同于模型、规划或抓取正确。
- [x] 按用户新要求将模拟标定偏差验收收紧到 **不超过 0.1 mm**；可自由调整标定板姿态，
  降低过大倾角并改善成像距离，但保留求解所需的多轴变化。实际九姿态流程与
  1280×720、1920×1080 均须测量，不用近距离独立单元测试替代运行流程。
  分别报告单帧板位姿误差、外参平移误差、外参旋转对抓取工作区造成的位置误差；
  深度量化误差另列，不能混称为标定误差或调整真值来通过验收。
  本轮实际接口 1920 两轮、1280 一轮均取得九样本；实际 TCP 重放全部有效深度点后，
  最大外参位移分别约 0.046 / 0.090 mm。未连接真机，J4 硬件标定仍单列待验收。
- [x] 张开、首次接触、闭合分开渲染和计算；TCP 箭头不等于夹爪实体，离散网格相交
  不等于无穿透、稳定夹持或力封闭。候选几何、IK 和实际执行各自验收，不能互相替代。
- [x] 对照锁定版本 `demo_scene_pc.py` 完成 RGB-D → 分割 → 同坐标系实例/环境点云 →
  官方批量推理 → 场景碰撞筛选 → TCP 转换的运行验收。地面必须保留在环境点云中；
  RGB 的展示/分割用途和生成网络零颜色通道分别记录，不能混称为“只接收点云”或“RGB 生成网络”。
  使用官方 `graspmoe`、`dense-topandside`；保留默认 -2/0 cm 采样并补充经实测核对的 +2 cm
  向外采样，原因及同输入对照记录在 `tools/graspgenx/PARAMETERS.md`。不改变返回位姿，
  用户已撤销顶部采样要求，不保留额外方向筛选。
- [x] 按最终中心点规划边界验收：MTC 只加载刚性地面和目标中心参考点；独立夹爪网格必须
  证明实际执行的候选在张开时不穿入物体、闭合时双指首次接触相对两面，并完成抬升。
  不能只证明候选集合中存在可抓姿态，不能把规划附着等同于真正夹住。
- [x] 放置按用户最新要求：TCP 到所选放置点 Z + 0.07 m，张合平面平行地面后打开，
  然后回工作位。取消原先下降到区域上表面的流程；运输碰筐、投影是否落筐只保留诊断，
  不再作为本次通过条件，也不恢复环境几何。释放目标不随物体尺寸或区域高度额外累加。

- [x] Rust 全 workspace runtime-feature 测试 156 项通过；Clippy `-D warnings` 与
  `cargo machete` 通过。
- [x] OpenCV 编译期/运行期版本测试通过；最终 `camera-node` 的全部 OpenCV 动态库来自
  `/opt/opencv5/lib/*.so.500`。
- [x] Python 模型服务 5 项测试与 Ruff 通过；前端格式、ESLint、TypeScript、23 项单测和 5 个
  Next.js 16.3.4 生产构建通过；peer 依赖无冲突，工具链主版本与其插件支持范围一致。
- [x] 最终 Playwright 28 项通过、1 项依赖未连接输入硬件按设计跳过；Compose software-flow
  完整通过，包含双分辨率、九姿态标定、实际模型、MTC、工作位恢复和动作演示。
  当前 0.7 官方评分与 -2/0/+2 cm 采样下，另录制 20 次实际反馈并通过独立几何检查。
- [x] 导出运行态彩色图、深度图、YOLOE 分割、掩码和预设/九样本标定点云对比；两个实例的
  掩码深度覆盖率均为 100%，并明确区分立体视差、外参造成的空间位移和 AABB 包络体积差。
- [x] `gpt-5.6-sol` 实际完成“中文指令 → 多提示词 → YOLOE/GraspGenX → 真实场景 ID 校验 →
  MTC 软件抓放”，最终状态 `succeeded / pick and place complete`；运动执行端确认未连接真机。
- [x] 两个 simulation 来源均走唯一相机链路；方块/置物筐完成模型和抓取候选测试，点云网格完成
  帧、标定和预览测试。
- [x] 九姿态 simulation 标定完成并量化；无标定板场景重新运行模型和 MTC，软件抓放返回成功；
  另外进行实际轨迹几何证明，不把返回成功等同于已经夹住。
- [x] D415 `924322061032` 真机完成 profile、驱动参数、采集、SDK 对齐、节流和预览测试；最终
  实测 `61.97 FPS` 采集、`1.00 FPS` 上送，对齐后彩色/深度均为 `424×240` 和同一 frame id。
- [x] 整栈重启后相机保持默认未选择，D415 与 simulation 的保存配置仍可按稳定 id 恢复。
- [x] 完成格式、过时关键词、旧配置、重复路径、空壳、Docker 配置和 `git diff --check` 审查。

## 后续独立验收

- [ ] J4 恢复后，由用户提供九组最终姿态，完成 D415 的真实 ChArUco 手眼标定并确认应用结果。
