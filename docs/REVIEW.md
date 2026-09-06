# 相机、场景与模拟抓放验收

2026-09-06。本文只记录当前 **30 mm 方块**、最终运行镜像与最新释放约定。
旧 90 mm 资产、旧二十轮假阳性和旧 +12 cm 释放报告不作为当前通过依据。

## 结论与边界

- [x] 相机拆分、RGB-D/标定、按需模型、网页与 Docker 服务分层完成整栈回归。
- [x] 连续 20 次软件反馈抓放成功；逐轮用实际关节 FK 和独立网格核对张开、相对两面接触、
  抬升，以及 TCP 到所选点上方 7 cm、水平张合后打开、返回工作位。
- [x] 六张局部图旁提供同一反馈姿态的完整机械臂；生成后助手已目视核对。
- [x] 自然语言入口实际完成中文指令、模型调用、场景实例选择和 MTC 执行。
- [ ] 真实机械臂手眼标定：按用户既定安排等待 J4 恢复和九组最终姿态，不以模拟结果代替。

**这是软件执行和运动学/网格几何验收，不是摩擦、夹持力或真实硬件抓住物体的证明。**
闭合命令图没有接触动力学，穿透照实显示。MoveIt 按用户要求只建地面与目标中心参考点：
独立诊断仍发现假定刚性附着运输时碰筐，不能宣称整段路径避开未建模环境。
按最新约定，这些运输诊断不参与本次抓取及简化释放的通过判定。

## 当前资产与数值

| 项目 | 当前定义 / 实测 |
| --- | --- |
| 方块 | 30 × 30 × 30 mm；中心 (150, -50, 15) mm；yaw=-25°；底面 Z=0；27 cm³ |
| 置物筐 | 外部 300 × 400 × 80 mm；中心 XY=(100,250) mm；yaw=-29°；底面 Z=0 |
| 壁、底、沿 | 壁/底厚 5 mm；四边连续沿向外 15 mm、厚 5 mm；圆角 10 mm |
| 筐体积 | 解析圆角材料约 1219.49 cm³、离散网格约 1219.31 cm³；不是 AABB 包络体积 |
| 筐内空间 | 解析约 8480.89 cm³、离散圆角约 8480.85 cm³；差异来自圆角离散 |
| 相机 | 位置 (30,20,700) mm；原生 1920×1080，另验 1280×720；各自对应内参 |
| Z16 | 米制比例 0.001；地面深度 700 mm，方块顶部 670 mm，筐沿顶部 620 mm |
| 标定板 | ChArUco 5×5 / 单格 15 mm / 标记 11 mm / DICT_4X4_50；棋盘实体 75×75 mm |
| 夹爪角 | open 驱动角 60°、两指总张角约 120°；closed 驱动角 0° |
| 释放 | 所选点的 TCP Z + 70 mm，张合平面水平；不另加物体半高或筐高 |

唯一几何参数为 [测试几何](../backend/nodes/camera/assets/pick-place-scene-geometry.json)。
生成器和审计方法见 [工具说明](../tools/perception/README.md)。
`tv` 只是测试图右上区域能够被默认置信度模型稳定框选的提示词；没有生产类别分支。

## 逐项证据

| 验收项 | 结果 | 可复核产物 |
| --- | --- | --- |
| 同源 RGB / Z16 / 标签 | 同网格、同相机投影；运行 RGB 和 8 位深度预览均为零像素差 | [测量](assets/perception-validation/measurements.json)、[运行内嵌资产](assets/perception-validation/embedded-assets.json) |
| 结构与重复性 | 两种分辨率各五轮、五视角；网格封闭，壁/沿连续，全部真值像素有深度 | [1920](assets/perception-validation/simulation-fixture-verification.json)、[1280](assets/perception-validation/native1280/simulation-fixture-verification.json) |
| 图像、深度、掩码 | 三轮 YOLOE 输出一致；两实例有效深度覆盖均 100% | [合图](assets/perception-validation/overview.png)、[分割](assets/perception-validation/segmentation-overlay.png) |
| 标定与点云 | 实际九姿态 TCP 重放，1920 最大全场景外参位移 0.0463 mm、1280 0.0903 mm | [1920](assets/perception-validation/charuco/1920/charuco-validation.json)、[1280](assets/perception-validation/charuco/1280/charuco-validation.json) |
| 三维形状对照 | 预置与求解外参的刚体变换不改变实体形状/体积；掩码误差单独列出 | [点云图](assets/perception-validation/point-cloud-comparison.png)、[测量](assets/perception-validation/measurements.json) |
| 实际 20 次执行 | 20/20 成功；8478 个关节反馈样本；每轮回工作位 | [原始记录](../tools/diagnostics/results/pick-place-mtc-repeatability.json) |
| 实际抓取几何 | 20/20 张开不交叠、首次双侧接触具有相反面法向、随后抬升 | [逐轮报告](assets/perception-validation/executed-grasp-geometry.json) |
| 释放数值 | 最大 TCP 位置误差 0.0328 mm；水平张合检查通过 | 同上，保留逐轴误差及运行求解器精度来源 |
| 六图及整臂 | 实际反馈 FK 与局部 TCP 一致；闭合穿透没有隐藏 | [图片](assets/perception-validation/executed-closure-and-arm.png)、[位姿/摘要](assets/perception-validation/executed-closure-and-arm.json) |
| 自然语言链路 | 中文指令 → 多提示词 → 实际 ID → MTC，状态 succeeded | [AI 请求与结果](assets/perception-validation/ai-instruction.json) |

RGB/深度运行比较中的深度是 **8 位显示预览**，不是冒称取出了实时原始 Z16。
原始 Z16 的来源由相机内嵌字节、重渲染和对应标签分别核对；文件 SHA 与像素数组 SHA 不混用。

标定阈值是用户指定的 **≤0.1 mm**；实际接口 1920 两轮、1280 一轮各九个样本。
重放分别记录单帧角点/位姿、外参平移、旋转在整个有效深度场景造成的位移，不只看求解残差。
各分辨率报告旁保存其原始 calibration-session.json，正式感知使用的会话另存为
[inference-calibration-session.json](assets/perception-validation/inference-calibration-session.json)。
标定板只在 active 标定会话中进入同一 RGB-D；遮挡、斜视和非标定阶段清除均有测试。

多视角点云尺寸偏差是可见表面像素采样与 Z16 量化的测量结果，没有用任意 2.5 mm
门槛将它包装成完整实体精度。地面由射线/平面交点独立核对，Z16 量化单独检查。
simulation 不提供左右目图像，不能声称验收了立体匹配视差。
当前方块模型掩码 IoU 约 0.819，放置区域约 0.983；分割仍非精确实体真值，
故抓取几何始终对照独立 30 mm 实体，不把模型估计同时用作输入和验收真值。

## 本轮失败原因及保留的最小修改

1. **距离与构型共同影响可达性**。30 mm 方块原 XY=(100,-320) mm 首轮 56 个候选，
   MTC 49 个无 IK、7 个夹指碰地。仅增加抽样不能解决；相同候选移到更近 XY 对照，
   84 次查询中 69 次有解，(150,-50) mm 的 12 次中 11 次有解。
   因而仅更改测试摆放、重生成全部 RGB-D；没有把平移候选注入生产。
   搜索失败只是该次求解未找到解，不是全局不可达的数学证明。
2. **扫描参考深度不等于真实 TCP 深度**。官方 wizard 的 open 扫描中心 Z=60 mm，
   实际 TCP Z=93.38 mm。30 mm 方块的某些 OBB 表面采样使 TCP 落到地面下；
   近位置、同 RGB-D/种子对照，官方 -2/0 cm 仅 3 个场景筛选后候选，补充官方支持的
   +2 cm 向外采样后 10 个，8 个求得 IK、7 个状态有效。
   生产保留 `graspmoe / dense-topandside / score=0.7`，采样偏移为 **-2/0/+2 cm**；
   没有修改输出 TCP、评分或额外筛选顶部方向。这是对官方默认参数的明确变更，不冒称全默认。
   本次测试保存的场景点云邻近距离为 10 mm；通用新配置默认仍为官方 20 mm，网页可改，
   它是采样点距离筛选，不是 MoveIt 地面膨胀或穿透容差。
3. **标定采样条件与栅格偏差**。允许调整的九姿态改为板距约 17–22 cm、适度多轴变化；
   使用相机内参的 CharucoDetector，保留官方 marker 精修默认值。
   OpenCV σ=0.8 px 预滤波有同图真值对照，不修改深度/几何或把真值回填求解结果。
4. **速度与释放语义**。厂商速度覆盖曾与控制器限速不同源，现由型号 URDF 统一。
   StarArm 明确使用 34 rpm，不再折半。释放生成器直接表达 TCP +7 cm 和水平张合，
   删除原物体中心放置和向下阶段，未增加另一条规划路径。

夹爪直尺尺寸和网格本轮没有为通过测试而缩放。向导字段、TCP 完整变换、实际模型条件输入与
曾出现的同面擦碰假阳性详见 [夹爪生成/验收说明](../tools/graspgenx/README.md)；
官方参数与项目差异见 [参数核对](../tools/graspgenx/PARAMETERS.md)。

**旧错误验收已撤销**：曾把双指都碰上表面当作夹持，也曾人为将候选排名加到 MTC 代价。
现保留 MTC 默认方案代价，首个双侧接触检查相对法向，空记录或没有开闭序列不得通过。
“附着成功”“候选很多”“二十次 API 成功”都不是实际夹持证明。

## 方法级、复杂度与分层审查

| 范围 | 保留的路径 / 收敛结果 |
| --- | --- |
| capture worker / driver | 同一 pipeline，SDK Align；Tokio blocking worker 拥有同步设备；原生像素格式仅此处转 RGB8 |
| camera calibration | OpenCV 5 CharucoDetector / solvePnP / SHAH；配置、会话、参数归相机；删重复关节稳定门槛，沿用固定等待 10 s |
| CameraFrameBundle / packed_rgb | 原子 RGB-D、统一内参/外参快照；共享载荷与 stride 检查，删 scene 中重复颜色转换 |
| scene-core / process_scene | 显式请求异步推理；OpenCV 深度连续分组/矩形、nalgebra 变换；无场景类别硬编码 |
| compute infer | 官方批量推理及场景筛选；目标与非目标环境同坐标，环境保留地面；不进入 ROS |
| MTC create_task | 一个多候选生成器复用于抓取和释放，代替每候选一个嵌套 Alternatives；既有 IK/Connect/执行 |
| 运行与测试 | 测试真值、FCL、渲染不进入业务；完整记录源 SHA、反馈、误差和失败，不留临时脚本依赖 |
| 网页 | 共享折叠/灰点/单行省略；AI 与手动配置同场景栏目；视频复用节点内 WebSocket，浮动、拖动、收起 |
| Docker | 服务自有构建/运行依赖；ROS 仅 motion，RealSense 仅 camera，AI 仅 compute；厂商型号资产单源 |
| 证据 | 当前报告集中于本文；旧重复总结与过时图片移出正式证据；区分解析、离散网格和 AABB 体积 |

释放精度判定没有改生产容差：读取 MTC PipelinePlanner 既有每关节目标容差 1e-4 rad，
以及运行 TRAC-IK epsilon 1e-5，通过实际 URDF 链长传播出保守 FK 数值界。
当前约 0.164 mm / 0.035°；逐轮实际误差完整保留。这个数值界不参与夹指接触或碰撞检查。

## 最终回归

- [x] Rust 全 workspace runtime-feature 156 项；Clippy `-D warnings`；cargo-machete。
- [x] camera-node 运行 OpenCV 动态库均来自 `/opt/opencv5/lib/*.so.500`。
- [x] compute 实际运行镜像 pytest 5 项；Python 工具与模型代码 Ruff。
- [x] 前端格式、ESLint、TypeScript、23 项单测、五应用生产构建。
- [x] Playwright 28 项通过、1 项按设计跳过；Compose software-flow 完整通过。
- [x] 实际自然语言 API 和最终模型/执行链路；20 轮反馈几何检查。
- [x] 源码差异、配置、旧路径、工具 README、文件链接与暂存范围复查。

浏览器唯一跳过项依赖未连接的输入硬件；不是隐藏感知或抓放失败。
真实 D415 `924322061032` 的既有实测：62 个 profile（52 可用、10 个 YUYV 置灰），
采集 61.97 FPS、上送 1.00 FPS，SDK 对齐后两路 424×240、同一 frame id。
这些只是该次实测，不是生产分辨率/FPS 固定值。真实手眼标定仍按开头所述单列。

原始测试日志保留在项目 `temp/cube30-*.log`，稳定验收图表与二十轮记录已归档。
