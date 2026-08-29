# 功能绑定与分项演示审查结果

## 最终结构

系统保留一条业务链路：输入驱动生成统一 Action，`spatial-transform` 完成来源仲裁和空间量
转换，StarArm-102 motion 节点生成一个 MoveIt 目标，execution 节点把同一命令写入软件反馈
或串口。生成式测试输入只是 `controller-input` 的一种来源，没有前端动画、模拟专用运动学或
真机专用动作旁路。

统一目录包含十四个输入 Action 和一个反馈测试，页面按 TCP 基础自由度、圆弧复合、夹爪动
作、控制动作、力度反馈、其他复合六组完整展示。消息字段、绑定目录、生成场景、空间消费者
和页面目录的集合测试保持一致；夹爪力度反馈只存在于输出契约，不倒灌进输入帧。

## 动作完整性

- TCP 基础动作覆盖三个平移和三个定点旋转，即刚体 twist 的六个基础分量。
- 垂直圆弧、水平圆弧、工具轴向平移和工具轴向螺旋是基础分量的确定复合，不建立第二套规
  划器。
- 夹爪张开、夹爪连续开合、接管和急停属于非 TCP 业务 Action；默认位、测试位、准备相对
  控制和手动关节命令仍是运动页离散命令。
- SDL3、NOLO CV1 和生成式测试源只报告实际能力，不改变业务动作全集。

核对依据为 ROS 2 Jazzy
[`geometry_msgs/Twist`](https://docs.ros.org/en/jazzy/p/geometry_msgs/msg/Twist.html)、MoveIt
[`Realtime Servo`](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)、
MoveIt Task Constructor
[`MoveRelative`](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/propagating_stages.html)
以及《[Modern Robotics》第 3 章](https://modernrobotics.northwestern.edu/chapters/chapter3/)。

## 复杂度、迁移与造轮子审查

- `INPUT_ACTIONS` 是 Rust 运行时动作键、类型和动作域的唯一目录；空间/姿态来源互斥只由
  `spatial-core` 的动作域仲裁一次。TypeScript 目录补充中文说明和页面分组，集合测试防止
  键值漂移。
- 十五项演示共用一个采样器、一个请求入口和一个配置恢复出口。Gateway 只做类型检查与转
  发，不计算姿态或机械臂目标。
- 删除旧完整轮播 fixture 和入口，回放 fixture 改为动作语义结构；未保留 schema 2、旧场景、
  `model.json` 或设备专属模拟路径。
- 四元数和矩阵使用 `nalgebra`、`transforms3d`；姿态融合、滤波、设备访问、串口、Web 与消息
  编排分别使用 `fusion-ahrs`、`one_euro_filter`、SDL3/hidapi、serialport、Axum 和 Dora。
  新代码没有重复实现这些库已有的方法。
- 未增加运行门限、保护条件、自动重试、降级状态机或第二执行路径。演示数值只复用现有线
  速度、角速度、100 Hz 和三秒双向余弦轮廓。

## 验收结果

- Rust：格式、Clippy 和 75 项测试通过。
- Python/MoveIt：镜像内 21 项 `unittest` 通过。
- 前端：格式、类型检查、Lint 和 10 项单元测试通过；Next.js 16.3.3 四个应用生产构建通过。
- 浏览器：Playwright 18 项通过。
- Compose：完整构建、`down`、`up -d` 和所有服务健康检查通过。
- 软件链路：十五个输入演示/反馈测试连续通过，结束后来源、Action 绑定和反馈绑定恢复；网
  页机械臂只消费后端反馈。
- 真机证据：纵向、横向、垂直平移和定点垂直旋转的最大关节变化分别为 1.8°、2.0°、2.4°、
  7.1°，夹爪 0°～90° 行程已确认。其余 TCP 项目的软件目标已通过相同链路；运行环境自动
  审核因既往定点水平旋转曾产生约 43° 关节重分配而拒绝再次下发，因此未把未执行项目记作
  真机通过，也没有把该外部限制转化为生产代码限制。
