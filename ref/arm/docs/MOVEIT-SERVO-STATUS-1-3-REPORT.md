# Star Arm 102-FL MoveIt Servo 状态 1/3 问题

## 运行环境

- ROS 2 Jazzy
- MoveIt Servo 2.12.4
- 机械臂：Star Arm 102-FL
- 使用厂家最新提供的 URDF 和 STL
- MoveIt 运动链：`base_link -> link6`
- 当前起始位：J1–J6 `[0°, 0°, -3°, 0°, 0°, 0°]`

## 问题现象

使用 MoveIt Servo 做连续笛卡尔控制时，`/servo_node/status` 会频繁在以下两个状态之间切换：

```text
1: DECELERATE_FOR_APPROACHING_SINGULARITY
3: DECELERATE_FOR_LEAVING_SINGULARITY
```

没有运动指令时可以恢复到 `0: NO_WARNING`。发送小幅前后、左右、上下移动或姿态变化后，
状态 1 和 3 可能反复出现，Servo 随之反复减速。

关节空间的 MoveGroup 规划和返回起始位可以正常执行。问题主要发生在 MoveIt Servo 的连续
笛卡尔控制过程中。

## 已完成的排查

1. 最初使用的厂家 URDF 中存在可疑几何值，例如 `0.120565306190578`。
2. 已换用厂家后来提供的新版 URDF 和 STL。新版 J3 原点为：

   ```xml
   xyz="0.12162 -0.0021 0"
   ```

3. 新版模型的连杆树完整，J1–J6 起始位雅可比矩阵为满秩，但起始位附近的数值条件仍然较差。
4. 曾测试额外的 `tool0`，但该坐标落在夹爪转轴附近且没有实物测量依据，现已删除。
5. 当前直接使用厂家模型中的 `link6` 作为 Servo 末端。
6. `ros2_control` 的 J5 和夹爪 command interface 范围已与新版 URDF 对齐；其他关节原本一致。
7. 已测试多个接近零位的起始姿态，状态 1/3 仍会在笛卡尔运动过程中出现。
8. 有限的奇异位 hard stop 会导致状态 2 后无法发送反向退出指令，因此当前只保留奇异状态检测
   和减速，不使用有限 hard stop。该调整可以退出奇异区域，但不会消除状态 1/3 的反复切换。
9. 状态来自 MoveIt Servo 的 `/servo_node/status`，不是网页自行判断，也与串口是否连接无关。
10. 已将厂家默认 KDL IK 插件替换为 Jazzy 的 TRAC-IK 2.0.2。Servo 和 move_group 均确认加载
    TRAC-IK，`/compute_ik` 能正常返回六轴解；标准模拟中状态 1/3 仍然复现。因此问题不是
    KDL 单次逆解失败，仍指向当前模型和姿态的雅可比条件。

## 复现方法

1. 将 J1–J6 移动到 `[0°, 0°, -3°, 0°, 0°, 0°]`。
2. 启动 MoveIt Servo Pose 命令模式。
3. 连续发送小幅笛卡尔位置或姿态目标。
4. 观察状态：

   ```bash
   ros2 topic echo /servo_node/status
   ```

5. 可以看到状态码 1 和 3 在运动过程中反复出现。

## 需要厂家确认

1. 新版 URDF 中 J1–J6 的 `origin` 和 `axis` 是否已经按 Star Arm 102-FL 实物核对。
2. `link6` 是否是厂家建议的 MoveIt Servo 末端 link。
3. 厂家是否有经过 MoveIt Servo 笛卡尔控制验证的 URDF、SRDF 和运动学配置。
4. 厂家测试这套模型时是否也会遇到状态 1/3 频繁切换。
5. 是否有厂家验证过的非奇异起始姿态或适用于这款小型机械臂的 Servo 配置。
