# Success Apple Catch

## 1. 本文档对应的成功结果

2026-08-08，RM65 + RealSense Eye-in-Hand + ROS 2 Jazzy + MoveIt 完成了第一轮真实单苹果低速抓取。

本轮结果：

- 固定鸟瞰位识别一颗桌面苹果；
- 自动读取苹果在 `base_link` 下的三维中心；
- 真实执行 `SCAN → PREGRASP → VERIFY`；
- 在 VERIFY 等待人工确认；
- 真实执行 `VERIFY → GRASP`；
- RS485 夹爪闭合；
- 等待 1 秒后直线上抬 0.20 m；
- 苹果被实际夹住并成功抬起；
- 到达后程序立即停止，没有放置、循环识别或二次抓取。

成功日志：

`results/single_apple_grasp_20260808_153839.json`（本地运行证据，未提交）

日志最终状态：

```text
status: PASS_SINGLE_ROUND_COMPLETE
single_round: true
gripper_close_commanded: true
lift_completed: true
```

---

## 2. 为什么这次能够跑通

YOLO 一直能够识别苹果，但 YOLO 只解决了“图像中哪里有苹果”，不能单独完成机械臂抓取。

本次真正跑通的完整数据链是：

```text
YOLO 找到苹果二维区域
        ↓
RealSense 对齐深度得到苹果表面三维点
        ↓
按照苹果球体近似估算苹果中心
        ↓
手眼 TF 把相机坐标转换到 base_link
        ↓
得到实时 apple_center(base_link)
        ↓
MoveIt 让 tcp_link 到达抓取目标
        ↓
RS485 夹爪闭合
        ↓
沿原方向上抬 0.20 m
```

本次成功不是某一个环节单独起作用，而是以下环节同时正确：

1. 机械臂驱动、关节状态和 MoveIt 控制器正常；
2. 机械臂 TF 树与 RealSense TF 树已经连接；
3. 手眼标定外参方向正确并在启动时发布；
4. MoveIt 使用真实夹持中心 `tcp_link`，不再直接使用 Link6；
5. 苹果三维中心由实时图像与深度计算，不是代码写死；
6. 使用固定鸟瞰位，避免观察姿态变化造成额外误差；
7. PREGRASP、VERIFY、GRASP、LIFT 全部先做 IK、碰撞和路径检查；
8. VERIFY 到位后必须人工确认；
9. 夹爪使用正确的工具端 RS485/Modbus 协议；
10. 完整流程禁止自动机械回零，避免夹爪意外先闭合。

---

## 3. 已确定的 TCP

`tcp_link` 是两指实际夹持苹果的工具中心点，不是相机位置，也不是 Link6 法兰中心。

固定变换：

```text
Link6 → tcp_link
xyz = [0, 0, 0.138] m
rpy = [0, 0, 0]
```

即 TCP 位于 Link6 正 Z 轴方向 13.8 cm。

如果直接把苹果坐标作为 Link6 的目标，MoveIt 会让法兰中心到达苹果，产生约 138 mm 的工具几何误差。现在 MoveIt 使用 `tcp_link` 求 IK，计算的是：

> Link6 应该到达哪里，才能让距离它 138 mm 的两指夹持中心到达苹果。

注意：0.138 m 只能补偿一次。控制器工具坐标和 ROS/MoveIt TCP 不能同时重复加入这段偏移。

---

## 4. 固定鸟瞰位

第一版抓取只使用一个固定 SCAN，不切换其他观察姿态。

```yaml
joint1:  2.510152891
joint2:  0.354521179
joint3:  0.647129746
joint4:  0.061472860
joint5:  2.045541343
joint6: -0.071433319
```

固定鸟瞰位的作用：

- 相机视野和距离稳定；
- 苹果深度估算更稳定；
- 减少 Eye-in-Hand 手眼残余误差；
- 每次任务都从同一条件开始；
- 便于重复检查和复现实验。

---

## 5. 本次成功使用的参数

### 5.1 苹果和目标点

本轮读取到的苹果中心：

```text
apple_center(base_link)
[0.3390548, -0.2263290, -0.0633634] m
```

10 帧采样的 XYZ 峰峰值：

```text
[0.519, 0.753, 0.092] mm
```

目标点：

| 点位 | base_link 坐标（m） | 含义 |
|---|---|---|
| PREGRASP | [0.3390548, -0.2263290, 0.0366366] | 苹果中心上方 0.10 m |
| VERIFY | [0.3390548, -0.2263290, -0.0333634] | 苹果中心上方 0.03 m |
| GRASP | [0.3390548, -0.2263290, -0.0533634] | 苹果中心上方 0.01 m |
| LIFT | [0.3390548, -0.2263290, 0.1466366] | 从 GRASP 向上 0.20 m |

GRASP 暂时不是精确苹果中心，而是中心上方 10 mm。原因是精确中心与膨胀后的桌面保护碰撞体约有 5.9 mm 冲突；中心上方 10 mm 已通过碰撞检查并成功完成真实抓取。

### 5.2 运动参数

```text
SCAN → PREGRASP:
  velocity scaling     = 0.05
  acceleration scaling = 0.05

PREGRASP → VERIFY:
  velocity scaling     = 0.05
  acceleration scaling = 0.05
  TCP speed cap        = 0.010 m/s

VERIFY → GRASP:
  velocity scaling     = 0.03
  acceleration scaling = 0.03
  TCP speed cap        = 0.006 m/s

GRASP → LIFT:
  velocity scaling     = 0.03
  acceleration scaling = 0.03
  TCP speed cap        = 0.010 m/s
```

所有笛卡尔路径都必须满足：

```text
fraction >= 0.999
```

本轮三段笛卡尔路径实际均为：

```text
fraction = 1.0000
```

### 5.3 夹爪参数

夹爪使用自定义工具端 RS485 / Modbus RTU，不使用睿尔曼标准夹爪话题。

```text
RS485 port       = 1
baudrate         = 9600
tool voltage     = 24 V
device address   = 1
initial opening  = 70 mm
grasp target     = 25 mm
wait after close = 1 s
```

`25 mm` 是位置命令，不是夹持力数值。当前夹爪接口没有力或电流反馈。

重要：完整抓取流程只调用通信配置，不调用机械回零。

```python
gripper.configure(...)
gripper.set_opening_mm(..., 70.0)
```

禁止在机械臂靠近苹果时调用：

```python
gripper.initialize(...)
```

因为 `initialize()` 包含机械回零，回零过程会让夹爪先闭合。本次调试中它曾意外夹起苹果，现已从完整抓取流程移除。

### 5.4 视觉参数

当前使用：

```text
model: yolo11n-seg.pt
class: apple
minimum confidence: 0.25
```

本轮苹果顶视图置信度大约为 0.27～0.29。检测框位置正确，因此恢复苹果专用的 0.25 阈值。

低阈值不能单独触发抓取，还必须同时满足：

- 画面中人工确认只有一颗苹果；
- 连续 10 帧三维坐标稳定；
- 深度像素有效；
- 苹果半径在允许范围；
- TF 转换成功；
- IK 成功；
- 碰撞检查成功；
- 笛卡尔路径完整；
- VERIFY 人工确认。

---

## 6. 每次启动流程

### 6.0 推荐：抓取与放回一条命令

需要在同一轮中完成抓取、上抬、人工确认放回、释放和返回 SCAN 时，使用：

```bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --execute --operator-confirmed --return-after-lift
```

该模式包含两个关键人工 Enter：

1. VERIFY：确认 TCP 位于两指中间正上方，再下降和闭爪；
2. LIFT：确认苹果夹持稳定、原位置可以放回，再下降释放。

IK、碰撞和全部路径预检通过后，程序自动执行到 VERIFY，不再要求输入开始口令。

如果明确希望 VERIFY 到位后自动下降、闭爪并上抬，必须主动增加
`--autonomous`：

```bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --execute --operator-confirmed --autonomous --return-after-lift
```

`--autonomous` 只取消 VERIFY 的 Enter；带有
`--return-after-lift` 时，LIFT 后的“确认放回”Enter 仍然保留。未提供
`--autonomous` 时保持原来的人工确认行为。

抓取候选采用长期保留的分阶段搜索：

1. 保持完全垂直，依次检查相对基准 yaw 的
   `0°/90°/180°/270°`；
2. 垂直阶段全部失败才扩展到 pitch `+10°/-10°`；
3. 第二阶段也全部失败才扩展到 pitch `+20°/-20°`。

每个姿态使用四个确定性关节种子，共享 80 ms 的纯 IK 求解预算。纯 IK
关闭碰撞检查，得到解后再调用规划场景状态检查，因此日志可以区分
`NO_IK` 与 `COLLISION`。通过者按当前关节变化、关节限位余量、
yaw 偏移和 pitch 大小排序；只有排名第一的候选获得一次最多 5 秒的完整
OMPL 与笛卡尔路径预检。拒绝时终端仍只打印 `REJECT`，详细原因保存在
本轮 JSON 日志中。

完整顺序：

```text
SCAN → PREGRASP → VERIFY
→ [人工 Enter]
→ GRASP → 闭爪 → LIFT
→ [人工 Enter：确认现在需要放回]
→ RELEASE → 打开 70 mm
→ PREGRASP → SCAN → 停止
```

不带 `--return-after-lift` 时，仍保持原成功基线：抓取上抬后立即停止，不自动放回。

### 6.1 启动完整 ROS 2、MoveIt、RealSense、TF、RViz 和苹果定位

新终端：

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash

ros2 launch $APPLE_CATCH_ROOT/apple_pick_v2/apple_hand_eye_all.launch.py \
  allow_trajectory_execution:=true
```

等待以下节点和话题稳定：

```bash
ros2 node list | grep -E 'rm_driver|rm_control|move_group|robot_state_publisher|apple_center'
ros2 topic info /joint_states
ros2 topic echo --once /apple_pick_v2/apple_diagnostics
```

确认：

- RViz 能看到机械臂；
- RViz 能看到图像和点云；
- 苹果 Marker 位于真实苹果附近；
- `base_link → tcp_link` TF 存在；
- 画面中只有一颗苹果；
- 工作区无人、无遮挡；
- 急停可用；
- 夹爪附近没有物体妨碍开到 70 mm。

### 6.2 执行单轮抓取

另一个新终端：

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash

$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --execute --operator-confirmed
```

程序首先：

1. 配置 RS485 和 24 V；
2. 不做机械回零；
3. 命令夹爪打开到配置文件当前值（现为 80 mm）；
4. 回到固定鸟瞰位；
5. 连续读取 10 帧苹果坐标；
6. 打印 APPLE、PREGRASP、VERIFY、GRASP、LIFT；
7. 检查当前状态、IK、碰撞和全部路径。

预检通过后，机械臂自动真实执行：

```text
SCAN → PREGRASP → VERIFY
```

到达 VERIFY 后，终端显示：

```text
请人工确认 TCP 位于两指中间正上方；确认后按 Enter
```

此时必须现场检查：

- 苹果在两指中间；
- 没有明显 X/Y 横向偏差；
- 再下降约 20 mm 不会碰桌面；
- 夹爪路径无遮挡；
- 急停可用。

只有确认安全才按 Enter。

按 Enter 后程序执行：

```text
VERIFY
  → 低速直线到 GRASP
  → 检查 TCP 到位误差
  → 夹爪闭合到 25 mm
  → 等待 1 秒
  → 低速直线上抬 0.20 m
  → 停止程序
```

程序不会去放置点，不会循环识别，也不会再次抓取。

---

## 7. 本轮实际精度

```text
PREGRASP 稳定到位误差: 2.87 mm
VERIFY 稳定到位误差:   0.84 mm
GRASP 稳定到位误差:    0.93 mm
LIFT 稳定到位误差:     0.49 mm
```

最终现场确认苹果被实际夹住并成功抬起。

---

## 8. 失败即停规则

以下任一情况出现时，程序必须停止后续动作，并且在 GRASP 之前不得闭爪：

- 苹果样本不足；
- 苹果三维坐标波动超限；
- TF 不连通；
- 苹果置信度或半径异常；
- MoveIt 接口不可用；
- 当前机械臂状态碰撞；
- PREGRASP、VERIFY、GRASP 或 LIFT 的 IK 失败；
- 碰撞检查失败；
- 笛卡尔路径 fraction 不足；
- 轨迹包含异常关节跳变；
- 到位误差超过限制；
- VERIFY 未得到人工确认；
- 操作者按 Ctrl-C；
- 任意真实轨迹执行失败。

闭爪之前发生失败时，夹爪必须保持打开。

---

## 9. 已解决的关键问题

### 问题一：机械臂和相机是两棵 TF 树

原状态：

```text
base_link → ... → Link6

camera_link → ... → camera_color_optical_frame
```

两者没有连接，苹果无法转换到 `base_link`。

解决：

```text
base_link → ... → Link6 → camera_link → ... → camera_color_optical_frame
```

使用已有手眼标定外参，在启动时发布 Link6 与相机之间的固定 TF。

### 问题二：缺少真实 TCP

原先若使用 Link6 抓苹果，会忽略夹爪的 138 mm 工具长度。

解决：

```text
Link6 → tcp_link = [0, 0, 0.138] m
```

MoveIt 使用 `tcp_link` 作为 IK 和目标 link。

### 问题三：夹爪初始化意外闭合

原因：

`initialize()` 同时包含通信初始化和机械回零，机械回零会先闭合夹爪。

解决：

- 将通信配置拆分为 `configure()`；
- 完整抓取只调用 `configure()`；
- 已标定夹爪不在抓取流程中自动回零；
- 程序主动打开到 70 mm。

### 问题四：固定鸟瞰下苹果置信度临界

当前顶视苹果的 YOLO 置信度约 0.28，使用 0.40 或 0.45 会导致长时间没有样本。

解决：

- 使用苹果专用阈值 0.25；
- 同时保留 10 帧稳定性、深度、半径、TF、IK、碰撞和人工确认保护。

### 问题五：精确苹果中心与桌面保护体冲突

精确中心目标与膨胀后的桌面碰撞体约冲突 5.9 mm。

解决：

- 第一版 GRASP 使用苹果中心上方 10 mm；
- 该目标已通过碰撞检查；
- 真机抓取成功。

---

## 10. 当前适用范围与后续工作

当前已经验证的是：

> 一颗苹果、固定鸟瞰位、固定垂直抓取姿态、人工确认后执行的一轮真实抓取。

暂时不要直接扩展为任意姿态或多苹果自动连续抓取。

此前五姿态手眼一致性测试结果：

```text
最大姿态间距离: 26.07 mm
```

因此当前成功也依赖固定鸟瞰位。后续建议依次完成：

1. 重新优化多姿态手眼标定一致性；
2. 增加 YOLO 苹果候选数量发布；
3. 支持多苹果目标选择；
4. 增加夹爪位置/电流/夹持状态反馈；
5. 建立安全放置点；
6. 验证放置流程后再考虑循环抓取。

在这些工作完成之前，保留当前成功程序作为不可随意改动的基线。

### 2026-08-08 大关节运动阈值调整

`single_apple_full_grasp.yaml` 中的单段关节运动检查阈值已由
`2.60 rad` 调整为 `5.00 rad`。该值允许明显的腕部解分支切换；
MoveIt 的关节硬限位与碰撞检查仍然生效，但相机和夹爪外部线缆没有建模。

### 2026-08-08 IK 稳定性修正

固定垂直姿态在部分苹果位置存在多组翻腕解，单次 `/compute_ik` 会偶发返回
`-31`。2026-08-08 当时的实现会显式使用实时六关节状态作为 IK 种子，有限尝试 12 次，
从通过碰撞检查的解中选择相对当前姿态最大关节变化最小的一组。12 次均失败
才停止流程；不会无限重试，也不会因为 IK 重试而执行机械臂运动。

进一步真机诊断确认：独立单点 IK 在腕部多解区域可能误报 `-31`，而 MoveIt
整段规划能找到关节变化很小的连续分支。完整抓取流程因此不再用四个相互独立
的 IK 结果作为门槛，而是依次传递真实轨迹终点：

```text
SCAN→PREGRASP 轨迹终点
→ PREGRASP→VERIFY 轨迹终点
→ VERIFY→GRASP 轨迹终点
→ GRASP→LIFT 轨迹终点
```

每段仍要求碰撞检查通过、笛卡尔 `fraction>=0.999`、关节变化和时间戳有效。

### 2026-08-10 候选规划器重构

旧的“0.7 秒快速 IK 后仍逐个 pitch 运行完整规划”已经移除。现行结构是：

```text
分阶段生成 yaw/pitch 姿态
→ 每个姿态四个种子共享 80 ms 纯 IK 预算
→ 对 IK 解独立进行规划场景碰撞检查
→ 当前关节变化/限位余量/姿态偏移评分
→ 仅排名第一的姿态运行一次最多 5 秒 OMPL
→ 连续传递轨迹终点检查 VERIFY/GRASP/LIFT/RELEASE
→ VERIFY 或 --autonomous 执行
```

对 2026-08-10 的旧失败点
`PREGRASP=[0.4855093,-0.1860989,0.041993]` 做了不执行运动的真实 MoveIt
服务回归：

- 完全垂直四个 yaw：均为 `NO_IK`；
- pitch `±10°`：均为 `NO_IK`；
- pitch `+20°/yaw 0°` 与 pitch `-20°/yaw 180°` 找到碰撞安全 IK；
- 排名第一候选的 `SCAN→PREGRASP` 和
  `PREGRASP→VERIFY` 通过；
- `VERIFY→GRASP` 仅得到约 `0.80～0.83` 的路径比例，低于
  `0.999`，因此程序正确安全拒绝，没有执行真机运动。

这证明该 XYZ 并非简单超出机械臂位置范围；限制出现在固定抓取姿态及最后
15 mm 接近路径的组合条件。新筛选器已经按预期工作，但这个特定点尚不能
标记为完整抓取可执行。

### 2026-08-10 VERIFY→GRASP 假碰撞修复

真实运行日志
`single_apple_grasp_20260810_133219.json` 显示第一名候选在
`VERIFY→GRASP` 只有 `fraction=0.8333`。进一步把纯 IK 与碰撞检查
分开后确认：

```text
纯 IK：PASS
碰撞：apple_test_table_guard <-> gripper_body_approx
```

当时的 `gripper_body_approx` 是覆盖整个夹爪的
`80×80×170 mm` 实心长方体，并不表示真实两指之间的空隙；桌面高度又由
`苹果中心 - 视觉半径` 推算后额外向上膨胀 5 mm。碰撞阈值扫描结果为：

- 额外膨胀 3～5 mm：简化夹爪与桌面碰撞；
- 额外膨胀 0～2 mm：当前姿态存在碰撞安全 IK；
- 因此保留 2 mm 保护量，没有关闭桌面碰撞检查。

同时，快速候选检查不再只检查 PREGRASP。每个候选的 80 ms 总预算现在由
`PREGRASP/VERIFY/GRASP/LIFT` 四点共享；任意一点无 IK 或发生碰撞，
候选立即被淘汰。评分采用四点中的最差代价，防止再次选中“高处可达、低处
不可达”的姿态。

使用本次真实苹果坐标进行的不执行运动回归结果：

```text
stage 0（垂直）：无四点完整候选
stage 1（±10°）：无四点完整候选
stage 2：yaw 0° / pitch +20° 与 yaw 180° / pitch -20° 通过
最终选择：yaw 0° / pitch +20°

SCAN→PREGRASP          PASS
PREGRASP→VERIFY        fraction=1.0000
VERIFY→GRASP           fraction=1.0000
GRASP→LIFT             fraction=1.0000
LIFT→RELEASE           fraction=1.0000
RELEASE→PREGRASP       fraction=1.0000
```

以上仅为真实 MoveIt 服务的 plan-only 回归，没有发送机械臂轨迹或夹爪
命令。由于该姿态包含 20° 倾角、桌面保护余量为 2 mm 且夹爪没有力反馈，
第一次真机执行仍需操作员针对该轮明确授权并保持急停可触及。

---

## 11. 相关文件

```text
$APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py
$APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.yaml
$APPLE_CATCH_ROOT/apple_pick_v2/fixed_scan_real_verify.py
$APPLE_CATCH_ROOT/apple_pick_v2/fixed_scan_real_verify.yaml
$APPLE_CATCH_ROOT/apple_pick_v2/apple_center_localizer_v2.py
$APPLE_CATCH_ROOT/apple_pick_v2/apple_hand_eye_all.launch.py
$APPLE_CATCH_ROOT/modbus_gripper_ros.py
$APPLE_CATCH_ROOT/apple_pick_v2/results/single_apple_grasp_20260808_153839.json
```

这套文件和本成功日志共同构成当前“单苹果固定鸟瞰真实抓取”的成功基线。

---

## 12. 2026-08-10 固定 SCAN XY 尺度模型（已连续现场确认）

加入模型前的 Git 基线为：

```text
ddb11c8 feat: add guarded visual TCP offset measurement
```

八个固定 SCAN、受控向下姿态测量点的全量相似变换拟合为：

```text
scale = 0.9249983311
rotation = +2.2829447234 deg
translation = [+0.01199984868, -0.00628986080] m
```

模型只修正 `apple_center` 的 XY，Z 不变，原有抓取高度
`GRASP.z = apple.z + 0.015 m` 不变。配置带有有效工作区和最大 50 mm
校正量保护；原始中心、校正中心和本轮校正量全部写入 JSON 日志。

该模型的八点拟合 RMSE 为 6.41 mm、最大残差为 9.63 mm。模型接入后已在
不同苹果位置连续完成四轮真实抓取，其中后三轮包含完整自动放回与返回 SCAN：

```text
single_apple_grasp_20260810_154238.json  PASS_SINGLE_ROUND_COMPLETE
single_apple_grasp_20260810_154404.json  PASS_PICK_RETURN_COMPLETE
single_apple_grasp_20260810_154510.json  PASS_PICK_RETURN_COMPLETE
single_apple_grasp_20260810_154610.json  PASS_PICK_RETURN_COMPLETE
```

四轮使用的原始→校正 XY 变化不同，证明程序不是使用固定平移，而是按苹果
位置执行尺度/旋转变换。后三轮操作员现场确认抓取效果准确，并完成抓取、上抬、
放回、开爪和返回固定 SCAN。因此该模型现在属于新的成功基线。

如需诊断对比，可临时关闭并恢复旧行为：

```bash
--disable-visual-xy-correction
```

保留不带 `--autonomous` 的人工 VERIFY 模式作为后续异常诊断入口。

正式抓取主程序的实时 MoveIt 全路径预检（不配置夹爪、不执行轨迹）：

```bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --preflight-only --operator-confirmed
```

### 单轮全自动抓取并放回

尺度模型完成连续现场验证后，新增显式全自动开关：

```bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --execute --operator-confirmed --fully-autonomous
```

`--fully-autonomous` 只授权一轮：

```text
固定 SCAN 识别唯一苹果
→ 自动规划并执行 PREGRASP
→ 自动执行 VERIFY
→ 自动执行 GRASP 并闭爪
→ 自动上抬 LIFT
→ 自动回原 GRASP/RELEASE
→ 自动开爪
→ 自动撤离到 PREGRASP
→ 自动返回固定 SCAN
→ 停止，不循环识别
```

该参数仍必须与 `--execute --operator-confirmed` 同时提供。所有往返路径必须
先通过 IK、碰撞和 `fraction=1.0` 预检；任一步失败即停。原有不加
`--fully-autonomous` 的 VERIFY/LIFT 人工确认模式继续保留。
