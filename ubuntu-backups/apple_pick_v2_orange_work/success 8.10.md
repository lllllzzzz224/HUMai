# Success 8.10：视觉误差建模与单苹果全自动抓取成功记录

日期：2026-08-10
平台：RM65 + RealSense Eye-in-Hand + ROS 2 Jazzy + MoveIt + RS485 两指夹爪

## 1. 今天最终实现的结果

今天已经在真机上完整实现并现场验证：

```text
固定 SCAN 鸟瞰位
  → 识别唯一苹果并读取 base_link 下的三维中心
  → 对视觉 XY 坐标应用八点拟合的尺度/旋转/平移校正
  → 自动生成 PREGRASP、VERIFY、GRASP、LIFT
  → 快速筛选抓取姿态并完成全部路径预检
  → 真实执行到 PREGRASP 和 VERIFY
  → 自动下降到 GRASP 并闭合夹爪
  → 自动上抬至 LIFT
  → 自动沿原路径放回苹果并打开夹爪
  → 自动撤离并返回固定 SCAN
  → 单轮停止，不循环抓取
```

这套完整流程已经全部真机跑通，不只是 RViz 预览或 plan-only。加入视觉 XY 尺度模型后，现场抓取位置明显改善，并完成了多个不同苹果位置的成功抓取和放回。

## 2. 成功前已经具备的基础条件

### 2.1 TF 与 TCP

机械臂 TF 树和 RealSense TF 树已经由手眼外参连接。MoveIt 的工具目标使用真实夹持中心 `tcp_link`，不是相机，也不是 Link6 法兰原点。

```text
Link6 → tcp_link
xyz = [0, 0, 0.138] m
rpy = [0, 0, 0]
```

这 138 mm 表示 Link6 到两指夹住苹果时的中心。它只能生效一次，不能再在控制器 `Arm_Tip` 和 ROS/MoveIt 中重复补偿。

### 2.2 固定 SCAN 鸟瞰位

所有视觉采样和每轮抓取都从同一固定关节位开始：

```yaml
joint1:  2.510152891
joint2:  0.354521179
joint3:  0.647129746
joint4:  0.061472860
joint5:  2.045541343
joint6: -0.071433319
```

固定 SCAN 的作用是保持 Eye-in-Hand 相机的距离、角度和视野一致，使误差模型可以重复使用。

### 2.3 抓取点和运动参数

当前目标定义：

| 点位 | 定义 |
|---|---|
| PREGRASP | 校正后苹果中心上方 0.10 m |
| VERIFY | 校正后苹果中心上方 0.03 m |
| GRASP | 校正后苹果中心上方 0.015 m |
| LIFT | 从 GRASP 沿 base_link 正 Z 上抬 0.20 m |

笛卡尔抓取段速度和加速度缩放为 `0.03`，路径要求 `fraction >= 0.999`。当前桌面安全膨胀为 2 mm，既保留碰撞保护，又避免过大的简化碰撞体误挡真实可行路径。

### 2.4 夹爪

夹爪使用工具端 RS485 / Modbus：

```text
端口：1
波特率：9600
电压：24 V
从站地址：1
打开目标：80 mm
闭合目标：20 mm
闭合后稳定等待：1 s
```

`20 mm` 是位置目标，不是夹持力。抓取程序只恢复通信并发送开合位置，绝不调用机械回零 `initialize()`；机械回零曾导致夹爪先闭合，因此不能放进自动抓取流程。

## 3. 今天先解决的问题：视觉坐标存在随位置变化的偏差

此前抓取路径本身能够执行，但 TCP 有时落在苹果旁边。为区分 MoveIt 规划误差和视觉定位误差，采用了以下测量定义：

```text
Δ = 人工对准后的 tcp_link(base_link)
    - 固定 SCAN 下的视觉 apple_center(base_link)
```

如果 MoveIt 能把 TCP 稳定送到目标，而不同桌面位置的 Δ 又呈规律变化，那么问题不是简单的固定平移，也不能只加一个常数补偿。

## 4. 为测量偏差新增的受保护采集流程

测量程序：

```text
$APPLE_CATCH_ROOT/apple_pick_v2/measure_visual_tcp_offset.py
```

每轮测量执行：

1. 自动打开夹爪到 80 mm，全程不发送闭爪命令；
2. 自动安全返回同一个固定 SCAN；
3. 采集 10 个新的 `apple_center(base_link)` 样本并冻结中位数；
4. 保持苹果不动，由操作者用示教器只调整 XYZ，使两指中心对准真实苹果中心；
5. 采集 20 个 `base_link → tcp_link` TF 样本；
6. 计算视觉中心、真实 TCP 和二者偏差；
7. 保存完整 JSON/CSV；
8. 尝试安全上撤并回到固定 SCAN，准备下一轮。

为保证采集有效，还做了这些保护：

- 每次视觉采样前必须经过固定 SCAN 门禁；
- 夹爪只打开，不回零、不闭合；
- TCP 姿态门禁最终改为检查工具正 Z 与 base_link 负 Z 的夹角；
- 忽略不影响垂直性的 yaw，允许垂直倾角误差 6°；
- 自动回位加入状态同步和 100/80/60/50 mm 分级安全上撤；
- 已经处于高位或接近 SCAN 时，直接做小范围关节回位，避免强行上撤失败。

测量启动命令：

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash

$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/measure_visual_tcp_offset.py \
  --auto-scan --label vertical_test_1
```

## 5. 今天采集的数据

最终用于模型的是 8 组受控测量。下面是当时的本地日志路径；现场运行日志不纳入公开仓库：

```text
results/visual_tcp_offset/offset_20260810_145937/result.json
results/visual_tcp_offset/offset_20260810_150231/result.json
results/visual_tcp_offset/offset_20260810_150638/result.json
results/visual_tcp_offset/offset_20260810_150745/result.json
results/visual_tcp_offset/offset_20260810_152208/result.json
results/visual_tcp_offset/offset_20260810_152326/result.json
results/visual_tcp_offset/offset_20260810_152446/result.json
results/visual_tcp_offset/offset_20260810_152710/result.json
```

每组包含 10 帧视觉中心、20 帧 TCP、关节位置、姿态门禁结果和偏差。视觉样本自身通常只有约毫米级或更小波动，TCP 静止采样波动约百分之一毫米量级。因此反复出现的厘米级 XY 差异不是机械臂到位抖动，而是视觉坐标到真实夹持中心之间的系统性位置误差。

早期有 3 组手工姿态差异达到约 17°～24°，不满足同姿态对比条件，保留原始数据但没有用于最终模型。

## 6. 八点视觉 XY 尺度模型

数据说明单一固定偏移不足，因此采用 base_link 平面内的二维相似变换：

```text
x' = s × (cosθ × x - sinθ × y) + tx
y' = s × (sinθ × x + cosθ × y) + ty
z' = z
```

拟合参数：

```yaml
scale: 0.9249983311376341
rotation_deg: 2.2829447233570446
translation_xy_m: [0.01199984868, -0.00628986080]
fit_point_count: 8
fit_rmse_mm: 6.411567017694393
fit_max_residual_mm: 9.626213316333969
```

适用范围和保护：

```yaml
valid_visual_x_m: [0.22, 0.50]
valid_visual_y_m: [-0.33, 0.00]
max_correction_m: 0.050
```

模型只校正 X/Y，Z 保持原视觉结果，继续使用已经验证的 `GRASP = 苹果中心上方 15 mm`。模型只适用于当前固定 SCAN、当前相机安装和当前工作区；相机位置、手眼外参、TCP 或鸟瞰位改变后必须重新采集。

## 7. 模型如何接入抓取流程

配置位于：

```text
$APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.yaml
```

执行代码位于：

```text
$APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py
```

程序先保留原始视觉中心，再只应用一次模型：

```text
raw apple_center
  → apply_visual_xy_correction()
  → corrected apple_center
  → PREGRASP / VERIFY / GRASP / LIFT / 桌面场景
```

每轮 JSON 都保存原始坐标、校正后坐标、校正量和模型参数，便于回溯。临时排查时可用以下参数关闭模型：

```text
--disable-visual-xy-correction
```

当前模型已经过真机成功验证，应保持启用。

## 8. 规划算法的最终结构

程序不再为每个倾角等待完整规划，而是：

```text
目标苹果
  → 生成少量 yaw/pitch 抓取候选
  → 每个路点进行 80 ms 多种子 IK 快筛
  → 独立碰撞检查
  → 按关节变化、限位余量和倾角排序
  → 只给当前阶段最佳候选做 5 s 完整规划
  → 预检 SCAN→PREGRASP→VERIFY→GRASP→LIFT
  → 如需放回，再预检 LIFT→RELEASE→PREGRASP→SCAN
  → 全部通过后才允许真机执行
```

候选分阶段搜索：

```text
stage 0: pitch 0°，yaw 偏移 0°/90°/180°/270°
stage 1: pitch +10°/-10°
stage 2: pitch +20°/-20°
```

前一阶段存在可行解时不会继续扩大倾角。今天的全自动真机成功既包含 `pitch 0°`，也包含工作区边缘自动选择 `pitch +20°` 的情况。

## 9. 今天的真机成功证据

视觉模型接入后，以下日志完成了真实运动：

| 日志 | 结果 | 说明 |
|---|---|---|
| `single_apple_grasp_20260810_154238.json` | `PASS_SINGLE_ROUND_COMPLETE` | 校正后抓取并上抬 |
| `single_apple_grasp_20260810_154404.json` | `PASS_PICK_RETURN_COMPLETE` | 抓取、放回、开爪、回 SCAN |
| `single_apple_grasp_20260810_154510.json` | `PASS_PICK_RETURN_COMPLETE` | 抓取、放回、开爪、回 SCAN |
| `single_apple_grasp_20260810_154610.json` | `PASS_PICK_RETURN_COMPLETE` | 抓取、放回、开爪、回 SCAN |
| `single_apple_grasp_20260810_155351.json` | `PASS_PICK_RETURN_COMPLETE` | 全自动；pitch 0° |
| `single_apple_grasp_20260810_155525.json` | `PASS_PICK_RETURN_COMPLETE` | 全自动；自动选择 pitch +20° |

最后两轮日志同时记录：

```text
autonomous_requested: true
fully_autonomous_requested: true
gripper_close_commanded: true
lift_completed: true
gripper_reopened: true
returned_to_scan: true
```

其中两轮全自动视觉校正示例：

```text
15:53:51
raw       = [0.4035932, -0.1632677, -0.0620265] m
corrected = [0.3910425, -0.1423212, -0.0620265] m
selected  = yaw +0°, pitch 0°

15:55:25
raw       = [0.4234101, -0.3259713, -0.0556060] m
corrected = [0.4153536, -0.2919722, -0.0556060] m
selected  = yaw +0°, pitch +20°
```

操作者现场确认加入模型后的实际抓取效果准确，完整自动抓取和放回均已真机实现。

## 10. 当前完整启动命令

### 10.1 启动 ROS 2、机械臂、MoveIt、RealSense、TF、RViz 和苹果定位

终端 1：

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash

ros2 launch $APPLE_CATCH_ROOT/apple_pick_v2/apple_hand_eye_all.launch.py \
  allow_trajectory_execution:=true
```

等待机械臂模型、相机图像、点云和苹果 Marker 稳定。画面中只放一颗苹果，确认急停可用。

### 10.2 推荐：单轮全自动抓取、放回并回 SCAN

终端 2：

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash

$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --execute --operator-confirmed --fully-autonomous
```

`--fully-autonomous` 会自动启用 VERIFY 后继续抓取和 LIFT 后自动放回，不需要再额外添加 `--autonomous` 或 `--return-after-lift`。它只执行一轮，完成后回到 SCAN 并停止，不会自动寻找第二颗苹果。

### 10.3 保留的人工确认模式

VERIFY 和放回前都要求 Enter：

```bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --execute --operator-confirmed --return-after-lift
```

VERIFY 后自动抓取，但放回前保留 Enter：

```bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --execute --operator-confirmed --autonomous --return-after-lift
```

只做实时视觉和完整路径预检，不控制夹爪、不执行轨迹：

```bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --preflight-only --operator-confirmed --return-after-lift
```

## 11. 当前 PASS 的准确含义

当前 `PASS_PICK_RETURN_COMPLETE` 能证明：

- 苹果视觉采样和坐标门禁通过；
- 视觉 XY 模型已应用；
- IK、碰撞和整条轨迹预检通过；
- 真实运动指令执行成功并到达目标；
- 夹爪闭合位置命令已被接口接受；
- 机械臂到达 LIFT；
- 放回、打开夹爪、撤离和返回 SCAN 已执行成功。

但代码目前还不能仅靠日志证明“苹果此刻一定夹在两指之间”。夹爪接口没有可靠的力、电流或最终开度反馈，1 秒等待只是闭爪后的稳定时间，不是抓取成功判据。当前自动放回由 `--fully-autonomous` 的明确授权和流程执行状态触发，不是依靠相机长时间看不到苹果来触发。

## 12. 下一阶段保留：更可靠的物理抓取成功判定

后续不要只使用“原位置没有苹果”，因为相机移动、遮挡、出视野或检测失败都会造成假阴性。至少组合以下四个条件：

```text
夹爪闭合命令成功
  +
机械臂实际到达 LIFT
  +
原桌面苹果位置连续 N 帧没有苹果
  +
夹爪两指 ROI 连续 N 帧检测到苹果或近距离物体
```

建议状态机：

```text
GRASP 闭爪
  → 到达 LIFT
  → 同时观察原桌面 ROI 和夹爪 ROI
  → 连续多帧满足两个视觉条件
  → GRASP_CONFIRMED
  → 才进入自动放回

否则
  → GRASP_UNCERTAIN
  → 停在安全 LIFT，保持夹爪状态并等待人工处理
```

以后若能读取夹爪实际开度、电流或夹持力，应再加入硬件反馈。该融合判定是下一阶段增强项，今天尚未写入自动执行门禁；今天的全自动流程本身已经真机跑通。

## 13. 关键文件和版本存档

主要文件：

```text
apple_pick_v2/apple_hand_eye_all.launch.py
apple_pick_v2/apple_center_localizer_v2.py
apple_pick_v2/fixed_scan_real_verify.py
apple_pick_v2/measure_visual_tcp_offset.py
apple_pick_v2/single_apple_full_grasp.py
apple_pick_v2/single_apple_full_grasp.yaml
apple_pick_v2/success apple catch.md
apple_pick_v2/success 8.10.md
```

今天视觉尺度模型的 Git 存档：

```text
a66388f feat: retain validated visual XY scale correction
```

相关前置存档：

```text
ddb11c8 feat: add guarded visual TCP offset measurement
774ac38 fix: validate full grasp waypoints before planning
f7fe6e4 snapshot: guarded single-apple grasp baseline before path fix
```

## 14. 明天继续前的检查清单

- 相机、夹爪或机械臂底座没有移动；
- `Link6 → tcp_link = 0.138 m` 没有重复补偿；
- 夹爪可以打开到 80 mm，程序不会调用机械回零；
- 画面中只有一颗苹果；
- 苹果位于尺度模型有效工作区；
- RViz 中机械臂、点云和苹果 Marker 正常；
- `/joint_states`、`apple_center` 和 MoveIt 服务持续可用；
- 首轮低速执行时操作者在急停旁观察；
- 每轮完成回到 SCAN 后，再移动苹果并启动下一轮。

## 15. 下一阶段：从桌面苹果迁移到树上苹果的动态点云避障

### 15.1 当前点云和 MoveIt 场景的真实关系

2026-08-10 运行时检查结果：

```text
PointCloud2:
  topic: /camera/camera/depth/color/points
  publisher: /camera/camera
  subscriber: /rviz
```

当前点云只有 RViz 订阅。RViz 中虽然能够看到桌面、平台和苹果的三维点，但这些点目前只是显示数据，没有进入 MoveIt 的碰撞世界，`move_group` 没有使用它们进行避障。

当前 MoveIt 场景中的桌面也不是由点云识别得到的，而是代码采用了以下临时假设：

```python
raw_table_top = apple_center_z - apple_radius
```

然后把这个高度扩展为配置中的大长方体：

```yaml
table_size_xyz_m: [0.75, 1.20, 0.10]
```

这个方案只适用于“苹果直接放在同一张平整桌面上”。苹果放在高约 24 cm 的小平台后，程序把小平台顶面错误扩展为整张高桌面，最终产生：

```text
apple_test_table_guard <-> gripper_body_approx
```

碰撞。该次日志中 `pitch +20°` 的 PREGRASP 和 VERIFY 已找到 IK，GRASP 也有数值 IK，真正挡住它的是错误生成的大平面，而不是机械臂绝对无法到达。

### 15.2 固定桌面也不是最终方案

把原桌面高度改成固定参数可以解决实验室中的小平台问题，但仍然不是最终通用架构。后续目标是树上苹果抓取，现场可能只有：

- 树干；
- 树枝；
- 叶片；
- 多个不同高度和深度的苹果；
- 临时支撑、果筐或其他障碍物；
- 完全没有桌面。

因此最终不能要求环境中一定存在固定桌面，也不能从目标苹果反推整个支撑面。正确的数据关系应当是：

```text
苹果三维坐标
    只负责生成抓取目标

环境点云
    独立负责生成动态障碍物

URDF / SRDF
    负责机械臂自身碰撞

可选静态物体
    只在已知场景中按需加载
```

推荐场景配置思想：

```yaml
environment:
  mode: dynamic_pointcloud
  static_objects: []       # 树上模式可以为空
  use_octomap: true
  remove_target_apple: true
```

实验室桌面以后只能作为可选静态物体，而不是抓取程序的必备前提。

### 15.3 目标架构

```text
RealSense 原始 PointCloud2
        ↓
转换到 base_link
        ↓
限制在机械臂工作空间
        ↓
体素降采样和离群点过滤
        ↓
机械臂、夹爪、相机自身点云过滤
        ↓
移除当前目标苹果的点云
        ↓
生成或更新 MoveIt OctoMap
        ↓
树枝、树干、平台和临时物体成为真实障碍
        ↓
MoveIt 在动态三维环境中筛选抓取姿态和路径
```

这样苹果高度变化时，只改变苹果的目标坐标，不会生成任何无限平面或大桌面。小平台、树枝和树干只按点云中实际观测到的范围成为障碍物。

### 15.4 原始点云不能直接送进 MoveIt

需要先过滤，原因如下：

1. 原始点云可能包含机械臂、夹爪和相机自身，不过滤会产生自碰撞假象；
2. 目标苹果也在点云中，不移除就会导致 GRASP 与苹果障碍物碰撞；
3. Eye-in-Hand 相机运动后，旧视角留下的体素必须清除或设置衰减；
4. 深度边缘、反光和遮挡会产生漂浮噪点；
5. 树叶会形成大量稀疏点，需要区分硬树枝和可接触叶片；
6. TF 与点云时间不一致会把障碍物投影到错误位置。

安全原则：

- 保留 MoveIt 的机械臂自身碰撞检查；
- 保留夹爪和 RealSense 附加碰撞体；
- 不直接关闭碰撞检查；
- 不把目标苹果作为普通障碍物；
- 每次从稳定观察位重新生成或刷新环境；
- 点云过期、TF 不可用或过滤失败时，禁止真实执行。

### 15.5 树上抓取还需要改变目标点生成方式

当前桌面抓取采用：

```text
PREGRASP = 苹果中心沿 base_link +Z 上方 0.10 m
GRASP    = 苹果中心附近
LIFT     = 沿 base_link +Z 上抬 0.20 m
```

这只适合从上往下抓。树上的苹果可能需要从前方、侧方或斜方向接近，因此以后应改为：

```text
approach_axis = 根据相机视角、苹果表面、树枝位置和候选姿态确定

PREGRASP = GRASP - approach_axis × approach_distance
GRASP    = TCP 对准苹果抓取中心
RETREAT  = 沿 approach_axis 的反方向原路撤离
```

候选姿态不再只围绕垂直抓取的 yaw/pitch，而是根据点云障碍物选择碰撞最少、关节余量最大、撤离路径完整的三维接近方向。

### 15.6 当前八点 XY 模型的适用限制

当前 `visual_xy_correction` 是在固定 SCAN、原桌面高度和当前工作区采集的二维相似变换。它已经在桌面抓取中真机成功，但不能默认覆盖树上任意深度和高度。

迁移到树上前需要：

- 验证原始手眼 TF 在不同 XYZ 深度下的一致性；
- 在多个高度采集视觉中心与真实 TCP；
- 判断二维校正是否仍成立；
- 必要时改为三维残差模型，或重新优化手眼外参；
- 给校正模型增加有效 Z 范围，超出训练范围时失败即停。

动态点云解决的是环境障碍物问题，视觉校正解决的是苹果坐标精度问题，两者不能互相代替。

### 15.7 明天的复现顺序

明天继续时按以下顺序进行，先保持今天成功代码和日志不变：

1. 给当前成功版本和全自动成功日志建立新的 Git 检查点；
2. 删除“苹果中心减半径后扩展成整张桌面”的场景假设；
3. 不强制增加固定桌面，改为可选静态碰撞物体；
4. 新建过滤点云话题，输入使用
   `/camera/camera/depth/color/points`；
5. 完成工作区裁剪、体素降采样、离群点过滤和机器人自过滤；
6. 根据 YOLO 苹果掩膜与深度移除当前目标苹果点云；
7. 给 MoveIt 加载 PointCloud OctoMap 更新器；
8. 确认点云话题的订阅者中出现 MoveIt/OctoMap 更新节点，而不再只有 RViz；
9. 在 RViz 的 MotionPlanning/PlanningScene 中确认平台只以实际局部大小出现；
10. 保持机械臂不动，先测试 OctoMap 的刷新、清除和 TF 稳定性；
11. 分别用原桌面苹果和高平台苹果执行 `--preflight-only`；
12. 确认目标苹果已从障碍物中移除，而平台仍保留；
13. 使用人工确认模式进行第一轮低速真机测试；
14. 全部稳定后再恢复 `--fully-autonomous`；
15. 最后再扩展为树枝环境和非垂直接近方向。

### 15.8 明天的通过标准

```text
RViz 能看到原始点云
  +
MoveIt PlanningScene 能看到过滤后的局部障碍物
  +
平台不会被扩展成大平面
  +
目标苹果不会作为碰撞物挡住 GRASP
  +
相机移动后旧体素能够清除
  +
桌面和高平台苹果均能完成完整 plan-only
  +
任意点云/TF异常都会失败即停
```

### 15.9 当天临时实验改动：局部支撑保护

在动态点云 OctoMap 尚未接入前，2026-08-10 先把原来的全工作区虚拟大平面缩成苹果周围的局部正方形保护区：

```yaml
scene:
  support_guard_footprint_diameters: 5.0
```

计算方式：

```text
苹果直径 = 2 × 视觉半径
局部保护边长 = 苹果直径 × 5
局部保护中心 XY = 当前苹果中心 XY
局部保护顶面 Z = 苹果中心 Z - 苹果半径 + 2 mm
```

例如本轮高平台苹果半径约 29.5 mm，局部保护范围约为
`0.295 × 0.295 m`，不再生成原来的 `0.75 × 1.20 m` 高平面。

该改动只是实验过渡方案：

- 能避免苹果升高后整个工作区一起被抬高；
- 仍保留苹果正下方支撑面的碰撞保护；
- 局部范围之外的真实障碍物不会被 MoveIt 看见；
- 如果夹爪简化碰撞体在苹果正下方就与平台相交，缩小 XY 范围后仍会报碰撞；
- 明天仍按第 15.7 节接入过滤点云，最终删除这个苹果高度推算方案。

修改后已立即执行一次无运动、无夹爪命令的实时 `--preflight-only`：

```text
results/single_apple_grasp_20260810_164355.json
```

验证结果：

```text
苹果直径约 58.8 mm
局部保护范围约 0.294 × 0.294 m
局部范围配置：PASS
机械臂/夹爪真实执行：SKIPPED
完整规划：REJECT
```

剩余碰撞仍然是：

```text
apple_test_table_guard <-> gripper_body_approx
```

这说明缩小范围已经消除了远处的错误大平面，但当前冲突发生在苹果正下方的中心区域；只缩小 XY 范围不能消除该碰撞。当天到此停止，不关闭碰撞检查、不真实执行，留待动态点云和更准确夹爪几何一起处理。

随后修正了 PlanningScene 生命周期：虚拟支撑面只允许存在于当前一轮规划/执行期间。

```text
程序开始并连接 MoveIt
  → 删除同名历史残留
  → 本轮规划前 ADD 局部支撑面
  → 成功、失败或 Ctrl-C 进入 finally
  → REMOVE apple_test_table_guard
```

日志新增：

```text
stale_support_guard_removed_on_start
support_guard_removed_on_exit
```

正常退出后 RViz 不应继续显示该虚拟支撑面。若进程被强制杀死来不及执行
`finally`，下一轮程序启动时也会先删除残留。

今天最终结论：TCP、固定 SCAN、视觉三维定位、八点 XY 校正、MoveIt 候选筛选、低速笛卡尔抓取、RS485 夹爪、LIFT、自动放回和返回 SCAN 已形成一条能够在真机重复执行的单苹果完整流程。下一阶段不再以固定桌面为最终假设，而是把经过过滤的 RealSense 点云接入 MoveIt 动态碰撞场景，为不同高度、平台环境和树上苹果抓取建立通用基础。

## 16. 19:26 高平台真机测试：修正错误的 LIFT 规划门禁

### 16.1 之前为什么“目标附近可达，但整轮完全不动”

旧的快速筛选同时对 `PREGRASP`、`VERIFY`、`GRASP` 和 `LIFT` 做独立 IK。所有路点都使用从固定 SCAN 生成的少量确定性关节种子。

这对 LIFT 不合理。真实运动顺序是：

```text
SCAN → PREGRASP → VERIFY → GRASP → LIFT
```

因此 LIFT 的正确起点是已经到达的 `GRASP` 轨迹末端状态，而不是 SCAN。高平台测试中，`yaw 0° / pitch +20°` 已经能到达 PREGRASP、VERIFY 和 GRASP，却因为“从 SCAN 种子独立求 LIFT IK”返回 `NO_IK`，整个候选在真机运动前被误杀。

这不是机械臂真实不可达，也不是某个关节被物理卡住，而是快速筛选的起点假设错误。

### 16.2 正式保留的规划结构

正式代码已经移除快速筛选中的独立 LIFT IK 门禁：

```text
目标苹果
  → 快速 IK/碰撞筛选 PREGRASP、VERIFY、GRASP
  → 排序并选择抓取姿态
  → OMPL 完整规划 SCAN → PREGRASP
  → 以上一段轨迹末端为起点，连续笛卡尔预检 PREGRASP → VERIFY
  → 继续以上一段末端为起点，预检 VERIFY → GRASP
  → 从 GRASP 末端状态连续预检 GRASP → LIFT
  → 每段 fraction >= 0.999 后才允许真机执行
```

保留的关键点：

- 没有放宽 URDF 关节上下限；
- 没有关闭 MoveIt 碰撞检查；
- 没有降低笛卡尔完整路径比例要求；
- 没有修改 `Link6 → tcp_link = 0.138 m`；
- 修正的是 LIFT 的规划起点和筛选职责，不是绕过安全门禁。

对应正式文件：

```text
apple_pick_v2/single_apple_full_grasp.py
apple_pick_v2/single_apple_full_grasp.yaml
```

### 16.3 高平台上的逐级验证结果

本轮视觉中心位于较高平台，校正后的抓取 X/Y 约为：

```text
corrected apple_center ≈ [0.38215, -0.21184, 0.17807] m
selected pose          = yaw 0° / pitch +20°
```

验证过程：

| 日志 | 结果 | 结论 |
|---|---|---|
| `single_apple_grasp_20260810_192429.json` | 快速筛选 REJECT | 10 cm LIFT 被旧的独立 SCAN-seed IK 错误拒绝 |
| `single_apple_grasp_20260810_192525.json` | plan-only REJECT | 移除错误门禁后，10 cm 连续 LIFT 只能达到 `fraction=0.8529` |
| `single_apple_grasp_20260810_192558.json` | `PASS_PREFLIGHT_ONLY` | 7 cm 连续 LIFT 的全部路径均为 `fraction=1.0000` |
| `single_apple_grasp_20260810_192619.json` | `PASS_SINGLE_ROUND_COMPLETE` | 真机完成到位、闭爪和 7 cm 上抬 |

最后一轮实际记录：

```text
GRASP 目标     = [0.3821533, -0.2118408, 0.2030717] m
LIFT 目标      = [0.3821533, -0.2118408, 0.2730717] m
实际 LIFT TCP  = [0.3821045, -0.2120600, 0.2729097] m
LIFT 到位误差  ≈ 0.28 mm
```

当前默认 `lift_m` 暂定为经过真机完整验证的 `0.07 m`。它不是机械臂永远只能上抬 7 cm，而是当前高平台、当前抓取姿态下能够保证完整连续路径的实验值。

### 16.4 本轮“路径成功”和“夹住苹果”必须分开判断

本轮 MoveIt 路径与真机到位均成功，但现场苹果没有可靠夹住。当前参数是：

```yaml
targets:
  grasp_above_apple_m: 0.025
  lift_m: 0.07

modbus_gripper:
  close_mm: 20.0
```

可能需要让 TCP 再下降，或让夹爪闭合得更紧：

- 减小 `grasp_above_apple_m` 表示下降更深；
- 减小 `close_mm` 表示夹爪闭合得更紧；
- 每轮只改一个参数，先确定抓取深度，再确定闭合宽度；
- 所有改动仍必须通过碰撞检查和连续笛卡尔 `fraction >= 0.999`。

本节成功证明的是：机械臂能够依据视觉目标完成整条连续路径，不再被错误的独立 LIFT IK 门禁阻挡。夹持深度和闭合宽度继续由现场逐步寻找最优值。

## 17. 相机苹果识别不发布问题的改善与当前启动方式

### 17.1 已经保留的识别改善

`apple_center_localizer_v2.py` 已经保留以下改动：

- 不再只裁一块固定左侧 ROI；640×480 图像使用两块重叠的 480×480 全高 ROI，覆盖完整横向视野；
- YOLO 推理显式使用 `classes=[47]`，只保留通用模型中的 apple 类别；
- 优先选择没有接触 ROI 边缘的完整苹果框，避免截断苹果造成错误半径和深度；
- 保持 `conf=0.15`，没有通过锐化、CLAHE 或继续降低阈值掩盖问题；
- 每秒输出一次简洁统计：`frames`、`latest_frame_age`、`yolo_boxes`、`best_conf`、`conf_reject`、`depth_reject`、`tf_reject`、`published_center`；
- 每次节点启动保存原始彩图、YOLO 输入、局部标注图和完整标注图到 `results/vision_debug/<时间>/`。

高平台真机测试时，苹果置信度约 `0.91`，半径约 `29.7 mm`，连续 10 帧 XYZ 波动小于约 `0.7 mm`，并持续发布 `/apple_pick_v2/apple_center`。因此昨天的“画面可见但中心不发布”问题已经明显改善。

需要注意：节点名字存在不代表输入正常。如果 `latest_frame_age` 持续增大，说明 RealSense 或整套 launch 已停止，单独残留的视觉节点不会产生新结果。此时应重启总 launch，而不是修改置信度。

### 17.2 启动命令没有变化

终端 1 启动整套 ROS、真机驱动、MoveIt、RealSense、TF、RViz 和苹果定位：

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash

ros2 launch $APPLE_CATCH_ROOT/apple_pick_v2/apple_hand_eye_all.launch.py \
  allow_trajectory_execution:=true
```

该 launch 会自动用下面的 Python 环境启动苹果识别，无需再单独启动定位节点：

```text
$APPLE_YOLO_PYTHON
```

终端 2：自动抓取并在 LIFT 停止、保持夹爪闭合：

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash

$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --execute --operator-confirmed --autonomous
```

如果明确需要抓取后自动放回并返回 SCAN，使用：

```bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --execute --operator-confirmed --fully-autonomous
```

启动后可用下面的只读命令检查识别链路：

```bash
ros2 topic hz /apple_pick_v2/apple_center
ros2 topic echo /apple_pick_v2/apple_diagnostics
```
