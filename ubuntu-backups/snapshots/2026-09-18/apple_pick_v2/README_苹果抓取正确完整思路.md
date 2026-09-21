# RM65 Eye-in-Hand 苹果抓取：正确完整思路（2026-08-06）
\n> 本文保留 TCP、手眼验证与分阶段抓取的形成过程。当前可执行命令以仓库根目录 `README.md` 为准。

这是本项目从明天开始继续工作的唯一权威说明。此前的苹果抓取试验脚本和启动文档仅作历史保留，不再作为实现依据。

当前阶段仍然是“模型与视觉验证阶段”：没有解锁机械臂自动运动，也没有解锁 RS485 夹爪自动闭合。

## 1. 今天已经确认的事实

### 1.1 机械臂与控制器末端

- MoveIt 机械臂链的法兰 link 是 `Link6`。
- RM 控制器当前且唯一的工具坐标系名称是 `Arm_Tip`。
- 已通过官方 RM API 读取 `Arm_Tip` 的完整数值：

```text
Arm_Tip xyz = [0, 0, 0] m
Arm_Tip rpy = [0, 0, 0] rad
```

所以当前 `Arm_Tip` 等同于法兰，不包含 138 mm 的 TCP 补偿。

### 1.2 物理 TCP

- 夹爪抓取方向与 `Link6` 正 Z 轴平行。
- 夹爪竖直抓桌面苹果，不采用倾斜或侧抓。
- 两指夹住苹果时，苹果中心位于 `Link6` 正 Z 方向 138 mm。
- 170 mm 是夹爪总体长度，不是 TCP，后续算法不再使用它。

唯一 TCP 定义：

```text
Link6 -> tcp_link
xyz = [0, 0, 0.138] m
rpy = [0, 0, 0] rad
```

138 mm 只在 URDF/MoveIt 中出现一次。不得再在抓取代码里从目标位置减去“夹爪长度”“Arm_Tip 偏移”或其他人工 Z 补偿。

### 1.3 相机与手眼关系

相机是 Eye-in-Hand，刚性安装在末端。完整树应为：

```text
base_link -> ... -> Link6 -> tcp_link
                         -> gripper_body_approx
                         -> camera_link -> ... -> camera_color_optical_frame
```

`Link6 -> camera_link` 仍由已有手眼静态 TF 发布。增加 `tcp_link` 不改变手眼标定外参，也不能把相机外参重新挂到 `tcp_link` 下。

## 2. 今天已经落地的模型修改

### 活动 URDF

文件：`$ROS_WS/src/ros2_rm_robot/rm_description/urdf/rm_65.urdf`

新增：

- `tcp_link`，固定在 `Link6 +Z 0.138 m`。
- `gripper_body_approx`，作为第一版夹爪可视/碰撞模型。
- 近似长方体尺寸为 `0.080 × 0.080 × 0.170 m`，中心为 `Link6 +Z 0.085 m`。

这个长方体是保守近似，不代表精确夹爪外形。明天可先用于观察明显的桌面/机械臂碰撞；后续量出夹爪宽度、厚度、手指位置后，应改成夹爪主体和两根手指三个碰撞体。

### MoveIt SRDF

文件：`$ROS_WS/src/ros2_rm_robot/rm_moveit2_config/rm_65_config/config/rm_65_description.srdf`

`rm_group` 已从：

```xml
<chain base_link="base_link" tip_link="Link6"/>
```

改为：

```xml
<chain base_link="base_link" tip_link="tcp_link"/>
```

因此之后的 IK、姿态约束和目标位姿都应明确以 `tcp_link` 为目标末端。

### 同步文件与构建

- `rm_65.urdf.xacro` 已同步相同 TCP 和近似夹爪体。
- `apple_pick_v2/config.yaml` 已记录控制器 `Arm_Tip` 为零偏移且已验证。
- `rm_description` 与 `rm_65_config` 已重新构建。
- 静态 URDF、默认 Xacro、6F Xacro、6FB Xacro 均已通过解析检查。

修改前备份统一以 `.pre_tcp_20260806` 结尾，位于各自原文件旁边。

## 3. 明天启动顺序

今天仍在运行的 ROS 节点保存的是启动时读入的旧模型，必须全部正常退出后，明天重新启动。

### 终端 1：机械臂驱动

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
ros2 launch rm_bringup rm_65_bringup.launch.py
```

### 终端 2：RealSense

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  enable_sync:=true \
  pointcloud.enable:=true \
  publish_tf:=true
```

### 终端 3：手眼静态 TF

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
ros2 launch $APPLE_CATCH_ROOT/hand_eye_static_tf.launch.py
```

### 终端 4：MoveIt 与 RViz

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
ros2 launch rm_65_config real_moveit_demo.launch.py
```

### 重要：不要再运行旧 TCP 可视化发布器

不要运行：

```text
$APPLE_CATCH_ROOT/apple_pick_v2/tcp_visualizer.py
$APPLE_CATCH_ROOT/apple_pick_v2/publish_tcp_tf.py
```

`tcp_link` 已经属于 URDF，应由 `robot_state_publisher` 唯一发布。再次发布相同的 `Link6 -> tcp_link` 会形成重复 TF 发布者。

## 4. 明天第一项：只验证新模型，不运动

### 4.1 检查 TF

```bash
ros2 run tf2_ros tf2_echo Link6 tcp_link
```

预期持续输出：

```text
Translation: [0.000, 0.000, 0.138]
Rotation RPY: [0.000, 0.000, 0.000]
```

再检查相机树已经接通：

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

### 4.2 检查 MoveIt 末端

```bash
ros2 param get /move_group robot_description_semantic | grep tcp_link
```

应看到 `tip_link="tcp_link"`。

### 4.3 RViz 检查

RViz Fixed Frame 设为 `base_link`，加入 `RobotModel` 和 `TF`：

- `tcp_link` 原点应位于两指夹住苹果的中心附近。
- `tcp_link` 蓝色 Z 轴应沿夹爪下探方向。
- 灰色半透明长方体是 `gripper_body_approx`，应大致覆盖夹爪主体。
- 若长方体尺寸明显不合理，先量尺寸并修改模型，不进入规划。

这里只能验证几何关系和明显碰撞，不能证明抓取一定安全。

## 5. 手眼标定必须先通过多姿态验证

在桌面固定一个不移动的目标，让机械臂处于至少三个安全观察姿态。每个姿态识别目标并换算到 `base_link`。

正确现象：机械臂和相机姿态改变，但固定目标在 `base_link` 下的 XYZ 基本不变。

第一版判断建议：

- 各姿态坐标变化在约 10 mm 内：可以继续做规划验证。
- 变化达到或超过约 20 mm：停止抓取，先检查手眼外参、深度对齐、时间同步和目标深度估计。

不要把手眼误差当成 IK 误差，也不要通过手调抓取点补偿手眼问题。

视觉节点仍然只能发布结果：

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
"$APPLE_YOLO_PYTHON" "$APPLE_CATCH_ROOT/apple_pick_v2/apple_center_localizer_v2.py"
```

输出：

- `/apple_pick_v2/apple_center`：`base_link` 下的苹果中心。
- `/apple_pick_v2/hand_eye_markers`：RViz 苹果 Marker。

## 6. 正确的点位与坐标语义

### SCAN 鸟瞰点

- 唯一需要人工示教的点。
- 保存固定关节角，不保存一个靠手调补偿得到的苹果抓取位姿。
- 夹爪竖直、相机朝下、远离桌面，苹果区域完整进入视野。
- 每次识别前先回到 SCAN，使视角和深度条件稳定。

### GRASP 抓取点

识别得到苹果中心 `P_fruit` 后：

```text
tcp_link 的目标位置 = P_fruit
tcp_link 的目标姿态 = 固定垂直向下姿态 Q_down
```

不再计算 Link6 目标，不再减去 138 mm。MoveIt 会根据 URDF 自动把 TCP 目标转换成法兰和关节目标。

### PREGRASP 预抓取点

工具正 Z 指向下探方向，所以预抓取点是沿工具负 Z 退回 80–100 mm：

```text
P_pregrasp = P_fruit - Z_tcp_in_base * d
d 初值 = 0.09 m
```

这里必须使用“工具方向”生成预抓取点，不能默认 `base_link +Z` 永远等于桌面上方。第一版固定垂直姿态时，两者应当一致或接近，但代码仍应按工具轴表达。

## 7. 分阶段规划与执行顺序

完整流程固定为：

```text
HOME
  -> SCAN（人工固定关节位）
  -> 识别苹果并发布 RViz Marker
  -> 检查坐标、深度、工作空间和桌面高度
  -> 生成 PREGRASP
  -> MoveIt 只规划并在 RViz 预览
  -> 人工确认
  -> 移动到 PREGRASP
  -> 再次人工确认苹果位于两指中心
  -> 沿 tcp_link 正 Z 低速直线下降到 GRASP
  -> 人工确认
  -> RS485 低力闭合夹爪
  -> 沿原直线反向撤离 80–100 mm
  -> 返回 SCAN 或进入放置流程
```

到 PREGRASP 可以使用 MoveIt 常规规划；从 PREGRASP 到 GRASP、从 GRASP 撤离必须使用低速笛卡尔直线，并检查直线路径完成比例。不能让规划器在苹果附近自由绕行。

## 8. 在写执行代码前必须具备的安全门

- `Arm_Tip` 仍为零偏移；如果示教器中被修改，必须重新读取并停止执行。
- MoveIt 目标 link 只有 `tcp_link`，代码中不存在 138/150/170 mm 的人工末端补偿。
- 手眼多姿态验证通过。
- 苹果深度有效，且时间同步、对齐深度和相机内参有效。
- 桌面作为 CollisionObject 加入 PlanningScene；不能只靠高度 if 判断。
- 夹爪近似碰撞体在 RViz 中方向与大小合理。
- PREGRASP 的 IK、关节限位、自碰撞和环境碰撞检查通过。
- GRASP 直线段完成比例达到设定阈值，否则禁止执行。
- 第一版速度和加速度缩放保持很低，并保留人工确认与急停。
- RS485 夹爪只在 GRASP 到位且确认后低力闭合。

## 9. 明天建议只完成这三件事

1. 重启后验证 `Link6 -> tcp_link`、MoveIt tip 和 RViz 夹爪近似模型。
2. 量取夹爪真实宽度、厚度、手指长度，改进碰撞模型。
3. 固定桌面目标完成至少三个观察姿态的手眼一致性测试。

这三项全部通过后，再写新的 plan-only 节点，只生成 `PREGRASP/GRASP` 并在 RViz 预览；不要直接接真机执行。
