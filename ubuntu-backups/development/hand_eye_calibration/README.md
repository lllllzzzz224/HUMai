# Apple Catch

RM65 + RealSense Eye-in-Hand + ROS 2 Jazzy + MoveIt 2 + 工具端 RS485 两指夹爪的单苹果、连续苹果视觉抓取实现。

当前版本已经在真实机械臂完成：固定鸟瞰识别、视觉坐标校正、候选 IK/碰撞筛选、低速直线抓取、上抬、原位放回、返回鸟瞰，以及移动苹果后自动开始下一轮。

> 这是会驱动真实机械臂的实验代码。首次迁移必须重新核验机械臂 IP、TCP、手眼外参、SCAN 关节角、工作空间、视觉校正和夹爪开度。危险时使用示教器急停。

## 已验证流程

```text
SCAN
  → 唯一苹果稳定 2 秒
  → 3 秒安全倒计时
  → 快速 IK/碰撞筛选与完整路径预检
  → 规划后再次检查苹果是否移动
  → PREGRASP → VERIFY → GRASP
  → RS485 夹爪闭合 → LIFT
  → 原抓取位置 RELEASE → 开爪
  → PREGRASP → SCAN
  → 等待苹果移动至少 30 mm 后开始下一轮
```

核心安全行为：

- 连续模式必须显式指定 `--continuous --execute --operator-confirmed`；
- 运动前失败最多重试 3 次；第一次轨迹执行后发生异常立即锁停；
- 规划结束后苹果漂移超过 10 mm 时禁止执行；
- 视野中只允许一颗苹果；同一位置不会重复抓取；
- 不调用夹爪机械回零，避免初始化时意外闭爪；
- 每轮临时支撑碰撞面在结束或失败时从 PlanningScene 删除。

## 硬件与软件

已验证环境：

- Ubuntu 24.04、ROS 2 Jazzy、MoveIt 2；
- RealMan RM65 及 `rm_driver`、`rm_description`、`rm_control`、`rm_65_config`；
- Intel RealSense RGB-D 与 `realsense2_camera`；
- Eye-in-Hand 固定安装；
- 工具端 RS485 / Modbus RTU 两指夹爪；
- Python 3.12、Ultralytics、NumPy、OpenCV、PyYAML。

ROS Python 包必须由 ROS/ament 环境提供，不要用 pip 替代。视觉环境依赖可安装为：

```bash
python -m pip install -r requirements-vision.txt
```

## 克隆与环境变量

```bash
git clone https://github.com/lllllzzzz224/Apple-catch.git
cd Apple-catch

export APPLE_CATCH_ROOT="$PWD"
export ROS_WS=/absolute/path/to/your/ros2_ws
export APPLE_YOLO_PYTHON=/absolute/path/to/yolo/python
export APPLE_YOLO_MODEL=/absolute/path/to/yolo11n-seg.pt

source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
```

模型权重不提交到 Git。可让 Ultralytics 首次联网准备 `yolo11n-seg.pt`，也可以把已有权重路径写入 `APPLE_YOLO_MODEL`。当前视觉代码使用 COCO 的 apple 类；专用苹果模型需要同步核对类别 ID。

## 迁移前必须核验

当前仓库中的以下数值来自原实机，只能作为示例：

| 项目 | 当前验证值/位置 |
|---|---|
| TCP | `Link6 → tcp_link = xyz(0,0,0.138), rpy(0,0,0)` |
| 手眼外参 | `hand_eye_static_tf.launch.py`、`hand_eye_result.yaml` |
| SCAN 与运动参数 | `apple_pick_v2/single_apple_full_grasp.yaml` |
| 视觉 XY 校正 | 同一 YAML 的 `visual_xy_correction` |
| 夹爪协议 | `modbus_gripper_ros.py`，24 V、RS485/Modbus RTU |
| RM65 IP/驱动 | 你的 ROS 工作区 `rm_driver` 配置 |

0.138 m TCP 只能生效一次：如果控制器 `Arm_Tip` 已包含相同偏移，不能在 MoveIt 再重复补偿。改变相机安装、TCP、SCAN 或机械臂后，应重新完成手眼一致性与视觉偏差测量。

## 无运动测试

以下命令不会启动 ROS 或机械臂：

```bash
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
PYTHONPATH="$APPLE_CATCH_ROOT/apple_pick_v2:${PYTHONPATH}" \
  "$APPLE_YOLO_PYTHON" -m unittest discover \
  -s "$APPLE_CATCH_ROOT/apple_pick_v2/tests" -v
```

## 启动完整 ROS 栈

终端 1，只启动一次：

```bash
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"

ros2 launch "$APPLE_CATCH_ROOT/apple_pick_v2/apple_hand_eye_all.launch.py" \
  allow_trajectory_execution:=true \
  yolo_python:="$APPLE_YOLO_PYTHON" \
  yolo_model:="$APPLE_YOLO_MODEL"
```

另开终端检查是否误启动了两套系统：

```bash
ros2 action info /execute_trajectory
ros2 topic info /joint_states
ros2 node list | sort | uniq -d
```

执行前应只有一个 `/execute_trajectory` action server 和一个 `/joint_states` 发布者。若出现两个 `/move_group`、两个 `rm_driver` 或两个 `robot_state_publisher`，停止全部 launch 后只启动一次。

## 运行抓取

终端 2，先做单轮验证：

```bash
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"

"$APPLE_YOLO_PYTHON" \
  "$APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py" \
  --execute --operator-confirmed --fully-autonomous
```

完成单轮真机验证后，启动连续模式：

```bash
"$APPLE_YOLO_PYTHON" \
  "$APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py" \
  --continuous --execute --operator-confirmed
```

限制轮数可追加 `--max-rounds 1` 或 `--max-rounds 2`。连续模式每轮回到 SCAN 后，将苹果移动至少 30 mm 并立即离开机械臂工作区；不需要按 Enter。

## 常用参数

主要配置：[single_apple_full_grasp.yaml](apple_pick_v2/single_apple_full_grasp.yaml)

- `gripper.open_mm` / `gripper.close_mm`：夹爪开口和闭合位置；
- `targets.pregrasp_above_apple_m`、`verify_above_apple_m`、`grasp_above_apple_m`、`lift_m`：抓取高度；
- `moveit.velocity_scaling` / `acceleration_scaling`：关节空间速度；
- `grasp_cartesian`、`lift_cartesian`：下降、上抬和放回速度；
- `continuous`：目标稳定时间、换位距离、倒计时、重试次数；
- `visual_xy_correction`：当前固定 SCAN 下的实测视觉尺度校正。

修改 YAML 后重启抓取脚本即可；修改视觉节点或 launch 后需要重启完整 ROS 栈。

## 代码入口

- `apple_pick_v2/apple_center_localizer_v2.py`：YOLO + 对齐深度 + TF 苹果中心定位；
- `apple_pick_v2/single_apple_full_grasp.py`：规划、执行、抓取、放回与连续循环；
- `apple_pick_v2/fixed_scan_real_verify.py`：MoveIt 服务、路径验证和到位门禁；
- `apple_pick_v2/continuous_grasp_state.py`：连续模式稳定与故障状态机；
- `modbus_gripper_ros.py`：工具端 RS485/Modbus 夹爪；
- `apple_pick_v2/apple_hand_eye_all.launch.py`：RM65、MoveIt、RealSense、TF、视觉和 RViz 总启动。

## 成功思路与记录

- [连续苹果抓取说明](apple_pick_v2/连续苹果抓取README.md)
- [首次真机成功抓取](apple_pick_v2/success%20apple%20catch.md)
- [8.10 视觉误差模型与自动抓取](apple_pick_v2/success%208.10.md)
- [苹果抓取参数配置说明](apple_pick_v2/苹果抓取参数配置说明.md)
- [苹果抓取正确完整思路](apple_pick_v2/README_苹果抓取正确完整思路.md)
- [手眼一致性测试](apple_pick_v2/手眼一致性测试_苹果.md)

运行日志默认写入 `apple_pick_v2/results/`，该目录被 Git 忽略。GitHub 仓库只保留可复用代码、测试、配置和成功思路。
