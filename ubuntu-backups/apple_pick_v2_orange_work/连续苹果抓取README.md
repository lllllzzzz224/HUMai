# 连续苹果抓取 README

## 一键启动

先启动 ROS、相机、TF、MoveIt 和 RViz：

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
ros2 launch $APPLE_CATCH_ROOT/apple_pick_v2/apple_hand_eye_all.launch.py \
  allow_trajectory_execution:=true
```

再开一个终端启动连续抓取：

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --continuous --execute --operator-confirmed
```

测试指定轮数时追加 `--max-rounds 1` 或 `--max-rounds 2`。不写时读取 YAML；默认 `0` 表示一直等待下一颗。

## 自动流程

```text
固定 SCAN → 苹果稳定 → 3 秒安全倒计时 → 完整预检
→ PREGRASP → VERIFY → GRASP → 闭爪 → LIFT
→ 原抓取位置 RELEASE → 开爪 → PREGRASP → SCAN
→ 等待苹果被移动后自动开始下一轮
```

全程不需要按 Enter。每轮只允许视野内有一颗苹果；机械臂回到 SCAN 后，把苹果移动至少 30 mm，并立即离开机械臂工作区。

## 本次优化

- 原单轮成功路径、TCP 0.138 m、视觉 XY 尺度校正和所有运动/夹爪参数保持不变。
- 同一位置不会重复抓；目标需稳定 2 秒，再经过 3 秒安全倒计时。
- 全部路径规划完成后再次读取苹果位置；漂移超过 10 mm 不执行。
- 运动前失败最多重试 3 次；第一次真实轨迹开始后发生任何异常立即锁停，不自动开爪或恢复运动。
- 每轮和整次会话都写 JSON 日志到 `apple_pick_v2/results/`。
- 旧单轮回退命令 `--execute --operator-confirmed --fully-autonomous` 保留。

## 参数位置

文件：`apple_pick_v2/single_apple_full_grasp.yaml`

```yaml
continuous:
  max_rounds: 0
  min_position_change_m: 0.030
  stable_time_s: 2.0
  safety_countdown_s: 3.0
  retry_delay_s: 2.0
  max_pre_motion_retries: 3
```

停止等待可按 Ctrl+C；真实运动出现危险时优先使用示教器急停。

## 真机验证结果（2026-08-11）

- `--max-rounds 1`：`COMPLETE`，完整抓取/上抬/放回/回 SCAN。
- `--max-rounds 2`：两轮均为 `PASS_PICK_RETURN_COMPLETE`。
- 两轮苹果位置变化 155.858 mm；规划后复核漂移 0.015/0.021 mm。
- SCAN 规划容差仍为 0.015 rad；到位确认独立使用 0.020 rad，消除了控制器微小跟踪残差造成的误报。
- 最终会话日志当时保存为 `results/continuous_apple_grasp_20260811_182627.json`；运行日志不纳入公开仓库。
