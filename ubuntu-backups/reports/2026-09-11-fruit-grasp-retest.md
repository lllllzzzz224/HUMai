# 2026-09-11 果实抓取复测报告

## 结论状态

本报告记录本次重新执行的结果。2026-09-10 的历史成功日志只用于定位入口，不计入本次通过。

| 场景 | 离线验证 | 本次真机结果 | 说明 |
|---|---|---|---|
| 1. 单一橘子抓取 | 通过（公共抓取逻辑单测；实时抓取路径预检） | 程序完成，物理结果待现场确认 | 苹果与橘子串同场，orange 单类别过滤；从橘子串选择一颗，完成起吊、原位放回、回 SCAN |
| 2. 单一苹果抓取 | 通过（公共抓取逻辑单测；实时抓取路径预检） | 程序完成，物理结果待现场确认 | 苹果与橘子串同场，apple 单类别过滤；完成起吊、原位放回、回 SCAN |
| 3. 先抓橘子后抓苹果 | 通过（每轮实时预检） | 两轮程序均完成，物理结果待现场确认 | 按橘子→苹果顺序执行两次单轮，中间显式切换唯一视觉类别；不是自主跨类别任务编排 |

## 被测实现与环境

- 实机源码：`/home/li/hand_eye_calibration/apple_pick_v2`
- Git HEAD：`32e115e9048e52c41da44dc53e7097dfbed31d26`
- 本次复测时，`grasp_cli.py`、`single_apple_full_grasp.py`、`single_apple_full_grasp.yaml` 存在未提交修改；未修改这些生产文件。
- 主控 Python：`/home/li/ros_yolo_env/bin/python`，Python 3.12.3
- 视觉进程：`/home/li/anaconda3/envs/yolo11/bin/python`，Ultralytics 8.4.102、PyTorch 2.13.0+cu130
- ROS 2 Jazzy，`ROS_DOMAIN_ID=42`
- GPU：NVIDIA GeForce RTX 4060 Ti，驱动 595.84，显存 8188 MiB
- 机械臂：RealMan RM65，控制器 `192.168.1.18`
- 测试策略：`--place-target return`，抓取后原位放回，不依赖尚在审查的入盆流程。

关键文件 SHA-256：

```text
8ed7b46962e2345bc60059172cc48ca800860f0a1cb8eac96b73b9ec3cb9c641  grasp_cli.py
874c2fd14e946c7dcfaf1a85cc0504ef51bdb480aa2064c410160c91c729f35b  single_apple_full_grasp.py
6ea6288671e8d4422808d29a5f26559e2c4b5fc1b08de91683e2347afe298b64  single_apple_full_grasp.yaml
b17c14e757e993a7dcb31e75ee1dd3870aa264c648947601f12cbc16d08e5b0f  apple_center_localizer_v2.py
3b9407a7d15abce97a63a9d902b8e8bb3f9b1546627f316286c6ea5270af9fc9  apple_hand_eye_all.launch.py
```

## 已执行的离线验证

### 公共抓取逻辑单测

命令：

```bash
cd /home/li/hand_eye_calibration/apple_pick_v2
PYTHONDONTWRITEBYTECODE=1 /home/li/ros_yolo_env/bin/python -m unittest discover -s tests -v
```

实际结果：`Ran 25 tests in 0.205s`，`OK`。覆盖 CLI 真机授权门、连续两轮上限、目标稳定等待、轮间至少移动 30 mm、执行前失败可重试、运动开始后故障锁定、轨迹阶段顺序和 SCAN 到位容差。该结果只证明公共软件逻辑通过，不证明任何水果已被真机夹起。

## 真机只读健康检查

检查时间：2026-09-11 17:28（Asia/Shanghai）。

执行的只读检查：

```bash
source /opt/ros/jazzy/setup.bash
source /home/li/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 node list
ros2 topic list -t
ros2 node info /apple_center_localizer_v2
ros2 topic echo --once --full-length /apple_pick_v2/apple_diagnostics
ros2 topic echo --once /apple_pick_v2/apple_center
ros2 topic hz /camera/camera/color/image_raw --window 5
ros2 topic hz /joint_states --window 5
ros2 param get /move_group allow_trajectory_execution
ping -c 1 -W 1 192.168.1.18
```

实际结果：

- 在线节点包括 `/rm_driver`、`/rm_control`、`/move_group`、`/camera/camera`、`/apple_center_localizer_v2`。
- 相机彩色流约 29–30 Hz，关节状态约 198–201 Hz。
- MoveIt `allow_trajectory_execution=True`。
- 机械臂控制器网络可达，单次 ping 0.439 ms。
- `/dev/video0` 至 `/dev/video3`、`/dev/video8`、`/dev/video9` 存在；夹爪经机械臂末端 RS485，不依赖本机 ttyUSB。
- 视觉实际进程参数为 `--target orange`；同时存在一条较早 launch 命令行显示 `target:=apple`，说明进程经历过独立切换，执行前以实际视觉子进程和实时诊断为准。
- 当前橘子过滤视觉输出：置信度 0.540，估算半径 0.032261 m，中心 `[0.398666, -0.259219, -0.043518] m`。此前一次采样为置信度 0.579、半径 0.034348 m、中心 `[0.400058, -0.259616, -0.044491] m`。
- 当前关节位置 `[2.498351, 0.357795, 0.662123, 0.047377, 2.033780, -0.063640] rad`，接近 YAML 中 SCAN 关节位。

只读检查证明通信、感知与规划栈在线，但不代替现场对水果数量、摆放稳定性、急停和人员净空的确认。

## 场景执行记录

### 场景 1：单一目标类别（橘子）

现场为一只苹果与一串橘子同场。该场景依靠唯一 `--target orange` 发布者做类别隔离，从橘子串中选择一颗；不是只有一颗孤立橘子的物理环境。

首次按开发文档误运行了以下入口：

```bash
/home/li/ros_yolo_env/bin/python grasp_cli.py \
  --preflight-only --operator-confirmed --place-target return
```

实际结果：0.28 秒、退出码 0、无输出。源码核查确认 `grasp_cli.py` 只定义参数解析与授权门，没有 `main()`；该次为空运行，不计作预检。正确入口为 `single_apple_full_grasp.py`。

正确预检命令（不执行轨迹、不控制夹爪）：

```bash
cd /home/li/hand_eye_calibration/apple_pick_v2
source /opt/ros/jazzy/setup.bash
source /home/li/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
/home/li/ros_yolo_env/bin/python single_apple_full_grasp.py \
  --preflight-only --operator-confirmed --place-target return
```

实际结果：`PASS_PREFLIGHT_ONLY`。目标原始中心 `[0.398950, -0.259215, -0.045682] m`，校正后中心 `[0.390286, -0.231173, -0.045682] m`，置信度 0.543，半径 34.4 mm；选择 Stage 0（yaw 偏移 0°、pitch 0°）。`SCAN -> PREGRASP` 有效，`PREGRASP -> VERIFY`、`VERIFY -> GRASP`、`GRASP -> LIFT` 的笛卡尔 fraction 均为 1.0000。独立 `--preflight-only` 只检查至 LIFT，不含 return 放回段；放回段在随后真实执行开始前重新预检并通过。

预检证据：`reports/evidence/scenario1-orange-preflight.json`。

真机执行命令（在预检通过后运行）：

```bash
/home/li/ros_yolo_env/bin/python single_apple_full_grasp.py \
  --execute --operator-confirmed --fully-autonomous --place-target return
```

实际结果：进程退出码 0，程序状态 `PASS_PICK_RETURN_COMPLETE`。执行前重新采样目标并完成全部抓取和 return 路径预检，随后完成真实运动：

- 目标原始中心 `[0.398319, -0.259213, -0.044181] m`，校正后 `[0.389703, -0.231195, -0.044181] m`，置信度 0.516，半径 33.1 mm。
- `SCAN -> PREGRASP` 完成，PREGRASP 到位误差 1.27 mm。
- `PREGRASP -> VERIFY` 完成，VERIFY 到位误差 1.10 mm。
- `VERIFY -> GRASP` 完成，GRASP 到位误差 0.56 mm。
- 夹爪下发 2.0 mm 位置命令并等待 2.5 秒。
- `GRASP -> LIFT` 完成，起吊 70 mm，LIFT 到位误差 0.47 mm。
- `LIFT -> RELEASE` 完成，RELEASE 到位误差 0.38 mm；夹爪重新下发 80.0 mm 开爪命令。
- `RELEASE -> PREGRASP -> SCAN` 完成，最终 SCAN 最大关节误差 0.01997 rad。

真机程序证据：`reports/evidence/scenario1-orange-execution.json`。执行前/后图像：`reports/evidence/scenario1-orange-before.jpg`、`reports/evidence/scenario1-orange-after.jpg`。

物理判定：等待现场操作员确认橘子是否实际被夹起、稳定抬升并原位放回。由于程序无夹爪位置/力反馈，暂不把程序 PASS 直接写成物理抓取通过。

### 场景 2：单一目标类别（苹果）

现场仍保留橘子串，依靠唯一 `--target apple` 发布者做类别隔离。相机画面中苹果与橘子串有明显空间间隔。

视觉切换：停止原 `--target orange` 视觉子进程，仅启动 `--target apple`。第一次用 `nohup` 启动的 PID 随命令外壳结束而退出，日志为空；这次失败未进入预检或运动。随后改为持久前台终端启动，确认进程命令行含 `--target apple`，`/apple_pick_v2/apple_center` publisher count 为 1。

正确预检命令与场景 1 相同，视觉目标已切换为 apple。实际结果：`PASS_PREFLIGHT_ONLY`。苹果校正中心 `[0.437371, -0.145269, -0.052396] m`，置信度 0.834，半径 32.0 mm；Stage 0 无可用 IK，选择 Stage 1（yaw 偏移 0°、pitch +10°）；三段笛卡尔 fraction 均为 1.0000。

预检证据：`reports/evidence/scenario2-apple-preflight.json`。

真机执行命令与场景 1 相同。实际结果：进程退出码 0，程序状态 `PASS_PICK_RETURN_COMPLETE`。

- 目标校正中心 `[0.437269, -0.145604, -0.053264] m`，半径 32.84 mm。
- PREGRASP、VERIFY、GRASP、LIFT、RELEASE 到位误差依次为 1.50、0.73、0.77、0.47、0.71 mm。
- 闭爪命令 2.0 mm，起吊 70 mm，释放开爪命令 80.0 mm。
- 释放后 PREGRASP 到位误差 0.34 mm，最终回 SCAN 最大关节误差 0.01995 rad。

真机程序证据：`reports/evidence/scenario2-apple-execution.json`。执行前/后图像：`reports/evidence/scenario2-apple-before.jpg`、`reports/evidence/scenario2-apple-after.jpg`。

物理判定：等待现场操作员确认苹果是否实际被夹起、稳定抬升并原位放回。

### 场景 3：先橘子后苹果

当前 `--continuous --max-rounds 2` 只处理同一视觉目标类别，因此本次按两次单轮执行，中间显式切换唯一视觉发布者，实际顺序为 orange → apple。该结果验证了指定顺序的连续人工编排流程，不表示系统已实现自主跨类别任务编排。历史 2026-09-10 顺序为先苹果后橘子，与本次相反，未复用为本次结果。

#### 第 1 轮：橘子

- 视觉：唯一 `--target orange` 发布者，预检时目标校正中心 `[0.324350, -0.221182, -0.046863] m`，置信度 0.629，半径 31.6 mm。
- 预检：`PASS_PREFLIGHT_ONLY`，Stage 0、pitch 0°，三段笛卡尔 fraction 均为 1.0000。证据：`reports/evidence/scenario3-round1-orange-preflight.json`。
- 执行：程序状态 `PASS_PICK_RETURN_COMPLETE`，校正中心 `[0.324772, -0.221323, -0.046357] m`。PREGRASP、VERIFY、GRASP、LIFT、RELEASE 到位误差依次为 2.60、0.52、0.60、0.50、0.76 mm；最终回 SCAN 最大关节误差 0.02000 rad。证据：`reports/evidence/scenario3-round1-orange-execution.json`。

#### 第 2 轮：苹果

- 轮间状态：第 1 轮回 SCAN 后保存相机画面，确认苹果与橘子串仍在；停止 orange 发布者，启动唯一 `--target apple` 发布者。
- 视觉：一次瞬时置信度为 0.178，未据此执行；正式预检重新采集 10 个样本，汇总置信度 0.765，校正中心 `[0.428220, -0.129143, -0.055119] m`，坐标跨度不超过 5.1 mm。
- 预检：`PASS_PREFLIGHT_ONLY`，Stage 0、pitch 0°，三段笛卡尔 fraction 均为 1.0000。证据：`reports/evidence/scenario3-round2-apple-preflight.json`。
- 执行：程序状态 `PASS_PICK_RETURN_COMPLETE`，校正中心 `[0.428504, -0.128744, -0.055474] m`。PREGRASP、VERIFY、GRASP、LIFT、RELEASE 到位误差依次为 2.70、0.62、0.76、0.39、0.66 mm；最终回 SCAN 最大关节误差 0.01998 rad。证据：`reports/evidence/scenario3-round2-apple-execution.json`。

轮间与最终图像：`reports/evidence/scenario3-before.jpg`、`reports/evidence/scenario3-between-orange-apple.jpg`、`reports/evidence/scenario3-after.jpg`。

物理判定：两轮程序均完成，但仍等待现场操作员确认两次是否分别实际夹起橘子、苹果并稳定抬升、原位放回。

## 收尾状态

- 最后一轮执行进程退出码 0，未发现仍在运行的 `single_apple_full_grasp.py`（进程检索只匹配到检查命令自身）。
- `/execute_trajectory/_action/status` 中最后四个目标状态均为 4（SUCCEEDED），没有显示活动目标。
- 最终关节位置 `[2.505157, 0.343678, 0.656818, 0.049383, 2.031948, -0.080218] rad`；相对 YAML SCAN 的最大关节差约 0.0136 rad。
- 已恢复复测前的 orange 视觉过滤。持久 tmux 会话 `fruit_vision_retest` 中运行唯一 `--target orange` 进程；`/apple_pick_v2/apple_center` publisher count 为 1，恢复后已发布实时诊断。

## 判定限制

当前 return 流程的程序成功判定只确认 Modbus 写请求收到布尔 ACK，并固定等待 2.5 秒；它没有夹爪闭合位置/力反馈，也没有起吊后的视觉载荷确认。因此即使日志写出 `PASS_PICK_RETURN_COMPLETE`，本报告仍要求现场观察确认果实被实际夹起、稳定抬升、原位放回，才能将对应真机场景判为通过。
