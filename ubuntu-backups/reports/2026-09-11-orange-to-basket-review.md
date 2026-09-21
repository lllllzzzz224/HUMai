# 2026-09-11 橘子抓取入盆代码审查

## 范围与结论

本次只读审查以当前工作树 `/home/li/hand_eye_calibration/apple_pick_v2` 为准，覆盖：

- 视觉目标类别与候选锁定：`apple_center_localizer_v2.py`、`apple_hand_eye_all.launch.py`
- 手眼坐标、TCP、抓取姿态与候选规划：`single_apple_full_grasp.py`、`fixed_scan_real_verify.py`、`single_apple_full_grasp.yaml`
- 夹爪命令与结果：`modbus_gripper_ros.py`
- 抓取、起吊、转运、入盆、释放、回抽和 SCAN 复位
- 连续模式门禁、重试、超时与停止处理
- 9 月 11 日开发文档、管理脚本、现有测试与运行日志

共确认 5 项 P1、2 项 P2。当前实现不宜把 `--place-target basin` 视作已经完成安全闭环验证。最直接的物理风险是配置的安全转运高度没有进入实际轨迹，且规划场景没有盆体、完整桌面或被夹持果实；最直接的验证风险是 plan-only 流程会跳过整个入盆分支却仍记录“全部路径预检 PASS”。

## 审查发现

### P1 — 安全转运高度只被计算，实际携果轨迹直接走关节空间到盆口

- **证据位置**：
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.yaml:169`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1070`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1074`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1075`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1110`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1117`
  - `/home/li/hand_eye_calibration/apple_pick_v2/fixed_scan_real_verify.py:367`
  - `/home/li/hand_eye_calibration/apple_pick_v2/fixed_scan_real_verify.py:427`
- **触发条件**：任意 `place_target=basin` 的真实抓取在完成 LIFT 后进入盆工位。
- **问题**：配置定义 `transit_z_safe_m: 0.120`，执行代码也构造了 `transit_xyz` 和 `transit_pose`，但后续从未使用它们。下一条真实动作直接调用 OMPL 关节空间规划，从当前 LIFT 姿态运动到固定 `target_joints_rad`。因此规划器可以选择先下降、再横移或扫过工作区的轨迹，并不保证 TCP 或被夹持橘子全程处于安全高度。与此同时，规划场景只加入了以当前果实为中心、边长五个果径的局部支撑面和相机包络；没有完整桌面、盆体、藤蔓，也没有把橘子作为 attached collision object。
- **影响**：即便 MoveIt 返回无碰撞路径，机械臂、夹爪或被夹持橘子仍可能撞到真实桌面、盆沿或未建模障碍；与 `DEVELOPMENT.md:18`、`:98-100` 所称的“以安全高度转场”不一致。
- **建议**：将 LIFT 先以链式笛卡尔轨迹提升到经验证的 `transit_pose`，再从该终态规划到 `BASIN_APPROACH`；预检和执行必须复用同一段语义。给场景加入完整工作台和盆体几何，并在闭爪成功后把按实测半径膨胀的橘子附着到 TCP，释放后再解除附着。

### P1 — plan-only 预检默认跳过全部入盆路径，却仍报告“全部路径预检 PASS”

- **证据位置**：
  - `/home/li/hand_eye_calibration/apple_pick_v2/grasp_cli.py:128`
  - `/home/li/hand_eye_calibration/apple_pick_v2/grasp_cli.py:129`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:846`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:937`
  - `/home/li/hand_eye_calibration/scripts/robot_stack_manager.sh:315`
  - `/home/li/hand_eye_calibration/scripts/robot_stack_manager.sh:317`
  - `/home/li/hand_eye_calibration/apple_pick_v2/DEVELOPMENT.md:217`
  - `/home/li/hand_eye_calibration/apple_pick_v2/DEVELOPMENT.md:223`
- **触发条件**：执行管理脚本的 `grasp preview`，或用真正入口执行 `single_apple_full_grasp.py --preflight-only --operator-confirmed --place-target basin`。
- **问题**：非连续模式中，`auto_return` 只由 `--return-after-lift` 或 `--fully-autonomous` 决定。`prepare_round()` 又只在 `mode.auto_return` 为真时规划入盆路径。管理脚本的 preview 没有传这两个标志，所以 `LIFT -> BASIN_APPROACH -> BASIN_PLACE -> BASIN_APPROACH -> SCAN` 全部被跳过，随后代码仍写入“全部路径预检 PASS”。`--place-target basin` 本身不改变该门禁。
- **日志边界**：`results/single_apple_grasp_20260911_163123.json` 和 `results/single_apple_grasp_20260911_173055.json` 均显示 `return_after_lift_requested=false`，事件只覆盖抓取侧并随后记录 `PASS_PREFLIGHT_ONLY`；这能证明 `auto_return=false` 时所谓“全部路径”实际只到 LIFT。日志没有记录 `place_target`，且已知 173055 使用的是 `--place-target return`，因此不能单凭这两份日志声称当时请求了 basin。basin preview 的遗漏由上述 CLI 默认值、管理脚本参数和 `if mode.auto_return` 控制流直接证明。现有测试目录也没有 `place_target`、`basin` 或 `transit` 覆盖。
- **影响**：操作员会在从未验证新增入盆四段路径的情况下看到全链路 PASS，随后真机 `single` 才首次规划和执行这些段；plan-only 的安全价值被绕过。
- **建议**：把“是否规划放置段”从 `auto_return` 拆开；当 `preflight_only && place_target==basin` 时必须规划完整盆路径。只有实际覆盖了所有启用阶段才允许记录“全部路径预检 PASS”，日志中应逐段列出轨迹终态和 fraction。

### P1 — 连续入盆模式把盆口坐标当成上一抓取点，同一橘子会再次通过“新目标”门禁

- **证据位置**：
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1167`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1169`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1448`
  - `/home/li/hand_eye_calibration/apple_pick_v2/continuous_grasp_state.py:103`
  - `/home/li/hand_eye_calibration/apple_pick_v2/continuous_grasp_state.py:125`
  - `/home/li/hand_eye_calibration/apple_pick_v2/continuous_grasp_state.py:133`
- **触发条件**：`--continuous --place-target basin` 完成一轮动作后，原抓取点仍存在一个稳定检测，例如漏抓、滑脱、拿走后又把新橘子放在同一位置，或目标未被实际移动。
- **问题**：盆分支的 `RoundOutcome.release_center_m` 返回 `app_xyz`（盆口接近点），`continuous_main()` 随即把它交给 `gate.after_release()`。门禁于是计算“当前检测点到盆口”的距离，而不是“当前检测点到上一抓取点”的距离。工作区目标通常离盆口远大于 30 mm，因此原地目标只需稳定 2 秒并等待 3 秒倒计时，就会再次 READY。
- **隔离复现**：以抓取点 `(0.38, -0.30, -0.04)`、盆口 `(0.464, 0.006, 0.08)` 和当前 YAML 的 30 mm/2 s/3 s 配置运行纯状态机，距离为 `0.339252 m`，在 `t=5 s` 返回 `GateSnapshot(state='READY', ready=True)`。
- **影响**：违反文档 `/home/li/hand_eye_calibration/apple_pick_v2/DEVELOPMENT.md:236` 的“放入新果实后才开启下一轮”；在无人值守模式下可对同一位置自动重复整套真机动作，并把每轮都计为完成。
- **建议**：盆模式应把本轮冻结的 `prepared.target.corrected_center_m` 作为“已处理抓取点”，另设字段记录实际放料 TCP。连续门禁应按上一抓取点、目标 ID/类别和“目标消失后再出现”共同判定新一轮，而不是复用物理释放位置。

### P1 — 夹爪只有寄存器写入 ACK，没有抓持成功判定，漏抓/滑脱仍会计为成功

- **证据位置**：
  - `/home/li/hand_eye_calibration/modbus_gripper_ros.py:90`
  - `/home/li/hand_eye_calibration/modbus_gripper_ros.py:101`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.yaml:137`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.yaml:143`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1044`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1049`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1051`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1161`
- **触发条件**：夹爪命令被控制器接收，但橘子不在两指中间、闭爪途中打滑、夹持力不足或起吊后脱落。
- **问题**：`set_opening_mm()` 只等待 `/write_modbus_rtu_registers_result` 的布尔结果，这证明写命令被驱动接受，不证明夹爪到达位置、产生夹持力或仍持有果实。主流程固定等待 2.5 秒后就起吊，最后无条件写 `PASS_PICK_PLACE_BASIN_COMPLETE`。
- **影响**：运行日志和连续轮数会把空抓、滑脱、未入盆当作成功；连续模式随后可能继续下一颗，无法可靠识别失败或准确计数。此问题也影响 `place_target=return` 的实验结果判定。
- **建议**：读取夹爪实际位置/电流/堵转或力反馈并设置果径窗口；LIFT 稳定后用相机、夹爪状态或独立传感器确认果实仍被夹持。只有这两道门禁通过才能进入盆路径和增加 `completed_rounds`。

### P1 — 轨迹执行超时或 Ctrl+C 后不取消已接受的动作目标

- **证据位置**：
  - `/home/li/hand_eye_calibration/apple_pick_v2/fixed_scan_real_verify.py:290`
  - `/home/li/hand_eye_calibration/apple_pick_v2/fixed_scan_real_verify.py:300`
  - `/home/li/hand_eye_calibration/apple_pick_v2/fixed_scan_real_verify.py:302`
  - `/home/li/hand_eye_calibration/apple_pick_v2/fixed_scan_real_verify.py:305`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1500`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1512`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1617`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1629`
- **触发条件**：任一真实 `ExecuteTrajectory` 已被接受后，结果 future 超过 `motion_timeout_s`、通信中断、ROS shutdown，或操作员在运动中按 Ctrl+C。
- **问题**：`execute()` 在超时/无结果时直接抛错，未调用 action goal 的 `cancel_goal_async()`，也未发送驱动停止命令并等待停止确认。外层异常处理只把状态记为 STOPPED/FAULT，并在 finally 中移除支撑碰撞体和销毁节点。
- **影响**：本地程序已经打印/记录“失败即停”或退出时，控制器仍可能继续执行已下发轨迹；后续恢复脚本或人工判断可能基于错误的“已停”假设。
- **建议**：保留 ExecuteTrajectory goal handle；超时、KeyboardInterrupt 和 shutdown 都先取消 goal，等待取消结果和关节速度归零，必要时调用机器人驱动的 stop 接口。确认停止前不要移除规划场景；日志需区分“请求停止”和“已确认停止”。

### P2 — 9 月 11 日文档的三个 Ubuntu 原生命令调用了无执行入口的模块，会静默退出 0

- **证据位置**：
  - `/home/li/hand_eye_calibration/apple_pick_v2/grasp_cli.py:26`
  - `/home/li/hand_eye_calibration/apple_pick_v2/grasp_cli.py:89`
  - `/home/li/hand_eye_calibration/apple_pick_v2/grasp_cli.py:152`
  - `/home/li/hand_eye_calibration/apple_pick_v2/DEVELOPMENT.md:217`
  - `/home/li/hand_eye_calibration/apple_pick_v2/DEVELOPMENT.md:227`
  - `/home/li/hand_eye_calibration/apple_pick_v2/DEVELOPMENT.md:233`
- **触发条件**：按 DEVELOPMENT.md 6.2 执行预检、单次抓取或连续抓取命令。
- **问题**：文档运行 `python grasp_cli.py ...`，但该文件只定义 parser 和 `resolve_run_mode()`，没有 `main()` 或 `if __name__ == '__main__'`，也不会转发到 `single_apple_full_grasp.py`。隔离执行 `grasp_cli.py --definitely-invalid` 无输出并返回 exit code 0，说明连未知参数都不会被解析。即使改成真实入口，文档的预检命令还缺少代码要求的 `--operator-confirmed`（`grasp_cli.py:137-138`）。
- **影响**：操作员会把无输出的成功退出误认为预检/任务已完成，实际没有连接 ROS、没有规划，也没有动作；真实入盆与连续命令同样完全不执行。
- **建议**：文档统一调用 `single_apple_full_grasp.py`，或者给 `grasp_cli.py` 增加明确委托入口；预检命令补齐授权标志。加入 subprocess 测试，断言 `--help` 有输出、无效参数非零、三种文档命令确实进入预期模式。

### P2 — 释放点相对盆沿的 Z 方向说明反了，当前 TCP 实际在盆沿上方 3 cm

- **证据位置**：
  - `/home/li/hand_eye_calibration/apple_pick_v2/DEVELOPMENT.md:82`
  - `/home/li/hand_eye_calibration/apple_pick_v2/DEVELOPMENT.md:83`
  - `/home/li/hand_eye_calibration/apple_pick_v2/DEVELOPMENT.md:86`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.yaml:164`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.yaml:165`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1121`
  - `/home/li/hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py:1138`
- **触发条件**：按当前配置执行 `BASIN_APPROACH -> BASIN_PLACE` 并开爪。
- **问题**：同一 `base_link` 中，盆沿为 `z=-0.050 m`，释放 TCP 为 `z=-0.020 m`。因为 `-0.020 > -0.050`，释放点是盆沿**上方** 30 mm，不是文档写的“低于盆沿 3 cm”。代码按 YAML 绝对位姿执行，没有任何盆沿/盆底一致性校验。
- **影响**：实际动作是从盆沿上方开爪掉落，与文档所称的低位入盆和轻柔滑入不同；操作员也可能按错误说明摆放盆体。若真正把符号改为 `-0.080` 来实现“低于 3 cm”，又会距离文档所述盆底 `-0.087` 仅 7 mm，必须先核对 TCP、指尖和果实中心的几何关系，不能直接改数值。
- **建议**：明确三个 Z 值分别代表盆沿、盆底、TCP/果实中心中的哪一个；从物理几何计算安全释放高度并增加配置启动校验，例如 `rim_z + fruit_clearance <= release_tcp_z <= approach_z`。修正文档的上下方向和实际落差。

## 已确认正确的部分

- `apple_hand_eye_all.launch.py:100-101` 会把目标参数传给视觉节点，默认值在 `:126-128` 为 `orange`；视觉节点使用 COCO 零基类别 `ORANGE_CLASS_ID = 49`，见 `apple_center_localizer_v2.py:38`、`:81-82`。
- RGB/对齐深度时间差超过 100 ms 会被拒绝，检测框贴真实图像边缘时也不会发布抓取中心。
- `tcp_link` 在当前已安装 RM65 URDF 中由 Link6 正 Z 偏移 0.138 m；离线 FK 计算当前 `target_joints_rad` 得到 TCP `[0.463991, 0.005981, 0.079984] m`，与 `approach_xyz_m=[0.464, 0.006, 0.080]` 的每轴误差均小于 0.02 mm，姿态也与 yaw `2.847`、pitch `0.1745` 一致。
- GRASP 到 LIFT 使用基座 `+Z` 的 70 mm 笛卡尔路径，夹爪闭合后等待 2.5 秒才起吊；这符合 9 月 10 日实验记录修正。
- 现有 25 个单元测试在加载 ROS 环境、但测试过程未访问硬件的条件下全部通过，4 个关键 Python 文件 AST 解析通过，`git diff --check` 通过。

## 验证边界与剩余缺口

- 按任务约束未启动或修改 ROS 节点，未发送机械臂、夹爪或设备命令，也未执行真机入盆。
- 截至审查时，`apple_pick_v2/results` 中没有任何状态为 `PASS_PICK_PLACE_BASIN_COMPLETE` 的日志，因此尚无日志证明新增 basin 全流程完成。`single_apple_grasp_20260911_173253.json` 已记录 `place_target=return` 路径的 `PASS_PICK_RETURN_COMPLETE`、LIFT、原位 RELEASE 和返回 SCAN；它证明 return 动作序列完成，实际橘子是否稳定抓起并成功放回仍需现场观察者确认，因为代码没有抓持反馈。
- 未获得盆口直径、盆壁形状、盆体相对基座的实测误差、橘子携带时的真实包络和完整现场障碍模型，因此无法证明盆沿净空和转场碰撞安全。
- 现有测试没有覆盖新增 basin 分支、连续 basin 的上一目标门禁、动作取消、夹爪反馈或文档命令可执行性。
