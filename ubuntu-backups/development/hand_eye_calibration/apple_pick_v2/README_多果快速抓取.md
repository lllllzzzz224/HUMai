# 同类多果：自动选择、限时规划、放回

入口 `multi_fruit_grasp.py` 默认仅规划。现有单果、连续入口及盆子配置保持原样。
新入口仅使用原位放回模式，按稳定目标置信度从高到低自动处理，无每轮 Enter。
已尝试的果实在本会话中保持排除，不代表实际抓取成功。人可在可信工作区清空联锁允许的流程中取走已处理果实。

## 无运动验证

从仓库根目录执行（可用于开发机）：

```bash
source /opt/ros/jazzy/setup.bash
source /home/li/ros2_ws/install/setup.bash
/home/li/ros_yolo_env/bin/python apple_pick_v2/multi_fruit_grasp.py --help
/home/li/ros_yolo_env/bin/python apple_pick_v2/multi_fruit_grasp.py --replay --output /tmp/multi_fruit_replay.json
PYTHONPATH="apple_pick_v2:$PYTHONPATH" /home/li/ros_yolo_env/bin/python -m unittest discover -s apple_pick_v2/tests -v
```

回放为软件模拟；按 `.95 → .85 → .75` 排序并排除，未创建 ROS 节点、未调用模型/MoveIt/设备，不能作为真机耗时数据。

## 视觉与在线仅规划

已有视觉启动环境可加参数；不应为了切换模式重复启动正在运行的驱动：

```bash
ros2 launch ./apple_pick_v2/apple_hand_eye_all.launch.py target:=orange publish_candidates:=true inference_hz:=4.0
```

也可在已有相机/TF 环境仅运行视觉节点：

```bash
/home/li/ros_yolo_env/bin/python apple_pick_v2/apple_center_localizer_v2.py --target orange --publish-candidates true --inference-hz 4.0
```

新 runner 要求机械臂已经稳定在配置 SCAN 位，**不会自动移动到 SCAN**。
下列命令不配置夹爪、不发运动命令；它会向现有 MoveIt 场景加入支撑/相机保护及其他果实球形碰撞体：

```bash
/home/li/ros_yolo_env/bin/python apple_pick_v2/multi_fruit_grasp.py --target orange --planning-budget-s 2.0 --max-rounds 3 --output /tmp/multi_fruit_plan.json
```

默认保持监听，`--max-rounds 3` 限制为三次目标规划/尝试。无安全目标时持续等待（Ctrl-C 退出），不会把全部已尝试或阻塞误报为场景空。
帧率参数只是调用上限；模型运行慢于 4 Hz 时不会产生 4 Hz 的有效观测。默认源图像最大年龄 1 s，可用 `--max-capture-age-s` 调整。

## 执行与联锁契约

实际运动需要同时提供 `--execute --operator-confirmed --interlock-source <已配置安全输入源ID>`，且持续收到健康、清空、新鲜的输入。例如系统已经有可靠的 `cell-safety-controller` 集成后：

```bash
/home/li/ros_yolo_env/bin/python apple_pick_v2/multi_fruit_grasp.py --target orange --execute --operator-confirmed --interlock-source cell-safety-controller --workspace-clear-topic /apple_pick_v2/workspace_clear --planning-budget-s 2.0 --output /tmp/multi_fruit_execution.json
```

这里未实现或验证安全控制器；不能使用手工 `ros2 topic pub`、果实消失或固定 Bool 代替可靠清空信号。软件源 ID 只是路由检查，不提供身份认证或安全认证，部署方需保证 ROS 网络可信及输入与实际工作区状态相符。

`/apple_pick_v2/workspace_clear` 为 `std_msgs/String` JSON：

```json
{"version":1,"source":"cell-safety-controller","stamp_s":1789113600.123,"sequence":42,"healthy":true,"clear":true}
```

时间戳必须由可靠输入源在测量时生成，使用与 ROS 节点一致且同步的时钟；`sequence` 和源时间戳严格递增。默认源数据年龄及本地单调接收年龄均不超过 0.5 s（`--interlock-max-age-s`）。缺失、未知来源、重放、未来时间戳、false、健康状态异常均禁止动作。输入断开/过期时，夹爪后续发布被禁止、已接受运动请求取消，并锁定会话；取消请求不是停止证明。未确认远端完成或物理停止时记录 `FAULT_LATCHED_STOP_UNCONFIRMED`，不自动继续。夹爪没有已验证的取消/急停接口，因此外部安全链不可省略。

## 观测、选择与规划边界

视觉输出 `/apple_pick_v2/fruit_candidates`，类型 `std_msgs/String`：

```json
{"version":1,"frame":"base_link","capture_stamp_s":1789113600.123,"valid":true,"detections":[{"class_id":49,"class_name":"orange","confidence":0.95,"center_xyz_m":[0.3,0.0,0.1],"radius_m":0.03}],"source_model_duration_s":0.045,"source_localization_duration_s":0.01}
```

使用**源 RGB 捕获时间**；保留同一模型、深度/球拟合、TF 与原始 base 坐标，不把视觉 XY 校正写回观测。类别 apple=47、orange=49。重叠 tile 在 2D 定位前去重，3D 后再次去重。未定位的完整框、不能由完整框覆盖的边缘框、深度或 TF 无效使整个快照无效；空检测与无效快照不同。

眼在手上：只接纳机械臂处于 SCAN、关节新鲜且稳定至少 0.1 s 后捕获的快照。运动中的快照不用于空场景或目标身份更新；运动/非清空边界后重新积累至少两帧稳定证据。`/joint_states` 必须提供与 ROS 时钟一致的新鲜源时间戳；缺失或陈旧源戳无法用于自主模式。

同类一对一空间关联；目标移动后失效旧计划，唯一可确认关联会在新位置重新稳定；多个可能关联或已尝试果实大幅移动造成身份不确定时保守隔离。不会给不确定对象一个新的可抓 ID。身份/排除仅保留当前进程会话，重启前须人工确认场景。阻塞候选默认冷却 30 s，最多尝试两次。空场景需默认两帧、跨至少 0.05 s（`--empty-frames` / `--empty-interval-s`）；空场景也只是当前视野检测为空，非实际工作台清空证明。

每目标默认 **2 s 共享绝对单调规划截止时间**，覆盖真实原有 IK、多种子独立碰撞检查、场景读写、OMPL、Cartesian、返回 SCAN 完整预检和最后检查。没有为每个子请求重新计时。超时可能没有足够轨迹，正常记录超时，不保证 1–2 s 必定规划成功。服务协议无法取消远端服务工作，超时且完成不明会锁定，而非立即切到下一个目标；已接受规划 action 请求取消后最多额外排空 0.5 s，其耗时不冒充在 2 s 预算内。

完整轨迹仅在同一轮复用，执行前对实际关节起点、当前状态碰撞、完整规划场景与冻结目标/邻居重新检查；夹爪配置/打开后第一条轨迹前再次验证新鲜观测。轨迹从实际起点偏离超过 0.02 rad 时拒绝。未增加物理速度/加速度、未改变上抬高度，闭爪仍按现有配置等待 **2.5 s**。在线仅规划也要求正常 MoveIt 服务，回放不能代替它。

## 测量与验证范围

日志包括单调开始/结束时间、秒级高分辨率时长、状态、目标和类别、尝试次数。`inclusive:true` 表示包含子阶段，`parent_id` 标明嵌套：不要把 preflight 与内部 IK/OMPL 时长相加。执行、到位稳定、夹爪动作/等待分别记录；模型和定位时长来自视觉源，接收/选择/预检/再验证来自 runner。等待状态按连续区间记录；日志按状态/轮次变更或最多每 2 s 原子写入。

已完成纯状态、ROS 边界及真实 `prepare_round` 配合外部 ROS 双件测试。开发过程未连接并运行真机动作、未测试真实安全控制器、取消停止时延、模型吞吐、多果遮挡与轨迹可执行性；`COMMANDED_RETURN_COMPLETE` 仅为轨迹/动作完成，不是夹持成功检测。使用前仍需受控真机验证，不能把本次软件测试的毫秒耗时当成提速结果。
