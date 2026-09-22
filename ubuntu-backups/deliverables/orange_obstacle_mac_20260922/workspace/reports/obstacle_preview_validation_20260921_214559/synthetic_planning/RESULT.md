# 人工合成 plan-only 集成验证

此结果仅证明独立 ROS 域 87 中的规划、障碍读回与离散插值碰撞检查链路。不是橘子预抓取验证，不授权真实执行。

- 背景：3071 个历史录制障碍体素，以及附着的 auto_obstacle_camera_guard。
- 起点：录制关节姿态；人工终点只把 joint1 从 2.4988227967 rad 改为 1.8988227967 rad。
- 无新增障碍：起终有效；规划耗时 0.010258368 s，21 个轨迹点，77 个插值检查点全部有效。
- 绕行：添加边长 0.025 m 的 auto_obstacle_synthetic_detour，中心为 [0.1834601714, -0.2246103418, 0.3124883255] m。起终仍有效；直线关节插值 21 点中 13 点与该盒碰撞。
- RRTConnect 绕行成功：规划耗时 0.031846623 s，31 个轨迹点，117 个插值检查点全部有效，最大检查步长 0.0098 rad。有限采样不等同于连续路径安全证明。
- 全阻挡：中心 [0,0,0.4] m、边长 2 m 的盒使起点和终点均碰撞，动作返回 ABORTED(status=6)，MoveIt error_code=99999。此项验证碰撞输入被拒绝，不代表端点有效但不可达情形的完整性测试。
- 全程 goal.plan_only=true；动作图只有 /move_action 服务端，RViz 是 /execute_trajectory 客户端，没有该执行动作服务端。
- finally 仅清理本脚本拥有的两个 auto_obstacle_synthetic_* ID。恢复后场景摘要与基线一致，world、attached、ACM、Octomap、变换、padding、scale、colors 均通过摘要检查。
- 恢复摘要：1101fe65ce79cd4343c1182ad9bedecbe01e8e03b8b1b1aaff1996dfe86f8384。

原始证据：report.json、*.goal.json、*.action_result.json、*.samples.json、baseline_scene.json、detour_scene.json、blocked_scene.json、restored_scene.json、graph_before.json、graph_after.json。可复现脚本为 run_synthetic.py。运行日志在 /tmp/obstacle_synthetic_plan_validation.log。
