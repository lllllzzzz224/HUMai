# 苹果识别 RViz 启动说明

网线ip连接先临时恢复，执行：

  sudo ip address replace 192.168.1.100/24 dev enx00e04c3a4178
  sudo ip link set enx00e04c3a4178 up
  ping -c 3 192.168.1.18

  看到 0% packet loss 就表示电脑又能读到机械臂了。不要给有线网卡配置网关；默认网
  络仍走 Wi‑Fi，所以两者能同时使用。

  这个临时配置在重启后可能消失。要永久保存，可执行一次：

  sudo nmcli connection add type ethernet ifname enx00e04c3a4178 \
    con-name RM65-Ethernet \
    ipv4.method manual ipv4.addresses 192.168.1.100/24 \
    ipv4.never-default yes ipv6.method disabled \
    connection.autoconnect yes

  sudo nmcli connection up RM65-Ethernet

  之后启动真机栈前再测试：

  ping -c 3 192.168.1.18


本说明用于将“RealSense 识别苹果”打通到 RViz：在 `base_link` 坐标系中显示苹果三维中心、苹果球体 Marker、实时点云和机械臂模型。

`apple_rviz_detector.py` 是**纯视觉节点**：它不会发布机械臂运动、MoveIt 执行或夹爪控制命令。初次使用只完成到“识别 + RViz 验证 + 规划预览”为止。

## 0. 启动前确认

- 机械臂、相机和夹爪附近无人；急停保持可触及。
- 机械臂与电脑网络、ROS 驱动已可通信。
- 相机固定在机械臂末端，且未移动过相机、支架或工具坐标系；否则已有手眼外参失效，必须重新标定。
- 当前桌面在 `base_link` 下的高度是 `-0.090 m`。若桌面高度已改变，记录实际值并在第 4 步用 `--table-z` 覆盖。

不要同时启动两份 `rm_bringup` 或两份 RealSense 驱动；重复节点会导致 TF 和图像数据混乱。

## 1. 启动机械臂 ROS 驱动

打开终端 1：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rm_bringup rm_65_bringup.launch.py
```

保持该终端运行。另开终端检查：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic info /joint_states
ros2 run tf2_ros tf2_echo base_link Link6
```

应看到 `/joint_states` 有发布者，且 `base_link -> Link6` 持续输出变换。

## 2. 启动 RealSense ROS 驱动

若相机驱动尚未启动，打开终端 2：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true \
  enable_sync:=true pointcloud.enable:=true publish_tf:=true
```

保持该终端运行。检查关键话题：

```bash
ros2 topic list -t | grep -E 'color/image_raw|aligned_depth_to_color/image_raw|depth/color/points'
```

需要至少有以下话题：

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_color/camera_info
/camera/camera/depth/color/points
```

## 3. 发布手眼静态 TF

打开终端 3：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch /home/li/hand_eye_calibration/hand_eye_static_tf.launch.py
```

此文件不会修改 `hand_eye_result.yaml`。它将已有手眼外参、`Link6 -> ArmTip` 固定偏移和 RealSense 内部坐标变换合成为：

```text
base_link -> ... -> Link6 -> camera_link -> camera_color_optical_frame
```

确认完整坐标链已经可用：

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

必须持续输出平移和旋转；若显示 `frame does not exist`，不要进入下一步，先检查前三个终端。

## 4. 启动苹果三维识别节点

将苹果放在桌面上，先让机械臂保持静止。打开终端 4：

```bash
cd ~/hand_eye_calibration
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
conda activate yolo11
python apple_rviz_detector.py
```

若桌面高度不是 `-0.090 m`，例如实际为 `-0.085 m`：

```bash
python apple_rviz_detector.py --table-z -0.085
```

节点会在终端打印类似：

```text
apple centre base=[0.417, -0.415, -0.056] m
```

并发布以下只读可视化话题：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/apple_rviz/annotated_image` | `sensor_msgs/Image` | YOLO 分割结果，苹果掩膜为绿色。 |
| `/apple_rviz/centre` | `geometry_msgs/PoseStamped` | 苹果球心，坐标系为 `base_link`。 |
| `/apple_rviz/markers` | `visualization_msgs/MarkerArray` | 红色苹果球体、置信度、半径和拟合误差标签。 |

检测节点不创建 `/rm_driver/*_cmd`、MoveIt 执行或夹爪命令发布者。

## 5. 配置 RViz

启动 RViz（若机械臂 bringup 已自动打开 RViz，直接使用该窗口）：

```bash
rviz2
```

在左侧 `Displays` 配置：

1. `Global Options -> Fixed Frame` 设置为 `base_link`。
2. 添加 `RobotModel`，确认模型与真机当前姿态一致。
3. 添加 `TF`，勾选 `Show Axes`；应能看到 `Link6`、`camera_link` 和 `camera_color_optical_frame`。
4. 添加 `PointCloud2`，Topic 设为 `/camera/camera/depth/color/points`。
5. 添加 `Image`，Topic 设为 `/apple_rviz/annotated_image`。
6. 添加 `MarkerArray`，Topic 设为 `/apple_rviz/markers`。
7. 添加 `Pose`，Topic 设为 `/apple_rviz/centre`。

正确结果：

- Image 中苹果被绿色掩膜覆盖；没有掩膜时不要继续规划。
- 三维视图中的红色球体包住真实苹果，且球心没有落在桌面内或苹果外。
- 点云里的苹果、红球和真实苹果位置一致。
- 低速、小范围手动移动机械臂时，`camera_link` 与点云随机械臂移动，红球仍贴在真实苹果上。

若红球与真实苹果相差几厘米以上，停止后续操作。优先检查相机支架是否松动、桌面高度是否正确、静态 TF 是否只有一个发布者；不要通过随意修改抓取偏移“补偿”手眼标定错误。

## 6. 读取已验证的苹果中心

待 RViz 中红球稳定覆盖苹果后，读取一个中心坐标：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo --once /apple_rviz/centre
```

记录输出中的：

```text
pose.position.x
pose.position.y
pose.position.z
```

此点是苹果球心，单位为米，坐标系是 `base_link`。

## 7. 规划预览（不执行真机）

将第 6 步的 `X Y Z` 替换为实际读数：

```bash
cd ~/hand_eye_calibration
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
conda activate yolo11
python apple_candidate_plan_review.py --centre X Y Z
```

该程序要求 `apple_grasp_config.yaml` 中 `moveit.execute: false`，只计算 SCAN、PREGRASP、GRASP 三个候选位姿并向 RViz 发布 `/display_planned_path`；它不会执行机械臂轨迹，也不会操作夹爪。

在 RViz 的 MotionPlanning 面板中检查规划路径：

- 轨迹没有穿过桌面、苹果或夹爪本体；
- PREGRASP 位姿在苹果外侧且有足够距离；
- 机械臂没有出现不合理的大幅绕行；
- 所有候选位姿均在工作空间内。

只有以上检查都通过，才可以考虑使用项目中单独的 `apple_execute_scan_pregrasp.py` 做低速、双重文字确认的 `SCAN -> PREGRASP` 测试。该步骤也不闭合夹爪；不要将“识别成功”直接等同于“允许抓取”。

## 常见问题

| 现象 | 检查与处理 |
|---|---|
| `base_link` 或 `camera_color_optical_frame` 不存在 | 确认机械臂、RealSense、手眼静态 TF 三个终端都在运行。 |
| RViz 中点云不可见 | Fixed Frame 设为 `base_link`；确认第 3 步的 TF 查询持续成功。 |
| 没有苹果 Marker | 查看 `/apple_rviz/annotated_image` 是否出现绿色掩膜；调整光照、距离或 `--confidence`。 |
| Marker 在苹果外 | 保持机械臂静止，复核相机固定件、桌面 `--table-z` 和手眼标定。 |
| 同一 TF 被重复发布 | 每次只运行一个 `hand_eye_static_tf.launch.py`；先结束旧实例再启动新实例。 |
