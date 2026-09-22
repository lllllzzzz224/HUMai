# 静态障碍预览（禁止用于执行）

本模块把已保存的 RGB-D 转成三维表面体素，在独立 MoveIt 中写入并读回核对。树枝、树叶、目标果实和桌面有效表面均保留为碰撞几何。当前没有树干/树枝/树叶语义分类，也没有穿叶或移枝动作。所有输出固定 `executable=false`。

## 使用范围

- `obstacle_geometry.py`：无 ROS/硬件依赖，投影、三态空间查询、体素与增量计算。
- `obstacle_preview.py`：离线重放和 PLY/NPZ/JSON 导出，不接触相机或机器人。
- `obstacle_scene_ros.py`：场景服务、读回、录制姿态显示；只有显式提供有效候选时才创建仅规划动作客户端。
- `obstacle_preview.launch.py`：仅模型发布、MoveIt、可选 RViz，硬编码禁用轨迹执行和控制器管理。只允许 localhost、ROS 域 87，不启动任何驱动或控制器。

当前实现只适合静态观察预览。看到的表面进入碰撞检测，不代表未看到的背面已经建模。深度为零、视野外、物体背后和指定范围外均是未知。12 mm 体素、4 mm 膨胀是预览参数，尚无实测误差预算。模型中的夹爪盒宽80 mm，现有开爪设置85 mm，真实张开包络仍未验收。相机盒沿用基线近似尺寸110×50×50 mm，支架未测量；当前桌面保留为观测体素，没有另造一个“已测准”的桌面平面。

## 离线生成

在本目录运行，输入采集目录与输出目录必须不同，非空输出目录拒绝覆盖：

```bash
/home/li/anaconda3/envs/yolo11/bin/python obstacle_preview.py \
  --capture /absolute/path/to/capture.json \
  --output /absolute/path/to/new_preview \
  --config obstacle_preview.yaml
```

输出 `scene.json`、`quality.json`、`observation.ply`、`observation.npz`。支持已保存的 SDK RGB-D 数据和带控制器前后读取、URDF FK、手眼、光学变换的注册采集。后者必须满足时间夹持、静止阈值和变换链一致性。只有矩阵但缺少同步证据的输入会拒绝；仅 RGB-D 的输入保留相机坐标系，禁止写为基座场景。

可用 `--target-mask /absolute/path/mask.npy` 标记同帧诊断片段；该掩膜只标注，不删除任何几何，也不代表完整果体或已验证抓取候选。源采集/深度哈希、时间和变换来源均随结果保存。

## 隔离 MoveIt 预览

两个终端都设置：

```bash
source /opt/ros/jazzy/setup.bash
source /home/li/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=87
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR=/tmp/obstacle_preview_ros_logs
unset ROS_STATIC_PEERS
```

先确认域 87 没有其他实验节点。终端一：

```bash
ros2 launch ./obstacle_preview.launch.py rviz:=true
```

终端二：

```bash
/home/li/ros_yolo_env/bin/python obstacle_scene_ros.py \
  --scene /absolute/path/to/base_preview/scene.json \
  --output /absolute/path/to/new_result.json \
  --recorded-replay --hold-seconds 180
```

脚本读取 MoveIt 的 `allow_trajectory_execution=false` 后才写入。每个体素使用 `auto_obstacle_*` ID，增量场景不清空其他模块几何、附着体或 ACM。写入后比对几何、附着体、ACM、Octomap、固定变换及模型缩放/膨胀；不一致则失败。单次写入只新增/更新、保留旧占据，不自动删除；纯几何模块已经提供整格有效自由观测证据，后续连续更新仍需单独接入与验证。

`/offline_preview/joint_states` 的时间戳是显示回放时间，值来自记录的控制器姿态；标记为 `RECORDED_SNAPSHOT_NOT_LIVE`。RViz 同时显示原采集时间、点云和实际监控场景。绝不能将这个话题接给真机执行程序。结果文件在显示循环结束后写入，原始场景读回提前存为 `.readback.json`。用 Ctrl-C 停止两终端；没有后台硬件控制进程需要恢复。

## 仅规划候选

可选 `--candidate /absolute/path/pregrasp.json`。候选必须来自同一快照的已审核预抓取求解，含 `kind: pregrasp`、`validated: true`、`scene_version`、`source`、六轴 `joint_positions_rad`。这些字段记录上游验收结论，本模块不从检测框制造抓取姿态。没有候选时输出 `NO_VALIDATED_PREGRASP_CANDIDATE`。

规划目标强制 `plan_only=true`，显式采用记录的起始关节状态；检查起点有效性、实际轨迹起终点、每0.01 rad以内的插值状态、规划前后场景摘要。另对各连杆原点和 TCP 查询观测覆盖，给出自由/占用/未知样本数。该采样不覆盖全部连杆表面或连续扫掠体，`full_robot_swept_volume_certified=false`，因此 `PLAN_PREVIEW` 也不等于真机可执行。

## 验证

```bash
source /opt/ros/jazzy/setup.bash
source /home/li/ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/obstacle_preview_ros_logs PYTHONPATH="$PWD:$PYTHONPATH" \
  /home/li/ros_yolo_env/bin/python -m unittest discover -s tests -q
```

使用仓库原有的 unittest 风格，不需要额外安装 pytest。纯几何和导出测试也可用 yolo11 环境单独运行。现场相机不可用时只能验证已保存数据，不能将旧帧、旧姿态或合成规划当作当前实物成功。
