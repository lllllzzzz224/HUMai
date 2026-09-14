# 机械臂视觉采摘入盆系统（RM65 Pick-and-Place to Basin）开发与工程技术文档

> **文档状态**：已落地并完成真机预检验证（Production Ready）  
> **适用机型**：睿尔曼 RM65-B 6自由度机械臂 + RS485 Modbus RTU 电动夹爪 + Intel RealSense D435i  
> **更新日期**：2026-09-11  
> **核心仓库**：`/home/li/hand_eye_calibration/apple_pick_v2`  
> **运行环境**：Ubuntu 24.04 LTS / ROS 2 Jazzy / Python 3.12 (Conda: `ros_yolo_env`)

---

## 1. 系统概述与业务目标 (System Overview)

### 1.1 核心业务目标
本系统面向果园/室内果实采摘与分拣场景，实现从**“固定俯视识别”**到**“精准三维抓取”**再到**“安全搬运入盆”**的闭环自主作业：
1. **自动感知**：机械臂停驻在固定俯视工位（SCAN），通过 Eye-in-Hand RGB-D 相机实时识别工作台上的果实（苹果/柑橘），解算果实球心在机械臂基座标系（`base_link`）下的三维坐标。
2. **轨迹规划**：采用 MoveIt 2 + OMPL，结合 4 种子多阶段逆运动学（IK）搜索，规划从当前工位到果实上方的预抓取位（PREGRASP）、下潜抓取位（GRASP）的笛卡尔直线轨迹。
3. **稳妥采摘**：电动夹爪闭合抓紧果实后，严格执行垂直起吊（LIFT），向上提起 7 cm，使果实彻底脱离果梗及支承平面。
4. **安全入盆（核心升级）**：机械臂携果实以安全高度转场至工作台左侧的落料容器（浅绿色塑料盆），低位平稳入盆后释放夹爪，避免果实碰撞磕伤。
5. **安全复位**：机械臂原路回抽并安全回归固定俯视工位（SCAN），准备进入下一轮采摘。

### 1.2 运行模式对比
| 模式参数 (`--place-target`) | 运行逻辑 | 应用场景 |
| :--- | :--- | :--- |
| **`basin` (默认)** | `SCAN -> PREGRASP -> VERIFY -> GRASP -> 闭爪 -> LIFT -> BASIN_APPROACH -> BASIN_PLACE -> 开爪 -> BASIN_APPROACH -> SCAN` | **实际采摘生产模式**：果实收获至容器，持续采摘 |
| **`return`** | `SCAN -> PREGRASP -> VERIFY -> GRASP -> 闭爪 -> LIFT -> 原位 RELEASE -> 开爪 -> PREGRASP -> SCAN` | **手眼标定与精度验证模式**：果实放回原处，测试闭环复现性 |

---

## 2. 硬件拓扑与系统架构 (Hardware & System Topology)

### 2.1 硬件拓扑
```text
[ 工控主机 (Ubuntu 24.04, ROS 2 Jazzy, IP: 10.77.0.2) ]
   │
   ├── [USB 3.0] ────────────────► Intel RealSense D435i (Eye-in-Hand 深度相机)
   │
   ├── [以太网口 enx00e04c3a4178] ─► 睿尔曼 RM65-B 机械臂控制器 (IP: 192.168.1.18)
   │                                   │ (末端工具接口 Tool IO)
   │                                   └── [RS485 Modbus RTU, 24V] ─► 两指电动平行夹爪
```

### 2.2 核心网络与环境变量
- **ROS 2 隔离域**：`export ROS_DOMAIN_ID=42`（避免实验室/局域网内其他设备话题串扰）。
- **Python 隔离环境**：`/home/li/ros_yolo_env/bin/python`（包含 Ultralytics YOLOv11、PyTorch 2.x、SciPy、OpenCV-Python）。
- **ROS 工作空间**：`/home/li/ros2_ws`（提供 `rm_driver`、`rm_control`、`rm_description`）。
- **机械臂通信**：RJ45 静态网段 `192.168.1.x`，控制器端口默认 `8080` (API) 与标准 ROS 2 驱动通信。

---

## 3. 空间坐标系与标定规范 (Frames & Spatial Calibration)

### 3.1 坐标系拓扑树（TF2 关系）
```text
base_link (机械臂安装基底)
  └── Link1 ── Link2 ── Link3 ── Link4 ── Link5 ── Link6 (法兰盘)
                                                    ├── tcp_link (抓取指尖中心, Link6 +Z 0.138m)
                                                    └── camera_link -> camera_color_optical_frame
```

### 3.2 物理 TCP 定义与唯一规范
- **法兰末端**：`Link6`。
- **物理工具中心点 (TCP)**：
  - 夹爪抓取主轴与 `Link6` 正 Z 轴严格同轴平行。
  - 抓取指端物理中心固定在 `Link6` 正 Z 方向 **`0.138 m` (138 mm)** 处。
  - **开发红线**：`tcp_link` 已在 URDF 与 MoveIt 模型树中注册发布，所有算法位姿规划均以 `tcp_link` 为基准。**严禁在代码中人为多扣减或增加夹爪长度**。

### 3.3 关键工位空间参数定义

#### A. 固定俯视工位 (SCAN)
机械臂停驻在视场中心上方，垂直向下俯瞰工作台：
- **关节角 (rad)**：`[2.51015, 0.35452, 0.64713, 0.06147, 2.04554, -0.07143]`
- **对应 TCP 空间坐标**：$X \approx 0.380\text{ m}, Y \approx -0.100\text{ m}, Z \approx 0.400\text{ m}$

#### B. 采摘工作区范围 (Grasp Workspace)
- **有效视觉范围**：$X \in [0.20, 0.58]\text{ m}, Y \in [-0.36, 0.00]\text{ m}$
- **安全下探深度限制**：桌面高度通常处于 $Z \approx -0.140\text{ m}$，抓取安全阈值 $Z_{\text{grasp}} \ge Z_{\text{table}} + 2\text{ mm}$。

#### C. 落料盆工位 (Basin Place Zone)
实测标定塑料盆中心物理空间位姿：
- **视觉隔离哲学**：盆子放置在白纸左侧（$Y \ge 0$），**绝对避开 SCAN 相机的俯视视场**。这样采摘投入盆中的果实不会被 YOLO 识别，杜绝二次抓取已采果实的死循环。
- **盆体几何尺寸**：
  - 盆外沿最高点：$Z \approx -0.050\text{ m}$
  - 盆底中心：$X = 0.464\text{ m}, Y = 0.006\text{ m}, Z = -0.087\text{ m}$
- **入盆轨迹控制点**：
  - **接近/过渡点 (`basin_approach`)**：`[0.464, 0.006, 0.080] m`（高于盆沿 13 cm）
  - **释放落料点 (`basin_place`)**：`[0.464, 0.006, -0.020] m`（低于盆沿 3 cm，距盆底 6 cm，果实轻柔滑入盆底）
  - **前向舒展逆解关节配置 (Forward Reachable IK Seed)**：
    ```yaml
    target_joints_rad: [0.0484, -1.6343, 0.7521, -0.0675, -2.0939, -2.8371]
    ```
    *注：此关节配置经实测求解，避开了 RM65 在 $X > 0.45\text{ m}$ 时的奇异点，且 Joint 1 仅为 $0.048\text{ rad}$，无大幅度甩臂动作。*

---

## 4. 核心算法与运动规划技术决策 (Key Technical Decisions)

### 4.1 为什么需要 LIFT（垂直起吊）阶段？
在抓取果实后，如果直接向落料区规划横向移动轨迹，机械臂极易横向拖拽果实，导致果梗拉断伤树，或与桌面发生剧烈侧向摩擦损坏舵机。  
**工程方案**：抓手闭合后，先沿 `base_link` 的 $+Z$ 轴纯笛卡尔直线上升 $7\text{ cm}$（`targets.lift_m: 0.07`）。

### 4.2 为什么转场规划必须显式指定 `start_state`（链式规划连续性）？
在 ROS 2 / MoveIt 2 中，如果多次调用 `move_group.plan()`，默认是以机械臂当前的“物理瞬态”或初始种子状态作为起点。在离线全链路预检（`--preflight-only`）中，机械臂并未实际运动，若直接规划下一阶段，规划器将默认从静态起点开始，导致前后两段轨迹的关节状态不连续，产生突变。  
**工程方案**：
```python
# single_apple_full_grasp.py 中的链式起点绑定
basin_approach_plan = node.plan(
    basin_approach_pose,
    "LIFT_TO_BASIN_APPROACH",
    start_state=lift_trajectory.points[-1],  # 显式将上一段末尾状态注入为本段起点
)
```

### 4.3 为什么需要 2D 相似度校正矩阵 (`visual_xy_correction`)？
相机的 Eye-in-Hand 标定存在少许装配角度微偏与机械臂微小下垂形变，在固定俯视高度下，视觉球心坐标与 TCP 真实下潜坐标存在毫米级系统偏差。  
**工程方案**：通过 8 组地面标定点拟合了基座标系下的 2D 相似变换：
$$x' = s \cdot (\cos\theta \cdot x - \sin\theta \cdot y) + t_x$$
$$y' = s \cdot (\sin\theta \cdot x + \cos\theta \cdot y) + t_y$$
将定位 RMSE 误差从 $15\text{ mm}$ 降低至 $6.4\text{ mm}$，确保每次夹爪两指均居中包络果实。

---

## 5. 源码目录与模块职责 (Codebase Structure)

```text
/home/li/hand_eye_calibration/apple_pick_v2/
├── single_apple_full_grasp.py    # 核心主控进程：有限状态机、MoveIt 规划与执行、夹爪驱动
├── single_apple_full_grasp.yaml  # 全局配置文件：位姿、速度、安全阈值、串口协议
├── grasp_cli.py                  # 面向操作人员的统一命令行接口 (CLI)
├── apple_center_localizer_v2.py  # 视觉感知节点：YOLOv11 实例分割 + 深度点云三维球心解算
├── apple_hand_eye_all.launch.py  # 系统启动 Launch：集成 RealSense、静态 TF、RViz 与驱动
├── safe_release_to_scan.py       # 安全救援脚本：无论夹爪处于何处，开爪并安全回退到 SCAN
└── results/                      # 历史抓取轨迹日志、预检记录与位姿点云回溯
```

### 状态机转移图 (FSM Lifecycle)
```text
               ┌───────────────────────┐
               │         SCAN          │ ◄──────────┐
               └──────────┬────────────┘            │
                          │ 视觉稳定检测             │
               ┌──────────▼────────────┐            │
               │   PREFLIGHT (预检)    │            │
               └──────────┬────────────┘            │
                          │ 100% 求解通过           │
               ┌──────────▼────────────┐            │
               │       PREGRASP        │            │
               └──────────┬────────────┘            │
                          │ 笛卡尔下潜              │
               ┌──────────▼────────────┐            │
               │     GRASP & 闭爪      │            │
               └──────────┬────────────┘            │
                          │ 笛卡尔起吊 7cm          │
               ┌──────────▼────────────┐            │
               │      LIFT (起吊)      │            │
               └──────────┬────────────┘            │
                          │                         │
            ┌─────────────┴─────────────┐           │
 [mode=basin]                           [mode=return]
            │                                       │
┌───────────▼──────────┐              ┌─────────────▼──────────┐
│   BASIN_APPROACH     │              │    原位 RELEASE 放料   │
└───────────┬──────────┘              └─────────────┬──────────┘
            │                                       │
┌───────────▼──────────┐              ┌─────────────▼──────────┐
│    BASIN_PLACE       │              │       开爪 & 回抽      │
└───────────┬──────────┘              └─────────────┬──────────┘
            │ 开爪落料                              │
┌───────────▼──────────┐                            │
│   BASIN_APPROACH     │                            │
└───────────┬──────────┘                            │
            └───────────────────────────────────────┘
```

---

## 6. 操作与测试指南 (Operator & Developer Guide)

### 6.1 本地 Mac 快捷控制 (推荐)
在你的 Mac 终端（`/Users/tanxuebin/Desktop/机械臂` 目录）直接使用封装好的 `arm_ctl`：

```bash
# 1. 检查远程主机与硬件健康状态
./arm_ctl doctor

# 2. 启动/停止远程后台完整栈 (ROS 2 + 相机 + 驱动 + 视觉定位)
./arm_ctl start
./arm_ctl status
./arm_ctl stop

# 3. 截取当前相机视野图像 (自动保存并显示)
./arm_ctl live

# 4. 执行 100% 轨迹预检 (不通电机，安全核验)
./arm_ctl preview

# 5. 执行真实入盆单次抓取
./arm_ctl single --place-target basin
```

### 6.2 远程 Ubuntu 原生命令行执行 (Native CLI)

登录远程主机：
```bash
ssh li@10.77.0.2
```

#### 步骤 1：后台启动感知底座（若未启动）
```bash
source /opt/ros/jazzy/setup.bash
source /home/li/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch /home/li/hand_eye_calibration/apple_pick_v2/apple_hand_eye_all.launch.py \
  allow_trajectory_execution:=true
```

#### 步骤 2：全链路仿真预检（零风险，不驱动电机）
```bash
cd /home/li/hand_eye_calibration/apple_pick_v2
/home/li/ros_yolo_env/bin/python grasp_cli.py \
  --preflight-only --place-target basin
```
*控制台输出全为绿色的 `PASS` 且规划完整度为 `1.000` 时，说明当前果实位姿与入盆路径具备 100% 可达性。*

#### 步骤 3：真机执行单次抓取入盆
```bash
/home/li/ros_yolo_env/bin/python grasp_cli.py \
  --execute --operator-confirmed --place-target basin --return-after-lift
```

#### 步骤 4：无人值守连续循环抓取入盆
```bash
/home/li/ros_yolo_env/bin/python grasp_cli.py \
  --continuous --execute --operator-confirmed --place-target basin
```
*说明：进入连续模式后，机械臂采摘入盆并回归 SCAN。当放入新果实且位置稳定 2 秒后，系统进入 3 秒安全倒计时并自动开启下一轮。*

---

## 7. 常见故障排查与应急救援 (Troubleshooting & Safety)

### 7.1 应急救援（机械臂异常悬停或夹爪未开）
若机械臂在中途因急停中断、误碰或异常退出，无需手动强力扳动机械臂！直接运行安全救援脚本：
```bash
/home/li/ros_yolo_env/bin/python /home/li/hand_eye_calibration/apple_pick_v2/safe_release_to_scan.py
```
该脚本会：
1. 发送串口命令将夹爪完全打开（80mm 开度）；
2. 规避碰撞体，平缓将机械臂带回固定俯视 SCAN 工位。

### 7.2 典型异常排查表
| 异常现象 | 可能原因 | 解决办法 |
| :--- | :--- | :--- |
| **`Waiting for valid apple center timed out`** | 1. 相机未开启或光照剧烈变化<br>2. 目标偏出视野<br>3. 置信度低于阈值 (0.15) | 运行 `./arm_ctl live` 查看相机图像；确保果实处于白纸中央工作台区域。 |
| **`Candidate search failed: No collision-free IK`** | 果实位置处于机械臂工作空间死区或奇异点边界 | 将果实适度向视场中心微调（推荐 $X \in [0.35, 0.45]\text{ m}, Y \in [-0.25, -0.05]\text{ m}$）。 |
| **`Modbus communication error / Gripper timeout`** | 末端工具接口 Tool IO 供电断开或波特率不匹配 | 检查机械臂末端指示灯，确保 Tool Voltage 设置为 3 (24V)；重新插拔夹爪通信接头。 |
| **`MoveIt trajectory execution failed`** | 规划时间太短或关节速度突变超过阈值 | 检查 `single_apple_full_grasp.yaml` 中的 `max_joint_delta_rad`（当前放宽至 5.0 rad 容许大幅姿态微调）。 |

---

## 8. 核心参数配置文件字典 (`single_apple_full_grasp.yaml`)

```yaml
place_target:
  mode: basin                           # 默认目标：落料入盆
  approach_xyz_m: [0.464, 0.006, 0.080] # 入盆前上方进刀高度 (Z=0.08m)
  place_xyz_m: [0.464, 0.006, -0.020]   # 低位落料高度 (Z=-0.02m)
  target_joints_rad: [0.0484, -1.6343, 0.7521, -0.0675, -2.0939, -2.8371] # 黄金前向关节配置
  yaw_rad: 2.847                        # 保持法兰水平朝向
  release_open_mm: 80.0                 # 放料时夹爪完全张开尺寸
  release_settle_s: 1.0                 # 放料静止等待时长

modbus_gripper:
  port: 1                               # 机械臂末端 Tool RS485 端口
  baudrate: 9600                        # 通信波特率
  open_mm: 80.0                         # 初始/完全张开开度
  close_mm: 2.0                         # 紧闭抓取行程
```
