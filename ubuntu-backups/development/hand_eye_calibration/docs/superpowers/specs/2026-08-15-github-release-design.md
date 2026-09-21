# Apple Catch GitHub 发布设计

日期：2026-08-15
目标仓库：`https://github.com/lllllzzzz224/Apple-catch.git`

## 目标

把已经在 RM65 真机验证成功的单苹果及连续苹果抓取实现发布到一个可克隆、可配置、可验证的 GitHub 仓库。仓库首页应能让新使用者理解硬件依赖、安全边界、安装步骤、配置位置和启动顺序，同时保留成功思路与真机验证记录。

## 发布范围

保留并发布：

- `apple_pick_v2/` 中当前连续抓取所需的 Python、YAML、launch、RViz 和测试文件；
- `hand_eye_static_tf.launch.py`、`hand_eye_result.yaml` 和 `modbus_gripper_ros.py`；
- `apple_pick_v2/连续苹果抓取README.md`；
- `apple_pick_v2/success apple catch.md`；
- `apple_pick_v2/success 8.10.md`；
- `apple_pick_v2/苹果抓取参数配置说明.md`；
- 连续抓取的设计和实现计划文档；
- Git 中已验证的提交历史及标签。

不发布：

- 当前工作区中所有未跟踪的旧抓取脚本、第三方仓库和临时实验目录；
- `__pycache__`、IDE 文件、`.orig`、模型权重和运行期调试图片；
- 普通失败日志和重复现场日志；
- 密钥、访问令牌、GitHub 凭据或本机私有配置。

旧文档若仍宣称“禁止执行”或指向已经不存在的入口，不作为首页入口；必要时从当前发布树移除，避免与成功流程冲突。

## 仓库首页

根目录 `README.md` 改写为 Apple Catch 首页，内容按以下顺序组织：

1. 项目状态：RM65 + RealSense Eye-in-Hand + ROS 2 Jazzy + MoveIt + RS485 夹爪已完成真机验证；
2. 安全警告：只允许一套 ROS 栈、首次使用必须检查 TCP/手眼/SCAN/急停；
3. 系统数据流和自动状态机；
4. 软件和硬件依赖；
5. 克隆、Python 环境、ROS 工作区和模型权重准备；
6. 现场专用参数清单；
7. 单轮与连续模式启动命令；
8. 测试命令、故障检查和日志位置；
9. 成功思路、参数说明和验证记录链接。

README 不声称任意机械臂可以下载后直接执行。它应明确：代码可以直接复用，但机械臂 IP、RM 驱动、MoveIt 模型、手眼外参、TCP、SCAN 关节角、视觉 XY 校正和夹爪开度必须按现场核验。

## 可移植性

当前成功代码中的 `/home/li/...` 绝对路径改为以下规则：

- 项目内部文件使用 `Path(__file__).resolve()` 推导仓库路径；
- 结果文件写到仓库内 `apple_pick_v2/results/`；
- YOLO Python 与模型路径通过 launch 参数或环境变量覆盖，并提供当前环境兼容默认值；
- README 使用 `APPLE_CATCH_ROOT` 和 `ROS_WS` 示例变量，不要求用户名必须为 `li`；
- ROS 包仍通过已 source 的 ROS/ament 环境查找，不复制本机 `ros2_ws/install`。

不改变已经真机验证的运动、TCP、视觉校正、候选规划、夹爪和连续状态机数值。便携性修改只改变资源定位方式，并用测试覆盖。

## 模型与依赖

模型权重不提交到 Git：`.pt` 保持忽略。README 说明默认模型名称和两种准备方式：Ultralytics 首次联网下载，或把本地权重路径通过参数传入。

新增精简 Python 依赖说明，区分：

- pip/conda 依赖：Ultralytics、NumPy、PyYAML 和 OpenCV；
- ROS 依赖：ROS 2 Jazzy、MoveIt 2、RealSense ROS、RM65 驱动与 MoveIt 配置；
- 项目不会把 ROS Python 包写进 pip requirements。

## 安全与配置

发布前扫描所有已跟踪文件中的 token、私钥、密码、硬编码 IP、`/home/li` 和超大文件。`hand_eye_result.yaml`、TCP、SCAN、视觉校正及夹爪参数作为“当前实机示例”保留，但 README 将其标为硬件专用，禁止未经验证直接用于另一台设备。

启动说明加入唯一实例检查：

```bash
ros2 action info /execute_trajectory
ros2 topic info /joint_states
ros2 node list | sort | uniq -d
```

执行前必须只有一个 `/execute_trajectory` action server 和一个 `/joint_states` 发布者。仓库不自动杀进程，也不绕过示教器急停。

## 验证与交付

推送前必须完成：

- Python 单元测试全部通过；
- 关键 Python 文件通过 `py_compile`；
- launch/YAML 可以解析；
- 已跟踪文件敏感信息与大文件扫描通过；
- README 中的相对链接有效；
- `git status` 中仅包含本次明确选择的发布修改；
- GitHub remote 指向目标仓库，先普通 push，不使用 force push；
- 推送 `master` 和两个已验证标签：`continuous-grasp-baseline-20260811`、`continuous-grasp-validated-20260811`。

成功标准：从目标 GitHub 仓库克隆后，使用者可以依据根 README 安装外部 ROS 依赖、指定 YOLO 环境/模型、替换现场参数、运行单元测试，并按明确的两终端顺序启动系统；仓库不包含本地未跟踪实验文件或凭据。
