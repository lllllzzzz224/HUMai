# Static Obstacle Preview Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将当前静止 RGB-D 观察转换为可检查的三维障碍模型，并在不控制机械臂的条件下完成 MoveIt 场景写入、读回及仅规划验证。

**Architecture:** 纯 Python 几何模块负责输入校验和三态空间（占用、自由、未知），ROS 适配器只负责场景和规划请求。采集、几何、场景状态和规划结果分别保存证据；所有结果固定标记为不可直接执行。第一批验收到预抓取规划，闭爪、持果、释放和入盆的阶段模型不在本批接入实际执行。

**Tech Stack:** Python 3.12、NumPy、现有 RGB-D/YOLO 数据、ROS 2 Jazzy、MoveIt 2、RViz、pytest。

**Spec:** `../specs/2026-09-20-automatic-static-obstacle-scene-design.md`。用户 2026-09-21 的“ok继续”承接当前无动作建模及预检范围。

## Global Constraints

- “建模节点不创建运动动作客户端。”
- “新模块仅拥有 `auto_obstacle_*` 命名空间。”
- “桌面可从聚类输入中分离，但仍以支撑面碰撞模型保留。”
- “深度为零、视野外或物体背后的区域不自动标为自由。”
- “第一版不自动移动机械臂寻找新视角。”
- “不得删除现有支撑面、相机附着体、邻果或其他模块维护的对象。”
- 所有新增功能默认 `executable=false`；不创建 `/execute_trajectory` 客户端，不调用夹爪，不修改实际目录中的抓取脚本、标定、速度或 TCP。
- 开发在工作区 `development/hand_eye_calibration/apple_pick_v2` 新增独立文件；实际目录单果脚本已有更新，禁止整文件覆盖。
- 当前相机到基座链必须含拍摄时姿态及相机内部光学变换，历史 RGB-D 无同步姿态时只允许相机坐标系预览。
- 已安装 URDF 只有 80×80×170 mm 的夹爪整体盒，而现有配置开爪 85 mm；在测量确认实际张开包络前，仅允许模型/路径预览，不允许宣称全夹爪通道已验收。相机碰撞体原由抓取脚本动态加入，独立场景必须显式核对其存在。

## Review Focus

1. 毫米深度、米坐标和角度单位混用：几何模块拒绝无单位记录；用已知点测试转换。
2. 失去视野或无效深度使旧障碍消失：删除旧体素必须具有同一空间的有效自由观测。
3. 目标掩膜误删邻近枝条：按同帧掩膜和深度一致性区分，不能删除整个检测框；预抓取预览保留目标作为障碍。
4. 静止快照被误认为实时状态：输出记录真实采集时间、模型版本和状态来源；不发布伪装成实时的关节状态。
5. MoveIt 显示成功但实际使用旧几何：写入后读回核对；规划绑定相同场景摘要，任意不同均使结果无效。

## 文件与接口

所有路径相对 `apple_pick_v2/`。新增：

- `obstacle_geometry.py`：纯几何、数据校验和自由/占用/未知查询，无 ROS 或硬件导入。
- `obstacle_preview.py`：离线命令入口，输出点云、场景 JSON、质量报告；不能向机械臂发命令。
- `obstacle_scene_ros.py`：MoveIt 场景增量写入、读回、状态检查及 `plan_only=True` 规划。
- `obstacle_preview.launch.py`：仅启动模型、MoveIt、可选 RViz；轨迹执行固定关闭，不包含机器人驱动或控制器。
- `obstacle_preview.yaml`：显式空间范围、采集有效期、分辨率、误差包络及模型来源。没有实测误差依据时只输出观察预览，不能宣称几何验收通过。
- `tests/test_obstacle_geometry.py`、`tests/test_obstacle_preview.py`、`tests/test_obstacle_scene_ros.py`：纯离线和伪造接口测试。
- `README_自动障碍预览.md`：启动、限制、结果解释及恢复方式。

核心接口：

```python
def validate_observation(record: dict, now_s: float, max_age_s: float) -> None: ...
def depth_points(depth, intrinsics: dict, scale_m: float, transform=None): ...
def build_snapshot(depth, record: dict, config: dict, target_mask=None) -> dict: ...
def classify_points(points, depth, record: dict, config: dict): ...
def scene_delta(previous: dict, current: dict, cleared_ids: set[str]) -> dict: ...
def canonical_scene(objects: list[dict]) -> str: ...
```

快照包含 `schema_version`、`frame_id`、`capture_stamp_s`、`transform_provenance`、`robot_model_sha256`、`occupied_cells`、`unknown_policy`、`target_geometry`、`quality`、`executable=false`。自由空间通过与原始深度的投影比较查询，遮挡后方和视野外返回 `unknown`，不需要把整个环境枚举为自由体素。

## Task 1：纯几何与有效性

**Files:** Create `obstacle_geometry.py`; Test `tests/test_obstacle_geometry.py`。

**Interfaces:** consumes RGB-D metadata/NumPy arrays; produces the six pure functions above.

- [x] 先编写以下最小失败用例，并补齐零深度、无 TF、非正交旋转、过期/未来时间、机器人移动及单位异常的拒绝测试：

```python
def test_depth_units_and_translation():
    depth = np.array([[1000]], dtype=np.uint16)
    intr = {"fx": 100.0, "fy": 100.0, "ppx": 0.0, "ppy": 0.0}
    transform = np.eye(4)
    transform[0, 3] = 0.2
    np.testing.assert_allclose(depth_points(depth, intr, .001, transform), [[.2, 0., 1.]])

def test_old_occupied_cell_needs_positive_clear_evidence():
    old = {"occupied_cells": {"auto_obstacle_0_0_1": [0., 0., 1.]}}
    new = {"occupied_cells": {}}
    assert scene_delta(old, new, set())["remove"] == []
```

- [x] 运行 `python -m pytest tests/test_obstacle_geometry.py -q`，确认失败源于尚未实现功能。
- [x] 实现投影 `x=(u-cx)*z/fx, y=(v-cy)*z/fy`、齐次坐标变换、网格索引、确定性 ID、空间分类及验证。目标和桌面仍作为独立几何保留；任意整体框删除行为由测试禁止。
- [x] 复跑测试，增加“点在表面前/表面上/表面后”和“点在视野外”的分类用例，确保为自由/占用/未知/未知。

## Task 2：当前数据离线重放与模型导出

**Files:** Create `obstacle_preview.py`, `obstacle_preview.yaml`; Test `tests/test_obstacle_preview.py`。

**Interfaces:** consumes Task 1; produces `observation.ply`、`scene.json`、`quality.json` and CLI return code.

- [x] 编写输入缺同步姿态只能生成相机系预览的失败测试：

```python
def test_camera_only_capture_is_not_a_base_scene(tmp_path):
    result = run_preview_fixture(tmp_path, has_transform=False)
    assert result["frame_id"] == "camera_color_optical_frame"
    assert result["executable"] is False
    assert "missing_camera_to_base" in result["quality"]["limitations"]
```

`run_preview_fixture` 在该测试文件中创建 3×3 深度矩阵、合法内参和快照 JSON，调用 CLI 的 `run_preview(input_path, output_path, config)`，返回导出的 `scene.json`。

- [x] 运行 `python -m pytest tests/test_obstacle_preview.py -q` 验证失败。
- [x] 实现上述 `run_preview` 及 `--capture`、`--output`、`--config` 参数；文件路径必须显式给定，不覆盖原采集证据。
- [x] 重放 9/21 枝叶样本和 9/20 瓶盒样本，保存质量报告；仅对具有完整同步变换的新采集生成基座系模型。记录参数与处理耗时，不把成功输出 PLY 当成通过抓取验收。
- [x] 测试完整/缺失/损坏文件，校验输出文件中的单位、坐标、时间和 `executable=false`。

## Task 3：MoveIt 场景写入与读回

**Files:** Create `obstacle_scene_ros.py`; Test `tests/test_obstacle_scene_ros.py`。

**Interfaces:** consumes `scene.json`; uses `/apply_planning_scene` and `/get_planning_scene`; returns canonical scene digest and readback result.

- [x] 用伪造服务编写“其他模块对象不变”“读回不一致拒绝”“旧障碍不能因未观察到而消失”的测试：

```python
def test_foreign_objects_are_never_removed():
    delta = scene_delta({"occupied_cells": {}}, {"occupied_cells": {}}, {"apple_test_table_guard"})
    assert "apple_test_table_guard" not in delta["remove"]
```

- [x] 运行三个障碍测试文件，确认新接口测试先失败。
- [x] 为每个自有体素生成 `CollisionObject`，所有消息使用 `is_diff=True`；仅受验证自由观测支持的自有 ID 可删除。提交后读回几何与摘要，拒绝缺项或不一致。
- [x] 场景适配器在非 `base_link`、变换证据缺失、过期或质量不足时只输出诊断，不写入在线规划场景。目标模型保持存在，第一批不修改 Allowed Collision Matrix。
- [x] 独立预览场景启动后，核查支撑面、相机/支架包络和夹爪几何；仅在隔离预览域中可按有来源的配置新增 `auto_obstacle_support`、`auto_obstacle_camera_guard` 等自有模型。缺少经过核对的外形时标记 `robot_envelope_unverified`，不得输出完整碰撞验收通过。
- [x] 用伪造服务测试失败及超时；验证没有运动客户端、夹爪发布者或清空全场操作。

## Task 4：仅规划进程与可视化

**Files:** Create `obstacle_preview.launch.py`; update new ROS adapter and config; add launch/config inspection tests.

**Interfaces:** consumes installed `rm_65_config` and verified observation; displays actual monitored scene; accepts explicit current and candidate goal robot states for planning.

- [x] 用 AST/配置测试固定禁止轨迹执行，测试 launch 不包含 `rm_driver`、`rm_control`、`ros2_control_node` 或关节状态 GUI。
- [x] 仅启动 `move_group`、机器人模型发布和可选 RViz，`allow_trajectory_execution=false`。与现有 ROS 节点共存前检查冲突；优先使用独立预览域并记录域号。
- [x] 优先向场景显式提交采集时真实 `RobotState`，规划请求设置 `start_state`。若为展示发布快照关节，话题和界面必须注明“录制快照”，禁止伪装成刚采集的实时状态。
- [x] 所有 `/move_action` 请求强制 `planning_options.plan_only=True`，无执行后备路径。目标来自经验证的预抓取候选；候选不足时展示模型并报告缺口，不编造抓取姿态。
- [x] 规划前后检查场景摘要和真实状态有效性。以 `/check_state_validity` 核对轨迹中的机器人状态，并另行检查相关空间的未知覆盖；输出 `PLAN_PREVIEW` 或具体失败原因，始终不可直接执行。
- [x] 在图中显示点云、实际碰撞体、目标和可用轨迹；没有有效轨迹就显示相应状态。检查保存的 RViz 配置引用当前场景和坐标系。

## Task 5：现场证据、回归与交付

**Files:** Create `README_自动障碍预览.md`; reports under workspace `reports/`; update daily record.

- [x] 运行 `python -m pytest tests/test_obstacle_geometry.py tests/test_obstacle_preview.py tests/test_obstacle_scene_ros.py -q`，再运行现有纯离线测试套件；已有失败单独记录，不修改无关代码。
- [x] 以保存数据验证可见障碍、遮挡后未知空间、物体移走后有效清空证据；使用人工构造但标明来源的场景验证可绕行与完全阻塞两类规划。
- [ ] 对当前真场景记录同帧 RGB-D、真实姿态、变换来源、模型及配置哈希、场景读回、RViz 画面和规划结果。坐标或轮廓误差未经独立测量时明确保留未验收标志。
- [x] 独立审查新增模块及禁止动作边界，修复确认的问题，复跑受影响检查。仅提交本功能文件，保留既有未跟踪改动。
- [x] 交付时分别说明“代码可用”“模型显示/场景读回”“路径预览”“实物抓取”的状态，列出本次启动的进程和停止方法。

## 自检与执行方式

本计划对应设计允许的首批“无动作建模及预抓取规划”交付。实际闭爪、携果及入盆动作继续使用已验证基线，在完整阶段模型和场景刷新机制验收前不接入执行。树叶受控接触、移枝与动态重规划不在本批。

建议由主代理在本会话顺序实现，几何与 ROS 接口保持单一实现者，完成后安排独立审查。用户后续“继续”已确认本会话在当前开发副本实现，保持无真机动作范围。

## 2026-09-21 执行结果与差异

- 以原仓库 unittest 替代未安装的 pytest：新增23项+原72项，共95项通过。独立审查2项P2已补回归并修复。
- 当前one-shot ROS writer只新增/保持自有几何，不接自动删除；同ID几何变化拒绝，避免改变分辨率/膨胀时丢失旧占据。整格自由证据与scene_delta已实现纯函数，连续场景更新不在本轮部署。
- 历史基座快照3071体素和相机盒已写入/读回一致；RGB-D单独的最新15:10橘子帧仅做相机系模型，不混用更早姿态。
- 仅规划服务已实际验证合成绕行与碰撞输入拒绝，结果不代表橘子预抓取通过。覆盖检查只抽查连杆/TCP原点；完整机器人扫掠体和实际张开夹爪仍保持未验收。
- 当前相机枚举返回NO_CAMERA_DEVICE，无法补采同帧橘子+机器人姿态。因此现场当前真场景验收项保留未完成；已有历史RViz截图和完整读回报告。
- 证据目录：工作区reports/obstacle_preview_validation_20260921_214559。无运动/夹爪命令，未部署至实际运行目录。
