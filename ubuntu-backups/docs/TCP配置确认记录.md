# TCP 配置确认记录

记录日期：2026-08-06

## 已确认的机械结构参数

- 机械臂当前 MoveIt 末端模型 link：`Link6`。
- 夹爪经 RM 控制器末端 RS485 / Modbus RTU 通信；它不是 ROS 关节夹爪。
- `Link6` 的正 Z 轴为夹爪抓取方向。
- 夹爪相对桌面垂直，无侧向或倾斜安装。
- 苹果被两指夹持时的中心位于 `Link6` 正 Z 轴上，距离为 **0.138 m**。
- `0.170 m` 是夹爪总长度/结构尺寸，不作为当前抓取 TCP。

## 唯一 TCP 初值

```xml
<link name="tcp_link"/>

<joint name="link6_to_tcp" type="fixed">
  <parent link="Link6"/>
  <child link="tcp_link"/>
  <origin xyz="0 0 0.138" rpy="0 0 0"/>
</joint>
```

等价关系：

```text
T_Link6_tcp = xyz(0, 0, 0.138), rpy(0, 0, 0)
```

`tcp_link` 表示两指夹持中心，而不是夹爪最远端或苹果顶部。

## 当前软件状态

- 当前运行的 MoveIt 组：`rm_group`。
- 当前 SRDF tip：`Link6`，尚无 `tcp_link`。
- 当前真机 URDF、MoveIt URDF 与 TF 树均未定义 `tcp_link`。
- 控制器当前工具坐标系名称为 `Arm_Tip`；其数值外参尚未通过 ROS 驱动读取。
- 现有手眼外参以末端/相机相对关系表示；新增 `tcp_link` 不改变相机相对 `Link6` 的外参。
- 抓取代码当前将 ArmTip 目标换算为 Link6 目标；引入 `tcp_link` 后必须统一目标语义，避免重复补偿。

## 后续实施范围（尚未执行）

1. 在真机实际加载的 RM65 URDF 中添加 `Link6 -> tcp_link` 固定 joint。
2. 在 RM65 MoveIt SRDF 中将 `rm_group` 的 tip 从 `Link6` 改为 `tcp_link`。
3. 同步维护 Xacro 源文件，避免后续重新生成 URDF 时丢失 TCP。
4. 重构抓取目标换算：规划约束和 IK 均以 `tcp_link` 为末端。
5. 在 RViz 显示 `tcp_link` 坐标轴，确认其落在实际两指夹持中心。
6. 先做无执行规划预览，再做低速 `SCAN -> PREGRASP` 验证。

本记录只归档已确认参数；未修改任何 URDF、SRDF、MoveIt 或抓取代码。
