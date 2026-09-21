# 历史连续抓取耗时分析

来源为原始连续会话及逐轮 JSON；仅读取，没有修改日志或运行机械臂。

**计时边界：**旧日志精度为整秒。准备段包含候选筛选、规划及相关调用，不能当作精确求解器时间；运动段包含局部重新规划、到位确认、夹爪等待与运动。轮间等待包含人工换位、目标稳定门禁和采样等待，无法仅凭日志分解。

## continuous_apple_grasp_20260811_181721.json

会话状态：COMPLETE；记录完成 1 轮；会话总长 41 秒；最后一轮结束至会话退出 0 秒。

| 轮次/尝试 | 轮前等待(s) | 准备至预检(s) | 预检后至运动(s) | 运动至回SCAN(s) | 轮总长(s) | 状态 |
|---|---:|---:|---:|---:|---:|---|
| 1/1 | 11 | 0 | 5 | 25 | 30 | PASS_PICK_RETURN_COMPLETE |

- 第 1 轮证据：[single_apple_grasp_20260811_181732_round001_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260811_181732_round001_attempt01.json)

## continuous_apple_grasp_20260811_181837.json

会话状态：FAULT_LATCHED；记录完成 1 轮；会话总长 155 秒；最后一轮结束至会话退出 0 秒。

| 轮次/尝试 | 轮前等待(s) | 准备至预检(s) | 预检后至运动(s) | 运动至回SCAN(s) | 轮总长(s) | 状态 |
|---|---:|---:|---:|---:|---:|---|
| 1/1 | 11 | 1 | 4 | 26 | 31 | PASS_PICK_RETURN_COMPLETE |
| 2/1 | 74 | 0 | 5 | 未知 | 39 | FAILED_STOPPED |

- 第 1 轮证据：[single_apple_grasp_20260811_181848_round001_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260811_181848_round001_attempt01.json)
- 第 2 轮证据：[single_apple_grasp_20260811_182033_round002_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260811_182033_round002_attempt01.json)
  错误：未到达固定鸟瞰位
  已观察到开始运动至故障记录结束 34 秒；没有回到 SCAN 的完成事件。

## continuous_apple_grasp_20260811_182627.json

会话状态：COMPLETE；记录完成 2 轮；会话总长 90 秒；最后一轮结束至会话退出 0 秒。

| 轮次/尝试 | 轮前等待(s) | 准备至预检(s) | 预检后至运动(s) | 运动至回SCAN(s) | 轮总长(s) | 状态 |
|---|---:|---:|---:|---:|---:|---|
| 1/1 | 11 | 0 | 5 | 26 | 31 | PASS_PICK_RETURN_COMPLETE |
| 2/1 | 18 | 0 | 5 | 25 | 30 | PASS_PICK_RETURN_COMPLETE |

- 第 1 轮证据：[single_apple_grasp_20260811_182638_round001_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260811_182638_round001_attempt01.json)
- 第 2 轮证据：[single_apple_grasp_20260811_182727_round002_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260811_182727_round002_attempt01.json)

## continuous_apple_grasp_20260811_183500.json

会话状态：FAULT_LATCHED；记录完成 0 轮；会话总长 52 秒；最后一轮结束至会话退出 0 秒。

| 轮次/尝试 | 轮前等待(s) | 准备至预检(s) | 预检后至运动(s) | 运动至回SCAN(s) | 轮总长(s) | 状态 |
|---|---:|---:|---:|---:|---:|---|
| 1/1 | 22 | 0 | 15 | 未知 | 30 | FAILED_STOPPED |

- 第 1 轮证据：[single_apple_grasp_20260811_183522_round001_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260811_183522_round001_attempt01.json)
  错误：PREGRASP 到位或稳定性检查失败：actual=[0.30663, -0.282984, 0.038566], target=[0.305319, -0.28171, 0.038605]
  已观察到开始运动至故障记录结束 15 秒；没有回到 SCAN 的完成事件。

## continuous_apple_grasp_20260811_202234.json

会话状态：STOPPED_BY_OPERATOR；记录完成 5 轮；会话总长 508 秒；最后一轮结束至会话退出 282 秒。

| 轮次/尝试 | 轮前等待(s) | 准备至预检(s) | 预检后至运动(s) | 运动至回SCAN(s) | 轮总长(s) | 状态 |
|---|---:|---:|---:|---:|---:|---|
| 1/1 | 10 | 0 | 6 | 24 | 30 | PASS_PICK_RETURN_COMPLETE |
| 2/1 | 11 | 1 | 5 | 25 | 31 | PASS_PICK_RETURN_COMPLETE |
| 3/1 | 30 | 0 | 6 | 24 | 30 | PASS_PICK_RETURN_COMPLETE |
| 4/1 | 12 | 0 | 5 | 25 | 30 | PASS_PICK_RETURN_COMPLETE |
| 5/1 | 10 | 0 | 5 | 27 | 32 | PASS_PICK_RETURN_COMPLETE |

- 第 1 轮证据：[single_apple_grasp_20260811_202244_round001_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260811_202244_round001_attempt01.json)
- 第 2 轮证据：[single_apple_grasp_20260811_202325_round002_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260811_202325_round002_attempt01.json)
- 第 3 轮证据：[single_apple_grasp_20260811_202426_round003_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260811_202426_round003_attempt01.json)
- 第 4 轮证据：[single_apple_grasp_20260811_202508_round004_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260811_202508_round004_attempt01.json)
- 第 5 轮证据：[single_apple_grasp_20260811_202548_round005_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260811_202548_round005_attempt01.json)

## continuous_apple_grasp_20260831_151930.json

会话状态：STOPPED_BY_OPERATOR；记录完成 1 轮；会话总长 83 秒；最后一轮结束至会话退出 34 秒。

| 轮次/尝试 | 轮前等待(s) | 准备至预检(s) | 预检后至运动(s) | 运动至回SCAN(s) | 轮总长(s) | 状态 |
|---|---:|---:|---:|---:|---:|---|
| 1/1 | 13 | 0 | 7 | 29 | 36 | PASS_PICK_RETURN_COMPLETE |

- 第 1 轮证据：[single_apple_grasp_20260831_151943_round001_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260831_151943_round001_attempt01.json)

## continuous_apple_grasp_20260831_152850.json

会话状态：STOPPED_BY_OPERATOR；记录完成 2 轮；会话总长 100 秒；最后一轮结束至会话退出 10 秒。

| 轮次/尝试 | 轮前等待(s) | 准备至预检(s) | 预检后至运动(s) | 运动至回SCAN(s) | 轮总长(s) | 状态 |
|---|---:|---:|---:|---:|---:|---|
| 1/1 | 10 | 0 | 5 | 26 | 31 | PASS_PICK_RETURN_COMPLETE |
| 2/1 | 19 | 0 | 5 | 25 | 30 | PASS_PICK_RETURN_COMPLETE |

- 第 1 轮证据：[single_apple_grasp_20260831_152900_round001_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260831_152900_round001_attempt01.json)
- 第 2 轮证据：[single_apple_grasp_20260831_152950_round002_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260831_152950_round002_attempt01.json)

## continuous_apple_grasp_20260831_153153.json

会话状态：STOPPED_BY_OPERATOR；记录完成 0 轮；会话总长 22 秒；无逐轮记录的会话等待 22 秒。

| 轮次/尝试 | 轮前等待(s) | 准备至预检(s) | 预检后至运动(s) | 运动至回SCAN(s) | 轮总长(s) | 状态 |
|---|---:|---:|---:|---:|---:|---|


## 未关联到完整会话的逐轮记录

下列文件独立存在，未重复计入上面的会话；没有会话关联时不推断轮间等待。

| 文件 | 准备至预检(s) | 预检后至运动(s) | 轮总长(s) | 状态 / 原因 |
|---|---:|---:|---:|---|
| [single_apple_grasp_20260831_153557_round001_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260831_153557_round001_attempt01.json) | 0 | 未知 | 11 | FAILED_STOPPED / 规划后苹果位置变化 127.8mm 超过 10.0mm |
| [single_apple_grasp_20260831_153610_round001_attempt02.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260831_153610_round001_attempt02.json) | 0 | 未知 | 5 | FAILED_STOPPED / 规划后苹果位置变化 127.7mm 超过 10.0mm |
| [single_apple_grasp_20260831_153617_round001_attempt03.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260831_153617_round001_attempt03.json) | 0 | 未知 | 5 | FAILED_STOPPED / 规划后苹果位置变化 127.7mm 超过 10.0mm |
| [single_apple_grasp_20260831_153634_round001_attempt01.json](/home/li/hand_eye_calibration/apple_pick_v2/results/single_apple_grasp_20260831_153634_round001_attempt01.json) | 1 | 6 | 32 | PASS_PICK_RETURN_COMPLETE / 无报错 |

## 已完成动作轮次统计

共 8 个会话，关联 12 份 PASS_PICK_RETURN_COMPLETE 逐轮记录；另有 1 份未关联完整会话的 PASS 逐轮记录。下表合计 13 份 PASS 记录，该状态不证明物理夹持成功。

| 项目 | 样本数 | 平均秒数 | 最小–最大秒数 |
|---|---:|---:|---:|
| 逐轮总长 | 13 | 31.08 | 30–36 |
| 准备至预检 | 13 | 0.23 | 0–1 |
| 预检后至运动 | 13 | 5.31 | 4–7 |
| 运动至回SCAN | 13 | 25.54 | 24–29 |

## 源码对应与优化优先级

1. 原 collect_apple() 每次清空队列并重新收集 10 个样本；视觉默认 2 Hz。连续流程 prepare_round() 后的 validate_frozen_target() 再调用 collect_target()，约 5 秒的复采等待与多轮日志一致。应以新鲜、稳定的持续缓存复核目标，不能直接删掉位置复核。
2. 原连续模式需要位置变化至少 30 mm、稳定 2 秒再倒计时 3 秒。轮前间隔并非纯规划开销，也不能全部认定为软件浪费。
3. 运动段占主要活动时间，但包含到位停留、局部规划、夹爪等待；先加高分辨率分段计时，再评估轨迹复用。不可从整秒事件推导毫秒级规划速度。
4. 优先消除重复采样、重复规划和无意义重试；历史会话还包含故障锁停与结束后的空闲，应单独记录。保持现有速度、抬升高度和夹爪等待，直到新的真机测量支持调整。

