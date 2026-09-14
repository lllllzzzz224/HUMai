# Ubuntu 机械臂代码快照

## 来源

- 主机：`li@10.77.0.2`
- 获取日期：2026-09-05
- 连接方式：Mac 与 Ubuntu 千兆网线直连

## 保存范围

- `hand_eye_calibration/` ← `/home/li/hand_eye_calibration/`
- `rm_rl_grasp/` ← `/home/li/ros2_ws/src/rm_rl_grasp/`
- `home-probes/` ← `/home/li/gate3_*.py`

## 排除项

- Git 元数据：`.git/`、`.worktrees/`
- 构建产物：`build/`、`install/`、`log/`
- 缓存：`__pycache__/`、`.pytest_cache/`、`*.pyc`

## 验收边界

该目录是 Ubuntu 源代码与相关资产的本地快照。复制成功不代表这些代码可在 macOS 上直接运行；真实机械臂控制仍依赖 Ubuntu、ROS 2、硬件驱动、相机和机械臂网络。

## 复制结果

- `hand_eye_calibration/`：507 个文件，约 283 MB。
- `rm_rl_grasp/`：2405 个文件，约 170 MB。
- `home-probes/`：20 个文件，约 92 KB。
- 未发现被排除的 `.git/`、`build/`、`install/`、`log/` 或 `__pycache__/` 目录。

## 抽样哈希校验

以下 SHA-256 已与 Ubuntu 原文件逐项比对一致：

```text
298ebc749d1b464991b1d23640dae07749a2974be7ab8986013dadbcfa5e8751  hand_eye_calibration/apple_pick_v2/single_apple_full_grasp.py
55ed65c56c91713d23e8402371c6c49a6fd84f257f7dce452e8d70e41dcbe152  hand_eye_calibration/models/yolo11n-seg.pt
b24fb0cd58f33c8e59966a1500f786ddc2ba98cdc41c4aa72f6802b2d081ea17  rm_rl_grasp/launch/apple_grasp_demo.launch.py
50642cd8b151016bb50e28d37a4b3434427d4e659e139626dd2c0cde8cbc468f  home-probes/gate3_ik_probe.py
```
