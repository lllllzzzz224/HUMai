#!/usr/bin/env python3
"""Plan and execute one low-speed return to the configured fixed SCAN pose."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
import rclpy

from fixed_scan_real_verify import FixedScanVerify, ROOT, load_config


DEFAULT_CONFIG = ROOT / "single_apple_full_grasp.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(description="Low-speed return to fixed SCAN")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--execute", action="store_true")
    args, ros_args = parser.parse_known_args()
    cfg = load_config(args.config)

    rclpy.init(args=ros_args)
    node = FixedScanVerify(cfg)
    try:
        node.wait_interfaces()
        if float(cfg["moveit"]["velocity_scaling"]) > 0.05 or float(
            cfg["moveit"]["acceleration_scaling"]
        ) > 0.05:
            raise RuntimeError("拒绝执行：速度或加速度缩放超过 0.05")
        node.current_state_is_valid()
        current = node.current_joint_vector()
        target = node.scan_vector()
        print(
            f"当前关节={np.round(current, 5).tolist()}\n"
            f"SCAN关节={np.round(target, 5).tolist()}",
            flush=True,
        )
        trajectory = node.plan(node.make_joint_goal(target), "CURRENT -> SCAN")
        if not args.execute:
            print("PLAN-ONLY：缺少 --execute，真机未运动。", flush=True)
            return 0
        node.execute(trajectory, "CURRENT -> SCAN 低速回位")
        node.wait_scan()
        error = float(np.max(np.abs(node.current_joint_vector() - target)))
        print(f"已回到固定 SCAN；最大关节误差={error:.5f} rad。", flush=True)
        print("未发送任何夹爪命令。", flush=True)
        return 0
    except (KeyboardInterrupt, RuntimeError) as exc:
        node.get_logger().error(f"失败即停：{exc}")
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
