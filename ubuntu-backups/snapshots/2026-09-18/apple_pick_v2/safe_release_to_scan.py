#!/usr/bin/env python3
"""Place the held apple back at its logged GRASP point, retreat, then return to SCAN."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy

from fixed_scan_real_verify import ROOT, load_config, vertical_pose
from single_apple_full_grasp import SingleAppleFullGrasp


DEFAULT_CONFIG = ROOT / "single_apple_full_grasp.yaml"
DEFAULT_LOG = ROOT / "results" / "single_apple_grasp_20260808_153839.json"


def load_success_log(path: Path) -> dict:
    data = json.loads(path.expanduser().read_text(encoding="utf-8"))
    if data.get("status") != "PASS_SINGLE_ROUND_COMPLETE":
        raise RuntimeError(f"不是成功抓取日志：status={data.get('status')}")
    required = (
        "apple_center_base_m",
        "apple_radius_m",
        "pregrasp_base_m",
        "grasp_base_m",
        "lift_base_m",
    )
    missing = [key for key in required if key not in data]
    if missing:
        raise RuntimeError(f"抓取日志缺少字段：{missing}")
    return data


def main() -> int:
    parser = argparse.ArgumentParser(description="受保护的 LIFT → RELEASE → SCAN 复位")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run-log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--operator-confirmed", action="store_true")
    args, ros_args = parser.parse_known_args()

    cfg = load_config(args.config)
    result_dir = ROOT / "results"
    result_dir.mkdir(exist_ok=True)
    output = result_dir / f"safe_release_to_scan_{time.strftime('%Y%m%d_%H%M%S')}.json"
    record = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source_grasp_log": str(args.run_log.expanduser()),
        "status": "STARTED",
        "gripper_open_commanded": False,
        "returned_to_scan": False,
    }

    rclpy.init(args=ros_args)
    node = SingleAppleFullGrasp(cfg)
    try:
        if not args.execute or not args.operator_confirmed:
            raise RuntimeError("真实复位必须同时提供 --execute --operator-confirmed")
        source = load_success_log(args.run_log)
        apple = np.asarray(source["apple_center_base_m"], dtype=np.float64)
        radius = float(source["apple_radius_m"])
        yaw = float(cfg["targets"]["vertical_yaw_rad"])
        pre_pose = vertical_pose(source["pregrasp_base_m"], yaw)
        release_pose = vertical_pose(source["grasp_base_m"], yaw)
        logged_lift = np.asarray(source["lift_base_m"], dtype=np.float64)

        record.update({
            "apple_center_base_m": apple.tolist(),
            "release_base_m": list(map(float, source["grasp_base_m"])),
            "pregrasp_base_m": list(map(float, source["pregrasp_base_m"])),
            "logged_lift_base_m": logged_lift.tolist(),
            "release_opening_mm": float(cfg["modbus_gripper"]["open_mm"]),
        })
        node.wait_interfaces()

        actual_start = node.tcp_position()
        start_error = float(np.linalg.norm(actual_start - logged_lift))
        record["actual_start_tcp_base_m"] = np.round(actual_start, 7).tolist()
        record["start_error_from_logged_lift_m"] = start_error
        if start_error > 0.010:
            raise RuntimeError(
                f"当前 TCP 不在成功日志的 LIFT 附近：误差 {start_error * 1000.0:.1f} mm；"
                "拒绝使用旧抓取位置下降"
            )
        node.event("当前 LIFT 对应检查", "PASS", error_mm=round(start_error * 1000.0, 3))

        # Communication setup only. Never execute mechanical zeroing while holding fruit.
        node.gripper.configure(cfg["modbus_gripper"])
        node.event("夹爪通信配置", "PASS", mechanical_zeroing=False)

        table_top = node.apply_scene(apple, radius)
        record["table_guard_top_z_m"] = table_top
        node.current_state_is_valid()
        release_state = node.solve_ik(release_pose, "RELEASE")
        node.solve_ik(pre_pose, "PREGRASP")

        down_preflight = node.cartesian_to(
            release_pose,
            "LIFT -> RELEASE 预检",
            velocity_scaling=0.03,
            acceleration_scaling=0.03,
            max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
        )
        node.cartesian_to(
            pre_pose,
            "RELEASE -> PREGRASP 预检",
            start_state=release_state,
            velocity_scaling=0.03,
            acceleration_scaling=0.03,
            max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
        )
        node.event("放回与撤离路径预检", "PASS")

        phrase = "EXECUTE SAFE RELEASE"
        typed = input(
            "\n确认苹果仍被夹住、原抓取位置没有障碍、急停可用。"
            f"输入 {phrase} 将苹果放回原位："
        ).strip()
        if typed != phrase:
            raise RuntimeError("复位口令不匹配；机械臂和夹爪保持当前状态")

        node.execute(down_preflight, "LIFT -> RELEASE")
        actual_release = node.wait_tcp_arrival(release_pose, "RELEASE")
        record["actual_release_tcp_base_m"] = np.round(actual_release, 7).tolist()
        node.event("到达 RELEASE", "PASS")

        node.gripper.set_opening_mm(
            cfg["modbus_gripper"], cfg["modbus_gripper"]["open_mm"]
        )
        record["gripper_open_commanded"] = True
        node.event("释放苹果", "PASS", opening_mm=cfg["modbus_gripper"]["open_mm"])
        node.settle(1.0)

        retreat = node.cartesian_to(
            pre_pose,
            "RELEASE -> PREGRASP",
            velocity_scaling=0.03,
            acceleration_scaling=0.03,
            max_tcp_speed_m_s=float(cfg["lift_cartesian"]["max_tcp_speed_m_s"]),
        )
        node.execute(retreat, "RELEASE -> PREGRASP")
        node.wait_tcp_arrival(pre_pose, "PREGRASP")
        node.event("释放后直线撤离", "PASS")

        scan_trajectory = node.plan(
            node.make_joint_goal(node.scan_vector()), "PREGRASP -> SCAN"
        )
        node.execute(scan_trajectory, "PREGRASP -> SCAN")
        node.wait_scan()
        node.event("返回固定鸟瞰位", "PASS")
        record["returned_to_scan"] = True
        record["status"] = "PASS_RELEASED_AND_RETURNED_TO_SCAN"
        print(
            "\nPASS：苹果已放回原抓取位置，夹爪已打开到 70 mm，"
            "机械臂已返回固定 SCAN。现在可以移动苹果准备下一轮。",
            flush=True,
        )
        return 0
    except KeyboardInterrupt:
        record["status"] = "STOPPED_BY_OPERATOR"
        node.event("复位流程", "STOPPED_BY_OPERATOR")
        return 130
    except RuntimeError as error:
        record["status"] = "FAILED_STOPPED"
        record["error"] = str(error)
        node.get_logger().error(f"失败即停：{error}")
        return 2
    finally:
        record["events"] = node.events
        record["finished_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        output.write_text(json.dumps(record, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"复位执行日志：{output}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
