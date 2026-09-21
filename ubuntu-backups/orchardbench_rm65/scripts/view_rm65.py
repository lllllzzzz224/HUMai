#!/usr/bin/env python3
"""Load the fixed-base RM65 Stage 3A model without simulation or control."""

from __future__ import annotations

import argparse
from pathlib import Path

import newton
import warp as wp


REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOT_URDF = REPO_ROOT / "assets" / "robots" / "rm65" / "rm65_with_gripper.urdf"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", choices=("gl", "null"), default="gl")
    parser.add_argument("--frames", type=int, default=0,
                        help="frame limit; 0 keeps the GL viewer open until closed")
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not ROBOT_URDF.is_file():
        raise FileNotFoundError(f"generated RM65 URDF not found: {ROBOT_URDF}")

    builder = newton.ModelBuilder()
    builder.add_urdf(str(ROBOT_URDF), floating=False, enable_self_collisions=False)
    builder.add_ground_plane()
    model = builder.finalize()
    state = model.state()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state)

    body_names = [str(name).rsplit("/", 1)[-1] for name in model.body_label]
    joint_names = [str(name).rsplit("/", 1)[-1] for name in model.joint_label]
    for required in ("Link6", "gripper_mount", "gripper_left_finger", "gripper_right_finger", "gripper_tcp"):
        if required not in body_names:
            raise RuntimeError(f"required body was not imported: {required}")
    print(f"[rm65-static] fixed base; bodies={model.body_count}, joints={model.joint_count}")
    print(f"[rm65-static] imported RM joints: {[name for name in joint_names if name.startswith('joint') or name.startswith('gripper_')]}")
    print("[rm65-static] TCP: Link6 -> gripper_tcp = 0.170 m")

    if args.viewer == "gl":
        viewer = newton.viewer.ViewerGL(headless=args.headless)
    else:
        viewer = newton.viewer.ViewerNull(num_frames=args.frames or 1)
    viewer.set_model(model)
    try:
        viewer.set_camera(pos=wp.vec3(1.2, 1.2, 0.9), pitch=-15.0, yaw=-135.0)
    except Exception:
        pass

    frame = 0
    while viewer.is_running() and not (args.frames and frame >= args.frames):
        viewer.begin_frame(0.0)
        viewer.log_state(state)
        viewer.end_frame()
        frame += 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
