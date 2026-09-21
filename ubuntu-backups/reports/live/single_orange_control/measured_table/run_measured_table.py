#!/usr/bin/env python3
"""Temporary measured-table preflight; original production module is unchanged."""
import sys
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene

sys.path.insert(0, '/home/li/hand_eye_calibration/apple_pick_v2')
import single_apple_full_grasp as production

class MeasuredTableGrasp(production.SingleAppleFullGrasp):
    def apply_scene(self, apple, radius):
        cfg = self.cfg['scene']
        measured = float(cfg['measured_table_top_z_m'])
        inflation = float(cfg['table_safety_inflation_m'])
        old_top = float(apple[2] - radius) + inflation
        new_top = measured + inflation
        if not np.isfinite(new_top) or new_top < old_top:
            raise RuntimeError('Measured table must raise, never lower, existing protection')
        if abs(measured - (-0.072)) > 1e-9 or abs(inflation - 0.002) > 1e-9:
            raise RuntimeError('Unexpected measured table or inflation')
        # Parent preserves camera attached geometry and the original footprint.
        parent_top = super().apply_scene(apple, radius)
        if abs(parent_top - old_top) > 1e-9:
            raise RuntimeError('Unexpected parent guard height')
        size = list(map(float, cfg['table_size_xyz_m']))
        count = float(cfg.get('support_guard_footprint_diameters', 0.0))
        if count > 0.0:
            footprint = count * 2.0 * float(radius)
            size = [footprint, footprint, size[2]]
            xy = [float(apple[0]), float(apple[1])]
        else:
            xy = list(map(float, cfg['table_center_xy_m']))
        pose = Pose()
        pose.position.x, pose.position.y = xy
        pose.position.z = new_top - 0.5 * size[2]
        pose.orientation.w = 1.0
        table = CollisionObject()
        table.header.frame_id = self.base
        table.id = self.SUPPORT_GUARD_ID
        table.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)]
        table.primitive_poses = [pose]
        table.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [table]
        future = self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('Measured table protection was not accepted')
        self.event('Measured table guard raised', 'PASS', old_top_m=old_top,
                   measured_top_m=measured, safety_top_m=new_top,
                   dimensions_m=size, center_xy_m=xy,
                   scope='current single-fruit workstation only')
        return new_top

if __name__ == '__main__':
    args, extra = production.parse_cli(sys.argv[1:], production.DEFAULT_CONFIG)
    preflight = args.preflight_only and not args.execute
    execution = (args.execute and args.operator_confirmed and args.fully_autonomous
                 and args.place_target == 'basin' and not args.preflight_only
                 and not args.verify_only and not args.continuous
                 and not args.disable_visual_xy_correction and args.max_rounds is None)
    if extra or not (preflight or execution):
        raise SystemExit('Only preflight or explicit single full basin execution is permitted')
    production.SingleAppleFullGrasp = MeasuredTableGrasp
    raise SystemExit(production.main())
