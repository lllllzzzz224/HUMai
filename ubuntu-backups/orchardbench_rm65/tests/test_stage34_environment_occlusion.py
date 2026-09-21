"""Test-only non-RM occlusion characterization; no production scene changes.

Two minimal models contain the same apple-sized sphere and environment box.
Only the box's test placement differs. Sensor, intrinsics, depth semantics,
shape/body self-filter method and FruitPerception are the production ones.
"""
import json
import math
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
import newton
import warp as wp
from newton.sensors import SensorTiledCamera
from treesim.config import RobotParams
from treesim.perception import FruitPerception
from treesim.rm65_camera import CameraBinding


class NonRMEnvironmentOcclusion(unittest.TestCase):
    def observe(self, blocked):
        cfg=RobotParams()
        center=np.array([0.,0.,-.6]);radius=.032
        box_position=(0.,0.,-.3) if blocked else (.5,0.,-.3)
        with wp.ScopedDevice('cpu'):
            builder=newton.ModelBuilder()
            apple=builder.add_body(label='test_target_sphere',xform=wp.transform(wp.vec3(*center),wp.quat_identity()))
            apple_shape=builder.add_shape_sphere(apple,radius=radius,label='test_target_sphere_shape')
            obstacle=builder.add_body(label='test_non_rm_environment_box',xform=wp.transform(wp.vec3(*box_position),wp.quat_identity()))
            obstacle_shape=builder.add_shape_box(obstacle,hx=.15,hy=.15,hz=.02,label='test_environment_box_shape')
            model=builder.finalize(device='cpu');state=model.state()
            sensor=SensorTiledCamera(model)
            W,H=cfg.camera_width,cfg.camera_height
            rays=sensor.utils.compute_pinhole_camera_rays(W,H,math.radians(cfg.camera_fov))
            depth=sensor.utils.create_depth_image_output(W,H,1)
            shapes=sensor.utils.create_shape_index_image_output(W,H,1)
            cameras=wp.array([[wp.transform_identity()]],dtype=wp.transform,device=model.device)
            model.bvh_refit_shapes(state)
            sensor.update(state,cameras,rays,depth_image=depth,shape_index_image=shapes,
                clear_data=SensorTiledCamera.ClearData(clear_depth=cfg.camera_range))
            raw=depth.numpy()[0,0].copy();hits=shapes.numpy()[0,0].copy()
            sb=model.shape_body.numpy()
            # This test model has NO RM bodies. Invoke the unmodified production
            # exact shape_index -> shape_body membership filter with that set.
            membership=SimpleNamespace(self_bodies=frozenset())
            detector_depth,own=CameraBinding.filter_depth(membership,raw,hits,sb)
            detector=FruitPerception(W,H,cfg.camera_fov)
            detections=detector.detect(detector_depth,np.zeros(3),np.eye(3))
            # Truth is accessed for scoring only AFTER the formal detect call.
            dot=detector.dirs@center;disc=dot**2-(center@center-radius**2)
            target_rays=(disc>0)&(dot>0)
            entry=dot-np.sqrt(np.maximum(disc,0.))
            target_hits=target_rays&(hits==apple_shape)
            blocker_hits=target_rays&(hits==obstacle_shape)
            matches=[dict(center_world=d.center_world.tolist(),radius=d.radius,npix=d.npix,rms=d.rms,
                center_error_m=float(np.linalg.norm(d.center_world-center)),radius_error_m=abs(d.radius-radius))
                for d in detections if np.linalg.norm(d.center_world-center)<.05]
            row=dict(case='blocked' if blocked else 'clear',physics_steps=0,device='cpu',
                intrinsics=dict(width=W,height=H,fov_deg=cfg.camera_fov,range_m=cfg.camera_range,camera_every=cfg.camera_every),
                T_world_camera=np.eye(4).tolist(),sphere=dict(body=apple,shape=apple_shape,body_name=model.body_label[apple],shape_name=model.shape_label[apple_shape],type='SPHERE',center=center.tolist(),radius_m=radius),
                obstacle=dict(body=obstacle,shape=obstacle_shape,body_name=model.body_label[obstacle],shape_name=model.shape_label[obstacle_shape],type='BOX',position=list(box_position),quaternion_xyzw=[0,0,0,1],half_extents_m=[.15,.15,.02],category='non-RM non-apple environment'),
                target_ray_count=int(target_rays.sum()),target_hit_count=int(target_hits.sum()),
                blocker_first_hit_count=int(np.count_nonzero(blocker_hits&(raw<entry))),
                target_visible_pixels=int(np.count_nonzero(hits==apple_shape)),
                obstacle_pixel_count=int(np.count_nonzero(hits==obstacle_shape)),
                obstacle_self_mask_count=int(np.count_nonzero(own&(hits==obstacle_shape))),
                obstacle_detector_finite_count=int(np.count_nonzero(np.isfinite(detector_depth)&(hits==obstacle_shape))),
                self_mask_total=int(own.sum()),detector_depth_equals_raw=bool(np.array_equal(detector_depth,raw.astype(float))),
                detection_count=len(detections),target_matches=matches)
            output=os.environ.get('STAGE34_OCCLUSION_LOG_DIR')
            if output:
                p=Path(output);p.mkdir(parents=True,exist_ok=True)
                with (p/(row['case']+'.json')).open('x') as f:json.dump(row,f,indent=2)
                np.savez_compressed(p/(row['case']+'.npz'),raw_depth=raw,detector_depth=detector_depth,
                    shape_index=hits,self_mask=own,T_world_camera=np.eye(4),target_rays_for_scoring_only=target_rays)
            print('NON_RM_OCCLUSION '+json.dumps(row),flush=True)
            return row

    def test_01_unoccluded_target_naturally_detected(self):
        r=self.observe(False)
        self.assertGreaterEqual(r['target_ray_count'],50)
        self.assertEqual(r['target_hit_count'],r['target_ray_count'])
        self.assertEqual(r['target_visible_pixels'],r['target_ray_count'])
        self.assertEqual(r['self_mask_total'],0)
        self.assertTrue(r['detector_depth_equals_raw'])
        self.assertEqual(len(r['target_matches']),1)
        self.assertLessEqual(r['target_matches'][0]['center_error_m'],.005)
        self.assertLessEqual(r['target_matches'][0]['radius_error_m'],.005)

    def test_02_environment_fully_occludes_without_self_mask_removal(self):
        r=self.observe(True)
        self.assertGreaterEqual(r['target_ray_count'],50)
        self.assertEqual(r['blocker_first_hit_count'],r['target_ray_count'])
        self.assertEqual(r['target_visible_pixels'],0)
        self.assertEqual(r['target_hit_count'],0)
        self.assertGreater(r['obstacle_pixel_count'],0)
        self.assertEqual(r['obstacle_self_mask_count'],0)
        self.assertEqual(r['obstacle_detector_finite_count'],r['obstacle_pixel_count'])
        self.assertTrue(r['detector_depth_equals_raw'])
        self.assertEqual(r['target_matches'],[])

if __name__=='__main__':unittest.main()
