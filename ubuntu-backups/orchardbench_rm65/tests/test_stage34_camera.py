"""Stage 3.4 contracts: actual transforms, sampling and ray-hit self filtering."""
import importlib
import importlib.util
import unittest
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from scripts import grow_tree


def camera_module(test):
    test.assertIsNotNone(importlib.util.find_spec('treesim.rm65_camera'),
                         'Stage 3.4 RM camera implementation is absent')
    return importlib.import_module('treesim.rm65_camera')


def mapping():
    # Deliberately noncontiguous; a second environment must not leak into env 0.
    return {'model': 'rm65', 'mount': 'fixed',
            'body_indices': {'base_link': [4, 1], 'gripper_mount': [2, 5],
                             'gripper_left_finger': [6, 0]}}


class ObservationMode(unittest.TestCase):
    def args(self):
        a = grow_tree.parse_args(['--robot-model','rm65','--mount','fixed',
                                 '--rm-control','joint','--apples','--viewer','gl'])
        a.rm_camera = True
        return a

    def test_explicit_observation_is_accepted(self):
        a = self.args()
        try:
            c = grow_tree.make_config(a)
        except ValueError as exc:
            self.fail(f'explicit observation mode rejected: {exc}')
        self.assertTrue(c.robot.camera)
        self.assertTrue(c.robot.rm_camera)
        self.assertTrue(c.fruit.enabled)

    def test_invalid_observation_combinations_fail_before_build(self):
        for field, value in [('robot_model','franka'), ('mount','ridgeback'),
                             ('rm_control','static'), ('apples',False), ('auto',True),
                             ('num_envs',2), ('solver','xpbd'), ('mj_solver','newton'),
                             ('substeps',2), ('warmup',1), ('viewer','null'),
                             ('brk',True), ('no_robot_camera',True)]:
            with self.subTest(field=field):
                a = self.args(); setattr(a,field,value)
                with self.assertRaises(ValueError):
                    grow_tree.make_config(a)

    def test_default_rm_has_no_camera(self):
        m = camera_module(self)
        a = grow_tree.parse_args(['--robot-model','rm65','--mount','fixed','--rm-control','joint'])
        c = grow_tree.make_config(a)
        self.assertFalse(c.robot.camera)
        # No model or renderer is supplied: disabled factory must return before either is touched.
        self.assertIsNone(m.create_rm_camera(None, None, SimpleNamespace(config=c), c.robot))

    def test_franka_default_configuration_unchanged(self):
        c = grow_tree.make_config(grow_tree.parse_args(['--robot']))
        self.assertEqual((c.robot.model,c.robot.mount),('franka','ridgeback'))
        self.assertTrue(c.robot.camera)


class BindingContracts(unittest.TestCase):
    def test_exact_per_environment_names_without_franka_keys(self):
        m = camera_module(self)
        b = m.CameraBinding(mapping(), env=1)
        self.assertEqual(b.parent_body,5)
        self.assertEqual(b.self_bodies, frozenset((1,5,0)))

    def test_missing_duplicate_and_wrong_parent_rejected(self):
        m = camera_module(self)
        for data in [dict(model='rm65',mount='fixed',body_indices={'base_link':[1]}),
                     dict(model='rm65',mount='fixed',body_indices={'base_link':[1],'gripper_mount':[1]})]:
            with self.assertRaises(ValueError):
                m.CameraBinding(data)
        with self.assertRaises(ValueError):
            m.CameraBinding(mapping(),parent_link='fr3_hand')
        with self.assertRaises(ValueError):
            m.CameraBinding(mapping(),env=2)

    def test_invalid_extrinsics_rejected(self):
        m = camera_module(self)
        for xyz in [(1,2), (np.nan,0,0), (np.inf,0,0)]:
            with self.assertRaises(ValueError): m.CameraBinding(mapping(),translation=xyz)
        for quat in [(0,0,0,0),(0,0,0,2),(np.nan,0,0,1),(0,0,1)]:
            with self.assertRaises(ValueError): m.CameraBinding(mapping(),quaternion_xyzw=quat)

    def test_fixed_axes_and_independent_matrix_product(self):
        m = camera_module(self); b = m.CameraBinding(mapping())
        # Geometric expectation derived from the approved look-at and parent +Y.
        z = np.array([.11,0,-.35]); z /= np.linalg.norm(z)
        expected = np.eye(4)
        expected[:3,:3] = np.column_stack((np.cross([0,1,0],z),[0,1,0],z))
        expected[:3,3] = [.11,0,.03]
        np.testing.assert_allclose(b.T_parent_camera,expected,atol=1e-12)
        parent = np.eye(4); parent[:3,:3]=Rotation.from_euler('xyz',[.3,-.7,1.1]).as_matrix()
        parent[:3,3]=[1.8,.2,.9]
        poses=np.tile([0,0,0,0,0,0,1.],(7,1))
        poses[2]=np.r_[parent[:3,3],Rotation.from_matrix(parent[:3,:3]).as_quat()]
        np.testing.assert_allclose(b.world_pose(poses),parent@expected,atol=1e-12)

    def test_frame_roundtrip(self):
        m=camera_module(self); b=m.CameraBinding(mapping())
        poses=np.tile([0,0,0,0,0,0,1.],(7,1)); poses[2,:3]=[.4,.7,1.2]
        camera=b.world_pose(poses)
        base=np.eye(4);base[:3,:3]=Rotation.from_euler('z',1.2).as_matrix();base[:3,3]=[1.8,0,0]
        point=np.array([.3,-.2,-.7,1.])
        world=camera@point
        local=np.linalg.inv(base)@world
        np.testing.assert_allclose(np.linalg.inv(camera)@base@local,point,atol=1e-12)

    def test_parent_motion_updates_pose_but_finger_motion_does_not(self):
        m=camera_module(self);b=m.CameraBinding(mapping())
        poses=np.tile([0,0,0,0,0,0,1.],(7,1)); before=b.world_pose(poses)
        poses[6,:3]=[.3,.2,.1]
        np.testing.assert_array_equal(b.world_pose(poses),before)
        poses[2,:3]=[.2,-.1,.5]
        np.testing.assert_allclose(b.world_pose(poses)[:3,3]-before[:3,3],[.2,-.1,.5],atol=1e-12)

    def test_exact_mask_preserves_environment_and_other_robot(self):
        m=camera_module(self);b=m.CameraBinding(mapping())
        hit=np.array([[0,1,2,3,4,0xffffffff]],dtype=np.uint32)
        shape_body=np.array([2,5,-1,6,3])
        masked, own=b.filter_depth(np.ones(hit.shape),hit,shape_body)
        np.testing.assert_array_equal(own,[[True,False,False,True,False,False]])
        self.assertTrue(np.isnan(masked[0,0]) and np.isnan(masked[0,3]))
        np.testing.assert_array_equal(masked[0,[1,2,4,5]],np.ones(4))
        with self.assertRaises(ValueError):
            b.filter_depth(np.ones((1,1)),np.array([[99]],dtype=np.uint32),shape_body)


class SensorContracts(unittest.TestCase):
    def fixture(self, occluder=True):
        import newton
        import warp as wp
        m=camera_module(self)
        builder=newton.ModelBuilder()
        parent=builder.add_body(label='gripper_mount',xform=wp.transform_identity())
        # Optical center at origin, camera -Z forward for this rendering contract fixture.
        if occluder:
            builder.add_shape_box(parent,xform=wp.transform(wp.vec3(0,0,-.3),wp.quat_identity()),hx=.15,hy=.15,hz=.02)
        target=builder.add_body(label='environment_apple',xform=wp.transform(wp.vec3(0,0,-.6),wp.quat_identity()))
        builder.add_shape_sphere(target,radius=.035)
        model=builder.finalize(device='cpu')
        data=dict(model='rm65',mount='fixed',body_indices={'gripper_mount':[parent]})
        cfg=grow_tree.make_config(grow_tree.parse_args(['--robot-model','rm65','--mount','fixed']))
        cfg.robot.camera_width=64;cfg.robot.camera_height=48
        binding=m.CameraBinding(data,translation=(0,0,0),quaternion_xyzw=(0,0,0,1))
        tm=SimpleNamespace(robot_data=data,num_envs=1)
        camera=m.RMSyntheticCamera(model,None,tm,cfg.robot,binding=binding)
        return camera,model.state(),model

    def test_unique_samples_hold_owned_synchronized_buffers(self):
        camera,state,model=self.fixture(False)
        a=camera.update(state,0.,0)
        self.assertEqual((a.source,a.sample_id,a.simulation_time),('orchardbench_synthetic',0,0.))
        depth=a.depth.copy();pose=a.T_world_camera.copy()
        self.assertIsNone(camera.update(state,0.,0))
        self.assertIsNone(camera.update(state,1/60,1))
        b=camera.update(state,.2,12)
        self.assertEqual((b.sample_id,b.simulation_time),(1,.2))
        np.testing.assert_array_equal(a.depth,depth)
        np.testing.assert_array_equal(a.T_world_camera,pose)
        self.assertFalse(np.shares_memory(a.depth,b.depth))
        self.assertEqual(a.depth.shape,a.shape_index.shape)
        self.assertTrue(np.all(np.isfinite(a.T_world_camera)))
        with self.assertRaises(ValueError): camera.update(state,.1,13)

    def test_self_surface_occludes_without_hiding_geometry(self):
        camera,state,model=self.fixture(True)
        flags=model.shape_flags.numpy().copy()
        sample=camera.update(state,0.,0)
        # The middle ray hits the RM box at .28 m, not the sphere behind it.
        self.assertLess(float(sample.depth[24,32]),.31)
        self.assertTrue(sample.self_mask[24,32])
        self.assertTrue(np.isnan(sample.detection_depth[24,32]))
        self.assertEqual(len(sample.detections_world),0)
        np.testing.assert_array_equal(model.shape_flags.numpy(),flags)



class ViewerAndFrankaContracts(unittest.TestCase):
    def test_rm_observation_disables_mouse_force_but_preserves_franka(self):
        # GL context is the external dependency; make_viewer's mode decision is real.
        from unittest.mock import patch
        for observation in (False, True):
            a=grow_tree.parse_args(['--viewer','gl','--headless'])
            a.rm_camera=observation
            viewer=SimpleNamespace(picking_enabled=True)
            with patch('newton.viewer.ViewerGL',return_value=viewer):
                actual=grow_tree.make_viewer(a)
            self.assertEqual(actual.picking_enabled,not observation)

    def test_original_franka_binding_world_up_mask_and_aliases(self):
        import newton
        import warp as wp
        from treesim.robot import WristCamera
        with wp.ScopedDevice('cpu'):
            build=newton.ModelBuilder()
            for i in range(5):
                body=build.add_body(label='fr3_hand' if i==1 else f'fixture{i}',
                    xform=wp.transform(wp.vec3(0,0,0),wp.quat(0,0,2**-.5,2**-.5)) if i==1 else wp.transform_identity())
                build.add_shape_box(body,xform=wp.transform(wp.vec3(10,10,10),wp.quat_identity()),hx=.01,hy=.01,hz=.01)
            model=build.finalize(device='cpu')
            tm=SimpleNamespace(robot_data={'wrist':[1],'chassis':[0]},num_envs=1)
            cfg=grow_tree.make_config(grow_tree.parse_args(['--robot']))
            class Display:
                def log_image(self,*args,**kw): pass
                def log_points(self,*args,**kw): pass
            camera=WristCamera(model,Display(),tm,cfg.robot)
            camera.update(model.state())
            p,R=camera.cam_env[0]
            n=(.11**2+.35**2)**.5;a=.11/n;b=.35/n
            np.testing.assert_allclose(p,[0,.11,.03],atol=1e-7)
            np.testing.assert_allclose(R,[[-1,0,0],[0,b,a],[0,a,-b]],atol=1e-7)
            np.testing.assert_array_equal(camera._self_bodies[0],[0,1,2,3,4])
            self.assertIs(camera.detections,camera.dets_env[0])
            self.assertIs(camera.last_depth,camera.depth_env[0])
            self.assertEqual(camera.last_depth.shape,(144,192))

if __name__=='__main__': unittest.main()
