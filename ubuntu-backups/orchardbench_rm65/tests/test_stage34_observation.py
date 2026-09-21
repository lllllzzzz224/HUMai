"""Observation readiness is independent of commanded TCP arrival."""
import unittest
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from scripts import validate_rm65_camera as val
from treesim.rm65_acceptance import arrived, ArrivalWindow

class ObservationReadiness(unittest.TestCase):
    def window(self):
        self.assertTrue(callable(getattr(val,'ObservationWindow',None)),
                        'Stage 3.4 still uses the TCP arrival window')
        return val.ObservationWindow()

    def update(self,w,i,*,goal=None,sent=None,buffer=None,velocity=None,pose=None,expected=None,safety=True):
        goal=np.array([-.4,-.8,-.3,0,-.2,0.]) if goal is None else np.asarray(goal)
        pose=np.eye(4) if pose is None else pose
        return w.update(time_s=i/180.,target_buffer=goal.astype(np.float32) if buffer is None else buffer,
            effective_target=goal,sent_target=goal if sent is None else sent,
            velocity=np.zeros(6) if velocity is None else velocity,
            actual_camera=pose,expected_camera=pose if expected is None else expected,safety_ok=safety)

    def test_large_command_tcp_error_does_not_block_stable_observation(self):
        w=self.window()
        for i in range(46): ready=self.update(w,i)
        self.assertTrue(ready)
        self.assertTrue(w.ready)
        self.assertFalse(arrived(.014282,np.deg2rad(1.541),np.zeros(6)))
        old=ArrivalWindow()
        for _ in range(100): self.assertFalse(old.update(.014282,np.deg2rad(1.541),np.zeros(6)))
        self.assertTrue(arrived(.005,np.deg2rad(1),np.full(6,.005)))

    def test_full_quarter_second_required(self):
        w=self.window()
        for i in range(45): self.assertFalse(self.update(w,i))
        self.assertTrue(self.update(w,45))

    def test_unfinished_slew_or_incorrect_target_buffer_cannot_be_ready(self):
        for kind in ('sent','buffer'):
            w=self.window()
            for i in range(60):
                goal=np.array([-.4,-.8,-.3,0,-.2,0.]);bad=goal.copy();bad[0]+=.00001
                self.assertFalse(self.update(w,i,**{kind:bad}))

    def test_moving_arm_resets_consecutive_window(self):
        w=self.window()
        for i in range(46): self.update(w,i)
        self.assertFalse(self.update(w,46,velocity=np.array([0,0,.00501,0,0,0])))
        for i in range(47,92): self.assertFalse(self.update(w,i))
        self.assertTrue(self.update(w,92))

    def test_camera_position_and_rotation_peak_to_peak_are_checked(self):
        for kind in ('position','rotation'):
            w=self.window()
            for i in range(60):
                p=np.eye(4);sign=1 if i%2 else -1
                if kind=='position': p[0,3]=sign*.0003
                else: p[:3,:3]=Rotation.from_euler('y',sign*.06,degrees=True).as_matrix()
                self.assertFalse(self.update(w,i,pose=p))

    def test_camera_transform_mismatch_rejected_even_when_stationary(self):
        for kind in ('position','rotation'):
            w=self.window();p=np.eye(4)
            if kind=='position':p[0,3]=1.01e-5
            else:p[:3,:3]=Rotation.from_rotvec([0,0,1.01e-4]).as_matrix()
            with self.assertRaisesRegex(RuntimeError,'CAMERA_FRAME_MISMATCH'):
                self.update(w,0,pose=p,expected=np.eye(4))

    def test_target_change_restarts_stable_window(self):
        w=self.window()
        for i in range(46):self.update(w,i)
        self.assertFalse(self.update(w,46,goal=[-.4,-.8,-.3,0,-.2001,0]))

    def test_bad_state_safety_and_repeated_time_fail_closed(self):
        w=self.window()
        with self.assertRaises(RuntimeError):self.update(w,0,safety=False)
        w=self.window()
        with self.assertRaises(ValueError):self.update(w,0,velocity=np.full(6,np.nan))
        w=self.window();self.update(w,0)
        with self.assertRaises(ValueError):self.update(w,0)

    def test_case_uses_observation_window_not_stage33_gate(self):
        expected=self.window()
        case=val.ObservationCase(SimpleNamespace(log_dir=None))
        self.assertIsInstance(case.gate,type(expected))

if __name__=='__main__':unittest.main()
