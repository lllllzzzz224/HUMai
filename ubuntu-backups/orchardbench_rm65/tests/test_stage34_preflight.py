"""Regression for bounded offline paths; no model, controller or live state."""
import unittest
from unittest.mock import patch
import numpy as np
from scripts import validate_rm65_camera as validator
from treesim import rm65_control as rc

class ScratchSlewContracts(unittest.TestCase):
    def path(self,start,target):
        self.assertTrue(callable(getattr(validator,'scratch_slew_path',None)),
                        'bounded scratch_slew_path is missing; prepare still loops with array_equal')
        return validator.scratch_slew_path(start,target)

    def test_original_float32_decimal_loop_stagnates_but_new_path_terminates(self):
        target=np.array([.1,.2,.3,.4,.8,0],dtype=np.float64)
        old=np.zeros(6,dtype=np.float32)
        for _ in range(1000): old+=np.clip(target-old,-rc.RATE[:6]/60,rc.RATE[:6]/60)
        previous=old.copy();old+=np.clip(target-old,-rc.RATE[:6]/60,rc.RATE[:6]/60)
        self.assertFalse(np.array_equal(old,target))
        np.testing.assert_array_equal(old,previous)
        samples,audit=self.path(np.zeros(6,dtype=np.float32),target)
        self.assertLessEqual(audit['iterations'],482)
        self.assertLessEqual(audit['final_max_residual_rad'],audit['numerical_tolerance_rad'])
        self.assertEqual(len(samples),audit['iterations']+1)

    def test_positive_negative_and_mixed_axes_respect_original_slew(self):
        for target in ([.1,.2,.3,.4,.8,.1],[-.1,-.2,-.3,-.4,-.8,-.1],
                       [-.4,.2,-.3,.1,.8,-.2]):
            with self.subTest(target=target):
                start=np.zeros(6,dtype=np.float32);goal=np.array(target,dtype=np.float64)
                samples,audit=self.path(start,goal)
                values=np.asarray(samples)
                self.assertEqual(values.dtype,np.float64)
                self.assertTrue(np.all(np.abs(np.diff(values,axis=0))<=rc.RATE[:6]/60+1e-15))
                self.assertTrue(np.all(np.abs(values[-1]-goal)<=audit['numerical_tolerance_rad']))
                self.assertLessEqual(audit['iterations'],audit['iteration_bound'])
                self.assertLessEqual(audit['iteration_bound'],482)

    def test_already_within_representation_tolerance_terminates_without_movement(self):
        target=np.array([.1,.2,.3,.4,.8,-.2],dtype=np.float64)
        start=target.astype(np.float32)
        samples,audit=self.path(start,target)
        self.assertEqual(audit['iterations'],0)
        self.assertEqual(audit['termination_reason'],'numerical_tolerance')
        np.testing.assert_array_equal(samples[0],start.astype(np.float64))
        self.assertLessEqual(audit['final_max_residual_rad'],audit['numerical_tolerance_rad'])

    def test_nonfinite_and_malformed_inputs_reject_atomically(self):
        # No lazy generator can yield a valid prefix before rejecting the request.
        self.path(np.zeros(6),np.zeros(6))
        for bad in ([0,0,np.nan,0,0,0],[0,np.inf,0,0,0,0],[-np.inf,0,0,0,0,0],[0,0]):
            for which in (0,1):
                values=[np.zeros(6),np.zeros(6)];values[which]=np.asarray(bad)
                original=[v.copy() for v in values]
                with self.assertRaises(ValueError): validator.scratch_slew_path(*values)
                for a,b in zip(values,original): np.testing.assert_array_equal(a,b)

    def test_owns_copies_and_cannot_write_live_q_or_qd(self):
        live_q=np.zeros(8,dtype=np.float32);live_qd=np.arange(8,dtype=np.float32)
        target=np.array([.1,-.2,.3,-.4,.8,0.])
        before_q=live_q.copy();before_qd=live_qd.copy();before_target=target.copy()
        live_q.setflags(write=False);live_qd.setflags(write=False);target.setflags(write=False)
        samples,audit=self.path(live_q[:6],target)
        for sample in samples:
            self.assertFalse(np.shares_memory(sample,live_q))
            self.assertFalse(np.shares_memory(sample,target))
        np.testing.assert_array_equal(live_q,before_q)
        np.testing.assert_array_equal(live_qd,before_qd)
        np.testing.assert_array_equal(target,before_target)

    def test_iteration_guard_fails_explicitly_on_nonprogress(self):
        self.path(np.zeros(6),np.zeros(6))
        # Fault injection into the numerical step, not into the guard under test.
        with patch.object(validator.np,'clip',return_value=np.zeros(6)):
            with self.assertRaisesRegex(RuntimeError,'PREFLIGHT_ITERATION_BOUND'):
                validator.scratch_slew_path(np.zeros(6),np.full(6,.001))


class ViewerDirectoryContracts(unittest.TestCase):
    def test_main_isolates_viewer_files_and_restores_cwd_on_failure(self):
        import tempfile
        from pathlib import Path
        root=Path.cwd()
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)/'case'
            def stopped_gl(*args,**kw):
                self.assertEqual(Path.cwd(),out/'viewer-workdir')
                Path('imgui.ini').write_text('isolated viewer fixture')
                raise RuntimeError('controlled GL stop')
            with patch.object(validator.grow_tree,'main',side_effect=stopped_gl):
                with self.assertRaisesRegex(RuntimeError,'controlled GL stop'):
                    validator.main(['--case','clear','--apple','apple35','--q','0','0','0','0','0','0',
                                    '--log-dir',str(out)])
            self.assertEqual(Path.cwd(),root)
            self.assertTrue((out/'viewer-workdir/imgui.ini').is_file())

if __name__=='__main__': unittest.main()
