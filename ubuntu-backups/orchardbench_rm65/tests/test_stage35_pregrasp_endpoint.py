"""Only opt-in PREGRASP arrival may use 8 mm; all other gates stay intact."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import test_stage35_pregrasp_policy as fixtures
from treesim import rm65_close_only as co


class PregraspEndpointTests(unittest.TestCase):
    def program(self, distance=.006, enabled=True, velocity=0., collision=False, q=None):
        p,state=fixtures.TrackingPolicyTests().make_program(collision=collision,q=q)
        p.tracking.enabled=enabled
        p.goal_pose[0,3]=distance
        p.r.controller.sent[0]=distance;p.r.controller.goal[0]=distance
        p.r.controller.effective_arm_target[0]=distance
        if velocity:
            p.r.controller.read_arm_state=lambda s:(np.zeros(6),np.full(6,velocity))
        p.r.controller.check_state=lambda s:None
        p.r.sim.state_0=state;p.r.frames=0;p.r.max_base_error=0.;p.r.stopped=False
        p.r.audit.check_no_contact=lambda:None
        p.r.sim.render=lambda:None
        p.r.sim.viewer=NS(get_frame=lambda:fixtures.array(np.zeros((8,8,3),dtype=np.uint8)))
        p.gate.ready=True;p.motion=[NS()];p.motion_index=0
        return p,state

    def test_5_to_8mm_endpoint_needs_45_substeps_then_stops(self):
        for distance in (.005347,.006,.008):
            with self.subTest(distance=distance),tempfile.TemporaryDirectory() as d:
                p,state=self.program(distance);p.output=Path(d)
                for step in range(1,45):
                    p.r.substeps=step;p.after_substep(state)
                self.assertEqual(p.arrival_count,44)
                p.after_frame(p.r);self.assertFalse(p.done)
                p.r.substeps=45;p.after_substep(state);p.after_frame(p.r)
                self.assertEqual(p.state,'PREGRASP_ONLY_DONE');self.assertTrue(p.done)
                self.assertTrue(p.r.stopped);self.assertFalse(p.revalidated)
                result=json.loads((Path(d)/'result.json').read_text())
                self.assertAlmostEqual(result['endpoint']['tcp_position_error_m'],distance)
                self.assertFalse(p.close_authorized)

    def test_strictly_above_8mm_still_immediately_fails(self):
        p,state=self.program(np.nextafter(.008,np.inf))
        with self.assertRaisesRegex(RuntimeError,'TRACKING_ERROR'):p.after_substep(state)
        self.assertEqual(p.arrival_count,0);self.assertFalse(p.done)

    def test_default_and_ready_endpoints_remain_5mm(self):
        for stage,enabled in [('PREGRASP',False),('READY',True),('GRASP',True)]:
            p,state=self.program(.006,enabled=enabled);p.sequence.state=stage
            with self.assertRaisesRegex(RuntimeError,'UNSTABLE_STAGE_BOUNDARY'):p.boundary()
        p,state=self.program(.005,enabled=False);p.boundary()

    def test_orientation_failure_cannot_accumulate_arrival(self):
        p,state=self.program();a=np.deg2rad(1.001)
        p.goal_pose[:3,:3]=[[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]]
        p.after_substep(state);self.assertEqual(p.arrival_count,0)
        with self.assertRaisesRegex(RuntimeError,'UNSTABLE_STAGE_BOUNDARY'):p.boundary()

    def test_velocity_failure_cannot_accumulate_arrival(self):
        p,state=self.program(velocity=.00501)
        p.after_substep(state);self.assertEqual(p.arrival_count,0)
        with self.assertRaisesRegex(RuntimeError,'UNSTABLE_STAGE_BOUNDARY'):p.boundary()

    def test_camera_stability_still_required(self):
        p,state=self.program();p.gate.ready=False;p.arrival_count=45
        p.after_frame(p.r);self.assertFalse(p.done)
        with self.assertRaisesRegex(RuntimeError,'UNSTABLE_STAGE_BOUNDARY'):p.boundary()

    def test_collision_limit_and_fk_failures_cannot_pass(self):
        for kwargs,reason in [({'collision':True},'COLLISION'),({'q':[3.1,0,0,0,0,0]},'JOINT_MARGIN')]:
            p,state=self.program(**kwargs)
            with self.assertRaisesRegex(RuntimeError,reason):p.after_substep(state)
            self.assertFalse(p.done)
        p,state=self.program();bad=np.eye(4);bad[0,3]=.0002;p.actual_pose=lambda s:bad
        with self.assertRaisesRegex(RuntimeError,'ACTUAL_FK_MISMATCH'):p.after_substep(state)

    def test_close_fruit_distance_and_revalidation_remain_strict(self):
        for distance,fresh in [(.00501,True),(.008,True),(.004,False)]:
            with self.assertRaisesRegex(RuntimeError,'CLOSE_GATE'):
                co.close_gate('GRASP',True,fresh,[distance,0,0],[0,0,0])

    def test_original_deadline_is_still_terminal(self):
        p,state=self.program();p.started=True;p.deadline=42;p.r.frames=42
        with self.assertRaisesRegex(RuntimeError,'STAGE_MOTION_TIMEOUT: PREGRASP'):p.before_frame(p.r)
