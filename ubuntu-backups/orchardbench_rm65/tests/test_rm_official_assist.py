import importlib.util
import unittest
import numpy as np

class OfficialAssistTests(unittest.TestCase):
    def api(self):
        self.assertIsNotNone(importlib.util.find_spec('treesim.rm65_official'), 'official adapter missing')
        from treesim import rm65_official as m
        return m

    def test_bucket_score_wait_and_release(self):
        m=self.api(); b=m.Bucket((1.,2.,.31))
        self.assertFalse(b.placed([1.,2.,.4],released=False,elapsed=100))
        self.assertFalse(b.placed([1.,2.,.4],released=True,elapsed=59))
        self.assertTrue(b.placed([1.,2.,.4],released=True,elapsed=60))
        self.assertFalse(b.placed([1.18,2.,.4],released=True,elapsed=60))
        self.assertFalse(b.placed([1.,2.,.9],released=True,elapsed=60))

    def test_release_target_keeps_wrist_above_rim(self):
        m=self.api();b=m.Bucket((.95,.1,.1))
        self.assertGreaterEqual(b.release_point[2]-(b.floor[2]+.15),.20)

    def test_world_bucket_has_no_massless_dynamic_body(self):
        import newton
        m=self.api();b=newton.ModelBuilder();m.Bucket((.95,.1,.1)).build(b)
        self.assertEqual(b.body_count,0)
        self.assertEqual(b.shape_body,[-1]*5)

    def test_contacts_exclude_palm_stem_and_robot_bucket(self):
        m=self.api()
        self.assertTrue(m.expected_contact('gripper_left_finger','apple35',True))
        self.assertTrue(m.expected_contact('apple35','rm_receiving_bucket',False))
        for a,b in [('gripper_palm','apple35'),('gripper_left_finger','seg294'),('gripper_right_finger','rm_receiving_bucket')]:
            self.assertFalse(m.expected_contact(a,b,True))
        self.assertFalse(m.expected_contact('gripper_left_finger','apple35',False))

    def test_perception_requires_new_unique_frames(self):
        from types import SimpleNamespace as N
        m=self.api(); p=m.PerceptionAdapter()
        make=lambda i,centers:N(sample_id=i,simulation_time=i*.2,detections_world=[N(center_world=np.array(c)) for c in centers])
        np.testing.assert_allclose(p.accept(make(1,[[.8,.4,.6]])),[.8,.4,.6])
        with self.assertRaises(RuntimeError):p.accept(make(1,[[.8,.4,.6]]))
        with self.assertRaises(RuntimeError):p.accept(make(2,[]))
        with self.assertRaises(RuntimeError):p.accept(make(3,[[.8,.4,.6],[.81,.4,.6]]))
        with self.assertRaises(RuntimeError):p.accept(make(4,[[1.,.4,.6]]))

    def test_hold_requires_closed_stable_and_fresh_close_distance(self):
        m=self.api()
        self.assertTrue(m.hold_allowed(.004,True,True))
        for d,closed,stable in [(.006,True,True),(.004,False,True),(.004,True,False),(float('nan'),True,True)]:
            self.assertFalse(m.hold_allowed(d,closed,stable))

    def test_release_does_not_count_as_place(self):
        self.assertIsNotNone(importlib.util.find_spec('treesim.rm65_official_strategy'), 'local strategy missing')
        from treesim.rm65_official_strategy import LocalGraspStrategy
        from treesim.metrics import Metrics
        s=LocalGraspStrategy.__new__(LocalGraspStrategy);s.met=Metrics();s.met.pick_start(35,[0,0,0])
        s.met.pick_event('grasp');s.met.pick_event('detach')
        s.finish_metrics(False)
        self.assertEqual(s.met.summary()['grasp_success'],1)
        self.assertEqual(s.met.summary()['pick_success'],1)
        self.assertEqual(s.met.summary()['place_success'],0)

    def test_conservative_bound_never_certifies_overlapping_spheres(self):
        m=self.api();self.assertTrue(hasattr(m,'sphere_separation'))
        self.assertAlmostEqual(m.sphere_separation([0,0,0],.2,[.5,0,0],.2),.1)
        self.assertLessEqual(m.sphere_separation([0,0,0],.2,[.3,0,0],.2),0)

    def test_bucket_clearance_uses_proven_separation(self):
        from types import SimpleNamespace as N
        m=self.api();self.assertTrue(hasattr(m,'BucketClearance'))
        c=m.BucketClearance.__new__(m.BucketClearance)
        c.n=N(shape_label=['robot','rm_receiving_bucket/wall0']);c.gmap=np.array([0,1])
        c.d=N(geom_xpos=np.array([[0.,0,0],[.5,0,0]]));c.m=N(geom_rbound=np.array([.2,.2]))
        self.assertAlmostEqual(c.distance(0,1),.1)

    def test_closed_hand_geometry_and_measurement(self):
        m=self.api();self.assertTrue(hasattr(m,'scratch_joints'))
        snapshot=np.arange(10,dtype=float)
        q=m.scratch_joints(snapshot,np.arange(8),np.ones(6),[.025,.03])
        np.testing.assert_allclose(q[6:8],[.025,.03]);self.assertEqual(q[9],9)
        np.testing.assert_array_equal(snapshot,np.arange(10))
        self.assertFalse(m.measured_closed([0,0],[0,0],set()))
        self.assertFalse(m.measured_closed([.025,.025],[0,0],set()))
        self.assertTrue(m.measured_closed([.025,.025],[0,0],{0,1}))
        self.assertTrue(m.measured_closed([.06,.06],[0,0],set()))

    def test_stability_counts_every_sample(self):
        m=self.api();self.assertTrue(hasattr(m,'StableWindow'))
        w=m.StableWindow()
        for _ in range(44):self.assertFalse(w.update(True))
        self.assertFalse(w.update(False))
        for _ in range(44):self.assertFalse(w.update(True))
        self.assertTrue(w.update(True))

    def test_closed_hand_cannot_hold_before_new_camera_confirmation(self):
        from types import SimpleNamespace as N
        from treesim.rm65_official_strategy import LocalGraspStrategy
        from treesim.metrics import Metrics
        s=LocalGraspStrategy.__new__(LocalGraspStrategy)
        s.state='CLOSE';s.close_count=45;s.met=Metrics();s.samples=3
        s.r=N(frames=100);s.event=lambda *a,**k:None
        s.robot=N(actual=lambda:np.eye(4));s.perception=N(center=np.zeros(3))
        s.association=N(index=35)
        def premature_hold(*args,**kwargs):self.fail('hold before fresh camera confirmation')
        s.gripper=N(hold=premature_hold)
        s.after_frame(s.r)
        self.assertEqual(s.state,'HOLD_CONFIRM')
        self.assertEqual(s.samples,0)

if __name__=='__main__':unittest.main()
