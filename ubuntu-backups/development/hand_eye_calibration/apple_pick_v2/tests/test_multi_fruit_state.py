import importlib.util
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def detection(x, confidence=.9, cls=49):
    return dict(class_id=cls, class_name='orange' if cls == 49 else 'apple',
                confidence=confidence, center_xyz_m=[x, 0., .1], radius_m=.03)


def snapshot(stamp, detections, valid=True):
    return dict(version=1, frame='base_link', capture_stamp_s=stamp,
                valid=valid, detections=detections)


class StateTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('multi_fruit_state'), 'multi-target scheduler is missing')
        from multi_fruit_state import TargetManager
        self.manager = TargetManager(class_id=49, stable_frames=2)

    def feed(self, detections, start=10.):
        self.manager.update(snapshot(start, detections), start)
        self.manager.update(snapshot(start+.1, detections), start+.1)

    def test_literal_three_fruit_confidence_order_and_exclusion(self):
        self.feed([detection(.2,.75), detection(.4,.95), detection(.6,.85)])
        ordered=[]
        for _ in range(3):
            target=self.manager.lock(10.1)
            ordered.append(target.center[0])
            self.manager.mark_attempted(target.id)
        self.assertEqual(ordered,[.4,.6,.2])
        self.assertEqual(self.manager.summary(10.1),'ALL_ATTEMPTED')
        self.feed([detection(.2,.75), detection(.4,.95), detection(.6,.85)],11.)
        self.assertEqual(self.manager.eligible(11.1),[])

    def test_tile_duplicates_and_same_track_not_double_used(self):
        self.feed([detection(.2),detection(.202,.7)])
        self.assertEqual(len(self.manager.eligible(10.1)),1)

    def test_stale_and_reordered_capture_do_not_refresh(self):
        self.feed([detection(.2)])
        self.manager.update(snapshot(10.,[detection(.2)]),12.)
        self.assertEqual(self.manager.summary(12.),'STALE_PERCEPTION')

    def test_invalid_not_empty(self):
        self.feed([detection(.2)])
        self.manager.update(snapshot(10.2,[],False),10.2)
        self.assertEqual(self.manager.summary(10.2),'INVALID_PERCEPTION')
        self.manager.update(snapshot(10.3,[]),10.3)
        self.assertEqual(self.manager.summary(10.3),'NO_SAFE_TARGET')
        self.manager.update(snapshot(10.4,[]),10.4)
        self.assertEqual(self.manager.summary(10.4),'SCENE_EMPTY')

    def test_locked_target_disappears(self):
        self.feed([detection(.2),detection(.4)])
        target=self.manager.lock(10.1)
        self.manager.update(snapshot(10.2,[detection(.4)]),10.2)
        self.assertFalse(self.manager.revalidate(target,10.2))

    def test_ambiguous_moved_neighbor_is_quarantined(self):
        self.feed([detection(.2),detection(.26)])
        self.manager.mark_attempted(self.manager.lock(10.1).id)
        self.feed([detection(.23)],10.2)
        self.assertEqual(self.manager.eligible(10.3),[])

    def test_reappearance_keeps_attempted_identity(self):
        self.feed([detection(.2)])
        self.manager.mark_attempted(self.manager.lock(10.1).id)
        self.feed([],10.2)
        self.feed([detection(.2)],10.4)
        self.assertEqual(self.manager.summary(10.5),'ALL_ATTEMPTED')

    def test_post_motion_boundary_requires_new_capture(self):
        self.feed([detection(.2)])
        target=self.manager.lock(10.1)
        self.manager.motion_boundary(10.2)
        self.assertFalse(self.manager.revalidate(target,10.2))

    def test_direct_large_move_of_attempted_fruit_cannot_get_free_identity(self):
        self.feed([detection(.2)])
        self.manager.mark_attempted(self.manager.lock(10.1).id)
        self.feed([detection(.4)],10.2)
        self.assertEqual(self.manager.eligible(10.3),[])

    def test_unique_unattempted_movement_resets_stability_then_refreshes(self):
        self.feed([detection(.2)])
        frozen=self.manager.lock(10.1)
        self.manager.update(snapshot(10.2,[detection(.22)]),10.2)
        self.assertFalse(self.manager.revalidate(frozen,10.2))
        self.manager.update(snapshot(10.3,[detection(.22)]),10.3)
        self.assertAlmostEqual(self.manager.lock(10.3).center[0],.22)

    def test_boundary_requires_all_stability_evidence_to_be_new(self):
        self.feed([detection(.2)])
        self.manager.motion_boundary(10.2)
        self.manager.update(snapshot(10.3,[detection(.2)]),10.3)
        self.assertEqual(self.manager.eligible(10.3),[])
        self.manager.update(snapshot(10.4,[detection(.2)]),10.4)
        self.assertEqual(len(self.manager.eligible(10.4)),1)

if __name__=='__main__': unittest.main()
