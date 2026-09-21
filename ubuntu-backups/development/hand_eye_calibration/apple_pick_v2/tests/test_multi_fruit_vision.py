import importlib.util
import unittest
from test_multi_fruit_state import detection

class VisionTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec('multi_fruit_vision'), 'snapshot adapter is missing')
        from multi_fruit_vision import build_snapshot
        self.build = build_snapshot

    def test_valid_empty_is_distinct_from_failed_estimate(self):
        self.assertTrue(self.build(10., [], lambda d: None)['valid'])
        self.assertFalse(self.build(10., [dict(touches_edge=False)],lambda d: None)['valid'])

    def test_all_detections_are_estimated_and_deduplicated(self):
        candidates=[dict(touches_edge=False, fruit=detection(x,c)) for x,c in [(.2,.9),(.201,.8),(.4,.7)]]
        result=self.build(10.,candidates,lambda c:c['fruit'],model_duration_s=.012345)
        self.assertEqual(len(result['detections']),2)
        self.assertEqual(result['capture_stamp_s'],10.)
        self.assertAlmostEqual(result['source_model_duration_s'],.012345)

    def test_uncovered_partial_detection_invalidates_whole_snapshot(self):
        result=self.build(10.,[dict(touches_edge=True),dict(touches_edge=False,fruit=detection(.2))],lambda c:c['fruit'])
        self.assertFalse(result['valid'])

if __name__=='__main__': unittest.main()


class TileEfficiencyTests(unittest.TestCase):
    def test_tile_duplicates_localize_once_but_distinct_fruit_remain(self):
        from multi_fruit_vision import build_snapshot
        boxes=[dict(touches_edge=False,class_id=49,confidence=c,global_xyxy=b)
               for b,c in [([10,10,50,50],.9),([11,10,51,50],.8),([48,10,88,50],.7)]]
        calls=[]
        def estimate(box):
            calls.append(box)
            return detection(box['global_xyxy'][0]/100.)
        result=build_snapshot(10.,boxes,estimate)
        self.assertEqual(len(calls),2)
        self.assertEqual(len(result['detections']),2)
        self.assertGreaterEqual(result['source_localization_duration_s'],0.)
