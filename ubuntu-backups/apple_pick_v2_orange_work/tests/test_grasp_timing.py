import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


MODULE = Path(__file__).resolve().parents[1] / 'analyze_grasp_timing.py'


class TimingAnalysisTest(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('timing_analysis', MODULE)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)

    def sample(self):
        return {
            'timestamp': '2026-08-11 20:00:10',
            'finished_at': '2026-08-11 20:00:36',
            'status': 'PASS_PICK_RETURN_COMPLETE',
            'events': [
                {'time': '2026-08-11 20:00:11', 'step': '全部路径预检', 'result': 'PASS'},
                {'time': '2026-08-11 20:00:16', 'step': '自动开始 SCAN -> PREGRASP -> VERIFY'},
                {'time': '2026-08-11 20:00:36', 'step': '返回固定鸟瞰位', 'result': 'PASS'},
            ],
        }

    def test_separates_post_plan_validation_from_planning_and_movement(self):
        result = self.module.analyze_round(self.sample())
        self.assertEqual(result['duration_s'], 26)
        self.assertEqual(result['prepare_to_preflight_s'], 1)
        self.assertEqual(result['post_preflight_to_motion_s'], 5)
        self.assertEqual(result['motion_to_return_s'], 20)

    def test_missing_event_is_unknown_not_zero(self):
        record = self.sample()
        record['events'] = []
        result = self.module.analyze_round(record)
        self.assertIsNone(result['prepare_to_preflight_s'])
        self.assertIsNone(result['motion_to_return_s'])
        self.assertEqual(result['duration_s'], 26)

    def test_failed_motion_has_observed_time_but_no_fabricated_return(self):
        record = self.sample()
        record['status'] = 'FAILED_STOPPED'
        record['events'] = record['events'][:-1]
        result = self.module.analyze_round(record)
        self.assertIsNone(result['motion_to_return_s'])
        self.assertEqual(result['motion_to_finish_s'], 20)

    def test_clock_reversal_and_malformed_timestamp_not_negative_cost(self):
        record = self.sample()
        record['finished_at'] = '2026-08-11 19:00:00'
        self.assertIsNone(self.module.analyze_round(record)['duration_s'])
        record['finished_at'] = 'invalid'
        self.assertIsNone(self.module.analyze_round(record)['duration_s'])

    def test_session_idle_is_not_misreported_as_planner_latency(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'round.json').write_text(json.dumps(self.sample()))
            session = {
                'timestamp': '2026-08-11 20:00:00',
                'finished_at': '2026-08-11 20:01:00',
                'status': 'STOPPED_BY_OPERATOR',
                'completed_rounds': 1,
                'rounds': [{'log': '/old/machine/round.json', 'round_index': 1}],
            }
            path = root / 'continuous.json'
            path.write_text(json.dumps(session))
            result = self.module.analyze_session(path)
            self.assertEqual(result['rounds'][0]['wait_before_s'], 10)
            self.assertEqual(result['terminal_idle_s'], 24)
            self.assertEqual(result['session_duration_s'], 60)
            self.assertEqual(result['rounds'][0]['prepare_to_preflight_s'], 1)

    def test_missing_round_breaks_interval_attribution(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'round.json').write_text(json.dumps(self.sample()))
            path = root / 'continuous.json'
            path.write_text(json.dumps({
                'timestamp': '2026-08-11 20:00:00',
                'finished_at': '2026-08-11 20:01:00',
                'rounds': [{'log': 'missing.json'}, {'log': 'round.json'}],
            }))
            result = self.module.analyze_session(path)
            self.assertEqual(len(result['missing_logs']), 1)
            self.assertIsNone(result['rounds'][0]['wait_before_s'])

    def test_directory_includes_orphans_without_double_counting_linked_rounds(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for name in ['single_round_linked.json', 'single_round_orphan.json']:
                (root / name).write_text(json.dumps(self.sample()))
            (root / 'continuous_apple_grasp_test.json').write_text(json.dumps({
                'rounds': [{'log': '/old/single_round_linked.json'}],
            }))
            result = self.module.analyze_directory(root)
            self.assertEqual(len(result['sessions']), 1)
            self.assertEqual(len(result['unlinked_rounds']), 1)
            self.assertTrue(result['unlinked_rounds'][0]['log'].endswith('single_round_orphan.json'))
            self.assertIsNone(result['unlinked_rounds'][0]['wait_before_s'])


if __name__ == '__main__':
    unittest.main()
