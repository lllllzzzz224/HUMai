import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from obstacle_preview import run_preview, normalize_record


class PreviewTests(unittest.TestCase):
    def fixture(self, root):
        np.save(root/'depth_00.npy', np.full((3, 3), 1000, np.uint16))
        data = {'records': [dict(color_timestamp_ms=100000., depth_timestamp_ms=100000.,
                               depth_scale_m=.001, intrinsics=dict(fx=100., fy=100., ppx=1., ppy=1., width=3, height=3))]}
        (root/'capture.json').write_text(json.dumps(data))
        return data

    def test_camera_only_capture_cannot_claim_base_scene(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); self.fixture(root)
            s = run_preview(root/'capture.json', root/'preview', {})
            self.assertEqual(s['frame_id'], 'camera_color_optical_frame')
            self.assertIn('missing_camera_to_base', s['quality']['limitations'])
            self.assertFalse(json.loads((root/'preview/scene.json').read_text())['executable'])
            self.assertTrue((root/'preview/observation.ply').exists())

    def test_refuses_overwrite_or_missing_depth(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); self.fixture(root)
            with self.assertRaises(ValueError):
                run_preview(root/'capture.json', root, {})
            (root/'depth_00.npy').unlink()
            with self.assertRaises(FileNotFoundError):
                run_preview(root/'capture.json', root/'preview', {})

    def test_matrix_without_synchronized_provenance_is_not_accepted(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); data = self.fixture(root)
            data['records'][0]['camera_to_base'] = np.eye(4).tolist()
            (root/'capture.json').write_text(json.dumps(data))
            with self.assertRaises((ValueError, KeyError)):
                run_preview(root/'capture.json', root/'preview', {})

    def test_capture_error_or_nonempty_invalid_records_are_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td); data = self.fixture(root)
            data['error'] = 'camera failed'
            (root/'capture.json').write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                run_preview(root/'capture.json', root/'preview', {})


if __name__ == '__main__':
    unittest.main()
