import unittest

from fixed_scan_real_verify import scan_arrival_tolerance


class ScanArrivalToleranceTest(unittest.TestCase):
    def test_tracking_residual_inside_arrival_margin(self):
        cfg = {
            "scan": {
                "tolerance_rad": 0.015,
                "arrival_tolerance_rad": 0.020,
            }
        }
        measured_max_error = 0.015198988797729296
        self.assertGreater(
            scan_arrival_tolerance(cfg),
            cfg["scan"]["tolerance_rad"],
        )
        self.assertLessEqual(
            measured_max_error,
            scan_arrival_tolerance(cfg),
        )


if __name__ == "__main__":
    unittest.main()
