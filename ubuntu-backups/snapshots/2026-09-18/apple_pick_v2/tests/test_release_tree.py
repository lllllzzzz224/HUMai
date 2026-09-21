from pathlib import Path
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[2]


def tracked_files() -> set[str]:
    output = subprocess.check_output(
        ["git", "ls-files"],
        cwd=ROOT,
        text=True,
    )
    return set(output.splitlines())


class ReleaseTreeTest(unittest.TestCase):
    def test_no_runtime_result_is_tracked(self):
        self.assertFalse(
            any(path.startswith("apple_pick_v2/results/") for path in tracked_files())
        )

    def test_obsolete_entrypoints_are_not_tracked(self):
        obsolete = {
            "apple_pick_v2/8.7.md",
            "apple_pick_v2/README.md",
            "apple_pick_v2/START_HERE.md",
            "apple_pick_v2/运行入口.md",
        }
        self.assertTrue(obsolete.isdisjoint(tracked_files()))


if __name__ == "__main__":
    unittest.main()
