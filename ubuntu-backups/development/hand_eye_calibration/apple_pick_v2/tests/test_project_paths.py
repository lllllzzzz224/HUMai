import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

from project_paths import (
    APPLE_PICK_DIR,
    REPO_ROOT,
    RESULTS_DIR,
    default_model_path,
    default_yolo_python,
)


class ProjectPathsTest(unittest.TestCase):
    def test_repo_paths_are_derived_from_module_location(self):
        self.assertEqual(APPLE_PICK_DIR.name, "apple_pick_v2")
        self.assertEqual(REPO_ROOT, APPLE_PICK_DIR.parent)
        self.assertEqual(RESULTS_DIR, APPLE_PICK_DIR / "results")

    def test_model_environment_override_wins(self):
        with patch.dict(os.environ, {"APPLE_YOLO_MODEL": "/tmp/apple.pt"}):
            self.assertEqual(default_model_path(), Path("/tmp/apple.pt"))

    def test_python_environment_override_wins(self):
        with patch.dict(os.environ, {"APPLE_YOLO_PYTHON": "/tmp/yolo-python"}):
            self.assertEqual(default_yolo_python(), Path("/tmp/yolo-python"))

    def test_launch_file_loads_without_package_directory_on_pythonpath(self):
        root = Path(__file__).resolve().parents[2]
        launch_file = root / "apple_pick_v2" / "apple_hand_eye_all.launch.py"
        code = (
            "import importlib.util; "
            f"s=importlib.util.spec_from_file_location('portable_launch', {str(launch_file)!r}); "
            "m=importlib.util.module_from_spec(s); s.loader.exec_module(m)"
        )
        environment = os.environ.copy()
        environment["PYTHONPATH"] = os.pathsep.join(
            item
            for item in environment.get("PYTHONPATH", "").split(os.pathsep)
            if item and Path(item).resolve() != launch_file.parent.resolve()
        )
        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=root,
            env=environment,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
