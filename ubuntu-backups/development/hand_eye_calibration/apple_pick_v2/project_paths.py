"""Clone-portable paths for the Apple Catch scripts."""

from __future__ import annotations

import os
from pathlib import Path
import sys


APPLE_PICK_DIR = Path(__file__).resolve().parent
REPO_ROOT = APPLE_PICK_DIR.parent
RESULTS_DIR = APPLE_PICK_DIR / "results"


def _environment_path(name: str) -> Path | None:
    value = os.environ.get(name)
    return Path(value).expanduser() if value else None


def default_model_path() -> Path:
    return _environment_path("APPLE_YOLO_MODEL") or REPO_ROOT / "models" / "yolo11n-seg.pt"


def default_yolo_python() -> Path:
    override = _environment_path("APPLE_YOLO_PYTHON")
    if override is not None:
        return override
    candidates = (
        REPO_ROOT / ".venv" / "bin" / "python",
        Path.home() / "anaconda3" / "envs" / "yolo11" / "bin" / "python",
        Path(sys.executable),
    )
    return next((path for path in candidates if path.is_file()), Path(sys.executable))
