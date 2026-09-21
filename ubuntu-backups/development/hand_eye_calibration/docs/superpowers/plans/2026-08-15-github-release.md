# Apple Catch GitHub Release Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish only the validated RM65 apple-grasp implementation and successful design notes to `lllllzzzz224/Apple-catch`, with a clone-portable README and resource paths.

**Architecture:** Keep the validated motion, vision correction, TCP, gripper, and continuous-loop behavior unchanged. Make only project-resource locations configurable/relative, curate the tracked release tree, and verify the exact commit before a normal non-force push to the empty GitHub repository.

**Tech Stack:** Python 3.12, ROS 2 Jazzy, MoveIt 2, RealSense ROS, Ultralytics YOLO, NumPy, OpenCV, PyYAML, Git/GitHub.

## Global Constraints

- Do not start ROS nodes, move the physical arm, or command the gripper during release preparation.
- Do not change validated numeric motion, TCP, visual XY correction, collision, gripper, or continuous-mode parameters.
- Do not add any currently untracked legacy scripts, third-party repositories, calibration captures, caches, model weights, or runtime output.
- Keep `continuous-grasp-baseline-20260811` and `continuous-grasp-validated-20260811` tags.
- Use a normal push; never force-push.
- Treat hand-eye, TCP, SCAN, visual correction, and gripper values as current-hardware examples requiring local validation.

---

### Task 1: Isolate and inventory the release tree

**Files:**
- Inspect: `.gitignore`
- Inspect: all files returned by `git ls-files`
- Create worktree: `.worktrees/github-release`

**Interfaces:**
- Consumes: committed `master` at the approved release-design commit.
- Produces: isolated `github-release` branch containing no untracked user files.

- [ ] **Step 1: Create an isolated worktree from `master`**

```bash
git worktree add .worktrees/github-release -b github-release master
```

- [ ] **Step 2: Record the exact tracked inventory and sizes**

```bash
git ls-files -z | xargs -0 du -h | sort -h
git status --short --branch
```

Expected: only committed files are present and the worktree is clean.

- [ ] **Step 3: Scan the committed tree before modification**

```bash
rg -n '/home/li|ghp_|github_pat_|BEGIN .*PRIVATE KEY|password|token|192\.168\.|10\.[0-9]+\.' -- $(git ls-files)
find . -type f -size +20M -not -path './.git/*' -print
```

Expected: hardware paths may be reported; no credential/private-key hit and no file above 20 MiB.

---

### Task 2: Make repository resource paths clone-portable

**Files:**
- Create: `apple_pick_v2/project_paths.py`
- Create: `apple_pick_v2/tests/test_project_paths.py`
- Modify: `apple_pick_v2/apple_center_localizer_v2.py`
- Modify: `apple_pick_v2/apple_hand_eye_all.launch.py`
- Modify: `apple_pick_v2/hand_eye_consistency_test.py`
- Modify: `apple_pick_v2/measure_visual_tcp_offset.py`

**Interfaces:**
- Produces: `APPLE_PICK_DIR`, `REPO_ROOT`, `RESULTS_DIR`, `default_model_path()` and `resolve_executable()` helpers.
- Consumes: optional `APPLE_YOLO_MODEL` and `APPLE_YOLO_PYTHON` environment values.

- [ ] **Step 1: Write failing path tests**

```python
import os
from pathlib import Path
import unittest
from unittest.mock import patch

from project_paths import APPLE_PICK_DIR, REPO_ROOT, RESULTS_DIR, default_model_path


class ProjectPathsTest(unittest.TestCase):
    def test_repo_paths_are_derived_from_module_location(self):
        self.assertEqual(APPLE_PICK_DIR.name, "apple_pick_v2")
        self.assertEqual(REPO_ROOT, APPLE_PICK_DIR.parent)
        self.assertEqual(RESULTS_DIR, APPLE_PICK_DIR / "results")

    def test_model_environment_override_wins(self):
        with patch.dict(os.environ, {"APPLE_YOLO_MODEL": "/tmp/apple.pt"}):
            self.assertEqual(default_model_path(), Path("/tmp/apple.pt"))
```

- [ ] **Step 2: Run the new tests and confirm failure**

```bash
python -m unittest apple_pick_v2/tests/test_project_paths.py -v
```

Expected: FAIL because `project_paths` does not exist.

- [ ] **Step 3: Implement the minimal path module**

```python
from pathlib import Path
import os
import sys

APPLE_PICK_DIR = Path(__file__).resolve().parent
REPO_ROOT = APPLE_PICK_DIR.parent
RESULTS_DIR = APPLE_PICK_DIR / "results"


def default_model_path() -> Path:
    value = os.environ.get("APPLE_YOLO_MODEL")
    return Path(value).expanduser() if value else REPO_ROOT / "models" / "yolo11n-seg.pt"


def default_yolo_python() -> Path:
    value = os.environ.get("APPLE_YOLO_PYTHON")
    return Path(value).expanduser() if value else Path(sys.executable)
```

- [ ] **Step 4: Route all four scripts through relative paths**

`apple_center_localizer_v2.py` accepts `--model` and `--vision-debug-root`; `AppleCenterLocalizer` receives those paths rather than reading module-level `/home/li` constants. The launch file derives its own directory and passes launch arguments `yolo_python` and `yolo_model` into the vision process. Both measurement scripts use `RESULTS_DIR`.

- [ ] **Step 5: Run path and existing unit tests**

```bash
python -m unittest discover -s apple_pick_v2/tests -v
```

Expected: all tests PASS and no `/home/li` remains in executable Python or launch files.

- [ ] **Step 6: Commit portable resource lookup**

```bash
git add apple_pick_v2/project_paths.py apple_pick_v2/tests/test_project_paths.py \
  apple_pick_v2/apple_center_localizer_v2.py apple_pick_v2/apple_hand_eye_all.launch.py \
  apple_pick_v2/hand_eye_consistency_test.py apple_pick_v2/measure_visual_tcp_offset.py
git commit -m "fix: make apple catch resource paths portable"
```

---

### Task 3: Curate the successful release content

**Files:**
- Modify: `.gitignore`
- Delete from current release tree: tracked `apple_pick_v2/results/*.json`
- Delete from current release tree: obsolete `apple_pick_v2/8.7.md`, `apple_pick_v2/START_HERE.md`, `apple_pick_v2/运行入口.md`
- Create: `requirements-vision.txt`

**Interfaces:**
- Produces: a repository containing validated source/tests/docs without generated logs or contradictory pre-success entrypoints.

- [ ] **Step 1: Add a release-content regression test**

Create `apple_pick_v2/tests/test_release_tree.py`:

```python
from pathlib import Path
import subprocess
import unittest


class ReleaseTreeTest(unittest.TestCase):
    def test_no_runtime_result_is_tracked(self):
        root = Path(__file__).resolve().parents[2]
        tracked = subprocess.check_output(["git", "ls-files"], cwd=root, text=True)
        self.assertNotIn("apple_pick_v2/results/", tracked)

    def test_obsolete_entrypoints_are_not_tracked(self):
        root = Path(__file__).resolve().parents[2]
        tracked = set(subprocess.check_output(["git", "ls-files"], cwd=root, text=True).splitlines())
        self.assertTrue({"apple_pick_v2/8.7.md", "apple_pick_v2/START_HERE.md", "apple_pick_v2/运行入口.md"}.isdisjoint(tracked))
```

- [ ] **Step 2: Run the test and confirm failure**

```bash
python -m unittest apple_pick_v2/tests/test_release_tree.py -v
```

Expected: FAIL because old logs and entrypoint documents are tracked.

- [ ] **Step 3: Remove only tracked generated/obsolete files and simplify ignore rules**

Keep `apple_pick_v2/results/` ignored without success-log exceptions. Do not delete or add files outside the isolated worktree.

- [ ] **Step 4: Add vision-only Python dependencies**

```text
numpy
opencv-python
PyYAML
ultralytics
```

README will explain that `rclpy`, MoveIt messages, TF2 and sensor messages come from ROS 2 apt/workspace packages, not pip.

- [ ] **Step 5: Run the release-content test and commit**

```bash
python -m unittest apple_pick_v2/tests/test_release_tree.py -v
git add .gitignore requirements-vision.txt apple_pick_v2/tests/test_release_tree.py
git add -u apple_pick_v2/results apple_pick_v2/8.7.md apple_pick_v2/START_HERE.md apple_pick_v2/运行入口.md
git commit -m "chore: curate validated apple catch release"
```

---

### Task 4: Write the clone-to-run README and refresh success docs

**Files:**
- Create/replace: `README.md`
- Modify: `apple_pick_v2/连续苹果抓取README.md`
- Modify: `apple_pick_v2/success apple catch.md`
- Modify: `apple_pick_v2/success 8.10.md`
- Modify: `apple_pick_v2/苹果抓取参数配置说明.md`

**Interfaces:**
- Produces: one authoritative root entrypoint and portable relative links to successful design records.

- [ ] **Step 1: Write README sections required by the approved design**

Use clone-portable commands:

```bash
git clone https://github.com/lllllzzzz224/Apple-catch.git
cd Apple-catch
export APPLE_CATCH_ROOT="$PWD"
export ROS_WS=/absolute/path/to/your/ros2_ws
source /opt/ros/jazzy/setup.bash
source "$ROS_WS/install/setup.bash"
```

Document `APPLE_YOLO_PYTHON` and `APPLE_YOLO_MODEL`, the current-hardware parameters, unit tests, duplicate-node checks, launch command, single-round command, and guarded continuous command.

- [ ] **Step 2: Replace `/home/li` commands in retained success docs**

Use `$APPLE_CATCH_ROOT` for repository paths and `$ROS_WS` for the ROS overlay. Historical numeric log values remain as evidence, but absolute log paths become repository-relative references or are described as local runtime output.

- [ ] **Step 3: Check all Markdown links and forbidden absolute paths**

```bash
python - <<'PY'
from pathlib import Path
import re
root = Path('.')
for md in root.rglob('*.md'):
    text = md.read_text(encoding='utf-8')
    for target in re.findall(r'\[[^]]+\]\(([^)]+)\)', text):
        if '://' not in target and not (md.parent / target.split('#', 1)[0]).resolve().exists():
            raise SystemExit(f'broken link: {md}: {target}')
PY
rg -n '/home/li' README.md apple_pick_v2/*.md
```

Expected: link check PASS; `/home/li` has no active startup command in retained docs.

- [ ] **Step 4: Commit public documentation**

```bash
git add README.md apple_pick_v2/*.md
git commit -m "docs: add reusable Apple Catch setup guide"
```

---

### Task 5: Verify the exact release commit

**Files:**
- Verify only: tracked release tree.

**Interfaces:**
- Produces: test and security evidence for the exact commit to push.

- [ ] **Step 1: Run all unit tests**

```bash
python -m unittest discover -s apple_pick_v2/tests -v
```

Expected: all tests PASS.

- [ ] **Step 2: Compile executable Python files**

```bash
python -m py_compile apple_pick_v2/*.py hand_eye_static_tf.launch.py modbus_gripper_ros.py
```

Expected: exit 0.

- [ ] **Step 3: Parse YAML and launch Python syntax without starting ROS**

```bash
python - <<'PY'
from pathlib import Path
import yaml
for path in Path('.').glob('**/*.yaml'):
    yaml.safe_load(path.read_text(encoding='utf-8'))
print('YAML PASS')
PY
```

- [ ] **Step 4: Run credential, absolute-path and size scans**

```bash
rg -n 'ghp_|github_pat_|BEGIN .*PRIVATE KEY|password|token' -- $(git ls-files)
rg -n '/home/li' -- $(git ls-files '*.py' '*.launch.py' '*.md')
find . -type f -size +20M -not -path './.git/*' -print
```

Expected: no credentials, no active `/home/li` code/docs path, no file above 20 MiB.

- [ ] **Step 5: Verify clean index and review diff from validated tag**

```bash
git status --short --branch
git diff --stat continuous-grasp-validated-20260811..HEAD
git log --oneline continuous-grasp-validated-20260811..HEAD
```

Expected: clean branch with only release-focused commits.

---

### Task 6: Integrate and publish to GitHub

**Files:**
- Git refs only; no source change.

**Interfaces:**
- Consumes: verified `github-release` branch.
- Produces: GitHub `master` and preserved validated tags.

- [ ] **Step 1: Merge the verified release branch into local `master`**

```bash
git checkout master
git merge --ff-only github-release
python -m unittest discover -s apple_pick_v2/tests -v
```

Expected: fast-forward merge and all tests PASS on `master`.

- [ ] **Step 2: Add and verify the target remote**

```bash
git remote add origin https://github.com/lllllzzzz224/Apple-catch.git
git remote -v
git ls-remote --symref origin HEAD
```

Expected: `origin` is the approved empty repository.

- [ ] **Step 3: Push normally**

```bash
git push -u origin master
git push origin continuous-grasp-baseline-20260811 continuous-grasp-validated-20260811
```

Expected: pushes succeed without force.

- [ ] **Step 4: Verify remote refs and GitHub README**

```bash
git ls-remote --heads --tags origin
```

Expected: remote `refs/heads/master` equals local `master`; both validated tags exist.

- [ ] **Step 5: Remove only the owned temporary worktree and branch**

```bash
git worktree remove .worktrees/github-release
git worktree prune
git branch -d github-release
```

Expected: main worktree remains intact, including all user-owned untracked files.
