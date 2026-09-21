# Continuous Single-Apple Grasp Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an explicitly authorized, persistent real-robot mode that repeatedly grasps one visible apple, returns it to its own source position, waits for the operator to move it, and automatically starts the next round only after the new position is stable.

**Architecture:** Preserve the validated MoveIt and RS485 sequence by mechanically extracting it into prepare/execute round functions. Add a ROS-independent stability/state module and a thin continuous coordinator around the same long-lived ROS node; the old single-round CLI remains the default and continuous behavior requires `--continuous --execute --operator-confirmed`.

**Tech Stack:** Python 3.12, ROS 2 Jazzy `rclpy`, MoveIt 2 services/actions, NumPy, PyYAML, `unittest`, RealSense/YOLO PoseStamped diagnostics, RealMan RS485 gripper.

## Global Constraints

- The camera view contains exactly one apple.
- Each successful round must execute SCAN → PREGRASP → VERIFY → GRASP → LIFT → original-position RELEASE → PREGRASP → SCAN.
- Keep the current TCP, visual XY correction, target offsets, candidate yaw/pitch search, collision checks, Cartesian limits, and gripper parameters unchanged.
- Existing single-round options and behavior remain the default; YAML alone must never enable continuous real motion.
- Continuous motion requires the exact explicit gate `--continuous --execute --operator-confirmed`.
- The first round requires stability only; later rounds require at least `0.030 m` displacement from the previous release center plus stability.
- Defaults are `stable_time_s=2.0`, `safety_countdown_s=3.0`, `retry_delay_s=2.0`, and `max_pre_motion_retries=3`.
- `max_rounds=0` means unlimited; a positive value stops after that many successful pick-return rounds.
- Before the first trajectory of a round, failures may retry at SCAN. After the first trajectory starts, any failure latches the fault and stops the loop without opening the gripper or issuing recovery motion.
- Do not change YOLO thresholds, image processing, hand-eye transforms, TCP offsets, scene geometry, motion speeds, or grasp height as part of this feature.
- Do not stage or commit unrelated files from the existing dirty worktree.
- Real-robot validation remains low speed, one apple only, clear workspace, and physical emergency stop available.

---

## File Map

- Create `apple_pick_v2/continuous_grasp_state.py`: ROS-independent configuration, stability gate, and session state transitions.
- Create `apple_pick_v2/grasp_cli.py`: parser construction and run-mode validation so compatibility can be tested without ROS.
- Create `apple_pick_v2/tests/test_continuous_grasp_state.py`: deterministic clock/position tests.
- Create `apple_pick_v2/tests/test_grasp_cli.py`: old-mode compatibility and continuous authorization tests.
- Create `apple_pick_v2/tests/test_round_phase_boundaries.py`: regression tests for pre-motion versus post-motion failure classification.
- Modify `apple_pick_v2/single_apple_full_grasp.py`: mechanical prepare/execute extraction and persistent coordinator integration.
- Modify `apple_pick_v2/single_apple_full_grasp.yaml`: add only the `continuous` mapping.
- Modify `apple_pick_v2/success apple catch.md`: add continuous startup and rollback instructions.
- Modify `apple_pick_v2/success 8.10.md`: append the validated continuous test result only after real testing.
- Modify `apple_pick_v2/苹果抓取参数配置说明.md`: document continuous parameters and CLI precedence.

---

### Task 1: Preserve the Validated Pre-Feature Baseline

**Files:**
- Review and snapshot: `apple_pick_v2/apple_center_localizer_v2.py`
- Review and snapshot: `apple_pick_v2/fixed_scan_real_verify.py`
- Review and snapshot: `apple_pick_v2/single_apple_full_grasp.py`
- Review and snapshot: `apple_pick_v2/single_apple_full_grasp.yaml`
- Review and snapshot: `apple_pick_v2/success apple catch.md`
- Review and snapshot: `apple_pick_v2/success 8.10.md`
- Review and snapshot: `apple_pick_v2/苹果抓取参数配置说明.md`

**Interfaces:**
- Consumes: current dirty working tree and commits `a66388f`, `ddb11c8`, `774ac38`, `f7fe6e4`.
- Produces: one reviewed baseline commit and annotated tag `continuous-grasp-baseline-20260811`.

- [ ] **Step 1: Review only the feature-relevant differences**

```bash
git diff -- apple_pick_v2/apple_center_localizer_v2.py \
  apple_pick_v2/fixed_scan_real_verify.py \
  apple_pick_v2/single_apple_full_grasp.py \
  apple_pick_v2/single_apple_full_grasp.yaml \
  "apple_pick_v2/success apple catch.md"
```

Expected: changes correspond to the already field-tested vision fallback, candidate/path fixes, visual XY correction, current grasp parameters, and documentation. Stop if unrelated edits appear.

- [ ] **Step 2: Verify syntax, YAML, and whitespace before snapshotting**

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
$APPLE_YOLO_PYTHON -m py_compile \
  apple_pick_v2/apple_center_localizer_v2.py \
  apple_pick_v2/fixed_scan_real_verify.py \
  apple_pick_v2/single_apple_full_grasp.py
$APPLE_YOLO_PYTHON -c \
  "import pathlib,yaml; yaml.safe_load(pathlib.Path('apple_pick_v2/single_apple_full_grasp.yaml').read_text())"
git diff --check
```

Expected: all commands exit `0`; no robot command is sent.

- [ ] **Step 3: Stage only reviewed baseline files**

```bash
git add apple_pick_v2/apple_center_localizer_v2.py \
  apple_pick_v2/fixed_scan_real_verify.py \
  apple_pick_v2/single_apple_full_grasp.py \
  apple_pick_v2/single_apple_full_grasp.yaml \
  "apple_pick_v2/success apple catch.md" \
  "apple_pick_v2/success 8.10.md" \
  "apple_pick_v2/苹果抓取参数配置说明.md"
```

Expected: all seven paths are explicitly staged after their content review; no other untracked file is staged.

- [ ] **Step 4: Confirm the staged boundary**

```bash
git diff --cached --check
git diff --cached --name-only
git status --short
```

Expected: the cached list contains only reviewed `apple_pick_v2` baseline files; unrelated root-level files remain unstaged.

- [ ] **Step 5: Commit and tag the baseline**

```bash
git commit -m "snapshot: preserve validated apple grasp before loop"
git tag -a continuous-grasp-baseline-20260811 -m \
  "Validated single-apple baseline before persistent loop"
git rev-parse HEAD
git rev-parse continuous-grasp-baseline-20260811
```

Expected: the commit and tag resolve to the same object lineage and are copied into the continuous session notes.

---

### Task 2: Build the Pure Stability Gate and Session State Machine

**Files:**
- Create: `apple_pick_v2/continuous_grasp_state.py`
- Create: `apple_pick_v2/tests/__init__.py`
- Create: `apple_pick_v2/tests/test_continuous_grasp_state.py`

**Interfaces:**
- Consumes: corrected `base_link` apple centers as three finite floats and monotonic seconds supplied by the caller.
- Produces: `ContinuousSettings.from_mapping(mapping, cli_max_rounds)`, `AppleStabilityGate.observe(center_m, now_s) -> GateSnapshot`, `AppleStabilityGate.missing()`, `AppleStabilityGate.after_release(center_m)`, `AppleStabilityGate.reject(center_m)`, and `ContinuousSession` state transitions.

- [ ] **Step 1: Write failing configuration and first-round stability tests**

```python
# apple_pick_v2/tests/test_continuous_grasp_state.py
import unittest
import numpy as np

from continuous_grasp_state import (
    AppleStabilityGate,
    ContinuousSettings,
)


class StabilityGateTest(unittest.TestCase):
    def setUp(self):
        self.settings = ContinuousSettings(
            max_rounds=0,
            min_position_change_m=0.030,
            stable_time_s=2.0,
            safety_countdown_s=3.0,
            retry_delay_s=2.0,
            max_pre_motion_retries=3,
            max_axis_span_m=0.010,
        )
        self.gate = AppleStabilityGate(self.settings)
        self.apple = np.asarray([0.40, -0.20, -0.05])

    def test_first_round_requires_stability_and_countdown(self):
        self.assertEqual(self.gate.observe(self.apple, 0.0).state, "STABILITY_CHECK")
        self.assertEqual(self.gate.observe(self.apple, 2.1).state, "SAFETY_COUNTDOWN")
        ready = self.gate.observe(self.apple, 5.1)
        self.assertEqual(ready.state, "READY")
        self.assertTrue(ready.ready)

    def test_cli_max_rounds_overrides_yaml(self):
        settings = ContinuousSettings.from_mapping(
            {
                "max_rounds": 0,
                "min_position_change_m": 0.030,
                "stable_time_s": 2.0,
                "safety_countdown_s": 3.0,
                "retry_delay_s": 2.0,
                "max_pre_motion_retries": 3,
            },
            max_axis_span_m=0.010,
            cli_max_rounds=2,
        )
        self.assertEqual(settings.max_rounds, 2)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the tests and verify the missing module failure**

```bash
PYTHONPATH=apple_pick_v2 $APPLE_YOLO_PYTHON \
  -m unittest apple_pick_v2.tests.test_continuous_grasp_state -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'continuous_grasp_state'`.

- [ ] **Step 3: Implement validated settings and initial stability timing**

```python
# apple_pick_v2/continuous_grasp_state.py
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np


class SessionState(str, Enum):
    START = "START"
    ENSURE_SCAN = "ENSURE_SCAN"
    WAIT_NEW_APPLE = "WAIT_NEW_APPLE"
    STABILITY_CHECK = "STABILITY_CHECK"
    SAFETY_COUNTDOWN = "SAFETY_COUNTDOWN"
    PLAN_ROUND = "PLAN_ROUND"
    EXECUTE_PICK = "EXECUTE_PICK"
    EXECUTE_LIFT = "EXECUTE_LIFT"
    EXECUTE_RETURN = "EXECUTE_RETURN"
    ROUND_COMPLETE = "ROUND_COMPLETE"
    COMPLETE = "COMPLETE"
    STOPPED_BY_OPERATOR = "STOPPED_BY_OPERATOR"
    FAULT_LATCHED = "FAULT_LATCHED"


@dataclass(frozen=True)
class ContinuousSettings:
    max_rounds: int
    min_position_change_m: float
    stable_time_s: float
    safety_countdown_s: float
    retry_delay_s: float
    max_pre_motion_retries: int
    max_axis_span_m: float

    @classmethod
    def from_mapping(cls, mapping, *, max_axis_span_m, cli_max_rounds=None):
        max_rounds = int(mapping["max_rounds"] if cli_max_rounds is None else cli_max_rounds)
        values = cls(
            max_rounds=max_rounds,
            min_position_change_m=float(mapping["min_position_change_m"]),
            stable_time_s=float(mapping["stable_time_s"]),
            safety_countdown_s=float(mapping["safety_countdown_s"]),
            retry_delay_s=float(mapping["retry_delay_s"]),
            max_pre_motion_retries=int(mapping["max_pre_motion_retries"]),
            max_axis_span_m=float(max_axis_span_m),
        )
        if values.max_rounds < 0:
            raise ValueError("max_rounds 必须 >= 0")
        if values.min_position_change_m <= 0.0:
            raise ValueError("min_position_change_m 必须 > 0")
        if values.stable_time_s <= 0.0 or values.safety_countdown_s < 0.0:
            raise ValueError("稳定时间必须 > 0，安全倒计时必须 >= 0")
        if values.retry_delay_s <= 0.0 or values.max_pre_motion_retries < 1:
            raise ValueError("重试间隔必须 > 0，重试次数必须 >= 1")
        if values.max_axis_span_m <= 0.0:
            raise ValueError("max_axis_span_m 必须 > 0")
        return values


@dataclass(frozen=True)
class GateSnapshot:
    state: str
    ready: bool
    stable_elapsed_s: float
    distance_from_reference_m: float | None
    center_m: tuple[float, float, float] | None


class AppleStabilityGate:
    def __init__(self, settings: ContinuousSettings):
        self.settings = settings
        self.last_release_center = None
        self.rejected_center = None
        self._samples = []
        self._stable_since = None

    def _reference(self):
        return self.rejected_center if self.rejected_center is not None else self.last_release_center

    def missing(self):
        self._samples.clear()
        self._stable_since = None

    def after_release(self, center_m):
        self.last_release_center = self._validated_center(center_m)
        self.rejected_center = None
        self.missing()

    def reject(self, center_m):
        self.rejected_center = self._validated_center(center_m)
        self.missing()

    @staticmethod
    def _validated_center(center_m):
        center = np.asarray(center_m, dtype=np.float64)
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            raise ValueError("苹果中心必须是三个有限数值")
        return center

    def observe(self, center_m, now_s):
        center = self._validated_center(center_m)
        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError("now_s 必须是有限数值")
        reference = self._reference()
        distance = None if reference is None else float(np.linalg.norm(center - reference))
        if distance is not None and distance < self.settings.min_position_change_m:
            self.missing()
            return GateSnapshot("WAIT_NEW_APPLE", False, 0.0, distance, tuple(center))
        if self._stable_since is None:
            self._stable_since = now
            self._samples = [center]
        else:
            self._samples.append(center)
            span = np.ptp(np.asarray(self._samples), axis=0)
            if np.any(span > self.settings.max_axis_span_m):
                self._stable_since = now
                self._samples = [center]
        elapsed = max(0.0, now - self._stable_since)
        ready_after = self.settings.stable_time_s + self.settings.safety_countdown_s
        if elapsed >= ready_after:
            state = "READY"
        elif elapsed >= self.settings.stable_time_s:
            state = "SAFETY_COUNTDOWN"
        else:
            state = "STABILITY_CHECK"
        return GateSnapshot(state, state == "READY", elapsed, distance, tuple(center))
```

- [ ] **Step 4: Add failing tests for same-position suppression, movement, jitter, missing data, rejection, and round limit**

Add these methods to `StabilityGateTest` and add `ContinuousSessionTest`:
First extend the existing import block so every referenced type is explicit:

```python
from continuous_grasp_state import AppleStabilityGate, ContinuousSession, ContinuousSettings, SessionState
```


```python
    def test_same_release_position_is_suppressed_until_moved_30_mm(self):
        self.gate.after_release(self.apple)
        self.assertEqual(self.gate.observe(self.apple + [0.020, 0.0, 0.0], 0.0).state, "WAIT_NEW_APPLE")
        self.assertEqual(self.gate.observe(self.apple + [0.031, 0.0, 0.0], 1.0).state, "STABILITY_CHECK")

    def test_jitter_over_axis_limit_restarts_timer(self):
        self.gate.observe(self.apple, 0.0)
        shifted = self.apple + [0.011, 0.0, 0.0]
        self.assertEqual(self.gate.observe(shifted, 1.9).stable_elapsed_s, 0.0)
        self.assertFalse(self.gate.observe(shifted, 5.0).ready)

    def test_missing_data_resets_stability_not_release_reference(self):
        self.gate.after_release(self.apple)
        moved = self.apple + [0.040, 0.0, 0.0]
        self.gate.observe(moved, 0.0)
        self.gate.missing()
        snapshot = self.gate.observe(moved, 4.0)
        self.assertEqual(snapshot.state, "STABILITY_CHECK")

    def test_rejected_target_requires_another_position_change(self):
        rejected = self.apple + [0.040, 0.0, 0.0]
        self.gate.reject(rejected)
        self.assertEqual(self.gate.observe(rejected, 0.0).state, "WAIT_NEW_APPLE")
        self.assertEqual(self.gate.observe(rejected + [0.031, 0.0, 0.0], 1.0).state, "STABILITY_CHECK")


class ContinuousSessionTest(unittest.TestCase):
    def test_round_limit_and_fault_are_terminal(self):
        session = ContinuousSession(max_rounds=2)
        session.round_completed()
        self.assertFalse(session.complete)
        session.round_completed()
        self.assertTrue(session.complete)
        self.assertEqual(session.state, SessionState.COMPLETE)

        faulted = ContinuousSession(max_rounds=0)
        faulted.motion_started()
        faulted.fail(RuntimeError("execute failed"))
        self.assertEqual(faulted.state, SessionState.FAULT_LATCHED)
        self.assertEqual(faulted.stop_reason, "execute failed")
```

- [ ] **Step 5: Run tests and verify `ContinuousSession` is missing**

```bash
PYTHONPATH=apple_pick_v2 $APPLE_YOLO_PYTHON \
  -m unittest apple_pick_v2.tests.test_continuous_grasp_state -v
```

Expected: stability tests PASS and session tests FAIL because `ContinuousSession` is not defined/importable.

- [ ] **Step 6: Implement the session state transitions**

Append to `continuous_grasp_state.py`:

```python
class ContinuousSession:
    def __init__(self, *, max_rounds: int):
        if max_rounds < 0:
            raise ValueError("max_rounds 必须 >= 0")
        self.max_rounds = max_rounds
        self.completed_rounds = 0
        self.state = SessionState.START
        self.motion_has_started = False
        self.stop_reason = None

    @property
    def complete(self):
        return self.state in {
            SessionState.COMPLETE,
            SessionState.STOPPED_BY_OPERATOR,
            SessionState.FAULT_LATCHED,
        }

    def set_state(self, state: SessionState):
        if self.complete:
            raise RuntimeError(f"终止状态 {self.state.value} 不允许继续转换")
        self.state = state

    def motion_started(self):
        self.motion_has_started = True
        self.state = SessionState.EXECUTE_PICK

    def round_completed(self):
        self.completed_rounds += 1
        self.motion_has_started = False
        if self.max_rounds > 0 and self.completed_rounds >= self.max_rounds:
            self.state = SessionState.COMPLETE
        else:
            self.state = SessionState.WAIT_NEW_APPLE

    def fail(self, error: Exception):
        self.stop_reason = str(error)
        self.state = (
            SessionState.FAULT_LATCHED
            if self.motion_has_started
            else SessionState.WAIT_NEW_APPLE
        )

    def stop(self):
        self.stop_reason = "Ctrl+C"
        self.state = SessionState.STOPPED_BY_OPERATOR
```

- [ ] **Step 7: Run the pure test suite**

```bash
PYTHONPATH=apple_pick_v2 $APPLE_YOLO_PYTHON \
  -m unittest apple_pick_v2.tests.test_continuous_grasp_state -v
```

Expected: all tests PASS.

- [ ] **Step 8: Commit the pure state machine**

```bash
git add apple_pick_v2/continuous_grasp_state.py \
  apple_pick_v2/tests/__init__.py \
  apple_pick_v2/tests/test_continuous_grasp_state.py
git commit -m "feat: add tested continuous grasp state machine"
```

---

### Task 3: Add a Backward-Compatible CLI and YAML Resolver

**Files:**
- Create: `apple_pick_v2/grasp_cli.py`
- Create: `apple_pick_v2/tests/test_grasp_cli.py`
- Modify: `apple_pick_v2/single_apple_full_grasp.yaml`
- Modify: `apple_pick_v2/single_apple_full_grasp.py:381-426`

**Interfaces:**
- Consumes: existing single-round flags, `continuous` YAML mapping, optional CLI `--max-rounds`.
- Produces: `RunMode`, `build_parser(default_config)`, `parse_cli(argv, default_config)`, and `resolve_run_mode(args, cfg)`.

- [ ] **Step 1: Write failing tests that pin old semantics and new authorization**

```python
# apple_pick_v2/tests/test_grasp_cli.py
import unittest
from pathlib import Path

from grasp_cli import parse_cli, resolve_run_mode


CFG = {
    "continuous": {
        "max_rounds": 0,
        "min_position_change_m": 0.030,
        "stable_time_s": 2.0,
        "safety_countdown_s": 3.0,
        "retry_delay_s": 2.0,
        "max_pre_motion_retries": 3,
    },
    "vision": {"max_axis_span_m": 0.010},
}


class GraspCliTest(unittest.TestCase):
    def parse(self, words):
        args, ros_args = parse_cli(words, Path("config.yaml"))
        self.assertEqual(ros_args, [])
        return args

    def test_old_fully_autonomous_remains_single_round(self):
        mode = resolve_run_mode(
            self.parse(["--execute", "--operator-confirmed", "--fully-autonomous"]),
            CFG,
        )
        self.assertFalse(mode.continuous)
        self.assertTrue(mode.auto_grasp)
        self.assertTrue(mode.auto_return)
        self.assertEqual(mode.max_rounds, 1)

    def test_continuous_requires_both_real_robot_gates(self):
        for words in (["--continuous"], ["--continuous", "--execute"]):
            with self.subTest(words=words), self.assertRaisesRegex(RuntimeError, "--execute --operator-confirmed"):
                resolve_run_mode(self.parse(words), CFG)

    def test_continuous_rejects_old_mode_flags(self):
        words = ["--continuous", "--execute", "--operator-confirmed", "--fully-autonomous"]
        with self.assertRaisesRegex(RuntimeError, "不能组合"):
            resolve_run_mode(self.parse(words), CFG)

    def test_cli_round_limit_overrides_yaml(self):
        words = ["--continuous", "--execute", "--operator-confirmed", "--max-rounds", "2"]
        mode = resolve_run_mode(self.parse(words), CFG)
        self.assertEqual(mode.max_rounds, 2)
        self.assertTrue(mode.auto_grasp)
        self.assertTrue(mode.auto_return)
```

- [ ] **Step 2: Run the test and verify the missing module failure**

```bash
PYTHONPATH=apple_pick_v2 $APPLE_YOLO_PYTHON \
  -m unittest apple_pick_v2.tests.test_grasp_cli -v
```

Expected: FAIL with `ModuleNotFoundError: No module named 'grasp_cli'`.

- [ ] **Step 3: Implement the parser and exact mode resolver**

Create `grasp_cli.py` with the existing help text copied verbatim and these new interfaces:

```python
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

from continuous_grasp_state import ContinuousSettings


@dataclass(frozen=True)
class RunMode:
    continuous: bool
    execute: bool
    operator_confirmed: bool
    verify_only: bool
    preflight_only: bool
    auto_grasp: bool
    auto_return: bool
    max_rounds: int
    settings: ContinuousSettings | None


def build_parser(default_config: Path):
    parser = argparse.ArgumentParser(description="固定鸟瞰位单苹果完整低速抓取")
    parser.add_argument("--config", type=Path, default=default_config)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--operator-confirmed", action="store_true")
    parser.add_argument("--verify-only", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--autonomous", action="store_true")
    parser.add_argument("--fully-autonomous", action="store_true")
    parser.add_argument("--return-after-lift", action="store_true")
    parser.add_argument("--disable-visual-xy-correction", action="store_true")
    parser.add_argument("--continuous", action="store_true")
    parser.add_argument("--max-rounds", type=int, default=None)
    return parser


def parse_cli(argv, default_config):
    return build_parser(default_config).parse_known_args(argv)


def resolve_run_mode(args, cfg):
    if args.continuous:
        old_modes = (
            args.verify_only,
            args.preflight_only,
            args.autonomous,
            args.fully_autonomous,
            args.return_after_lift,
        )
        if any(old_modes):
            raise RuntimeError("--continuous 不能组合旧单轮模式参数")
        if not args.execute or not args.operator_confirmed:
            raise RuntimeError("连续真机模式必须同时提供 --execute --operator-confirmed")
        settings = ContinuousSettings.from_mapping(
            cfg["continuous"],
            max_axis_span_m=cfg["vision"]["max_axis_span_m"],
            cli_max_rounds=args.max_rounds,
        )
        return RunMode(True, True, True, False, False, True, True, settings.max_rounds, settings)

    if args.max_rounds is not None:
        raise RuntimeError("--max-rounds 只能与 --continuous 一起使用")
    auto_grasp = bool(args.autonomous or args.fully_autonomous)
    auto_return = bool(args.return_after_lift or args.fully_autonomous)
    if args.verify_only and (args.preflight_only or auto_grasp or auto_return):
        raise RuntimeError("--verify-only 不能与预检/自动抓取/放回参数组合使用")
    if args.preflight_only:
        if args.execute:
            raise RuntimeError("--preflight-only 与 --execute 不能同时使用")
        if args.fully_autonomous:
            raise RuntimeError("--preflight-only 与 --fully-autonomous 不能同时使用")
        if not args.operator_confirmed:
            raise RuntimeError("实时单苹果预检仍需提供 --operator-confirmed")
    elif not args.execute or not args.operator_confirmed:
        raise RuntimeError("真机单轮抓取必须同时提供 --execute --operator-confirmed")
    return RunMode(False, bool(args.execute), bool(args.operator_confirmed), bool(args.verify_only), bool(args.preflight_only), auto_grasp, auto_return, 1, None)
```

Before committing, restore every old option's original help string from `single_apple_full_grasp.py`; only the new flag descriptions may be new.

- [ ] **Step 4: Add the YAML mapping without modifying any existing value**

Append to `single_apple_full_grasp.yaml`:

```yaml
continuous:
  max_rounds: 0
  min_position_change_m: 0.030
  stable_time_s: 2.0
  safety_countdown_s: 3.0
  retry_delay_s: 2.0
  max_pre_motion_retries: 3
```

- [ ] **Step 5: Make the existing script delegate only parsing and validation**

Replace the inline parser construction at `single_apple_full_grasp.py:381-426` with imports and calls while leaving all post-validation motion logic untouched:

```python
from grasp_cli import parse_cli, resolve_run_mode


def main(argv=None) -> int:
    args, ros_args = parse_cli(argv, DEFAULT_CONFIG)
    cfg = load_config(args.config)
    mode = resolve_run_mode(args, cfg)
    if args.fully_autonomous:
        args.autonomous = True
        args.return_after_lift = True
```

Keep the existing `args` fields in place during this task so the remaining single-round code executes exactly as before. Do not add the continuous loop yet.

- [ ] **Step 6: Run CLI, config, and syntax regression**

```bash
PYTHONPATH=apple_pick_v2 $APPLE_YOLO_PYTHON \
  -m unittest apple_pick_v2.tests.test_grasp_cli -v
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
$APPLE_YOLO_PYTHON -m py_compile \
  apple_pick_v2/grasp_cli.py \
  apple_pick_v2/single_apple_full_grasp.py
$APPLE_YOLO_PYTHON \
  apple_pick_v2/single_apple_full_grasp.py --help
```

Expected: tests PASS, compilation exits `0`, and help lists every old flag plus `--continuous` and `--max-rounds` without creating a ROS node or moving the robot.

- [ ] **Step 7: Commit the compatibility gate**

```bash
git add apple_pick_v2/grasp_cli.py \
  apple_pick_v2/tests/test_grasp_cli.py \
  apple_pick_v2/single_apple_full_grasp.py \
  apple_pick_v2/single_apple_full_grasp.yaml
git commit -m "feat: add explicit continuous grasp authorization"
```

---

### Task 4: Extract the Validated Single Round into Prepare and Execute Phases

**Files:**
- Modify: `apple_pick_v2/single_apple_full_grasp.py:381-1083`
- Create: `apple_pick_v2/tests/test_round_phase_boundaries.py`

**Interfaces:**
- Consumes: one long-lived `SingleAppleFullGrasp`, `RunMode`, config mapping, and an optional frozen `AppleTarget`.
- Produces: `AppleTarget`, `PreparedRound`, `RoundOutcome`, `collect_target`, `prepare_round`, `execute_prepared_round`, `write_round_log`, and a backward-compatible single-round coordinator.

- [ ] **Step 1: Add dataclasses and a failing boundary test**

Insert these public types above `main()`:

```python
from dataclasses import dataclass


@dataclass(frozen=True)
class AppleTarget:
    raw_center_m: np.ndarray
    corrected_center_m: np.ndarray
    radius_m: float
    diagnostics: dict
    correction: dict


@dataclass
class PreparedRound:
    target: AppleTarget
    pre_pose: Pose
    verify_pose: Pose
    grasp_pose: Pose
    lift_pose: Pose
    pre_trajectory: object
    grasp_state: RobotState | None
    record: dict

@dataclass(frozen=True)
class RoundOutcome:
    status: str
    release_center_m: tuple[float, float, float] | None
    motion_started: bool
```

Create the test:

```python
# apple_pick_v2/tests/test_round_phase_boundaries.py
import unittest

from single_apple_full_grasp import mark_motion_started


class RoundPhaseBoundaryTest(unittest.TestCase):
    def test_motion_flag_is_false_until_first_execute(self):
        record = {"motion_started": False}
        self.assertFalse(record["motion_started"])
        mark_motion_started(record)
        self.assertTrue(record["motion_started"])
```

- [ ] **Step 2: Run the boundary test and verify it fails**

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
PYTHONPATH=apple_pick_v2 $APPLE_YOLO_PYTHON \
  -m unittest apple_pick_v2.tests.test_round_phase_boundaries -v
```

Expected: FAIL because `mark_motion_started` is not defined.

- [ ] **Step 3: Add record and target helpers**

Implement these exact boundaries:

```python
def mark_motion_started(record: dict) -> None:
    record["motion_started"] = True


def new_round_record(args, *, round_index: int, continuous: bool) -> dict:
    return {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "status": "STARTED",
        "single_round": not continuous,
        "continuous": continuous,
        "round_index": int(round_index),
        "autonomous_requested": bool(args.autonomous or continuous),
        "fully_autonomous_requested": bool(args.fully_autonomous or continuous),
        "preflight_only": bool(args.preflight_only),
        "verify_only": bool(args.verify_only),
        "return_after_lift_requested": bool(args.return_after_lift or continuous),
        "visual_xy_correction_disabled_by_cli": bool(args.disable_visual_xy_correction),
        "single_apple_scene_operator_confirmed": bool(args.operator_confirmed),
        "motion_started": False,
        "gripper_close_commanded": False,
        "lift_completed": False,
        "gripper_reopened": False,
        "returned_to_scan": False,
    }


def collect_target(node, cfg, *, disabled_by_cli: bool) -> AppleTarget:
    raw, radius, diagnostics = node.collect_apple()
    corrected, correction = apply_visual_xy_correction(
        raw, cfg, disabled_by_cli=disabled_by_cli
    )
    return AppleTarget(
        raw_center_m=np.asarray(raw, dtype=np.float64),
        corrected_center_m=np.asarray(corrected, dtype=np.float64),
        radius_m=float(radius),
        diagnostics=dict(diagnostics),
        correction=dict(correction),
    )


def write_round_log(path: Path, node, record: dict, event_start: int) -> None:
    record["events"] = list(node.events[event_start:])
    record["finished_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    path.write_text(json.dumps(record, indent=2, ensure_ascii=False), encoding="utf-8")
```

- [ ] **Step 4: Mechanically extract preparation without changing planning order or constants**

Create:

```python
def prepare_round(node, cfg, args, mode, target: AppleTarget, record: dict) -> PreparedRound:
    """Build and validate every trajectory; never execute or command the gripper."""
```

Move the current target-reporting and planning statements from lines `523-871` into this function. Apply these exact substitutions only:

- `raw_apple` → `target.raw_center_m`
- `apple` → `target.corrected_center_m`
- `radius` → `target.radius_m`
- `diagnostics` → `target.diagnostics`
- `correction` → `target.correction`
- `args.return_after_lift` → `mode.auto_return`
- `args.verify_only` → `mode.verify_only`

Initialize `grasp_state = None` before candidate planning, assign the existing chained GRASP preflight end state when GRASP is part of the mode, and return `PreparedRound(target, pre_pose, verify_pose, grasp_pose, lift_pose, pre_traj, grasp_state, record)`. Do not remove any current preflight call, collision check, required fraction, velocity scaling, or TCP speed cap. The existing release recheck must use `prepared.grasp_state` exactly where the old code used local `grasp_state`; reject execution if an auto-return mode somehow reaches that point with `grasp_state is None`.

- [ ] **Step 5: Mechanically extract execution and set the phase boundary once**

Create:

```python
def execute_prepared_round(node, cfg, args, mode, prepared: PreparedRound) -> RoundOutcome:
    record = prepared.record
    mark_motion_started(record)
    node.execute(prepared.pre_trajectory, "SCAN -> PREGRASP")
```

Move the current execution statements from lines `903-1065` after that first call. Preserve their order and numeric/config values. Apply only these substitutions:

- `pre_pose`, `verify_pose`, `grasp_pose`, `lift_pose` → corresponding `prepared` fields.
- `args.autonomous` → `mode.auto_grasp`.
- `args.return_after_lift` → `mode.auto_return`.
- `args.fully_autonomous` in the release confirmation branch → `mode.continuous or args.fully_autonomous`.

For a successful original-position release and SCAN return, return:

```python
return RoundOutcome(
    status="PASS_PICK_RETURN_COMPLETE",
    release_center_m=tuple(prepared.target.corrected_center_m.tolist()),
    motion_started=True,
)
```

For successful non-return single modes, return the same existing status and `release_center_m=None`. Do not add recovery motion to exception paths.

- [ ] **Step 6: Rebuild the old single-round coordinator with the extracted functions**

Create `run_single_main(node, cfg, args, mode, record) -> int`. The default path inside that function must remain:

```python
target = collect_target(
    node,
    cfg,
    disabled_by_cli=args.disable_visual_xy_correction,
)
prepared = prepare_round(node, cfg, args, mode, target, record)
if mode.preflight_only:
    record["status"] = "PASS_PREFLIGHT_ONLY"
    return 0
outcome = execute_prepared_round(node, cfg, args, mode, prepared)
record["status"] = outcome.status
return 0
```

Keep initialization, gripper configuration/open command, SCAN arrival, stale support removal, final support removal, JSON writing, node destruction, and `rclpy.shutdown()` in the same lifecycle positions as before.

- [ ] **Step 7: Run automated compatibility checks**

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
PYTHONPATH=apple_pick_v2 $APPLE_YOLO_PYTHON \
  -m unittest discover -s apple_pick_v2/tests -v
$APPLE_YOLO_PYTHON -m py_compile \
  apple_pick_v2/single_apple_full_grasp.py
git diff --check
```

Expected: all tests PASS and compilation exits `0`.

- [ ] **Step 8: Commit the behavior-preserving extraction**

```bash
git add apple_pick_v2/single_apple_full_grasp.py \
  apple_pick_v2/tests/test_round_phase_boundaries.py
git commit -m "refactor: isolate apple grasp prepare and execute phases"
```

---

### Task 5: Integrate the Persistent Continuous Coordinator

**Files:**
- Modify: `apple_pick_v2/single_apple_full_grasp.py`
- Modify: `apple_pick_v2/tests/test_continuous_grasp_state.py`

**Interfaces:**
- Consumes: `ContinuousSettings`, `ContinuousSession`, `AppleStabilityGate`, `collect_target`, `prepare_round`, and `execute_prepared_round`.
- Produces: `wait_for_ready_target`, `continuous_main`, per-round JSON files, and `continuous_apple_grasp_<timestamp>.json` session summary.

- [ ] **Step 1: Write failing coordinator tests for retry and post-motion fault classification**

Add to `test_continuous_grasp_state.py`:

```python
    def test_pre_motion_failure_returns_to_waiting(self):
        session = ContinuousSession(max_rounds=0)
        session.set_state(SessionState.PLAN_ROUND)
        session.fail(RuntimeError("no plan"))
        self.assertEqual(session.state, SessionState.WAIT_NEW_APPLE)

    def test_post_motion_failure_latches(self):
        session = ContinuousSession(max_rounds=0)
        session.motion_started()
        session.fail(RuntimeError("trajectory timeout"))
        self.assertEqual(session.state, SessionState.FAULT_LATCHED)
```

- [ ] **Step 2: Run the tests and verify current transition behavior**

```bash
PYTHONPATH=apple_pick_v2 $APPLE_YOLO_PYTHON \
  -m unittest apple_pick_v2.tests.test_continuous_grasp_state -v
```

Expected: tests PASS if Task 2 implemented the contract; if either fails, fix only the pure transition method before ROS integration.

- [ ] **Step 3: Implement stable target acquisition with no motion side effects**

Add to `single_apple_full_grasp.py`:

```python
def wait_for_ready_target(node, cfg, args, gate, session_record):
    last_state = None
    while rclpy.ok():
        try:
            target = collect_target(
                node,
                cfg,
                disabled_by_cli=args.disable_visual_xy_correction,
            )
        except RuntimeError as error:
            gate.missing()
            session_record["vision_wait_errors"] += 1
            node.get_logger().warning(f"等待稳定苹果：{error}")
            node.settle(float(gate.settings.retry_delay_s))
            continue
        snapshot = gate.observe(target.corrected_center_m, time.monotonic())
        if snapshot.state != last_state:
            node.get_logger().info(
                f"连续抓取门禁：{snapshot.state}，稳定 {snapshot.stable_elapsed_s:.1f}s"
            )
            last_state = snapshot.state
        if snapshot.ready:
            return target, snapshot
    raise KeyboardInterrupt
```

- [ ] **Step 4: Implement final target validation before the first trajectory**

```python
def validate_frozen_target(node, cfg, args, frozen: AppleTarget) -> AppleTarget:
    latest = collect_target(
        node,
        cfg,
        disabled_by_cli=args.disable_visual_xy_correction,
    )
    distance = float(np.linalg.norm(latest.corrected_center_m - frozen.corrected_center_m))
    limit = float(cfg["vision"]["max_axis_span_m"])
    if distance > limit:
        raise RuntimeError(
            f"规划后苹果位置变化 {distance * 1000.0:.1f}mm 超过 {limit * 1000.0:.1f}mm"
        )
    return latest
```

This function is called after all planning succeeds and immediately before `execute_prepared_round`; failure is therefore pre-motion and returns to SCAN waiting.

- [ ] **Step 5: Implement the persistent loop and per-round cleanup**

Add:

```python
def continuous_main(node, cfg, args, mode, result_dir: Path) -> int:
    settings = mode.settings
    session = ContinuousSession(max_rounds=settings.max_rounds)
    gate = AppleStabilityGate(settings)
    started = time.strftime("%Y%m%d_%H%M%S")
    session_path = result_dir / f"continuous_apple_grasp_{started}.json"
    session_record = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "status": "STARTED",
        "settings": settings.__dict__,
        "completed_rounds": 0,
        "pre_motion_rejections": 0,
        "vision_wait_errors": 0,
        "rounds": [],
    }
    try:
        while rclpy.ok() and not session.complete:
            session.set_state(SessionState.WAIT_NEW_APPLE)
            target, snapshot = wait_for_ready_target(node, cfg, args, gate, session_record)
            attempts = 0
            while attempts < settings.max_pre_motion_retries:
                attempts += 1
                event_start = len(node.events)
                round_record = new_round_record(
                    args,
                    round_index=session.completed_rounds + 1,
                    continuous=True,
                )
                round_path = result_dir / (
                    f"single_apple_grasp_{time.strftime('%Y%m%d_%H%M%S')}_"
                    f"round{session.completed_rounds + 1:03d}.json"
                )
                try:
                    session.set_state(SessionState.PLAN_ROUND)
                    prepared = prepare_round(node, cfg, args, mode, target, round_record)
                    validate_frozen_target(node, cfg, args, target)
                    session.motion_started()
                    outcome = execute_prepared_round(node, cfg, args, mode, prepared)
                    round_record["status"] = outcome.status
                    write_round_log(round_path, node, round_record, event_start)
                    session_record["rounds"].append({"log": str(round_path), "status": outcome.status})
                    node.remove_support_guard()
                    gate.after_release(outcome.release_center_m)
                    session.round_completed()
                    session_record["completed_rounds"] = session.completed_rounds
                    break
                except RuntimeError as error:
                    round_record["status"] = "FAILED_STOPPED"
                    round_record["error"] = str(error)
                    write_round_log(round_path, node, round_record, event_start)
                    session.fail(error)
                    session_record["rounds"].append({"log": str(round_path), "status": round_record["status"], "error": str(error)})
                    if session.state == SessionState.FAULT_LATCHED:
                        raise
                    session_record["pre_motion_rejections"] += 1
                    node.remove_support_guard()
                    node.settle(settings.retry_delay_s)
            else:
                gate.reject(target.corrected_center_m)
        session_record["status"] = session.state.value
        return 0
    except KeyboardInterrupt:
        session.stop()
        session_record["status"] = session.state.value
        return 130
    except RuntimeError as error:
        session_record["status"] = "FAULT_LATCHED"
        session_record["error"] = str(error)
        return 2
    finally:
        session_record["finished_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        session_path.write_text(json.dumps(session_record, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"连续执行汇总：{session_path}", flush=True)
```

During implementation, ensure `session.motion_started()` and `mark_motion_started(record)` occur immediately before the same first `node.execute`. Do not allow a double transition to change fault classification. Ensure `node.remove_support_guard()` runs after every pre-motion rejection and once at process exit.

- [ ] **Step 6: Route only explicit continuous mode to the loop**

Before the one-time interface, gripper-open, and SCAN initialization, create the legacy record/output only for the non-continuous branch:

```python
single_record = None
single_output = None
if not mode.continuous:
    single_record = new_round_record(args, round_index=1, continuous=False)
    single_output = result_dir / f"single_apple_grasp_{time.strftime('%Y%m%d_%H%M%S')}.json"
```

After one-time initialization, route explicitly:

```python
if mode.continuous:
    return continuous_main(node, cfg, args, mode, result_dir)
return run_single_main(node, cfg, args, mode, single_record)
```

In the outer `finally`, write `single_output` only when `single_record is not None`; the continuous branch writes only its session and per-round logs. The single branch must retain its existing JSON filename and terminal messages. The continuous branch must not call `gripper.initialize()` and must keep the existing configure + `open_mm` command.

- [ ] **Step 7: Run all non-motion tests and inspect the diff for changed constants**

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
PYTHONPATH=apple_pick_v2 $APPLE_YOLO_PYTHON \
  -m unittest discover -s apple_pick_v2/tests -v
$APPLE_YOLO_PYTHON -m py_compile \
  apple_pick_v2/continuous_grasp_state.py \
  apple_pick_v2/grasp_cli.py \
  apple_pick_v2/single_apple_full_grasp.py
git diff --check
git diff --word-diff=porcelain continuous-grasp-baseline-20260811 -- \
  apple_pick_v2/single_apple_full_grasp.py \
  apple_pick_v2/single_apple_full_grasp.yaml
```

Expected: tests PASS; only extraction, continuous routing, record fields, and the new YAML mapping differ. Existing numeric motion, vision, TCP, target, candidate, scene, and gripper values remain byte-for-byte unchanged.

- [ ] **Step 8: Commit the coordinator**

```bash
git add apple_pick_v2/single_apple_full_grasp.py \
  apple_pick_v2/tests/test_continuous_grasp_state.py
git commit -m "feat: add persistent apple grasp coordinator"
```

---

### Task 6: Document Operation and Recovery

**Files:**
- Modify: `apple_pick_v2/success apple catch.md`
- Modify: `apple_pick_v2/苹果抓取参数配置说明.md`

**Interfaces:**
- Consumes: final CLI/YAML behavior from Tasks 3–5.
- Produces: exact startup, safe stop, max-round override, log, and rollback instructions.

- [ ] **Step 1: Add the exact continuous configuration table**

Document:

```text
continuous.max_rounds: 0 means unlimited; positive means successful rounds
continuous.min_position_change_m: 0.030 m from last release center
continuous.stable_time_s: 2.0 s stable observation
continuous.safety_countdown_s: 3.0 s additional clear-workspace delay
continuous.retry_delay_s: 2.0 s before a pre-motion retry
continuous.max_pre_motion_retries: 3 retries for one unchanged target
CLI --max-rounds overrides YAML only for that process
```

- [ ] **Step 2: Add exact startup commands**

```bash
# Terminal 1: unchanged system launch
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
ros2 launch $APPLE_CATCH_ROOT/apple_pick_v2/apple_hand_eye_all.launch.py \
  allow_trajectory_execution:=true

# Terminal 2: two-round field validation
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --continuous --execute --operator-confirmed --max-rounds 2
```

Also retain the existing `--fully-autonomous` single-round command unchanged and label it as the rollback path.

- [ ] **Step 3: Document operator behavior and failure semantics**

State explicitly: after SCAN, move the apple at least 30 mm, leave the robot workspace before the countdown finishes, do not add a second apple, use `Ctrl+C` between rounds for normal stop, and use physical emergency stop for immediate motion stop. A post-motion failure does not open the gripper or return automatically.

- [ ] **Step 4: Verify docs and commit**

```bash
rg -n "--continuous|--max-rounds|min_position_change_m|FAULT_LATCHED|fully-autonomous" \
  "apple_pick_v2/success apple catch.md" \
  "apple_pick_v2/苹果抓取参数配置说明.md"
git diff --check
git add "apple_pick_v2/success apple catch.md" \
  "apple_pick_v2/苹果抓取参数配置说明.md"
git commit -m "docs: add continuous apple grasp operation guide"
```

---

### Task 7: Run No-Motion and Old-Mode Regression

**Files:**
- Verify: all files from Tasks 2–6
- Output: `apple_pick_v2/results/` preflight JSON

**Interfaces:**
- Consumes: running ROS/MoveIt/RealSense stack and one visible apple.
- Produces: automated test output, topic health evidence, and an old single-round preflight log with no robot/gripper motion.

- [ ] **Step 1: Run the complete automated test and compile suite**

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
PYTHONPATH=apple_pick_v2 $APPLE_YOLO_PYTHON \
  -m unittest discover -s apple_pick_v2/tests -v
$APPLE_YOLO_PYTHON -m py_compile apple_pick_v2/*.py
git diff --check
```

Expected: PASS and zero compile/whitespace errors.

- [ ] **Step 2: Verify the live system without sending motion**

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
ros2 topic info /joint_states
ros2 topic info /apple_pick_v2/apple_center
timeout 10s ros2 topic hz /apple_pick_v2/apple_center
ros2 run tf2_ros tf2_echo Link6 tcp_link
```

Expected: one joint-state publisher, a live apple-center rate, and `Link6 -> tcp_link` translation `[0, 0, 0.138]`. Stop `tf2_echo` after several samples.

- [ ] **Step 3: Run the unchanged old preflight-only entry**

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --preflight-only --operator-confirmed
```

Expected: `PASS_PREFLIGHT_ONLY`, no gripper command, no trajectory execution, and one new `single_apple_grasp_*.json`.

- [ ] **Step 4: Inspect the preflight log and compare core fields to the baseline**

```bash
ls -1t apple_pick_v2/results/single_apple_grasp_*.json | head -1
$APPLE_YOLO_PYTHON -m json.tool \
  "$(ls -1t apple_pick_v2/results/single_apple_grasp_*.json | head -1)"
```

Expected: visual correction applied, candidate screening present, all full path preflight fields pass, `motion_started=false`, `gripper_close_commanded=false`, and status `PASS_PREFLIGHT_ONLY`.

- [ ] **Step 5: Commit any regression-only fixes after rerunning Steps 1–4**

```bash
git add apple_pick_v2/continuous_grasp_state.py \
  apple_pick_v2/grasp_cli.py \
  apple_pick_v2/single_apple_full_grasp.py \
  apple_pick_v2/single_apple_full_grasp.yaml \
  apple_pick_v2/tests
git commit -m "fix: preserve single-round grasp compatibility"
```

Skip this commit if no fix was needed.

---

### Task 8: Validate One Continuous Round on the Real Robot

**Files:**
- Execute: `apple_pick_v2/single_apple_full_grasp.py`
- Inspect: `apple_pick_v2/results/continuous_apple_grasp_*.json`
- Inspect: per-round `apple_pick_v2/results/single_apple_grasp_*_round001.json`

**Interfaces:**
- Consumes: operator-authorized clear workspace, one apple, running ROS/MoveIt/vision, physical emergency stop.
- Produces: one complete automatic pick-return-SCAN round with `max_rounds=1`.

- [ ] **Step 1: Perform read-only prechecks**

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
ros2 node list
ros2 topic info /joint_states
ros2 topic info /apple_pick_v2/apple_center
timeout 10s ros2 topic hz /apple_pick_v2/apple_center
```

Expected: driver, robot_state_publisher, move_group, RealSense, static TF, and one active apple localization stream. Stop if duplicate critical nodes or stale topics are present.

- [ ] **Step 2: Execute exactly one continuous round**

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --continuous --execute --operator-confirmed --max-rounds 1
```

Expected physical sequence: open gripper, SCAN, stable countdown, PREGRASP, VERIFY, GRASP, close, LIFT, original-position RELEASE, open, retreat, SCAN, stop. No Enter is requested.

- [ ] **Step 3: Inspect machine state and logs**

```bash
ls -1t apple_pick_v2/results/continuous_apple_grasp_*.json | head -1
ls -1t apple_pick_v2/results/single_apple_grasp_*_round001.json | head -1
```

Expected: session `status=COMPLETE`, `completed_rounds=1`; round `status=PASS_PICK_RETURN_COMPLETE`, `motion_started=true`, `gripper_reopened=true`, `returned_to_scan=true`.

- [ ] **Step 4: Stop and diagnose before further motion if any field or physical action differs**

Do not proceed to two rounds. Use `superpowers:systematic-debugging`, inspect the exact JSON stage, and preserve current pose/gripper state. Any fix must rerun Task 7 and Task 8 before Task 9.

---

### Task 9: Validate Two Continuous Rounds and Same-Position Suppression

**Files:**
- Execute: `apple_pick_v2/single_apple_full_grasp.py`
- Inspect: one session JSON and two round JSON files.

**Interfaces:**
- Consumes: Task 8 PASS, one apple, operator moves it only while the robot is waiting at SCAN.
- Produces: evidence that the old release location does not retrigger and a changed stable location does.

- [ ] **Step 1: Start an exactly two-round session**

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --continuous --execute --operator-confirmed --max-rounds 2
```

- [ ] **Step 2: Verify no same-position retrigger after round one**

After round one returns the apple and reaches SCAN, leave the apple untouched for at least 10 seconds. Expected terminal state remains `WAIT_NEW_APPLE`; no MoveIt execution goal or gripper close command is sent.

- [ ] **Step 3: Move the apple and leave the workspace**

Move the only apple by more than 30 mm while the robot remains at SCAN, remove hands from the workspace, and wait. Expected terminal transitions: `STABILITY_CHECK` → `SAFETY_COUNTDOWN` → planning → automatic second round, without Enter.

- [ ] **Step 4: Verify second-round completion and automatic session stop**

Expected: the second round performs the full pick-return-SCAN sequence and the process exits with `COMPLETE` because `max_rounds=2`.

- [ ] **Step 5: Inspect all logs**

```bash
session_log="$(ls -1t apple_pick_v2/results/continuous_apple_grasp_*.json | head -1)"
$APPLE_YOLO_PYTHON -m json.tool "$session_log"
```

Expected: `completed_rounds=2`, two round paths, both round statuses `PASS_PICK_RETURN_COMPLETE`, final session status `COMPLETE`, and the distance between round centers at least `0.030 m`.

---

### Task 10: Record the Validated Result and Create the Recovery Point

**Files:**
- Modify: `apple_pick_v2/success 8.10.md`
- Modify: `apple_pick_v2/success apple catch.md`
- Verify: all implementation and test files.

**Interfaces:**
- Consumes: Task 9 PASS logs and baseline tag.
- Produces: final tested commit and annotated tag `continuous-grasp-validated-20260811`.

- [ ] **Step 1: Append only measured results**

Record the exact session log path, both round log paths, two corrected apple centers, displacement, selected grasp poses, outcome flags, baseline tag, and final startup command. Do not write “validated” unless the physical two-round test and JSON both pass.

- [ ] **Step 2: Run the final verification suite**

```bash
source /opt/ros/jazzy/setup.bash
source $ROS_WS/install/setup.bash
PYTHONPATH=apple_pick_v2 $APPLE_YOLO_PYTHON \
  -m unittest discover -s apple_pick_v2/tests -v
$APPLE_YOLO_PYTHON -m py_compile apple_pick_v2/*.py
$APPLE_YOLO_PYTHON -c \
  "import pathlib,yaml; yaml.safe_load(pathlib.Path('apple_pick_v2/single_apple_full_grasp.yaml').read_text())"
git diff --check
git status --short
```

Expected: tests/compile/YAML/whitespace checks PASS. Review `git status` and stage only continuous feature files and the two explicitly updated success documents.

- [ ] **Step 3: Commit and tag the validated implementation**

```bash
git add apple_pick_v2/continuous_grasp_state.py \
  apple_pick_v2/grasp_cli.py \
  apple_pick_v2/single_apple_full_grasp.py \
  apple_pick_v2/single_apple_full_grasp.yaml \
  apple_pick_v2/tests \
  "apple_pick_v2/success apple catch.md" \
  "apple_pick_v2/success 8.10.md" \
  "apple_pick_v2/苹果抓取参数配置说明.md"
git commit -m "feat: validate persistent apple grasp loop"
git tag -a continuous-grasp-validated-20260811 -m \
  "Two-round real-robot continuous apple grasp validated"
git log -8 --oneline --decorate
```

- [ ] **Step 4: Confirm rollback paths**

```bash
git show --stat continuous-grasp-baseline-20260811
git show --stat continuous-grasp-validated-20260811
```

Operational rollback is the unchanged old command:

```bash
$APPLE_YOLO_PYTHON \
  $APPLE_CATCH_ROOT/apple_pick_v2/single_apple_full_grasp.py \
  --execute --operator-confirmed --fully-autonomous
```

Do not use `git reset --hard`; inspect or create a recovery branch from the baseline tag if source rollback is later required.
