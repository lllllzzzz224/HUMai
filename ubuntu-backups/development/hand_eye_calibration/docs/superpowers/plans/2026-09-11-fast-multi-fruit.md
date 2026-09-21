# Fast Multi-Fruit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Add opt-in, measured multi-target automatic same-class scheduling while preserving legacy grasping.
**Architecture:** Optional complete vision snapshots feed a pure target manager and a persistent ROS runner. Reuse existing return grasp geometry with explicit runtime budgets and fresh interlock gating; keep new runtime separate from legacy entrypoints.
**Tech Stack:** Python 3.12, ROS 2 Jazzy, existing MoveIt services/actions, unittest, numpy.
**Spec:** docs/superpowers/specs/2026-09-11-fast-multi-fruit.md

## Global Constraints

- No real motion or device state changes during development.
- Preserve existing single-fruit and legacy continuous entrypoints and current basket modifications.
- New mode uses return placement only and defaults to no-motion planning.
- Execution requires --execute, --operator-confirmed and a fresh reliable workspace-clear interlock.
- Keep physical speed/acceleration, lift height and 2.5 s close settle unchanged.
- No claims that attempted targets were physically grasped or that mock timing equals hardware speed.
- All new timing uses monotonic high-resolution durations; report inclusive spans explicitly.
- Write and run failing behavioral tests before implementing, then verify the old suite.

### Task 1: Multi-target perception, scheduler, bounded runner and measured replay

**Files:**
- Modify apple_pick_v2/apple_center_localizer_v2.py and apple_hand_eye_all.launch.py for opt-in snapshots and inference-rate configuration.
- Create apple_pick_v2/multi_fruit_state.py for pure target identity, freshness, ranking and attempted exclusion.
- Create apple_pick_v2/multi_fruit_grasp.py for CLI, subscriptions, budget/interlock-aware runtime and high-resolution round logs.
- Create focused helper modules if needed; do not grow single_apple_full_grasp.py unnecessarily.
- Create apple_pick_v2/tests/test_multi_fruit_state.py, test_multi_fruit_vision.py, test_multi_fruit_runtime.py and test_multi_fruit_integration.py.
- Create apple_pick_v2/README_多果快速抓取.md with exact commands, signal contract and validation boundary.

**Interfaces:**
- Vision publishes /apple_pick_v2/fruit_candidates as String JSON: version, base frame, source capture stamp, snapshot validity and detections containing class_id/class_name, confidence, center_xyz_m and radius_m. Publish valid empty snapshots separately from invalid/missing perception. Include source model time if measured.
- Pure state consumes timestamped detections and time, exposes eligible candidates and stable locked target, mark-attempted/mark-blocked, and a remaining-state summary. Association is one-to-one with same-class checks, duplicate suppression and conservative ambiguity handling.
- Runtime imports AppleTarget/prepare_round/execute_prepared_round or a narrowly factored equivalent. Explicitly pass a frozen selected target; do not read the legacy selected apple topic as the new target. Only accept capture timestamps after last scene-motion boundary for actuation validation.
- Runtime phase API uses a context manager with monotonic start/end/status and target metadata; a plan deadline is a shared absolute time, not a timeout restarted for each service. Plan-only never configures the gripper or executes even initial SCAN repositioning.

- [ ] Write and execute red tests for literal three-fruit confidence order, tile duplicates, stale captures, no double use of one track, attempted target staying excluded, ambiguous moved fruit, invalid snapshot versus empty scene, budget exhaustion, failure on stale interlock and no motion in default mode.
- [ ] Implement pure state and snapshot adapter; default legacy vision behavior remains unchanged.
- [ ] Implement persistent runner with bounded preflight and fresh post-plan revalidation. Use fresh rolling observations, no repeated 10-frame collect_target. Skip blocked candidates after a bounded budget. Treat all-attempted and no-safe-target as distinct from empty.
- [ ] Retain joint/state/collision validation and scene validity; track pending action handles and cancel accepted motion on fault. Do not continue a round after interlock loss. When sensors/stops are unconfirmed, latch and report that condition.
- [ ] Add deterministic three-fruit replay/integration tests for A then B then C, exclusion, disappearance and moved-neighbor rejection, using actual scheduler/runtime logic with only external hardware calls doubled.
- [ ] Run all tests under sourced ROS environment, inspect AST/imports/CLI help, and report changed files, test results and unresolved physical validation.
- [ ] Commit only owned implementation files in the isolated clone. Write task report with tests and limitations.

### Task 2: Historical analysis and integration acceptance

**Files:** create apple_pick_v2/analyze_grasp_timing.py plus tests; write historical analysis into workspace reports. Read original results without modifying them.

- [ ] Test duration extraction against literal dated events and missing-event logs. Missing timing must be unknown, never zero or called planner time.
- [ ] Analyze all original continuous session JSONs, separating active round duration, round start to preflight, post-preflight to first motion, motion/settle span, inter-round waits and terminal idle.
- [ ] Identify repeated visual collection cost from original flow; do not equate long idle periods with solver latency.
- [ ] Review Task 1 diff for contract compliance and actual cancellation/freshness handling. Fix actionable issues and verify affected tests.
- [ ] Produce reusable analysis command and a Chinese report with per-round evidence and proposed optimization priorities.
- [ ] Verify original source hashes; back up and install only tested changed files to original apple_pick_v2. Run final no-motion verification there. Do not start a live grasp.
