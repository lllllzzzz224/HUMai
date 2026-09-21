# Final rereview outcome

Current reviewed HEAD: `c14f4f42bdba96ded6846b12966abce237af5510`.

Spec verdict: **PASS**.
Quality verdict: **PASS**.
Outstanding actionable findings: **none**.

Narrow read-only rereview of `b4859db..c14f4f4` confirms resolution of all four findings without changes to unaffected physical parameters or legacy entrypoints:

- Execution `BudgetExpired` after entering execution now latches the runner and cancels; recoverable planning timeouts are restricted to pre-motion work. The new regression exercises the real guarded action adapter, a timeout whose canceled result settles during drain, and refusal to schedule another round.
- The shared actuation guard now validates a positive finite source timestamp with ROS-clock age in `[0, 1]` seconds, in addition to the existing joint receipt/completeness/finite-value guards. Regression coverage rejects stale, future and nonfinite source stamps and accepts a fresh one.
- Valid-source interlock messages advance sequence and source-stamp watermarks before evaluating clear/healthy. Both newer `clear=false` and `healthy=false` observations reject delayed older permissive messages; a genuinely newer clear input can recover while idle.
- `StateWaitTimer` closes idle spans before selection/preflight, publishes a separate nonduplicating `ongoing_wait` snapshot, and is closed in main's `finally` before final persistence. Deterministic-clock regressions cover wait-to-work boundaries, terminal closure and ongoing snapshots without duplicate closed spans.

Inspected the five added regression tests, the updated task report and `/tmp/multi_fruit_tests_review.txt`, which records **72 tests passing**. Did not rerun the unchanged full suite. No new regression identified in this bounded fix diff. No hardware activity or source edits during review; only this review artifact was updated. This PASS is a software review verdict and makes no hardware validation or measured speedup claim.

---

# Independent final review — fast multi-fruit

Reviewed baseline `c87d13a` through implementation HEAD `b4859db`, the supplied final-review diff, task brief/report and approved spec. Historical analyzer review remains PASS as documented separately in `history-review.md`.

Initial verdict at `b4859db`: **CHANGES REQUIRED** for spec and quality. All four findings below are now resolved by `c14f4f42bdba96ded6846b12966abce237af5510`.

## Initial findings — resolved

1. **P1 — Latch execution timeouts even when cancellation settles.** `apple_pick_v2/multi_fruit_runtime.py:133–135` handles every `BudgetExpired` as `PLANNING_TIMEOUT`, even after `motion_started=True`. This is reachable through the real `GuardedTransport.wait` action timeout path (`multi_fruit_transport.py:48–58`): when an accepted execution action times out and its future finishes during cancellation drain, the transport rethrows `BudgetExpired` without setting its own fault. The runner then marks the target blocked and permits later rounds instead of latching the interrupted physical sequence. A focused offline probe with the real transport and runner produced `PLANNING_TIMEOUT`, `runner.fault=None`, `transport.fault=None`. Treat any timeout after entering execution as an execution fault, cancel, and prevent subsequent scheduling; reserve recoverable planning timeout for pre-motion planning only. Add a regression where an accepted motion times out and its canceled result arrives during drain.

2. **P1 — Enforce joint source timestamp freshness in the active guard.** `apple_pick_v2/multi_fruit_grasp.py:214–217` checks only local joint-message receipt age, completeness and finite positions. The ROS source timestamp is validated only in `on_candidates` (`:163–166`), which returns immediately during active execution. A stream of recently delivered stale/replayed joint states can therefore satisfy every active guard and cached-trajectory start/current-state check. A focused offline probe confirmed that `guard()` accepts a 99-second-old source stamp with a recent receipt. Check a positive finite ROS-clock-synchronized source stamp in the shared guard used during arming/execution, and reject invalid/stale source stamps before each actuation boundary. Cover stale source with fresh receipt during active execution.

3. **P1 — Retain interlock ordering across unsafe messages.** `apple_pick_v2/multi_fruit_runtime.py:63–69` returns before updating the source timestamp and sequence when an otherwise valid message says `clear=false` or `healthy=false`. Consequently, a delayed older permissive message can override a newer unsafe observation while idle. A pure-state probe with fresh messages `clear seq=1, stamp=10.0` → `unsafe seq=3, stamp=10.2` → delayed `clear seq=2, stamp=10.1` produced `safe=True` → `False` → `True`. This can authorize a new round despite the most recent actual observation being unsafe. Validate the message envelope and source ordering separately from its safety value; advance ordering high-water marks for valid-source unsafe/unhealthy observations and only grant clear for a newer valid healthy/clear observation. Cover both false clear and false healthy with out-of-order delayed true inputs.

4. **P2 — Close state waits at the actual work boundary and on exit.** `apple_pick_v2/multi_fruit_grasp.py:434–441` closes the prior wait only after synchronous `runner.step()` has finished. When a wait becomes actionable, that prior `state_wait` therefore includes the new round's preflight and potentially its entire execution, although those phases are separately logged with no parent relation to the wait. The final ongoing wait is never closed in `finally` (`:454–462`), so Ctrl-C after ALL_ATTEMPTED or interlock waiting omits the entire terminal tail; periodic output also contains no in-progress wait span. This undermines the requested separation of planning/actuation costs from waiting. Close waiting when work actually begins, record/snapshot ongoing waits without double counting, and flush the final interval on every exit. Add deterministic-clock coverage for wait→round and terminal wait→interrupt.

## Passing aspects and scope

- The live vision publisher and launch options feed the separate persistent runner; frozen target coordinates reach actual inherited `prepare_round`, not the legacy ten-frame selected-target collector.
- Same-class confidence sorting, duplicate suppression, one-to-one spatial association, conservative ambiguous-track quarantine and session-local attempted exclusion are implemented. Fresh rolling observations and post-motion stability reset are present.
- The same absolute planning deadline reaches IK, validity, scene, Cartesian and OMPL transport calls. Unresolved remote services and pending goal acceptance latch instead of silently allowing a subsequent target. Action cancellation is requested without claiming a confirmed stop.
- Default online mode has no initial SCAN reposition and creates no gripper; execution requires the explicit CLI gates and stamped interlock. Placement is return-only.
- Existing single-fruit implementation, YAML, basin edits and legacy entrypoints are unchanged in the reviewed diff. Physical speed/acceleration, lift geometry and close settling come from the inherited configuration/validated return planner.
- Normal phase logging is monotonic with explicit inclusive parent spans; the state-wait edge cases above remain exceptions.

Verification: inspected the supplied 67-test PASS evidence without rerunning the unchanged suite. Ran only three focused offline probes for the execution-timeout, stale-source-joint and interlock-ordering findings. No ROS node, service request, robot action, gripper command, live inference or source edit was performed. No actual speedup or hardware-safety validation is claimed.
