"""Pure state and stability primitives for persistent apple grasping.

This module deliberately has no ROS imports so its safety gates can be tested
without connecting to MoveIt, the robot, or the gripper.
"""

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
    def from_mapping(
        cls,
        mapping,
        *,
        max_axis_span_m,
        cli_max_rounds=None,
    ) -> "ContinuousSettings":
        max_rounds = int(
            mapping["max_rounds"]
            if cli_max_rounds is None
            else cli_max_rounds
        )
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
        self.last_release_center: np.ndarray | None = None
        self.rejected_center: np.ndarray | None = None
        self._samples: list[np.ndarray] = []
        self._stable_since: float | None = None

    def _reference(self) -> np.ndarray | None:
        if self.rejected_center is not None:
            return self.rejected_center
        return self.last_release_center

    def missing(self) -> None:
        self._samples.clear()
        self._stable_since = None

    def after_release(self, center_m) -> None:
        self.last_release_center = self._validated_center(center_m)
        self.rejected_center = None
        self.missing()

    def reject(self, center_m) -> None:
        self.rejected_center = self._validated_center(center_m)
        self.missing()

    @staticmethod
    def _validated_center(center_m) -> np.ndarray:
        center = np.asarray(center_m, dtype=np.float64)
        if center.shape != (3,) or not np.all(np.isfinite(center)):
            raise ValueError("苹果中心必须是三个有限数值")
        return center

    def observe(self, center_m, now_s) -> GateSnapshot:
        center = self._validated_center(center_m)
        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError("now_s 必须是有限数值")

        reference = self._reference()
        distance = (
            None
            if reference is None
            else float(np.linalg.norm(center - reference))
        )
        if (
            distance is not None
            and distance < self.settings.min_position_change_m
        ):
            self.missing()
            return GateSnapshot(
                "WAIT_NEW_APPLE",
                False,
                0.0,
                distance,
                tuple(center),
            )

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
        ready_after = (
            self.settings.stable_time_s + self.settings.safety_countdown_s
        )
        if elapsed >= ready_after:
            state = "READY"
        elif elapsed >= self.settings.stable_time_s:
            state = "SAFETY_COUNTDOWN"
        else:
            state = "STABILITY_CHECK"
        return GateSnapshot(
            state,
            state == "READY",
            elapsed,
            distance,
            tuple(center),
        )


class ContinuousSession:
    """Tracks loop progress and enforces the post-motion fault latch."""

    _TERMINAL_STATES = {
        SessionState.COMPLETE,
        SessionState.STOPPED_BY_OPERATOR,
        SessionState.FAULT_LATCHED,
    }

    def __init__(self, *, max_rounds: int):
        if int(max_rounds) < 0:
            raise ValueError("max_rounds 必须 >= 0")
        self.max_rounds = int(max_rounds)
        self.completed_rounds = 0
        self.state = SessionState.START
        self.motion_has_started = False
        self.stop_reason: str | None = None

    @property
    def terminal(self) -> bool:
        return self.state in self._TERMINAL_STATES

    def set_state(self, state: SessionState) -> None:
        if self.terminal:
            raise RuntimeError(f"终止状态 {self.state.value} 不允许继续迁移")
        self.state = SessionState(state)

    def motion_started(self) -> None:
        if self.terminal:
            raise RuntimeError("终止状态不允许开始运动")
        self.motion_has_started = True
        self.state = SessionState.EXECUTE_PICK

    def round_completed(self) -> None:
        if self.terminal:
            raise RuntimeError("终止状态不允许完成新一轮")
        self.completed_rounds += 1
        self.motion_has_started = False
        if self.max_rounds and self.completed_rounds >= self.max_rounds:
            self.state = SessionState.COMPLETE
        else:
            self.state = SessionState.WAIT_NEW_APPLE

    def fail(self, reason: str) -> None:
        self.stop_reason = str(reason)
        if self.motion_has_started:
            self.state = SessionState.FAULT_LATCHED
        else:
            self.state = SessionState.WAIT_NEW_APPLE

    def stop(self, reason: str = "operator requested stop") -> None:
        self.stop_reason = str(reason)
        self.state = SessionState.STOPPED_BY_OPERATOR
