"""Hardware-independent runtime contracts and session control."""
from contextlib import contextmanager
from math import isfinite
import time


class BudgetExpired(TimeoutError):
    pass


class HardwareFault(Exception):
    """Completion/stop is unconfirmed: do not schedule another round."""


class Budget:
    def __init__(self, seconds, clock=time.perf_counter):
        if not isfinite(seconds) or seconds <= 0:
            raise ValueError('budget must be finite and positive')
        self.clock = clock
        self.deadline = clock()+seconds

    def remaining(self):
        remaining = self.deadline-self.clock()
        if remaining <= 0:
            raise BudgetExpired('shared per-target planning deadline exhausted')
        return remaining


class PhaseLog:
    def __init__(self, clock=time.perf_counter):
        self.clock, self.spans, self.stack = clock, [], []
        self.target_id = None

    @contextmanager
    def phase(self, name, **metadata):
        span = dict(id=len(self.spans)+1, parent_id=self.stack[-1] if self.stack else None,
                    phase=name, target_id=self.target_id, start_monotonic_s=self.clock(),
                    inclusive=True, status='OK')
        span.update(metadata)
        self.spans.append(span)
        self.stack.append(span['id'])
        try:
            yield span
        except BaseException as error:
            span.update(status='ERROR', error=str(error), error_type=type(error).__name__)
            raise
        finally:
            span['end_monotonic_s'] = self.clock()
            span['duration_s'] = span['end_monotonic_s']-span['start_monotonic_s']
            self.stack.pop()


class StateWaitTimer:
    """Accumulate continuous idle intervals; close before any active round work."""
    def __init__(self,log):
        self.log,self.status,self.started=log,None,None

    def set(self,status):
        if status==self.status: return status
        self.close()
        self.status,self.started=status,self.log.clock()
        return status

    def snapshot(self):
        if self.started is None: return None
        return dict(status=self.status,start_monotonic_s=self.started,
                    elapsed_s=self.log.clock()-self.started,in_progress=True)

    def close(self):
        if self.started is None: return
        end=self.log.clock()
        self.log.spans.append(dict(id=len(self.log.spans)+1,parent_id=None,phase='state_wait',
            status=self.status,start_monotonic_s=self.started,end_monotonic_s=end,
            duration_s=end-self.started,inclusive=True))
        self.status,self.started=None,None


class Interlock:
    def __init__(self, source, max_age_s=.5):
        self.source, self.max_age_s = source, max_age_s
        self.stamp, self.received, self.sequence = 0., 0., -1
        self.clear = False

    def update(self, value, ros_now, monotonic_now):
        self.clear = False
        try:
            stamp, sequence = float(value['stamp_s']), value['sequence']
            if (not self.source or value['source'] != self.source or value['version'] != 1 or
                    type(sequence) is not int or sequence <= self.sequence or
                    not isfinite(stamp) or stamp <= self.stamp or
                    not 0 <= ros_now-stamp <= self.max_age_s):
                return
            self.stamp, self.sequence, self.received = stamp, sequence, monotonic_now
            self.clear = value.get('healthy') is True and value.get('clear') is True
        except (ValueError, KeyError, TypeError):
            pass

    def safe(self, ros_now, monotonic_now):
        return (self.clear and 0 <= ros_now-self.stamp <= self.max_age_s and
                0 <= monotonic_now-self.received <= self.max_age_s)


class Runner:
    def __init__(self, manager, backend, *, execute=False, operator_confirmed=False,
                 planning_budget_s=2., ros_now=time.time, log=None):
        self.manager, self.backend = manager, backend
        self.execute_requested, self.operator_confirmed = execute, operator_confirmed
        self.planning_budget_s, self.ros_now = planning_budget_s, ros_now
        self.log = log or PhaseLog()
        self.wait_timer = StateWaitTimer(self.log)
        self.records = []
        self.fault = None

    def step(self):
        if self.fault:
            self.wait_timer.close()
            return 'FAULT_LATCHED_STOP_UNCONFIRMED'
        if not self.manager.eligible(self.ros_now()):
            return self.wait_timer.set(self.manager.summary(self.ros_now()))
        if self.execute_requested and (not self.operator_confirmed or not self.backend.safe()):
            return self.wait_timer.set('WAITING_INTERLOCK')
        self.wait_timer.close()
        with self.log.phase('selection'):
            target = self.manager.lock(self.ros_now())
        if target is None:
            return self.wait_timer.set(self.manager.summary(self.ros_now()))
        if self.execute_requested and (not self.operator_confirmed or not self.backend.safe()):
            return self.wait_timer.set('WAITING_INTERLOCK')
        self.log.target_id = target.id
        record = dict(target_id=target.id, class_id=target.class_id, class_name=target.class_name,
                      confidence=target.confidence, capture_stamp_s=target.capture,
                      attempt_count=self.manager.tracks[target.id].attempts+1,
                      motion_started=False, physical_grasp_success='UNVERIFIED')
        self.records.append(record)
        motion_started = False
        revision = self.manager.scene_revision
        budget = Budget(self.planning_budget_s)
        try:
            with self.log.phase('preflight'):
                prepared = self.backend.prepare(target,budget,record)
            with self.log.phase('revalidation'):
                budget.remaining()
                if (not self.manager.revalidate(target,self.ros_now()) or
                        self.manager.scene_revision != revision or not self.backend.revalidate(prepared,budget)):
                    record['status'] = 'TARGET_CHANGED'
                    self.manager.mark_blocked(target.id,self.ros_now())
                    return record['status']
                if self.execute_requested and not self.backend.safe():
                    raise HardwareFault('workspace-clear interlock lost after planning')
            if not self.execute_requested:
                # "attempted" is scheduling exclusion here, never a physical success claim.
                self.manager.mark_attempted(target.id)
                record['status'] = 'PLANNED_ONLY'
                return record['status']
            self.manager.mark_attempted(target.id)
            motion_started = True
            with self.log.phase('execution'):
                self.backend.execute(prepared,record)
            record['status'] = 'COMMANDED_RETURN_COMPLETE'
        except BudgetExpired as error:
            if motion_started:
                self.fault = str(error)
                self.backend.cancel()
                record.update(status='FAULT_LATCHED_STOP_UNCONFIRMED',error=self.fault)
            else:
                record.update(status='PLANNING_TIMEOUT',error=str(error))
                self.manager.mark_blocked(target.id,self.ros_now())
        except RuntimeError as error:
            if motion_started:
                self.fault = str(error)
                self.backend.cancel()
                record.update(status='FAULT_LATCHED_STOP_UNCONFIRMED',error=self.fault)
            else:
                self.manager.mark_blocked(target.id,self.ros_now())
                record.update(status='PLANNING_REJECTED',error=str(error))
        except (HardwareFault, KeyboardInterrupt) as error:
            self.fault = str(error) or 'operator interrupt'
            self.backend.cancel()
            record.update(status='FAULT_LATCHED_STOP_UNCONFIRMED',error=self.fault)
        finally:
            if motion_started:
                self.manager.motion_boundary(self.ros_now())
            self.log.target_id = None
        return record['status']
