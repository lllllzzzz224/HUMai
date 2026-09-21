"""Bounded, atomic Cartesian preflight; never writes controller targets.

Every segment is fully solved and checked before its waypoint list is exposed.
Only fixed-orientation translation is supported. Actual radian coordinates are
used throughout, with no wrapping or clamping.
"""
from dataclasses import dataclass
import math
import time
import numpy as np
from .rm65_kinematics import validate_pose,pose_error


@dataclass(frozen=True)
class Waypoint:
    label: str
    q_arm: np.ndarray
    target_pose_base: np.ndarray


class PlanningFailure(RuntimeError):
    def __init__(self,category,details):
        self.category=category
        self.details=details
        super().__init__(f'{category}: {details}')


def _checked(callback,category,*args):
    if callback is None:
        return
    try:
        verdict=callback(*(a.copy() for a in args))
        if verdict is not None and not bool(verdict):
            raise PlanningFailure(category,'check rejected candidate')
    except PlanningFailure:
        raise
    except Exception as exc:
        raise PlanningFailure(category,str(exc)) from exc


def solve_cartesian_segment(kin,start_pose_base,end_pose_base,start_q,
                            collision_check=None,audit=None,check_slew=None,
                            label='segment',max_step_m=.001,max_q_step=.1,
                            timeout_s=120.,max_iterations=100):
    """Return complete waypoints excluding start, or raise with no partial list.

    audit receives keyword event, label, result, candidates and seed_role.
    The first solve uses supplied measured/start q; later solves use the previous
    accepted solution as their primary seed. collision_check(q) and
    check_slew(previous_q,next_q) may return false or raise to reject; None means
    a successful raise-style check. Callback execution is inside the deadline,
    checked immediately after return (Python callbacks are not interruptible).
    """
    start_time=time.monotonic()
    try:
        start,end=validate_pose(start_pose_base),validate_pose(end_pose_base)
        previous=np.asarray(start_q,dtype=float).copy()
        if previous.shape!=(6,) or not np.isfinite(previous).all():
            raise ValueError('start_q must contain six finite actual positions')
        if not np.isfinite(max_step_m) or not 0<max_step_m<=.001:
            raise ValueError('Cartesian spacing must be >0 and <=0.001 m')
        if not np.isfinite(max_q_step) or not 0<max_q_step<=.1:
            raise ValueError('joint spacing must be >0 and <=0.1 rad')
        if not np.isfinite(timeout_s) or timeout_s<=0:
            raise ValueError('timeout must be finite and positive')
        if pose_error(start,end)[1]>1e-8:
            raise ValueError('Cartesian segment requires fixed orientation')
    except (TypeError,ValueError) as exc:
        raise PlanningFailure('invalid_input',str(exc)) from exc

    def deadline():
        remaining=timeout_s-(time.monotonic()-start_time)
        if remaining<=0:
            raise PlanningFailure('timeout',{'label':label,'timeout_s':timeout_s})
        return remaining

    def check_limits(q):
        if np.any(q<kin.soft_lower) or np.any(q>kin.soft_upper):
            raise PlanningFailure('soft_limit',{'label':label,'q_arm':q.tolist()})

    check_limits(previous)
    _checked(collision_check,'collision',previous)
    deadline()
    count=max(1,math.ceil(float(np.linalg.norm(end[:3,3]-start[:3,3]))/max_step_m))
    # Disallow unbounded memory/CPU work even with a maliciously tiny spacing.
    if count>10000:
        raise PlanningFailure('invalid_input','segment exceeds 10000 waypoints')
    prepared=[]
    for i in range(1,count+1):
        target=start.copy();target[:3,3]=start[:3,3]+(end[:3,3]-start[:3,3])*(i/count)
        point_label=f'{label}/{i:04d}'
        remaining=deadline()
        try:
            result=kin.solve(target,previous.copy(),collision_check=collision_check,
                             max_iterations=max_iterations,timeout_s=remaining)
        except Exception as exc:
            raise PlanningFailure('ik_error',{'label':point_label,'error':str(exc)}) from exc
        if audit is not None:
            audit(event='ik_candidates',label=point_label,result=result.status,
                  candidates=result.candidates,seed_role='provided_start' if i==1 else 'previous_accepted')
        deadline()
        if result.status!='success' or result.q_arm is None:
            reasons=[c.get('reason','') for c in result.candidates]
            category='timeout' if result.status=='timeout' else 'ik_failure'
            if result.status=='collision_check_error': category='collision_check_error'
            elif any(reason=='collision' or reason.startswith('collision:') for reason in reasons): category='collision'
            elif any('soft_limit' in reason for reason in reasons): category='soft_limit'
            raise PlanningFailure(category,{'label':point_label,'status':result.status,'reasons':reasons})
        q=np.asarray(result.q_arm,dtype=float)
        if q.shape!=(6,) or not np.isfinite(q).all():
            raise PlanningFailure('invalid_candidate',point_label)
        check_limits(q)
        difference=float(np.max(np.abs(q-previous)))
        if audit is not None:
            audit(event='waypoint_continuity',label=point_label,joint_delta=(q-previous).tolist(),
                  max_joint_step=difference,accepted=difference<=max_q_step)
        deadline()
        if difference>max_q_step:
            raise PlanningFailure('discontinuity',{'label':point_label,'max_abs_dq':difference})
        p,r=pose_error(kin.fk(q),target)
        if p>.0005 or r>np.deg2rad(.1):
            raise PlanningFailure('pose_residual',{'label':point_label,'position_error':p,'rotation_error':r})
        _checked(collision_check,'collision',q)
        _checked(check_slew,'slew_collision',previous,q)
        deadline()
        prepared.append(Waypoint(point_label,q.copy(),target.copy()))
        previous=q.copy()
    return prepared
