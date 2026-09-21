"""Stage 3.3: formal entry, isolated planning, then three empty-hand cycles.

All target poses carry an explicit base/world convention. No apple interactions.
Run from repository root with --viewer null --frames 30000 --seed 31.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scripts import grow_tree
from treesim.rm65_kinematics import (RM65Kinematics, pose_error, pose_from_xyz_quat,
    target_pose_base_to_world, target_pose_world_to_base)
from treesim.rm65_planning import Waypoint, PlanningFailure, solve_cartesian_segment
from treesim.rm65_runtime import ScratchScene, UnsafePose, emit


def state_pose(row):
    row=np.asarray(row,dtype=float)
    return pose_from_xyz_quat(row[:3],row[3:]/np.linalg.norm(row[3:]))


class ValidationProgram:
    def __init__(self):
        self.done=False
        self.index=0
        self.active=False
        self.consecutive=0
        self.elapsed=0
        self.completed=[]
        self.max_fk_position=self.max_fk_rotation=0.
        self.max_chain_position=self.max_chain_rotation=0.
        self.max_arrival_position=self.max_arrival_rotation=0.

    def prepare(self,runtime):
        self.runtime=runtime
        self.kin=RM65Kinematics(device=runtime.sim.model.device)
        self.T_world_base=state_pose(runtime.base_initial)
        c=runtime.controller
        actual_q,_=c.read_arm_state(runtime.sim.state_0)
        live_before=self.live_snapshot()
        self.scratch=ScratchScene(runtime)
        # Positive targets originate in a certified joint posture, never a guessed xyz.
        source=np.array([.2,-.4,.6,.3,.7,-.2])
        self.scratch.check_slew(actual_q,source)
        singular_values=self.kin.singular_values(source)
        if singular_values[-1] < .005:
            raise PlanningFailure('SINGULAR_SOURCE',singular_values.tolist())
        grasp=self.kin.fk(source)
        emit(event='positive_source',q=source,singular_values=singular_values,
             target_pose_base=grasp,source='independent URDF FK',home_branch='unwrapped joint slew checked')
        # Static Newton/independent FK and frame contract, including each joint sign.
        fk_q=[np.zeros(6),source]
        for j in range(6):
            for sign in (-1,1):
                q=np.zeros(6);q[j]=sign*.05;fk_q.append(q)
        for q in fk_q:
            base=self.kin.fk(q); nf=self.kin.fk_newton(q)
            p,r=pose_error(base,nf)
            self.max_chain_position=max(self.max_chain_position,p)
            self.max_chain_rotation=max(self.max_chain_rotation,r)
            if p>1e-5 or r>1e-4:
                raise PlanningFailure('FK_CHAIN_MISMATCH',dict(position=p,rotation=r))
            world=target_pose_base_to_world(base,self.T_world_base)
            back=target_pose_world_to_base(world,self.T_world_base)
            np.testing.assert_allclose(back,base,atol=1e-12,rtol=0)
        # Explicit world target -> base at the IK boundary, including grasp roundtrip.
        result=self.kin.solve(target_pose_world_to_base(
            target_pose_base_to_world(grasp,self.T_world_base),self.T_world_base),
            actual_q,extra_seeds=[source],collision_check=self.scratch.check)
        emit(event='ik_candidates',label='source_grasp',result=result.status,candidates=result.candidates,
             seed_role='measured_actual_first')
        if result.q_arm is None:
            raise PlanningFailure('IK_GRASP',result.status)
        self.scratch.check_slew(actual_q,result.q_arm)
        pre=grasp.copy();pre[:3,3]-=.02*grasp[:3,2]
        pre_world=target_pose_base_to_world(pre,self.T_world_base)
        pr=self.kin.solve(target_pose_world_to_base(pre_world,self.T_world_base),
                          actual_q,extra_seeds=[result.q_arm],collision_check=self.scratch.check)
        emit(event='ik_candidates',label='pre_grasp',result=pr.status,candidates=pr.candidates,
             seed_role='measured_actual_first')
        if pr.q_arm is None:
            raise PlanningFailure('IK_PREGRASP',pr.status)
        self.scratch.check_slew(actual_q,pr.q_arm)
        forward=solve_cartesian_segment(self.kin,pre,grasp,pr.q_arm,
            collision_check=self.scratch.check,check_slew=self.scratch.check_slew,audit=emit,label='approach')
        retract=solve_cartesian_segment(self.kin,grasp,pre,forward[-1].q_arm,
            collision_check=self.scratch.check,check_slew=self.scratch.check_slew,audit=emit,label='retract')
        self.scratch.check_slew(retract[-1].q_arm,np.zeros(6))
        home=Waypoint('home',np.zeros(6),self.kin.fk(np.zeros(6)))
        cycle=[home,Waypoint('pre_grasp',pr.q_arm,pre),*forward,*retract,home]
        self.plan=[]
        for round_index in range(3):
            self.plan.extend(Waypoint(f'round{round_index+1}/{w.label}',w.q_arm.copy(),w.target_pose_base.copy()) for w in cycle)
        self.failure_checks(source)
        self.assert_live_snapshot(live_before)
        emit(event='prepared_plan',rounds=3,waypoints=len(self.plan),scratch_samples=self.scratch.samples,
             whole_sequence_preflight=True,live_state_and_target_unchanged=True,
             discrete_collision_check_only=True,chain_max_position=self.max_chain_position,
             chain_max_rotation=self.max_chain_rotation)
        runtime.listeners.append(self.after_substep)

    def live_snapshot(self):
        r=self.runtime
        return [a.numpy().copy() for a in (r.sim.state_0.joint_q,r.sim.state_0.joint_qd,
            r.sim.state_0.body_q,r.sim.state_0.body_qd,r.sim.control.joint_target_q)]

    def assert_live_snapshot(self,before):
        for a,b in zip(before,self.live_snapshot()): np.testing.assert_array_equal(a,b)

    def failure_checks(self,source):
        before=self.live_snapshot()
        far=np.eye(4);far[:3,3]=[10.,10.,10.]
        invalid=np.full((4,4),np.nan)
        cases=[('invalid',invalid,source,{}),('unreachable',far,source,{}),
               ('timeout',self.kin.fk(source),source,dict(timeout_s=1e-12)),
               ('limit',self.kin.fk(self.kin.soft_upper+.02),self.kin.soft_upper+.02,dict(max_iterations=0))]
        for label,pose,seed,kw in cases:
            result=self.kin.solve(pose,seed,**kw)
            emit(event='negative_ik',case=label,result=result.status,candidates=result.candidates)
            if result.q_arm is not None: raise AssertionError(f'negative accepted: {label}')
            self.assert_live_snapshot(before)
        # Known Stage 3.2 installation collision, checked only in isolated state.
        colliding=np.zeros(6);colliding[1]=-2.258
        try:self.scratch.check(colliding)
        except UnsafePose as exc:
            if 'ground' not in exc.pair:
                raise AssertionError('expected ground collision classification') from exc
            emit(event='negative_collision',result='REJECT',error=str(exc))
        else:raise AssertionError('known ground collision was accepted')
        self.assert_live_snapshot(before)

    def before_frame(self,runtime):
        if not self.active:
            w=self.plan[self.index]
            q=runtime.controller
            # All plan points were approved before the first physics step.
            if np.any(w.q_arm<q.soft_lower) or np.any(w.q_arm>q.soft_upper):
                raise PlanningFailure('LIMIT','prepared point corrupted')
            if q.set_arm_targets(w.q_arm):
                raise PlanningFailure('LIMIT','unexpected clamp')
            self.consecutive=self.elapsed=0
            self.active=True
            emit(event='waypoint_start',index=self.index,label=w.label,q=w.q_arm)

    def after_substep(self,state):
        runtime=self.runtime;c=runtime.controller
        q,v=c.read_arm_state(state)
        actual=state_pose(state.body_q.numpy()[runtime.audit.body_ids['gripper_tcp']])
        fk=target_pose_base_to_world(self.kin.fk(q),self.T_world_base)
        p,r=pose_error(actual,fk)
        self.max_fk_position=max(self.max_fk_position,p);self.max_fk_rotation=max(self.max_fk_rotation,r)
        if p>.0001 or r>np.deg2rad(.01):
            raise PlanningFailure('ACTUAL_FK_MISMATCH',dict(position=p,rotation=r))
        target=target_pose_base_to_world(self.plan[self.index].target_pose_base,self.T_world_base)
        ep,er=pose_error(actual,target)
        self.elapsed+=1
        good=c.is_at_target(state) and ep<=.001 and er<=np.deg2rad(.5)
        self.consecutive=self.consecutive+1 if good else 0
        if self.elapsed % 90 == 0:
            emit(event='tracking_sample',label=self.plan[self.index].label,substeps=self.elapsed,
                 actual_q=q,actual_qd=v,goal=c.goal[:6],sent=c.sent[:6],
                 position_error=ep,rotation_error=er,consecutive=self.consecutive,
                 actual_fk_max_position=self.max_fk_position,actual_fk_max_rotation=self.max_fk_rotation)
        if self.elapsed>=1800 and self.consecutive<45:
            detail=dict(label=self.plan[self.index].label,actual_q=q.tolist(),actual_qd=v.tolist(),
                        goal=c.goal[:6].tolist(),position_error=ep,rotation_error=er,
                        consecutive=self.consecutive,completed=self.completed,
                        actual_fk_max_position=self.max_fk_position,actual_fk_max_rotation=self.max_fk_rotation)
            emit(event='arrival_timeout',**detail)
            raise PlanningFailure('TIMEOUT',detail)
        self.last_error=(ep,er)

    def after_frame(self,runtime):
        if self.consecutive>=45:
            p,r=self.last_error
            self.max_arrival_position=max(self.max_arrival_position,p)
            self.max_arrival_rotation=max(self.max_arrival_rotation,r)
            self.completed.append(self.plan[self.index].label)
            emit(event='waypoint_arrived',index=self.index,label=self.plan[self.index].label,
                 substeps=self.elapsed,position_error=p,rotation_error=r)
            self.index+=1;self.active=False
            if self.index==len(self.plan):
                self.done=True
                emit(event='stage33_sequence',result='PASS',rounds=3,completed=len(self.completed),
                     frames=runtime.frames,substeps=runtime.substeps,
                     chain_max_position=self.max_chain_position,chain_max_rotation=self.max_chain_rotation,
                     actual_fk_max_position=self.max_fk_position,actual_fk_max_rotation=self.max_fk_rotation,
                     arrival_max_position=self.max_arrival_position,arrival_max_rotation=self.max_arrival_rotation,
                     scratch_samples=self.scratch.samples)


def main():
    args=grow_tree.parse_args()
    if (args.robot_model,args.mount,args.rm_control,args.viewer,args.seed,args.num_envs)!=(
            'rm65','fixed','joint','null',31,1):
        raise ValueError('Stage 3.3 requires rm65+fixed+joint, null viewer, seed31, one environment')
    grow_tree.main(rm_program=ValidationProgram())


if __name__=='__main__':
    try:main()
    except Exception as exc:
        emit(event='stage33_stop',category=getattr(exc,'category',type(exc).__name__),error=str(exc))
        raise
