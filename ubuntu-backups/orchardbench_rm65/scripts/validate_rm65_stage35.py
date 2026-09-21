"""Stage 3.5 fixed candidate: observe, then plan in scratch; never close or hold.

A PASS covers only this candidate prerequisite, not picking or installation ranking.
Stage 3.4 timing and safety semantics are reused unchanged except for the explicitly
separate Stage 3.5 observation budget computed before any physical step.
"""
import argparse
import os
import sys
import json
import math
import traceback
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts import grow_tree
from scripts.validate_rm65_camera import ObservationCase,serial,state_pose,scratch_slew_path
from treesim.rm65_runtime import ScratchScene
from treesim.rm65_acceptance import EnvironmentClearance
from treesim.rm65_kinematics import target_pose_world_to_base,pose_error
from treesim.rm65_planning import solve_cartesian_segment
from treesim.rm65_control import RATE
import warp as wp
from PIL import Image


def unique_detection_center(detections):
    """Select from sensor output alone; truth-scored matches are never inputs."""
    if len(detections) != 1:
        raise RuntimeError(f'DETECTION_AMBIGUITY: expected one natural detection, got {len(detections)}')
    center = np.array(detections[0].center_world, dtype=float, copy=True)
    if center.shape != (3,) or not np.isfinite(center).all():
        raise RuntimeError('INVALID_DETECTION_CENTER')
    return center


def joint_margin(kin, q):
    margin = np.minimum(np.asarray(q)-kin.soft_lower, kin.soft_upper-np.asarray(q))
    if not np.isfinite(margin).all() or np.min(margin) < .10:
        raise RuntimeError('JOINT_MARGIN ' + str(margin))
    return margin


def observation_budget(start,target):
    start=np.array(start,dtype=float,copy=True);target=np.array(target,dtype=float,copy=True)
    if start.shape!=(6,) or target.shape!=(6,) or not np.isfinite(start).all() or not np.isfinite(target).all():
        raise ValueError('budget requires finite six-axis vectors')
    ratios=np.abs(target-start)/(np.asarray(RATE[:6])/60.)
    if not np.isfinite(ratios).all():raise ValueError('unrepresentable slew budget')
    slew=math.ceil(float(np.max(ratios)))
    return dict(slew_frames=slew,settling_frames=600,ready_deadline_frames=slew+600,
                acquisition_frames=36,total_frames=slew+636,hz=60,
                rate_rad_s=np.asarray(RATE[:6]).tolist(),settling_seconds=10.)


class Candidate(ObservationCase):
 def prepare(self,r):
  super().prepare(r)
  current,_=r.controller.read_arm_state(r.sim.state_0)
  self.ready_margin=np.minimum(joint_margin(self.kin,current),joint_margin(self.kin,self.q))
  self.budget=observation_budget(current,self.q)
  apples=list(r.tree.apple_data['apple_body']);i=apples.index(self.apple)
  parent=int(r.tree.apple_data['parent_body'][i])
  if r.sim.model.body_label[parent].rsplit('/',1)[-1]!='seg294':
   raise RuntimeError('TARGET_PARENT_MISMATCH')
  self.save('budget.json',self.budget)
  self.trace_file=(self.output/'substeps.jsonl').open('x',buffering=1)

 def after_substep(self,state):
  super().after_substep(state)
  r=self.r
  self.ready_margin=np.minimum(self.ready_margin,joint_margin(self.kin,self.last_motion['q']))
  data=dict(self.last_motion,base_pose=state.body_q.numpy()[r.audit.body_ids['base_link']],
            held=int(r.sim.apples._held_host.sum()),detached=int(r.sim.apples.broken_count),
            safety='passed original runtime and observation gates')
  self.trace_file.write(json.dumps(data,default=serial)+'\n')

 def after_frame(self,r):
  if self.ready_time is None and r.frames>=self.budget['ready_deadline_frames']:
   raise RuntimeError('TIMEOUT: Stage 3.5 planned slew plus 10 s settling exhausted')
  if self.ready_time is not None and r.sim.sim_time-self.ready_time>self.budget['acquisition_frames']/60.+1e-9:
   raise RuntimeError('TIMEOUT: three fresh samples not acquired')

 def before_frame(self,r):
  viewer=r.sim.viewer
  if not self.active:
   pos=np.array([2.45,-1.55,1.55]);look=np.array([.87,.36,.65]);d=look-pos;d/=np.linalg.norm(d)
   viewer.set_camera(wp.vec3(*pos),pitch=math.degrees(math.asin(d[2])),yaw=math.degrees(math.atan2(d[1],d[0])))
   viewer.renderer.set_title('Stage 3.5 candidate 01 - observation only - NOT accepted')
  if r.frames==1 or (self.ready_time is not None and not (self.output/'stable.png').exists()):
   name='installation.png' if r.frames==1 else 'stable.png'
   Image.fromarray(viewer.get_frame().numpy()).save(self.output/name)
  super().before_frame(r)
 def after_camera_sample(self,r,s):
  # Fix the input before the scoring callback reads any simulator truth.
  center=unique_detection_center(s.detections_world) if self.gate.ready else None
  super().after_camera_sample(r,s)
  if not self.done:return
  self.done=False
  # Stage34 natural detection has passed all three fresh-sample checks.
  base=state_pose(r.base_initial)
  actualq,_=r.controller.read_arm_state(r.sim.state_0);pose=base@self.kin.fk(actualq);z=pose[:3,2]
  live_arrays=(r.sim.state_0.joint_q,r.sim.state_0.joint_qd,r.sim.state_0.body_q,
               r.sim.state_0.body_qd,r.sim.control.joint_target_q)
  live_before=[a.numpy().copy() for a in live_arrays];steps_before=r.substeps
  scratch=ScratchScene(r);clear=EnvironmentClearance(r);phase='plan_only';events=[];plan=[]
  path_margin=joint_margin(self.kin,actualq)
  def check(q):
   nonlocal path_margin
   path_margin=np.minimum(path_margin,joint_margin(self.kin,q))
   scratch.check(q);clear.check(scratch.state,phase);return True
  def slew(a,b):
   for q in scratch_slew_path(a,b)[0]:check(q)
  def audit(**data):
   r.sim.render()  # keep GUI responsive; no physics step during planning
   with (self.output/'ik.jsonl').open('a') as f:f.write(json.dumps(data,default=serial)+'\n')
  self.save('plan-start.json',dict(detected_center_world=center,detection_selection='unique raw camera detection; no truth input',sample_id=s.sample_id,target_base=(np.linalg.inv(base)@np.r_[center,1])[:3],measured_q=actualq,T_world_base=base,T_world_tcp=pose,apple_parent='seg294',physics_steps=r.substeps,execution_of_pregrasp_grasp_retract=False))
  q=actualq
  for phase,offset in [('pregrasp',.08),('grasp',0.),('retract',.12)]:
   goal=pose.copy();goal[:3,3]=center-offset*z
   phase_seed=q.copy()
   pts=solve_cartesian_segment(self.kin,target_pose_world_to_base(pose,base),target_pose_world_to_base(goal,base),q,collision_check=check,check_slew=slew,audit=audit,label=phase,timeout_s=600,max_iterations=50)
   for p in pts:
    margin=joint_margin(self.kin,p.q_arm)
    plan.append(dict(stage=p.label,q=p.q_arm,margin=margin,target_pose_base=p.target_pose_base,error=pose_error(self.kin.fk(p.q_arm),p.target_pose_base)))
   events.append(dict(stage=phase,seed=phase_seed,final_q=pts[-1].q_arm,
                      start_pose_world=pose.copy(),goal_pose_world=goal.copy(),waypoints=len(pts)))
   q=pts[-1].q_arm;pose=goal
  for a,b in zip(live_arrays,live_before):np.testing.assert_array_equal(a.numpy(),b)
  if r.substeps!=steps_before:raise RuntimeError('PLAN_CHANGED_PHYSICS')
  r.sim.render()
  Image.fromarray(r.sim.viewer.get_frame().numpy()).save(self.output/'accepted-installation.png')
  self.save('plan-result.json',dict(result='PASS',plan=plan,stages=events,minimum=clear.minimum,installation_min=clear.install_min,physics_steps=r.substeps,ready_minimum_margin=self.ready_margin,path_minimum_margin=path_margin,live_state_and_targets_unchanged=True,discrete_clearance_only=True,baseline_picker_enabled=r.picking))
  self.done=True

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mount',required=True,type=float,nargs=4,metavar=('X','Y','Z','YAW'))
    parser.add_argument('--q',required=True,type=float,nargs=6)
    parser.add_argument('--log-dir',required=True,type=Path)
    a=parser.parse_args(argv)
    a.log_dir=a.log_dir.resolve();a.log_dir.mkdir(parents=True,exist_ok=False)
    a.apple='apple35';a.case='clear'
    budget=observation_budget(np.zeros(6),a.q)
    p=Candidate(a)
    p.save('configuration.json',dict(seed=31,target='apple35',parent='seg294',mount=a.mount,
        observation_q=a.q,budget=budget,gui=True,physics='MuJoCo+CG / 60Hz / 3 substeps',
        scope='observe then scratch-only pregrasp/grasp/retract; no close/hold',process_id=os.getpid()))
    cwd=Path.cwd();viewer_dir=a.log_dir/'viewer-workdir';viewer_dir.mkdir()
    try:
        os.chdir(viewer_dir)
        grow_tree.main(['--robot-model','rm65','--mount','fixed','--rm-control','joint',
            '--rm-camera','--apples','--viewer','gl','--seed','31','--frames',str(budget['total_frames']),
            '--stage35-fixed-mount',*[str(v) for v in a.mount]],rm_program=p)
    except Exception as exc:
        p.save('stop.json',dict(result='FAIL',error=str(exc),traceback=traceback.format_exc(),
            samples=p.samples,last_motion=getattr(p,'last_motion',None),
            physics_steps=getattr(getattr(p,'r',None),'substeps',0),frames=getattr(getattr(p,'r',None),'frames',0)))
        raise
    finally:
        if hasattr(p,'trace_file'):p.trace_file.close()
        os.chdir(cwd)

if __name__=='__main__':main()
